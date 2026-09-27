# Stage 7: Camera → BEV 链路剖析

在纯视觉（Camera-only）3D 感知中，最大痛点是**单目深度缺失**。一张 2D 图像如何"无中生有"地变成 3D 空间的 BEV 特征？目前有两大主流流派：**Bottom-Up（以 LSS 为代表）**和 **Top-Down（以 BEVFormer 为代表）**。

---

## 1. 物理与几何概念

| 流派 | 核心思路 | 代表算法 |
|---|---|---|
| **Bottom-Up（LSS）** | 预测每像素深度 → 沿射线发射到 3D → 拍扁到 BEV | LSS, BEVDet, BEVDepth |
| **Top-Down（Query-based）** | BEV 空间铺满 Query → 投影回图像去"抠"特征 | BEVFormer, DETR3D |

- **视锥体（Frustum）**：相机图像上的每个像素，在 3D 物理空间中对应着从光心射出的一条射线（Ray）。所有射线构成视锥体。
- **深度 Bin**：把射线在深度方向上离散化为若干格子（如从 2m 到 50m 均匀分 48 格），每格代表一个候选的深度平面。
- **Voxel Pooling（体素池化）**：把飘散在 3D 空间中的视锥特征，按 BEV 网格聚合（累加/取最大）到致密的 2D 特征图上。

---

## 2. 数学公式推导：LSS 深度外积（Outer Product）

设特征图上某像素 `(u, v)` 的特征向量为 `f ∈ R^C`，深度预测分支输出该像素在 `|D|` 个深度 Bin 上的概率分布 `p ∈ R^{|D|}`（经 Softmax 归一化）。

LSS 将其"升维"到 3D 视锥体特征 `F_3D` 的方式是对每个深度 d 做加权：

```math
F_{3D}(d, u, v) = p_d(u, v) \times f(u, v) \quad \in \mathbb{R}^C
```

这个操作的物理意义：**特征向量 f 沿射线被复制了 |D| 份，但每份都被该深度的概率 p_d 加权**。网络极度确信某物体在 10m 时，10m 处概率≈1，其余≈0，实现了 2D 到 3D 的软性深度转化。

在代码中，这等价于以下广播乘法（Outer Product）：

```
# depth:  (6, |D|, H_f, W_f) — 深度概率图
# feat:   (6,   C, H_f, W_f) — 图像特征图
# 广播后:  (6, C, |D|, H_f, W_f) — 视锥 3D 特征
F_3D = feat[:, :, None] * depth[:, None, :, :]
```

### 从像素坐标到 3D 坐标

对于深度 d 的候选点，利用相机内参 K 的逆矩阵可以反投影到 Camera 坐标系：

```math
\begin{bmatrix} X_c \\ Y_c \\ Z_c \end{bmatrix}
= d \cdot K^{-1} \begin{bmatrix} u \\ v \\ 1 \end{bmatrix}
```

再通过外参 `cam2ego` 转换到 Ego 坐标系，即可确定每个视锥点在 BEV 中的位置。

---

## 3. 精确的源码定位

LSS 核心算子（在 BEVDepth/BEVDet 等实现中）：

**① `create_frustum`（预计算视锥 3D 坐标，离线）**

在模型初始化时，基于特征图尺寸、Depth Bin 设置和相机内参，预计算出所有像素在所有深度候选下对应的 Camera 3D 坐标。此为静态张量，无需在每次 forward 中重新计算。

**② `get_geometry`（视锥点 → Ego 坐标）**

```python
# 将预计算好的 Camera 系视锥坐标转换到 Ego 坐标系
points = cam2ego @ frustum_points  # (6, D, H_f, W_f, 3)
```

**③ `BEVPool` / `voxel_pooling`（Scatter 到 BEV 平面）**

- 将 Ego 坐标系下的视锥 3D 点映射到 BEV 网格索引 `(bev_x, bev_y)`。
- 把落入同一 BEV 格的所有点的特征做 Sum 聚合（或 Max）。
- 工业界使用的 `BEVPoolv2` 通过并行排序 + CUDA 前缀和将此步骤加速数十倍。

---

## 4. 张量流转分析 (Tensor Flow)

以 6 摄像头的 LSS 链路为例：

| 步骤 | 操作 | Shape |
|---|---|---|
| 输入图像 | 6 路环视相机 | `(6, 3, 256, 704)` |
| 2D Backbone | ResNet + FPN | `(6, C, 16, 44)` — H/16, W/16 |
| 深度预测头 | Conv → Softmax | `(6, 118, 16, 44)` — 118 个 Depth Bin |
| **外积（Lift）** | feat × depth，广播乘法 | `(6, C, 118, 16, 44)` |
| 坐标转换 | frustum → Ego 坐标 | `(6×C×118×16×44, 3)` — 海量稀疏 3D 点 |
| **BEVPool（Splat）** | Scatter + Sum 按 BEV Grid 聚合 | `(1, C, 128, 128)` |
| 2D BEV Backbone | ResNet + FPN on BEV | `(1, 256, 128, 128)` |

---

## 5. 高频面试考点总结

**🔥 考点 1：LSS 的 Voxel Pooling 为什么慢？BEVPoolv2 如何优化？**

原版 Voxel Pooling 问题在于"原子加锁冲突"：海量视锥点被映射到相同 BEV 格时，GPU 多线程同时执行 `atomic_add`，产生严重的锁竞争，GPU 利用率极低。

BEVPoolv2 的优化步骤：
1. **并行排序（Parallel Sort）**：预先对所有视锥点按 BEV 格索引排序，使属于同一格子的点在内存中连续存储。
2. **前缀和（Prefix Sum）**：排序后用 CUDA 前缀和在 O(1) 时间内对连续内存块求和，完全消除原子锁。
3. 结果：Pooling 速度提升 20~30 倍，成为量产部署的关键优化。

**🔥 考点 2：LSS（Bottom-Up）和 BEVFormer（Top-Down）哪种更好？各自适合什么场景？**

- **LSS（显式深度）**：强依赖深度准确性，深度猜错会把特征扔到错误 BEV 格。但可以利用 LiDAR 稀疏深度做监督（BEVDepth），在远距离定位上有优势，且部署相对简单（无 Attention）。
- **BEVFormer（隐式深度）**：通过注意力机制全图搜索特征，没有"扔错位置"的问题。但计算量与 Query 数量成正比，高分辨率 BEV 时计算开销大，且 Deformable Attention 在部分边端芯片上支持不好。
