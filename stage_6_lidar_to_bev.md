# Stage 6: LiDAR → BEV 链路剖析

自动驾驶网络极其喜欢在 BEV（Bird's Eye View，鸟瞰图）空间做感知。在 BEV 下，物体没有透视遮挡，物理尺寸度量精确（1 个 Pixel 严格代表实际 0.1×0.1 米）。

对离散、无序的 3D LiDAR 点云，将其转化为致密、规则的 2D BEV 特征图，是 3D 检测算法（如 PointPillars, CenterPoint）最核心的链路。

---

## 1. 物理与几何概念

| 概念 | 定义 | 用途 |
|---|---|---|
| **体素化（Voxelization）** | 把点云切分到三维 L×W×H 网格（Voxel）中 | 离散化为规则网格供 CNN 处理 |
| **柱体化（Pillarization）** | 不切分 Z 轴，一刀从天切到地，退化为"柱子"（Pillar） | PointPillars 核心，免去昂贵 3D 卷积 |
| **稀疏张量（Sparse Tensor）** | 只记录有点的 Voxel 的特征和 3D 索引 | 90% 以上体素是空气，避免显存爆炸 |
| **伪图像（Pseudo Image）** | Pillar 特征散布回 2D 平面后的致密特征图 | 连接 2D CNN Backbone 的桥梁 |

---

## 2. 数学公式推导：网格坐标映射

设点云检测范围为 `[x_min, y_min, z_min]` 到 `[x_max, y_max, z_max]`，体素大小为 `[vx, vy, vz]`。

对任意点 `P = (x, y, z)`，归属的 Voxel 索引为：

```math
c_x = \left\lfloor \frac{x - x_{min}}{v_x} \right\rfloor \qquad
c_y = \left\lfloor \frac{y - y_{min}}{v_y} \right\rfloor \qquad
c_z = \left\lfloor \frac{z - z_{min}}{v_z} \right\rfloor
```

BEV 特征图的空间分辨率（网格数量）：

```math
W_{BEV} = \left\lceil \frac{x_{max} - x_{min}}{v_x} \right\rceil \qquad
H_{BEV} = \left\lceil \frac{y_{max} - y_{min}}{v_y} \right\rceil
```

以 nuScenes 常用配置为例：范围 `[-51.2, 51.2]` 米，体素大小 0.2 米 → BEV 分辨率 = `512 × 512`。

---

## 3. 精确的源码定位

LiDAR → BEV 标准链路被极度模块化，分为三步：

**① Voxelization（底层 CUDA 实现）**
- 调用入口：`mmdet3d/models/data_preprocessors/data_preprocessor.py` 中的 `voxelize` 函数
- 底层算子：MMCV 的 Hard Voxelization CUDA Kernel

**② VoxelEncoder（Pillar 内部特征提取）**
- 文件：[`mmdet3d/models/voxel_encoders/pillar_encoder.py`](file:///root/mmdetection3d/mmdet3d/models/voxel_encoders/pillar_encoder.py)
- 类名：`PillarFeatureNet`
- 核心逻辑：对每个 Pillar 内所有点，通过简化版 PointNet（Linear + BN + ReLU + MaxPool）提取聚合特征向量。

**③ MiddleEncoder（稀疏 → 稠密致密化）**
- 文件：[`mmdet3d/models/middle_encoders/pillar_scatter.py`](file:///root/mmdetection3d/mmdet3d/models/middle_encoders/pillar_scatter.py)
- 类名：`PointPillarsScatter`
- 核心逻辑：按照各 Pillar 的 `(y, x)` 空间索引，将特征向量"散布"填入全零的 Canvas（BEV 特征图）对应格子。

---

## 4. 张量流转分析 (Tensor Flow)

以 PointPillars 为例（这是张量形状变形的艺术）：

| 步骤 | 操作 | Shape |
|---|---|---|
| 原始点云 | LiDAR 扫描点 | `(N, 5)` — `[x,y,z,r,Δt]` |
| **Voxelization** | 硬切分，截断/填零 | — |
| └ voxels 输出 | 每 Pillar 最多 32 个点 | `(M, 32, 5)`，M≈16000 |
| └ coors 输出 | 每 Pillar 的空间索引 | `(M, 4)` — `[batch,z,y,x]` |
| **PillarFeatureNet** | Linear(5→64) + BN + ReLU | `(M, 32, 64)` |
| └ MaxPool | 对 32 个点做最大池化 | `(M, 64)` |
| **PointPillarsScatter** | 按 `(y,x)` 索引散布填入 | — |
| └ BEV Canvas 输出 | **伪图像（Pseudo Image）** | `(B, 64, H_BEV, W_BEV)` |
| **2D Backbone (ResNet)** | 标准 2D 卷积 + FPN | `(B, C, H/2, W/2)` 等多尺度 |
| **CenterHead** | 预测 Heatmap + 回归分支 | `(B, num_class, H_BEV, W_BEV)` |

---

## 5. 高频面试考点总结

**🔥 考点 1：Hard Voxelization (HV) 和 Dynamic Voxelization (DV) 的区别？**

- **HV（Hard）**：每个 Voxel 强制保留固定点数（如 32 个）。点少补零（显存浪费），点多随机丢弃（信息丢失）。输出 Tensor 形状规则，可用标准 PyTorch 算子处理。
- **DV（Dynamic）**：不限制每个 Voxel 内点数，不丢弃任何点。利用特殊 CUDA 并行算子（基于动态哈希映射的 `torch_scatter`）直接聚合。实现复杂，但在远距离小目标检测上（不丢弃远处稀疏点）通常能涨 1~2 个点 mAP。

**🔥 考点 2：Sparse Convolution（稀疏卷积）跟普通 3D 卷积的本质区别是什么？**

因为 3D 点云极端稀疏（99% 是空气），普通 3D 卷积运行在致密网格上，绝大多数乘加运算都是在跟 0 相乘，极度消耗显存且根本训不动。Sparse Conv 依赖哈希表构建规则字典（Rulebook），只在记录下来的"有数据的活跃 Voxel"上执行卷积核运算，跳过所有空位。速度提升数百倍，内存消耗从 `O(L·W·H)` 降到 `O(M)`（M 为有效体素数）。
