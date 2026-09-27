# Stage 4: 空间对齐 (Spatial Alignment) 源码剖析

多传感器数据不仅格式异构，它们在空间上的相对位置也完全不同。将它们对齐到同一个坐标系下，即"空间对齐"，做不好融合出的特征就会出现严重的"重影"或"偏差"。

---

## 1. 物理与几何概念：坐标系转换 vs 位姿表达

阅读源码和数据集时，必须严格区分以下两个概念：

| 概念 | 定义 | 用法 |
|---|---|---|
| **坐标系转换矩阵** | 把同一个点从 A 坐标系投影到 B 坐标系 | `P_B = T(A→B) · P_A` |
| **位姿矩阵（Pose）** | 传感器 A 在坐标系 B 中的位置和朝向 | **数值上完全等价于上述转换矩阵** |

举例：`lidar2ego` 这个矩阵，既代表了 LiDAR 传感器在车身（Ego）坐标系下的安装位置和朝向，又代表了 `P_ego = T_lidar2ego · P_lidar` 的数学计算。两者是同一个矩阵，只是视角不同。

三个核心对齐矩阵：
- **`lidar2ego`**：LiDAR → 车身坐标系（外参，一旦标定通常固定不变）。
- **`cam2ego`**：相机 → 车身坐标系（外参，每个相机各一个矩阵）。
- **`ego2global`**：车身 → 世界坐标系（随时间剧烈变化，由 GPS/IMU 实时推算）。

---

## 2. 数学公式推导：以 Ego 坐标系为桥梁

### Camera 1 → Camera 2 跨相机对齐

不直接标定两相机间的关系，而是通过 Ego 坐标系中转（星型拓扑）：

**Step 1：Camera 1 → Ego**

```math
P_{ego} = T_{cam1\_to\_ego} \cdot P_{cam1}
```

**Step 2：Ego → Camera 2**

```math
P_{cam2} = T_{cam2\_to\_ego}^{-1} \cdot P_{ego}
```

**合并（完整链路）**

```math
P_{cam2} = T_{cam2\_to\_ego}^{-1} \cdot T_{cam1\_to\_ego} \cdot P_{cam1}
```

### LiDAR → Global 完整链路

```math
P_{global} = T_{ego2global} \cdot T_{lidar2ego} \cdot P_{lidar}
```

### 外参矩阵求逆的快速公式

由于外参矩阵由旋转矩阵 R 和平移向量 t 组成，其逆矩阵有解析解，**无需高斯消元**：

```math
T^{-1} =
\begin{bmatrix} R & t \\ 0 & 1 \end{bmatrix}^{-1}
=
\begin{bmatrix} R^T & -R^T t \\ 0 & 1 \end{bmatrix}
```

只需对 R 做转置（O(1)），再计算 `-R^T t`，效率极高。

---

## 3. 精确的源码定位

**① 数据解析层**：[`mmdet3d/datasets/nuscenes_dataset.py`](file:///root/mmdetection3d/mmdet3d/datasets/nuscenes_dataset.py)

```python
camera_info['ego2global'] = info['ego2global']
```
从 `.pkl` 数据字典中剥离标定参数，缓存进 `info` 字典。

**② 时序多视图对齐层**：[`mmdet3d/datasets/transforms/loading.py`](file:///root/mmdetection3d/mmdet3d/datasets/transforms/loading.py)，L123~L151

核心逻辑（历史帧 `lidar2cam` 转换为当前 Lidar → 历史 Camera 的变换矩阵）：

```python
cur2prev = np.linalg.inv(pad_prev_ego2global).dot(pad_cur_ego2global)
results['lidar2cam'][idx] = results['lidar2cam'][idx].dot(cur2prev)
```

**③ 评测还原层**：[`mmdet3d/evaluation/metrics/nuscenes_metric.py`](file:///root/mmdetection3d/mmdet3d/evaluation/metrics/nuscenes_metric.py)

将网络预测的 Ego 坐标系 Box 转换回 Global 坐标系（先旋转后平移）：

```python
box.rotate(pyquaternion.Quaternion(matrix=ego2global))
box.translate(ego2global_translation)
```

---

## 4. 张量流转分析 (Tensor Flow)

以 LiDAR 点云投影到 Camera 图像（PointPainting 的第一步）为例：

| 步骤 | 操作 | Shape |
|---|---|---|
| 输入点云 | LiDAR 坐标系 | `(N, 3)` |
| 补齐齐次坐标 | `cat([pts, ones])` | `(N, 4)` |
| **LiDAR → Ego** | `pts @ lidar2ego.T` | `(N, 4)` |
| **Ego → Camera** | `pts @ inv(cam2ego).T` | `(N, 4)` |
| **Camera → Pixel** | `pts @ K.T` + 深度归一化 | `(N, 2)` 像素坐标 |
| 查找图像特征 | 双线性插值采样 | `(N, C)` 图像特征 |

关键数据字典变化：
- 输入 dict：包含 `lidar2ego (4×4)`, `cam2ego (4×4)`, `cam2img (3×4)`
- 输出 dict：追加 `lidar2cam (4×4)` = `inv(cam2ego) @ lidar2ego`

---

## 5. 高频面试考点总结

**🔥 考点 1：外参矩阵求逆为什么可以用 `R.T` 代替通用求逆？这在工程中有多重要？**

旋转矩阵 R 是正交矩阵，满足 `R^T = R^{-1}`。因此 4×4 外参矩阵的逆矩阵有解析解（如上述公式），无需调用代价高昂的高斯消元法（O(n³)）。在实时推理中，每帧需要对 6 个相机各求一次逆，若使用 `torch.inverse()` 则会串行执行，而解析公式可以并行化为几次矩阵乘法，在高频调用场景下有数量级的速度差异。

**🔥 考点 2：为什么多传感器融合网络都以 Ego 坐标系为统一基准，而不用 Global 坐标系？**

两个核心原因：
1. **数值稳定性**：Global 坐标系基于 UTM 或经纬度投影，点的坐标数值极其庞大（离原点可能有几万公里）。在 FP32 单精度浮点的 GPU 计算中会导致"大数吃小数"的精度截断，使网络无法收敛。
2. **平移不变性**：以 Ego 为原点，网络学习到的是"距离我 5 米有个行人"，这具有极强的泛化能力。使用 Global 坐标系则意味着网络还要额外拟合车辆当前的绝对地理位置，既没有必要，也严重损害泛化性。
