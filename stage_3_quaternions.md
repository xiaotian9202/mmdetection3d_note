# Stage 3: 3D 旋转表征与四元数 (Quaternion)

表征 3D 空间中的物体朝向、相机姿态，如何用数学来表示"旋转"是一个绕不开的坑。在 MMDetection3D 和 nuScenes 的坐标流转中，你会频繁看到四元数的身影。

---

## 1. 物理与几何概念（为何青睐四元数？）

三种常见的 3D 旋转表征方式对比：

| 特性 | 欧拉角 | 旋转矩阵 | 四元数 |
|---|---|---|---|
| 参数数量 | 3 | 9（实际 3 DOF） | 4（实际 3 DOF） |
| 万向节死锁 | **有** | 无 | 无 |
| 插值平滑性 | 差 | 中 | **极佳（Slerp）** |
| 计算效率 | 高 | 中 | 高 |
| 网络直接回归 | 可以 | **极难**（需保持正交性） | 较难（需归一化） |

- **欧拉角（Euler Angles）**：用绕 X, Y, Z 三轴的角度（Roll, Pitch, Yaw）表示，直观但存在**万向节死锁（Gimbal Lock）**——当某个旋转角达到 90° 时丢失一个自由度。
- **旋转矩阵（Rotation Matrix）**：9 个参数只表达 3 个自由度，数据冗余严重，且必须时刻保持正交性，深度学习中很难作为网络输出直接回归。
- **四元数（Quaternion）**：用 4 个数 `[w, x, y, z]` 无歧义地表达任意 3D 旋转，**没有万向节死锁**，支持极度平滑的球面线性插值（Slerp），是自动驾驶时空对齐的首选。

---

## 2. 数学公式推导

### 四元数的旋转轴-角表示

如果一个物体绕着归一化的 3D 轴 `u = (ux, uy, uz)` 旋转了 θ 角度，对应的四元数 q 为：

```math
q = \left[\cos\!\left(\frac{\theta}{2}\right),\quad \sin\!\left(\frac{\theta}{2}\right) u_x,\quad \sin\!\left(\frac{\theta}{2}\right) u_y,\quad \sin\!\left(\frac{\theta}{2}\right) u_z\right]
```

在 nuScenes 和 MMDetection3D 源码中，简写为数组 `[w, x, y, z]`。

### 四元数作用于 3D 点（旋转运算）

要用四元数 q 旋转一个 3D 点 `P = (px, py, pz)`：

1. 将点写为纯四元数形式：`P_q = [0, px, py, pz]`
2. 求共轭（已归一化时等于逆）：`q* = [w, -x, -y, -z]`
3. 旋转后的点由两次四元数乘法得到：

```math
P'_q = q \otimes P_q \otimes q^{*}
```

结果 `P'_q` 的虚部 `(x', y', z')` 就是旋转后的 3D 坐标。

### 旋转矩阵 → 四元数（快速公式）

给定旋转矩阵 R，四元数 w 分量为：

```math
w = \frac{1}{2}\sqrt{1 + R_{00} + R_{11} + R_{22}}
```

其余分量由矩阵元素差值求出（Shepperd's method）。

---

## 3. 精确的源码定位

MMDetection3D 中 `pyquaternion` 的实际使用：

**① 文件路径**：[`mmdet3d/evaluation/metrics/nuscenes_metric.py`](file:///root/mmdetection3d/mmdet3d/evaluation/metrics/nuscenes_metric.py)

**② 应用 1：Yaw 角 → 四元数（提交评估用）**

```python
quat = pyquaternion.Quaternion(axis=[0, 0, 1], radians=box_yaw[i])
```
明确指出旋转轴是 `[0, 0, 1]`（Z 轴），印证了 LiDAR 坐标系的 Yaw 定义。

**③ 应用 2：旋转矩阵 → 四元数（多帧对齐用）**

```python
pyquaternion.Quaternion(matrix=ego2global, rtol=1e-05, atol=1e-07)
```
将 4×4 外参矩阵中的 3×3 旋转矩阵无损转化为四元数，为后续 Slerp 运动补偿做准备。

**④ 文件路径**：[`mmdet3d/datasets/convert_utils.py`](file:///root/mmdetection3d/mmdet3d/datasets/convert_utils.py)

在 nuScenes 数据预处理时从原始 annotation 中提取四元数，并转换为旋转矩阵存入 `.pkl` 文件。

---

## 4. 张量流转分析 (Tensor Flow)

网络 Head 预测朝向角的流转路径（以 CenterPoint 为例）：

| 步骤 | 操作 | 数据格式 |
|---|---|---|
| Head 输出 | 预测 sin(θ) 和 cos(θ) 两个分支 | `(N, 1)` + `(N, 1)` |
| 后处理 | `θ = atan2(sin_pred, cos_pred)` | 标量 θ（rad） |
| 拼接 Box | `[x, y, z, l, w, h, θ]` | `(N, 7)` |
| 评估提交 | `Quaternion(axis=[0,0,1], radians=θ)` | `[w, x, y, z]` 四元数 |
| nuScenes API | 四元数 + 中心点 + 速度 | nuScenes Box 对象 |

---

## 5. 高频面试考点总结

**🔥 考点 1：3D 检测网络 Head 为什么只回归 Yaw 角，而不回归完整的 Roll、Pitch？**

自动驾驶的大部分场景符合"2.5D 假设"——汽车基本行驶在平坦地面上，Roll 和 Pitch 几乎为零。强行让网络预测这两个额外自由度不仅信噪比极低，还会大幅增加收敛难度。直接回归绕 Z 轴的 Yaw 角已满足绝大多数 3D Bounding Box 的定位需求。

**🔥 考点 2：为什么连续帧点云融合时，需要用四元数 Slerp 插值，而不能直接对欧拉角做线性插值？**

欧拉角在数学上是不连续的：从 179° 线性插值到 -179°，会错误地绕过整个 360° 而非直接跨过 2°。同时欧拉角存在万向节死锁。四元数存在于单位超球面上，Slerp（球面线性插值）保证了物体在两姿态之间沿最短路径、以恒定角速度旋转，对高精度的 Ego 运动补偿至关重要。
