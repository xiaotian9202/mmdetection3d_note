# Stage 5: 时间同步与运动补偿 (Temporal Alignment)

在实际行驶中，车辆以 120km/h 飞驰时，LiDAR 两次扫描之间（10Hz，间隔 0.1s）车辆已经开出了 3.3 米。若不做时间补偿，上一帧的点云直接叠加当前帧，同一个静止路灯会出现 3.3 米的"重影"！

---

## 1. 物理与几何概念

- **时间戳对齐（Timestamp Synchronization）**：异构传感器触发频率不同（LiDAR 10Hz，Camera 最高 60Hz）。通常选 LiDAR 关键帧的时间戳 T_ref 为基准，在其他传感器中寻找距 T_ref 最近的帧作为匹配帧。
- **自车运动补偿（Ego Motion Compensation）**：把历史时刻 `T_past` 接收到的点云，转换到当前时刻 `T_cur` 的 Ego 坐标系下，假装这些点是当前时刻刚刚扫描出来的。
- **运动物体的"彗星尾巴"**：运动补偿只消除了 Ego 自身的运动，路上其他运动的车辆在多帧拼接后会拖出位移轨迹，这恰恰为网络提供了估计目标速度的依据。

---

## 2. 数学公式推导

要把历史 Ego 坐标系下的点转到当前 Ego 坐标系，以 Global（世界）坐标系为不变桥梁：

**Step 1：历史 Ego → Global（历史时刻的车体位姿）**

```math
P_{global} = T_{ego2global}(t_{past}) \cdot P_{past\_ego}
```

**Step 2：Global → 当前 Ego（当前时刻的车体位姿的逆）**

```math
P_{cur\_ego} = T_{ego2global}(t_{cur})^{-1} \cdot P_{global}
```

**合并（点云多帧拼接的核心算式）**

```math
P_{cur\_ego} = T_{ego2global}(t_{cur})^{-1} \cdot T_{ego2global}(t_{past}) \cdot P_{past\_ego}
```

这个从左到右的乘法链，正是源码中 `np.linalg.inv(cur_ego2global).dot(prev_ego2global)` 的数学本质。

---

## 3. 精确的源码定位

MMDetection3D 为了追求极致的 DataLoader 速度，做了一个工程优化：**把矩阵乘法在离线生成 `.pkl` 时就提前算好，存为 `lidar2sensor` 矩阵**，避免每次 DataLoader 都重新求逆和连乘。

**① 核心类**：`LoadPointsFromMultiSweeps`
- 文件：[`mmdet3d/datasets/transforms/loading.py`](file:///root/mmdetection3d/mmdet3d/datasets/transforms/loading.py)，L316~L453
- 调用入口：`transform(results)` 方法

**② 核心代码（L441~L446）**：

```python
sweep_ts = sweep['timestamp']
lidar2sensor = np.array(sweep['lidar_points']['lidar2sensor'])
# 旋转：历史点云坐标系 → 当前 LiDAR 坐标系
points_sweep[:, :3] = points_sweep[:, :3] @ lidar2sensor[:3, :3]
# 平移
points_sweep[:, :3] -= lidar2sensor[:3, 3]
# 写入时间差通道（Δt）
points_sweep[:, 4] = ts - sweep_ts
```

`lidar2sensor` 就是在 pkl 预处理阶段离线计算好的 `T_{ego2global}(t_cur)^{-1} · T_{ego2global}(t_past) · T_{lidar2ego}`。

---

## 4. 张量流转分析 (Tensor Flow)

以 nuScenes 10 帧融合为例：

| 步骤 | 操作 | Shape |
|---|---|---|
| 加载关键帧 | 当前帧 LiDAR 点云 | `(N_0, 5)` — `[x,y,z,r,0]` |
| 循环加载历史帧 | 读取 sweep[i] 的点云 | `(N_i, 5)` |
| 坐标补偿 | `pts[:, :3] @ lidar2sensor[:3,:3] - t` | `(N_i, 3)` 空间对齐 |
| 写入时间差 | `pts[:, 4] = Δt = t_cur - t_past` | `(N_i, 1)` 时间通道 |
| 拼接所有帧 | `cat(sweep_points_list)` | `(sum_N, 5)` |
| 最终输出 | 送入 VoxelEncoder | `(sum_N, 5)` — `[x,y,z,r,Δt]` |

`Δt` 通道是关键：网络通过学习这个时间差，结合物体在多帧中的位移，可以直接预测目标的速度向量 `(vx, vy)`。

---

## 5. 高频面试考点总结

**🔥 考点 1：做运动补偿时需要补偿动态障碍物（如其他行驶中的车）吗？**

不需要，也不能补偿！运动补偿公式的本质是消除**自车 Ego** 的运动。补偿后，静止物体（树木、建筑）的多帧点云会完美重合；而运动物体（其他车）会拖出"彗星尾巴"。恰恰是这条尾巴，配合每个点携带的 `Δt` 通道，让 3D 检测网络能够直接从点云中预测目标速度（Velocity），这是单帧点云绝对做不到的。

**🔥 考点 2：纯视觉 BEV（如 BEVFormer）的时间对齐是怎么做的？可以直接挪动像素吗？**

不能直接挪像素——图像是密集 2D Grid，无法像点云一样直接乘矩阵挪位置。两种主流方案：
1. **特征空间对齐（BEVDepth/BEVStereo）**：先把历史帧图像转成历史 BEV 特征（历史 Ego 空间），再用 Ego Motion 矩阵结合 `grid_sample` 双线性插值，将历史 BEV 特征"拉拽"对齐到当前 BEV 空间，最后 Concat。
2. **查询空间对齐（BEVFormer）**：当前帧的 BEV Query 通过 Ego Motion 补偿矩阵，反推它在历史帧 Ego 坐标系下的 3D 空间位置，然后投影回历史帧 2D 图像去采样特征（Temporal Self-Attention）。
