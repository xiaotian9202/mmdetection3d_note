# Stage 1: 多传感器坐标系拓扑

坐标系可以说是 3D 感知中最基础、也最容易让人在工程中栽跟头的地方。各大厂的感知库（如 MMDetection3D, OpenPCDet）都有自己的一套坐标系封装逻辑。
我们将严格按照 5 个维度来彻底拆解 MMDetection3D 中的坐标系设计。

---

## 1. 物理与几何概念（直击本质）

在一辆自动驾驶汽车（Ego Vehicle）上，通常挂载了多个异构传感器。为了将它们观测到的数据统一处理，我们需要明确几大坐标系的几何拓扑（均为**右手坐标系**）：

| 坐标系 | 原点 | X 轴 | Y 轴 | Z 轴 | 旋转轴 | 底面中心 |
|---|---|---|---|---|---|---|
| **LiDAR** | LiDAR 几何中心 | 向前 | 向左 | 向上 | Z 轴 (axis=2) | `(0.5, 0.5, 0)` |
| **Camera** | 相机光学中心 | 向右 | 向下 | 向前（深度） | Y 轴 (axis=1) | `(0.5, 1.0, 0.5)` |
| **Depth** | 深度相机原点 | 向右 | 向前 | 向上 | Z 轴 (axis=2) | `(0.5, 0.5, 0)` |
| **Ego (Vehicle)** | 车辆后轴中心/IMU | 向前 | 向左 | 向上 | — | — |
| **Global (World)** | 地球参考点（如 UTM） | 东 | 北 | 上 | — | — |
| **Pixel (Image)** | 图像左上角 | u 向右 | v 向下 | — | — | — |

### 三个核心坐标系的 ASCII 可视化

```
LiDAR 坐标系               Camera 坐标系              Depth 坐标系

     up z    x front              z front               up z    y front
        ^   ^                    /                         ^   ^
        |  /                    /                          |  /
        | /                    0 ------> x right           | /
 left y <--- 0                 |                           0 ------> x right
                               |
                               v
                          down y
```

### 各坐标系的角色说明

- **LiDAR 坐标系**：最符合人类直觉，车辆正前方就是 X 轴，高度是 Z 轴。点云数据默认在此空间下。
- **Camera 坐标系**：由小孔成像模型决定。Z 轴代表深度，Y 轴向下是因为图像像素坐标起点在左上角。
- **Depth 坐标系**：主要用于 VoteNet、H3DNet 等基于深度图的室内 3D 检测。与 LiDAR 系类似（Z 轴向上），但 X 轴指向右方、Y 轴指向前方。
- **Ego 坐标系**：所有传感器数据的统一"集散地"，多传感器对齐的桥梁坐标系。
- **Global 坐标系**：用于高精地图匹配、时空对齐和多车协同。
- **Pixel 坐标系**：2D 图像平面，用于 3D 到 2D 的最终投影。

> ⚠️ **重要提示**：MMDetection3D 在 v1.0.0rc0 进行了重大重构，统一将三个核心坐标系（LiDAR、Camera、Depth）设为**右手坐标系**，偏航角（Yaw）统一为**逆时针方向递增**（从重力轴负方向看）。旧版 KITTI 等 LiDAR 坐标系曾使用左手定义，不可混用。

---

## 2. 数学公式推导（3D Box 的拓扑约定）

一个标准的 3D 边界框（3D Box）由 7 个参数描述：`[x, y, z, dx, dy, dz, r]`（或记作 `[x, y, z, x_size, y_size, z_size, yaw]`）。在不同的坐标系下，这些参数的物理含义和数学基准完全不同。

### LiDAR Box 约定
- 中心点 `[x, y, z]` 默认在 Box 的**底部中心**，相对比例为 `(0.5, 0.5, 0)`。
- **维度含义**：`dx`(x_size) = 沿 X 轴（前后长 l）、`dy`(y_size) = 沿 Y 轴（左右宽 w）、`dz`(z_size) = 沿 Z 轴（上下高 h）。
- 偏航角 r（Yaw）是**绕 Z 轴**旋转的。r=0 时 Box 方向沿 X 轴正方向（正前方），从 +X 向 +Y 逆时针递增。

```
    LiDAR BEV (Bird's Eye View)

              x front (yaw=0)
                ^
                |
                |   __________
                |  |          |  ← dx (along x, 即长 l)
                |  |   Box    |
                |  |__________|  ← dy (along y, 即宽 w)
                |
 (yaw=π/2) y <--- 0
```

### Camera Box 约定
- 中心点 `[x, y, z]` 默认在 Box 的**底部中心**，相对比例为 `(0.5, 1.0, 0.5)`（Y 轴向下，1.0 代表最下方）。
- **维度含义**：`dx`(x_size) = 沿 X 轴（左右宽）、`dy`(y_size) = 沿 Y 轴（上下高）、`dz`(z_size) = 沿 Z 轴（前后深度）。
- 偏航角 r（Yaw）是**绕 Y 轴**旋转的。r=0 时 Box 方向沿 X 轴正方向，从 +X 向 +Z **顺时针递减**（因为 Y 轴向下，右手定则导致旋转方向与 LiDAR/Depth 相反）。

> ⚠️ **注意**：Camera 坐标系中，`height` 属性返回的是 `tensor[:, 4]`（即 `y_size = dy`），而非基类默认的 `tensor[:, 5]`，这是因为 Camera 系的重力方向在 Y 轴。

### Depth Box 约定
- 中心点 `[x, y, z]` 默认在 Box 的**底部中心**，相对比例为 `(0.5, 0.5, 0)`。
- **维度含义**：`dx`(x_size) = 沿 X 轴（左右宽）、`dy`(y_size) = 沿 Y 轴（前后深度）、`dz`(z_size) = 沿 Z 轴（上下高）。
- 偏航角 r（Yaw）是**绕 Z 轴**旋转的。r=0 时 Box 方向沿 X 轴正方向，从 +X 向 +Y 逆时针递增。

### 三种 Box 约定对照表

| 属性 | LiDAR Box | Camera Box | Depth Box |
|---|---|---|---|
| 底面中心 (origin) | `(0.5, 0.5, 0)` | `(0.5, 1.0, 0.5)` | `(0.5, 0.5, 0)` |
| YAW_AXIS | `2` (Z 轴) | `1` (Y 轴) | `2` (Z 轴) |
| `dx` 物理含义 | 长（前后） | 宽（左右） | 宽（左右） |
| `dy` 物理含义 | 宽（左右） | 高（上下） | 深（前后） |
| `dz` 物理含义 | 高（上下） | 深（前后） | 高（上下） |
| Yaw=0 方向 | +X (前) | +X (右) | +X (右) |
| Yaw 递增方向 | 逆时针 (+X→+Y) | 从 +X 到 +Z 递减 | 逆时针 (+X→+Y) |

### LiDAR → Camera 的坐标转换

从 LiDAR 到 Camera 坐标系的转换本质是一个正交轴置换。根据官方文档，**反向推导**：

**点坐标转换**（Camera → LiDAR，官方给出的公式）：
- $x_{LiDAR} = z_{camera}$
- $y_{LiDAR} = -x_{camera}$
- $z_{LiDAR} = -y_{camera}$

**因此 LiDAR → Camera**（取逆）：
- $x_{cam} = -y_{lidar}$
- $y_{cam} = -z_{lidar}$
- $z_{cam} = x_{lidar}$

```math
\mathbf{R}_{lidar \rightarrow cam} =
\begin{bmatrix} 0 & -1 & 0 \\ 0 & 0 & -1 \\ 1 & 0 & 0 \end{bmatrix}
```

**验证**：LiDAR 的前方 (X=1,0,0) → Camera 的 (0,0,1) = 深度方向 ✅
LiDAR 的左方 (0,Y=1,0) → Camera 的 (-1,0,0) = 负 X，即反向右方 ✅
LiDAR 的上方 (0,0,Z=1) → Camera 的 (0,-1,0) = 反向下方 ✅

**逆矩阵 Camera → LiDAR**（正交矩阵转置即逆）：

```math
\mathbf{R}_{cam \rightarrow lidar} = \mathbf{R}_{lidar \rightarrow cam}^T =
\begin{bmatrix} 0 & 0 & 1 \\ -1 & 0 & 0 \\ 0 & -1 & 0 \end{bmatrix}
```

**Box 维度转换**（LiDAR → Camera）：
- $dx_{cam} = dx_{lidar}$（LiDAR 前后长 → Camera X 方向宽，但符号与标号映射变了）
- $dy_{cam} = dz_{lidar}$（LiDAR 高 → Camera Y 方向高）
- $dz_{cam} = dy_{lidar}$（LiDAR 宽 → Camera Z 方向深度）

> 🔑 理解关键：维度置换公式 `dx_cam = dx_lidar, dy_cam = dz_lidar, dz_cam = dy_lidar` 对应源码中 `cat([x_size, z_size, y_size], dim=-1)`。

**Yaw 角转换**：
$$r_{LiDAR} = -\frac{\pi}{2} - r_{camera} \quad \Longleftrightarrow \quad r_{camera} = -\frac{\pi}{2} - r_{LiDAR}$$

---

## 3. 核心源码全景深度剖析与逐行精讲

在 MMDetection3D 中，坐标系统一转换的灵魂位于 `mmdet3d/structures/bbox_3d/`。
我们把最核心的三个代码文件彻底展开，并进行**逐行逐段的源码级精讲**。

### 3.1 转换总线中枢：`Box3DMode.convert` 完整源码与逐行讲解

源码路径：[`mmdet3d/structures/bbox_3d/box_3d_mode.py`](file:///root/mmdetection3d/mmdet3d/structures/bbox_3d/box_3d_mode.py)

```python
@unique
class Box3DMode(IntEnum):
    LIDAR = 0
    CAM = 1
    DEPTH = 2

    @staticmethod
    def convert(
        box: Union[Sequence[float], np.ndarray, Tensor, BaseInstance3DBoxes],
        src: 'Box3DMode',
        dst: 'Box3DMode',
        rt_mat: Optional[Union[np.ndarray, Tensor]] = None,
        with_yaw: bool = True,
        correct_yaw: bool = False
    ) -> Union[Sequence[float], np.ndarray, Tensor, BaseInstance3DBoxes]:
        # [Line 1-2] 源与目标相同，直接返回（快速直通）
        if src == dst:
            return box

        # [Line 4-7] 类型判断：输入可能是 numpy 数组、Box 实例类、或者是 Python 单个 box 的 list/tuple
        is_numpy = isinstance(box, np.ndarray)
        is_Instance3DBoxes = isinstance(box, BaseInstance3DBoxes)
        single_box = isinstance(box, (list, tuple))

        # [Line 8-20] 统一包装为二维 Tensor 便于批量并行矩阵运算
        if single_box:
            assert len(box) >= 7, (
                'Box3DMode.convert takes either a k-tuple/list or '
                'an Nxk array/tensor, where k >= 7')
            arr = torch.tensor(box)[None, :]  # (7,) -> (1, 7)
        else:
            # 深拷贝 clone 避免就地修改外部原始数据
            if is_numpy:
                arr = torch.from_numpy(np.asarray(box)).clone()
            elif is_Instance3DBoxes:
                arr = box.tensor.clone()
            else:
                arr = box.clone()

        # [Line 22-24] 若传入的是面向对象 Box 实例，提取其内在的 with_yaw 状态
        if is_Instance3DBoxes:
            with_yaw = box.with_yaw

        # [Line 26-29] 7DOF 特征切片分离
        # 尺寸切片保留最后一维尺寸 (N, 1)，方便后续按列顺序重新拼接
        x_size, y_size, z_size = arr[..., 3:4], arr[..., 4:5], arr[..., 5:6]
        if with_yaw:
            yaw = arr[..., 6:7]

        # ==================== 六大坐标系分支转换逻辑 ====================

        # 分支 1：LIDAR -> CAM
        if src == Box3DMode.LIDAR and dst == Box3DMode.CAM:
            if rt_mat is None:
                # 默认标准正交轴置换矩阵
                rt_mat = arr.new_tensor([[0, -1, 0], [0, 0, -1], [1, 0, 0]])
            # 尺寸重排：LiDAR 的 (dx, dy, dz) -> Camera 的 (dx, dz, dy)
            xyz_size = torch.cat([x_size, z_size, y_size], dim=-1)
            if with_yaw:
                if correct_yaw:
                    # 构造 LiDAR 坐标系下的 3D 朝向单位矢量 (绕 Z 轴旋转)
                    yaw_vector = torch.cat([
                        torch.cos(yaw),
                        torch.sin(yaw),
                        torch.zeros_like(yaw)
                    ], dim=1)
                else:
                    # 纯平面几何快速经验公式
                    yaw = -yaw - np.pi / 2
                    yaw = limit_period(yaw, period=np.pi * 2)

        # 分支 2：CAM -> LIDAR
        elif src == Box3DMode.CAM and dst == Box3DMode.LIDAR:
            if rt_mat is None:
                # 默认逆转换矩阵
                rt_mat = arr.new_tensor([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
            # 尺寸重排：Camera 的 (dx, dy, dz) -> LiDAR 的 (dx, dz, dy)
            xyz_size = torch.cat([x_size, z_size, y_size], dim=-1)
            if with_yaw:
                if correct_yaw:
                    # 构造 Camera 坐标系下的 3D 朝向矢量 (绕 Y 轴旋转，Y 向下，故 yaw 递减取负角)
                    yaw_vector = torch.cat([
                        torch.cos(-yaw),
                        torch.zeros_like(yaw),
                        torch.sin(-yaw)
                    ], dim=1)
                else:
                    yaw = -yaw - np.pi / 2
                    yaw = limit_period(yaw, period=np.pi * 2)

        # 分支 3：DEPTH -> CAM
        elif src == Box3DMode.DEPTH and dst == Box3DMode.CAM:
            if rt_mat is None:
                rt_mat = arr.new_tensor([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
            xyz_size = torch.cat([x_size, z_size, y_size], dim=-1)
            if with_yaw:
                if correct_yaw:
                    yaw_vector = torch.cat([
                        torch.cos(yaw),
                        torch.sin(yaw),
                        torch.zeros_like(yaw)
                    ], dim=1)
                else:
                    yaw = -yaw

        # 分支 4：CAM -> DEPTH
        elif src == Box3DMode.CAM and dst == Box3DMode.DEPTH:
            if rt_mat is None:
                rt_mat = arr.new_tensor([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
            xyz_size = torch.cat([x_size, z_size, y_size], dim=-1)
            if with_yaw:
                if correct_yaw:
                    yaw_vector = torch.cat([
                        torch.cos(-yaw),
                        torch.zeros_like(yaw),
                        torch.sin(-yaw)
                    ], dim=1)
                else:
                    yaw = -yaw

        # 分支 5：LIDAR -> DEPTH
        elif src == Box3DMode.LIDAR and dst == Box3DMode.DEPTH:
            if rt_mat is None:
                rt_mat = arr.new_tensor([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
            # LiDAR 与 Depth 的高度轴都是 Z，因此尺寸无需对调 (dx, dy, dz)
            xyz_size = torch.cat([x_size, y_size, z_size], dim=-1)
            if with_yaw:
                if correct_yaw:
                    yaw_vector = torch.cat([
                        torch.cos(yaw),
                        torch.sin(yaw),
                        torch.zeros_like(yaw)
                    ], dim=1)
                else:
                    yaw = yaw + np.pi / 2
                    yaw = limit_period(yaw, period=np.pi * 2)

        # 分支 6：DEPTH -> LIDAR
        elif src == Box3DMode.DEPTH and dst == Box3DMode.LIDAR:
            if rt_mat is None:
                rt_mat = arr.new_tensor([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
            xyz_size = torch.cat([x_size, y_size, z_size], dim=-1)
            if with_yaw:
                if correct_yaw:
                    yaw_vector = torch.cat([
                        torch.cos(yaw),
                        torch.sin(yaw),
                        torch.zeros_like(yaw)
                    ], dim=1)
                else:
                    yaw = yaw - np.pi / 2
                    yaw = limit_period(yaw, period=np.pi * 2)
        else:
            raise NotImplementedError(
                f'Conversion from Box3DMode {src} to {dst} is not supported yet')

        # ==================== 中心点刚体变换运算 ====================
        if not isinstance(rt_mat, Tensor):
            rt_mat = arr.new_tensor(rt_mat)

        # 若变换矩阵包含平移外参 (4x4 齐次矩阵)
        if rt_mat.size(1) == 4:
            # 扩展齐次坐标 [x, y, z, 1]
            extended_xyz = torch.cat(
                [arr[..., :3], arr.new_ones(arr.size(0), 1)], dim=-1)
            # 行向量右乘转置矩阵: v' = v @ T.T 等价于列向量左乘 T @ v_col
            xyz = extended_xyz @ rt_mat.t()
        else:
            # 纯 3x3 旋转矩阵直接乘
            xyz = arr[..., :3] @ rt_mat.t()

        # ==================== 严格 3D 向量旋转重映射偏航角 ====================
        if with_yaw and correct_yaw:
            # 将源坐标系下的 3D 朝向向量通过外参矩阵旋转至目标空间
            rot_yaw_vector = yaw_vector @ rt_mat[:3, :3].t()

            # 若目标坐标系是 CAM:
            # Cam 系中 Yaw=0 在 +X，Yaw=-pi/2 在 +Z (深度)，故从 X-Z 平面反解角度为 atan2(-Z, X)
            if dst == Box3DMode.CAM:
                yaw = torch.atan2(-rot_yaw_vector[:, [2]], rot_yaw_vector[:, [0]])
            # 若目标坐标系是 LIDAR 或 DEPTH:
            # 旋转轴均为向上 Z 轴，水平面为 X-Y，直接从 atan2(Y, X) 反解
            elif dst in [Box3DMode.LIDAR, Box3DMode.DEPTH]:
                yaw = torch.atan2(rot_yaw_vector[:, [1]], rot_yaw_vector[:, [0]])

            yaw = limit_period(yaw, period=np.pi * 2)

        # ==================== 结果组装与输出多态包装 ====================
        if with_yaw:
            remains = arr[..., 7:]  # 保留 7 维之后的额外字段 (如速度 vx, vy 或类别)
            arr = torch.cat([xyz[..., :3], xyz_size, yaw, remains], dim=-1)
        else:
            remains = arr[..., 6:]
            arr = torch.cat([xyz[..., :3], xyz_size, remains], dim=-1)

        # 还原回调用者原始的 Python 类型
        original_type = type(box)
        if single_box:
            return original_type(arr.flatten().tolist())
        if is_numpy:
            return arr.numpy()
        elif is_Instance3DBoxes:
            # 面向对象自动类型多态映射
            if dst == Box3DMode.CAM:
                target_type = CameraInstance3DBoxes
            elif dst == Box3DMode.LIDAR:
                target_type = LiDARInstance3DBoxes
            elif dst == Box3DMode.DEPTH:
                target_type = DepthInstance3DBoxes
            else:
                raise NotImplementedError(
                    f'Conversion to {dst} through {original_type} is not supported yet')
            return target_type(arr, box_dim=arr.size(-1), with_yaw=with_yaw)
        else:
            return arr
```

#### 🔍 逐行原理解析要点

1. **输入与输出的透明多态设计**：
   - 算法无论传入 Python 的 `[x,y,z,l,w,h,r]`，还是 NumPy 的 `ndarray`，还是 PyTorch 的 `Tensor`，甚至 MMDet3D 封装好的 `LiDARInstance3DBoxes`，该方法均能自动解包并在内部以高吞吐的 PyTorch 张量并行计算，最后原样打包返回。
2. **为什么矩阵乘法全带 `.t()`（如 `xyz @ rt_mat.t()`）？**
   - 在线性代数传统公式中，坐标是列向量：$\mathbf{p}' = \mathbf{R}\mathbf{p} + \mathbf{t}$。
   - 但在深度学习框架中，张量形状是 `(N, 3)`，每一行代表一个点（行向量）。根据转置乘法规则：
     $$\mathbf{p}'^T = (\mathbf{R}\mathbf{p})^T = \mathbf{p}^T \mathbf{R}^T$$
     因此代码中必须写成 `xyz @ rt_mat.t()`。
3. **`correct_yaw=True` 与 `correct_yaw=False` 的本质区别**：
   - `correct_yaw=False`：直接假设激光雷达与相机安装在**绝对水平面上无任何俯仰和滚转**，直接使用二维标量平面公式 $r_{cam} = -r_{lidar} - \pi/2$。
   - `correct_yaw=True`：考虑了**真实世界中传感器存在 Pitch 和 Roll 安装倾角**。它把当前物体的朝向看成一个真正的三维空间朝向矢量（3D Vector），利用 3×3 旋转外参矩阵乘法完成空间旋转，再将其投影到目标相机的地面水平面上，利用 `torch.atan2` 重新反解偏航角。在越野、颠簸路面或倾斜外参标定下，必须开启此选项！

---

### 3.2 面向对象门面接口：`LiDARInstance3DBoxes.convert_to`

源码路径：[`mmdet3d/structures/bbox_3d/lidar_box3d.py`](file:///root/mmdetection3d/mmdet3d/structures/bbox_3d/lidar_box3d.py)

```python
class LiDARInstance3DBoxes(BaseInstance3DBoxes):
    YAW_AXIS = 2

    def convert_to(self,
                   dst: int,
                   rt_mat: Optional[Union[Tensor, np.ndarray]] = None,
                   correct_yaw: bool = False) -> 'BaseInstance3DBoxes':
        """将自身转换为目标 dst 坐标系模式"""
        from .box_3d_mode import Box3DMode
        return Box3DMode.convert(
            box=self,
            src=Box3DMode.LIDAR,
            dst=dst,
            rt_mat=rt_mat,
            correct_yaw=correct_yaw)
```

- **设计模式**：采用经典的外观模式（Facade）和委托模式。用户只需要调用 `lidar_boxes.convert_to(Box3DMode.CAM, extrinsics)`，无需手动判断源坐标系是什么，类自身通过多态绑定了 `src=Box3DMode.LIDAR`。

---

### 3.3 几何底座：8 角点生成算法 `corners` 源码与数学精讲

无论是计算 3D IoU、点云裁切过滤还是 3D 框绘制，都必须将 7DOF 转换为 8 个空间顶点坐标。

源码路径：[`mmdet3d/structures/bbox_3d/lidar_box3d.py`](file:///root/mmdetection3d/mmdet3d/structures/bbox_3d/lidar_box3d.py)

```python
    @property
    def corners(self) -> Tensor:
        """计算 3D Box 的 8 个角点坐标，形状为 (N, 8, 3)"""
        if self.tensor.numel() == 0:
            return torch.empty([0, 8, 3], device=self.tensor.device)

        dims = self.dims  # (N, 3)，包含 [dx, dy, dz]

        # [Line 1] 生成 8 个顶点的 0-1 二进制组合:
        # np.arange(8) 经过 np.unravel_index(..., [2, 2, 2])
        # 得到: (0,0,0), (0,0,1), (0,1,0), (0,1,1), (1,0,0), (1,0,1), (1,1,0), (1,1,1)
        corners_norm = torch.from_numpy(
            np.stack(np.unravel_index(np.arange(8), [2] * 3), axis=1)).to(
                device=dims.device, dtype=dims.dtype)

        # [Line 2] 重排为空间顺时针连通顺序 (先底面 4 点顺时针，再顶面 4 点顺时针)
        corners_norm = corners_norm[[0, 1, 3, 2, 4, 5, 7, 6]]

        # [Line 3] 减去相对底面中心 (0.5, 0.5, 0)
        # 归一化后的坐标中心移至局部原点: X∈[-0.5, 0.5], Y∈[-0.5, 0.5], Z∈[0, 1.0]
        corners_norm = corners_norm - dims.new_tensor([0.5, 0.5, 0])

        # [Line 4] 缩放回物理尺寸: (N, 1, 3) * (1, 8, 3) -> (N, 8, 3)
        corners = dims.view([-1, 1, 3]) * corners_norm.reshape([1, 8, 3])

        # [Line 5] 绕 Z 轴 (YAW_AXIS = 2) 旋转偏航角
        corners = rotation_3d_in_axis(
            corners, self.tensor[:, 6], axis=self.YAW_AXIS)

        # [Line 6] 加上物体的绝对底面中心坐标完成平移
        corners += self.tensor[:, :3].view(-1, 1, 3)
        return corners
```

#### 📐 角点生成的几何推导步骤：
1. **拓扑编码**：利用三位二进制计数法（0 到 7）列举 3D 立方体的所有可能顶点。
2. **顺时针顺序约束**：将点重排为 `[0, 1, 3, 2, 4, 5, 7, 6]`，确保下底面 4 个点与上顶面 4 个点构成顺时针封闭闭环，方便后续使用凸包（Convex Hull）计算与绘图连线。
3. **坐标原点对齐**：因 LiDAR Box 原点在底面几何中心，减去 `[0.5, 0.5, 0]` 使局部 X/Y 对称居中，而 Z 轴底面为 0、顶面为 1。
4. **尺度与旋转**：先拉伸各轴尺寸，然后通过绕轴旋转函数 `rotation_3d_in_axis` 绕 Z 轴应用旋转矩阵，最后加上平移。

---

### 3.4 角度截断函数：`limit_period` 源码拆解

源码路径：[`mmdet3d/structures/bbox_3d/utils.py`](file:///root/mmdetection3d/mmdet3d/structures/bbox_3d/utils.py)

```python
def limit_period(val: Union[Tensor, np.ndarray],
                 offset: float = 0.5,
                 period: float = np.pi * 2) -> Union[Tensor, np.ndarray]:
    """将连续角度值循环限制在特定区间内，默认约束在 [-pi, pi)"""
    limited_val = val - torch.floor(val / period + offset) * period
    return limited_val
```

- **数学原理**：
  若要求角度落在 $[-T \cdot \text{offset},\, T \cdot (1 - \text{offset}))$ 内（当 $T = 2\pi, \text{offset} = 0.5$ 时，区间即 $[-\pi, \pi)$）：
  $$\text{val}_{\text{limited}} = \text{val} - \left\lfloor \frac{\text{val}}{2\pi} + 0.5 \right\rfloor \times 2\pi$$
  利用 `torch.floor` 向下取整，优雅消除了角度跳变（如 $+1.1\pi$ 会自动折算为 $-0.9\pi$），保证了网络回归时 Loss 计算的连续性。

---

## 4. 张量流转分析 (Tensor Flow)

以 LiDAR Box → Camera Box 为例：

| 步骤 | 操作 | Shape | 详细说明 |
|---|---|---|---|
| 输入 Raw Data | LiDAR 原始标注 | `(N, 7)` 或 `(N, 9)` | 9 维含速度 vx, vy |
| 封装 | `LiDARInstance3DBoxes(tensor)` | `(N, 7)` | 自动转为底面中心 |
| 提取中心点 | `xyz = tensor[..., :3]` | `(N, 3)` | |
| 提取尺寸 | `dims = tensor[..., 3:6]` | `(N, 3)` | `[dx, dy, dz]` |
| 提取 Yaw | `yaw = tensor[..., 6]` | `(N,)` | |
| **中心点变换** | `xyz_new = xyz @ rt_mat[:3,:3].T + rt_mat[:3,3]` | `(N, 3)` | 若 rt_mat 为 4×4 含平移 |
| **维度置换** | `dims_new = cat([dx, dz, dy], dim=-1)` | `(N, 3)` | LiDAR `[dx,dy,dz]` → Camera `[dx,dz,dy]` |
| **Yaw 重映射** | `yaw_new = -π/2 - yaw` | `(N,)` | 经验公式，或 `correct_yaw=True` 经由 3D 向量旋转重投影 |
| 拼接输出 | `cat([xyz_new, dims_new, yaw_new, ...])` | `(N, 7)` | |
| 封装输出 | `CameraInstance3DBoxes(arr)` | `(N, 7)` | |

最终对象存入 `Det3DDataSample.gt_instances_3d.bboxes_3d`，供 Detection Head 使用。

---

## 5. 高频面试考点总结

**🔥 考点 1：为什么要把 3D Box 封装成类，而不是直接传 Tensor？**

因为 3D Box 是**坐标系强依赖**的数据结构。同样是 `(N, 7)` 的 Tensor，第 6 维（Yaw）在 LiDAR 坐标系是绕 Z 轴旋转，在 Camera 坐标系是绕 Y 轴旋转。直接传 Tensor 极易引发坐标系混淆，且 8 个角点的计算（底面中心不同）、IoU 的计算也完全不一样。封装成类通过多态在内部统一解决这些差异，对算法组件上层透明。

**🔥 考点 2：LiDAR 转 Camera 时，长宽高（l, w, h）如何置换？**

LiDAR 坐标系（X 前, Y 左, Z 上）中：`dx=l`（前后长），`dy=w`（左右宽），`dz=h`（上下高）。
Camera 坐标系（X 右, Y 下, Z 前）中：维度置换规则为 `dx_cam = dx_lidar`, `dy_cam = dz_lidar`, `dz_cam = dy_lidar`。
源码中体现为 `torch.cat([x_size, z_size, y_size], dim=-1)`，即保持 `dx` 不变，交换 `dy` 和 `dz` 的位置。
物理直觉：LiDAR 的高度方向（Z）映射到 Camera 的 Y 方向（也是高度），LiDAR 的宽度方向（Y）映射到 Camera 的 Z 方向（深度）。

**🔥 考点 3：为什么 Camera Box 的底面中心是 `(0.5, 1.0, 0.5)` 而不是 `(0.5, 0.5, 0)`？**

因为 Camera 坐标系的 Y 轴**向下**，地面在 Y 轴正方向。一个物体的底部（与地面接触的面）在 Y 轴方向上的坐标最大，对应归一化坐标 `1.0`。而 LiDAR 和 Depth 坐标系中 Z 轴向上，底面的 Z 坐标最小，对应 `0`。这一差异直接影响 `gravity_center` 的计算：
- LiDAR/Depth：`gravity_z = bottom_z + 0.5 * dz`（向上偏移半高）
- Camera：`gravity_y = bottom_y - 0.5 * dy`（向上偏移半高，Y 轴向下所以减）

**🔥 考点 4：三个坐标系之间的 Yaw 角转换公式是什么？**

| 转换方向 | 公式 |
|---|---|
| LiDAR → Camera | $r_{cam} = -\frac{\pi}{2} - r_{lidar}$ |
| Camera → LiDAR | $r_{lidar} = -\frac{\pi}{2} - r_{cam}$ |
| LiDAR → Depth | $r_{depth} = r_{lidar} + \frac{\pi}{2}$ |
| Depth → LiDAR | $r_{lidar} = r_{depth} - \frac{\pi}{2}$ |
| Depth → Camera | $r_{cam} = -r_{depth}$ |
| Camera → Depth | $r_{depth} = -r_{cam}$ |

本质原因：三个坐标系的"参考方向"(Yaw=0)不一样，LiDAR 的 +X 是前方，Camera 的 +X 是右方，二者差 π/2；又因为旋转轴方向不同（Z up vs Y down），需要取反。

**🔥 考点 5：`Box3DMode.convert()` 中 `correct_yaw` 参数到底解决了什么痛点？**

`correct_yaw=False`（默认值）仅使用了平面几何经验公式（如 $r_{cam} = -r_{lidar} - \pi/2$），该公式成立的前提是**传感器严格水平安装，不存在任何俯仰角（Pitch）和滚转角（Roll）**。
然而在实际自动驾驶车辆上，传感器标定外参往往存在微小的 Pitch 和 Roll 倾角。当 `correct_yaw=True` 时：
1. 源码先在源空间构造出一个真正的三维朝向单位矢量（如 LiDAR 下为 `[cos(yaw), sin(yaw), 0]`）。
2. 将该矢量乘以真实的空间 3×3 旋转外参矩阵 `rot_yaw_vector = yaw_vector @ rt_mat[:3, :3].t()`。
3. 最后投影到目标传感器坐标系的水平面，使用 `torch.atan2` 重新反解出真实的偏航角。
这一机制杜绝了因传感器安装倾斜导致的角度转换累积误差。

---

## 6. 常见踩坑点

| 坑 | 说明 | 解决方式 |
|---|---|---|
| KITTI 旧版 LiDAR 是左手系 | SECOND 等早期框架的 LiDAR box 为 `(w,l,h)` 且 Yaw 方向相反 | 使用 MMDet3D 的数据转换脚本，不要手动处理 |
| NuScenes box 维度顺序 | NuScenes SDK 中 Box 的前两个维度是 `(dy, dx)` 即 `(w, l)` | MMDet3D 内部已做转换，直接使用即可 |
| Camera BEV | Camera BEV 是 `(x, z, dx, dz, -r)` 而非 `(x, y, ...)` | Y 轴是高度轴（重力轴），BEV 忽略 Y 轴 |
| `rt_mat` 来源混乱 | 不同数据集给的标定矩阵可能是 LiDAR→Cam 或 Cam→LiDAR | 务必确认方向，用错方向会导致投影结果镜像翻转 |
| RoI-Aware Pooling 坐标限制 | 该算子仅支持 LiDAR/Depth Box，不支持 Camera Box | 使用前先转换到目标坐标系 |
