# MMDetection3D 核心数学与感知算法实战笔记

> 🚗 本项目为 **MMDetection3D 与自动驾驶三维感知系统** 的系统化进阶学习笔记。
> 从初高中数学几何直觉出发，推导工业级矩阵与三维投影公式，并逐行对齐 MMDetection3D 开源算法源码实现。配有全自适应高清矢量原理图（SVG），开箱即用。

---

## 📚 章节导航 (Table of Contents)

| 阶段 | 核心主题 | 文档链接 | 核心内容提要 |
|:---:|:---|:---:|:---|
| **Stage 1** | **坐标系体系** | [stage_1_coordinate_systems.md](./stage_1_coordinate_systems.md) | LiDAR、Camera、Ego（底盘车体）、World 坐标系定义、右手定则与正负朝向规范 |
| **Stage 2** | **刚体变换与投影数学** | [stage_2_rigid_transformation.md](./stage_2_rigid_transformation.md) | 2D/3D 旋转矩阵、SE(3) 齐次变换、逆变换性质、针孔相机双平面模型、KITTI 全局极线校正投影链、nuScenes 时空补偿、MMDetection3D 核心函数剖析 |
| **Stage 3** | **四元数与空间旋转** | [stage_3_quaternions.md](./stage_3_quaternions.md) | 欧拉角万向节死锁问题、四元数代数运算、轴角转换与 SLERP 球面线性插值 |
| **Stage 4** | **空间外参对齐与标定** | [stage_4_spatial_alignment.md](./stage_4_spatial_alignment.md) | 手眼标定（Hand-Eye）、ICP 点云配准、多传感器联合外参优化与视锥过滤 |
| **Stage 5** | **时间戳对齐与时空补偿** | [stage_5_temporal_alignment.md](./stage_5_temporal_alignment.md) | 传感器异步曝光机制、自车高频 IMU/里程计运动补偿算法流水线 |
| **Stage 6** | **LiDAR 点云到 BEV 特征** | [stage_6_lidar_to_bev.md](./stage_6_lidar_to_bev.md) | 体素化（Voxelization）、PointPillars 柱体编码与稀疏卷积网络实现 |
| **Stage 7** | **相机图像到 BEV 视锥转换** | [stage_7_camera_to_bev.md](./stage_7_camera_to_bev.md) | LSS (Lift-Splat-Shoot) 深度分布预测、BEVDet 与 BEVDepth 视锥池化原理 |
| **Stage 8** | **跨模态前融合与 BEV 融合** | [stage_8_camera_lidar_fusion.md](./stage_8_camera_lidar_fusion.md) | PointPainting 早期融合、BEVFusion 多模态特征级融合架构与后处理评测 |

---

## 🎨 矢量原理图集 (Vector Diagrams)

项目内置 6 张精心绘制的高清矢量图（SVG），采用**自适应响应式布局（Adaptive Layout）**，无论在电脑端全屏还是移动端/窄窗口下阅读，文字与图元均不重叠：

1. **[自动驾驶多传感器坐标系流转全景架构](./coordinate_systems_flow.svg)**
   - 全景展示 World ➔ Ego ➔ LiDAR / Camera ➔ Image 平面的刚体变换流转链路。
2. **[2D 平面旋转几何原理与极坐标推导](./rotation_2d_polar.svg)**
   - 从高中和角公式到 $2\times2$ 正交旋转矩阵的零门槛几何推导。
3. **[针孔相机成像与双平面立体模型](./camera_pinhole_planes.svg)**
   - 物理像平面（$Z=f$）与归一化平面（$Z=1$）解耦硬件与纯几何的三维立体视锥模型。
4. **[相似直角三角形推导归一化坐标](./similar_triangles_depth.svg)**
   - 初中 AA 相似三角形严密证明 $x_n = X_c / Z_c$ 的透视除法几何根基。
5. **[KITTI 标定全链路与立体极线校正架构](./kitti_projection_chain.svg)**
   - 彻底揭示为何激光雷达点云必须先经 Cam0 锚点并补乘 $\mathbf{R}^{(0)}_{\text{rect}}$ 摆正矩阵。
6. **[KITTI 航向角与观测角几何解耦](./kitti_yaw_vs_alpha.svg)**
   - 鸟瞰图（BEV）解析全局航向角（$\text{rotation\_y}$）、视线方位角（$\beta$）与观测角（$\alpha$）的换算关系。

---

## 🛠️ 技术特点

- **零门槛推导**：复杂数学推导从零开始，拒绝公式直接倾倒，高中生亦可读懂；
- **源码逐行对照**：关键算法严格对照 `mmdet3d/core/bbox` 与 MMDetection3D 官方源码；
- **工业界实战真知**：直击 KITTI / nuScenes 标定文件结构、多目立体校正、时间戳运动畸变等真实工程坑点；
- **高频面试真题精讲**：精选自动驾驶算法与感知融合岗位高频技术面试题及参考解答。

---

## 📄 License

本项目内容采用 [MIT License](https://opensource.org/licenses/MIT) 开源许可，供学术研究与工程技术交流使用。
