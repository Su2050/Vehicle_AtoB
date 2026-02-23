# Isaac Lab 迁移指南

本项目目前使用基于运动学（Kinematics）的 2D A* 路径规划。若要迁移到 NVIDIA Isaac Lab（基于物理的 3D 仿真环境），需要进行以下适配：

## 1. 坐标系与单位
- **本项目**: 2D 平面 (x, y, theta)。单位：米。
- **Isaac Lab**: 3D 空间 (x, y, z, quaternion)。单位：米。
  - 映射：2D `(x, y)` -> 3D `(x, y, 0)`。
  - 旋转：2D `theta` -> 3D Quaternion `(0, 0, sin(theta/2), cos(theta/2))` (绕 Z 轴旋转)。

## 2. 碰撞检测 (Collision Detection)
- **本项目 (`check_collision`)**: 使用硬编码的数学不等式（如 `nx > 3.0`）和几何形状（漏斗形安全走廊）。
- **Isaac Lab**: 使用 **PhysX 物理引擎**。
  - **静态环境**: 将墙壁、货架导入为 USD (Universal Scene Description) 资产，并添加 `Collision API`。
  - **动态干涉**: 叉车与环境的碰撞将由物理引擎自动计算（接触力、穿透深度）。
  - **迁移建议**: 
    - 不要移植 `check_collision` 函数。
    - 而是创建对应的 3D 资产（如创建一个长方体代表墙壁，放置在 x=3.0 处）。
    - 安全走廊（Safe Corridor）如果不是物理墙壁而是虚拟限制，可以在 Isaac Lab 中通过 `Cost Function` 或 `Reward Function`（强化学习中）来实现，或者在控制器中作为约束条件。

## 3. 叉车模型 (Robot Model)
- **本项目**: 简单的矩形 + 运动学自行车模型 (Bicycle Model)。
- **Isaac Lab**: 需要完整的 **URDF** 或 **USD** 机器人模型。
  - 包含车轮、车体、货叉的物理属性（质量、惯量、摩擦系数）。
  - 驱动方式：从直接设置 `(x, y, theta)` 变为控制 **关节速度/力矩** (Velocity/Effort Control)。
  - **迁移建议**:
    - 寻找现成的叉车 USD 资产（NVIDIA Nucleus 服务器上有提供）。
    - 配置 `Articulation` 根节点。
    - 驱动轮关节配置为 `Drive` 模式。

## 4. 控制与执行 (Control & Execution)
- **本项目**: `plan_path` 输出离散的位姿序列 `(x, y, theta)`。
- **Isaac Lab**: 机器人需要连续的控制信号。
  - **方案 A (轨迹跟踪)**: 使用 PID 或 MPC (Model Predictive Control) 控制器，让 Isaac Lab 中的叉车跟随本项目生成的路径点。
  - **方案 B (端到端 RL)**: 如果使用 Isaac Lab 训练强化学习策略，本项目的 A* 算法可以作为 "Teacher Policy" 或用于生成参考轨迹（Reference Trajectory）。

## 5. 传感器仿真 (Sensors)
- **本项目**: 假设拥有完美的全局定位 (God View)。
- **Isaac Lab**: 可以仿真真实的传感器。
  - **Lidar/Camera**: 用于感知障碍物。
  - **IMU/Odometry**: 用于估计自身位置。
  - 迁移后，路径规划算法可能需要改为基于局部地图（Local Map）或占用栅格（Occupancy Grid）的规划，以适应感知的不确定性。
