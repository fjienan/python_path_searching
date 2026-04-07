# python_path_searching 包使用说明

## 🚀 Quick Start（快速开始）

### 1️⃣ Clone 下载

```bash
cd ~/ros2_ws/src
git clone https://github.com/fynngwu/python_path_searching.git
# 或者如果已经下载了仓库，直接确保包位于 ~/ros2_ws/src/python_path_searching/
```

### 2️⃣ 安装依赖

```bash
# ROS2 依赖（以 Humble 为例）
sudo apt install ros-humble-rclpy ros-humble-nav-msgs ros-humble-geometry-msgs \
  ros-humble-visualization-msgs ros-humble-sensor-msgs

# Python 依赖
pip3 install numpy opencv-python pyyaml
```

### 3️⃣ 编译包

```bash
cd ~/ros2_ws
# 只编译 python_path_searching 包
colcon build --packages-select python_path_searching --symlink-install
```

### 4️⃣ 启动

```bash
source install/setup.bash
ros2 launch python_path_searching path_tracking.launch.py
```

---

## 📋 详细说明

本说明文档介绍如何在 `ros2_ws` 工作空间中**只编译这个包**，然后 source 环境并启动本包中的 Python launch 文件。

### 环境准备

1. 已正确安装 ROS 2（例如 Humble、Iron 等）。
2. 已创建工作空间，并将本包放在：

```bash
cd ~/ros2_ws
# 只编译 python_path_searching 包
colcon build --packages-select python_path_searching
source install/setup.bash
ros2 launch python_path_searching path_tracking.launch.py
```


---

## 🤖 omnidirectional_tracker_node.py 功能说明

### 概述
`omnidirectional_tracker_node.py` 实现了机器人的路径跟踪控制，支持梯形速度规划和定位模块集成。

### 主要功能

#### 1. 接收定位模块实时位置
- **订阅话题**: `/localization/pose` (geometry_msgs/PoseStamped)
- **QoS**: 可靠性传输，TRANSIENT_LOCAL持久化
- **功能**: 接收定位模块发布的机器人实时位姿，更新当前机器人位置和偏航角

#### 2. 坐标转换
- **map_to_grid_coords()**: 将物理坐标(米)转换为网格索引
  - 输入: 物理坐标 (x, y)
  - 输出: 网格坐标 (row, col)
  - 转换公式:
    ```python
    row = floor((x - origin_x) / resolution)
    col = floor((y - origin_y) / resolution)
    ```
  - 参数:
    - `origin_x = 3.2`, `origin_y = 1.2` (地图原点)
    - `resolution = 1.2` (格子边长，单位: 米)

#### 3. 梯形速度控制
- **trap_velocity_control()**: 实现从格子中心到下一个格子中心的加减速控制
  - **三阶段控制** (格子间距1.2米):
    1. **加速阶段**: 距离 > 0.8m 时，加速到最大速度
    2. **匀速阶段**: 0.8m ≥ 距离 > 0.4m 时，保持最大速度
    3. **减速阶段**: 距离 ≤ 0.4m 时，线性减速至目标点
  - **参数**:
    - `trap_max_vel = 0.3 m/s` (最大速度)
    - `trap_accel_distance = 0.4 m` (加速距离)
    - `trap_decel_distance = 0.4 m` (减速距离)
  - **输出**: (velocity_x, velocity_y, direction) - x/y速度和朝向

#### 4. 速度命令输出
- **发布话题**: `/cmd_vel` (geometry_msgs/Twist)
- **频率**: 50 Hz
- **内容**:
  - `linear.x`: x方向速度
  - `linear.y`: y方向速度
  - `angular.z`: 旋转角度/方向

#### 5. 机器人分布位置发布
- **发布话题**: `/robot_distribution` (geometry_msgs/PoseStamped)
- **频率**: 每次接收定位位置时发布
- **功能**: 向其他模块提供机器人的实时分布位置
- **坐标系**: map坐标系

### 订阅话题

| 话题 | 类型 | 功能 |
|------|------|------|
| `/localization/pose` | PoseStamped | 接收定位模块实时位置 |
| `/path_planning/path` | Path | 接收路径规划结果 |
| `/odom_world` | Odometry | 接收里程计数据 |
| `/kfs_grid_data` | String | 接收KFS网格数据 |

### 发布话题

| 话题 | 类型 | 功能 |
|------|------|------|
| `/cmd_vel` | Twist | 发送速度控制命令 |
| `/robot_distribution` | PoseStamped | 发布机器人分布位置 |

### 网格配置

- **网格大小**: 4行 × 3列
- **地图原点**: [3.2, 1.2, 0.0] (米)
- **分辨率**: 1.2 米/格
- **目标到达阈值**: 0.1 米

### 控制参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `kp_linear` | 5.0 | 线性速度PID比例系数 |
| `kd_linear` | 0.1 | 线性速度PID微分系数 |
| `kp_angular` | 10.0 | 角速度PID比例系数 |
| `max_linear_vel` | 0.5 m/s | 最大线速度 |
| `max_angular_vel` | 5.0 rad/s | 最大角速度 |
| `control_frequency` | 50 Hz | 控制频率 |

