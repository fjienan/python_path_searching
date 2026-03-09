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
