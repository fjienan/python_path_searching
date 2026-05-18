# Python Path Searching

基于 ROS2 的森林搜索路径规划系统，使用 DFS 算法进行全局路径规划，逐段 PID 控制实现路径跟踪。

## 项目结构

```
ros2 topic pub /can_go std_msgs/msg/Bool "data: true"
```

```
python_path_searching/
├── core/                    # 核心算法和工具类
│   ├── astar.py            # A* 路径规划算法（已弃用）
|   ├── dfs.py              # DFS 路径规划算法
│   ├── grid_utils.py       # 网格坐标转换工具
│   ├── pid_controller.py   # PID 控制器
│   └── transform_utils.py   # 坐标变换工具（四元数/欧拉角）
│
├── app/                     # ROS2 节点（主程序）
│   ├── astar_planner_node.py    # A* 路径规划节点（已弃用）
│   ├── dfs_planner_node.py      # DFS 路径规划节点
│   ├── tracker_node.py          # 轨迹跟踪节点（统一节点）
│   ├── path_decision_node.py    # 路径决策节点（已弃用）
│   └── odom_simulator.py        # 里程计模拟器
│
├── config/
│   └── para.yaml              # 可调参数配置
│
└── launch/
    └── path_tracking.launch.py  # 启动文件
```

## 系统流程

```
手动输入 KFS 分布
       ↓
astar_planner_node → /planning/path (关键点 + 目标姿态 yaw)
       ↓
tracker_node → /cmd_vel (逐段 PID 控制)
       ↓
odom_simulator ← /cmd_vel 积分 → /odom_world
       ↓
path_decision_node → /can_go (关键点处是否允许通过)
       ↓ 反馈
   回到 tracker_node
```

### 状态机流程

轨迹跟踪节点

状态机流程：
  单向/全向模式：HOLD → MOVE → ADJ(if precision needed) → HOLD(if can_go required) → MOVE...
           - HOLD: 停在关键点，等 can_go=True 才继续
           - MOVE: 角度大于阈值时先修正，直线前进到目标点，实时校正偏离直线误差、角度误差（阈值较大）
           - ADJ: 如需精细化对齐才进入（阈值较小），如果在某点不需要抓取kfs操作则不会进入 ADJ，以加快运行速度。

  全向模式获得的路径无需旋转对齐，直接控制 x/y 轴速度即可；单向模式需要先旋转对齐，再控制前进速度，侧向误差较大时也会进行侧向修正。

## 节点说明

### astar_planner_node.py

**订阅**：`/odom_world`、`/kfs_grid_data`、`/task/trigger`、`/planning/direction`
**发布**：`/planning/path`（每个路径点带 position 和 orientation.yaw）

### tracker_node.py

**订阅**：`/planning/path`、`/odom_world`、`/can_go`
**发布**：`/cmd_vel`

支持两种模式，通过 `motion_type` 参数切换：
- `omnidirectional`：X/Y/Yaw 三路独立 PID
- `unidirectional`：先转角（HOLD 等 can_go）再前进

### path_decision_node.py

**订阅**：`/odom_world`、`/kfs_grid_data`、`/planning/path`
**发布**：`/can_go`（Bool）

到达 R2 KFS 格子前发送 `can_go=False` 让 tracker 在 HOLD 状态暂停。

### odom_simulator.py

接收 `/cmd_vel` 积分出 `/odom_world`。全向模式下直接用 `linear.x/linear.y`；单向模式下把 `linear.x` 当作前向速度按当前 yaw 投影到全局坐标系。

## 配置参数

所有可调参数集中在 `config/para.yaml`：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `tracker.motion_type` | `'omnidirectional'` | `'omnidirectional'` 或 `'unidirectional'` |
| `tracker.control_frequency` | `50.0` | 控制频率 (Hz) |
| `tracker.arrive_threshold` | `0.12` | 到达阈值 (m) |
| `tracker.angle_threshold` | `0.08` | 角度对齐阈值 (rad) |
| `tracker.kp_x/kp_y/kp_yaw` | `3.0/3.0/5.0` | 全向模式 PID-P |
| `tracker.kp_angle/kp_dist` | `4.0/2.5` | 单向模式 PID-P |
| `tracker.max_vel_forward` | `0.5` | 最大前向速度 (m/s) |
| `tracker.max_angular` | `2.0` | 最大角速度 (rad/s) |

切换运动模式只需修改一行：

```yaml
tracker:
  motion_type: 'unidirectional'   # 改为 'omnidirectional' 即为全向模式
```

## 使用方法

### 编译

```bash
cd ~/ws_ros2
colcon build --packages-select python_path_searching
source install/setup.bash
```

### 运行

```bash
ros2 launch python_path_searching path_tracking.launch.py
```
