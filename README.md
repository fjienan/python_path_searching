# Python Path Searching

基于 ROS2 的森林搜索路径规划系统，使用 DFS 算法进行全局路径规划，逐段 PID 控制实现路径跟踪。

## 项目结构

```
python_path_searching/
├── core/                    # 核心算法和工具类
│   ├── dfs.py              # DFS 路径规划算法
│   ├── grid_utils.py       # 网格坐标转换工具
│   ├── pid_controller.py   # PID 控制器
│   ├── step.py             # 步进模型
│   └── transform_utils.py   # 坐标变换工具（四元数/欧拉角）
│
├── app/                     # ROS2 节点（主程序）
│   ├── dfs_planner_node.py      # DFS 路径规划节点
│   ├── tracker_node.py          # 轨迹跟踪节点（统一节点）
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
KFS 管理器发布 /kfs_grid_data
       ↓
dfs_planner_node → /planning/path (关键点 + 目标姿态 yaw)
       ↓
tracker_node → /cmd_vel (逐段 PID 控制)
       ↓
odom_simulator ← /cmd_vel 积分 → /odom_world (仅仿真模式)
        ↓ 反馈
    /can_go → 回到 tracker_node
```

### 状态机流程

轨迹跟踪节点状态机：
  单向/全向模式：HOLD → MOVE → ADJ(if precision needed) → HOLD(if can_go required) → MOVE...
           - HOLD: 停在关键点，等 can_go=True 才继续
           - MOVE: 角度大于阈值时先修正，直线前进到目标点，实时校正偏离直线误差、角度误差（阈值较大）
           - ADJ: 如需精细化对齐才进入（阈值较小），如果在某点不需要抓取kfs操作则不会进入 ADJ，以加快运行速度。

  全向模式获得的路径无需旋转对齐，直接控制 x/y 轴速度即可；单向模式需要先旋转对齐，再控制前进速度，侧向误差较大时也会进行侧向修正。

## 节点说明

### dfs_planner_node.py

**订阅**：`/odom_world`、`/kfs_grid_data`
**发布**：`/planning/path`（每个路径点带 position 和 orientation.yaw）

### tracker_node.py

**订阅**：`/planning/path`、`/odom_world`、`/can_go`、`/current_state`
**发布**：`/cmd_vel`

支持两种模式，通过 `motion_type` 参数切换：
- `omnidirectional`：X/Y/Yaw 三路独立 PID
- `unidirectional`：先转角（HOLD 等 can_go）再前进

### odom_simulator.py

接收 `/cmd_vel` 积分出 `/odom_world`。全向模式下直接用 `linear.x/linear.y`；单向模式下把 `linear.x` 当作前向速度按当前 yaw 投影到全局坐标系。

## 配置参数

所有可调参数集中在 `config/para.yaml`：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `motion_type` | `'unidirectional'` | `'omnidirectional'` 或 `'unidirectional'` |
| `control_frequency` | `100.0` | 控制频率 (Hz) |
| `arrive_threshold` | `0.12` | 到达阈值 (m) |
| `angle_threshold` | `0.05` | 角度对齐阈值 (rad) |
| `max_vel` | `0.8` | 最大前向速度 (m/s) |
| `max_angular` | `3.0` | 最大角速度 (rad/s) |

切换运动模式只需修改一行：

```yaml
motion_type: 'unidirectional'   # 改为 'omnidirectional' 即为全向模式
```

## 使用方法

### 本地调试

para.yaml:
    odom_world:         '/odom_world'
    cmd_vel:            '/cmd_vel'

path_tracking.launch.py 的最后几行取消注释:
        Node(
            package=pkg_name,
            executable='odom_simulator',
            name='odom_simulator',
            output='screen',
            parameters=[params_file],
        ),

重新编译：
```bash
colcon build
source install/setup.bash
```

先启动Kfs管理器：
```bash
ros2 launch triple_map_manager kfs_direct.launch.py
```

再启动路径规划：
```bash
ros2 launch python_path_searching path_tracking.launch.py
```

---

### 上机调试

para.yaml:
    odom_world:           '/odin1/odometry_highfreq'
    cmd_vel:              '/t0x0101_'

path_tracking.launch.py 的最后几行注释掉:
        # Node(
        #     package=pkg_name,
        #     executable='odom_simulator',
        #     name='odom_simulator',
        #     output='screen',
        #     parameters=[params_file],
        # ),

重新编译：
```bash
colcon build
source install/setup.bash
```

先启动重定位 relocation
再启动kfs管理器 kfs
再电机c板reset 上电 reset 再开control
最后开路径规划 path

---

### 常用 ros2 topic 命令

```bash
ros2 topic list                                              # 列出所有话题
ros2 topic echo /current_state                               # 查看升降状态
ros2 topic echo /t0x0101_                                    # 查看速度状态
ros2 topic pub /can_go std_msgs/msg/Bool "{data: true}" -1   # 发一次
ros2 topic pub /can_go std_msgs/msg/Bool "{data: true}" -r n # 以nHz频率持续发布
ros2 topic pub /t0x0101_ std_msgs/msg/Float32MultiArray "{data: [x, y, z]}" -r 100  # 发布速度
ros2 topic pub /t0x0101_ std_msgs/msg/Float32MultiArray "{data: [0.3, 0.0, 0.0]}" -r 100  # x方向0.3m/s
```
