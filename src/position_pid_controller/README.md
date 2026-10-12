# position_pid_controller v1.2.0

本包是无人机平面双模式 PID，运行在 ROS 2 Jazzy：

1. 地图固定目标模式：用于起飞点、横移点和前向追逐点；
2. 视觉相对位移模式：用于追上小车后的摄像头辅助定位。

本包只产生 x/y 平移速度和偏航角速度，不直接控制高度、电机或串口。

## 模式一：地图固定目标

输入 `/pid/target_pose`，使用当前 `/map/current_pose` 计算 map 坐标下的
x、y、yaw 误差：

```text
目标 map 位姿 - 当前 map 位姿 -> x/y/yaw PID -> /pid/cmd_vel (map)
```

该模式保留原有的速度限幅、最小有效速度、进入/退出双阈值和输入超时保护。

## 模式二：视觉相对位移

当 `/competition/vision_assist_requested=true` 时，PID 不再追踪旧的固定点，而
使用摄像头发布的：

```text
/vision/relative_error
类型：geometry_msgs/msg/Vector3Stamped
header.frame_id：base_link
单位：米
vector.x：目标在机头前方为正、后方为负
vector.y：目标在机体左侧为正、右侧为负
```

这个向量表示“飞机还应该向哪个方向移动多少”，不是目标的 map 绝对坐标。
`vector.z` 当前不使用，高度仍由飞控内部控制。

视觉 PID 先计算机体系前后/左右速度，再按当前 map yaw 转成 map 速度发布。现有
`flight_controller_bridge` 随后做逆变换，因此飞控最终收到的仍是相机定义的
机体系速度，不会发生重复旋转。

视觉模式的 `angular.z` 始终严格为 `0`，不会主动改变机头方向；飞控定点模式
负责保持当前偏航。

## ROS 2 接口

输入：

| 话题 | 类型 | 作用 |
|---|---|---|
| `/map/current_pose` | `geometry_msgs/msg/PoseStamped` | 当前 map 位置与 yaw |
| `/map/current_pose_valid` | `std_msgs/msg/Bool` | 定位有效标志 |
| `/pid/target_pose` | `geometry_msgs/msg/PoseStamped` | 固定目标模式的新目标 |
| `/competition/active_task_mode` | `std_msgs/msg/UInt8` | 选择task1或task2专用地图限速 |
| `/competition/vision_assist_requested` | `std_msgs/msg/Bool` | 比赛状态机请求视觉模式 |
| `/vision/target_valid` | `std_msgs/msg/Bool` | 视觉目标有效标志 |
| `/vision/relative_error` | `geometry_msgs/msg/Vector3Stamped` | 机体系相对位移 |

输出：

| 话题 | 类型 | 作用 |
|---|---|---|
| `/pid/cmd_vel` | `geometry_msgs/msg/TwistStamped` | map frame 平面速度与 yaw_rate |
| `/pid/control_valid` | `std_msgs/msg/Bool` | 当前控制输出是否可使用 |
| `/pid/target_reached` | `std_msgs/msg/Bool` | 当前模式的平面目标是否进入保持区 |
| `/pid/visual_control_active` | `std_msgs/msg/Bool` | 是否正在使用合法视觉数据闭环 |

## 失效行为

视觉模式请求为 true 时，以下任一情况都会持续输出严格零速度，并令
`/pid/control_valid=false`、`/pid/visual_control_active=false`：

- 尚未收到相对位移；
- `target_valid=false`；
- 相对位移超过 500 ms 没有新消息；
- `frame_id` 不是 `base_link`；
- x/y 为 NaN 或无穷大；
- map 位姿失效或超时。

视觉数据丢失时不会自行改回旧固定目标。只有任务管理器明确把
`vision_assist_requested` 改为 false，才恢复固定目标模式。

## 分任务地图速度

- task1保持原值：全局0.25 m/s、右移0.19 m/s、前追0.23 m/s；
- task2使用独立值：全局0.28 m/s、右移0.23 m/s、前追0.25 m/s；
- 视觉控制仍独立限幅为0.20 m/s，不受上述地图导航速度影响。

## 参数

参数文件：

```text
/home/ljz/ros2_ws/src/position_pid_controller/config/position_pid_params.yaml
```

固定目标和视觉模式拥有不同 PID 参数及速度限制：

- `x_* / y_* / yaw_*`：地图固定目标；
- `vision_forward_*`：视觉前后方向；
- `vision_left_*`：视觉左右方向；
- `vision_min_xy_speed / vision_max_xy_speed`：视觉合成速度限幅；
- `vision_enter_tolerance / vision_exit_tolerance`：视觉对准迟滞区；
- `vision_timeout_ms`：视觉三类输入刷新超时。

当前参数是保守初值，不是实飞完成标定的最终值。首次视觉联调应拆桨，依次确认：

1. 相机 x 正时，XCOM 收到前进速度；
2. 相机 x 负时，收到后退速度；
3. 相机 y 正时，收到向左速度；
4. 相机 y 负时，收到向右速度；
5. 四种情况下 yaw_rate 均为 0；
6. `header.stamp` 可以留空；停止相对位移消息超过 500 ms 后速度变为零且控制无效。

## 构建与测试

```bash
cd /home/ljz/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select position_pid_controller
source /home/ljz/ros2_ws/install/setup.bash
ROS_DOMAIN_ID=42 colcon test --packages-select position_pid_controller
```
