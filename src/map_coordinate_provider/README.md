# map_coordinate_provider 0.2.0（完整位姿版）

这个功能包专门为新版 `position_pid_controller` 编写。它从 ROS 2 TF 中读取
`map -> base_link`，在同一条消息里同时发布当前位置的 x、y 和姿态四元数。新版 PID
从四元数中提取 yaw，因此 x、y、yaw 可以同时参与控制。

本版已经完全移除旧设计中的：

- `pid_axis: x/y/yaw` 单轴选择；
- `/pid/measurement` 单数据话题；
- `Float64` 单变量 PID 接口。

## 1. 为什么 PoseStamped 能同时包含 x、y、yaw

输出 `/map/current_pose` 的类型是 `geometry_msgs/msg/PoseStamped`：

```text
pose.position.x       当前 x
pose.position.y       当前 y
pose.orientation      完整四元数，新 PID 从这里计算 yaw
header.stamp          这一帧 TF 的时间戳
header.frame_id       map
```

注意：`orientation.z` 不是 yaw 本身。姿态必须保持为完整四元数，新 PID 会正确换算并
处理 `-pi/pi` 边界。

## 2. 与新版 PID 完全对应的接口

| 方向 | 话题 | 消息类型 | 用途 |
|---|---|---|---|
| 发布 | `/map/current_pose` | `geometry_msgs/msg/PoseStamped` | 同时提供 x、y 和完整姿态 |
| 发布 | `/map/current_pose_valid` | `std_msgs/msg/Bool` | 告诉 PID 当前位姿是否可靠 |

对应的 `position_pid_controller` 正好订阅这两个话题。位置包不再直接向任何 PID 单轴
输入发送数据。

## 3. 数据流

```text
雷达 /scan
    ↓
SLAM / 定位节点
    ↓  发布 TF：map -> odom -> base_link
map_coordinate_provider
    ├── /map/current_pose        完整 x、y、姿态、时间戳
    └── /map/current_pose_valid  true/false
                    ↓
position_pid_controller
    └── /pid/cmd_vel             vx、vy、yaw_rate
```

RViz 只是显示工具，不负责生成当前位置。必须确保雷达驱动和 SLAM/定位节点已经实际
发布 `map -> base_link` TF。

## 4. 替换树莓派上的旧包

不要直接删除旧包。先停止旧节点，然后将旧包移到工作空间外备份：

```bash
ls -ld /home/ljz/ros2_ws/src/map_coordinate_provider
ls -ld /home/ljz/map_coordinate_provider_old_single_axis_backup
mv /home/ljz/ros2_ws/src/map_coordinate_provider \
   /home/ljz/map_coordinate_provider_old_single_axis_backup
```

如果第二条命令显示备份目录已存在，请换一个新备份名，不要覆盖。

将本次压缩包解压到工作空间的 `src`：

```bash
unzip ~/Downloads/map_coordinate_provider_v2.zip -d /home/ljz/ros2_ws/src/
```

正确目录必须是：

```text
/home/ljz/ros2_ws/src/map_coordinate_provider/package.xml
```

不能出现下面这种重复嵌套：

```text
/home/ljz/ros2_ws/src/map_coordinate_provider/map_coordinate_provider/package.xml
```

## 5. 修改参数

编辑：

```text
/home/ljz/ros2_ws/src/map_coordinate_provider/config/coordinate_params.yaml
```

默认配置：

```yaml
map_frame: "map"
base_frame: "base_link"
publish_rate_hz: 20.0
tf_stale_timeout_ms: 500.0
print_coordinates: true
```

如果无人机机体 frame 不是 `base_link`，必须把 `base_frame` 改成 TF 中的真实名称。
位置包的 `map_frame` 还必须与 PID 参数文件中的 `map_frame` 完全相同。

## 6. 构建新版位置包和 PID 包

```bash
cd /home/ljz/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select \
  map_coordinate_provider position_pid_controller
source /home/ljz/ros2_ws/install/setup.bash
```

如果之前构建过同名旧包，并且出现奇怪的旧接口残留，可只清理这两个包对应的构建
产物后再编译；不要删除整个工作空间中的其他功能包。

## 7. 正确启动顺序

先启动雷达和 SLAM/定位，再确认 TF：

```bash
source /home/ljz/ros2_ws/install/setup.bash
ros2 run tf2_ros tf2_echo map base_link
```

终端持续出现 Translation 和 Rotation 后，启动新版位置包：

```bash
ros2 launch map_coordinate_provider coordinate.launch.py
```

确认两个输出：

```bash
ros2 topic echo /map/current_pose
ros2 topic echo /map/current_pose_valid
```

正常情况下终端每秒打印一次：

```text
当前地图位姿：x=1.234 m, y=0.567 m, yaw=0.120 rad（frame=map）
```

最后启动三轴 PID：

```bash
ros2 launch position_pid_controller position_pid.launch.py
```

检查连接关系：

```bash
ros2 topic info /map/current_pose -v
ros2 topic info /map/current_pose_valid -v
ros2 topic echo /pid/control_valid
ros2 topic echo /pid/cmd_vel
```

`/map/current_pose` 应有一个位置包发布者和一个 PID 订阅者。

## 8. 坐标无效时的行为

以下情况会持续发布 `/map/current_pose_valid=false`，并停止发布新位姿：

- 找不到 `map -> base_link` TF；
- TF 时间戳为空、来自未来或已经过期；
- x、y、z 含有 NaN/Inf；
- 四元数为全零或含有 NaN/Inf。

新版 PID 收到 false 或发现位姿超时后，会持续输出零速度和
`/pid/control_valid=false`，不会继续使用上一条非零速度。

## 9. 单元测试

```bash
cd /home/ljz/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon test --packages-select map_coordinate_provider
colcon test-result --verbose
```

测试会检查有限坐标、四元数归一化、yaw 换算以及非法四元数拒绝逻辑。

## 10. 常见问题

### 一直显示 valid=false

先运行：

```bash
ros2 run tf2_ros tf2_echo map base_link
```

如果找不到变换，检查真实 frame 名称，常见机体 frame 还有 `base_footprint`。如果 TF
存在但更新很慢，可适当增大 `tf_stale_timeout_ms`，同时也要对应检查 PID 的
`pose_timeout_ms`。

### 还能看到 /pid/measurement

新版包不会创建它。这通常表示旧节点仍在运行，或者终端 source 了旧的工作空间。
关闭旧节点，重新 source `/home/ljz/ros2_ws/install/setup.bash`，再用下面命令查看发布者：

```bash
ros2 topic info /pid/measurement -v
```

### RViz 中能看到地图，但没有 current_pose

能显示地图不等于 `map -> base_link` TF 一定存在。仍应使用 `tf2_echo` 检查 TF 链，并
确认 `base_frame` 与实际无人机机体 frame 相同。
