# uav_bringup v1.4

本包统一启动无人机树莓派端完整系统：

- 树莓派—飞控串口桥；
- N10 雷达驱动；
- 静态 TF、laser scan matcher、Cartographer 和占据栅格；
- `map -> base_link` 当前坐标发布；
- 地图目标/视觉相对位移双模式 PID；
- D题陆空协同比赛任务状态机；
- 比赛阶段到飞控V2指令的转换节点；
- 摄像头串口桥和小车串口桥；
- 只读地面站状态桥；
- 可选 RViz。

摄像头桥接收视觉数据，小车桥接收统一任务状态；地面站桥只发送 UAV_STATUS 并接收可选心跳，不拥有飞行控制权限。

## 启动顺序

| 启动后时间 | 节点 |
|---:|---|
| 0 s | 飞控串口桥、雷达驱动 |
| 3 s | 静态 TF、scan matcher、Cartographer、占据栅格、可选 RViz |
| 6 s | 地图坐标发布 |
| 7 s | 双模式 PID、摄像头桥、小车桥、地面站桥 |
| 8 s | V2比赛飞控指令节点（先建立STANDBY session） |
| 9 s | D题比赛任务状态机 |

任一关键节点退出时，总 launch 会关闭整套系统；systemd 随后完整重启，避免半套
控制链继续运行。
地面站桥不是关键控制节点；设备缺失、串口断开或节点自身退出都不能关闭飞行系统。

## 上电默认行为

默认：

```text
competition_auto_start_for_test=false
```

因此开机只启动并等待所有节点，任务状态保持 `WAIT_START`，先等待小车发布task1或task2，再等待START，不会自动发送移动航点，
也不会解锁飞控。正式比赛由小车无线通信节点先发布任务编号（1=抛投，2=动态起降），再发布START：

当前飞控与树莓派均固定使用V2。等待期间会发送零运动量的V2 STANDBY用于建立
flight session，但不会发送TAKEOFF；V1/V2不会自动猜测、切换或回退。


```bash
ros2 topic pub --once /competition/task_mode std_msgs/msg/UInt8 "{data: 1}"

ros2 topic pub --once /competition/start std_msgs/msg/Bool "{data: true}"
```

只有拆桨台架联调才允许：

```bash
ros2 launch uav_bringup full_system.launch.py \
  competition_auto_start_for_test:=true
```

## 手动启动

```bash
source /opt/ros/jazzy/setup.bash
source /home/ljz/ros2_ws/install/setup.bash
ros2 launch uav_bringup full_system.launch.py
```

人工调试 RViz：

```bash
ros2 launch uav_bringup full_system.launch.py use_rviz:=true
```

可覆盖的主要参数：

- `flight_serial_device`，默认 `/dev/ttyUSB0`；
- `flight_protocol_version`，当前默认且必须与已烧录飞控一致为 `2`；
- `lidar_serial_device`，默认 `/dev/wheeltec_laser`；
- `configuration_basename`，默认 `n10_2d.lua`；
- `competition_auto_start_for_test`，正式运行必须为 false。

## 自动飞行数据记录

总启动包默认以轻量模式启动 `flight_data_recorder`，但在小车START以前不会创建飞行包，也不会向任何控制话题发布数据。每轮任务保存在：

```text
/home/ljz/ros2_ws/logs/flights/YYYYMMDD_HHMMSS_mmm_taskN/
```

每轮目录包含：

- `events.jsonl`：按时间排列的坐标、PID、视觉、小车、任务阶段和飞控V2数据，文本可直接分析；
- `bag/`：同一轮ROS 2话题的MCAP录包，可使用 `ros2 bag play` 回放；
- `manifest.json`：任务、协议和坐标约定；
- `summary.json`：持续时间、经过阶段、飞控session、消息数量和是否安全关闭；
- `rosbag.log`：底层录包程序日志。

任务完成、ERROR或ABORTED、返回WAIT_START或整套系统关闭时会安全结束本轮记录。记录器不是关键控制节点，记录失败不会关闭飞行系统。轻量模式默认不记录雷达原始 `/scan`，以降低CPU和磁盘I/O；只有定位专项排查时才临时设置 `record_lidar_scan:=true`。

## 开机服务

源文件和服务模板修改后先构建：

```bash
cd /home/ljz/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select uav_bringup
source /home/ljz/ros2_ws/install/setup.bash
```

如需把最新服务模板同步到 `/etc/systemd/system`，执行：

```bash
/home/ljz/ros2_ws/install/uav_bringup/lib/uav_bringup/install_boot_service.sh
```

查看运行状态：

```bash
systemctl status uav-navigation.service
journalctl -u uav-navigation.service -f
```

服务最多等待雷达设备 30 秒。飞控串口暂时不存在时，桥接节点保持安全零输出并周期
重连。服务模板同样将 `UAV_COMPETITION_AUTO_START_FOR_TEST` 设为 false。

## 比赛前必须完成的现场确认

- 拆桨确认右移方向；当前实飞标定参数 `-0.375 m`，因为机体左侧为正、右侧为负；
- 确认相机 `base_link` 数据为 x 前正、y 左正、单位米；
- 确认视觉模式四个移动方向与 XCOM 速度一致，yaw_rate 始终为 0；
- 确认视觉消息停止 500 ms 后飞控收到零速度；
- 解决并实测飞控高度字段 `C` 长时间不更新的问题；
- 保留遥控接管、飞控命令看门狗、failsafe 和电机急停；
- 无额外TTL时，可用V2状态回显与XCOM实际非零速度作为临时闭环证据，但这不能证明PD7的0x41物理帧已经被独立抓取；
- 首飞应系留并使用保守速度，不能直接按完整赛道距离自由飞行。
