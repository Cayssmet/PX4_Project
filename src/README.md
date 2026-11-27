# src/ 目录结构（v3.0.0 起）

> 2023 电赛 G 题《空地协同智能消防》版本。旧赛题（凌霄飞控 + 自定义串口 V2）的包
> 已整体移入 `../archive/legacy_2023_uav/`，`colcon build` 不会再碰它们。

## 一句话看全局

```
雷达 /scan ──► Cartographer ──► TF(map/odom/base_link) ──► map_coordinate_provider
                                                                    │
                                                                    ▼
   K230 视觉 ──► k230_bridge ──►┌──────────────┐          /map/current_pose
   （红点+视觉导航）            │ competition_g │──► /mission/target_waypoint
                                │  任务状态机   │                    │
                                └──────┬───────┘                    ▼
                                       │                    position_pid_controller
                                       │                            │
                                       ▼                            ▼ /pid/cmd_vel
                                  pos_link ◄──TCP──> 小车    px4_offboard（飞控适配）
                                                                    │
                                                                    ▼ MAVLink
                                                                 PX4 飞控
```

## 分层与包

| 层 | 目录 | 包 | 职责 |
|---|---|---|---|
| 接口 | `interfaces/` | `uav_interfaces` | 全系统消息/服务定义 |
| 定位 | `localization/` | `LSLIDAR_X_ROS2-20240228`（包名 `lslidar_driver`） | 镭神 N10 驱动，发 `/scan` |
| 定位 | `localization/` | `cartographer_config` | Cartographer 配置（`n10_2d.lua`） |
| 定位 | `localization/` | `map_coordinate_provider` | TF → `/map/current_pose` |
| 感知 | `perception/` | `k230_bridge` | **待写**：K230 串口桥（红点结果 + 视觉导航标志位） |
| 控制 | `control/` | `position_pid_controller` | 位置环，出 `/pid/cmd_vel`（map 系速度） |
| 飞控 | `flight/` | `px4_offboard` | 飞控适配：`mavlink_codec`（零依赖编解码）+ `frames`（坐标换算）+ `px4_node`（**只读**，点名拉状态 → `/flight/status`） |
| 任务 | `mission/` | `competition_g` | **待写**：蛇形航线 + 任务状态机 |
| 通信 | `comm/` | `pos_link` | **已跑通**：热点 + TCP（5000 端口、12 字节定长帧），**树莓派 → ESP32-S3** 坐标下发（订阅 `/pos_link/target`，按 1 Hz 推）。**这是唯一一条车链路**，以后加字段/加消息类型都在本包上改 |
| 记录 | `recorder/` | `flight_data_recorder` | 飞行数据落盘 |
| 启动 | `bringup/` | `uav_bringup` | launch / systemd / udev |

## 三条铁律

1. **`odom -> base_link` 只有 `cartographer_node` 一个发布者。** 任何节点都不得再发这条 TF，
   否则 TF 树会打架、PID 会收到两个来源的位置。
2. **位置环在树莓派，飞控只做执行器。** 下发的是速度设定点（`/pid/cmd_vel`），
   不是位置设定点 —— 因为光流只给速度，EKF2 没有水平位置观测。
3. **坐标转换只允许出现在 `px4_offboard` 一处。** 地图系 ENU/FLU ↔ 飞控 NED/FRD 的换算
   错一次就是炸机，全工程只留一个实现、一套测试。

## 已归档（不再参与构建）

`archive/legacy_2023_uav/`：`flight_controller_bridge`（自定义串口 V2）、`camera_serial_bridge`、
`car_serial_bridge`、`gcs_serial_bridge`、`waypoint_manager`（旧任务层）、`csm`、
`ros2_laser_scan_matcher`（已确认是死代码，会与 Cartographer 抢 `odom`）、`zips/`（5 个游离压缩包）。

要取回旧代码：`git checkout v2.0.0 -- src/` 或直接看 `archive/` 目录。
