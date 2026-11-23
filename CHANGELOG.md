# 版本变更记录

## v3.0.0 - 按 2023 电赛 G 题《空地协同智能消防》重构

**这是一次破坏性结构调整**：旧赛题（D 题 / 凌霄飞控 / 自定义串口 V2）整套工程退出构建，但**没有从 git 历史里删除**。

### 归档（移入 `archive/legacy_2023_uav/`，不再参与 `colcon build`）

- `flight_controller_bridge`（自定义串口 V2 协议 + 凌霄飞控）
- `camera_serial_bridge`（旧树莓派 USB 摄像头串口桥）
- `car_serial_bridge`（旧蓝牙串口小车桥）
- `gcs_serial_bridge`（旧地面站蓝牙链路）
- `waypoint_manager`（旧 D 题任务层）
- `csm` + `ros2_laser_scan_matcher`（已确认死代码，会与 Cartographer 抢 `odom`）
- `zips/` 下 5 个游离压缩包
- `uav_bringup_launch/`：旧的 `full_system.launch.py`、`.orig` 与配套测试

### 重新分组（**包名一律不变**，包内引用零改动）

```
src/
  interfaces/    uav_interfaces
  localization/  LSLIDAR_X_ROS2-20240228 / cartographer_config / map_coordinate_provider
  perception/    k230_bridge        ← 新增骨架
  control/       position_pid_controller
  flight/        px4_offboard
  mission/       competition_g      ← 新增骨架
  comm/          car_link           ← 新增骨架
  recorder/      flight_data_recorder
  bringup/       uav_bringup
```

### 新增

- `src/perception/k230_bridge`（v0.1.0 骨架）：K230 庐山派下视相机桥。红点检测与视觉导航**全部由队友在 K230 上实现**，本包只接结论与状态标志位，树莓派不处理图像。
- `src/mission/competition_g`（v0.1.0 骨架）：G 题任务层。含蛇形航线规划结论（y = 4/12/20/28/36 dm，航程 272 dm）与任务状态机设计。
- `src/comm/car_link`（v0.1.0 骨架）：树莓派开热点作 AP + TCP Server，ESP32-S3 小车作 Client；按赛题要求 1 Hz 上报场地坐标。
- `src/bringup/uav_bringup/launch/competition_g.launch.py`：只拉起真实存在且已实测的节点（雷达 → Cartographer → 坐标 → PID → 记录），飞控/K230/任务/小车四处留注释占位。
- `docs/赛题G_空地协同消防_解读与差距分析.md`：题目逐条解读、评分拆解、航线与时间预算、四个现场风险（安全网对雷达是否透明 / SLAM↔场地坐标标定 / 60° 遮光罩可见性 / 自由落体投放）、新旧资产对照、分工与待确认清单。
- `src/README.md`：新目录结构、全链路图、三条铁律。

### 改动

- `uav_bringup` 升到 v2.0.0：`package.xml` 去掉全部已归档包依赖；`CMakeLists.txt` 移除指向旧 launch 的 pytest。
- `VERSION` → `v3.0.0`；`VERSIONS.md` 增加 `v3.x` 版本线与留痕。

### 保留（一行未改）

`lslidar_driver`、`lslidar_msgs`、`cartographer_config`、`map_coordinate_provider`、`position_pid_controller`、`flight_data_recorder`、`uav_interfaces`、`px4_offboard`。

### 为什么改传输层

实测：uXRCE-DDS 在 115200 下飞控固定上行约 **35 KB/s**，而线路只有 11.5 KB/s，
导致 `vehicle_attitude` 实测 1.2 Hz（应 100+）、`vehicle_local_position` 0.79 Hz（应 50）。
改用 MAVLink 按需轮询后：**`LOCAL_POSITION_NED` 稳定 24.6 Hz，总流量 6.5 KB/s**。
细节见赛题文档第六节。

## v2.0.0 - PX4 迁移脚手架

**旧工程零改动**：`src/` 下原有 14 个包一行未改，只**新增**了一个包和一个脚本。这一版没有任何可执行节点，也没有接进 `full_system.launch.py`，所以对现有系统没有任何运行影响。

- 新增 `src/px4_offboard/`（v2.0.0）：ENU/FLU ↔ NED/FRD 坐标换算库、PX4 专用 QoS 助手（`BEST_EFFORT` + `TRANSIENT_LOCAL`）、以及两者交叉校验的 12 组 gtest。
- ★ 姿态换算采用 **两次显式 Hamilton 四元数积** `NED_ENU_Q ⊗ q ⊗ AIRCRAFT_BASELINK_Q`（与 `px4_ros_com` 逐字一致），**刻意不用闭式**——本项目验证期手工推导闭式曾连续推错三次。
- ★ 新增 `BodyVectorWorldConsistency` 测试：2 万组随机位姿上验证"姿态换算法则"与"机体矢量换算法则"互相自洽，两条法则若非同源会立刻失败。
- 新增 `tools/setup_px4_msgs.sh`：按分支拉取 `px4_msgs`（默认 `release/1.17`，校验 sha `86d8239e…`）。**`px4_msgs` 不入库**，`.gitignore` 已排除 `src/px4_msgs/`。
- 无本地 C++ 编译器，故把 `frame_transforms.cpp` 逐行转写为临时 Python 镜像跑同一套断言：**22 项全部 PASS，最大误差 7.6e-15**（gtest 容差 1e-9）。C++ 编译层面的验证仍需在树莓派上 `colcon test`。
- 本版同样**只增加文件，不修改旧工程**，`git checkout v1.2.0` 可回到"还没碰 PX4"的状态。

## v1.2.0 - 迁移前冻结基线

**本版零代码改动**：`src/` 下没有任何文件被修改，改动全部在文档与工具脚本。目的是留下一个"还没碰 PX4"的可回退点。

- 新增 `docs/PX4伴飞工程计划书.md`（19 节，决策与施工计划）。
- 新增 `docs/PX4树莓派伴飞实战路线.md`（参数/命令/硬件坑查询手册）。
- 新增 `docs/M1_操作手册_派能读飞控.md`（照着敲的 M1 逐步手册）。
- 新增 `docs/PX4学习记录总结_至2026-10-06.md`（早期学习记录并入仓库）。
- 新增 `tools/verify_frames.py`：ENU/FLU → NED/FRD 坐标换算的随机位姿交叉校验脚本。
- 对 `docs/px4_migration_analysis_flight_controller_bridge.md`（仓库原有旧分析）做勘误：纠正 `/fmu/in/vehicle_odometry` 这个不存在的话题、心跳频率口径、串级 PID 的定性，正文前加入更正说明。
- 逐字核实 PX4 v1.17.0 源码后回填：DDS 话题白名单、`EKF2_HGT_REF`、commander 安全参数（`COM_RC_OVERRIDE` 必须改为 3）、`mc_pos_control` / land detector 参数。
- `VERSIONS.md` 增加 `v1.x` / `v2.x` 版本线划分与留痕规则。

## v1.1 - 2026-07-29

- 增加 D 题任务管理流程：起飞悬停、向左横移、向前追逐和视觉辅助阶段。
- PID 支持固定目标控制及视觉相对位移控制。
- 增加全系统 bringup 与开机自启入口。
- 完成树莓派、雷达、Cartographer、PID、串口和飞控联合测试。
- 偏航 PID 初步校准为较温和的 P 控制，保留后续实飞微调空间。
- 将测试报告与 rosbag 统一整理到 `ros2_ws/logs/`。

## v1.0

- 建立雷达驱动、二维建图、地图坐标、航点、PID 和飞控串口控制基础链路。
