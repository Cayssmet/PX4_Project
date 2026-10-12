# 版本变更记录

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
