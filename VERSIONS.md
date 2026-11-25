# 无人机导航工作区版本清单

当前工作区发布版本：**v3.2.0**（2023 电赛 G 题《空地协同智能消防》：ESP32-S3 坐标链路真机跑通）

> 想回到"还没碰 PX4"的状态：`git checkout v1.2.0`。想回到"还没有 v2 迁移代码"的状态：`git checkout v1.1`。
> 想回到"还没按 G 题重构"的状态：`git checkout v2.0.0`。

## 自研功能包

| 功能包 | 版本 | 说明 |
| --- | --- | --- |
| `uav_interfaces` | v1.0 | ROS 2 消息与接口定义 |
| `cartographer_config` | v1.0 | N10 雷达 Cartographer 配置 |
| `map_coordinate_provider` | v1.0 | 地图坐标与姿态输出 |
| `position_pid_controller` | v1.1 | 固定目标及视觉相对位移控制、偏航参数校准 |
| `uav_bringup` | v2.0.0 | 全系统 launch 与开机自启入口；v2.0.0 重写为 `competition_g.launch.py`，只拉起真实存在的节点 |
| `px4_offboard` | v3.1.0 | 飞控适配层。**v3.1.0 改为纯 Python（ament_python）**：`mavlink_codec`（零依赖 MAVLink v1 编解码）+ `frames`（ENU/FLU↔NED/FRD 换算）+ `px4_node`（**只读**：点名拉状态 → `/flight/status`）。取消了 uXRCE-DDS 与 `px4_msgs` 依赖；C++ 骨架移入 `archive/legacy_v2_cpp_frame_transforms/` |
| `k230_bridge` | v0.1.0 | **新增（骨架）**：K230 庐山派下视相机串口桥，只接红点结论与视觉导航标志位 |
| `competition_g` | v0.1.0 | **新增（骨架）**：G 题任务层，蛇形航线 + 任务状态机 |
| `car_link` | v0.1.0 | **新增（骨架）**：与 ESP32-S3 消防车的 WiFi TCP 链路 |
| `pos_link` | v1.0.0 | **新增（已跑通）**：树莓派开热点 + TCP 定长帧，ESP32-S3 以 1 Hz 发 12 字节坐标帧 → `/esp32/position`。真机实测 1.000 Hz 无丢帧；含 `firmware/esp32_pos_tx`（ESP-IDF v5.5.3）与协议文档 `D_UAV_ESP32_POS_LINK_v1.0` |

> 已移入 `archive/legacy_2023_uav/` 的旧包：`flight_controller_bridge`、`camera_serial_bridge`、
> `car_serial_bridge`、`gcs_serial_bridge`、`waypoint_manager`、`csm`、`ros2_laser_scan_matcher`。
> 它们**不参与 `colcon build`**，但 git 历史完整保留。

## 第三方功能包

第三方包保留上游 `package.xml` 版本，避免伪造或覆盖依赖版本；下表的“集成版本”表示它们已纳入当前工作区发布版本。

| 功能包 | 上游版本 | 集成版本 |
| --- | --- | --- |
| `lslidar_driver` | 1.2.0 | v1.1 |
| `lslidar_msgs` | 1.2.0 | v1.1 |
| `csm` | 0.0.0（上游清单） | v1.1 |
| `ros2_laser_scan_matcher` | 0.0.0（上游清单） | v1.1 |

## 版本规则

- `v1.0`：基础导航、建图、PID 和串口控制链路可运行版本。
- `v1.1`：针对 D 题流程及当前 PID 校准结果的版本。
- `v1.2.0`：**迁移前冻结基线**。只增加文档（PX4 伴飞工程计划书、实战路线手册、M1 操作手册、旧桥迁移分析对账）与源码核实结论，`src/` **一行代码都没有改动**。任何时候想回到"还没碰 PX4"的状态，`git checkout v1.2.0` 即可。
- `v2.0.0`：**PX4 迁移脚手架**。新增 `px4_offboard` 包（`frame_transforms` 坐标换算库 + `qos_profiles` + 12 组 gtest）与 `tools/setup_px4_msgs.sh`；`.gitignore` 排除 `src/px4_msgs/`。**没有节点、没有接进 `full_system.launch.py`，旧工程 `src/` 一行未改**，`flight_controller_bridge` 原样保留可回退。
- `v3.0.0`：**按 2023 电赛 G 题重构**。旧赛题（D 题 / 凌霄飞控 / 自定义串口 V2）的 7 个包与 5 个游离 zip 整体移入 `archive/legacy_2023_uav/`，不再参与构建；`src/` 改为按层分组（`interfaces` / `localization` / `perception` / `control` / `flight` / `mission` / `comm` / `recorder` / `bringup`）；新增 3 个骨架包 `k230_bridge` / `competition_g` / `car_link`；`uav_bringup` 重写为 `competition_g.launch.py`；新增赛题解读文档。**所有保留包的包名不变**，因此包内引用零改动。
- `v3.0.1`：通信协议定稿（`docs/通信协议设计_v1.md`）：三条自定义链路（Pi↔K230 串口 / Pi↔小车 TCP / Pi↔飞控 MAVLink）共用一个 `AA 55` 帧封装；新增 `CMD_SET_HEIGHT`、`HEIGHT_FIXED`/`OVER_RATE` 降级位、两个联调工具（`k230_rate_probe.py`、`car_link_sim.py`），`proto_selftest.py` 扩到 33 项。
- `v3.1.0`：**飞控链路从 uXRCE-DDS 改为 MAVLink**。实测 DDS 固定灌 27 条话题约 35 KB/s，而 CH340 线在 115200 只有 11.5 KB/s、460800 直接收不到数据；改 MAVLink 后按需点名，**6.5 KB/s 就拿到 25 Hz 位置**。`px4_offboard` 从 ament_cmake 改为 **ament_python**，新增 `mavlink_codec.py`（零依赖编解码，含 3 个与 pymavlink 逐字节一致的参考帧）、`frames.py`（换算，含 2 万组随机位姿不变式）、`px4_node.py`（**只读**：点名 32/30/245/230 → `/flight/status`）。取消 `px4_msgs` 依赖；C++ 骨架移入 `archive/legacy_v2_cpp_frame_transforms/`。
- `v3.2.0`：**ESP32-S3 坐标链路跑通**。新增 `src/comm/pos_link`（ament_python：`frame.py` 12 字节定长帧编解码 + `bridge_node.py` TCP Server → `/esp32/position`）、`firmware/esp32_pos_tx`（ESP-IDF v5.5.3，扫描锁定 BSSID、失联 5 次自动解锁、`WIFI_AUTH_WPA_PSK` 门槛）、`docs/protocols/D_UAV_ESP32_POS_LINK_v1.0.md`、`deploy/systemd/pos-link.service`、`tools/pos_link_sim.py`。真机实测 1.000 Hz、无丢帧；记录 211（authmode threshold）长坑。
- ROS 2 `package.xml` 和 CMake 使用三段式版本，例如 `1.1.0`；文档中简写为 `v1.1`。
- 后续功能修改先更新对应包版本，再同步本文件和根目录 `VERSION`。

## 版本线划分（2026-10 起）

| 版本线 | 含义 | 状态 |
| --- | --- | --- |
| `v1.x` | 凌霄飞控 + 自定义串口协议（V1/V2 线缆协议）时代 | **冻结**，只读不改，`v1.2.0` 为最终基线 |
| `v2.x` | PX4 1.17 + uXRCE-DDS + offboard 迁移线 | **冻结**，`v2.0.0` 为最终基线（uXRCE-DDS 因物理层带宽不足已放弃，改走 MAVLink） |
| `v3.x` | 2023 电赛 G 题《空地协同智能消防》+ PX4 + MAVLink | **进行中** |

`v2.x` 的编号约定：**每个里程碑（M1 链路 / M2 命令 / M3 位姿上行 / M4 offboard 最小闭环 / M5 定点悬停 / M6 任务接入）发一个次版本号**，脚手架与破坏性结构调整走主版本号。每版都要：① 在 `CHANGELOG.md` 写清改了什么；② 同步本文件；③ 打一个 git tag（`git tag -a v2.0.0 -m "..."`）并 push。

### `v2.x` 版本留痕表

| 版本 | 内容 | tag |
| --- | --- | --- |
| `v1.2.0` | 迁移前冻结基线（纯文档） | `v1.2.0` |
| `v2.0.0` | PX4 迁移脚手架：`px4_offboard` 包（坐标系换算库 + QoS 助手 + gtest 交叉校验 + `px4_msgs` 拉取脚本），**尚未接入任何运行中的链路** | `v2.0.0` |
| `v3.0.0` | **按 G 题重构**：旧赛题 7 包 + 5 zip 归档；`src/` 按层分组；新增 `k230_bridge` / `competition_g` / `car_link` 骨架；`uav_bringup` 换成 `competition_g.launch.py`；新增 `docs/赛题G_空地协同消防_解读与差距分析.md` | `v3.0.0` |
| `v3.0.1` | 通信协议定稿：三条链路共用一个 `AA 55` 帧封装；`CMD_SET_HEIGHT` + `HEIGHT_FIXED`/`OVER_RATE` 降级位；新增 `tools/k230_rate_probe.py`、`tools/car_link_sim.py`；`proto_selftest.py` 33 项全绿 | `v3.0.1` |
| `v3.1.0` | **飞控链路改 MAVLink**：`px4_offboard` 改 ament_python，新增 `mavlink_codec.py` / `frames.py` / `px4_node.py`（只读）；C++ 骨架移入 `archive/legacy_v2_cpp_frame_transforms/`；板级测试 18+16 项全绿 | `v3.1.0` |
| `v3.2.0` | **ESP32-S3 坐标链路跑通**：新增 `src/comm/pos_link` + `firmware/esp32_pos_tx`；树莓派开热点当 AP、ESP32 作 STA，12 字节定长帧 1 Hz → `/esp32/position`；真机 1.000 Hz 无丢帧 | `v3.2.0` |
