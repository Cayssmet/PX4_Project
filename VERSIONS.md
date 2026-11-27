# 无人机导航工作区版本清单

当前工作区发布版本：**v3.3.1**（2023 电赛 G 题《空地协同智能消防》：`pos_link` 已迁入 `~/ros2_ws`，环路实测通过）

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
| `pos_link` | v1.2.0 | **已跑通，且是全系统唯一一条车链路**：树莓派开热点 + TCP，**树莓派 → ESP32-S3 下发坐标**（订阅 `/pos_link/target`，1 Hz 推 12 字节定长帧）；ESP32 侧 `firmware/esp32_pos_rx` 收帧执行。坐标源是接口，换源不改代码；协议文档 `D_UAV_ESP32_POS_LINK_v1.0`。**v3.3.0 起原 `car_link` 已删除，以后给小车加字段/加消息类型都在本包上改** |

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
- `v3.2.1`：**pos_link 方向反转**。v3.2.0 归档时把链路写反了（ESP32 → 树莓派），按项目实际需求改为**树莓派 → ESP32-S3**：树莓派算坐标、按 1 Hz 下发，ESP32 接收后执行。协议字节不变（12 字节定长帧 + CRC-16/CCITT-FALSE + 两条参考帧照旧）；改的是传输角色与代码：`bridge_node.py` 由「监听解析」改为「订阅 `input_topic` → 定时推送」（坐标源做成接口），`frame.py` 增 `FrameReader`，固件 `esp32_pos_tx` 改为 `esp32_pos_rx`（收帧 + `on_position()` 钩子），`tools/pos_link_sim.py` 改为假 ESP32 收包，文档与时序全面更正。
- `v3.3.0`：**小车通信统一到 `pos_link`**。删除骨架包 `src/comm/car_link` 与 `tools/car_link_sim.py`（那套 9000 端口、`AA 55 + VER + TYPE + LEN` 变长帧从未实现，留着只会让队友照错文档写）；`docs/通信协议设计_v1.md` §3 按实际实现重写（12 字节定长帧、`:5000`、单向下发），原多字段想法压缩为「候选扩展字段」并在文中写明扩展规则；`uav_bringup` 的 `car_link` 依赖改 `pos_link`、launch 里的占位节点删除；同步 `src/README.md`、赛题解读、`competition_g` README。另并入 v3.2.1 之后的三个上板修复：`pos_link/package.xml` 非法 XML、`rclpy` 命名冲突、ESP32 目标固定与 ESP-IDF 产物忽略规则。
- `v3.3.1`：**`pos_link` 迁入 `~/ros2_ws` + 校验接口预留**。核对 `/home/ljz/px4_ws` 里的旧副本，除 `bridge_node.py` 少两行注释外与仓库逐字节一致；已复制到 `~/ros2_ws/src/comm/pos_link` 并 `colcon build` 通过，环路实测（新构建、5100 端口）收到 5 个好帧、0 坏帧、0 次序号跳变。代码上 `pos_link` 升 **v1.2.0**：`bridge_node.py` 新增 `_validate_target(msg, x, y)` 预留校验接口（当前一律放行），`frame_id`/场地范围/跳变检测以后只改这一个函数；**场地范围校验按决定暂不启用**。
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
| `v3.2.1` | **pos_link 方向反转**：改为树莓派 → ESP32-S3 下发坐标（`pos_link` v1.1.0，固件 `esp32_pos_rx`）；坐标源做成 `input_topic` 接口；协议字节不变 | `v3.2.1` |
| `v3.3.0` | **小车通信统一到 `pos_link`**：删除 `car_link` 包与 `car_link_sim.py`；协议文档 §3 按实际实现重写；并入三个上板修复 | `v3.3.0` |

> 历史条目里出现的 `car_link`（v3.0.0 / v3.0.1 行）只作记录 —— 该包已在 v3.3.0 删除。
