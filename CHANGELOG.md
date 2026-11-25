# 版本变更记录

## v3.2.0 - ESP32-S3 坐标链路跑通：树莓派开热点，12 字节定长帧 1 Hz

树莓派用**自带 WiFi 模块开热点当 AP**，ESP32-S3 作 STA 连上来，以 1 Hz 推 12 字节
定长坐标帧，树莓派侧发布成 `/esp32/position`。真机实测 **1.000 Hz、无丢帧**，
桥接进程重启后 ESP32 自动重连。

这条链路解决的是「树莓派怎么和自家的 ESP32 无线说话」这个最底层的问题：
不开热点就得依赖手机热点或路由器，场地里没有；用 DDS 直连又过不了 WiFi 的发现关。
先用最小的 12 字节定长帧把热点、DHCP、重连这些坑趟平，再往上叠 `car_link` 的多字段协议。

### 新增

- **`src/comm/pos_link`**（ament_python）
  - `frame.py` —— 12 字节定长帧的编解码与 CRC-16/CCITT-FALSE（纯标准库，不依赖 ROS）。
  - `bridge_node.py` —— TCP Server（`0.0.0.0:5000`）→ `/esp32/position`
    （`geometry_msgs/PointStamped`）。每客户端一线程、`SO_REUSEADDR`、
    用帧头 `AA 55` 重同步、坏帧退 1 字节重找、序号跳变记账、1 s 看门狗（3 s 无帧告警）。
  - `test/test_frame.py` —— CRC 标准向量（`"123456789" == 0x29B1`）+ 两帧参考字节
    + 往返编解码 + 坏帧头/坏帧尾/坏 CRC/未知类型/长度错误 + int16 边界，**不需要 ROS**。
  - `launch/pos_link.launch.py`、`config/pos_link.yaml`、`README.md`。
- **`firmware/esp32_pos_tx`**（ESP-IDF v5.5.3，目标 `esp32s3`）
  - 扫描一次锁定 AP 的 BSSID 与信道（省掉每次开机的全信道扫描）；
    **连失 5 次自动解锁重新扫描** —— 否则热点换了信道就永远连不上。
  - `WIFI_PS_NONE`（1 Hz 的数据量下省电没有意义，但省电模式会拉长重传超时）。
  - 断线等 1 s 重连，**不在 WiFi 事件回调里死循环**。
- **`docs/protocols/D_UAV_ESP32_POS_LINK_v1.0.md`** —— 拓扑、帧格式、时序、
  异常处理、**双端对账表**、实测数据。
- **`deploy/systemd/pos-link.service`** —— 开机自启，`StartLimitIntervalSec=0`
  （否则端口冲突重启 5 次就被钉死在 failed）。
- **`tools/pos_link_sim.py`** —— 假 ESP32，按 1 Hz 发包；不 import `pos_link.frame`，
  没 `colcon build` 也能跑。

### 踩过的坑（供以后查）

- **`WIFI_REASON_NO_AP_FOUND_IN_AUTHMODE_THRESHOLD`（211）**，连续 11 次，全程没有
  一次 `init -> auth`。根因不在信号、不在信道、不在密码：NetworkManager 生成的 AP
  同时通告 WPA(v1) 和 WPA2，ESP32 把该热点归为 `WPA_PSK`（扫描结果 `auth=2`、
  `pair=5`(TKIP+CCMP)、`group=3`(TKIP)），而固件门槛是 `WPA2_PSK`。
  诊断的关键一步是**把扫描结果打印出来**，而不是继续猜。
- **`WIFI_AUTH_OPEN` 不能当"关掉门槛"用**：密码 ≥8 位时 ESP-IDF 会静默把 OPEN 门槛
  提升成 WPA2，日志里能看到 `authmode threshold changes from OPEN to WPA2`。
  真要放宽，写 `WIFI_AUTH_WPA_PSK`。
- **开热点不需要 `hostapd`**：NetworkManager 1.46 + `dnsmasq-base` 就够。
  计划里原本是 hostapd + dnsmasq + 三个自建 systemd 单元，实测发现 NM 已经管着 wlan0，
  装了 hostapd 反而有被误启动抢网卡的风险 —— 全部砍掉，换成一条 `nmcli` profile。
- **Windows 上 `ping` 不通但 SSH 通**：代理软件的 TUN 虚拟网卡会拦 ICMP，
  判断连通性要用 `Test-NetConnection <ip> -Port 22`。
- **切 AP 后树莓派没有外网**：默认路由被热点接管。这不影响本链路（ROS 2 本地通信走
  loopback），但**会丢掉 NTP**，长时间跑日志时间会漂，rosbag 时间对齐要注意。

### 参数与端口

| 项 | 值 |
|---|---|
| SSID / 密码 | `PiLink` / `raspberry123`（WPA2-PSK，2.4 GHz 信道 6） |
| 树莓派 AP | `192.168.50.1/24`，DHCP 池 `192.168.50.10–254` |
| TCP 端口 | `5000` |
| 帧 | 12 字节定长，`AA 55 / 01 / seq / x(int16) / y(int16) / CRC16 / 0D 0A` |
| 频率 | 1 Hz |

> 与 `car_link` 的 9000 端口、`AA 55` **变长**帧是两条独立链路，协议不通用，别混。

## v3.1.0 - 飞控链路改走 MAVLink：`px4_offboard` 换成 Python，只读节点可运行

uXRCE-DDS 那条路走到头了。实测飞控固定向外灌 27 条话题、约 **35 KB/s**，
而 CH340 这条线在 115200 上物理上限只有 **11.5 KB/s**，460800 更是**一条数据都收不到**
（换波特率不解决，因为发送量由固件决定、我们改不了）。
改 MAVLink 后按需点名，**6.5 KB/s 就拿到 25 Hz 位置**，115200 只用了 57%。

### 传输层的三个硬结论（都是真机实测）

- **`SET_MESSAGE_INTERVAL(244)` 在 PX4 1.17 上回 `UNSUPPORTED`**（命令似乎没进 mavlink
  接收机那个分支；源码里该分支失败只会返回 `FAILED`，不可能是 `UNSUPPORTED`）。
  而 **`REQUEST_MESSAGE(512)` 回 `ACCEPTED` 且每次都能拉出一条** ⇒ 用「定时点名」
  代替「设置流频率」，频率完全由我们自己控。
- **实测点名频率**：`LOCAL_POSITION_NED` 目标 25 Hz → 实到 **24.60 Hz**；
  `ESTIMATOR_STATUS` 2.00、`EXTENDED_SYS_STATE` 2.07、`HEARTBEAT` 2.13。
  总流量 **下行 4236 B/s + 上行 2383 B/s**。
- ★ **MAVLink 路线上没有 `OffboardControlMode`**：设定点流本身就是心跳。
  `SET_POSITION_TARGET_LOCAL_NED` 以 ≥2 Hz 持续发才算 offboard 存活，
  停发超过 `COM_OF_LOSS_T`（默认 1 s）就退出。写上行时别去找 `OffboardControlMode`。

### `px4_offboard` 从 ament_cmake 改为 ament_python

理由：这台机器**没有 C++ 工具链**（`g++`/`cmake` 全无），那 12 个 gtest 从来没跑过；
而 Python 版的两套测试是**实跑通过**的。而且一份实现比两份好 —— C++ 换算库和
Python 节点各存一份数学，迟早漂移。

- **`mavlink_codec.py`** —— 零依赖 MAVLink v1 编解码（只用标准库）。
  树莓派上没有 `pip3`、PyPI 也不一定连得上，而这里只需要十几条消息，
  手工组帧的字节已用 pymavlink 参考帧**逐字节核对**。
- **`frames.py`** —— ENU/FLU ↔ NED/FRD 换算，纯标准库、不依赖 ROS。
- **`px4_node.py`** —— **只读**节点：定时点名 32/30/245/230 → 整理成 `/flight/status`。
  **不发任何控制量**，可以在装桨、通电的飞机上放心跑。串口读写放独立线程，
  掉线自动重连。
- **`config/px4_offboard.yaml`** + **`launch/px4_offboard.launch.py`** —— 串口设备名与
  波特率都是参数；`baud` **必须等于飞控 `SER_TEL3_BAUD`**，不一致时一个字节都收不到且不报错。
- **C++ 骨架移入 `archive/legacy_v2_cpp_frame_transforms/`**（含 `frame_transforms.*`、
  `qos_profiles.hpp`、12 组 gtest、原 `CMakeLists.txt`），附 README 说明为什么归档
  以及怎么取回。**没有删任何东西**。

### 测试（本机实跑，不依赖 ROS）

```
test/test_mavlink_codec.py    18/18 通过
test/test_frames.py           16/16 通过
```

`test_mavlink_codec.py` 里有三个 **GOLDEN 参考帧**（pymavlink 生成、已被 PX4 接受，
分别是 `HEARTBEAT`、`COMMAND_LONG(244)`、`PARAM_REQUEST_READ`），组帧必须逐字节相等。
还包括：v2 帧头 10 字节、坏 CRC 丢弃后仍能继续解析、3 字节分片、垃圾前缀重同步、
`PARAM_VALUE` 整数还原（`SYS_AUTOSTART` 4001）。

`test_frames.py` 里最关键的是 `test_body_vector_world_consistency`：
把机体速度先转 FRD 再按姿态转到世界，必须等于「先按地图姿态转到世界再按世界法则转换」。
**两条法则若不是同源，这条必挂** —— 这是「前后左右整体转 90°」那类炸机的守门员。

### 顺带记下的两个协议坑

- **v2 帧头是 10 字节**（带签名再 +13）。写成 12 会让载荷整体错位 2 字节，
  表现是「所有参数都读成 0」。
- **PX4 把整数参数的位模式当 float 传**，`PARAM_VALUE` 要按 `param_type` 还原，
  否则 `SYS_AUTOSTART=4001` 读出来是 `0.0000`。

## v3.0.1 - 通信协议落实：高度下发、两个联调工具、双 CH340 的坑

协议 v1 里三个悬而未决的问题已由实测/确认关闭，并补上两个可直接跑的联调工具。

### 协议增补（`docs/通信协议设计_v1.md`）

- **新增链路 A 的 `0x03 CMD_SET_HEIGHT`（Pi → K230，10 Hz）**：K230 把像素偏差换成地面 cm
  要乘以高度，以前飞控不报高度只能用固定值，现在由 Pi 把 EKF 的 `dist_bottom`（优先）或
  `LOCAL_POSITION_NED.z` 推下去。高度失效时发 `valid=0`，K230 退回固定值并置 `HEIGHT_FIXED`。
- **`VISION_STATE.flags` 扩到 bit6**：新增 `HEIGHT_FIXED`(bit5)、`OVER_RATE`(bit6) 两个降级位。
- **K230 接入改成两阶段**：现在 GPIO UART `/dev/ttyAMA0`，之后改 CH340 USB `/dev/k230`；
  设备名必须是 launch 参数，换线不改代码。
- **新增 §9「两个 CH340 的坑」**：飞控与 K230 都是 `1a86:7523` 且都无序列号，只能靠
  `ENV{ID_PATH}` 区分；插反不报错、只会让两个软链互换。附 udev 模板、三条操作纪律与自检办法。
- Q1–Q4 状态改为「已定」，Q3（K230 能否稳 30 Hz）标注为**未实测**，给了实测判据。

### 工具（`tools/`）

- **新增 `tools/k230_rate_probe.py`**：解析 `VISION_STATE` 统计真实帧率、帧间隔中位数/p95/最大与
  首几帧解码，按「平均 ≥20 Hz 且 p95 <80 ms」给结论。顺带能识别「这个口上是 MAVLink 帧」，
  用来判断两条 CH340 有没有插反。
- **新增 `tools/car_link_sim.py`**：假消防车 TCP Client。解码全部下行帧，并**自己按收到的位置点
  独立累加航程**，与 Pi 发的 `MILEAGE` 对比 —— 直接验证赛题要求的「车上显示累计航程」两边一致。
- **`tools/proto_selftest.py` 扩到 33 项**：补 `CMD_SET_HEIGHT`、`MILEAGE`、`MISSION_STATE`
  编解码与 `HEIGHT_FIXED`/`OVER_RATE` 两个标志位，全部通过（CRC 向量仍为 `0x2E0E`）。

### 文档

- `src/perception/k230_bridge/README.md`、`src/comm/car_link/README.md` 由「格式待定」
  改为指向已定稿协议 + 帧表 + 联调命令。

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
