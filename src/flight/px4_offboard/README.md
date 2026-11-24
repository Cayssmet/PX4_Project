# px4_offboard —— 树莓派 ↔ PX4 的 MAVLink 桥

这个包负责一件事：**让树莓派和 PX4 飞控说话**。

它不含任何 uXRCE-DDS 代码 —— 那条路线在 v3.0.0 阶段实测过，
飞控固定向外灌 27 条话题、约 35 KB/s，而 CH340 这条线在 115200 只有
11.5 KB/s、460800 直接收不到数据，线路被撑爆。所以改走 MAVLink。

## 三个模块

| 模块 | 依赖 | 作用 |
|---|---|---|
| `mavlink_codec.py` | **纯标准库** | MAVLink v1 组帧/解析（只实现用到的十几条消息） |
| `frames.py` | **纯标准库** | ENU/FLU ↔ NED/FRD 的坐标与姿态换算 |
| `px4_node.py` | rclpy + pyserial | 节点：定时点名拉状态 → 发 `/flight/status` |

前两个不 import ROS，可以脱离一切环境单独测试 —— 这是故意的，
因为坐标换算是这类项目炸机原因第一名，必须能在任何机器上随时验。

## 为什么不用 pymavlink

树莓派上没有 `pip3`，PyPI 也不一定连得上。而这里只需要十几条消息，
手工组帧的字节已经用 `pymavlink` 生成的参考帧**逐字节核对过**
（见 `test/test_mavlink_codec.py` 里三个 `GOLDEN_*` 常量）。
少一个依赖，就少一类「昨天还能跑今天跑不起来」。

## 现在能做什么（第一阶段：只读）

`px4_node` **只读**，不发任何控制量，可以放心在装桨、通电的飞机上跑。

```bash
# 1) 先让出串口（DDS 时代的服务会占着它）
sudo systemctl stop micro-xrce-agent

# 2) 起节点
ros2 launch px4_offboard px4_offboard.launch.py

# 3) 另开一个终端看结果
ros2 topic echo /flight/status
```

看到 `communication_ok: true`、`armed` / `current_mode` / `current_height`
跟着飞机状态变，就说明链路通了。

## 关键设计（写上行之前务必读）

### 1. MAVLink 上没有 `OffboardControlMode`

DDS 路线上要另发 `OffboardControlMode` 当心跳。**MAVLink 路线上设定点流本身就是心跳**：
`SET_POSITION_TARGET_LOCAL_NED` 以 **≥2 Hz** 持续发，PX4 才认为外部控制在线；
停发超过 `COM_OF_LOSS_T`（默认 1 s）就退出 offboard。

### 2. 流频率用 `REQUEST_MESSAGE(512)` 点名，不要用 `SET_MESSAGE_INTERVAL(244)`

实测：`244` 在 PX4 1.17 上回 `UNSUPPORTED`（命令似乎没进 mavlink 接收机那个分支），
而 `512` 回 `ACCEPTED` 且**每次都能拉出一条**。所以频率完全由我们自己控：

| 消息 | 频率 | 说明 |
|---|---|---|
| `LOCAL_POSITION_NED`(32) | 25 Hz | 位置/速度，`z` 向下为正，**高度 = -z** |
| `ATTITUDE`(30) | 10 Hz | 姿态 |
| `EXTENDED_SYS_STATE`(245) | 2 Hz | `landed_state`（落地/空中） |
| `ESTIMATOR_STATUS`(230) | 2 Hz | 位置估计质量 |

实测这条组合的总流量 **6.5 KB/s**（下行 4.2 + 上行 2.4），115200 用了 57%。

### 3. 两个反复踩过的协议坑

* **v2 帧头是 10 字节**（带签名再 +13）。写成 12 会让载荷整体错位 2 字节，
  表现是「所有参数都读成 0」。
* **PX4 把整数参数的位模式当 float 传**。`PARAM_VALUE` 要按 `param_type` 还原，
  否则 `SYS_AUTOSTART=4001` 读出来是 `0.0000`。

### 4. 坐标系四条法则

```
世界系位置/速度： (x, y, z)_ENU -> (y, x, -z)_NED     ← 换 x/y + z 取反
世界系姿态：     q_ned_frd = NED_ENU_Q ⊗ q_enu_flu ⊗ AIRCRAFT_BASELINK_Q
机体系矢量：     (x, y, z)_FLU -> (x, -y, -z)_FRD     ← 不换位，只 y/z 取反
偏航：           yaw_ned = π/2 - yaw_enu
```

⚠ 第 1 条和第 3 条**法则不一样**。把世界系的「换 x/y」套到机体速度上，
前后左右会整体转 90°。`test_frames.py::test_body_vector_world_consistency`
就是专门盯这条不变式的。

⚠ 偏航是**反射**不是常量偏移（两边 z 轴正方向相反），所以不存在能对全部
航向成立的常数 offset。全工程只允许 `frames.py` 这一处做换算。

## 自检

不需要 ROS、不需要 pytest：

```bash
python3 test/test_mavlink_codec.py     # 18 项：含 3 个与 pymavlink 逐字节一致的参考帧
python3 test/test_frames.py            # 16 项：含 2 万组随机位姿的不变式
```

装了 pytest 也可以 `colcon test --packages-select px4_offboard`。

## 串口设备名

飞控是 CH340（`1a86:7523`），**没有唯一序列号**，只能靠插在哪个 USB 口区分。
`K230` 之后也要走 CH340，两个设备抢同一套 VID/PID —— 详见
`docs/通信协议设计_v1.md` §9「两个 CH340 的坑」。

```bash
ls -l /dev/flight_controller        # udev 固定名
udevadm info -q property -n /dev/ttyUSB0 | grep -E 'ID_PATH|ID_SERIAL'
```

## 下一步

第二阶段（尚未实现）才做上行：

1. `/map/current_pose` → `VISION_POSITION_ESTIMATE(102)` 30–50 Hz
2. `/pid/cmd_vel` → `SET_POSITION_TARGET_LOCAL_NED(84)` 10–20 Hz
3. `/flight/command` → `COMMAND_LONG(76)`（解锁 / 切模式 / 起降）
4. 先只读跑通验收，再一条条开上行
