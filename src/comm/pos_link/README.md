# pos_link —— ESP32-S3 坐标链路（WiFi + TCP）

树莓派用**自带 WiFi 模块开热点当 AP**，ESP32-S3 作 STA 连上来，以 **1 Hz** 推送
**12 字节定长坐标帧**；本包在树莓派侧收帧、校验，并发布成
`/esp32/position`（`geometry_msgs/PointStamped`）。

```
ESP32-S3 ──WiFi──► 树莓派 wlan0 192.168.50.1 (AP, SSID PiLink)
                        │
                        └─ bridge 节点 TCP :5000 ──► /esp32/position
```

## 一分钟跑起来

树莓派上（热点已在跑）::

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select pos_link
source install/setup.bash
ros2 launch pos_link pos_link.launch.py
```

ESP32-S3 上电后应当看到::

```
[INFO] [wifi_bridge]: TCP 监听 0.0.0.0:5000
[INFO] [wifi_bridge]: 客户端接入 192.168.50.240:62349
[INFO] [wifi_bridge]: 帧 seq=0 x=7 y=3 累计=1
```

验证::

```bash
ros2 topic echo /esp32/position
ros2 topic hz /esp32/position        # 应为 1.000 Hz 左右
```

没有 ESP32 也能验：`python3 ../../../tools/pos_link_sim.py`（假 ESP32，按 1 Hz 发包）。

## 协议

| 偏移 | 长度 | 字段 |
|---|---|---|
| 0 | 2 | 帧头 `0xAA 0x55` |
| 2 | 1 | 类型，`0x01` = 坐标帧 |
| 3 | 1 | 序号 seq，0–255 循环 |
| 4 | 2 | x，int16 小端 |
| 6 | 2 | y，int16 小端 |
| 8 | 2 | CRC16-CCITT-FALSE，覆盖偏移 0–7 |
| 10 | 2 | 帧尾 `0x0D 0x0A` |

完整说明（含参考帧、双端对账表）见
`docs/protocols/D_UAV_ESP32_POS_LINK_v1.0.md`。

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `listen_port` | `5000` | 必须和固件 `HOST_PORT` 一致 |
| `frame_id` | `esp32` | 只是标签，本系统不做 TF 变换 |
| `timeout_s` | `3.0` | 超时告警阈值（1 Hz 下等于连丢 3 帧） |

`config/pos_link.yaml` 里的节点名必须写 `wifi_bridge` —— 那是代码里定死的节点名。

## 设计取舍

- **为什么不用 DDS / micro-ROS 直连？** DDS 的发现靠多播，跨 WiFi 经常发现不到；
  1 Hz 两路坐标用 TCP + 12 字节定长帧，故障面小得多，抓包也看得懂。
- **为什么是定长帧而不是变长帧？** 12 字节、1 Hz，带宽从来不是瓶颈。定长让接收端
  只要 12 字节窗口 + 两个常数比较就能切帧，不用维护长度字段和状态机。
- **为什么不做重传？** TCP 已经保证了不丢不乱；上层再叠重传，只会让「丢一帧」
  变成「卡两秒」。1 Hz 的坐标丢一帧，下一秒就补上了。
- **为什么 ESP32 主动连、树莓派只听？** 树莓派不用知道 ESP32 的 IP；ESP32 重启、
  DHCP 换地址、掉线重连，树莓派这边都只是「一个客户端断开、一会儿再来一个」。

## 测试

```bash
cd src/comm/pos_link && python3 -m pytest test -q
```

`test_frame.py` 覆盖 CRC 标准向量、两帧参考字节、往返编解码、坏 CRC / 坏帧尾 /
坏帧头 / 未知类型 / 长度错误，以及 int16 边界。**不需要 ROS**。

## 与 car_link 的区别

两条链路互不相干，别混：

| | `pos_link`（本包） | `car_link` |
|---|---|---|
| 端口 | 5000 | 9000 |
| 帧 | 12 字节定长 | `AA 55` 变长（带 LEN 字段） |
| 频率 | 1 Hz | 2 Hz（`DRONE_POSE`） |
| 内容 | 只有 x/y | 位姿、火点、航程、状态、ACK |
| 状态 | **已真机跑通** | 骨架，协议已冻结 |

`car_link` 是指挥车的正式链路；`pos_link` 是同一套「树莓派开热点 + ESP32 连上来」
思路的最小可用实现，先把它跑通、把热点和重连这些坑趟平，再往上叠 `car_link` 的多字段协议。
