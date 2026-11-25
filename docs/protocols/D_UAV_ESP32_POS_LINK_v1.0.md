# D_UAV_ESP32_POS_LINK v1.0 —— 树莓派 ↔ ESP32-S3 坐标链路

| 项 | 值 |
|---|---|
| 文档编号 | `D_UAV_ESP32_POS_LINK` |
| 版本 | v1.0 |
| 状态 | **已真机跑通**（树莓派 5 + Ubuntu 24.04 + ROS 2 Jazzy + ESP32-S3 / ESP-IDF v5.5.3） |
| 涉及包 | `src/comm/pos_link`（树莓派侧）、`firmware/esp32_pos_tx`（ESP32 侧） |
| 上游相关 | `D_UAV_UNIFIED_DEVICE_PROTOCOL_v1.7`（三条链路的统一 `AA 55` 封装思路）、`docs/通信协议设计_v1.md`（`car_link` 的变长帧） |

本文件是**这一条链路**的完整定义：拓扑、地址、帧、时序、异常处理、双端对账表。
两边实现有任何一处对不上，现象都是「TCP 连上了，但一帧也解不出来」，
所以改协议必须同步改 `pos_link/frame.py`、`firmware/esp32_pos_tx/main/main.c`
和本文件三处。

---

## 1. 链路拓扑

```
                     ┌──────────────────────────────┐
                     │ 树莓派 5                      │
   ESP32-S3          │  wlan0  192.168.50.1/24  (AP) │
   (STA) ──WiFi──────┤   ├─ NetworkManager 内置 DHCP │
                     │   └─ pos_link bridge  :5000   │
                     │  eth0   192.168.100.1/24      │── 网线 ── 笔记本（调试/SSH 保底通道）
                     └──────────────────────────────┘
```

- **树莓派开热点，ESP32 作客户端**。方向是 ESP32 → 树莓派，TCP 由 ESP32 主动发起。
- 树莓派**不发起连接、不需要知道 ESP32 的 IP**。ESP32 重启 / DHCP 换地址 / 掉线重连，
  对树莓派来说都只是「一个客户端断开、一会儿再来一个」。
- 树莓派此时**没有外网**（默认路由被热点接管）。这不是故障：本链路不需要 NTP、不需要
  装包、ROS 2 本地通信走 loopback，全都正常。

### 1.1 地址与参数

| 项 | 值 | 在哪改 |
|---|---|---|
| SSID | `PiLink` | `nmcli connection modify PiLink ... ssid` |
| 密码 | `raspberry123`（WPA2-PSK，CCMP） | 同上 `802-11-wireless-security.psk` |
| 频段 / 信道 | 2.4 GHz / 信道 6（2437 MHz） | `802-11-wireless.band bg`、`.channel 6` |
| 树莓派 AP 地址 | `192.168.50.1/24` | `ipv4.addresses` |
| DHCP 池 | `192.168.50.10 – 192.168.50.254`，租期 60 min | NetworkManager 自动生成 dnsmasq 参数 |
| TCP 端口 | **5000** | `pos_link.yaml` `listen_port` / 固件 `HOST_PORT` |
| 发送周期 | **1 Hz**（1000 ms） | 固件 `SEND_PERIOD_MS` |
| 有线调试网段 | 树莓派 `192.168.100.1/24` ↔ 笔记本 `192.168.100.2/24` | `nmcli connection modify eth0 ...` |

> ★ **必须选 2.4 GHz。** ESP32-S3 只支持 2.4 GHz；hostapd 里写成 `hw_mode=a`（5 GHz）
> 或 nmcli 写成 `band=a`，ESP32 连 SSID 都搜不到。

---

## 2. 帧格式

**固定 12 字节，小端。**

| 偏移 | 长度 | 字段 | 说明 |
|---|---|---|---|
| 0 | 2 | 帧头 | 固定 `0xAA 0x55` |
| 2 | 1 | 类型 | `0x01` = 坐标帧（本版本唯一类型） |
| 3 | 1 | 序号 | 0–255 循环递增，用于发现丢帧 |
| 4 | 2 | x | int16，小端 |
| 6 | 2 | y | int16，小端 |
| 8 | 2 | CRC16 | CRC-16/CCITT-FALSE，覆盖偏移 0–7（含帧头） |
| 10 | 2 | 帧尾 | 固定 `0x0D 0x0A` |

- 坐标单位由应用层自定（当前是厘米级整数，取值 0–999）。
- int16 范围 −32768…32767，比三位数坐标宽得多，留了余量。
- 序号只用于**统计**丢帧，不做重传。

### 2.1 CRC-16/CCITT-FALSE

| 参数 | 值 |
|---|---|
| 多项式 | `0x1021` |
| 初值 | `0xFFFF` |
| 输入反射 | 否 |
| 输出异或 | 否 |
| 校验范围 | 帧偏移 0–7，共 8 字节 |

**标准自测向量：`crc16_ccitt("123456789") == 0x29B1`。**
两边实现都对不上这个值，就不用往下查了，先把 CRC 写对。

### 2.2 参考帧（两侧必须逐字节一致）

| 含义 | 字节 |
|---|---|
| x=100, y=200, seq=0 | `AA 55 01 00 64 00 C8 00 E5 FC 0D 0A` |
| x=7, y=3, seq=1 | `AA 55 01 01 07 00 03 00 14 5E 0D 0A` |

这两条已经写成单元测试（`src/comm/pos_link/test/test_frame.py` 的
`test_build_frame_known_bytes` 和 `test_build_frame_known_bytes_seq1`），
改协议时测试会先红。

---

## 3. 时序

```
ESP32                                      树莓派 (bridge)
  │  WiFi 关联 + DHCP                        │
  │  ─────────────────────────────────────►  │  dnsmasq 分配 192.168.50.x
  │                                          │
  │  TCP SYN ─────────────────────────────►  │  accept()，日志「客户端接入」
  │  ◄───────────────────────────── SYN/ACK  │
  │                                          │
  │  [AA 55 01 00 ... 0D 0A] ─────────────►  │  校验 → publish /esp32/position
  │            （每 1000 ms 一帧）            │
  │            ...                           │
  ✗  断电 / 走出覆盖 / AP 抖动                │
  │                                          │  1 s watchdog，3 s 无帧告警
  │  重连（断开后立即重试，失败间隔 1 s）      │  「客户端断开」→「客户端接入」
```

- **重连策略**：ESP32 收到 `WIFI_EVENT_STA_DISCONNECTED` 后等 1 s 再 `esp_wifi_connect()`；
  TCP 发不出去就 `close()` 重建。**不要在 WiFi 事件回调里死循环重连** —— 会饿死事件任务。
- **BSSID / 信道锁定**：ESP32 扫描一次后锁定 AP 的 MAC 和信道，之后跳过扫描（省 ~2.5 s）。
  连失 5 次会自动解锁、重新扫描 —— 否则热点换了信道，固件会**永远**连不上。
- **省电**：ESP32 侧 `esp_wifi_set_ps(WIFI_PS_NONE)`。1 Hz 的数据量下省电毫无意义，
  但省电模式会让 TCP 重传超时变长、延迟抖动变大。

---

## 4. 异常处理

| 现象 | 树莓派侧行为 | 说明 |
|---|---|---|
| 缓冲区出现非帧头字节 | 丢弃，直到下一个 `AA 55` | 上电、重连时 TCP 流里可能混入半帧 |
| CRC / 帧尾不对 | 丢帧，`_n_bad++`，退 1 字节重找帧头 | 不能整段丢：假帧头后面可能紧跟真帧 |
| 序号不连续 | `_n_gap++`，照常发布 | 只统计，不重传 |
| 3 s 无帧 | WARN 日志（5 s 节流），打印总帧/坏帧/跳变 | 不退出，等重连 |
| 端口被占 | 启动即报 `OSError: [Errno 98] Address already in use` | 先 `pkill -f bridge` 再重启服务 |

### 4.1 已知坑（都踩过）

1. **`WIFI_REASON_NO_AP_FOUND_IN_AUTHMODE_THRESHOLD`（211）** —— ESP32 找到了 AP，
   但它的安全等级被判定为低于固件设的门槛。本次根因：NetworkManager 生成的 AP 同时通告
   WPA(v1) 和 WPA2，ESP32 把该热点归为 `WPA_PSK`（`auth=2`），而固件门槛被设成
   `WPA2_PSK`。**修法**：固件 `threshold.authmode = WIFI_AUTH_WPA_PSK`，同时把 AP 的
   组播/单播加密都改成 CCMP。
   > 注意：`WIFI_AUTH_OPEN` **不管用** —— 密码 ≥8 位时 ESP-IDF 会静默把 OPEN 门槛
   > 提升成 WPA2，日志里能看到 `authmode threshold changes from OPEN to WPA2`。
2. **Windows 上 `ping` 不通但 SSH 通** —— 代理软件的 TUN 虚拟网卡会拦截 ICMP。
   用 `Test-NetConnection <ip> -Port 22` 判断连通性。
3. **开热点不需要 `hostapd`** —— NetworkManager 1.46 + `dnsmasq-base` 就够了；
   装了 hostapd 反而有被误启动、抢走 wlan0 的风险。

---

## 5. 双端对账表

改协议时逐行核对，任何一行不一致都会表现为「连上但不解帧」。

| 参数 | 树莓派 | ESP32-S3 |
|---|---|---|
| 帧长 | `FRAME_LEN = 12` | `FRAME_LEN 12` |
| 帧头 | `HEAD = b"\xAA\x55"` | `0xAA, 0x55` |
| 帧尾 | `TAIL = b"\x0D\x0A"` | `0x0D, 0x0A` |
| 类型 | `TYPE_POS = 0x01` | `TYPE_POS 0x01` |
| 坐标字节序 | `struct.unpack_from("<hh", ...)` | `x & 0xFF, (x >> 8) & 0xFF` |
| CRC | `crc16_ccitt(f[0:8])` | `crc16_ccitt(f, 8)` |
| 端口 | `listen_port: 5000` | `HOST_PORT 5000` |
| 对端地址 | 不关心（只听） | `HOST_IP "192.168.50.1"` |
| 周期 | — | `SEND_PERIOD_MS 1000` |

---

## 6. 实测数据

真机（树莓派 5 内置 WiFi + ESP32-S3）跑通后的记录：

- WiFi 关联：`rssi ≈ −33 dBm`，`channel 6`，`BW20`，`auth = WPA2-PSK`。
- ESP32 侧首次连接：第 1 次 `ASSOC_FAIL(203)`、第 2 次 `CONNECTION_FAIL(205)`、
  第 3 次成功 —— 上电时 AP 偶尔还没准备好，重试逻辑有效。
- 发送：`1 Hz`，`ros2 topic hz /esp32/position` 实测 **1.000 Hz**，
  std dev ≈ 0.02 s，min 0.960 s / max 1.040 s。
- 长时间运行：`累计` 计数每次日志 +5（日志 5 s 节流），即 **无丢帧**。
- 桥接进程重启后 ESP32 **自动重连**，无需人工干预。

---

## 7. 变更记录

| 版本 | 日期 | 内容 |
|---|---|---|
| v1.0 | 2026-11 | 首次定稿：12 字节定长帧 + CRC16，5000 端口，1 Hz；记录 211 坑与双端对账表 |
