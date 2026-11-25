# D_UAV_ESP32_POS_LINK_v1.0 —— 树莓派 → ESP32-S3 坐标下行链路

| 项 | 值 |
|---|---|
| 协议版本 | v1.0（2026-11，方向：树莓派 → ESP32） |
| 涉及包 | `src/comm/pos_link`（树莓派侧）、`firmware/esp32_pos_rx`（ESP32 侧） |
| 底层 | WiFi 2.4 GHz（树莓派开热点）+ TCP |
| 状态 | 树莓派侧已跑通单元测试；ESP32 侧接收固件已入库，**端到端待上板复核** |

> 本协议只管「怎么把一对坐标可靠地送到 ESP32」。坐标怎么算出来、ESP32 收到后
> 用来干什么，都不在这里规定。改字段 = 同时改 `pos_link/frame.py`、
> `tools/pos_link_sim.py`、`firmware/esp32_pos_rx/main/main.c` 三处，缺一处必出错。

---

## 1. 拓扑与角色

```
        树莓派 5                                    ESP32-S3
 ┌─────────────────────────┐                 ┌───────────────────────┐
 │ wlan0  192.168.50.1/24  │   WiFi 2.4GHz   │ STA，DHCP 拿 192.168.50.x │
 │ 热点 SSID: PiLink       │ ◄───────────────┤ SSID/PASS 写死在固件里 │
 │                         │                 │                       │
 │ pos_link_bridge         │   TCP :5000     │ tcp_connect()         │
 │  └ 监听 0.0.0.0:5000 ───┼─────────────────┤ └ 主动连 192.168.50.1 │
 │                          │  12 字节/帧 1Hz │  └ recv → 校验 → 执行 │
 └─────────────────────────┘                 └───────────────────────┘
```

**为什么树莓派当服务端**：ESP32 的 IP 是 dnsmasq 分出来的，树莓派事先不知道；
而树莓派在热点里永远是 `192.168.50.1`。所以让 ESP32 主动连上来，树莓派只管
记住这条连接往里推。两边都不需要维护对方的地址表。

| 角色 | 谁 | 做什么 |
|---|---|---|
| AP / TCP Server | 树莓派 | 开热点、发 DHCP、监听 5000、按 1 Hz 推帧 |
| STA / TCP Client | ESP32-S3 | 连热点、连服务端、收帧、校验、执行 |
| 坐标源 | 上层 ROS 2 节点 | 往 `input_topic` 发 `PointStamped`（本协议不关心） |

---

## 2. 地址与参数

| 项 | 值 | 在哪配 |
|---|---|---|
| SSID | `PiLink` | `nmcli` 连接 profile / 固件 `WIFI_SSID` |
| 密码 | `raspberry123` | 同上 |
| 信道 | 6（2437 MHz，20 MHz 带宽） | `nmcli` profile `802-11-wireless.channel` |
| 树莓派地址 | `192.168.50.1/24` | `nmcli` profile `ipv4.addresses` |
| DHCP 池 | `192.168.50.10 – 192.168.50.254`，租期 60 min | NetworkManager 的 `shared` 模式 |
| TCP 端口 | **5000** | `config/pos_link.yaml` `listen_port` / 固件 `HOST_PORT` |
| 下发周期 | 1000 ms | `config/pos_link.yaml` `send_period_ms` |

---

## 3. 传输层约定

- **TCP 长连接**，连上就一直用，断了重连。不搞 UDP：1 Hz 的数据量用不着，
  而丢一帧坐标就得等下一秒，不如让 TCP 保证「到了就是对的」。
- `TCP_NODELAY` 打开。每帧才 12 字节，Nagle 攒包只会平白多几十毫秒。
- 树莓派侧对每个客户端设 1 s 发送超时：某个 ESP32 卡住不能把 ROS 定时器拖住。
- ESP32 侧设 3 s 接收超时：超时不当断线，只是打印「3 秒没收到帧」继续等——
  树莓派可能只是暂时没有坐标源。
- 一个 ESP32 掉线不影响其他连接；树莓派会把它从连接表里摘掉并打一行警告。

---

## 4. 帧格式（12 字节，小端）

| 偏移 | 字节数 | 字段 | 说明 |
|---|---|---|---|
| 0 | 1 | `0xAA` | 帧头 |
| 1 | 1 | `0x55` | 帧头 |
| 2 | 1 | 类型 | `0x01` = 坐标帧（预留其它类型） |
| 3 | 1 | `seq` | 0–255 循环，用来发现丢帧 |
| 4–5 | 2 | `x` | `int16`，小端 |
| 6–7 | 2 | `y` | `int16`，小端 |
| 8–9 | 2 | `crc` | CRC-16/CCITT-FALSE，小端，覆盖 **0–7** |
| 10 | 1 | `0x0D` | 帧尾 |
| 11 | 1 | `0x0A` | 帧尾 |

- 坐标单位不做规定（cm / mm 都行），两侧必须一致。当前 G 题场景用 **cm**，
  场地 480×400 cm，`int16` 远远够用。
- 超出 `int16` 的值：树莓派侧**先四舍五入再夹到 `[-32768, 32767]`** 并告警，
  绝不抛异常中断链路。NaN/Inf 直接丢弃。

### CRC 参数

| 参数 | 值 |
|---|---|
| 算法 | CRC-16/CCITT-FALSE |
| poly | `0x1021` |
| init | `0xFFFF` |
| refin / refout | 否 / 否 |
| xorout | 无 |
| 校验向量 | `"123456789"` → `0x29B1` |

### 参考帧（三处代码必须一致）

```
x=100, y=200, seq=0 →  AA 55 01 00 64 00 C8 00 E5 FC 0D 0A
x=7,   y=3,   seq=1 →  AA 55 01 01 07 00 03 00 14 5E 0D 0A
```

这两条已经写成单元测试（`test/test_frame.py`），也写在假 ESP32
（`tools/pos_link_sim.py`）里。改协议忘了改某一处，测试或抓包会立刻露馅。

---

## 5. 时序

```
树莓派                                       ESP32
  │  监听 5000                                │
  │  ◄────────── TCP SYN ──────────────────── │ 上电/重连
  │  ─────────── SYN-ACK ──────────────────► │
  │  ◄────────── ACK ──────────────────────── │
  │                                            │
  │  ── 帧(seq=0) ──────────────────────────► │ 校验 → on_position()
  │  ── 帧(seq=1) ──────────────────────────► │   （1000 ms 后）
  │  ── 帧(seq=2) ──────────────────────────► │
  │            …… 一直推，直到任一方断开 ……      │
```

- 树莓派**不等 ACK**，也不重传。TCP 保证顺序与完整性，1 Hz 下丢一帧等于晚一秒，
  对执行机构来说下一帧就到了，没必要做应用层重传。
- 树莓派在**没有客户端**时照常运转，只是不打包（省电、少日志噪音）。
- 树莓派在**没有坐标**时不发帧。ESP32 侧表现为「3 秒没收到帧」告警，连接不断。

---

## 6. 异常处理

| 场景 | 树莓派侧 | ESP32 侧 |
|---|---|---|
| ESP32 掉电/重启 | `sendall` 抛错 → 摘掉该连接，打警告 | 上电后重新 `tcp_connect()` |
| 树莓派进程重启 | — | `recv` 返回 0 → 关 socket，1 s 后重连 |
| 树莓派没有坐标源 | 每 10 s 打「还没有坐标，暂不发送」 | 3 s 后打「3 秒没收到帧」 |
| 坐标过期（源卡住） | 默认继续发最后值 + 告警（`hold_last_on_stale`） | 无感（照收） |
| 帧校验失败 | — | 计坏帧数，丢弃，继续找下一个 `AA 55` |
| 字节流错位 | — | 逐字节状态机重新同步，不整段丢弃 |
| 端口被占用 | 启动即报 `端口 5000 绑定失败` | 连不上，2 s 后重试 |
| ESP32 连上了但收不到 | 检查是否「还没有坐标」 | 检查 `input_topic` 有没有人发 |

---

## 7. 踩过的坑

### 7.1 `reason=211`（`NO_AP_FOUND_IN_AUTHMODE_THRESHOLD`）

ESP32 明明能扫到 `PiLink`（`rssi=-24`），却连不上、每 3.4 秒报一次
`WIFI_REASON_NO_AP_FOUND_IN_AUTHMODE_THRESHOLD`，全程没有一次 `init -> auth`。

根因不在信号、不在信道、不在密码：

1. NetworkManager 生成的 AP 同时通告 WPA(v1) 与 WPA2 的 IE。
2. 默认组播/单播加密是 **TKIP**，ESP32 扫描结果里把该热点归为
   `auth=2(WPA_PSK)`、`pair=5(TKIP+CCMP)`、`group=3(TKIP)`。
3. 固件门槛是 `WIFI_AUTH_WPA2_PSK` → 该热点被筛掉 → 211。

更阴的是：把门槛写成 `WIFI_AUTH_OPEN` **也没用**——密码 ≥ 8 位时 IDF 会自动
把门槛提升到 WPA2，日志里只有一句
`Password length matches WPA2 standards, authmode threshold changes from OPEN to WPA2`。

正解（两侧都做）：
- 固件：`threshold.authmode = WIFI_AUTH_WPA_PSK`（显式写 WPA_PSK 不会被自动提升）。
- 热点：`nmcli connection modify PiLink 802-11-wireless-security.group ccmp`
  + `802-11-wireless-security.pairwise ccmp`，把 TKIP 去掉。

即使改完 CCMP，ESP32 仍把该热点报成 `auth=2(WPA_PSK)`（wpa_supplicant 的 AP 模式
多带了一个 WPA(v1) IE）。**所以门槛必须留在 `WPA_PSK`，改回 WPA2 会立刻复发 211。**

### 7.2 BSSID 写死会锁死自己

固件开机扫描一次、记住「信道 + BSSID」，之后不再扫，省下每次开机 2~3 秒。
副作用：热点要是换了信道或重建了，旧 BSSID 就永远连不上。
所以固件里加了 **连失 5 次自动解锁**，改回按 SSID 重新扫描。

### 7.3 `Address already in use`

手动跑过一个 bridge，再 `systemctl start pos-link` 就 `[Errno 98]`，然后 5 秒一重启。
`pkill -f 'pos_link.*bridge'` → `ss -ltnp | grep ':5000'` 确认空 → 再启动。

---

## 8. 双端对账表

改协议时逐行对，两边必须一致：

| 字段 | 树莓派侧 | ESP32 侧 |
|---|---|---|
| 帧头 | `frame.HEAD = b"\xAA\x55"` | `FRAME_HEAD0/1` |
| 帧尾 | `frame.TAIL = b"\x0D\x0A"` | `FRAME_TAIL0/1` |
| 类型 | `TYPE_POS = 0x01` | `TYPE_POS` |
| 长度 | `FRAME_LEN = 12` | `FRAME_LEN 12` |
| 序号 | `build_frame(x, y, seq)` 的 `seq & 0xFF` | `f[3]` |
| 坐标 | `struct.pack("<BBhh", …)` | `f[4] \| f[5]<<8`（int16） |
| CRC | `crc16_ccitt(body)` | `crc16_ccitt(f, 8)` |
| 端口 | `listen_port` 默认 5000 | `HOST_PORT 5000` |
| 对端地址 | 监听 `0.0.0.0` | `HOST_IP "192.168.50.1"` |

---

## 9. 怎么验证

| 层次 | 做法 | 期望 |
|---|---|---|
| 编解码 | `pytest test -q`（不需要 ROS/硬件） | 全绿 |
| 服务端在发 | `ros2 topic pub -r 1 /pos_link/target …` + `tools/pos_link_sim.py --host 127.0.0.1` | 假 ESP32 每秒打印一行帧 |
| 真板子收 | 串口 monitor | `已连接 192.168.50.1:5000` → `收到 seq=.. x=.. y=..` |
| 序号连续 | 看串口 `seq` 或假 ESP32 的统计 | 连续递增（模 256），跳过说明丢帧 |
| 频率 | `ros2 topic hz`（在坐标源上）对比串口 | 应同为 1 Hz |

**尚未实测**：ESP32 侧端到端时延（树莓派 `sendall` 到 ESP32 `recv` 返回）。
上板后建议在两侧各打一次 `esp_timer_get_time()` 对比，或看串口相邻两帧间隔的
抖动（应为 1000 ms ± 几十 ms）。上一版（反方向）在这条链路上实测过 1.000 Hz、
std ≈ 0.02 s，可作为参考，但**不能直接当成本版的数据**。

---

## 10. 变更记录

| 版本 | 日期 | 变更 |
|---|---|---|
| v1.0 | 2026-11 | 首版。**方向为树莓派 → ESP32**：树莓派算坐标、打包、1 Hz 下发；ESP32 接收后执行。 |

> 归档时的早期版本（`v3.2.0` 那次提交）方向写反了（ESP32 → 树莓派），
> 本版已按项目实际需求改正：坐标由树莓派产生并下发，ESP32 只做执行端。
