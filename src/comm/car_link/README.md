# car_link —— 无人机 ↔ 消防车无线链路

## 拓扑（由赛题约束推出）

赛题要求「无人机与消防车之间采用无线通信；巡逻期间无人机每秒向消防车发送 1 次位置坐标」，
且**不允许场地内布置辅助定位装置**。所以链路必须是无人机 ↔ 车直连，不能经路由器中转。

```
树莓派（AP + TCP Server, 0.0.0.0:9000）
        │  WiFi 热点  UAV_G
        ▼
ESP32-S3 小车（TCP Client）  ← 队友负责
```

树莓派同时还要：① 通过网线接笔记本（SSH、地面站）；② 通过 USB 接飞控；③ 通过 USB/UART 接 K230。
热点只服务小车这一条链路，不承载 ROS 2 DDS 发现（`ROS_DOMAIN_ID` 保持本机/网线侧）。

★ **开热点不需要装 hostapd**。NetworkManager 1.46 会让 `wpa_supplicant` 直接进 AP 模式，
`ipv4.method shared` 用自带的 `dnsmasq-base` 发 DHCP、NM 自己写 iptables 做 NAT。
（实测：`nmcli` 1.46.0、`WIFI-PROPERTIES.AP: 是`、`wpa_supplicant v2.10`、hostapd 未安装但可用。）

## 赛题对数据的要求

| 方向 | 内容 | 频率 |
|---|---|---|
| 无人机 → 车 | 无人机当前位置坐标（**场地坐标，单位 dm**） | **1 Hz 必须**（我们发 2 Hz） |
| 无人机 → 车 | 火源坐标（发现后立即发） | 事件触发 |
| 无人机 → 车 | 累计巡逻航程 | 1 Hz（车侧显示） |
| 车 → 无人机 | 状态/是否已熄灭火源（可选） | 事件触发 |

★ 坐标必须是**以巡防区左下角为原点的场地坐标**，不是 SLAM 地图坐标 —— 需要一次标定把
`odom` 坐标系对齐到场地坐标系（旋转 + 平移）。有效范围 `x∈[0,480] y∈[0,400] cm`，
**超出必须报错而不是照发**。

## ★ 协议已定稿：见 `docs/通信协议设计_v1.md`

帧封装与链路 A 共用：`AA 55 | VER | TYPE | LEN(2,LE) | PAYLOAD | CRC16(2,LE)`，
CRC-16/CCITT-FALSE。参考实现：`python3 tools/proto_selftest.py`（33 项全绿）。

| 方向 | TYPE | 名称 | 频率 | 载荷 |
|---|---|---|---|---|
| Pi → 车 | `0x00` | HEARTBEAT | 1 Hz | 空 |
| Pi → 车 | `0x01` | DRONE_POSE | **2 Hz** | `"<hhhhHBB"` 12 B |
| Pi → 车 | `0x02` | FIRE_REPORT | 事件 | `"<BBhhI"` 10 B |
| Pi → 车 | `0x03` | MILEAGE | 1 Hz | `"<iiI"` 10 B |
| Pi → 车 | `0x04` | MISSION_STATE | 2 Hz | `"<BBI"` 6 B |
| 车 → Pi | `0x81` | CAR_STATUS | 1 Hz | `"<hhhBB"` 8 B |
| 车 → Pi | `0x82` | FIRE_EXTINGUISHED | 事件 | `fire_id(u8)` |
| 车 → Pi | `0x83` | ACK | 事件 | — |

**端口默认 9000，可配置。**

## ★ 没有真车也能联调：`tools/car_link_sim.py`

```bash
# 只收：看树莓派在发什么
python3 tools/car_link_sim.py --host 192.168.4.1

# 收发都要：模拟车上线
python3 tools/car_link_sim.py --host 192.168.4.1 --reply
```

它会解码所有帧、**自己按收到的位置点独立累加航程**，最后和树莓派发的 `MILEAGE` 对比 ——
赛题要求车上显示累计航程，这个对比直接验证两边算的是不是同一个数。

车侧（ESP32）代码由队友写，可以拿这个脚本当**行为参照**：收到什么、该回什么、什么时候回。

## 状态

- v0.1.0：只有目录骨架，没有节点代码。
- 协议已在 `docs/通信协议设计_v1.md` 冻结为 v1（端口 9000 已定；Q5 航迹形式、Q6 `fire_id` 规则待定）。
- 下一步：① 写 `car_link_node`（TCP server + 帧解析 + ROS 话题互转）；
  ② 用 `car_link_sim.py` 在真车到位前把 Pi 侧跑通。
