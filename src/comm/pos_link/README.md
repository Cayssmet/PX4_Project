# pos_link —— 树莓派 → ESP32-S3 坐标下行链路（WiFi + TCP）

树莓派用自带 WiFi 开热点当 AP，ESP32-S3 作 STA 连上来，树莓派把算好的坐标
以 **1 Hz、12 字节定长帧**推给 ESP32。ESP32 收到就直接拿去执行。

```
  坐标源（定位/任务/手敲命令）
        │  geometry_msgs/PointStamped
        ▼  <input_topic>  默认 /pos_link/target
  ┌──────────────────────── 树莓派 ────────────────────────┐
  │  pos_link_bridge: 订阅 → 打包 12 字节帧 → TCP 推送      │
  │  热点 PiLink 192.168.50.1/24，TCP 服务端 :5000          │
  └────────────────────────────┬───────────────────────────┘
                               │ WiFi（ESP32 主动连上来）
                               ▼
                    ESP32-S3：收帧 → 校验 → 执行
```

**这条链路只负责搬运**：坐标怎么算、算了准不准、ESP32 收到干什么，都不在本包范围。
换坐标源不用改代码，改 `input_topic` 就行。

## 快速开始

```bash
# 1. 编
cd <工作区根目录>
colcon build --packages-select pos_link
source install/setup.bash

# 2. 起桥接节点（会打印「已监听 0.0.0.0:5000」）
ros2 launch pos_link pos_link.launch.py

# 3. 没有真坐标源时，手敲一个假的（另开一个终端）
ros2 topic pub -r 1 /pos_link/target geometry_msgs/msg/PointStamped \
  "{header: {frame_id: map}, point: {x: 120.0, y: 80.0}}"

# 4. 没有真板子时，用假 ESP32 看帧到没到
python3 tools/pos_link_sim.py            # 默认连 192.168.50.1:5000
python3 tools/pos_link_sim.py --host 127.0.0.1   # 就在本机连
```

正常情况下桥接节点每 5 秒打印一行 `已下发 seq=.. x=.. y=..`，假 ESP32 每帧打一行。

## 坐标从哪来（接口）

本节点**只订阅一个话题**，这就是它的全部输入接口：

| 项目 | 值 |
|---|---|
| 消息类型 | `geometry_msgs/PointStamped` |
| 默认话题 | `/pos_link/target`（参数 `input_topic` 可改） |
| 取值字段 | `point.x`、`point.y`（单位随你，协议里只是 int16） |
| 频率 | 任意。节点自己按 `send_period_ms` 重采样，只取「最近一次」 |
| 坐标系 | 不解释。`frame_id` 只做日志，本包不做 TF 变换 |

几种接法：

- **定位节点直接发**：把它的输出 remap 到 `/pos_link/target`。
- **任务层算完再发**：`competition_g` 决策后发目标点，本包负责送下去。
- **临时手测**：`ros2 topic pub`（见上）。
- **换成非话题来源**（比如服务调用、读文件）：改 `bridge_node.py` 的
  `_on_target()` 即可，它是唯一的入口，改完不用动别的地方。

坐标源断了：默认 `hold_last_on_stale: true`——继续发最后一次值（对执行机构来说
「知道位置但不动」比「突然没指令」安全），同时打告警。想改成「断了就停发」，
把该参数设成 `false`。

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `listen_port` | `5000` | 监听端口，必须和固件 `HOST_PORT` 一致 |
| `input_topic` | `/pos_link/target` | 坐标来源话题 |
| `send_period_ms` | `1000` | 下发周期（毫秒），最小 50 |
| `hold_last_on_stale` | `true` | 坐标过期还发不发 |
| `stale_warn_s` | `5.0` | 多久没新坐标算过期 |

`config/pos_link.yaml` 里的顶层键必须是 **`pos_link_bridge`** —— 那是代码里
定死的节点名，写错了参数会静默不生效。

## 怎么确认在正常工作

| 看什么 | 正常应该看到 |
|---|---|
| 节点启动日志 | `已监听 0.0.0.0:5000（等 ESP32 连上来）…` |
| ESP32 连上 | `ESP32 已接入 192.168.50.240:xxxxx（当前 1 个连接）` |
| 有坐标、在发 | `已下发 seq=12 x=120 y=80（1 个连接）` |
| 每 10 秒统计 | `统计：已发 130 帧，在线连接 1 个，累计断开 0 次` |
| 变量核对 | `ros2 param get /pos_link_bridge input_topic` |

## 和 `car_link` 的关系

同一套「树莓派开热点 + ESP32 连上来」的底层，但**两条独立链路、互不影响**：

| | `pos_link`（本包） | `car_link` |
|---|---|---|
| 端口 | 5000 | 9000 |
| 帧 | 12 字节定长 | `AA 55` + 变长 `LEN` + 多字段 |
| 方向 | 树莓派 → ESP32（单向下发） | 双向 |
| 用途 | 只送坐标（最小可用） | 指挥车正式协议 |

## 测试

```bash
cd src/comm/pos_link
python3 -m pytest test -q      # 不需要 ROS、不需要硬件
```

测试里有两条**参考帧**的期望字节，和协议文档、固件 `build_frame()` 是同一份值。
改了协议这三处必须一起改，否则测试会红——这是故意的。
