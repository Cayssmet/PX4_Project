# esp32_pos_rx —— ESP32-S3 坐标接收端

树莓派算好坐标、打包、通过 WiFi 推下来；**这块板子是执行端**：
连上热点、连上树莓派的 TCP 服务、收 12 字节坐标帧、校验、交给你的机构。

配套的树莓派侧：`src/comm/pos_link`（订阅坐标话题 → 打包 → 1 Hz 推送）。
协议细节：`docs/protocols/D_UAV_ESP32_POS_LINK_v1.0.md`。

```
树莓派 192.168.50.1:5000  ──(12 字节帧, 1 Hz)──►  本固件
       （算坐标、当服务端）                         （收帧、执行）
```

## 目录

```
esp32_pos_rx/
├── CMakeLists.txt        ← 工程入口，project(esp32_pos_rx)
├── main/
│   ├── CMakeLists.txt    ← 组件登记
│   └── main.c            ← 全部逻辑在这里
└── sdkconfig.defaults    ← 钉住目标芯片 esp32s3（入库）
```

## 烧录（VSCode ESP-IDF 插件）

1. 用 VSCode **打开 `esp32_pos_rx` 这个目录本身**（不是它的上级目录）——
   打开层级错了插件会提示「不是标准的 ESP-IDF 工程」。
2. 底部状态栏确认三件事：**目标 = esp32s3**、**串口 = 你板子的 COM 口**、
   **烧录方式 = UART/JTAG**。
3. 点 🔨 编译 → 点 ⚡ 烧录 → 点 📺 监视器（Monitor）。

命令行 `idf.py` 在这台机器上用不了（`export.ps1` 的 tools 路径拼不出来），
一律走插件，别折腾。

## 上电后应该看到什么

```
I (xxx) pos_rx: WiFi 已启动
I (xxx) pos_rx: ===== 开始扫描周围 AP =====
I (xxx) pos_rx:   [1] <== ssid="PiLink" rssi=-24 ch=6 auth=2(WPA_PSK) pair=4 group=4
I (xxx) pos_rx: ===== 扫描结束，找到目标 =====
I (xxx) pos_rx: 锁定目标: bssid=2c:cf:67:81:8f:18 ch=6 rssi=-33
I (xxx) pos_rx: 开始连接 PiLink ...
I (xxx) pos_rx: 拿到 IP: 192.168.50.240
I (xxx) pos_rx: TCP 已连接 192.168.50.1:5000，开始等坐标帧
I (xxx) pos_rx: 收到 seq=0 x=120 y=80        ← 每 1 秒一行
I (xxx) pos_rx: 收到 seq=1 x=120 y=80
```

看到「TCP 已连接」就说明网络全通了，剩下只是树莓派那边有没有坐标。

## 常见现象对照

| 现象 | 原因 | 处理 |
|---|---|---|
| 连报 `reason=211` | AP 的加密方式被判定为 WPA1（见协议文档 §7.1） | 固件门槛保持 `WIFI_AUTH_WPA_PSK`；树莓派把 group/pairwise 改成 `ccmp` |
| 连报 `reason=203/205` | 还没连上就被打断 / 握手超时 | 多数是信号弱或 AP 刚重启，等几秒会自愈 |
| `reason=201` | 扫不到该 SSID | 热点没开 / 信道不在 1–13 / 天线 |
| 一直 `连接 192.168.50.1:5000 失败` | 树莓派上的 bridge 没起 | 树莓派上 `ros2 launch pos_link pos_link.launch.py` |
| 连上了但 `3 秒没收到帧` | 树莓派没有坐标源 | 树莓派上看桥接日志，或 `ros2 topic pub` 喂一个 |
| 反复 `帧校验失败` | 帧格式两边不一致 | 逐行对协议文档 §8 的双端对账表 |
| 上电就重启 / `Brownout` | 供电不够 | 换 USB 口/线，加 100 µF + 10 µF 去耦 |

## 接你的执行机构

打开 `main/main.c`，找 `on_position()`：

```c
static void on_position(uint8_t seq, int16_t x, int16_t y)
{
    ESP_LOGI(TAG, "收到 seq=%u x=%d y=%d", seq, x, y);
    // ★ 换成你的动作：算差分、限幅、写 PWM、转 UART 给另一颗 MCU…
}
```

就这一个入口。别在里面做超过几十毫秒的活——它跑在收帧任务里，
1 帧/秒的节奏被拖慢了就会开始报「3 秒没收到帧」。

## 要改的 4 个参数

`main/main.c` 顶部：

| 宏 | 默认 | 说明 |
|---|---|---|
| `WIFI_SSID` | `PiLink` | 树莓派热点名 |
| `WIFI_PASS` | `raspberry123` | 热点密码 |
| `HOST_IP` | `192.168.50.1` | 树莓派在热点里的固定地址 |
| `HOST_PORT` | `5000` | 必须和 `config/pos_link.yaml` 的 `listen_port` 一致 |

## 硬件注意

- **供电**：ESP32-S3 发射瞬间 300–400 mA，USB 线别太细，LDO 留 500 mA 余量，
  否则表现为随机 `Brownout detector was triggered` 重启。
- **串口**：用板上标 UART 的那个 USB 口看日志；只有 USB-OTG 口的话靠原生 USB，
  也能看到 log，但 `idf.py monitor` 的重置快捷键行为不同。
- **天线**：板载 PCB 天线那块别贴金属、别压在电源线下面，`rssi` 掉到 -70 以下
  就要查摆放。

## 和旧版 `esp32_pos_tx` 的区别

方向反了：旧版是 ESP32 → 树莓派（发坐标），新版是树莓派 → ESP32（收坐标）。
包名、`project()`、日志 TAG 都跟着改了；WiFi 连接、扫描锁 BSSID、失联解锁、
211 门槛这些底层逻辑原样保留。
