# esp32_pos_tx —— ESP32-S3 坐标发送端

连树莓派热点 `PiLink`，每秒发一个 12 字节坐标帧到 `192.168.50.1:5000`。

配套接收端：`src/comm/pos_link`（把帧解析成 ROS 2 话题 `/esp32/position`）。
协议文档：`docs/protocols/D_UAV_ESP32_POS_LINK_v1.0.md`。

---

## 一、文件结构（ESP-IDF 项目就三个必需文件）

```
esp32_pos_tx/
├── CMakeLists.txt        ← 顶层，写死 3 行，不用改
├── main/
│   ├── CMakeLists.txt    ← 声明 main.c
│   └── main.c            ← 你唯一要写的代码
├── sdkconfig.defaults    ← 钉住目标芯片 esp32s3（入库）
├── sdkconfig             ← 第一次编译自动生成（不进 git）
└── build/                ← 编译产物，删了能重建（不进 git）
```

`sdkconfig` 是 menuconfig 存下来的配置，第一次 `idf.py build` 会自动生成。丢了也不怕，重新 build 就有默认值。

---

## 二、在 VSCode 里从零建这个项目

### 方式 A：用现有的空项目替换文件（推荐，最省事）

1. 在 VSCode 里用 ESP-IDF 插件新建工程：
   `Ctrl+Shift+P` → 输入 `ESP-IDF: New Project`
   - 选择 **ESP-IDF 框架目录**
   - 工程名随意，比如 `pos_tx`
   - 模板选 **sample_project**（最干净的空模板）
   - 目标芯片选 **esp32s3**
2. 把你看到的 `main/main.c` 内容**整个删掉**，换成我这份。
3. 顶层 `CMakeLists.txt` 和 `main/CMakeLists.txt` 也照我这份改（模板里默认的其实也能用）。

### 方式 B：命令行（更快，但要开 ESP-IDF 终端）

> ⚠️ 有些 Windows 环境的 `export.ps1` 会因为 `IDF_TOOLS_PATH` 配置起不来，报
> `tool xtensa-esp-elf has no installed versions`。遇到这种情况就用方式 A，
> 别在命令行上死磕——VSCode 插件自己拼的环境是对的。

VSCode 里 `Ctrl+Shift+P` → `ESP-IDF: Open ESP-IDF Terminal`，然后：

```bash
cd <你的工作区目录>
idf.py create-project esp32_pos_tx
cd esp32_pos_tx
idf.py set-target esp32s3
```

然后把这三个文件复制进去覆盖。

---

## 三、编译 / 烧录 / 看串口

### VSCode 底部那排按钮（ESP-IDF 插件提供的）

| 按钮/图标 | 作用 | 快捷键 |
|---|---|---|
| 🔌 插头图标 | **选择串口**（COMx） | — |
| 芯片图标 | **选择目标** esp32s3 | — |
| 🗑 垃圾桶 | **Full Clean** 清空 build | — |
| 🔨 锤子 | **Build** 编译 | `Ctrl+E B` |
| ⚡ 闪电 | **Flash** 烧录 | `Ctrl+E F` |
| 📺 显示器 | **Monitor** 看串口输出 | `Ctrl+E M` |

> 第一次用**先点「选择目标」选 esp32s3**。这一步错了会烧进去跑不起来。

### 命令行等价操作

```bash
idf.py set-target esp32s3     # 只做一次
idf.py build                  # 编译
idf.py -p COM5 flash          # 烧录，COM5 换成你的串口
idf.py -p COM5 monitor        # 看串口，Ctrl+] 退出
```

`idf.py flash monitor` 可以一条命令烧完直接看输出。

---

## 四、编译成功后长这样

```
...
Project build complete. To flash, run:
 idf.py flash
```

烧录时长这样：

```
esptool.py v4.x
Serial port COM5
Connecting....
Chip is ESP32-S3 (QFN56) (revision v0.2)
...
Wrote 123456 bytes ...
Hard resetting via RTS pin...
```

---

## 五、烧进去之后，串口应该出现这些

**正常流程：**

```
I (312) pos_tx: WiFi 已启动，开始连接 PiLink ...
I (1523) wifi:new:<6,0>, old:<1,0>, ap:<255,255>, sta:<6,0>, prof:1
I (2013) wifi:state: init -> auth (b0)
I (2050) wifi:state: auth -> assoc (0)
I (2110) wifi:state: assoc -> run (10)
I (2200) wifi:connected with PiLink, aid = 1, channel 6, BW20, bssid = ...
I (2500) esp_netif_handlers: sta ip: 192.168.50.240, mask: 255.255.255.0, gw: 192.168.50.1
I (2500) pos_tx: 拿到 IP: 192.168.50.240
I (2510) pos_tx: TCP 已连接 192.168.50.1:5000
I (3510) pos_tx: 已发送 seq=0 x=7 y=3
I (4510) pos_tx: 已发送 seq=1 x=14 y=6
```

看到 `已发送 seq=` 每秒一行，就是成了。

**此时树莓派上的桥接节点会同时打印：**

```
[INFO] 客户端接入 192.168.50.240:54321
[INFO] 帧 seq=0 x=7 y=3 累计=1
```

---

## 六、出错对照表（按串口里的 reason 号查）

| 串口看到 | 意思 | 怎么办 |
|---|---|---|
| `reason=201` (NO_AP_FOUND) | 没找到这个 SSID | 热点没开 / SSID 拼错 / 热点在 5GHz |
| `reason=15` (4WAY_HANDSHAKE_TIMEOUT) | 密码错 | 核对 `WIFI_PASS` |
| `reason=205` (CONNECTION_FAIL) | 连上了又被踢 | AP 侧问题，看树莓派 `nmcli` 状态 |
| `reason=211` (NO_AP_FOUND_IN_AUTHMODE_THRESHOLD) | **AP 找到了，但加密方式低于门槛被自己过滤掉** | 见下方「211 专项」 |
| `reason=2` (AUTH_EXPIRE) | 认证超时 | 距离太远或信号太弱 |
| `connect ... 失败, errno=113` | 主机不可达 | `HOST_IP` 不对，或树莓派防火墙挡了 |
| `connect ... 失败, errno=111` | 连接被拒绝 | 树莓派上 `bridge.py` 没启动 |
| `socket() 失败` | 内存不足 | 罕见，重启板子 |
| `Brownout detector was triggered` | **供电不足** | 换 USB 口/线，或外接 5V 电源 |

#### 211 专项：为什么明明能看到热点却连不上

`211` 是最容易被误读的错误码——它不是「没找到 AP」，而是「找到了，但被自己的
`threshold.authmode` 滤掉了」。

判断方法：看串口有没有 `wifi:state: init -> auth`。**没有**就说明根本没走到认证阶段，
属于被过滤；密码错是 `reason=15`，不是 211。

排查时打开扫描打印（`main.c` 里的 `find_target_ap()`），重点看 `auth=` 和 `pair=`：

```
[1] <== ssid="PiLink" rssi=-24 ch=6 auth=2(WPA_PSK) pair=5 group=3
```

`auth=2` 说明 ESP32 把这个热点归类为 **WPA1**；`pair=5` 是 TKIP+CCMP、`group=3` 是 TKIP。
门槛若是 `WPA2_PSK`，就会被直接过滤掉。

还有一层陷阱：把门槛写成 `WIFI_AUTH_OPEN` 也没用，IDF 会自动抬回去：

```
W (3146) wifi:Password length matches WPA2 standards, authmode threshold changes from OPEN to WPA2
```

**正确做法（两侧都要做）：**

1. 树莓派侧只通告 CCMP（`pi/setup-ap.sh` 已包含这两行）：

   ```bash
   sudo nmcli connection modify PiLink 802-11-wireless-security.pairwise ccmp
   sudo nmcli connection modify PiLink 802-11-wireless-security.group  ccmp
   sudo nmcli connection up PiLink
   ```

2. 固件侧门槛显式写 `WPA_PSK`：

   ```c
   s_wc.sta.threshold.authmode = WIFI_AUTH_WPA_PSK;
   ```

   > 别改回 `WIFI_AUTH_WPA2_PSK`：即使热点已经是纯 CCMP，`wpa_supplicant` 的 AP 模式
   > 仍会附带一个 WPA(v1) IE，ESP32 依旧报 `auth=2(WPA_PSK)`，一改回去就复发 211。

**烧录阶段的错误：**

| 报错 | 意思 | 怎么办 |
|---|---|---|
| `Failed to connect to ESP32-S3: No serial data received` | 没进下载模式 | 按住 **BOOT** 不放，点一下 **RST**，再松开 BOOT，重烧 |
| `Could not open port COM5` | 串口被占用 | 先关掉 monitor 窗口 / 关掉别的串口助手 |
| `A fatal error occurred: MD5 of file does not match` | 通信不稳 | 降低波特率：`idf.py -p COM5 -b 115200 flash` |
| `CMake Error: The current CMakeCache.txt is different` | 换过目标芯片 | 删掉 `build/` 目录重新编译 |

---

## 七、硬件接线的坑（ESP32-S3 特有）

**1. 板子上常有两个 USB 口**

- 标 **UART** 的那个（芯片是 CP2102/CH343）→ **用这个烧录**，最稳
- 标 **USB** 的那个（原生 USB-JTAG）→ 也能用，但驱动和串口行为有时不一样

**2. 供电**

WiFi 发射瞬间电流能到 **300~400mA**。如果用劣质线或电脑前置 USB 口，会看到：

```
Brownout detector was triggered
rst:0x7 (TG0WDT_SYS_RST)
```

表现为**开机就重启、反复重启**。换到主板后面的 USB 口，或者用带独立供电的 USB Hub。

**3. 天线**

S3 模组板载天线在板子边缘，别用手捏着、别贴着金属桌面、别塞进金属盒子里。

---

## 八、验收标准

在树莓派上开桥接节点：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch pos_link pos_link.launch.py
```

另开一个终端：

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /esp32/position
```

如果每秒刷出一条 `x` / `y` 在变的 `PointStamped`，整条链路就通了：
**ESP32-S3 → 2.4GHz WiFi → 树莓派 AP → TCP → ROS 2 话题**

---

## 九、想改发送内容

改 `app_main()` 里这两行：

```c
s_x = (int16_t)((s_x + 7) % 1000);
s_y = (int16_t)((s_y + 3) % 1000);
```

换成读传感器、算坐标、读串口都行。只要保证 x / y 落在 **0~999**，因为协议是 int16，接收端也没做范围检查。

如果要发别的数据类型，改 `build_frame()` 里的 `f[2]` 类型字节，接收端 `bridge.py` 里对应加分支。
