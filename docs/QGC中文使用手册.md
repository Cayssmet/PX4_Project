# QGroundControl 中文使用手册（对接本机 5.0.8 版本）

> 适用对象：本机 `D:\09_PX4\QGroundControl\bin\QGroundControl.exe`（QGC **5.0.8**，Qt 6 构建）。
> 编写目的：不追求看懂 QGC 全部功能，只覆盖「让 MicoAir743v2 + PX4 1.17.0 飞起来」这条主线用得到的页面。
>
> ⚠️ 两条重要前提
> 1. **本机这份 QGC 没有中文语言包**，切语言不会让主界面变中文（详见附录 A）。所以本手册给你的是「英文界面 + 中文解释」对照。
> 2. 5.0.x 的界面布局与老教程（4.x）大体一致但个别按钮位置有差别。**凡是你屏幕上和我写的对不上，截图发我，我按你的界面重新标**。

---

## 1. 怎么启动

双击 `D:\09_PX4\启动QGC.cmd`（我放的启动脚本，用软件渲染，避开显卡驱动问题）。

若想自己开命令行：

```bat
set QT_QUICK_BACKEND=software
"D:\09_PX4\QGroundControl\bin\QGroundControl.exe"
```

第一次启动会弹出防火墙询问，允许即可。QGC 的配置文件在 `C:\Users\lenovo\AppData\Roaming\QGroundControl\`，**删掉这个目录就等于恢复出厂**（出诡异问题时用）。

---

## 2. QGC 的整体结构（先建立心理地图）

QGC 的界面就三块，别被吓到：

```
┌──────────────────────────────────────────────────────────┐
│ ①左上角 Q 图标 ②顶部工具栏 ③右上角连接状态                │
├───────────┬──────────────────────────────────────────────┤
│           │                                              │
│ ④左侧栏   │  ⑤主内容区（随你选的功能变）                  │
│ (仅在车辆  │                                              │
│  设置里)   │                                              │
└───────────┴──────────────────────────────────────────────┘
```

| 编号 | 是什么 | 英文原文 | 干什么 |
|---|---|---|---|
| ① | 左上角 Q 图标 | `Application Settings` | **QGC 自己的设置**：语言、单位、地图、串口、日志、虚拟摇杆 |
| ② | 顶部工具栏 | `Fly` / `Plan` / 车辆设置图标 / `Analyze` | **飞行** / **航线规划** / **飞控配置** / **分析工具** |
| ③ | 右上角 | `Disconnected` / `Connected` | 有没有连上飞控。点它选串口 |
| ④ | 左侧栏 | （无标题） | 只在「车辆设置」里出现，是各项配置的分类列表 |
| ⑤ | 主内容区 | — | 你实际操作的地方 |

**记住这一句就够了**：左边 Q 图标是"改 QGC"，顶部第二个是"改飞控"。

---

## 3. 你只需要这 8 个页面

按你项目的实际需要排序（★ = 马上就要用）：

| # | 页面 | 英文 | 左侧栏位置 | 你用它干什么 |
|---|---|---|---|---|
| 1 | 飞行视图 | `Fly` | 顶部工具栏 | 看状态、解锁、看模式和电池、看报错 |
| 2 | **执行器** | `Actuators` | 车辆设置 → Motors / Actuators | ★ **测电机顺序和转向**（接线后立刻要用） |
| 3 | **参数** | `Parameters` | 车辆设置 → Parameters | ★ **改所有参数**（最有用，见 §5） |
| 4 | 固件 | `Firmware` | 车辆设置 → Firmware | 刷/升级 PX4 固件（装错版本才用） |
| 5 | 机架 | `Airframe` | 车辆设置 → Airframe | 选机型，决定 `SYS_AUTOSTART`（现在是 4001） |
| 6 | 传感器 | `Sensors` | 车辆设置 → Sensors | 陀螺/加速度/罗盘校准 + 飞控安装方向 |
| 7 | 遥控 | `Radio` | 车辆设置 → Radio | 校准遥控器、看通道映射 |
| 8 | 飞行模式 | `Flight Modes` | 车辆设置 → Flight Modes | 把三段开关映射到飞行模式、配解锁开关和急停 |

其余页面（`Summary`、`Power`、`Safety`、`Tuning`、`Analyze` 里的大部分）**现在可以先不看**。

---

## 4. 关键页面操作路径

### 4.1 飞行视图 `Fly`（你最常看的一页）

屏幕上方那一条状态栏就是全部信息：

| 英文 | 中文含义 | 你现在该看到什么 |
|---|---|---|
| `Ready to Fly` | 可以飞行 | 校准全做完 + 遥控连着 + 没有严重报错 |
| `Not Ready to Fly` | 不可以飞行 | 点它，会展开原因列表 |
| `Manual` / `Stabilized` / `Position` / `Offboard` | 当前飞行模式 | 你拨 CH5 开关时这里会变 |
| `Disarmed` / `Armed` | 未解锁 / 已解锁 | 拨 CH6 开关时会变 |
| `Battery` 百分比 | 电池电量 | 没接电池或没配电源检测时会报错 |
| `GPS` 卫星数 | GPS 状态 | **你不用 GPS，这里长期是 0，正常** |
| `Telemetry` 信号格 | 数传链路质量 | 用 USB 直连时有值 |

**报错去哪看**：右上角会弹出红色提示条，同时左侧 `Analyze` → `Console`（控制台）里有完整日志。**遇到"解锁不了""飞不起来"，第一站永远是 Console。**

### 4.2 执行器 `Actuators`（接线后第一件事）

这是"电机 1234 顺序"的唯一正确配置入口。完整步骤：

1. QGC 连上飞控（USB 线即可）
2. 顶部 → 车辆设置图标 → 左侧栏找 `Motors` 或 `Actuators`
3. **电调协议**选 `DShot300`（BLS / BL32 / 蓝鸟固件都支持；BLS 不支持 DShot600）。
   DShot 是数字协议，**没有"电调校准"这一步**，改完可能要重启飞控才生效
4. **把四个桨全部拆掉**（这一步不商量）
5. 打开页面里的保险开关（`Actuator test` / 滑块区会从灰变亮）——**此时必须接着动力电池**，只插 USB 电机不会转
6. 用左下角的滑块逐个点 1 → 2 → 3 → 4，记下"哪个滑块动的是哪个物理位置的电机"
7. 目标：滑块 1 = **前右**，2 = **后左**，3 = **前左**，4 = **后右**
8. 对不上，就在通道映射里改顺序。（微空官方教程在自家四合一电调上填的是 `3-2-1-4`；不同电调不一样，你要试）
9. **旋向**不在这个页面改。旋向错 = 把那个电机接到电调的三根线中**任意两根对调**，重新测

### 4.3 传感器 `Sensors`

只做三件事，按顺序：

| 页内标签 | 中文 | 做法 |
|---|---|---|
| `Gyroscope` | 陀螺仪零偏校准 | 飞机静止放平，点 `OK`，几秒自动完成 |
| `Accelerometer` | 加速度计（六面校准） | 按提示六个方向各放一次，六个方框全绿就完成 |
| `Compass` | 罗盘校准 | 把飞机拿在手里（离开桌面金属），六个方向旋转 |

同一页里还有个 `Orientations`（方向）子项，就是"飞控安装方向"。**我建议你始终保持飞控箭头朝机头、这里保持 `ROTATION_NONE`**，别引入额外变量。

### 4.4 遥控 `Radio`

1. 遥控器开机、油门拉到最低、对好频
2. 点 `Calibrate`，按提示把所有摇杆和开关都推到最大最小行程
3. 校准完看下面的通道条：拨 CH5 应该看到某个通道在三个值之间跳，CH6 两个值——记下它们是哪个通道号
4. 回 `Flight Modes` 页去映射

### 4.5 飞行模式 `Flight Modes`

| 页面字段 | 中文 | 你的配置 |
|---|---|---|
| `Mode Channel` | 模式切换通道 | CH5（三段开关） |
| 三个模式槽 | 三段各对应什么模式 | 建议 `Stabilized` / `Position` / `Return` |
| `Arm switch channel` | 解锁通道 | CH6（两段开关） |
| `Emergency Kill switch channel` | 急停通道 | **强烈建议配**，一个新通道，拨一下立刻上锁 |

### 4.6 参数 `Parameters`（★ 最重要的一页，也是绕过英文的关键）

**这一页是"不用看懂界面也能干活"的入口**：左上角有个搜索框，直接输参数名，只留匹配的几行。你只需要知道参数名（这我有清单），根本不用理解界面其它文字。

用法：车辆设置 → `Parameters` → 搜索框输入 → 双击 `Value` 列 → 改数值 → 回车 → 右上角 `Tools` → `Save to file` 备份。

**本项目必须改的参数**（都已在计划书里核实）：

| 参数 | 值 | 为什么 |
|---|---|---|
| `SYS_AUTOSTART` | `4001` | 机架类型 = Generic Quad X |
| `UXRCE_DDS_CFG` | TELEM2 | 派走 uXRCE-DDS |
| `SER_TEL2_BAUD` | `921600` | ★ 不改就一个话题都收不到，且不报错 |
| `MAV_1_CONFIG` | `0`(Disabled) | 释放 TELEM2 给 XRCE |
| `COM_RC_OVERRIDE` | `3` | ★ 不改的话进了 offboard 推杆夺不回控制权 |
| `COM_OBL_RC_ACT` | `Position` | offboard 丢遥控时切定点 |
| `COM_OF_LOSS_T` | 默认 1.0 | offboard 断流多久退模式 |
| `EKF2_HGT_REF` | `3`(Vision) | 无 GPS 必须改 |
| `EKF2_EV_CTRL` | 按位 | 外部位姿融合开关，默认全关 |
| `EKF2_EV_DELAY` | 实测值 | 视觉延迟 |
| `RC_INPUT_PROTO` | `SBUS` 或 `Auto` | FS-iA10B 是 SBUS |

**改完务必 `Save to file` 导出一份参数备份**，刷固件时不会丢参数但能对比。

### 4.7 分析工具 `Analyze`（只在出问题时去）

| 工具 | 中文 | 什么时候用 |
|---|---|---|
| `Console` | 控制台 | ★ 看报错，最常用 |
| `MAVLink Inspector` | MAVLink 检查器 | 看某个消息里的具体数值（比如 `SCALED_IMU`、`GLOBAL_POSITION_INT`） |
| `Log Download` | 飞行日志下载 | 需要复盘飞行时 |
| `Mavlink Console` | 飞控命令行 | PX4 的 `nsh` 终端，能跑 `uxrce_dds_client status` 这类命令（★ 验证 DDS 连没连上就靠它） |

**注意**：`Mavlink Console` 里**不要用 `type_text` 之类的方式批量输入**，逐条敲。

---

## 5. 绕过英文的两个实用技巧

1. **参数搜索框**：几乎所有配置最终都是改参数。知道参数名 → 搜 → 改 → 完事。界面其它文字看不懂不影响。
2. **状态栏只记 4 个词**：`Ready to Fly`（能飞）/ `Not Ready to Fly`（不能飞）/ `Armed`（已解锁）/ `Disarmed`（未解锁）。其余忽略。

---

## 6. 常见英文提示的中文含义

这些是你大概率会遇到的原文提示，遇到时按右列处理：

| 屏幕原文 | 中文含义 | 处理 |
|---|---|---|
| `Disarmed by auto preflight disarming` | 解锁后太久没起飞动作，飞控自动上锁 | 解锁后 10 秒内要给起飞动作 |
| `Disarming denied: not landed` | 空中不允许上锁 | 正常行为，落地后再上锁 |
| `Failsafe activated: entering Hold for 5 seconds` | 遥控信号断了，进入 Hold 5 秒 | 检查遥控/接收机接线，之后会自动 RTL |
| `Arming denied: ...` | 解锁被拒，后面跟原因 | 去 Console 看完整原因 |
| `Not enough horizontal position accuracy` | 水平定位精度不够 | 无 GPS 且没外部位姿时的正常现象 |
| `Preflight...` | 起飞前检查未通过 | 按提示逐项修 |
| `Compass inconsistent` / `High magnetic interference` | 罗盘数据不一致 / 磁干扰大 | 远离电机电源线、重新校准 |
| `RC connection lost` | 遥控链路丢失 | 查接收机和 UART6 |
| `No connection` / `Disconnected` | 没连上飞控 | 换数据线、换 USB 口、查串口选择 |

---

## 7. 本项目专用速查

**你现在这个阶段（接线 + 电机）需要按顺序做的 QGC 操作：**

1. 启动 QGC → USB 连飞控 → 确认右上角 `Connected`
2. 车辆设置 → `Airframe` → 确认是 `Generic Quadcopter`（`SYS_AUTOSTART=4001`）
3. 车辆设置 → `Parameters` → 搜 `RC_INPUT_PROTO` 设为 `SBUS`
4. 车辆设置 → `Radio` → 校准遥控器，记下 CH5/CH6 通道号
5. 车辆设置 → `Flight Modes` → 映射飞行模式、解锁开关、急停开关
6. 车辆设置 → `Sensors` → 陀螺仪 / 加速度计 / 罗盘 三项校准
7. 车辆设置 → `Actuators` → `DShot300` → **拆桨** → 开保险开关 → 逐个测 1-4 号电机
8. 对不上的改通道映射；旋向错的对调电机两根线
9. 全部对上后 → 装桨（**桨必须朝下吹风**）→ 自稳模式低空试飞

---

## 8. 连接故障排查（★ 本项目专有架构，必读）

### 8.1 为什么要绕开 QGC 的串口

本机这份 **QGC 5.0.8 有一个实测复现的缺陷**：它打开串口后**独占**（别的程序立刻报 `Access to the path 'COM9' is denied.`），但**一个字节都不解析** —— 跑一小时也不写 tlog，也不下载参数表。

而飞控那一侧是**完全正常**的（实测：独占 COM9 采样 5 秒收到 40,000~68,300 字节、上千个 `0xFD` 帧头、107 种 MAVLink 消息、≈20 KB/s）。

所以本项目的做法是**让 QGC 走 UDP，完全不碰串口**：

```
飞控 ──USB──> COM9 ──[mavlink_usb_udp_bridge.py 桥]──> UDP 127.0.0.1:14550 ──> QGC
```

桥只做**字节搬运**，不解析 MAVLink、不改协议，因此不引入任何协议层风险。

### 8.2 一键启动

双击 **`tools\start_qgc_with_bridge.cmd`**（纯 ASCII，可以直接双击）：

1. 先在**最小化窗口**里启动 `tools\bridge_supervise.cmd`，由它托管桥进程 —— **桥崩了 2 秒后自动重启**，不会出现「QGC 悄悄掉线」；
2. 等 2 秒让桥先占住 COM9；
3. 再启动 QGC。

那个最小化的黑窗口就是**桥的监视窗口**，里面每 5 秒打印一行双向流量：

```
[bridge] 飞控->QGC   110592 B (27 包/5s) | QGC->飞控     195 B (10 包/5s)
```

**关掉这个窗口 = 桥停止 = QGC 掉线。** 要停就先关 QGC 再关它。

### 8.2b 开机自动起桥（不需要管理员）

桥必须**早于 QGC** 拿到 COM9，而重启后一切归零。两种做法：

| 做法 | 是否需要管理员 | 说明 |
|---|---|---|
| ★ **启动文件夹**（已安装） | 不需要 | `%APPDATA%\Microsoft\Windows\Start Menu\Programs\Startup\PX4_QGC_Bridge.cmd`，登录时自动最小化起桥 |
| 计划任务 `/sc onlogon` | **需要**（`schtasks` 会报 `ERROR: Access is denied.`） | 若想改用这个，用管理员 PowerShell 执行 |
| 计划任务 `/sc once`（当前 `PX4_QGC_Bridge`） | 不需要 | **不会自启**，只能 `schtasks /run /tn PX4_QGC_Bridge` 手动拉；重启后是 `Ready` 而不是 `Running` |

> ⚠ **只保留一种方式**。若启动文件夹和计划任务同时生效，第二个实例抢不到 COM9，会在 2 秒重试循环里刷屏。

### 8.3 四条铁律（违反任何一条都会"又连不上"）

| # | 铁律 | 违反后的症状 |
|---|---|---|
| 1 | QGC → 应用程序设置 → 常规：**「Pixhawk」自动连接必须不勾、「UDP」必须勾** | QGC 开机就去抢 COM9 → 独占而不读 → 别的程序全部 `拒绝访问` |
| 2 | **Comm Links 里那条 `FC-USB`（指向 COM9）必须彻底删掉，不是取消勾选** | 它带 `Link0\auto=true`，QGC 一启动就去开 COM9；桥拿不到口 → 桥一直"等待串口" → QGC 收不到任何数据，表现为"连不上"，而 `Comm Links` 里断开重连**完全无效** |
| 3 | **永远不要**在 QGC 的 Comm Links 页面连 COM9（新建也不行） | 直接把桥踢掉，QGC 自己反而读不到数据 |
| 4 | **任何别的程序都不许占 COM9** —— 探针脚本、串口助手、第二个 QGC 实例、Arduino IDE 全算 | 桥的串口被抢 → 桥读异常 → QGC 掉线，而且你看到的是"COM9 拒绝访问"这种误导性报错 |

> ⚠ 已三次踩到铁律 1/2：QGC 会**把 ini 里的设置改回去**。若又连不上，先查
> `%APPDATA%\QGroundControl\QGroundControl.ini`：
> - `autoConnectPixhawk` 必须是 `false`、`autoConnectUDP` 必须是 `true`
> - `[LinkConfigurations]` 里应该是 **`count=0`**，且**不应存在任何 `Link0\` 键**
>
> 改之前先备份成 `QGroundControl.ini.bakN`。改的时候 **QGC 必须是关掉的**，否则它退出时会把 ini 再写回去。
>
> **为什么"断开重连"没用**：那条链路是"串口链路"，而 QGC 在串口上根本不工作；而正常通路是 UDP 自动连接，它不在 Comm Links 列表里，所以在列表里点 Disconnect/Connect 影响不到它。

### 8.4 怎么判断"这一次是真的连上了"

**别信 QGC 顶栏那个会闪烁的 `Connected`。** 三个硬判据：

| 判据 | 命令 / 位置 | 连上的表现 |
|---|---|---|
| ★ 参数表缓存更新时间 | `%APPDATA%\QGroundControl\ParamCache\1_1.v2` 的时间戳 | **刚刚刷新**（QGC 只有成功下载完整参数表才会写这个文件，43~44 KB） |
| 桥的流量 | 桥窗口那一行 | `QGC->飞控` 列**非 0**（有几百字节，说明 QGC 在回话） |
| 参数页 | 车辆设置 → Parameters | 能搜到参数、能改、改完能读回 |

### 8.5 故障对照表

| 现象 | 原因 | 处理 |
|---|---|---|
| 几秒就连上 | 桥先起了、COM9 没被抢 | 正常 |
| 一整晚连不上，QGC 也不报错 | QGC 自己在串口路径上卡死（占口不读） | 用 `start_qgc_with_bridge.cmd` 走 UDP |
| 「COM9 拒绝访问」/ `Access to the path 'COM9' is denied` | **几乎总是 QGC 自己占着**（不是别人）。这是结果不是原因 | 先完全退出 QGC，COM9 立刻可开 |
| 桥窗口刷 `UDP 接收异常 [WinError 10054]` | QGC 还没启动 / 已经关掉，Windows 回了 ICMP 端口不可达 | 正常（新脚本已静默），起 QGC 后自动消失 |
| 桥报 `串口断开（读取异常 ... PermissionError(13, '设备不识别此命令。')）` | 飞控被拔掉、重新枚举，或 COM9 被别的程序抢走 | 桥会自己等；插回飞控 / 关掉抢口程序即可。**协议没断，QGC 不用重启** |
| 连上后飞控数据速率突然变低（如 106→63 KB/5s） | 飞控重启了（重新插拔 / 参数写入后自动重启） | 正常，等它起完 |
| 桥进程凭空消失 | 旧版脚本没有自愈 | 新版由 `bridge_supervise.cmd` 托管，2 秒自动重启 |

### 8.6 为什么"QGC 断开"往往不是飞控的问题

前面几次排查的结论值得记住：**飞控从没出过问题**，每一次"连不上"都能在 Windows 这一侧找到原因（QGC 占口不读、COM9 被抢、USB 重新枚举、脚本没有自愈）。所以再遇到连不上，**先看桥窗口和 `ParamCache` 时间戳**，而不是去重新插拔、重刷固件。

---

## 9. 执行器页变空壳 / 电机测试报"命令不支持"（★ 2026-11-08 实测定案）

这是本项目踩过的最隐蔽的一个坑：**飞控一切正常、参数能改、能解锁能飞，唯独 Actuators 页只剩一小块"电机 config"，没有每路输出的滑块，点电机测试还报 `MAV_CMD_DO_MOTOR_TEST` 命令不支持。**

### 9.1 症状

| 现象 | 细节 |
|---|---|
| Actuators 页只剩"电机 config" | 有几何/转向的下拉框，**没有每路输出的滑块和 Enable 开关** |
| 点电机测试报错 | `MAV_CMD_DO_MOTOR_TEST` 命令不支持 |
| 其它一切正常 | 参数页能读写、能解锁、能飞、光流测距都正常 |

### 9.2 根因

**`MAV_PROTO_VER` 被设成了 1（Always use version 1）。**

PX4 在 MAVLink v1 通道上**不再发送 `COMPONENT_INFORMATION` 报文**（就是"组件信息/执行器元数据"那套东西）。QGC 拿不到执行器元数据，就建不出新的执行器界面，于是**退回它内置的旧版电机测试通道**——而旧通道用的 `MAV_CMD_DO_MOTOR_TEST`（209）**在 PX4 1.17 里已经被删掉了**（只保留 `MAV_CMD_ACTUATOR_TEST`，310）。所以旧界面也点不动。

把 `MAV_PROTO_VER` 改回 **2** 并重启飞控，界面立刻恢复。

### 9.3 证据（当时怎么确认的，供日后复核）

| 证据 | 数据 |
|---|---|
| tlog 报文统计 | `MAV_PROTO_VER=1` 期间，**10 个会话全部**：`COMPONENT_INFORMATION` 收到 0 条、MAVFTP(`FILE_TRANSFER_PROTOCOL`) 0 次 |
| 请求有回、数据没发 | QGC 发 `MAV_CMD_REQUEST_MESSAGE`(512) 请求 msgid 395/397，飞控回 `COMMAND_ACK result=0 (ACCEPTED)`，**但从不发对应消息** |
| 电机测试命令变了 | 09:37（PROTO_VER=2）：`MAV_CMD_ACTUATOR_TEST`(310) ×233 次，全部 ACCEPTED；20:20（PROTO_VER=1）：`MAV_CMD_DO_MOTOR_TEST`(209) ×1，返回 `result=3 UNSUPPORTED` |
| 元数据缓存时间戳（最硬的证据） | `QGCCompInfoCache\*.meta` 四个类型停在 `11:21:22–11:21:25`；改回 2 后**同一分钟内全部刷新到 20:43:18–20:43:22** |
| 排除"参数没下完" | `%APPDATA%\QGroundControl\ParamCache\1_1.v2` 当天 20:20:44 就已写好（44,360 B），参数早就就绪 |

### 9.4 修法（1 分钟）

1. QGC → **Vehicle Setup → Parameters**，搜索框输 `MAV_PROTO_VER`，双击改成 **2（Always use version 2）**
2. **Tools → Reboot Vehicle**，等 30 秒让飞控重启完
3. **完全退出 QGC，再重新打开**，连上后**等满 60 秒**（元数据是连上后异步下载的）
4. 再进 Actuators 页

> ⚠ 改完一定要**重启飞控**。`MAV_PROTO_VER` 走的是 mavlink 实例启动路径，不重启不生效。

### 9.5 ★ 不要为了 MTF-01 把 `MAV_PROTO_VER` 降到 1

MicoAir 官方文档里写着「PX4 1.17.x 起需要在微空助手里把协议设成 `MAVLink (PX4)`，并设置 `MAV_PROTO_VER = 1`」。**后半句只对 `AUTO`（自动识别）模式有意义。**

本项目实测：在微空助手里**显式选了 `MAVLink (PX4)`** 之后，`MAV_PROTO_VER=2` 时光流和测距完全正常——

- PROTO_VER=1 期间：`DISTANCE_SENSOR` 5883 条 / `OPTICAL_FLOW_RAD` 5883 条
- PROTO_VER=2 期间：同样都有数据

⇒ **PX4 解析 `OPTICAL_FLOW_RAD` / `DISTANCE_SENSOR` 与 MAVLink 版本无关，只与"传感器是否真的在发 MAVLink"有关。** 为了让光流工作而把全链路降到 v1，代价是丢掉整个组件信息通道（执行器界面、事件元数据等），不值得。

### 9.6 怎么判断"元数据这次到底下来没有"

不用看界面，直接看文件时间戳：

```powershell
Get-ChildItem "$env:LOCALAPPDATA\QGroundControl\QGroundControl\cache\QGCCompInfoCache" -Filter "*.meta" |
  Sort-Object LastWriteTime -Descending | Select-Object -First 4 LastWriteTime, Name
```

四个 `.meta` 分别是 `_00_`(GENERAL) / `_01_`(PARAMETER) / `_04_`(EVENTS) / `_05_`(ACTUATORS)。**它们在你连上后的几十秒内一起刷新 = 整条组件信息链路通了**；如果都停在上一次的时间，就是没通。

### 9.7 界面救不回来时，功能完全等价的命令行

QGC 的 `Analyze → MAVLink Console` 里（**必须拆桨、必须未解锁**）：

```
param show PWM_MAIN_FUNC*              # 1~4 号输出分别接了什么（本项目应为 101/102/103/104）
param set PWM_MAIN_FUNC1 103           # 例：把 MAIN1 改成 3 号电机
param save                             # ★ 必须，否则重启就丢
actuator_test set -m 1 -v 0.1 -t 2     # 1 号电机 10% 油门转 2 秒
actuator_test iterate-motors           # 依次把每个电机点一遍
actuator_test stop                     # 立即停
listener actuator_outputs              # 看四路实际输出
```

`actuator_test` 的 `-v` 取 −1..1（负值反转，需要 `CA_R_REV` 里那一路是 3D 模式），`-t` 不填则一直转到你按回车。这套命令和界面滑块走的是同一条 `actuator_test` uORB 通道，效果完全一样，只是要点键盘。

---

## 附录 A：为什么这份 QGC 没有中文

核查过程与结果（供日后复核）：

| 检查项 | 结果 |
|---|---|
| `translations\` 目录 | 31 个 `.qm`，**全部为 `qt_*.qm`**（Qt 框架自带的控件翻译） |
| 是否存在 `qgc_zh_CN.qm` | **不存在**（`qgc*` 通配符零命中） |
| 可执行文件中是否内嵌中文 | UTF-16 搜 `简体中文` / `中文语言` / `Language` 均未命中 |
| 可执行文件中 `qgc_` 相关串 | 只有 `qgc_source_` / `qgc_json_` / `qgc_translation_file_decompressed.ts` / `qgc_translated_metadata.json` / gstreamer 插件名，**无 `.qm` 资源名** |
| 结论 | QGC 主界面（飞行视图/车辆设置/参数/执行器）**只能英文**。语言下拉框至多影响 Qt 标准对话框 |

> 这不是你装错了，是这份构建**没有打包 QGC 自己的中文翻译**。想彻底汉化只有两条路：①换一个带 `qgc_zh_CN.qm` 的官方安装版；②自己编译。两者都要重新下载/编译，**投入产出比很差**——用本手册的对照法两三次就熟了。

---

## 附录 B：一句话记住 QGC 在系统里的位置

**QGC 只是录入和显示工具。参数真正存在飞控的参数区里，QGC 关掉不影响飞控。** 你用 QGC 改的每一个参数，最终都写进了飞控的 flash；你用 QGC 看到的每一个数值，都是飞控通过 MAVLink 报上来的。所以「QGC 里看到的」不等于「真实的控制逻辑」——真正的控制在飞控固件里跑。
