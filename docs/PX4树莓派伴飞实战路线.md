# PX4 + 树莓派 5 伴飞实战路线（真机，不用 SITL）

> 目标：树莓派跑机载软件栈（雷达/定位/SLAM/无线通信），实时向 PX4 飞控输出控制指令；
> 飞控保留 1 kHz 内环稳定，派专注 10~50 Hz 的任务/外环。
> 与既有资产的衔接见第 11 节。
>
> 📌 **本文档定位 = 手册**（参数逐条、命令逐条、硬件坑逐条，需要抄命令时翻这里）。
> 决策与施工顺序请看配套的 **`PX4伴飞工程计划书.md`** —— 那里有思维梳理、坐标系换算（已数值验证）、
> 里程碑 M0~M6、试验矩阵和风险登记册。两份文档冲突时以计划书为准。

---

## 0. 先把架构摆正：PX4 固件不跑在树莓派上

你说"在树莓派中跑一套 PX4 系统，实时控制输出数据给 PX4 飞控"——这句话需要拆成两半：

| 谁    | 跑什么                                                       | 为什么                                                     |
| ----- | ------------------------------------------------------------ | ---------------------------------------------------------- |
| 飞控  | **PX4 固件**（STM32H743，NuttX RTOS，速率环 1 kHz、EKF2 400 Hz） | 硬实时、掉电不丢、看门狗、失控保护。Linux 做不到           |
| 树莓派 | **PX4 伴飞栈**：ROS 2 + XRCE-DDS Agent + 你自己的节点        | 算力、内存、USB/CSI/以太网接口、跑 SLAM/雷达/神经网络      |

**关键认知：派从不直接写电机输出。** 派发的是**设定点**（位置 / 速度 / 姿态 / 推力目标），
飞控收到后由它自己的实时控制器解算成四路 PWM/DShot。中间这条链路断了，飞控按 `COM_OF_LOSS_T`
自动退出 offboard 并执行 failsafe——这是你的安全底线，也是派可以"死"而飞机不会掉的原因。

"实时控制输出数据"在 PX4 术语里就叫 **offboard 模式**（外部控制模式）。协议桥用 **uXRCE-DDS**
（把飞控的 uORB topic 直接映射成 ROS 2 topic），不用再手搓 MAVLink 打包。

### 0.1 你缺的那一环：三样东西的关系

很多人的困惑是"派该**通过什么系统**把数据发给飞控"。答案是一条**三层**的链路，
每一层解决不同的问题：

```
① ROS 2 topic        ← 派这一侧，你的代码直接打交道的东西
        ↕
② uXRCE-DDS          ← 中间的翻译官（派上的 Agent + 飞控里的 client）
        ↕
③ uORB topic         ← 飞控这一侧，PX4 内部的东西
```

**关键认知：①和③其实是同一套"发布/订阅"模式的两个副本。**

| | 飞控内部 | 派这一侧 |
| --- | --- | --- |
| 叫什么 | **uORB** | **ROS 2 topic** |
| 话题长什么样 | `trajectory_setpoint` | `/fmu/in/trajectory_setpoint` |
| 谁在发/收 | PX4 的各个模块 | 你的 ROS 2 节点 |
| 消息定义来自 | PX4 源码的 `.msg` | `px4_msgs` 这个 ROS 2 包 |
| 中间谁翻译 | ← **uXRCE-DDS** → | |

- `/fmu/in/*` = **派 → 飞控**（飞控"收进来"）
- `/fmu/out/*` = **飞控 → 派**（飞控"发出去"）

> `in` / `out` 是**站在飞控的角度**说的。记住这一点就不会搞反方向。

所以"派通过什么系统发送"的答案是：**派用 ROS 2 发布话题，话题名以 `/fmu/in/` 开头。**
不需要你手搓 MAVLink 打包，uXRCE-DDS 把这件事包了。

### 0.2 一条指令的完整旅程（★ 最该记住的一张表）

你在派上写这么一行（伪代码）：

```python
node.publish("/fmu/in/trajectory_setpoint", position=[0, 0, -2])
```

意思是"飞到原点正上方 2 米"。这句意图接下来会经过这九步：

| 步 | 在哪 | 发生什么 |
| --- | --- | --- |
| 1 | 派 | 你的节点把消息发到 ROS 2 话题 `/fmu/in/trajectory_setpoint` |
| 2 | 派 | **MicroXRCEAgent**（一个后台程序）收到，序列化成字节流，套上 XRCE-DDS 协议头 |
| 3 | 线缆 | 字节从派串口出去（921600 波特率，TX/RX/GND 三根线） |
| 4 | 飞控 | `uxrce_dds_client` 模块收到、解包，**发布成 uORB 话题** `trajectory_setpoint` |
| 5 | 飞控 | **`commander` 守门**：现在是 offboard 模式吗？有位置估计吗？不满足就**直接丢掉** |
| 6 | 飞控 | `mc_pos_control` 读设定点 + `vehicle_local_position`，算出**姿态目标** |
| 7 | 飞控 | `mc_att_control` 把姿态目标算成**角速率目标** |
| 8 | 飞控 | `mc_rate_control` 以 **1 kHz** 算出力矩 |
| 9 | 飞控 | `control_allocator` 混控成四路 PWM/DShot → 电调 → 电机 |

**最容易忽略的是第 4、5 步：**

- **第 4 步**：`uxrce_dds_client` 只是个"投递员"，它把消息丢进 uORB 就不管了。
  你的指令能不能生效，由后面的模块说了算。
- **第 5 步**：`commander` 是**守门人**，条件不满足时**静默丢弃**——
  你只会看到飞机不动，**没有任何报错**。这是"offboard 切了没反应"的绝大多数原因。

> 请记住这一句：**你在派上发消息 ≠ 飞控会执行。飞控有完全的否决权。**

### 0.3 PX4 内部是怎么运作的

PX4 不是一个"大程序"，而是**一堆互相不认识的小程序（模块），
全靠 uORB 这个内部广播站对话**。

```
传感器层   sensors ─────► sensor_combined（原始 IMU 数据）
                              │
估计层     ekf2 ──────────────┴──► vehicle_attitude
                                  vehicle_local_position      ▲
                                  vehicle_odometry            │
                                                              │ 你喂的位姿
决策层     commander ◄── 模式状态机 / 解锁 / failsafe         │ 从这里进来
           navigator ◄── 任务 / RTL / 降落                    │
                                                              │
控制层     mc_pos_control ◄── 设定点 + 位置估计 ──────────────┘
                │  50~100 Hz
           mc_att_control
                │  250~1000 Hz
           mc_rate_control
                │  1 kHz   ★ 硬实时，只有飞控做得到
           control_allocator
                │
输出层     PWM / DShot ──► 电调 ──► 电机
```

| 层 | 典型频率 | 谁执行 | 为什么是这个频率 |
| --- | --- | --- | --- |
| 传感器采集 | 1 kHz+ | 飞控 | 硬件中断驱动 |
| EKF2 状态估计 | 100~400 Hz | 飞控 | |
| **位置环** | 50~100 Hz | 飞控 | ← 派能参与的最上面一层 |
| **姿态环** | 250~1000 Hz | **飞控** | 派抖动几十毫秒 → 姿态跟着抖 |
| **速率环** | **1 kHz** | **只有飞控** | 硬实时，Linux 绝对做不到 |

> 只需要记住一句：**派只碰最上面那一层（位置目标 / 速度目标），下面三层永远归飞控。**

**为什么派做不了速率环？** 派跑的是 Linux，不是实时系统。
你的 SLAM 线程、WiFi 中断、内存回收随时可能抢走 CPU 几十毫秒。
速率环要的是"每 1 毫秒准时"，抖一次电机就会响一下，抖多了就是炸机。
飞控上的 NuttX 是 RTOS，保证这件事。

### 0.4 一次真实飞行的完整时序

把上面两层串起来，你就明白 §7 那个 offboard 时序**为什么长那样**：

```
派                                          飞控
│ ① 持续以 ≥2 Hz 发 OffboardControlMode ────►  这是"心跳"，证明派还活着
│                                    commander: 心跳够了 → 允许进 offboard
│ ② 发 DO_SET_MODE(offboard) ───────────────►
│ ③ 持续发 TrajectorySetpoint ──────────────►  mc_pos_control → … → 电机
│ ④ 持续读 /fmu/out/vehicle_local_position ◄─  我在哪（闭环的反馈）

✗ 派崩了 / 串口断了
│                                    commander: 超过 COM_OF_LOSS_T 没心跳
│                                    → 退出 offboard
│                                    → 执行 COM_OBL_RC_ACT（悬停/降落/返航）
```

**为什么心跳必须是 `OffboardControlMode`，不能拿 `TrajectorySetpoint` 顶替？**
因为飞控要能区分"派在正常发指令"和"派死机了、但最后一条消息还躺在总线上"。
心跳是一个专门的、只用来证明"我还活着"的信号，和"往哪飞"解耦。

**为什么派必须能读到 `vehicle_local_position`？**
因为位置环的本质就是 `误差 = 目标 − 当前`。
你只给"目标"不给"当前"，飞控手里没有反馈量可以减。
这就是 §8.2「位姿必须回灌 EKF2」的由来。

### 0.5 关于"在 Linux 开发板上装 PX4 系统"（地瓜派那件事）

先纠正一个说法：**PX4 不是操作系统，它就是一套飞控固件（一个程序）。**

- 在飞控上，PX4 跑在 **NuttX** 这个 RTOS 之上——**NuttX 才是操作系统**。
- PX4 同时还能编译成 **Linux 上的普通进程**（叫 posix 平台，SITL 用的就是它），
  所以"装在地瓜派这类 Linux 开发板上"这件事技术上成立，你同伴没做错，只是用途可能不同。

你同伴做的，基本是下面两种之一：

| 可能 | 是什么 | 能真飞吗 |
| --- | --- | --- |
| (a) PX4 **SITL** | 在 Linux 上跑的**仿真**，用来开发调试，不输出真实 PWM | ❌ 不能。就是你说"不太需要"的那个 |
| (b) PX4-on-Linux **真机** | PX4 当飞控，靠 Navio2/PilotPi 这类 HAT、或一块刷了 I/O 固件的板子输出 PWM | ⚠️ 能，但 Linux 非硬实时，1 kHz 速率环会抖 |

> **建议你直接问他一句**：「你那块板子上跑的是 **SITL**，还是接了 **PWM 输出板**？」
> 一句话就能分清，也能避免你们俩后续讨论时鸡同鸭讲。

**但不管你同伴怎么做，你的方案更适合你——因为你已经有那个关键硬件了。**

### 0.6 两种"PX4 上派"的路子（只推荐第一种）

| 路线 | 是什么 | 结论 |
| --- | --- | --- |
| **A 伴飞电脑（推荐）** | 飞控跑 PX4，派跑 ROS 2 + XRCE Agent，两者串口相连 | 你已有 H743 飞控，直接走这条。官方主推、资料最多、电赛标配 |
| B PX4-on-Linux（`px4_raspberrypi_default`） | 派**自己当飞控**，PX4 作为普通 Linux 进程跑，再靠 PWM 板输出 | 需要 Navio2/PilotPi 类 HAT；Linux 非硬实时；**你已有 H743，别走这条** |

**路线 A 为什么更好，具体好在哪：**

| 保障 | 路线 A（你） | 路线 B（派当飞控） |
| --- | --- | --- |
| 速率环实时性 | NuttX 硬实时 | Linux，会被抢占 |
| 派崩溃/重启 | 飞控自己稳悬停，最多触发 failsafe | **直接掉** |
| 独立供电与看门狗 | 有 | 无 |
| 失控保护 / 遥控接管 | PX4 完整实现 | 要自己搭 |

> 结论：**PX4 = 飞控固件；树莓派 = 伴飞电脑。** 本文全部按路线 A 展开。

---

## 1. 目标系统全景

```
        ┌──────────── 树莓派 5（Ubuntu 24.04 / ROS 2 Jazzy） ──────┐
        │ 雷达/激光 → SLAM 节点(10~30Hz) → 外环 PID/任务状态机      │
        │                              ↓ 50 Hz                     │
        │                    /fmu/in/trajectory_setpoint           │
        │        /fmu/in/offboard_control_mode   /fmu/in/vehicle_command
        │                              ↑ /fmu/out/vehicle_local_position
        │                     micro-XRCE-DDS Agent                 │
        └───────────────────────────┬──────────────────────────────┘
                                    │ UART 921600 (TX/RX/GND 交叉)
        ┌───────────────────────────┴──────────────────────────────┐
        │ MicoAir743v2 飞控 (PX4 1.17)                             │
        │ uxrce_dds_client → uORB → commander → 位置/姿态控制器      │
        │ → 混控 → 1 kHz 速率环 → PWM/DShot → 电调 → 电机           │
        └──────────────────────────────────────────────────────────┘
```

**分工铁律：慢环在派，快环在飞控。** 派抖动 50 ms 无所谓（飞控有外推），
飞控掉一拍电机就会抖。派上不要试图跑速率环。

---

## 2. 硬件清单与四个必踩的坑

### 清单

| 项             | 说明                                                             | 必需 |
| -------------- | ---------------------------------------------------------------- | ---- |
| MicoAir743v2   | 已有（Board ID 1179，`micoair_h743-v2`）                          | ✅   |
| 树莓派 5       | 建议 8 GB；SD 卡换成 NVMe/SSD（日志写爆 SD 卡是常态）             | ✅   |
| 独立 5V/5A BEC | **给派单独供电**，从电池直取。别用飞控的 5V BEC                   | ✅   |
| 3 根杜邦线     | TX/RX/GND 交叉                                                    | ✅   |
| 数传/图传      | 飞控 TELEM1 留给自己；派用 WiFi/4G 回传 QGC                       | 建议 |
| 光流/测距      | **室内 offboard 的硬前提**，不是可选件（见 M3 前置），`MTF-01`/`MT-06` 之类 | ✅   |

### 坑 1：飞控的 5V BEC 带不动树莓派 5

MicoAir743v2 的 5V BEC 上限 **3 A**，而且要和接收机、GPS、光流共享。
树莓派 5 官方要求 **5V/5A（25 W）**，3 A 只能降级运行还会掉 USB 限流警告，
电机启动瞬间的电压跌落直接让派重启（重启期间 offboard 断流 → 飞控 failsafe → 炸机）。
**必须给派配独立 BEC。**

### 坑 2：必须共地

飞控和派用两个 BEC 供电时，**GND 必须连在一起**，否则 UART 电平没有共同参考，通信随机出错。
TX↔RX 交叉、GND↔GND 直连，这是三根线的全部。

### 坑 3：3.3V 电平，别接 5V

H743 的 UART 和树莓派 GPIO 都是 **3.3V 电平**，可以直连，**不需要电平转换模块**。
但不要把飞控 TELEM 口的 5V 针接到派的 GPIO（派 GPIO 不容忍 5V 输入）。

### 坑 4：UART8 被板载蓝牙占了

见第 3 节串口映射表——`TELEM4`（UART8）内部接了蓝牙数传，默认 115200。
**给派选 TELEM1 或 TELEM2。**

---

## 3. 飞控侧配置（MicoAir743v2 / PX4 1.17）

### 3.1 串口映射（官方手册，[来源](https://micoair.cn/zh/docs/flight-controller/micoair743/micoair743v2-flight-controller-manual)）

| PX4 设备 | PX4 接口名          | 物理接口 | 建议用途                     |
| -------- | ------------------- | -------- | ---------------------------- |
| `ttyACM0`| USB                 | USB-C    | 调试 / QGC                   |
| `ttyS0`  | `TELEM1`            | UART1    | 数传电台 → QGC               |
| `ttyS1`  | `GPS2`              | UART2    | DisplayPort（图传）          |
| `ttyS2`  | `GPS1`              | UART3    | GPS（你现在没装）            |
| **`ttyS3`** | **`TELEM2`**     | **UART4**| **★ 接树莓派**               |
| `ttyS4`  | `TELEM3`            | UART5    | 备用                         |
| `ttyS5`  | `RC`                | UART6    | FS-iA10B SBUS（已占用）      |
| `ttyS6`  | `URT6`              | UART7    | 电调遥测                     |
| `ttyS7`  | `TELEM4`/`SERIAL4`  | UART8    | ⚠ 与板载蓝牙共用，别接派     |

### 3.2 参数清单（QGC → Parameters）

| 参数              | 建议值                | 作用                                                                 |
| ----------------- | --------------------- | -------------------------------------------------------------------- |
| `UXRCE_DDS_CFG`   | `TELEM2`              | 在该串口启用 uXRCE-DDS **client**（枚举值以 QGC 下拉框为准，别背数字） |
| `SER_TEL2_BAUD`   | `921600`              | TELEM2 波特率，必须和派端 Agent 的 `-b` 一致                          |
| `MAV_1_CONFIG`    | **`0`**               | ⚠ 默认可用的 MAV_1 就占着 TELEM2 跑 MAVLink，**必须清零**才能腾给 XRCE |
| `MAV_0_CONFIG`    | `TELEM1`（保持默认）  | 数传/QGC 继续走 TELEM1                                                |
| `UXRCE_DDS_PRT`   | `8888`（默认）        | Agent 的 UDP 端口（仅以太网方案用）                                   |
| `UXRCE_DDS_AG_IP` | `2130706433`（默认）  | = `127.0.0.1`。只吃十进制 int32，**不支持点分格式**（仅以太网方案用）  |
| `UXRCE_DDS_SYNCT` | `1`（默认）           | 客户端/Agent 时间戳同步，**别关**                                     |
| `UXRCE_DDS_KEY`   | `1`（默认）           | 多机共用一个 Agent 时每机必须唯一非零                                 |
| `COM_RC_OVERRIDE` | **显式启用（含 offboard）** | ⚠ **offboard 下默认不生效**，不改就打杆夺不回控制权（见 M3）      |
| `COM_RC_STICK_OV` | 默认                  | 触发夺权的杆量阈值                                                    |
| `COM_OF_LOSS_T`   | `0.5`（默认）         | 断流多久退出 offboard                                                 |
| `COM_OBL_RC_ACT`  | `0` = Position        | 退出 offboard 后进哪个模式：0 Position / 1 Altitude / 2 Manual / 3 Return / 4 Land |

> **更省事的替代方案**：把 XRCE 放到 **TELEM3**（UART5，`/dev/ttyS4`）。
> TELEM3 默认没有 MAVLink 占用，只需 `UXRCE_DDS_CFG=TELEM3` + `SER_TEL3_BAUD=921600`，
> **一个 MAVLink 参数都不用动**。新手更推荐这条，少一个出错点。

### 3.3 飞控侧验收（不接派，先自己过一遍）

官方 PX4 v1.17.0 的 `boards/micoair/h743-v2/default.px4board` 里明确写着
`CONFIG_MODULES_UXRCE_DDS_CLIENT=y` —— 也就是说
**官方 release 固件自带 XRCE 客户端，不需要自己编译**（除非你改了消息定义）。

桌面、**无桨**、USB 供电即可验证：USB 连 QGC → MAVLink Console 敲
`uxrce_dds_client status`，有回显即 OK。

---

## 4. 树莓派侧配置

### 4.1 操作系统怎么选（决定 ROS 2 版本）

| 系统                          | 配 ROS 2      | 说明                                                      |
| ----------------------------- | ------------- | --------------------------------------------------------- |
| **Ubuntu Server 24.04 arm64** | **Jazzy**     | ★ 推荐：ROS 2 官方 apt 包齐全，PX4 支持良好              |
| Ubuntu Server 22.04 arm64     | Humble        | 生态最稳，但 Pi 5 上要折腾内核/固件                      |
| Raspberry Pi OS Bookworm      | 需源码编译    | 不推荐给 ROS 2 新手                                       |

**结论：刷 Ubuntu Server 24.04 (64-bit) for Raspberry Pi，装 ROS 2 Jazzy。**

> ✅ **本项目已定：Ubuntu + ROS 2 Jazzy**（树莓派 5 已就绪）。
> 因此 **Micro-XRCE-DDS-Agent 装 v2.4.3**（见 §5），**不要装 v3.x**——
> PX4 1.17 的 XRCE 客户端基于 v2.x，装错大版本会"连上了但建不了 topic"，极难查。

> **但这条结论要跟雷达驱动对一次账。** 镭神 N10 的官方教程是在
> **Ubuntu 22.04 + ROS 2 Humble** 上写的（见 8.10），驱动是 C++ + `pcl_conversions`，
> 在 Jazzy 上**大概率能编**，但官方没验证过。
> - **先照 8.10 把 `colcon build` 跑通再往下做**（几分钟的事，能省掉后面返工）。
> - 万一 Jazzy 编不过，退到 Humble + Ubuntu 22.04 即可，
>   整条路线其余部分不受影响（XRCE Agent 换成 v2.4.2）。
> - **系统版本定错了，后面所有东西都要重来**，所以这一步优先验证。

**版本对照速查（三个东西必须对齐）**：

| 你的选择 | ROS 2 | Micro-XRCE-DDS-Agent | 镭神 N10 驱动 |
| --- | --- | --- | --- |
| **Ubuntu 24.04（本项目）** | **Jazzy** | **v2.4.3** | 官方未验证，预计可行 |
| Ubuntu 22.04 | Humble | **v2.4.2** | 官方教程环境 ✅ |

### 4.2 系统准备

```bash
sudo apt update && sudo apt full-upgrade -y
sudo apt install -y python3-pip git cmake build-essential
# 串口权限：把当前用户加进 dialout，重新登录生效
sudo usermod -aG dialout $USER
```

**串口要做的三件事**（`sudo raspi-config` → Interface → Serial Port，
或手改 `/boot/firmware/config.txt`）：

1. **关闭串口登录控制台** (`console=serial0` 从 cmdline.txt 移除)，否则内核会占用串口。
2. **启用串口硬件** (`enable_uart=1`)。
3. **关掉蓝牙、释放 PL011**：`dtoverlay=disable-bt`。
   Pi 的 mini-UART 波特率随 CPU 频率漂移，921600 下必错；要的是 PL011 那条。

**设备名陷阱**：不同系统上可能是 `/dev/ttyAMA0`、`/dev/ttyS0` 或 `/dev/serial0`。
**不要硬编码**，用 udev 规则固定一个别名：

```bash
ls -l /dev/serial* /dev/ttyAMA*        # 先看清哪个是 PL011
```

建议加 `/etc/udev/rules.d/99-fc.rules` 把飞控串口软链成 `/dev/fc`，代码里只写 `/dev/fc`。
（回环自测：短接 GPIO14/15，`python3 -c` 发一串收一串，收到即串口通了。）

### 4.3 其他必做

- **关 WiFi 省电**：`sudo iw dev wlan0 set power_save off`，否则无线回传会周期性卡顿。
- **日志别写 SD 卡**：挂 NVMe/SSD，或把 rosbag 写到外置盘。
- **实时性**：50 Hz offboard 用标准内核就够，**不需要** PREEMPT_RT。
  真要做就把 SLAM 线程绑到大核、别让它在小核上被抢。

---

## 5. 里程碑 M1：链路打通（最先做，最有成就感）

1. **接线**：飞控 TELEM2 的 `TX→派 RX(GPIO15)`、`RX→派 TX(GPIO14)`、`GND→GND`。
   共地是硬要求。
2. **飞控端**：`UXRCE_DDS_CFG=TELEM2`、`SER_TEL2_BAUD=921600`、
   **`MAV_1_CONFIG=0`**，然后重启飞控。三条缺一不可，尤其最后一条——
   MAV_1 默认就占着 TELEM2 跑 MAVLink，不清零 XRCE 起不来。

3. **派端编译安装 Agent**：

```bash
git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && mkdir build && cd build
cmake .. && make -j4
sudo make install && sudo ldconfig /usr/local/lib/
```

   > ⚠ **版本是硬约束**：PX4 1.17 的 XRCE 客户端基于 **v2.x**，
   > **与最新的 v3.x Agent 不兼容**（v3.x 连上来了但建不出 topic）。
   > ROS 2 **Jazzy → Agent v2.4.3**；**Humble → Agent v2.4.2**。

4. **启动 Agent**（串口模式，`-b` 必须等于 `SER_TEL2_BAUD`）：

```bash
sudo MicroXRCEAgent serial --dev /dev/fc -b 921600
```

   > 官方文档里写的 `/dev/AMA0` 是笔误，实际应为 `/dev/ttyAMA0`。
   > 你自己的设备名以第 4.2 节实测为准（建议用 udev 软链成 `/dev/fc`）。

5. **验收**：

```bash
ros2 topic list | grep fmu
# 期望看到 /fmu/in/... 和 /fmu/out/...
ros2 topic echo /fmu/out/vehicle_status --once
ros2 topic hz /fmu/out/vehicle_local_position     # 应有稳定频率
```

看到 `fmu` 开头的 topic 并且有数据 → **M1 完成**，你已经把派和飞控接通了。
把 Agent 做成 systemd 服务（`Type=simple` + `Restart=always`），开机自启。

---

### 5.1 派到底能读到什么（`dds_topics.yaml` @ v1.17.0 全量核实）

**这是最容易白费一整天的地方，先看这里再写代码。**

XRCE-DDS 不是"所有 uORB 话题都对派开放"，而是由飞控源码里的白名单
`src/modules/uxrce_dds_client/dds_topics.yaml` 决定的。
**不在表里的话题，派上根本不会出现**，`ros2 topic list` 里找不到，
订阅代码写得再对也收不到——而且不会有任何报错提示。

**飞控 → 派（`publications` 段，即 `/fmu/out/*`）——全部 27 条：**

| ROS 2 topic | 类型 | rate_limit |
| --- | --- | --- |
| `/fmu/out/vehicle_local_position` | VehicleLocalPosition | **50** |
| `/fmu/out/vehicle_global_position` | VehicleGlobalPosition | 50 |
| `/fmu/out/vehicle_gps_position` | SensorGps | 50 |
| `/fmu/out/vehicle_odometry` | VehicleOdometry | — |
| `/fmu/out/vehicle_attitude` | VehicleAttitude | — |
| `/fmu/out/vehicle_status` | VehicleStatus | 5 |
| `/fmu/out/vehicle_control_mode` | VehicleControlMode | 50 |
| `/fmu/out/vehicle_command_ack` | VehicleCommandAck | — |
| `/fmu/out/vehicle_land_detected` | VehicleLandDetected | 5 |
| `/fmu/out/sensor_combined` | SensorCombined | — |
| `/fmu/out/battery_status` | BatteryStatus | 1 |
| `/fmu/out/home_position` | HomePosition | 5 |
| `/fmu/out/estimator_status_flags` | EstimatorStatusFlags | 5 |
| `/fmu/out/failsafe_flags` | FailsafeFlags | 5 |
| `/fmu/out/manual_control_setpoint` | ManualControlSetpoint | 25 |
| `/fmu/out/position_setpoint_triplet` | PositionSetpointTriplet | 5 |
| `/fmu/out/mode_completed` | ModeCompleted | 50 |
| `/fmu/out/arming_check_request` | ArmingCheckRequest | 5 |
| `/fmu/out/collision_constraints` | CollisionConstraints | 50 |
| `/fmu/out/timesync_status` | TimesyncStatus | 10 |
| `/fmu/out/wind` | Wind | 1 |
| `/fmu/out/airspeed_validated` | AirspeedValidated | 50 |
| `/fmu/out/vtol_vehicle_status` | VtolVehicleStatus | — |
| `/fmu/out/transponder_report` | TransponderReport | — |
| `/fmu/out/gimbal_device_attitude_status` | GimbalDeviceAttitudeStatus | 20 |
| `/fmu/out/register_ext_component_reply` | RegisterExtComponentReply | — |
| `/fmu/out/message_format_response` | MessageFormatResponse | — |

**派 → 飞控（`subscriptions` 段，即 `/fmu/in/*`）——与多旋翼相关的：**

| ROS 2 topic | 类型 |
| --- | --- |
| `/fmu/in/offboard_control_mode` | OffboardControlMode |
| `/fmu/in/trajectory_setpoint` | TrajectorySetpoint |
| `/fmu/in/vehicle_command` | VehicleCommand |
| `/fmu/in/vehicle_command_mode_executor` | VehicleCommand |
| `/fmu/in/vehicle_visual_odometry` | VehicleOdometry |
| `/fmu/in/vehicle_mocap_odometry` | VehicleOdometry |
| `/fmu/in/vehicle_attitude_setpoint` | VehicleAttitudeSetpoint |
| `/fmu/in/vehicle_rates_setpoint` | VehicleRatesSetpoint |
| `/fmu/in/vehicle_thrust_setpoint` | VehicleThrustSetpoint |
| `/fmu/in/vehicle_torque_setpoint` | VehicleTorqueSetpoint |
| `/fmu/in/distance_sensor` | DistanceSensor |
| `/fmu/in/sensor_optical_flow` | SensorOpticalFlow |
| `/fmu/in/obstacle_distance` | ObstacleDistance |
| `/fmu/in/onboard_computer_status` | OnboardComputerStatus |
| `/fmu/in/telemetry_status` | TelemetryStatus |
| `/fmu/in/goto_setpoint` | GotoSetpoint |
| `/fmu/in/manual_control_input` | ManualControlSetpoint |
| `/fmu/in/aux_global_position` | VehicleGlobalPosition |
| `/fmu/in/config_control_setpoints` | VehicleControlMode |
| `/fmu/in/config_overrides_request` | ConfigOverrides |
| `/fmu/in/mode_completed` | ModeCompleted |
| `/fmu/in/actuator_motors` | ActuatorMotors |
| `/fmu/in/actuator_servos` | ActuatorServos |
| `/fmu/in/landing_gear` | LandingGear |

> 另有约 13 条与多旋翼无关的已在文档中省略：`fixed_wing_longitudinal_setpoint`、
> `fixed_wing_lateral_setpoint`、`longitudinal_control_configuration`、
> `lateral_control_configuration`、`rover_position_setpoint`、`rover_speed_setpoint`、
> `rover_attitude_setpoint`、`rover_rate_setpoint`、`rover_throttle_setpoint`、
> `rover_steering_setpoint`，以及 `register_ext_component_request`、
> `unregister_ext_component`、`arming_check_reply`、`message_format_request`。
> 源码里 `subscriptions_multi:` 段是空的（文件到此结束）。

**三条必须记住的限制：**

1. **`rate_limit` 是真实限流。** 带 `rate_limit` 的话题，飞控最多按该频率往 DDS 推。
   `vehicle_local_position` 被限到 **50 Hz**，不是 EKF2 内部跑的那个频率。
   想靠它做高频控制回路是不行的——但控制回路本来就在飞控里做，派只需要 50 Hz 做决策。
2. **`/fmu/out/vehicle_angular_velocity` 在源码里是被注释掉的**，
   所以**派拿不到高频角速率**。别在派上设计任何依赖角速度的控制器。
3. **`distance_sensor` 和 `sensor_optical_flow` 只有 `in` 方向，没有 `out`。**
   也就是说：**派读不到光流和测距的原始数据。** 详见下一节。

---

### 5.2 光流 + 测距怎么接：直连飞控，派不用读原始数据

**结论：光流/测距模块直接接飞控（UART/SPI），不要绕道树莓派。**
你的判断是对的，这里说清为什么，以及"那派上的高度怎么办"。

**为什么不绕道派**：EKF2 融合光流需要高频、低延迟、稳定的数据流。
数据绕一圈"模块 → 派 → ROS 2 → XRCE-DDS → 飞控"，会凭空引入几十到几百毫秒延迟和抖动，
结果只会比直连更差，而且多了一堆会出错的环节。**没有任何好处。**

**那派上的高度信息怎么办？——不需要额外接线，也不用读原始值。**

飞控已经把测距数据和光流数据融合进 `vehicle_local_position` 了，直接读它就有：

| 字段 | 含义 |
| --- | --- |
| `z` | NED 高度（**向下为正**，即 `高度 = -z`），单位 m |
| `vz` | 垂直速度，m/s |
| **`dist_bottom`** | **对地距离，m ← 就是测距模块测出来的那个值（已融合）** |
| **`dist_bottom_valid`** | 上面这个值是否有效 |
| **`dist_bottom_sensor_bitfield`** | 该值来自什么传感器：`0`=NONE，`1`=RANGE（测距），`2`=FLOW（光流） |
| `xy_valid` / `z_valid` / `v_xy_valid` / `v_z_valid` | 各路估计是否有效 |
| `eph` / `epv` / `evh` / `evv` | 水平/垂直 位置、速度的标准差 |
| `dead_reckoning` | 是否在纯惯导推算（**为 true 说明位置源已经丢了，要警惕**） |
| `hagl_min` / `hagl_max_z` / `hagl_max_xy` | 估计器给出的离地高度上下限 |

> **融合后的 `dist_bottom` 比原始测距值更好用**：它已经过飞控的滤波和有效性判定，
> 带 `dist_bottom_valid` 标志和方差，还告诉你数据来源。
> 你自己去接原始串口，反而要重新做一遍这些判断。

所以正确做法是：**光流/测距接飞控，派订阅 `/fmu/out/vehicle_local_position` 取高度。**
零额外接线，拿到的是更可靠的融合值。

> 如果以后确实需要原始值（比如调试"测距到底有没有在喂飞控"），
> 看 `dist_bottom_sensor_bitfield` 是不是 `1`、`dist_bottom_valid` 是不是 `true`
> 就够了——不需要真去读那个串口。

---

## 6. 里程碑 M2：ROS 2 工作区跑起来

- 建 ROS 2 workspace，`git clone` **`px4_msgs`**，
  **切到与 PX4 1.17 对应的分支**（先 `git branch -r` 看分支名，找 `release/1.17` 之类；
  对不上就选版本号最接近的），`colcon build` 后 `source install/setup.bash`。
- 写一个最小的订阅节点，打印 `vehicle_local_position` 的 x/y/z 和 `vehicle_status` 的解锁状态。
- **验收**：手动拿起飞机晃动，终端数值跟着变。至此你有了"读"的能力。
  （注意：**光流/测距装好之前，`vehicle_local_position` 的 x/y 会是无效值**，这是正常的，
  别以为坏了——原因见 M3 前置条件。）

---

## 7. 里程碑 M3：Offboard 最小闭环（★核心）

**全程无桨、桌面、USB/电池台架。**

### ⚠ 前置条件：必须先有位置源（最容易被忽略的一条）

官方 offboard 文档原文：该模式需要 position 或 pose/attitude 信息 ——
"e.g. GPS, optical flow, visual-inertial odometry, mocap, etc."

室内无 GPS、又还没装光流/激光时，EKF2 出不了 x/y 位置估计，
**offboard 位置模式根本不会生效** —— 不是你的代码写错了。
所以：**M3 想真正飞，之前必须先有位置源（见 M4a）。**
台架上可以先用 `body_rate`/`attitude` 档验证数据通路通不通，
但"能起飞"必须等位置源到位。

### 标准时序（顺序不能错）

```
1. 等 /fmu/out/vehicle_status 出现（飞控在线）
2. 持续发布 /fmu/in/offboard_control_mode（position=true），≥ 2 Hz
   —— PX4 需要连续收到超过 1 秒才会认账；实践上写 50 Hz 最省心
3. 发 /fmu/in/trajectory_setpoint（NED；未使用的字段填 NaN）
4. 发 /fmu/in/vehicle_command: DO_SET_MODE, param1=1(自定义), param2=6(OFFBOARD)
5. arm: VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1
6. 把 setpoint 的 z 抬高（NED 下 z 为负，-1.5 表示升高 1.5 m）
7. 退出：发 Position 模式 / disarm
```

### 必须知道的细节

- **"心跳"是 `OffboardControlMode`，不是 `TrajectorySetpoint`。**
  官方说位置保持时 `TrajectorySetpoint` 发一次就够，但 `OffboardControlMode` 必须持续流。
  两者都按 50 Hz 发最不容易踩坑。
- **`OffboardControlMode` 里第一个非零的控制位生效**，优先级依次为
  `position > velocity > acceleration > attitude > body_rate > thrust_and_torque > direct_actuator`。
- **NED 坐标系**：x 前、y 右、z **下**。起飞是 z 变负。
- 时间戳字段必填（微秒级），Python 里：
  `msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)`
- **QoS 必须改**（PX4 不用 ROS 2 默认 QoS，用默认值订阅不到数据）：
  - 订阅 PX4 发布的话题 → `BEST_EFFORT` + `TRANSIENT_LOCAL` + `KEEP_LAST(1)`
  - 向 PX4 发布 → `BEST_EFFORT` + `VOLATILE` + `KEEP_LAST(1)`
- 参考骨架直接抄 `px4_msgs` 仓库自带的 examples，别从零写。

### 失效与夺权（★首飞前必须逐条确认）

| 参数              | 默认                  | 你要做的                                                                 |
| ----------------- | --------------------- | ------------------------------------------------------------------------ |
| `COM_OF_LOSS_T`   | 0.5 s                 | 断流超过这个时间 → 自动退出 offboard                                     |
| `COM_OBL_RC_ACT`  | Position              | 退出后进哪个模式：0 Position / 1 Altitude / 2 Manual / 3 Return / 4 Land |
| `COM_RC_OVERRIDE` | **offboard 下不生效** | **必须显式启用**，否则打杆抢不回控制权                                   |
| `COM_RC_IN_MODE`  | 默认需要遥控          | 设 4 可完全禁用摇杆（做无遥控自主飞行时用）                              |

> ⚠ 官方原文：`COM_RC_OVERRIDE` "is not enabled for offboard mode by default"。
> 也就是说**出厂默认下，你打杆是抢不回控制权的**。首飞前一定改。

### 验收

无桨状态下：能进 offboard、能 arm、`ros2 topic echo /fmu/out/vehicle_local_position`
看到 z 按你的指令变化、摇杆能夺回控制权、拔掉串口线飞控能自动退出 offboard。
**这四条全过，才允许装桨。**

---

## 8. 里程碑 M4~M5：位置源与雷达回灌

### 8.1 顺序：先有位置源，offboard 才可能生效

| 阶段 | 做什么                                                                       | 产出                                       |
| ---- | ---------------------------------------------------------------------------- | ------------------------------------------ |
| M4a  | 装 **光流 + 激光/超声测距**（`MTF-01`/`MT-06` 之类），QGC 里配好光流与测距融合 | 室内有 x/y/z 位置估计，**offboard 才可用** |
| M4b  | 雷达/激光 SLAM 单独跑通（先不接飞控）                                        | 高精度水平定位                             |
| M5   | 把位姿回灌进飞控（见 8.2~8.7）                                               | 定点 / 自主飞行                            |

### 8.2 ★ 位姿必须回灌给 PX4（最关键的一条）

**只往飞控发目标点是不够的。** PX4 位置环的本质是 `误差 = 目标 − 当前`，
你只发目标却不告诉它"当前在哪"，它没有反馈量可减 ——
轻则 offboard 模式被拒绝进入，重则进去之后直接飘走。

所以雷达 SLAM 算出的位姿**必须回灌进 EKF2**（用 `VehicleOdometry`）：

```
┌────────────── 树莓派 5（Ubuntu 24.04 + ROS 2 Jazzy）──────────────┐
│                                                                  │
│  /scan ──► SLAM(slam_toolbox) ──► 地图 ──► 任务决策/路径规划       │
│              │                                    │              │
│              └──► 位姿 (x, y, yaw) ───────────────┘              │
│                        │                                         │
│           ┌────────────┴────────────┐                            │
│           │  坐标变换 + 原点对齐     │                            │
│           │  ENU→NED，高度来自测距   │                            │
│           └────────────┬────────────┘                            │
│                        │                                         │
│      ┌─────────────────┴─────────────────┐                       │
│      ▼                                   ▼                       │
│  /fmu/in/vehicle_odometry      /fmu/in/trajectory_setpoint       │
│  「我在哪」← 关键！              「去哪」                          │
└──────┬───────────────────────────────────┬──────────────────────┘
       │        uXRCE-DDS / UART (TELEM2/3)│
┌──────▼───────────────────────────────────▼──────────────────────┐
│                      PX4 飞控 (MicoAir743v2)                     │
│   EKF2 融合外部位姿 → 位置估计 → 位置控制器 → 姿态/角速率环 → 电机 │
└──────────────────────────────────────────────────────────────────┘
```

**注意最后一段**：PX4 内部的姿态环和角速率环（跑在 1 kHz）**永远由飞控做**，
不要在派上替代 —— 派跑 Linux，抖动是毫秒级，角速率环抖一下就炸机。
派只发"目标"，这是正确的分工。

### 8.3 2D 激光雷达的三个硬限制

**① 高度从哪来？** 2D 雷达扫的是一个**水平面**，只能给 x/y 和偏航，**给不了 z**。
室内气压计受地效和气流影响漂得厉害。→ 必须补一个**向下测距**（激光/超声）。

**② 坐标系必须变换（不做必炸）**

|      | ROS 惯例 (REP-103) | PX4           |
| ---- | ------------------ | ------------- |
| 导航系 | ENU（东-北-天）    | **NED**（北-东-地） |
| 机体系 | FLU（前-左-上）    | **FRD**（前-右-下） |

位置的换算（ENU → NED）：

```
x_NED =  y_ENU        （北）
y_NED =  x_ENU        （东）
z_NED = -z_ENU        （地）
yaw_NED = π/2 − yaw_ENU
```

姿态**不要手工推四元数**，用旋转矩阵中转（`C_ENU←NED` 与 `C_FLU←FRD` 都是正交阵）：

```
R_NED←FRD = C_ENU←NEDᵀ · R_ENU←FLU · C_FLU←FRD

C_ENU←NED = [[0,1,0],[1,0,0],[0,0,-1]]
C_FLU←FRD = [[1,0,0],[0,-1,0],[0,0,-1]]
```

> 实用技巧：`VehicleOdometry.pose_frame` 填 **`POSE_FRAME_FRD`（=2）**，
> 它的定义就是"相对真北有任意固定朝向偏移，Z 向下"——
> 正好匹配 SLAM 的 map 系本来就是任意朝向这一事实。用 NED(=1) 反而要求你先把朝向对准真北。

**③ 原点对齐。** SLAM 的 `map` 原点是任意的（通常是你启动雷达那一刻的位置），
而 PX4 的 local NED 原点是 **EKF2 初始化那一刻**。两者不重合就有一个巨大常值偏置，
飞控会以为自己在几米外，一解锁就猛冲。→ 喂给飞控前减掉偏置，
或用 `EKF2_EV_POS_X/Y/Z` 描述雷达相对**机体中心（COG）**的安装位置。

### 8.4 EKF2 外部位姿参数（已对 PX4 v1.17.0 源码逐条核实）

> 源码路径：`src/modules/ekf2/params_external_vision.yaml`
> （**不在** `module.yaml` 里，别去那儿找）

| 参数              | 类型     | 默认 | 范围     | 说明                                                                 |
| ----------------- | -------- | ---- | -------- | -------------------------------------------------------------------- |
| **`EKF2_EV_CTRL`** | bitmask  | **`0`** | 0–15     | **默认 0 = 外部位姿完全不融合！必须手动置位**                        |
| `EKF2_EV_DELAY`   | float    | `0`  | 0–300 ms | 视觉/位姿数据相对 IMU 的延迟。**重启生效**；SLAM 延迟不补偿会发散    |
| `EKF2_EV_NOISE_MD`| enum     | `0`  | 0/1      | 0 = 用消息自带方差（参数作下限）；1 = 直接用参数值                   |
| `EKF2_EV_QMIN`    | int32    | `0`  | 0–100    | quality 门限，低于它不融合。quality 是可选字段                       |
| `EKF2_EV_POS_X`   | float    | `0.0` | —        | 传感器焦点在机体系 X 的位置（相对 COG），单位 m                      |
| `EKF2_EV_POS_Y`   | float    | `0.0` | —        | 同上，Y                                                              |
| `EKF2_EV_POS_Z`   | float    | `0.0` | —        | 同上，Z                                                              |

**`EKF2_EV_CTRL` 位定义**（官方原文）：

| bit | 含义                   |
| --- | ---------------------- |
| 0   | Horizontal position fusion（水平位置融合） |
| 1   | Vertical position fusion（垂直位置融合）   |
| 2   | 3D velocity fusion（三维速度融合）         |
| 3   | Yaw（偏航融合）                            |

> 典型配置：只信雷达的 x/y，高度交给测距/气压计 → **`EKF2_EV_CTRL = 1`**（只开 bit0）。
> 如果雷达只做水平、高度由测距提供，就**不要**开 bit1，否则两路高度会打架。

配套的噪声/门限参数（前缀不同，容易漏）：

| 参数             | 默认  | 范围   | 说明                         |
| ---------------- | ----- | ------ | ---------------------------- |
| `EKF2_EVP_NOISE` | `0.1` | ≥0.01 m | 视觉位置测量噪声            |
| `EKF2_EVP_GATE`  | `5.0` | ≥1.0 SD | 位置融合的新息一致性门限     |
| `EKF2_EVV_NOISE` | `0.1` | ≥0.01 m/s | 视觉速度测量噪声          |
| `EKF2_EVV_GATE`  | `3.0` | ≥1.0 SD | 速度融合门限                 |
| `EKF2_EVA_NOISE` | `0.1` | ≥0.05 rad | 视觉角度测量噪声          |

### 8.5 `VehicleOdometry` 消息（v1.17.0 核实）

> 源码路径注意：**`msg/versioned/VehicleOdometry.msg`**（顶层没有这个文件）。
> 用 ROS 2 的 `px4_msgs` 时无感，但直接翻源码找不到别慌。
> 文件头写明 "Fits ROS REP 147 for aerial vehicles"。

| 字段                  | 类型        | 说明                                                                    |
| --------------------- | ----------- | ----------------------------------------------------------------------- |
| `timestamp`           | uint64      | 微秒。**必填**                                                          |
| `timestamp_sample`    | uint64      | 微秒                                                                    |
| `pose_frame`          | uint8       | 0 UNKNOWN / **1 NED** / **2 FRD**（推荐 2，见 8.3）                     |
| `position`            | float32[3]  | m。**原点 = 启动时 GC（重心）的位置**；无效填 NaN                       |
| `q`                   | float32[4]  | `(w,x,y,z)`，Hamilton 约定，右手系、被动旋转（body→world）              |
| `velocity_frame`      | uint8       | 0 UNKNOWN / 1 NED / 2 FRD / 3 BODY_FRD                                  |
| `velocity`            | float32[3]  | m/s                                                                     |
| `angular_velocity`    | float32[3]  | rad/s，**固定为机体 FRD 系**                                            |
| `position_variance`   | float32[3]  | m²，位置误差方差                                                        |
| `orientation_variance`| float32[3]  | rad²，姿态误差方差                                                      |
| `velocity_variance`   | float32[3]  | m²/s²                                                                   |
| `reset_counter`       | uint8       | 姿态/速度/位置的重置计数                                                |
| `quality`             | int8        | 质量，**当前未使用**（但受 `EKF2_EV_QMIN` 门控）                        |

> ⚠ 用 `EKF2_EV_NOISE_MD=0`（默认）时，**`*_variance` 字段必须填**，
> 填 0 会被参数下限顶掉，但填 NaN 可能直接被丢弃。

### 8.6 `TrajectorySetpoint` 消息（v1.17.0 核实）

> 源码路径同样是 **`msg/versioned/TrajectorySetpoint.msg`**（顶层是 `TrajectorySetpoint6dof.msg`）。
> 文件头原文："Trajectory setpoint in NED frame / Input to PID position controller /
> **setting a value to NaN means the state should not be controlled**"

| 字段         | 类型       | 说明                                     |
| ------------ | ---------- | ---------------------------------------- |
| `timestamp`  | uint64     | 微秒                                     |
| `position`   | float32[3] | m，NED 局部世界系                        |
| `velocity`   | float32[3] | m/s                                      |
| `acceleration`| float32[3] | m/s²                                    |
| `jerk`       | float32[3] | m/s³（**仅用于日志**，不影响控制）        |
| `yaw`        | float32    | rad，−π..+π，期望偏航角                  |
| `yawspeed`   | float32    | rad/s，绕 NED z 轴的角速度               |

**NaN 语义很重要**：只想控位置，就把 `velocity`/`acceleration` 填 NaN，
而不是填 0 —— 填 0 表示"要求速度为零"，是完全不同的控制目标。

### 8.7 分步落地（每步无桨验证完再进下一步）

| 步骤 | 做什么                                                        | 怎么验                              |
| ---- | ------------------------------------------------------------- | ----------------------------------- |
| M5a  | 雷达 SLAM 单独跑通，**先不接飞控**                            | `rviz2` 里看位姿，走一圈看漂不漂     |
| M5b  | 写死坐标变换函数 + **单元测试**                               | 手动把飞机转 90°，看输出的偏航对不对 |
| M5c  | 只发 `VehicleOdometry`，**先不发 setpoint**                   | 比对 `vehicle_local_position` 与 SLAM |
| M5d  | 最后才发 setpoint：先原地悬停，再走 1 米方形                  | 系留     |

### 8.8 建议的分工：保命层 + 任务层

不要把飞行安全全押在 SLAM 上（现场光变、反光地面、人员走动都会让它出问题）：

| 层                 | 用什么         | 干什么                                             |
| ------------------ | -------------- | -------------------------------------------------- |
| **保命层（进 EKF2）** | 光流 + 测距    | 高频低延迟、不漂，保证稳悬停                       |
| **任务层（派内部）**  | 雷达 SLAM      | 建图、全局定位、路径规划、决定"去哪"                |
| **提精层（可选）**    | SLAM 位姿也进 EKF2 | 提高绝对定位精度，但要配好延迟与噪声            |

**SLAM 崩了飞机还能靠光流稳住悬停** —— 这个冗余在赛场上差别很大。

---

### 8.9 光流 + 测距参数（已对 PX4 v1.17.0 源码逐条核实）

装了光流和向下测距之后，飞控默认配置**不一定是你想要的行为**。这一节是必查清单。

**先说三个源码路径事实**（直接去翻源码会找不到）：

- 光流参数在 **`src/modules/ekf2/params_optical_flow.yaml`**
- 测距参数在 **`src/modules/ekf2/params_range_finder.yaml`**（注意是 `range_finder` 带下划线，
  `params_rangefinder.yaml` 和 `params_flow.yaml` 都是 **404**）
- **`SENS_FLOW_*` 不在任何 YAML 里**，在 **`src/modules/sensors/sensor_params_flow.c`** 里用 C 宏定义

#### ⚠ 第一个大坑：`EKF2_OF_CTRL` 和 `EKF2_EV_CTRL` 的默认值是**相反的**

| 参数 | 类型 | **默认值** | 含义 |
| --- | --- | --- | --- |
| `EKF2_EV_CTRL` | **bitmask** | **`0`** | 外部位姿（SLAM）——**默认全关，必须手动开** |
| `EKF2_OF_CTRL` | **boolean** | **`1`** | 光流——**默认就是开的** |

> 官方原文：`EKF2_OF_CTRL` — type: `boolean`, default: `1`, short: "Optical flow aiding",
> long: "Enable optical flow fusion."
> 它**不是** bitmask，没有 bit 位定义、没有 `values:` 子项。别跟 `EKF2_EV_CTRL` 搞混。

#### 第二个大坑：`EKF2_RNG_CTRL` 默认是"条件模式"，不是"一直用"

| 取值 | 原文含义 |
| --- | --- |
| `0` | Disable range fusion |
| **`1`** | **Enabled (conditional mode) ← 默认值** |
| `2` | Enabled |

**默认的 `1`（conditional mode）只在同时满足两个条件时才用测距融合高度：**

- 水平速度 < `EKF2_RNG_A_VMAX`（默认 **1.0 m/s**）
- 离地高度 < `EKF2_RNG_A_HMAX`（默认 **5.0 m**）

室内慢速飞刚好在范围内，所以"默认也能用"。但**如果你要全程靠测距定高，必须改成 `2`**，
否则一旦水平速度超过 1 m/s，测距就被静默停用，高度开始漂。

> ⚠ `EKF2_RNG_CTRL` 的官方描述里有一段 **WARNING 原文**：
> "Range finder measurements are less reliable and can experience unexpected errors.
> For these reasons, **if accurate control of height relative to ground is required,
> it is recommended to use the `MPC_ALT_MODE` parameter instead**, unless baro errors are
> severe enough to cause problems with landing and takeoff."
>
> 也就是说：**精确对地高度控制，官方推荐先看 `MPC_ALT_MODE`**，测距是次选。

#### 光流参数 `EKF2_OF_*`（来自 `params_optical_flow.yaml`）

| 参数 | 类型 | 默认 | 范围 | 单位 | 备注 |
| --- | --- | --- | --- | --- | --- |
| `EKF2_OF_CTRL` | boolean | `1` | — | — | 光流融合总开关（**默认已开**） |
| `EKF2_OF_DELAY` | float | `20` | 0–300 | ms | **reboot_required**；原文 "Assumes measurement is timestamped at trailing edge of integration period" |
| `EKF2_OF_GATE` | float | `3.0` | min 1.0 | SD | 新息一致性检验门限 |
| `EKF2_OF_N_MIN` | float | `0.15` | min 0.05 | rad/s | 质量最好时的测量噪声 |
| `EKF2_OF_N_MAX` | float | `0.5` | min 0.05 | rad/s | 质量最差时的测量噪声 |
| `EKF2_OF_QMIN` | int32 | `1` | 0–255 | — | **空中**使用光流的最低质量分 |
| `EKF2_OF_QMIN_GND` | int32 | `0` | 0–255 | — | **地面**使用光流的最低质量分 |
| `EKF2_OF_GYR_SRC` | enum | `0` | — | — | `0`=Auto（光流消息自带陀螺就用它，否则用内置陀螺）/ `1`=Internal（始终用内置陀螺） |
| `EKF2_OF_POS_X/Y/Z` | float | `0.0` | — | m | 光流焦点相对**重心**的位置（杆臂） |

#### 测距参数 `EKF2_RNG_*`（来自 `params_range_finder.yaml`）

| 参数 | 类型 | 默认 | 范围 | 单位 | 备注 |
| --- | --- | --- | --- | --- | --- |
| `EKF2_RNG_CTRL` | enum | **`1`** | — | — | 见上面第二个大坑；`0`关 / `1`条件 / `2`常开 |
| `EKF2_RNG_DELAY` | float | `5` | 0–300 | ms | **reboot_required** |
| `EKF2_RNG_GATE` | float | `5.0` | min 1.0 | SD | 新息一致性门限 |
| `EKF2_RNG_NOISE` | float | `0.1` | min 0.01 | m | 测量噪声 |
| `EKF2_RNG_SFE` | float | `0.05` | 0.0–0.2 | m/m | 噪声随距离增长的系数 |
| `EKF2_RNG_PITCH` | float | `0.0` | −0.75–0.75 | rad | **安装俯仰偏置**（雷达斜装了填这里） |
| `EKF2_RNG_A_VMAX` | float | `1.0` | 0.1–2 | m/s | 条件模式下允许的最大水平速度 |
| `EKF2_RNG_A_HMAX` | float | `5.0` | 1.0–10.0 | m | 条件模式下允许的最大离地高度 |
| `EKF2_RNG_QLTY_T` | float | `1.0` | 0.1–5 | s | 信号质量需持续非零多久才判定有效 |
| `EKF2_RNG_K_GATE` | float | `1.0` | 0.1–5.0 | SD | 运动学一致性检验门限（原文：更容易受卡死/反射影响时调小） |
| `EKF2_RNG_FOG` | float | `3.0` | 0.0–20.0 | m | 起雾/雨检测距离上限，`0`=禁用 |
| `EKF2_RNG_POS_X/Y/Z` | float | `0.0` | — | m | 测距仪原点相对**重心**的位置（杆臂） |

> 注意两个名字容易记错：**没有** `EKF2_RNG_AID`（这个角色由 `EKF2_RNG_CTRL` 承担）；
> **没有** `EKF2_RNG_K_G`，实际名字是 **`EKF2_RNG_K_GATE`**。

#### 传感器层参数 `SENS_FLOW_*`（来自 `sensor_params_flow.c`，**不在 YAML 里**）

| 参数 | 类型 | 默认 | 范围 | 单位 | 备注 |
| --- | --- | --- | --- | --- | --- |
| **`SENS_FLOW_ROT`** | int32 | `0` | 0–7 | — | **光流模块相对机体的偏航旋转，每档 45°** |
| `SENS_FLOW_MINHGT` | float | `0.08` | 0.0–1.0 | m | 可靠工作最低高度 |
| `SENS_FLOW_MAXHGT` | float | `100.` | 1.0–100.0 | m | 可靠工作最高高度（完全依赖光流时，高度设定点会被限到这个值以内） |
| `SENS_FLOW_MAXR` | float | `8.` | min 1.0 | rad/s | 可靠测量的最大角流率；超过则估计器不融合，且控制回路会把地速限制到它的 **50%** 以内 |
| `SENS_FLOW_RATE` | float | `70.0` | 1–200 | Hz | 光流最大发布率，**reboot_required** |
| `SENS_FLOW_SCALE` | float | `1.` | 0.5–1.5 | — | 光流尺度因子 |

**`SENS_FLOW_ROT` 的 8 个取值（原文）**：

| 值 | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 含义 | No rotation | Yaw 45° | Yaw 90° | Yaw 135° | Yaw 180° | Yaw 225° | Yaw 270° | Yaw 315° |

> 原文对零点的定义：**"Zero rotation is defined as X on flow board pointing towards front of vehicle."**
> 所以光流板装歪了**不用拆下来重装**，量出偏角填 `SENS_FLOW_ROT` 就行。
> 但注意它只能填 45° 的整数倍——**装的时候尽量摆正**。

#### 本节到手后要改的（装完光流+测距，逐条过一遍）

1. 确认 `EKF2_OF_CTRL = 1`（默认就是，但确认一下）
2. **按实际安装量出 `SENS_FLOW_ROT`**（这一步不做，光流融合出来是错的）
3. 按实际安装填 `EKF2_OF_POS_X/Y/Z` 和 `EKF2_RNG_POS_X/Y/Z`（杆臂，重心为原点）
4. 雷达/测距斜装的话填 `EKF2_RNG_PITCH`
5. 想全程用测距定高 → `EKF2_RNG_CTRL = 2`；只在起降用 → 保持默认 `1`
6. 看 `EKF2_RNG_CTRL` 那段 WARNING，评估是否该改用 `MPC_ALT_MODE`
7. 调完**重启飞控**（`EKF2_OF_DELAY`、`EKF2_RNG_DELAY`、`SENS_FLOW_RATE` 都是 reboot_required）

---

### 8.10 镭神 N10 雷达：驱动与配置

#### 硬件规格（信源汇总，**以你手上的 datasheet 为准**）

| 项目 | 数值 |
| --- | --- |
| 测距范围 | **0.02 – 25 m** |
| 扫描角度 | **360°** |
| 扫描频率 | **10 Hz** |
| 角度分辨率 | **0.48°** |
| 测距原理 | **TOF** |
| 精度 | **±20 mm** |
| 单圈点数 | ≈ **750**（360° ÷ 0.48°） |

> ⚠ 上面几项来自官方产品页与 datasheet；另有信源称角分辨率 0.5°、精度 ±3 cm，
> 与 datasheet 有出入，**以随货 datasheet 为准**。
> ⚠ **不要用驱动里的 `max_range: 50.0` 反推雷达量程** —— 那是驱动的默认值（见下），
> 不是雷达实际能力。N10 实际是 25 m。

#### 两个硬件版本，先确认你是哪个

N10 有**串口版**和**网口版**两种，**驱动配置完全不同**。先看实物接头再动手。

> ✅ **本项目已确认：串口版**，通过转接板接到树莓派。
> 所以走 **`lslidar_n10_uart.yaml` + `lsn10_launch.py`** 这一套，
> `device_ip` / `msop_port` / `difop_port` 那几个网口参数**全部与你无关**，忽略即可。
>
> ⚠ **仍需自己确认的两项**（我未查到可靠公开来源，请翻随货资料包）：
> 1. **串口波特率** —— 转接板出来的波特率必须和驱动里配置的一致，
>    不一致的现象是 `/scan` 完全没数据或全是噪点。
> 2. **供电电压** —— 转接板是 5 V 还是 12 V 供电。
>    接错电压是最容易烧板子的一步，**动手前先确认**。
>
> 另：转接板通常是 USB-转-TTL，**那 `serial_port` 应填 `/dev/ttyUSB0` 之类**，
> 而不是默认的 `/dev/wheeltec_lidar`。用
> `ls /dev/ttyUSB* /dev/ttyACM*` 看插拔前后的变化来确定。

#### 驱动来源（两条路，都可以）

**官方 GitHub**：<https://github.com/Lslidar/Lslidar_ROS2_driver>

> 注意：镭神官方 GitHub 是**用户账号** `Lslidar`，不是组织。
> 所以 `https://github.com/orgs/Lslidar/...` 会 **404**，别被这个误导。

**随货资料包**：官方也通过资料包分发 `lslidar_driver.zip` + `lslidar_msgs.zip`
（古月居 / 轮趣等教程都是"解压资料包"而不是 `git clone`）。

**两个包的分工**：

| 包 | 作用 |
| --- | --- |
| `lslidar_driver` | 驱动节点 + config + launch |
| `lslidar_msgs` | 自定义消息/服务（**必须一起编，缺了编不过**） |

#### 关键文件

| 用途 | 路径 |
| --- | --- |
| N10 **串口版**参数 | `lslidar_driver/config/lslidar_n10_uart.yaml` |
| N10 **网口版**参数 | `lslidar_driver/config/lslidar_n10_net.yaml` |
| N10 **串口版**启动 | `lslidar_driver/launch/lsn10_launch.py` |
| N10 **网口版**启动 | `lslidar_driver/launch/lsn10_net_launch.py` |
| 自带 RViz 配置 | `lslidar_driver/rviz/lslidar_x10.rviz` |

#### 依赖（ROS 2 Humble 实测环境）

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions
sudo apt install -y \
  ros-$ROS_DISTRO-pcl-conversions \
  ros-$ROS_DISTRO-builtin-interfaces \
  ros-$ROS_DISTRO-rosidl-default-generators \
  libpcl-dev \
  libpcap-dev \
  libyaml-cpp-dev

# 编译时报缺 diagnostic_updater 再补这一条：
sudo apt install -y ros-$ROS_DISTRO-diagnostic-updater
```

#### 串口版参数（`lslidar_n10_uart.yaml`）

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `lidar_type` | `"X10"` | **N10 挂在 X10 这个驱动类型族下，不要改** |
| `lidar_model` | `"N10"` | 雷达型号 |
| `serial_port` | `"/dev/wheeltec_lidar"` | 按实际改，如 `"/dev/ttyUSB0"` |
| `device_ip` | `192.168.1.200` | 网口版雷达 IP（串口版无关） |
| `msop_port` | `2368` | 雷达**数据**端口（网口版） |
| `difop_port` | `2369` | 雷达**设备**端口（网口版） |
| `frame_id` | `"laser"` | 雷达坐标系名 |
| `laserscan_topic` | `"/scan"` | |
| `pointcloud_topic` | `"lslidar_point_cloud"` | |
| `publish_scan` | `true` | 是否发布 `/scan` |
| `use_high_precision` | `false` | 高精度模式；**增大数据量，算力弱先别开** |
| `min_range` | `0.15` | 最小有效距离，m |
| `max_range` | `50.0` | 最大有效距离，m（**驱动默认值，非雷达量程**） |
| `is_pretreatment` | `false` | 开启后才应用下面的偏移 |
| `x_offset` / `y_offset` / `z_offset` | `0.0` | 点云平移 |
| `roll` / `pitch` / `yaw` | `0.0` | 点云旋转 |

> ⚠ `lidar_type: "X10"` 这条很反直觉但**不能改**——N10 复用 X10 那套解析逻辑。
> 把型号改成 N10 却动了 `lidar_type` 是最常见的"启动没数据"原因之一。

#### 角度屏蔽（单位是 **0.01°**，很容易填错）

```yaml
# 默认：不屏蔽
angle_disable_min: [0]
angle_disable_max: [0]

# 屏蔽 0° ~ 90°：
angle_disable_min: [0]
angle_disable_max: [9000]

# 屏蔽两段：0°~90° 和 180°~270°
angle_disable_min: [0, 18000]
angle_disable_max: [9000, 27000]
```

> **单位是 0.01°，所以 `9000` = 90°**。
> 装到机架上之后，**用这个把机体自身（起落架、机臂、桨）挡住的那几个扇区切掉**，
> 否则 SLAM 会把自家人当成墙。

#### 发布的话题

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/scan` | `sensor_msgs/msg/LaserScan` | SLAM / 避障 / Nav2 的输入 |
| `/lslidar_point_cloud` | `sensor_msgs/msg/PointCloud2` | RViz 看原始点云 |

#### 最小跑通流程（串口版）

```bash
mkdir -p ~/lslidar_ws/src
# 把 lslidar_driver 和 lslidar_msgs 放进 ~/lslidar_ws/src
cd ~/lslidar_ws
sudo apt install -y python3-colcon-common-extensions libpcap-dev libpcl-dev libyaml-cpp-dev
colcon build
source install/setup.bash
ros2 launch lslidar_driver lsn10_launch.py
```

另开终端验收：

```bash
source ~/lslidar_ws/install/setup.bash
ros2 topic list                 # 应有 /scan 和 /lslidar_point_cloud
ros2 topic hz /scan             # 应稳定在 10 Hz 左右
```

RViz2 可视化：

```bash
rviz2 -d ~/lslidar_ws/install/lslidar_driver/share/lslidar_driver/rviz/lslidar_x10.rviz
```

RViz2 里 **Fixed Frame 设成 `laser`**（对应参数里的 `frame_id: "laser"`），
再 `Add → By topic → /scan → LaserScan`，应该能看到一圈激光点。

> ⚠ **改完 `config/*.yaml` 必须重新 `colcon build` 并重新 `source`。**
> 否则启动时读的还是 `install/` 目录里的旧参数文件，你会以为改的参数没生效。

#### M5a 验收（这一步**不要接飞控**）

- `/scan` 稳定 10 Hz
- RViz2 里能看到一圈干净的点，没有大面积空洞或噪点
- 手动把雷达转 90°，RViz2 里的点跟着转（说明朝向没搞反）

**这一步只证明雷达本身没问题**，还没到让它影响飞行的时候。
SLAM 和坐标变换是 M5b 的事。

---

## 9. 通信链路（派 ↔ 地面站）

- 飞控 MAVLink 继续走 TELEM1 接数传；派走 WiFi/4G。
- 想在 QGC 上同时看到飞控：在派上跑 **mavlink-router**，
  把 TELEM1 的 MAVLink 转发/汇聚到地面站，并和 ROS 2 链路共存。
- **XRCE-DDS 和 MAVLink 走不同 UART，互不冲突**——这正是保留 TELEM1 的原因。

---

## 10. 安全纪律

1. **装桨前**：M1~M3 全部验收通过。
2. **首飞**：室内、系留或限制高度 < 1 m，手放遥控器上，随时打杆夺回。
3. **失能开关**：遥控器上一个通道直接切 Position/Stabilized。
4. **断流演练**：主动拔串口线，确认 failsafe 行为符合预期，再上真机。
5. 派重启 = offboard 断流，所以**派要单独供电、单独开关**，永远别和飞控抢电。

---

## 11. 与既有资产的衔接

| 已有                              | 怎么用                                                                 |
| --------------------------------- | ---------------------------------------------------------------------- |
| `仿日志文件/flight14.py`          | 逻辑（等心跳→喂中位→解锁→爬升→降落→disarm）**完全平移到真机**，只换连接方式。真机走 USB/UART 而不是 UDP 19450 |
| WSL Ubuntu 22.04 + PX4 源码       | 继续用来编译飞控固件：`make micoair_h743-v2_default`，产物用 QGC 烧写    |
| QGroundControl 绿色版             | 继续做参数配置、传感器校准、MAVLink Console、日志下载                   |
| `PX4学习记录总结_至2026-10-06.md` | uORB / MAVLink / 模式解码 / offboard 硬规矩等概念全部适用，不用重学     |
| SITL 那套                         | 留着当"写节点不想接硬件"时的回归测试环境，不冲突                        |

---

## 12. 常见坑速查

| 现象                              | 原因 / 处理                                                       |
| --------------------------------- | ----------------------------------------------------------------- |
| `ros2 topic list` 没有 `fmu/*`    | Agent 没起 / 波特率不一致 / 串口被 console 占用 / TX/RX 没交叉     |
| Agent 起来了但飞控无反应          | `UXRCE_DDS_CFG` 没设或设错口                                       |
| 一进 offboard 就掉                | 设定点频率不够或未先喂流；时间戳字段没填                          |
| 派随机重启                        | 供电不足（见坑 1）；换独立 5V/5A BEC，加电容                      |
| 通信偶发乱码                      | 用了 mini-UART（CPU 降频漂移）；关蓝牙改用 PL011；或降波特率      |
| 派能连但 QGC 掉了                 | MAVLink 和 XRCE 抢同一个 UART；分到不同口                          |
| `uxrce_dds_client status` 找不到  | 固件没编入该模块；换官方 release 或自编译                          |
| 切 offboard 没反应 / 立刻退出     | **没有位置估计**（室内无 GPS 又没光流/SLAM）；见 M3 前置条件      |
| 打杆抢不回控制权                  | `COM_RC_OVERRIDE` 对 offboard **默认不生效**，需显式启用           |
| Agent 连上了但一个 topic 都没有   | Agent 是 v3.x；PX4 1.17 只兼容 v2.4.2 / v2.4.3                     |
| 订阅 PX4 topic 收不到数据         | QoS 用了 ROS 2 默认值；要 `BEST_EFFORT` + `TRANSIENT_LOCAL`        |
| 喂了 odometry 但飞控完全不理      | **`EKF2_EV_CTRL` 默认是 0**（外部位姿全部关闭），必须手动置位      |
| 上电就朝一个方向猛冲              | SLAM 原点与 PX4 local NED 原点不重合，有巨大常值偏置；见 8.3       |
| 位置估计发散 / 越飞越偏           | `EKF2_EV_DELAY` 没配（SLAM 有几十~几百 ms 延迟）；见 8.4           |
| 高度失控（x/y 正常）              | 2D 雷达给不了 z，又没装向下测距；见 8.3                            |
| 搜不到 `msg/VehicleOdometry.msg`  | v1.17 起移到 `msg/versioned/` 下了（`VehicleLocalPosition` 同理）  |
| 某个 topic 死活订阅不到、也不报错 | 该话题**不在** `dds_topics.yaml` 白名单里；见 5.1                  |
| 想读光流/测距原始数据             | **读不到**（只有 `in` 方向）；读 `vehicle_local_position` 的融合值  |
| 想在派上做高频角速率控制          | `vehicle_angular_velocity` 在源码里被注释掉了，拿不到高频数据      |
| 光流装了但悬停还是漂              | **`SENS_FLOW_ROT` 没按实际安装角度填**；见 8.9                     |
| 一加速高度就开始漂                | **`EKF2_RNG_CTRL` 默认 `1`=条件模式**，速度 >1 m/s 就停用测距；见 8.9 |
| 以为光流也要手动开                | `EKF2_OF_CTRL` **默认就是 `1`（开）**，和 `EKF2_EV_CTRL` 默认不同   |
| 找不到 `SENS_FLOW_ROT` 的定义文件 | 它在 `sensor_params_flow.c` 里，**不在任何 YAML** 里；见 8.9        |
