# 树莓派伴飞 PX4：工程计划书

> **本文档定位**：决策与施工计划。回答三件事——①这套系统到底怎么运作；②新逻辑长什么样；③按什么顺序做、每一步怎么算通过。
> **配套文档**：`PX4树莓派伴飞实战路线.md`（参数逐条、命令逐条、硬件坑逐条的手册）。本文只写"为什么这么定"和"下一步做什么"，需要抄命令时去翻配套文档。
> **撰写依据**：PX4-Autopilot `v1.17.0` 源码与官方文档逐条核实；坐标系换算在本机做过 **20 万次随机位姿的数值验证**（脚本 `_verify_frames.py`）。文中凡未核实的都标了 **【待核实】**。

---

## 0. 先回答你亲口提的两个问题

你说：

> 「我知道我的工程逻辑是树莓派发送数据或坐标给飞控，但是我不知道树莓派该通过什么系统进行发送，也不知道这其中 PX4 是怎么运作的。」
> 「现在的需求肯定是不匹配的，但是建图逻辑我认为是可以保留的。」

**问题一：树莓派通过什么系统发送？**

答：**micro-XRCE-DDS Agent**（下面简称 Agent）。它是一个跑在树莓派上的**常驻服务**，作用是把两套完全不同的"消息世界"翻译对接起来：

| 世界 | 在哪 | 消息机制 | 谁看得懂 |
| --- | --- | --- | --- |
| ROS 2 世界 | 树莓派 + 你的节点 | DDS / ROS 2 topic | 你的 Python / C++ 节点 |
| 飞控世界 | H743 飞控固件内部 | **uORB**（PX4 内部的广播总线） | PX4 的各个模块 |

Agent 把 ROS 2 的 topic 变成飞控能懂的 uORB 消息，反之亦然。**你不需要写任何"发送系统"**——你只要在树莓派上像平时一样 `publish` 一个 ROS 2 topic，Agent 负责把它送到飞控。

命名规则（记死这两条）：

- `/fmu/in/<名字>` = **树莓派 → 飞控**（in = 进飞控）。你想命令飞控，就发这里的 topic。
- `/fmu/out/<名字>` = **飞控 → 树莓派**（out = 出飞控）。你想知道飞机状态，就订阅这里的 topic。

**问题二：PX4 怎么运作？**

答：PX4 固件**不是一个大程序**，而是几十个互不相识的**小模块**，全靠 uORB 这条内部广播总线对话。你的指令进去以后要过五道关（详见 §3）：

```
你的 TrajectorySetpoint
   → commander（守门：现在允不允许动？）
   → mc_pos_control（位置环，把"去哪"算成"该往哪飞、飞多快"）
   → mc_att_control（姿态环，算成"该倾斜多少"）
   → mc_rate_control（速率环，1 kHz，算成"三个轴各给多大力矩"）
   → control_allocator（把力矩分给 4 个电机）
   → PWM → 电调 → 电机
```

**关键结论：树莓派只碰最上面那一层。** 下面三层永远归飞控，因为派跑的是 Linux（非实时），抖几十毫秒在速率环上就是炸机。

---

# 第一部分 · 思维梳理

## 1. 先纠正三个认知（否则后面全错）

### 1.1 PX4 不跑在树莓派上

你要的**不是**"在树莓派里跑一套 PX4"。PX4 是**固件**，跑在 H743 飞控的 **NuttX 实时操作系统**上。树莓派跑的是**伴飞电脑（companion computer）**。

正确的心智模型：

```
        ┌──────────────────────────┐        ┌───────────────────────────┐
        │  树莓派 5（Linux，非实时）│        │  H743 飞控（NuttX，硬实时）│
        │  ─────────────────────   │        │  ─────────────────────    │
        │  你的任务逻辑             │        │  commander  守门/解锁      │
        │  雷达驱动 + SLAM 建图     │  串口  │  ekf2       状态估计       │
        │  PID / 状态机             │◄─────►│  mc_pos_control  位置环    │
        │  micro-XRCE-DDS Agent     │ XRCE  │  mc_att_control  姿态环    │
        │  （把 ROS 2 topic 翻译）  │        │  mc_rate_control 1 kHz ★   │
        └──────────────────────────┘        │  control_allocator → PWM   │
                                            └───────────────────────────┘
```

**"树莓派跑 PX4"的三条路，只有第三条是你现在这条路**：

| 路子 | 是什么 | 适配你吗 |
| --- | --- | --- |
| 在派上跑 SITL | 纯仿真，一个软件飞机 | ✗ 你说了不需要仿真 |
| 在派上原生编译 PX4（`px4_raspberrypi_default`） | 派**当飞控**，需要 Navio2 / PilotPi 之类的 HAT 板，且 Linux 非实时 | ✗ 你已有 H743；Linux 做不了速率环 |
| **派当伴飞电脑 + H743 当飞控 + uXRCE-DDS** | 派算、飞控飞 | ✅ **本计划** |

> 你同伴在"地瓜派上装了个 PX4 操作系统"——大概率是上面第 1 或第 2 条。那是另一条路，不适合你现在这套硬件。

### 1.2 派**永远不直接写电机输出**

派只发**设定点**（位置目标 / 速度目标 / 姿态目标）。飞控自己解算成 PWM。

为什么这是纪律而不是偏好：

- 派的输出一旦断掉（WiFi 抖动、Python GC、SLAM 卡顿），飞控必须能自己接管。设定点机制天然支持这个：**断流超过 `COM_OF_LOSS_T` 秒，飞控自动退出 offboard 并切到 `COM_OBL_RC_ACT` 指定的模式（默认 Position）**。
- 如果派直接写 PWM，上述保护全部失效。

### 1.3 你现有的建图逻辑可以**原样保留**

这是好消息：雷达驱动 → `/scan` → Cartographer → TF 这一段，和飞控一点关系都没有。**要改的只是"TF 位置"到"飞控"之间那一个缺口**，以及"任务指令"到"飞控"之间那一个缺口。

而且你现有工程已经有一个**正确的位置**来插这两个缺口——见 §10。

---

## 2. 全系统：三根链路

把整个工程拆成三根互不干扰的链路，是最省事的思维方式。**调试时按链路隔离故障**。

```
【链路 1 · 传感器】  镭神 N10 雷达 ──USB CDC-ACM──► 树莓派
                                                        │
                                                  lslidar_driver
                                                        │
                                                    /scan (50 Hz)
                                                        ▼
【链路 2 · 定位】                              cartographer_node
                                                        │
                                              TF: odom ──► base_link
                                                        │
                                          ★ 缺口 A：位姿回灌
                                                        ▼
                                         /fmu/in/vehicle_visual_odometry
                                                        │
                                              PX4 EKF2 融合
                                                        │
                                        /fmu/out/vehicle_local_position
                                        /fmu/out/vehicle_odometry
                                                        ▼
【链路 3 · 控制】   你的任务 / PID ──► 速度或位置设定点
                                                        │
                                          ★ 缺口 B：设定点下发
                                                        ▼
                                       /fmu/in/offboard_control_mode
                                       /fmu/in/trajectory_setpoint
                                                        │
                                              PX4 commander 守门
                                                        ▼
                                       位置环 → 姿态环 → 速率环 → 电机
```

**整个改造工程，就是在补两个缺口：A（上行，位姿）和 B（下行，设定点）。** 再加上基础设施（M1 的串口链路）和参数配置。就这么多。

---

## 3. PX4 怎么运作：一条指令的九步旅程

这是你最该记住的一张表。**出问题时，它能告诉你卡在哪一步。**

| 步 | 发生地 | 谁做 | 做什么 | 卡住的表现 |
| --- | --- | --- | --- | --- |
| 1 | 树莓派 | 你的节点 | `publish` 一个 ROS 2 topic | 节点没起来 / QoS 不匹配 |
| 2 | 树莓派 | Agent | 把 ROS 2 消息序列化成 XRCE 字节流 | Agent 没起 / 版本不对（v3.x 建不了 topic） |
| 3 | 串口 | 硬件 | 字节流走 UART | 波特率错 / 没共地 / 接错口 |
| 4 | 飞控 | `uxrce_dds_client` | 反序列化，发成一条 uORB 消息 | topic 不在白名单里 → **静默不存在** |
| 5 | 飞控 | **commander** | **守门**：当前模式允许接受这个设定点吗？解锁了吗？ | **不满足条件就静默丢弃** ★ |
| 6 | 飞控 | `mc_pos_control` | 位置环：目标位置 → 速度指令（50~100 Hz） | 没有位置估计 → 不生效 |
| 7 | 飞控 | `mc_att_control` | 姿态环：速度指令 → 期望姿态（250~1000 Hz） | |
| 8 | 飞控 | `mc_rate_control` | 速率环：期望姿态 → 三个轴力矩（**1 kHz，硬实时**） | 只有飞控做得到 |
| 9 | 飞控 | `control_allocator` | 力矩 + 推力 → 4 个电机的 PWM | 桨序 / 电机方向错 |

### 模块频率表

| 层 | 典型频率 | 谁执行 | 为什么 |
| --- | --- | --- | --- |
| 传感器采集 | 1 kHz+ | 飞控 | 硬件中断驱动 |
| EKF2 状态估计 | 100~400 Hz | 飞控 | |
| **位置环** | 50~100 Hz | 飞控 | ← **派能参与的最上面一层** |
| **姿态环** | 250~1000 Hz | **飞控** | 派抖动几十毫秒 → 姿态跟着抖 |
| **速率环** | **1 kHz** | **只有飞控** | 硬实时，Linux 绝对做不到 |

### 第 5 步"静默丢弃"是新手最大的坑

commander 的守门规则里，最常撞的两条：

1. **没解锁 / 不在 offboard 模式** → 你发的 `TrajectorySetpoint` 被扔掉，**不报错、不提示**。表现就是"我发了指令，飞机纹丝不动"。
2. **没有位置估计** → offboard 位置模式根本不会生效。官方文档原文：

   > "This mode requires position or pose/attitude information - e.g. GPS, optical flow, visual-inertial odometry, mocap, etc."

   室内无 GPS、又没接光流/VIO/外部视觉时，**你切 offboard 是切不进去的**。这一条决定了本计划的施工顺序：**必须先做位置源（M3），再谈 offboard（M4）**。

---

## 4. 分工边界表（谁负责什么，不许越界）

| 职责 | 树莓派 | 飞控 | 说明 |
| --- | --- | --- | --- |
| 雷达驱动、点云/扫描 | ✅ | | |
| SLAM 建图、定位 | ✅ | | 你现有的 Cartographer 链 |
| 目标点决策、任务状态机 | ✅ | | 你的任务逻辑 |
| 位置/速度外环 PID | ✅ | | 也可以交给飞控，见 §9 |
| 位置参考来源（"我在哪"） | ✅ 提供原始位姿 | ✅ 负责融合 | **融合归 EKF2，不要自己算一份再喂回去** |
| 姿态解算 | | ✅ | 飞控 IMU 1000 Hz |
| 姿态环 / 速率环 | ✗ 禁止 | ✅ | 实时性要求 |
| 电机混控输出 | ✗ 禁止 | ✅ | |
| 失效保护 / 遥控接管 | 提供状态 | ✅ 执行 | 飞控必须能在派死掉时自己活下来 |

---

# 第二部分 · 技术核心：坐标系

> **这一部分是整个工程最容易炸机的地方，也是我花时间核实最多的地方。**
> 结论都做过数值验证（20 万次随机位姿，`_verify_frames.py`），可以照着抄。

## 5. 四套坐标系，两两必须对齐

系统和系统之间，坐标系是**反的**：

| | 树莓派 / ROS 2 侧 | PX4 侧 |
| --- | --- | --- |
| **世界坐标系** | ENU：x=东、y=北、z=**上** | NED：x=北、y=东、z=**下** |
| **机体坐标系** | FLU（`base_link`）：x=前、y=**左**、z=**上** | FRD：x=前、y=**右**、z=**下** |
| 偏航角方向 | 从 x 轴向 y 轴，**逆时针为正** | 从 x 轴向 y 轴，**顺时针为正** |

你现有工程（已核实）**全程用 ENU + FLU，零 PX4 痕迹**。全工作区 grep `NED|FRD|px4|mavros|uxrce|offboard` 命中 11 处**全是误匹配**。所以换算代码必须从零写。

## 6. 换算公式（已数值验证，照抄）

### 6.1 位置

```
p_px4 = ( p_map.y , p_map.x , -p_map.z )
```

即：**x 和 y 互换，z 取反**。

直观理解：把"东/北/上"三个分量按"北/东/下"的顺序重新排列。

### 6.2 姿态

用两次四元数乘法，**不要用任何"看起来更简洁"的闭式**（我手工推了三次，三次结果都不一样；下面的写法与 PX4 官方 `px4_ros_com` 库逐字一致，不可能错）：

```
q_px4 = NED_ENU_Q ⊗ q_map ⊗ AIRCRAFT_BASELINK_Q

NED_ENU_Q          = (w, x, y, z) = (0, +1/√2, +1/√2, 0)
AIRCRAFT_BASELINK_Q = (w, x, y, z) = (0, +1, 0, 0)
```

如果确实需要一个闭式（已数值验证 **正确**）：

```
q_px4 = ( (w+z), (x+y), (x-y), (w-z) ) / √2
       其中 (w,x,y,z) = q_map
```

### 6.3 等价的旋转矩阵写法

```
R_NED←FRD = C · R_ENU←FLU · D

C = [[0, 1, 0],      D = [[1, 0, 0],
     [1, 0, 0],           [0,-1, 0],
     [0, 0,-1]]           [0, 0,-1]]
```

`C` 是"世界系 ENU→NED"，`D` 是"机体系 FLU→FRD"。

### 6.4 偏航角 —— **★ 最容易错的一条**

```
yaw_px4 = wrap_pi( π/2 − yaw_map )
```

**注意：是"减去"，而且方向是反的。** 见 §7.1。

### 6.5 机体速度矢量

```
v_FRD = ( vx , -vy , -vz )_FLU      # 只有 y、z 取反，x 不取反
```

### 6.6 验证过的物理语义（拿这几条做交叉检查）

| 树莓派侧（地图） | PX4 侧（NED/FRD） | 说明 |
| --- | --- | --- |
| 位置 (1, 2, 0.5) ENU（东1 北2 上0.5） | (2, 1, −0.5) | x/y 换、z 取反 ✅ 数值验证 |
| 机头朝地图 +X 平飞（yaw_flu = 0） | yaw = **+90°** | 地图 +X 在东，NED 里东就是 +90° |
| 机头朝地图 +Y 平飞（yaw_flu = +90°） | yaw = **0°** | 地图 +Y 在北 ✅ |
| FLU 抬头 10°（pitch=+10°） | FRD pitch = **−10°** | FRD 里 z 朝下，抬头是负俯仰 ✅ |
| 机体"向右" FLU=(0,+1,0) | FRD = (0,−1,0) | 右在 FRD 里是 +y ✅ |

**这张表就是你的验收标准。** M3 手持测试时，拿它逐条对，一条不对就别往下走。

## 7. 三个致命陷阱

### 7.1 陷阱一：偏航不是"加个常数偏移"

很多人第一反应是"地图和 PX4 的朝向差一个固定角度，加个 offset 就行"。

**错。** 因为 ENU 的 z 轴朝**上**、NED 的 z 轴朝**下**，"正方向旋转"在两边是**相反**的。所以两边偏航的关系是**镜像**：

```
yaw_px4 = π/2 − yaw_map     ← 减号！map 转逆时针，PX4 的 yaw 在减小
```

后果：如果你写成 `yaw_px4 = yaw_map + offset`，**飞机朝地图 +X 时看着是对的**（offset 刚好凑对），但**一旦它转过 30°，误差就变成 60°**，转 90° 误差 180°（掉头飞）。

⇒ **纪律：换算只允许通过一个函数 `map_to_px4_yaw()`，全工程不许出现第二处 yaw 换算。**

### 7.2 陷阱二：位置和速度的换算法则**不一样**

- **世界系**的位置和速度（地图系里的 m 和 m/s）：`(x, y, z) → (y, x, −z)`（换位 + z 取反）
- **机体系**的速度矢量：`(x, y, z) → (x, −y, −z)`（不换位，y 和 z 取反）

混淆这两个 → "前后左右全反"。这是本项目最常见的炸机原因之一。

### 7.3 陷阱三：offboard 下**打杆夺不回控制权**

PX4 官方文档原文（`COM_RC_OVERRIDE`）：

> "Controls whether stick movement ... causes a mode change to Position mode. **This is not enabled for offboard mode by default.**"

也就是说：**默认情况下，一旦进了 offboard，你推遥控器摇杆是没有反应的。** 只有切模式开关才有效。

⇒ **首飞前必须把 `COM_RC_OVERRIDE` 改成 3。** ✅ 已核实（`commander_params.c:432-446`）：`@bit 0` = auto 模式接管，**`@bit 1` = offboard 模式接管**；默认值是 **1**，也就是说 **offboard 那一位出厂就是关的**。要两处都允许接管就设 **3**。详见 §11.4。

---

# 第三部分 · 方案设计

## 8. 上行链路设计：雷达 SLAM → PX4 EKF2

### 8.1 数据流（保留你现有的一切，只加最后一段）

```
镭神 N10（USB CDC-ACM，/dev/wheeltec_laser，230400 8N1）
   │
lslidar_driver（现有，不改）  ──►  /scan（LaserScan）
   │
cartographer_node（现有，不改）
   ├─ 订阅 /scan
   ├─ 发布 TF  map ──► odom   ┐
   └─ 发布 TF  odom ──► base_link  ├─ 都是 50 Hz（pose_publish_period_sec = 20e-3）
   │
tf2 static: base_link ──► laser（全 0，现有）
   │
★ 新增：vision_pose 节点（50 Hz）
   ├─ lookup_transform("odom", "base_link")
   ├─ 换算成 NED/FRD（§6）
   └─ publish /fmu/in/vehicle_visual_odometry（VehicleOdometry）
   │
PX4 EKF2 融合  ──►  /fmu/out/vehicle_local_position、vehicle_odometry
   │
★ 新增：状态监视（订阅 estimator_status_flags 确认真的融合了）
```

### 8.2 关键决策与理由

| 决策 | 选择 | 理由 |
| --- | --- | --- |
| 用哪个 TF 作为 PX4 的位置源 | **`odom → base_link`**，不是 `map → base_link` | 前者是 Cartographer 的**局部 SLAM**输出，**不跳变**。你的 lua 里 `optimize_every_n_nodes = 0` 关了全局优化，但一旦将来打开，`map→odom` 会跳，喂给 EKF2 就是灾难。 |
| 位姿回灌走什么消息 | `/fmu/in/vehicle_visual_odometry` | 官方文档原文：**"EKF2 only subscribes to `vehicle_visual_odometry` topics"**。发到别的话题名 EKF2 收不到。 |
| `pose_frame` 填什么 | **`POSE_FRAME_FRD` (= 2)** | 官方定义："Constant arbitrary heading offset from True North." 地图朝向本来就是任意的，正好对应。 |
| 发布频率 | **50 Hz** | 官方原文："streamed at between **30 Hz (if containing covariances) and 50 Hz. If the message rate is too low, EKF2 will not fuse the external vision messages.**" Cartographer 的 TF 正好 50 Hz。 |
| 要不要把 PX4 的里程计喂回 TF | **绝对不要** | 现在 `odom→base_link` 的唯一发布者是 `cartographer_node`。一旦 PX4 也发同一个变换，tf2 会出现双发布者冲突/抖动。**PX4 只当 offboard 执行器，Cartographer 继续当 odom 的唯一权威。** |
| 要不要开 EKF2 的偏航融合 | **要**（`EKF2_EV_CTRL` 的 yaw 位） | 室内磁力计不可靠；开了之后 PX4 的航向基准锚定到 SLAM 上，§7.1 那个"任意朝向偏移"才有确定值。 |
| 高度源 | **外部视觉 + 气压计**（并设置 `EKF2_HGT_REF = Vision`） | 室内气压计漂移几十厘米；SLAM 的 z 更可信。 |

### 8.3 必须避开的坑：雷达驱动的"时钟跳变闩锁"

你现有驱动里有一段自加的加固代码（`lslidar_driver.cc:800-808`）：

> 相邻两帧扫描的时间间隔 `<= 0` 或 `> 0.5 s` → 置位 `scan_clock_jump_latched_ = true` → **永久停止发布 `/scan`，直到驱动重启**。日志 `"LaserScan clock discontinuity detected (%.3f s); scan output is stopped until the driver restarts."`

**为什么这对本计划是大雷**：一旦 `/scan` 停发 → Cartographer 没有输入 → TF 停止更新 → 位姿源消失 → EKF2 失去外部视觉 → **飞控触发 failsafe**。而且它**不自动恢复**。

触发风险：任何时间跳变——PX4 的时间同步、`prepare_uav_clock.sh`、NTP 校时、`use_sim_time`。

**计划内的对策**（列进 M3 的验收项）：
1. 在开车前先确认它对时间同步免疫：启动 Agent + 位姿节点后，连续观察 `/scan` 的 `ros2 topic hz` 10 分钟不掉。
2. 给这个闩锁加一个"上游健康检查"告警——一旦触发，树莓派主动**停止发布位姿**（让飞控干脆地 failsafe），而不是让飞控拿着越来越旧的位姿继续飞。
3. 长期方案：把闩锁改成可自动解除（或只在检测到真正异常时闩锁）。

### 8.4 顺便修掉的两个依赖问题（不紧急，但迁移时会绊你）

- `cartographer_config/package.xml` 里**缺 `tf2_ros` 声明**（launch 里用了 `tf2_ros/static_transform_publisher`）。干净环境 `rosdep install --from-paths src` 会漏装。
- `uav_bringup/package.xml` 里留着 `ros2_laser_scan_matcher` 的依赖，但那个节点**已经不在 launch 里了**（是死代码）。**不要恢复它**——它会在 `/odom` 话题和 `odom→base_link` TF 上和 Cartographer 正面冲突。

### 8.5 与仓库里已有那份迁移分析的对账（★ 有一处必须纠正）

仓库里 `docs/px4_migration_analysis_flight_controller_bridge.md`（957 行）是一份质量很高的旧工程拆解 + PX4 对照，结论与本计划书**大方向一致**（整体替换 `flight_controller_bridge`、V1/V2 线缆协议全部推翻、QoS 必须改、无位置源进不了 offboard、`COM_RC_OVERRIDE` 默认关闭、`timestamp` 必填微秒）。但有三处必须对账：

**① 硬错误：它写的外部位姿话题名不对。**

该文 §6.1 与 §7.2 两处都写 `/fmu/in/vehicle_odometry`。**PX4 侧没有这个订阅话题。**

- `vehicle_odometry` 在 PX4 的 `dds_topics.yaml` 里只出现在 **`publications`**（即 `/fmu/out/vehicle_odometry`，那是 EKF2 **输出**的估计量）。
- `subscriptions`（派 → 飞控）里能收 `VehicleOdometry` 的只有两个名字：**`/fmu/in/vehicle_visual_odometry`** 和 `/fmu/in/vehicle_mocap_odometry`。
- 官方文档原文：**"EKF2 only subscribes to `vehicle_visual_odometry` topics"**。

⇒ 发到 `/fmu/in/vehicle_odometry` **不报错，但什么都不会发生**——不在 `dds_topics.yaml` 白名单里的话题在 DDS 里根本不存在。这是最难查的一类失败。**必须用 `/fmu/in/vehicle_visual_odometry`。**

**② 频率口径**

该文 §7.2 写 `OffboardControlMode` 按 **50 Hz** 发。官方示例是 **10 Hz**（`create_wall_timer(100ms, timer_callback)`），硬门槛只有 **≥2 Hz**。50 Hz 无害但没必要（多占串口带宽、多占飞控 CPU）。**建议按官方 10 Hz。**

**③ 关于"串级 PID 冲突"：它列为"高风险"，本计划书的结论是"这是一个要拍死的选择"** —— 详见 §9.4。只要 offboard 用**速度模式**（`velocity = true`），PX4 的位置环就不参与，树莓派的 PID 是正常分工而非串级叠加。**它不是风险，是一个架构选择。**

---

## 9. 下行链路设计：任务指令 → PX4 offboard

### 9.1 数据流

```
你的任务 / PID ──► 设定点话题（地图系）
   │
★ 新增：offboard 节点（10 Hz 硬节拍，不可断）
   ├─ publish /fmu/in/offboard_control_mode   ← 心跳，≥2 Hz，实际发 10 Hz
   ├─ publish /fmu/in/trajectory_setpoint     ← 设定点（换算后的 NED/FRD）
   └─ 需要时 publish /fmu/in/vehicle_command  ← 切模式 / 解锁
   │
PX4 commander 守门 ──► mc_pos_control ──► ... ──► 电机
```

### 9.2 offboard 的三条硬规矩（官方原文核实）

1. **心跳必须是 `OffboardControlMode`，频率 ≥ 2 Hz**：
   > "PX4 requires that the vehicle is already receiving `OffboardControlMode` messages before it will arm in offboard mode, or before it will switch to offboard mode when flying. In addition, PX4 will switch out of offboard mode if the stream rate of `OffboardControlMode` messages drops below approximately **2 Hz**."
2. **必须连续喂够 1 秒**才会真正启用 offboard（不是"发一帧就进"）。
3. **心跳不能只在需要动的时候发**：官方明确举例——"要悬停时，`OffboardControlMode` 必须持续流，而 `TrajectorySetpoint` 只需要发一次"。

⇒ **实现纪律：心跳和设定点都放在同一个固定周期定时器里，周期 100 ms（10 Hz），无论有没有新指令都照发。** 这一条比任何控制算法都重要。

### 9.3 主循环的标准写法（官方示例，逐字）

```cpp
if (offboard_setpoint_counter_ == 10) {
    // 第 10 次心跳后：切 offboard 模式 + 解锁
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);   // param1=1 自定义, param2=6 OFFBOARD
    arm();
}
publish_offboard_control_mode();     // 心跳
publish_trajectory_setpoint();       // 设定点
if (offboard_setpoint_counter_ < 11) offboard_setpoint_counter_++;
```

**注意参数含义**：`VEHICLE_CMD_DO_SET_MODE` 的 `param1 = 1`（自定义模式）、`param2 = 6`（offboard）。而 `/fmu/out/vehicle_status` 里 `nav_state` 的 offboard 值是 **14**。**这是两个不同的编号体系，别混。**（`nav_state` 全表见 §11.5 附件）

### 9.4 两种控制模式：怎么选

`OffboardControlMode` 里**第一个被置真的字段**决定 PX4 需要什么估计量、以及你要填哪个设定点消息。对多旋翼常用的两种：

| 模式 | `OffboardControlMode` 置位 | 设定点消息字段 | PX4 需要 | 适用场景 |
| --- | --- | --- | --- | --- |
| **速度控制** | `velocity = true` | `TrajectorySetpoint.velocity`（NED），`position` 填 **NaN** | 速度估计 | **推荐先用这个**。你的 PID 天然输出速度；调试期更宽容 |
| **位置控制** | `position = true` | `TrajectorySetpoint.position`（NED），`velocity` 填 **NaN** | 位置估计 | 定点悬停、航点飞行。飞控的位置环比你自己写的稳 |

> **重要细节**：`TrajectorySetpoint` 里**不控制的状态必须填 `NaN`，不能填 0。** 填 0 的意思是"我要你去坐标原点"，不是"不用管"。这是极高频的错误。

**★ 必须先拍死的架构选择：位置环放哪一边？**

你旧工程里已经有 `position_pid_controller`（位置误差 → 速度指令 → `/pid/cmd_vel`），PX4 内部也有位置环。**两个位置环叠加 = 串级冲突 = 震荡**，所以必须二选一：

| 选择 | 树莓派做什么 | PX4 做什么 | 上层任务逻辑改动 | 结论 |
| --- | --- | --- | --- | --- |
| **A. 位置环留在树莓派** | `position_pid_controller` 照旧算速度；适配层把 `/pid/cmd_vel` 填进 `TrajectorySetpoint.velocity` | offboard **速度模式**：`mc_pos_control` 速度直通，**不生成自己的位置环** | **几乎为零** | ✅ **推荐** |
| **B. 位置环交给 PX4** | 只发目标点（`TrajectorySetpoint.position`），必须**摘掉** `position_pid_controller` | offboard 位置模式，飞控自己闭环 | 要重写任务层 | 定点更稳，但旧资产作废 |

**关键澄清**：`OffboardControlMode.velocity = true` 时 PX4 **不是**"也在跑位置环"，它是速度直通——把速度设定点直接转成姿态/推力。所以选择 A **不是串级冲突，而是正常分工**。把它说成"高风险"的常见来源，是把"发速度设定点"和"同时发位置设定点"混为一谈了。

**建议路线**：**M4 / M5 / M6 全都先走选择 A（速度控制）**——保留旧资产、改动最小、调试期更宽容。等任务层真的需要长距离航点飞行、且你愿意重写任务逻辑时，再评估切到 B。

### 9.5 上层的接口怎么设计（保持你的任务逻辑不用改）

你现有工程的架构里有一个**正确的位置**来插这个缺口：`flight_controller_bridge` 那个位置。计划是——

**做一个和下位机无关的"飞控适配层"**，对上层暴露两个稳定话题（与现有工程同名，方便迁移）：

| 话题 | 方向 | 内容 |
| --- | --- | --- |
| `/flight/command` | 上层 → 适配层 | 速度/位置/高度指令 + 动作（起飞、降落、悬停、返航） |
| `/flight/status` | 适配层 → 上层 | 位置、速度、姿态、模式、解锁状态、失效标志、链路健康 |

**下位机从"自定义串口 V2 协议 + 凌霄飞控"换成"uXRCE-DDS + PX4 offboard"，上层任务逻辑一行不改。**

这是整个方案里最有价值的一个决定——它把你的旧资产（建图链 + 任务状态机 + PID）全部保留下来，只替换最底下那 10%。

---

## 10. 未来的软件结构（先规划，不落地）

> 你说了先不要代码落地。这里只写**目录和职责**，作为 M2~M6 的施工图。

```
~/px4_ws/src/
├── px4_msgs/                      # git clone -b release/1.17 https://github.com/PX4/px4_msgs.git
├── px4_bridge/                    # ★ 本项目要写的包（唯一需要新写的）
│   ├── px4_bridge/
│   │   ├── frames.py              # 坐标系换算（唯一允许做 yaw 换算的地方）
│   │   ├── qos.py                 # PX4 专用 QoS（否则收不到任何东西）
│   │   ├── vision_pose.py         # TF ──► /fmu/in/vehicle_visual_odometry
│   │   ├── offboard.py            # 心跳 + 设定点 + 模式/解锁
│   │   └── flight_interface.py    # /flight/command、/flight/status 适配层
│   ├── config/px4_bridge.yaml
│   └── launch/px4_bridge.launch.py
└── （你现有的包：lslidar_driver、cartographer_config、map_coordinate_provider、position_pid_controller、waypoint_manager…）
```

**包依赖版本约定（已核实）**：

| 组件 | 版本 | 说明 |
| --- | --- | --- |
| Ubuntu | 24.04 | 你现在的 |
| ROS 2 | **Jazzy** | 你现在的 |
| `px4_msgs` | 分支 **`release/1.17`** | 已确认该分支存在（sha `86d8239e962f6939e05c3737784f60c02fa884db`） |
| Micro-XRCE-DDS **Agent** | **v2.4.3** | ★ PX4 1.17 的 XRCE 客户端基于 **v2.x**，与 v3.x Agent **不兼容**。装错的表现是"连上了但建不了 topic" |
| 飞控固件 | MicoAir743v2 官方 release（PX4 1.17） | 已核实 `CONFIG_MODULES_UXRCE_DDS_CLIENT=y`，**不用自己编译固件** |

---

# 第四部分 · 参数与配置计划

## 11. PX4 侧参数清单

> 全部在 QGroundControl → 参数（Parameters）里改。改完**通常需要重启飞控**。
> 标 **【待核实】** 的项，结论会随本轮参数核实补上。

### 11.1 链路（M1）

| 参数 | 建议值 | 作用 |
| --- | --- | --- |
| `UXRCE_DDS_CFG` | 选 **TELEM2**（在 QGC 下拉框里**按名称选**，不用记枚举值） | XRCE 客户端走哪个串口。源码 `src/modules/uxrce_dds_client/module.yaml` 的 `serial_config.port_config_param.name` 就是它 |
| **所选串口的波特率**（QGC 标签 "TELEM2 Baudrate"，参数名 **`SER_TEL2_BAUD`**） | **921600** | ★ **必须显式改，这是 M1 最容易漏的一项。** 模块启动行原文：`set UXRCE_DDS_ARGS "-t serial -d ${SERIAL_DEV} -b p:${BAUD_PARAM}"`，`${BAUD_PARAM}` 会展开成**所选串口的波特率参数名**——选 TELEM2 就吃 `SER_TEL2_BAUD`。默认一般只有 57600，与 Agent 的 `-b 921600` 不一致时**一个 topic 都收不到，且不报错** |
| `MAV_1_CONFIG` | **0** | ★ 官方默认 **MAV_1 = TELEM2** 给伴飞电脑发 MAVLink。**必须禁掉，否则和 XRCE 抢同一个串口**。原文："Disable by setting `MAV_0_CONFIG=0` or `MAV_1_CONFIG=0` to zero." |
| `UXRCE_DDS_SYNCT` | **1**（默认，别动） | 时间戳同步。关了的话派的时间戳进不了飞控时基 |
| `UXRCE_DDS_DOM_ID` | 0（默认） | DDS 域，和派上的 `ROS_DOMAIN_ID` 一致 |
| `UXRCE_DDS_KEY` | 1（默认） | 多机时每机必须唯一 |

### 11.2 EKF2 外部视觉（M3）

| 参数 | 建议值 | 逐字依据 |
| --- | --- | --- |
| `EKF2_EV_CTRL` | 分步：**先 3**（水平位置+垂直位置），稳定后 **加 yaw 位 → 11**，最后再考虑加速度位 → 15 | 官方定义 bitmask：`0: Horizontal position / 1: Vertical position / 2: 3D velocity / 3: Yaw`，**default 0**（即默认完全关闭） |
| `EKF2_HGT_REF` | **3 = Vision** | ✅ **已核实（v1.17.0）**：定义在 **`src/modules/ekf2/module.yaml:87-104`**，**不在任何 `params_*.yaml` 里**（所以按文件搜是搜不到的，这是它一度被当成"已删除"的原因）。`type: enum`，values 逐字 `0: Barometric pressure` / `1: GPS` / `2: Range sensor` / `3: Vision`，**`default: 1`（= GPS）**、`reboot_required: true`。★ **你没有 GPS，所以默认值 1 对你没有意义，必须显式改成 3。** long 原文里有一句室内必须知道的话：*"The range sensor and vision options should only be used when for operation over a flat surface as the local NED origin will move up and down with ground level."* ⇒ **用视觉做高度基准时，本地 NED 原点会跟着地面上下移动**；地面不平、或落点在小车上时，高度基准会跟着晃。<br>⚠ 残留未核实：C++ 消费端（`EKF2.cpp` / `EKF/height_control.cpp`）到底哪一行采用 vision 作高度基准**尚未读代码确认**，不能完全排除"YAML 定义了但代码已忽略" |
| `EKF2_EV_DELAY` | 起手 **30 ms**，再按 §13 的方法调 | "delay from the time of the position measurement to the time the message is received by the flight controller"。**这是最重要的调参项** |
| `EKF2_EV_NOISE_MD` | 0（默认） | "EV reported variance (parameter lower bound)"。即用我们报的方差，且有参数下限兜底 |
| `EKF2_EVP_NOISE` | 0.1 m（默认起步） | 水平位置噪声下限 |
| `EKF2_EVA_NOISE` | 0.1 rad（默认起步） | 姿态噪声下限 |
| `EKF2_EV_QMIN` | 0（默认） | "External vision will only be **started and fused if the quality metric is above this threshold**"。**注意：因为默认是 0、判定是"greater than"，所以 `quality` 不能填 0** ⇒ 位姿有效时填 100，无效时填 0 |
| `EKF2_EV_POS_X/Y/Z` | 0.0 | 视觉源相对重心的位置。雷达在重心正上方的话不为 0，见 §13.4 |

### 11.3 高度源（M3）

| 参数 | 建议值 | 说明 |
| --- | --- | --- |
| `EKF2_BARO_CTRL` | 1（默认） | 气压计辅助高度。室内漂移大，但能提供长时稳定 |
| `EKF2_RNG_CTRL` | 若装了测距仪：**1**（默认，conditional mode） | ★ 注意 conditional mode 只在**水平速度 < 1.0 m/s 且高度 < 5.0 m** 时才融合测距。超了会**静默停用**。要全程用就设 **2** |
| `EKF2_OF_CTRL` | 若装了光流：1（**默认已经是 1**） | 注意它和 `EKF2_EV_CTRL` 默认值相反——光流默认就开着 |

### 11.4 安全与失效（★ 首飞前逐条走查）

> ✅ 已对 **v1.17.0 源码逐字核实**（源码原件已缓存到 `D:\09_PX4\_px4_v1170\`）。

| 参数 | 建议值 | 源码位置与为什么 |
| --- | --- | --- |
| `COM_RC_OVERRIDE` | ★ **改成 3** | `src/modules/commander/commander_params.c:432-446`，`PARAM_DEFINE_INT32(COM_RC_OVERRIDE, 1)`，min 0 max 3。**bit0 = 自动模式下允许摇杆接管；bit1 = offboard 模式下允许摇杆接管。默认值 1 = 只开了 bit0，offboard 那位是关的** ⇒ **默认进了 offboard，推杆完全没反应。** 要在 offboard 下也能夺回控制权必须设 **3**。行为原文：*"moving the sticks more than `COM_RC_STICK_OV` immediately gives control back to the pilot by **switching to Position mode**, and if position is unavailable Altitude mode."* 位枚举源码 `Commander.hpp:211-213`（`AUTO_MODE_BIT=(1<<0)`、`OFFBOARD_MODE_BIT=(1<<1)`），使用点 `Commander.cpp:2946`（auto）/ `:2952`（offboard） |
| `COM_RC_STICK_OV` | 30（默认，单位 %） | `commander_params.c:448-461`，`PARAM_DEFINE_FLOAT(..., 30.0f)`，min 5 max 80。打杆超过这个百分比才触发接管 |
| `COM_RCL_EXCEPT` | 需要时置 **4** | `commander_params.c:618-633`，default 0，min 0 max 31。位值逐字：`bit0 Mission=1` / `bit1 Hold=2` / **`bit2 Offboard=4`** / `bit3 External Mode=8` / `bit4 Altitude Cruise=16`。置 4 = **offboard 模式下遥控丢失不触发 failsafe**。原文警告：*"External modes requiring stick input will still failsafe."* |
| `COM_OBL_RC_ACT` | 0（Position） | `commander_params.c:353-369`，default 0。**枚举逐字是 8 个（不是我早前写的 5 个）**：`0 Position mode` / `1 Altitude mode` / `2 Stabilized` / `3 Return` / `4 Land` / `5 Hold` / `6 Terminate` / `7 Disarm`。⚠ **2 是 Stabilized 不是 Manual**；`6 Terminate`、`7 Disarm` 是危险值，别设 |
| `COM_OF_LOSS_T` | **1.0**（默认，秒） | `commander_params.c:329-340`，`PARAM_DEFINE_FLOAT(..., 1.0f)`，min 0 max 60。**这就是 offboard 断流后多久触发 failsafe。** 原文：*"The offboard loss failsafe will only be entered after a timeout, set by `COM_OF_LOSS_T` in seconds."* 首飞保持 1.0，别放宽 |
| `NAV_DLL_ACT` | 0（默认，Disabled） | `commander_params.c:580-598`，default 0。枚举 `0 Disabled / 1 Hold / 2 Return / 3 Land / 5 Terminate / 6 Disarm`。★ **这一条修正了官方文档的说法**：`rcAndDataLinkCheck.cpp:80-81` 的判定原文是 `bool gcs_connection_required = _param_nav_dll_act.get() > 0;` ⇒ 默认 0 时 `gcs_connection_required=false`，**没接地面站并不阻止解锁**（只在事件里报一条 Info `"No connection to the ground control station"`）。但你**仍然需要遥控链路** |
| `COM_RC_IN_MODE` | 3（默认） | `commander_params.c:168-199`，default 3。枚举 `0 RC only / 1 MAVLink only / 2 RC or MAVLink with fallback / 3 RC or MAVLink keep first / 4 Disable manual control / 5~8 各种优先级`。**4 = 完全禁用摇杆输入，首飞绝不能用**（它还会把下面 `modeCheck.cpp:176-179` 的 `"No manual control input"` 从 Warning 升成 Error） |
| `COM_ARMABLE` | 1（默认） | `commander_params.c:980`，`0 Disallow arming / 1 Allow arming`。被置 0 时 `armPermissionCheck.cpp:38-53` 报 `"Vehicle is in safety configuration"` |
| `COM_ARM_WO_GPS` | 1（默认） | `commander_params.c:255`，`0 Deny arming / 1 Warning only / 2 Disabled`。室内无 GPS 场景 1 正好——警告但不阻止 |
| `COM_LOW_BAT_ACT` | 0（默认） | `commander_params.c:293`，`0 Warning / 2 Land mode / 3 Return at critical level, land at emergency level`。**你没有电源模块，这条暂时无效** |
| `COM_DISARM_PRFLT` | 10.0 s（默认） | `commander_params.c:243`。解锁后无操作多久自动上锁 |
| `COM_DISARM_LAND` | 2.0 s（默认） | `commander_params.c:227`。落地后多久自动上锁 |
| `COM_KILL_DISARM` | 5.0 s（默认） | `commander_params.c:747` |
| `COM_ARM_CHK_ESCS` | 0（默认） | `commander_params.c:699`。v1.17.0 **只有这一个 `COM_ARM_CHK_*`**，**没有** `COM_ARM_CHK_GPS` 之类 |
| `COM_ARM_AUTH_REQ`/`_ID`/`_MET`/`_TO` | 0 / 10 / 0 / 1（默认） | `commander_params.c:481` / `:490` / `:506` / `:519`。双人授权解锁，默认关闭 |
| `COM_ARM_IMU_ACC`/`_GYR`、`COM_ARM_MAG_ANG`/`_STR` | 0.7 / 0.25 / 60 / 2（默认） | `commander_params.c:392` / `:404` / `:416` / `:430` |

#### ★ 同一个 Offboard，有三个不同的数字——别混

| 场合 | 编号 | 出处 |
| --- | --- | --- |
| MAVLink `VEHICLE_CMD_DO_SET_MODE` 的 **`param2`** | **6** | MAVLink 的 `PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6`。官方 ROS 2 示例就是 `VEHICLE_CMD_DO_SET_MODE, 1, 6` |
| `VEHICLE_CMD_SET_NAV_STATE` 的 `param1` = `nav_state`（也就是 `/fmu/out/vehicle_status.nav_state`） | **14** | `msg/versioned/VehicleStatus.msg` |
| QGC 飞行模式槽位 / `COM_FLTMODE1~6` 参数枚举 | **7** | `src/modules/commander/module.yaml:18-52`：`0 Manual / 1 Altitude / 2 Position / 9 Position Slow / 3 Mission / 4 Hold / 10 Takeoff / 11 Land / 5 Return / 6 Acro / 7 Offboard / 8 Stabilized / 12 Follow Me / 13 Precision Land / 16 Altitude Cruise / 100~107 External Mode 1~8 / -1 Unassigned`；**`COM_FLTMODE1~6` 默认全部是 −1（Unassigned）** |

⇒ **MAVLink 发 6，读状态看 14，QGC 里选的是 7。**

#### 室内无 GPS 时切 offboard 到底被哪一条挡下（已定位到源码）

`src/modules/commander/HealthAndArmingChecks/checks/` 下共 **35 个检查**。**没有 `localPositionCheck.cpp`**（我早前的猜测是错的）——位置估计检查就叫 **`estimatorCheck.cpp`**，另有 `homePositionCheck.cpp`。你会在 QGC 消息栏看到的原文与等级：

| 报错文本（逐字） | 源码位置 | 等级 | 触发条件 |
| --- | --- | --- | --- |
| `"No offboard signal"` | `modeCheck.cpp:144-153`，并 `clearCanRunBits(mode_req_offboard_signal)` | Error | ★ **最常见**。块注释原文：*"The offboard component is not sending setpoints or the required estimate (e.g. position) is missing."* ⇒ 要么心跳没发，要么位置估计无效 |
| `"Navigation error: No valid position estimate"` | `modeCheck.cpp:85`（外层 `if` 在 83 行） | Error | `failsafe_flags.local_position_invalid` 为真。**室内无 GPS 无光流切 offboard，就是被这一条 + 上面那条挡下** |
| `"Navigation error: No valid altitude estimate"` | `modeCheck.cpp:117-120` | **Critical** | 高度估计无效 |
| `"No valid attitude estimate"` | `modeCheck.cpp:61-64` | **Critical** | 姿态无效 |
| `"Angular velocity not valid"` | `modeCheck.cpp:50-53` | **Critical** | 角速度无效 |
| `"Navigation error: No valid global position estimate"` | `modeCheck.cpp:100-110` | Error | 需要全局位置且无效 |
| `"No manual control input"` | `modeCheck.cpp:176-179` | Warning（`COM_RC_IN_MODE==4` 时升为 Error） | 没有遥控输入 |
| `"Home position not set"` | `modeCheck.cpp:158-161` | Info | 室内无 GPS 会有这条，但只是 Info，不挡 |
| `"Position estimate has low accuracy"` | `estimatorCheck.cpp:637` | Error | `eph` 超阈值 |
| `"Vehicle is in safety configuration"` | `armPermissionCheck.cpp:38-53` | Error | `COM_ARMABLE < 1` |
| `"Estimated position error is approaching the failsafe threshold"` | `estimatorCheck.cpp:707-718` | Error | 由 `cs_inertial_dead_reckoning`（`EstimatorStatusFlags.msg:36`）+ `cs_wind_dead_reckoning`（`:37`）合成，见 `estimatorCheck.cpp:701-702` |

**排障时直接查的三个判定点**

1. **本地位置无效的准确算法**在 `estimatorCheck.cpp:733-735`：
   ```cpp
   failsafe_flags.local_position_invalid =
       !checkPosVelValidity(now, xy_valid, lpos.eph, lpos_eph_threshold, lpos.timestamp,
                    _last_lpos_fail_time_us, !failsafe_flags.local_position_invalid);
   ```
   阈值 `lpos_eph_threshold = (COM_POS_FS_EPH < 0) ? INFINITY : COM_POS_FS_EPH`（`:664`），**`COM_POS_FS_EPH` 默认 5.0 m**（`commander_params.c:538`），带 **2.5× 迟滞**（`:803`）。
2. **offboard 通道自身的监控**在 `offboardCheck.cpp:40-66`：它**不报消息**，只写 `offboard_control_signal_lost`。`:53-62` 表明 —— `OffboardControlMode.position=true` 要求 `local_position_invalid==false`；`velocity=true` 要求 `local_velocity_invalid==false`；`acceleration=true` 要求 `attitude_invalid==false`。超时时间就是 `COM_OF_LOSS_T`。
   ⇒ **这就是"为什么 M3 位置源必须先于 M4 offboard"的源码级证据。**
3. **拒绝换模式的入口**：`ModeManagement::getModeStatus(uint32_t &valid_nav_state_mask, uint32_t &can_set_nav_state_mask) const`（`ModeManagement.cpp:631`，被 `Commander.cpp:1943` 调用）；真正清位的是 `ModeChecks::checkAndReport(const Context&, Report&)`（`modeCheck.cpp:36`，`clearCanRunBits` 见 53/64/86/109/120/141/152/161/181/186/192 行）。模式替换：`ModeManagement::getNavStateReplacementIfValid`（`:437`）、`getReplacedModeIfAny`（`:463`）。
4. **解锁入口与总门禁**：`Commander::arm(arm_disarm_reason_t calling_reason, bool run_preflight_checks)`（`Commander.cpp:549`），健康门在 `:610-618`：
   ```cpp
   610:  _health_and_arming_checks.update(false, true);
   612:  if (!_health_and_arming_checks.canArm(_vehicle_status.nav_state)) {
   614:      mavlink_log_critical(&_mavlink_log_pub, "Arming denied: Resolve system health failures first\t");
   617:      return TRANSITION_DENIED;
   ```
   运行时判定在 `:1869-1874`，结果写进 `_vehicle_status.pre_flight_checks_pass`；`:1920` `_actuator_armed.ready_to_arm = _vehicle_status.pre_flight_checks_pass || isArmed();`
   ⇒ **`vehicle_command_ack` 返回被拒时，QGC 消息栏一定有条 `"Arming denied: ..."` 或上面表格里的具体检查项，照着查。**

### 11.5 `nav_state` 枚举对照（调试时必须会看，逐字核实）

`/fmu/out/vehicle_status` 的 `nav_state` 字段：

| 值 | 模式 | 值 | 模式 |
| --- | --- | --- | --- |
| 0 | MANUAL | 15 | STAB |
| 1 | ALTCTL | 16 | FREE1 |
| 2 | POSCTL | 17 | AUTO_TAKEOFF |
| 3 | AUTO_MISSION | 18 | AUTO_LAND |
| 4 | AUTO_LOITER | 19 | AUTO_FOLLOW_TARGET |
| 5 | AUTO_RTL | 20 | AUTO_PRECLAND |
| 6 | POSITION_SLOW | 21 | ORBIT |
| 7 | FREE5 | 22 | AUTO_VTOL_TAKEOFF |
| 8 | ALTITUDE_CRUISE | 23~30 | EXTERNAL1~EXTERNAL8 |
| 9 | FREE3 | 31 | MAX |
| 10 | ACRO | | |
| 11 | FREE2 | | |
| 12 | DESCEND | | |
| 13 | TERMINATION | | |
| **14** | **OFFBOARD** ← 你要的 | | |

`arming_state`：**1 = DISARMED，2 = ARMED**。

### 11.6 控制限幅（M5 起调，建议室内先降下来）

`MPC_XY_VEL_MAX`、`MPC_Z_VEL_MAX_UP/DN`、`MPC_ACC_HOR`、`MPC_TILTMAX_AIR`、`MPC_ALT_MODE`（**室内定高的关键**）等。

> 这一组的逐条定义与室内推荐值 **【待核实中】**，本轮已派出核实任务，结果会补进本节。

### 11.7 数据在哪看（派侧的"仪表盘"）

| topic | 看什么 | rate_limit |
| --- | --- | --- |
| `/fmu/out/vehicle_status` | `nav_state`（14=offboard）、`arming_state`、`failsafe`、`pre_flight_checks_pass` | 5 Hz |
| `/fmu/out/vehicle_control_mode` | `flag_control_offboard_enabled` | 50 Hz |
| `/fmu/out/estimator_status_flags` | **`cs_ev_pos` / `cs_ev_hgt` / `cs_ev_yaw` / `cs_ev_vel`** —— 确认外部视觉**真的被融合了** | 5 Hz |
| `/fmu/out/vehicle_local_position` | `x/y/z`（NED，m）、`vx/vy/vz`、`heading`、`xy_valid`、`z_valid`、`dist_bottom` | 50 Hz |
| `/fmu/out/vehicle_command_ack` | 命令结果 | |
| `/fmu/out/timesync_status` | 时间同步偏移 | 10 Hz |

> **只有 `cs_ev_pos = true` 才说明 EKF2 真的在用你的位姿。** 光看"话题收到了"不算数。

---

## 12. 树莓派侧清单

| 项 | 现状 | 要做 |
| --- | --- | --- |
| 系统 | ✅ Ubuntu 24.04.4 / ROS 2 Jazzy / aarch64 | 无需改动 |
| 雷达规则 | ✅ `/etc/udev/rules.d/wheeltec_lidar.rules` → `/dev/wheeltec_laser` | 无需改动 |
| 飞控规则 | ✅ `/etc/udev/rules.d/99-uav-flight-controller.rules` → `/dev/flight_controller` | 无需改动（CH340 按物理 USB 口绑定） |
| 雷达波特率 | 230400 8N1 | 已确认（驱动 `lslidar_driver.cc`） |
| **Agent** | ❌ **完全没装** | 编译安装 **v2.4.3**，做成 systemd 服务开机自启 |
| `px4_msgs` | ❌ 没有 | clone `release/1.17` |
| `px4_bridge` 包 | ❌ 没有 | M2 起写 |
| 磁盘 | 88 GB 可用 | ✅ 够 |
| **网络** | ⚠ wlan0 + **Meta TUN 198.18.0.1/30（Clash Verge）** | ★ **DDS 组播发现会被 TUN 干扰**。跑 ROS 2 前先处理：关掉 Clash 的 TUN，或用 `ROS_LOCALHOST_ONLY=1`（仅本机节点间通信时可行） |
| **`/dev/ttyAMA0`** | ⚠ 被摄像头桥占用（`UAV_CAMERA_SERIAL_DEVICE=/dev/ttyAMA0`） | 如果将来要用 GPIO 串口直连飞控，得先腾出这个口。**当前方案用 CH340 USB 转接，不冲突** |
| 开机服务 | ⚠ `uav-navigation.service` enabled 但 failed（雷达没插，30 秒超时退出） | 旧工程的服务，改造完成后要重新规划 |

**接线计划**（当前硬件状态：飞控和雷达都还没接）：

```
飞控 TELEM2（UART4 / /dev/ttyS3）
   │  4 线：TX / RX / GND / 5V（★ 5V 只给转接板，不给派）
   ▼
CH340 USB-TTL 转接板
   │  USB
   ▼
树莓派 5 USB 口（现有 udev 规则锁定为 /dev/flight_controller）
```

⚠ 三个硬件纪律（配套文档有详述）：
1. **飞控的 5V BEC 只有 3 A，带不动树莓派 5** → 派必须独立供电。
2. **必须共地**。
3. **TELEM4 / UART8 被板载蓝牙占用，别用那个口。**

---

# 第五部分 · 实施计划

## 13. 里程碑总表

| 里程碑 | 目标 | 关键判据 | 风险等级 |
| --- | --- | --- | --- |
| **M0** | 飞控本体能用（不接派） | ✅ **已完成**：飞控可用、遥控切模式正常 | — |
| **M1** | 派能**读**飞控 | `ros2 topic list` 出现 `/fmu/out/*`；`vehicle_status` 约 5 Hz | 中（串口/参数） |
| **M2** | 派能**命令**飞控（不解锁） | 发 `DO_SET_MODE` 能收到 `vehicle_command_ack` | 中 |
| **M3** | **位置源打通**：EKF2 融合外部视觉 | `cs_ev_pos = true`；手持走一圈位置方向正确 | **高（坐标系）** |
| **M4** | offboard 最小闭环（**拆桨**） | `nav_state = 14`；`flag_control_offboard_enabled = true`；电机按指令转 | 中 |
| **M5** | 无 GPS 定点悬停（**系留/护栏**） | offboard 速度=0 悬停 30 s，位置漂移 < 0.3 m | **高** |
| **M6** | 任务层接入 | 位置闭环移动到指定位点，静态误差 < 0.2 m | 中 |

**顺序不可调换。** 尤其 **M3 必须在 M4 之前**——没有位置源，offboard 是切不进去的（§3）。

> **当前进度**：**M0 已完成** ⇒ 下一步进入 **M1**（派能读飞控）。M1 是纯软件 + 一根线，不需要动力系统；§19 的三件事可以并行先做（在派上装 Agent、编译 `px4_msgs`、修雷达驱动的时钟跳变闩锁）。

---

## 14. 每阶段详述

### M0 · 飞控本体能用（不接派）—— ✅ **已完成**

> **用户确认**：飞控已验证可用，遥控器切换模式也正常。

这一关的意义是拿到一个**可信基线**：H743 上的 PX4 1.17 是健康的、遥控链路是通的、模式开关真的会改 `nav_state`。已达成。

**⚠ 但"能用"里没体现的三项检查，务必在 M4 之前补掉**（都不需要新硬件，各约 10 分钟）：

| 遗留检查 | 怎么做 | 为什么必须在 M4 之前做掉 |
| --- | --- | --- |
| **failsafe 实走一遍** | 解锁状态下关掉遥控器，看 QGC 里 `failsafe` 是否置位、模式是否自动切换 | 这是保命链的最后一环。M5 悬停时你唯一能依靠的就是它，"理论上有"不算数 |
| **确认 Position 模式现在不生效** | 切到 Position，看 QGC 是否给出"需要位置估计"之类的告警 | 室内无 GPS、无光流时它**本来就不该生效**。先亲眼确认"现在还不行"，M3 之后才能确信"现在行了" |
| **只留遥控、拔掉 QGC，能否解锁** | 拔掉 QGC，仅用遥控尝试解锁 | 官方明确警告：默认必须有地面站或遥控链路才能解锁。这决定你以后能不能脱开电脑飞 |

**注意**：**此时没有位置源，Position 模式不会生效**——这是正常的，不要在这里浪费时间。

**另一个前置事实（需要你确认）**：如果这架飞机现在**还没有电机 / 电调 / 电源模块**，那么 M0 里"能手动飞"这一条本来就没法验证，**M4 / M5 也会被硬件卡住**。但这**不影响 M1 / M2 / M3**——它们完全不需要动力系统，而且恰好是整个工程里风险最高、最该先啃的部分。所以即使动力系统还没到位，**现在就可以开始 M1**。

---

### M1 · 派能读飞控（★ 整个工程的地基）

**目标**：树莓派上 `ros2 topic list` 能看到 `/fmu/out/*`。

**动作**
1. **接线**：飞控 TELEM2 ↔ CH340 ↔ 派 USB。确认 `/dev/flight_controller` 出现。
2. **飞控参数**：`UXRCE_DDS_CFG` = TELEM2；`MAV_1_CONFIG = 0`（★ 关键，否则抢串口）。
3. **派上装 Agent v2.4.3**（源码编译；snap 的 stable 版能连但建不了 topic，必须 `--edge`）：
   ```
   git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
   cd Micro-XRCE-DDS-Agent && mkdir build && cd build
   cmake .. && make -j4 && sudo make install && sudo ldconfig /usr/local/lib/
   ```
4. **手工起 Agent**：`MicroXRCEAgent serial --dev /dev/flight_controller -b 921600`
   （★ 官方文档写成 `/dev/AMA0` 是笔误）
5. **验证**：另开一个 ssh 窗口跑 `ros2 topic list | grep fmu`。

**判据**
- `ros2 topic list` 出现 `/fmu/in/...` 和 `/fmu/out/...`
- `ros2 topic hz /fmu/out/vehicle_status` ≈ 5 Hz
- 拨 CH5 模式开关，`ros2 topic echo /fmu/out/vehicle_status --field nav_state` 跟着变

**风险 / 常见失败**
| 现象 | 原因 |
| --- | --- |
| 一个 topic 都没有 | Agent 版本是 v3.x（必须 v2.4.3）；或 Agent 没起来 |
| Agent 报串口打不开 | 波特率不对；或 `sudo` 权限（用户要在 `dialout` 组） |
| 部分 topic 收不到 | 该 topic **不在 `dds_topics.yaml` 白名单里** → 静默不存在，不报错 |
| `ros2 topic hz` 显示 0 | Clash TUN 干扰 DDS 发现 → 关 TUN |

---

### M2 · 派能命令飞控（不解锁）

**目标**：打通下行，但**不解锁、不飞**。

**动作**
1. clone `px4_msgs` 的 `release/1.17` 分支，`colcon build`。
2. 写一个最小节点：发 `VehicleCommand{command = DO_SET_MODE(176), param1 = 1, param2 = 6}`。
3. 订阅 `/fmu/out/vehicle_command_ack` 看结果。
4. ★ **QoS 必须对**，否则什么都收不到：
   - 订阅 PX4 的发布（`/fmu/out/*`）：**BEST_EFFORT + TRANSIENT_LOCAL + KEEP_LAST**
   - 发布给 PX4 的订阅（`/fmu/in/*`）：**BEST_EFFORT + VOLATILE + KEEP_LAST(1)**
   - ROS 2 默认是 reliable，**和 PX4 不兼容**，会静默收不到。

**判据**：`vehicle_command_ack` 返回结果码；**因为没解锁，模式切不过去是正常的**——这一步只验证"命令通道通了"。

**风险**：如果在这里就发现"发了没反应"，先查 QoS，再查 topic 名，再查飞控是否在跑 `uxrce_dds_client`。

---

### M3 · 位置源打通（★ 最高风险，必须手持验证）

**目标**：让 EKF2 相信"外部视觉"给出的位姿。

**动作**
1. 派上确认建图链正常：`/scan` 有数据、`ros2 run tf2_tools view_frames` 看到 `odom→base_link` 存在且约 50 Hz。
2. 写位姿节点：50 Hz 查 TF `odom→base_link` → 按 §6 换算 → 发 `/fmu/in/vehicle_visual_odometry`。
   - `pose_frame = POSE_FRAME_FRD (2)`
   - `quality = 100`（位姿有效时）；位姿过期就**停止发布**（让飞控干脆地 failsafe）
   - 时间戳：`int(node.get_clock().now().nanoseconds / 1000)`（`UXRCE_DDS_SYNCT = 1` 会让飞控自己算偏移）
3. 飞控参数：`EKF2_EV_CTRL = 3`、`EKF2_HGT_REF = Vision`、`EKF2_EV_DELAY = 30`。
4. **拆桨**，手持无人机，在场地里走 + 转。

**判据（逐条过，一条不对就别往下走）**
| 检查 | 期望 |
| --- | --- |
| `/fmu/out/estimator_status_flags` | **`cs_ev_pos = true`、`cs_ev_hgt = true`** ← 最关键 |
| 往地图 **+Y**（北）方向走 | `/fmu/out/vehicle_local_position` 的 `x`（NED 北）**增大** |
| 往地图 **+X**（东）方向走 | 该消息的 `y`（NED 东）**增大** |
| **举高** | 该消息的 `z` **变负**（NED，向下为正） |
| 机头从地图 +X 转向地图 +Y | `heading` 从 +π/2 走向 **0** |
| 静止不动 30 s | 估计位置不漂 > 0.3 m |

**★ 反向验证（很值得做）**：故意把换算代码写错（比如把 yaw 的 `π/2 − x` 改成 `π/2 + x`），观察判据是否失败——**如果写错了还"通过"，说明你的验收方法本身有问题**。

**风险**
| 现象 | 原因 |
| --- | --- |
| `cs_ev_pos` 一直是 false | 话题名不对（必须是 `vehicle_visual_odometry`）；`quality` 填了 0；频率低于 30 Hz |
| 位置方向反了 | §6/§7 的换算错了 —— **这是炸机原因第一名** |
| 位置抖动/发散 | `EKF2_EV_DELAY` 不对（试着 20/40/60 扫一遍） |
| 跑一会儿 `/scan` 停了 | §8.3 的时钟跳变闩锁被触发 |
| 换了个位置后位姿跳变 | 用了 `map→base_link` 而不是 `odom→base_link` |

---

### M4 · offboard 最小闭环（**拆桨！**）

**目标**：派发心跳 + 设定点，飞控能进 offboard 并解锁。

**动作**
1. 写 offboard 节点：10 Hz 定时器，雷打不动地发 `OffboardControlMode`（`velocity = true`）+ `TrajectorySetpoint`。
2. 先只发"速度为 0"的设定点。
3. 第 10 次心跳后发 `DO_SET_MODE(1, 6)` + 解锁。
4. ★ **拆掉螺旋桨**，在桌上跑。
5. **`COM_RC_OVERRIDE` 改成 3**（`bit1` = offboard 接管位，默认值 1 不含它）；`COM_OF_LOSS_T` 保持 **1.0 s**。

**判据**
- `/fmu/out/vehicle_status`：`nav_state = 14`（OFFBOARD）、`arming_state = 2`（ARMED）
- `/fmu/out/vehicle_control_mode`：`flag_control_offboard_enabled = true`
- 4 路电机输出随设定点变化（速度设定点给个小值看响应）
  - **没接电调/电机也能验这一条**：PX4 在未接电调时照样计算 PWM。用 QGC → MAVLink Inspector 看 `ACTUATOR_OUTPUTS`（走 MAVLink，不受 DDS 白名单限制），或在派上核对 `dds_topics.yaml` 白名单后订阅对应的 `actuator_*` topic。
  - ⚠ 不要为了"看电机转"而临时把电调接上又不装桨——**空载电调 + 螺旋桨座旋转本身就是危险源**。宁可只用 `ACTUATOR_OUTPUTS` 数值验证。
- **人为杀掉 offboard 节点** → 飞控应在 `COM_OF_LOSS_T` 后自动切出 offboard（这是保命功能的验证，必做）

**风险**
| 现象 | 原因 |
| --- | --- |
| 进不了 offboard | 没有位置估计（回 M3）；或没先发心脏跳 |
| 能解锁但立刻切出 offboard | 心跳频率 < 2 Hz；或心跳和设定点不在同一个定时器里，有断流 |
| 解锁被拒 | 预检查没过（看 `pre_flight_checks_pass`）；或没有 QGC/遥控链路（官方明确警告过） |

---

### M5 · 无 GPS 定点悬停（系留或护栏内）

**目标**：offboard 速度=0，飞机稳定悬停。

**动作**：逐步放开限幅（先把 `MPC_XY_VEL_MAX`、`MPC_TILTMAX_AIR` 调小），从"离地 20 cm 悬停 10 s"开始，一点点加时间和高度。

**判据**：连续 30 s 位置漂移 < 0.3 m、高度波动 < 0.3 m、`cs_ev_pos` 全程 true。

**调参顺序**（一次只动一个）
1. `EKF2_EV_DELAY`（对悬停抖动影响最大）
2. `EKF2_EVP_NOISE` / `EKF2_EVA_NOISE`
3. `MPC_XY_VEL_P/I/D`、`MPC_Z_VEL_*`
4. 雷达安装位置 `EKF2_EV_POS_X/Y/Z`（如果雷达不在重心正上方）

---

### M6 · 任务层接入

**目标**：把雷达定位 + 位置 PID + 任务状态机接到 offboard 上。

**动作**：做 §9.5 那个"飞控适配层"，对上层保持 `/flight/command`、`/flight/status` 不变，下层换成 PX4 offboard。

**判据**：地图上给一个目标点，飞机闭环过去，静态误差 < 0.2 m。

---

## 15. 试验矩阵（不许跳级）

| 级别 | 桨 | 位置 | 人在哪 | 允许做什么 |
| --- | --- | --- | --- | --- |
| T0 | **无桨** | 桌面 | 手边 | 一切：切模式、解锁、发设定点、杀节点看 failsafe |
| T1 | 无桨（手持机） | 场地 | 手举 | M3 全部验收项 |
| T2 | **系留 / 护栏内** | 离地 < 0.3 m | 手扶遥控，随时切模式 | 悬停、小速度移动 |
| T3 | 有桨 | 自由 | 遥控在手，模式开关可随时切出 | M5/M6 |

**铁律**：
1. **T0/T1 阶段绝对不装桨。**
2. **遥控器和模式开关必须在手边，且已确认能夺回控制权**（§7.3）。
3. 每次只改一个变量。
4. 出现任何异常，**第一反应是切模式开关**，不是去拔电。

---

## 16. 风险登记册

| # | 风险 | 后果 | 对策 |
| --- | --- | --- | --- |
| R1 | **坐标系换算错误** | 一解锁就朝错误方向冲出去 | M3 手持逐条验收（§14 M3 表）；只允许一处换算函数 |
| R2 | **offboard 下打杆夺不回控制权** | 失控 | 首飞前把 `COM_RC_OVERRIDE` **改成 3**（`bit1` 才是 Offboard 位，默认值 1 不含它） |
| R3 | 没有位置估计就切 offboard | 静默无效，白忙 | 先做 M3，用 `cs_ev_pos` 做硬门禁 |
| R4 | `quality` 填 0 导致 EKF2 拒绝 | 外部视觉完全不生效 | 有效时填 100 |
| R5 | 心跳断流 | 飞控切出 offboard（可能掉高度） | 心跳和设定点同一个 10 Hz 定时器，永不缺席；`COM_OF_LOSS_T` 先设 1 s |
| R6 | **雷达驱动时钟跳变闩锁** | `/scan` 永久停发 → 位姿源消失 → failsafe | §8.3 的三条对策；开机后观察 10 分钟 |
| R7 | QoS 不匹配 | 什么都收不到，还不报错 | 严格按 §14 M2 的 QoS 表 |
| R8 | Agent 版本 v3.x | 连上了但建不了 topic | 固定 v2.4.3 |
| R9 | Clash TUN 干扰 DDS 组播 | topic 时有时无 | 跑 ROS 2 前关 TUN |
| R10 | 飞控 5V BEC 只有 3 A | 派供电不足、随机重启 | 派独立供电 |
| R11 | 位姿过期仍继续发 | 飞控拿着旧位姿飞 | 位姿过期就**停止发布**，让飞控干脆失效 |
| R12 | `map→odom` 跳变被喂给 EKF2 | 位置瞬间跳变 | 只用 `odom→base_link` |
| R13 | 忘了 `MAV_1_CONFIG = 0` | XRCE 和 MAVLink 抢 TELEM2 | M1 参数清单里当必改项 |
| R14 | 非实时系统上做实时活 | 抖动 → 姿态抖 | 纪律：派只发设定点（§4） |

---

## 17. 【待核实】清单（本计划里我不能拍胸脯的部分）

| # | 待核实项 | 影响 | 状态 |
| --- | --- | --- | --- |
| Q1 | ~~`UXRCE_DDS_CFG` 的枚举值表与波特率设置位置~~ **✅ 已解决** | M1 全部 | **已解决**：枚举值不用记（QGC 按名称选 TELEM2）。波特率位置已定位——模块启动行 `-b p:${BAUD_PARAM}`，`${BAUD_PARAM}` 展开为 `SER_TEL2_BAUD`，**必须显式设 921600** |
| Q2 | ~~`COM_RC_OVERRIDE` 的完整位值表（Offboard 是第几位）~~ **✅ 已解决** | **安全关键** | **已解决**：`commander_params.c:432-446`，default **1**，min 0 max 3；`bit0`=auto 接管、**`bit1`=offboard 接管**。默认只开 bit0 ⇒ **必须改成 3**，否则 offboard 下推杆没反应 |
| Q3 | ~~`COM_RCL_EXCEPT` 的位值表~~ **✅ 已解决** | 安全 | **已解决**：`commander_params.c:618-633`，default 0，min 0 max 31；`bit0 Mission / bit1 Hold / bit2 Offboard(=4) / bit3 External Mode / bit4 Altitude Cruise` |
| Q4 | ~~`EKF2_HGT_REF` 在 v1.17.0 是否仍存在~~ **✅ 已解决**：仍在，`src/modules/ekf2/module.yaml:87-104`，enum `0 Barometric / 1 GPS / 2 Range sensor / 3 Vision`，**default 1(GPS)** | M3 | **已解决（有残留项）**：残留 = C++ 消费端（`EKF2.cpp` / `EKF/height_control.cpp`）未读，不能排除"YAML 定义了但代码已忽略" |
| Q5 | `MPC_ALT_MODE` 的完整枚举与室内推荐值 | M5 定高 | 核实中 |
| Q6 | `MPC_*` 位置/速度环参数逐条定义与室内推荐值 | M5 | 核实中 |
| Q7 | ~~解锁被拒时 commander 具体报哪条检查失败、阈值参数名~~ **✅ 已解决** | M4 排障 | **已解决**：见 §11.4 的报错对照表（11 条逐字消息 + 源码行 + 等级）。关键阈值 `COM_POS_FS_EPH` 默认 **5.0 m**（`commander_params.c:538`，判定在 `estimatorCheck.cpp:733`，2.5× 迟滞在 `:803`）；总门禁 `Commander.cpp:610-618`，拒绝时必打 `"Arming denied: Resolve system health failures first"`。`COM_OBL_ACT` / `COM_OBL_TIMEOUT` 在 v1.17.0 **不存在**；`localPositionCheck.cpp` **不存在**（位置检查=`estimatorCheck.cpp`）；`COM_ARM_CHK_*` 只有 `COM_ARM_CHK_ESCS` |
| Q8 | 镭神 N10 的**串口版硬件**接线定义、供电电压（波特率已确认 230400） | M3 接线 | 需你翻随货资料包 |
| Q9 | 雷达的实际外参（相对重心的 x/y/z、安装俯仰） → `EKF2_EV_POS_X/Y/Z` | M5 精度 | 需实测 |

---

## 18. 旧资产去留对照表

| 资产 | 去留 | 理由 |
| --- | --- | --- |
| `lslidar_driver` + `lsn10.yaml` | **保留** | 纯传感器链，与飞控无耦合 |
| `cartographer_config` + `n10_2d.lua` 的全部调参 | **保留** | 场地相关，与飞控无关 |
| `base_link → laser` 静态 TF | **保留** | |
| `map_coordinate_provider`（TF → PoseStamped） | **保留** | 但**注意它的 `map_frame` 配的是 `"odom"` 不是 `"map"`**，这个约定要延续 |
| 雷达驱动里的 `scan->header.stamp = start_time` 修复 | **必须保留** | 改回 `now()` 会让 Cartographer 丢点（"earlier points"） |
| 雷达驱动里的时钟跳变闩锁 | **改造** | 见 §8.3，风险 > 收益 |
| `csm` + `ros2_laser_scan_matcher` | **不恢复** | 已是死代码；恢复会和 Cartographer 抢 `/odom` 和 `odom→base_link` |
| `flight_controller_bridge`（自定义串口 V2 协议 + 凌霄飞控） | **替换** | 换成 uXRCE-DDS + PX4 offboard；但**保持 `/flight/command`、`/flight/status` 话题接口不变** |
| 任务状态机 / PID / 建图上层逻辑 | **保留** | 只要 §9.5 的适配层做对，一行不用改 |
| 全工程的自定义坐标变换（`coordinates.hpp` 按 FLU 写的 `R(-yaw)`） | **重写** | 必须按 §6 换成 NED/FRD |

---

## 19. 现在就能做的三件事

即使硬件还没接，这三件事可以立刻开始：

1. **在树莓派上装 Agent v2.4.3 并做成 systemd 服务**（§14 M1 第 3 步）。这一步不需要飞控。
2. **clone `px4_msgs` 的 `release/1.17` 分支并编译**，确认 `ros2 interface show px4_msgs/msg/VehicleOdometry` 有输出。
3. **把 §8.3 的时钟跳变闩锁改成可自动解除的版本**（在上游驱动里，纯软件，可以离线做），并顺手补上 `cartographer_config/package.xml` 缺失的 `tf2_ros` 依赖。

---

*本文档的所有坐标系结论均经数值验证；所有 PX4 参数与消息字段均取自 `PX4-Autopilot v1.17.0` 源码或官方文档，标 **【待核实】** 者除外。*
