# D题任务一摄像头顺序执行与直接投放协议（v1.7）

日期：2026-07-30  
适用任务：任务一——伴飞、向小车平台投放沙包、返回 H 点降落  
基础协议：`D_UAV_UNIFIED_DEVICE_PROTOCOL_v1.7.md`  
线缆协议版本字节：仍为 `0x01`，本文的“v1.7”是文档修订号，不是线上版本字节

> 本文是统一协议v1.7的task1摄像头专项实现说明。实现人员应先读“完整流程”，再按文末数据包表开发；帧头、帧尾、长度、校验和和字段位置均以统一协议v1.7为准。

---

## 1. 四台设备分别负责什么

### 1.1 树莓派：全局任务指挥者

树莓派决定当前处于任务一的哪一步，并通过 `VISION_COMMAND` 告诉摄像头：

- 现在是否启用视觉；
- 现在是搜索、伴飞还是投放对准；
- 当前全局任务阶段 `mission_phase`；
- 当前要识别小车平台还是投放中心；
- 何时正式允许摄像头驱动舵机投放。

摄像头不能自行把全局任务从“伴飞”改为“投放”，也不能因为偶然看到目标就自行投放。

### 1.2 摄像头：识别、相对位置和投放执行者

摄像头负责：

- 识别小车平台或平台投放中心；
- 把目标相对无人机的平面偏移转换成 BODY 坐标的毫米值；
- 通过 `VISION_DATA` 报告识别状态、偏移和对准状态；
- 只在树莓派进入 `TASK1_DROP(0x11)` 后，直接驱动舵机一次；
- 舵机动作序列结束后报告 `VISION_DROP_COMPLETED(0x08)`。

摄像头只发送平面 `x/y`，不发送 z、高度或速度。摄像头报告“投放完成”的含义仅为舵机角度和延时序列已执行完，不要求再次通过图像确认沙包已经离开。

### 1.3 小车：任务启动和 C 点投放窗口提供者

小车通过按键选择任务：

- `KEY1`：`session_id=1`，任务一；
- `KEY2`：`session_id=2`，任务二。

任务一中，小车持续报告经过 A/B/C/D 的信息。经过 C 点后，小车减速；只有 `C_DECEL_COMPLETE=1`，才表示小车已经减速到约定速度、投放窗口已经稳定。

### 1.4 飞控：执行无人机运动

树莓派根据摄像头的相对 `x/y` 计算速度，再通过独立的树莓派—飞控 V2 协议控制飞机。摄像头不直接给飞控发送速度。

---

## 2. 一句话总流程

```text
KEY1选择任务一
→ 约3秒倒计时
→ 小车和无人机同时开始
→ 无人机起飞到150±10 cm并稳定悬停3秒
→ 向右横移约0.75 m
→ 向前追车，同时摄像头搜索小车
→ 在B点前形成伴飞
→ 小车经过C点并减速完成
→ 摄像头对准小车平台投放中心
→ 树莓派授权TASK1_DROP
→ 摄像头直接驱动舵机一次
→ 摄像头报告投放完成
→ 树莓派控制无人机返回H点并降落
```

“投放中心”特指小车平台上黑色同心圆和十字线的中心，不是场地上的固定坐标点。

---

## 3. 任务一完整顺序流程

下面每一步都固定说明：谁控制、树莓派发什么、摄像头处于什么状态、什么条件触发动作、摄像头做什么、摄像头回什么、下一步是什么。

### 阶段 0：设备上电，等待任务

**控制来源**

树莓派尚未收到小车的有效任务选择，系统处于 `READY(0x00)`。

**树莓派 → 摄像头**

```text
enable           = 0
vision_mode      = DISABLED(0x00)
task_mode        = 0
mission_phase    = READY(0x00)
target_type      = NONE(0x00)
command_flags    = 0；如需清除上一局，仅发送一次RESET_TRACKER
requested_rate_hz= 20
```

**摄像头内部状态**

```text
drop_armed     = false
drop_started   = false
drop_completed = false
drop_failed    = false
vision_state   = DISABLED(0x00)
```

**摄像头动作**

不识别目标，不驱动舵机；保持串口接收和状态回传能力。

**摄像头 → 树莓派**

发送 `VISION_DATA`：

```text
mission_phase_echo = READY(0x00)
vision_state       = DISABLED(0x00)
target_type        = NONE(0x00)
vision_flags       = 0x00
relative_x_mm      = 0
relative_y_mm      = 0
```

---

### 阶段 1：KEY1 选择任务一，进入约 3 秒倒计时

**控制来源**

小车 KEY1 选择固定 `session_id=1`、`task_mode=TASK1(0x01)`。树莓派验证后进入 `COUNTDOWN(0x01)`。

**树莓派 → 摄像头**

新一局第一次命令：

```text
enable        = 0
vision_mode   = DISABLED(0x00)
task_mode     = TASK1(0x01)
mission_phase = COUNTDOWN(0x01)
target_type   = NONE(0x00)
command_flags = RESET_TRACKER(位0=1)，只脉冲一次
```

随后倒计时期间继续发送相同命令，但 `command_flags=0`。

**摄像头动作**

用新 `session_id=1` 清空上一局的跟踪器和四个投放锁存变量，不驱动舵机。

**摄像头 → 树莓派**

继续报告 `DISABLED`、零偏移；`mission_phase_echo=COUNTDOWN(0x01)`。

**进入下一步条件**

约 3 秒间歇结束，小车发出 START/任务开始状态，树莓派进入 `MISSION_START(0x02)`。

---

### 阶段 2：小车和无人机同时开始

**控制来源**

小车启动，树莓派开始无人机任务一主流程。

**树莓派 → 摄像头**

```text
enable        = 0
vision_mode   = DISABLED
task_mode     = TASK1
mission_phase = MISSION_START(0x02)
target_type   = NONE
```

**摄像头动作与回传**

仍不参与控制，回传 `DISABLED` 和零偏移。摄像头串口在线状态仍可被树莓派监测。

**进入下一步条件**

树莓派向飞控发出起飞动作后进入 `TAKEOFF_HOVER(0x03)`。

---

### 阶段 3：起飞到 150±10 cm，并稳定悬停 3 秒

**控制来源**

飞控执行起飞和高度保持；树莓派负责按赛题要求累计稳定悬停时间。

**树莓派 → 摄像头**

```text
enable        = 0
vision_mode   = DISABLED
mission_phase = TAKEOFF_HOVER(0x03)
target_type   = NONE
```

**摄像头动作与回传**

不参与高度判断，不发送高度或 z；回传 `DISABLED`、零偏移。

**进入下一步条件**

无人机处于 `150±10 cm`，并满足任务一规定的稳定悬停 3 秒后，树莓派进入右移阶段。

---

### 阶段 4：无人机向右横移

**控制来源**

树莓派按预设距离向右横移约 0.75 m。BODY 坐标约定为 x 前、y 左，所以向右为 `vy<0`。

**树莓派 → 摄像头**

```text
enable        = 0
vision_mode   = DISABLED
mission_phase = RIGHT_SHIFT(0x04)
target_type   = NONE
```

**摄像头动作与回传**

不参与右移距离判断，回传 `DISABLED`、零偏移。

**进入下一步条件**

树莓派判定右移完成后进入向前追车阶段。

---

### 阶段 5：向前追车，同时搜索小车

**控制来源**

树莓派控制飞机向前；摄像头开始搜索小车平台。

当前实现不额外停留在全局 `SEARCH_CAR(0x06)` 阶段。全局阶段保持 `FORWARD_CHASE(0x05)`，摄像头的局部工作模式设为 `SEARCH(0x01)`。这样不会把“飞行阶段”和“视觉内部状态”混为一谈。

**树莓派 → 摄像头**

```text
enable        = 1
vision_mode   = SEARCH(0x01)
task_mode     = TASK1
mission_phase = FORWARD_CHASE(0x05)
target_type   = CAR_PLATFORM(0x01)
```

**摄像头状态和动作**

- 尚未找到小车：`vision_state=SEARCHING(0x02)`；
- 找到并能输出有效平面偏移：`vision_state=TRACKING(0x03)`。

**摄像头 → 树莓派**

未找到目标、图像新鲜：

```text
vision_state  = SEARCHING(0x02)
vision_flags  = IMAGE_FRESH = 0x10
relative_x/y  = 0
```

找到目标：

```text
vision_state  = TRACKING(0x03)
target_type   = CAR_PLATFORM(0x01)
vision_flags  = TARGET_VALID | XY_VALID | IMAGE_FRESH = 0x13
relative_x_mm = 目标相对无人机BODY前向偏移
relative_y_mm = 目标相对无人机BODY左向偏移
```

**进入下一步条件**

目标连续有效，树莓派可以使用相对 `x/y` 进入伴飞闭环，并切换到 `FOLLOW_CAR(0x07)`。必须在小车经过 B 点前形成伴飞，以满足赛题要求。

---

### 阶段 6：伴随小车飞行

**控制来源**

树莓派用摄像头相对 `x/y` 计算水平 PID 速度并发送给飞控。偏航目标保持不变，正常情况下 yaw 速度为 0。

**树莓派 → 摄像头**

```text
enable        = 1
vision_mode   = FOLLOW(0x02)
task_mode     = TASK1
mission_phase = FOLLOW_CAR(0x07)
target_type   = CAR_PLATFORM(0x01)
```

**摄像头 → 树莓派**

正常伴飞时：

```text
vision_state  = TRACKING(0x03)
vision_flags  = 0x13
relative_x_mm = 有符号毫米值，x前为正
relative_y_mm = 有符号毫米值，y左为正
```

目标丢失时：

```text
vision_state  = TARGET_LOST(0x06)
vision_flags  = 0x10（仅图像仍新鲜时）
relative_x/y  = 0
```

树莓派收到目标丢失后不能继续使用旧偏移，应停止视觉速度或执行当前约定的搜索策略。

**进入下一步条件**

只有小车报告以下两个条件均成立，树莓派才允许从伴飞转到投放对准：

```text
最近经过点 = C
C_DECEL_COMPLETE = 1
```

仅仅经过 C 点但还没有减速完成时，继续伴飞，不开始投放对准。若 `last_checkpoint` 已更新为 D，则不再允许启动新的对准或投放动作，因为任务一要求在 D 点前完成投放。

---

### 阶段 7：C 点后减速完成，开始对准投放中心

**控制来源**

小车的 `C_DECEL_COMPLETE` 提供投放窗口锚点；树莓派正式进入 `TASK1_ALIGN_DROP(0x10)`；摄像头只负责测量和报告是否对准。

**树莓派 → 摄像头**

```text
enable        = 1
vision_mode   = DROP_ALIGN(0x03)
task_mode     = TASK1
mission_phase = TASK1_ALIGN_DROP(0x10)
target_type   = DROP_CENTER(0x03)
```

**摄像头必须做的事**

1. 确认命令属于当前 `session_id=1`；
2. 锁存 `drop_armed=true`，表示本局已经合法经过对准阶段；
3. 识别小车平台黑色同心圆和十字线的中心；
4. 输出该中心相对无人机的 BODY 平面 `x/y` 毫米偏移；
5. 在偏移进入摄像头自己的对准阈值并保持规定时间后，置 `DROP_ALIGNED`。

**摄像头 → 树莓派**

正在靠近投放中心：

```text
vision_state = TRACKING(0x03)
vision_flags = 0x13
x/y          = 当前有效偏移
```

已经对准：

```text
vision_state = DROP_ALIGNED(0x04)
vision_flags = TARGET_VALID | XY_VALID | DROP_ALIGNED | IMAGE_FRESH
             = 0x17
x/y          = 当前有效偏移
```

**严禁动作**

`mission_phase=0x10` 时，无论对得多准，摄像头都不得驱动舵机。这一步只对准，不投放。

**树莓派进入下一步的建议条件**

以下条件持续满足约 0.2 秒：

```text
小车最近经过C点
C_DECEL_COMPLETE=1
摄像头数据新鲜
TARGET_VALID=1
XY_VALID=1
DROP_ALIGNED=1
树莓派视觉PID达到允许误差
飞控通信、高度和控制状态正常
```

满足后，树莓派切换为 `TASK1_DROP(0x11)`。当前树莓派代码尚需把 `/vision/drop_aligned` 正式加入这一判定，见第 7 节。

---

### 阶段 8：树莓派正式授权投放，摄像头直接驱动舵机

**控制来源**

只有树莓派发出的 `TASK1_DROP(0x11)` 是投放授权。摄像头不能根据小车 C 点标志、图像对准或内部计时单独投放。

**树莓派 → 摄像头**

```text
enable        = 1
vision_mode   = DROP_ALIGN(0x03)
task_mode     = TASK1
mission_phase = TASK1_DROP(0x11)
target_type   = DROP_CENTER(0x03)
```

**允许舵机动作的全部条件**

```text
当前session_id = 1
本局曾收到同一session的TASK1_ALIGN_DROP(0x10)
drop_armed = true
当前命令为TASK1_DROP(0x11)
目标仍有效且图像新鲜
DROP_ALIGNED = 1
drop_started = false
drop_completed = false
drop_failed = false
```

全部成立时，摄像头只执行一次预设舵机角度和延时序列，并立即锁存：

```text
drop_started = true
vision_state = VISION_DROP_EXECUTING(0x07)
```

**摄像头 → 树莓派：执行中**

```text
mission_phase_echo = TASK1_DROP(0x11)
vision_state       = VISION_DROP_EXECUTING(0x07)
target_type        = DROP_CENTER(0x03)
vision_flags       = 0x17（目标和对准仍有效时）
relative_x/y       = 当前有效偏移
```

**重复命令处理**

树莓派会周期重发相同的 `TASK1_DROP`。摄像头必须重复报告状态，但绝不能重复驱动舵机。

---

### 阶段 9：投放完成或投放失败

#### 9.1 舵机动作完成

当摄像头的预设舵机角度和延时序列全部结束时：

```text
drop_completed = true
vision_state   = VISION_DROP_COMPLETED(0x08)
```

这里不增加“从画面确认沙包离开”的条件。官方是否得分仍取决于沙包是否实际落入小车平台黑色圆形区域，因此舵机机构和投放提前量需要通过实测校准。

**摄像头 → 树莓派：完成**

```text
mission_phase_echo = TASK1_DROP(0x11)
vision_state       = VISION_DROP_COMPLETED(0x08)
target_type        = DROP_CENTER(0x03)
vision_flags       = IMAGE_FRESH(0x10)，不再强制要求目标或XY有效
relative_x_mm      = 0
relative_y_mm      = 0
```

摄像头应锁存并周期发送 `0x08`，直到：

- 树莓派进入后续阶段；或
- 收到新 session；或
- 收到一次 `RESET_TRACKER`。

树莓派收到新鲜的 `0x08` 后发布：

```text
/payload/drop_completed = true
```

任务状态机随后进入 `TASK1_RETURN_H(0x12)`。

#### 9.2 舵机执行失败或超时

出现舵机驱动错误、内部异常，或执行时间超过约定超时：

```text
drop_failed = true
vision_state = VISION_DROP_FAILED(0x09)
```

回传时 `vision_flags` 至少按图像实际情况设置 `IMAGE_FRESH`；若同时存在视觉错误，再置 `VISION_ERROR`。树莓派不得把 `0x09` 当作投放完成；应记录ACTION_FAILED、撤销投放请求、禁止本轮重复投放并立即进入TASK1_RETURN_H。树莓派自身等待投放结果超时或投放阶段出现其他错误时也采用相同返航处理。

建议摄像头投放超时设置为“实测舵机完整动作最长时间 + 0.5 秒”。树莓派当前默认等待 10 秒；若舵机动作更慢，双方必须同步调整。

---

### 阶段 10：返回 H 点

**控制来源**

树莓派收到投放完成后控制无人机返回 H 点，并向小车广播任务一投放完成的协调标志。摄像头不再参与导航。

**树莓派 → 摄像头**

```text
enable        = 0
vision_mode   = DISABLED
mission_phase = TASK1_RETURN_H(0x12)
target_type   = NONE
```

**摄像头动作与回传**

停止识别，不再驱动舵机；回传 `DISABLED`、零偏移。内部 `drop_completed` 可以保留到新 session 或 RESET，防止阶段抖动导致重复投放。

---

### 阶段 11：在 H 点降落并完成任务一

**树莓派 → 摄像头**

降落阶段：

```text
enable        = 0
vision_mode   = DISABLED
mission_phase = TASK1_LAND_H(0x13)
target_type   = NONE
```

完成阶段：

```text
enable        = 0
vision_mode   = DISABLED
mission_phase = TASK1_COMPLETE(0x14)
target_type   = NONE
```

**摄像头动作与回传**

不参与 H 点降落，持续回传在线状态。进入下一局的新 session 或收到 RESET 后，清空本局投放锁存。

任务一从启动到完成必须控制在 90 秒内。

---

## 4. 舵机触发真值表

| 当前情况 | 是否驱动舵机 | 摄像头行为 |
|---|---:|---|
| `phase=0x10`，已对准 | 否 | 只报 `DROP_ALIGNED(0x04)` |
| 启动后第一次看到的阶段就是 `0x11`，未经过同 session 的 `0x10` | 否 | 报 `VISION_DROP_FAILED(0x09)`，防止误投 |
| `phase=0x11`，但目标无效或未对准 | 否 | 继续跟踪和报告，不触发舵机 |
| `phase=0x11`，已 armed、已对准、尚未执行 | 是，仅一次 | 锁存 started，报 `0x07` |
| 舵机正在动作，又收到重复 `0x11` | 否 | 继续报 `0x07` |
| 舵机动作已完成，又收到重复 `0x11` | 否 | 继续报 `0x08` |
| 舵机动作失败/超时，又收到重复 `0x11` | 否 | 继续报 `0x09` |
| 新 session 或 RESET | 否 | 清除上一局锁存，等待重新经过 `0x10` |

---

## 5. 摄像头内部锁存变量

建议最少保留以下变量：

| 变量 | 置位时机 | 清除时机 | 用途 |
|---|---|---|---|
| `drop_armed` | 同 session 合法收到 `phase=0x10` | 新 session 或 RESET | 证明本局经过对准流程 |
| `drop_started` | 第一次合法执行舵机前 | 新 session 或 RESET | 防止重复驱动 |
| `drop_completed` | 舵机序列正常结束 | 新 session 或 RESET | 锁存完成结果 |
| `drop_failed` | 舵机错误、非法直入或超时 | 新 session 或 RESET | 锁存失败结果 |

这些变量是摄像头本地变量，不占用额外线缆字段。结果通过 `vision_state` 上报。

---

## 6. 数据新鲜度和状态规则

- 摄像头建议以 20 Hz 回传 `VISION_DATA`；可在 20～30 Hz 内稳定运行。
- `IMAGE_FRESH=1` 表示本帧使用的是新鲜图像处理结果，不代表一定识别到目标。
- `TARGET_VALID=1` 表示当前帧确实识别到指定 `target_type`。
- `XY_VALID=1` 才允许树莓派使用 `relative_x/y_mm`。
- 当 `XY_VALID=0` 时，`relative_x/y_mm` 必须填 0，树莓派不得沿用旧值。
- `mission_phase_echo` 必须回显摄像头当前正在执行的最近一条有效命令阶段，便于发现串口延迟或阶段错配。
- `target_type` 必须与命令一致；目标类型不一致的数据不能用于控制。
- 视觉数据过期、阶段回显不一致或 session 不一致时，树莓派不得继续使用该偏移。
- 本协议不要求摄像头发送速度、z、高度、帧处理耗时或“START 后秒数”。START 后秒数属于小车/地面站信息，不属于摄像头控制必需字段。

---

## 7. 树莓派 ROS 2 对应关系和当前待补代码

### 7.1 已有主题

摄像头桥当前应继续发布：

```text
/vision/relative_error
/vision/target_valid
/vision/drop_aligned
/vision/land_aligned
/vision/link_ok
/vision/state
/vision/target_type
```

树莓派向摄像头命令来源：

```text
/competition/phase
/competition/active_task_mode
```

### 7.2 需要补齐的直接投放结果映射

摄像头桥需要新增或补齐：

```text
vision_state=0x07 → /payload/drop_in_progress=true
vision_state=0x08 → /payload/drop_completed=true
vision_state=0x09 → /payload/drop_failed=true
```

### 7.3 当前任务状态机需要补齐的条件

当前任务管理器已经订阅 `/payload/drop_completed`，但还需要确认或补齐：

1. `TASK1_ALIGN_DROP → TASK1_DROP` 的判断必须显式包含新鲜的 `/vision/drop_aligned=true`；
2. 应订阅并记录 `/payload/drop_failed`，不能让失败状态被误认为仍在等待；
3. 进入 `TASK1_RETURN_H` 后应保持摄像头禁用；
4. 新 session 时必须清除树莓派侧上一局的完成锁存。

因此本文是目标接口说明；在上述映射补齐并测试前，不能仅凭文档认为摄像头直接投放链已经在现有树莓派代码中完成。

---

## 8. 整个数据包对应列表

### 8.1 通用外部帧格式

适用于小车、树莓派、摄像头和地面站之间的 v1.7 外部协议：

```text
DA 7E | version | message_type | sequence | payload_length |
session_id_hi session_id_lo | payload | checksum8 | 0D 0A
```

| 字节位置 | 字段 | 长度 | 说明 |
|---:|---|---:|---|
| 0～1 | 帧头 | 2 | 固定 `DA 7E` |
| 2 | version | 1 | 固定 `0x01` |
| 3 | message_type | 1 | 消息类型 |
| 4 | sequence | 1 | 每个发送方独立递增，`255→0` |
| 5 | payload_length | 1 | 负载字节数 |
| 6～7 | session_id | 2 | 大端；KEY1任务一固定为1 |
| 8～ | payload | N | 各消息负载 |
| 8+N | checksum8 | 1 | 从 version 到 payload 最后一个字节求和取低8位 |
| 9+N～10+N | 帧尾 | 2 | 固定 `0D 0A` |

总帧长为 `11 + payload_length`。

### 8.2 全部消息类型

| 消息 | message_type | payload长度 | 总帧长 | 主要方向和作用 |
|---|---:|---:|---:|---|
| `CAR_STATUS` | `0x10` | 17 | 28 | 小车→树莓派、地面站：任务、点位、分段距离和协调标志 |
| `MISSION_SYNC` | `0x11` | 12 | 23 | 树莓派→小车：统一任务阶段和无人机结果 |
| `VISION_COMMAND` | `0x20` | 8 | 19 | 树莓派→摄像头：启用、模式、阶段和目标 |
| `VISION_DATA` | `0x21` | 12 | 23 | 摄像头→树莓派：状态、平面偏移和投放结果 |
| `UAV_STATUS` | `0x30` | 18 | 29 | 树莓派→地面站：无人机地图坐标和状态 |
| `GCS_STATUS` | `0x31` | 4 | 15 | 地面站→树莓派、小车（可选）：地面站在线/显示状态 |

树莓派—飞控使用独立的 V2 30 字节协议，不套用本节 `DA 7E` 帧。

### 8.3 `VISION_COMMAND(0x20)` 完整负载

| payload偏移 | 字段 | 类型 | 任务一含义 |
|---:|---|---|---|
| 0 | `enable` | `uint8` | 0禁用；1启用视觉处理 |
| 1 | `vision_mode` | `uint8` | 见 8.4 |
| 2 | `task_mode` | `uint8` | 任务一固定 `0x01` |
| 3 | `mission_phase` | `uint8` | 见 8.6 |
| 4 | `target_type` | `uint8` | 见 8.5 |
| 5 | `command_flags` | `uint8` | 位0=`RESET_TRACKER`，仅新局/明确复位脉冲使用 |
| 6～7 | `requested_rate_hz` | `uint16 BE` | 建议 `20`，即 `00 14` |

### 8.4 `vision_mode` 对应表

| 值 | 名称 | 摄像头行为 |
|---:|---|---|
| `0x00` | `DISABLED` | 不识别、不输出有效偏移、不驱动舵机 |
| `0x01` | `SEARCH` | 搜索小车平台 |
| `0x02` | `FOLLOW` | 持续跟踪平台并输出相对x/y |
| `0x03` | `DROP_ALIGN` | 识别投放中心并判断投放对准 |
| `0x04` | `LAND_ALIGN` | 任务二降落对准；任务一不使用 |

### 8.5 `target_type` 对应表

| 值 | 名称 | 明确定义 |
|---:|---|---|
| `0x00` | `NONE` | 当前不要求识别目标 |
| `0x01` | `CAR_PLATFORM` | 小车整体或平台跟踪目标，用于搜索和伴飞 |
| `0x02` | `LANDING_RING` | 任务二降落标志，任务一不使用 |
| `0x03` | `DROP_CENTER` | 小车平台黑色同心圆和十字线的中心 |

### 8.6 任务一使用的 `mission_phase`

| 值 | 名称 | 摄像头工作 |
|---:|---|---|
| `0x00` | `READY` | 禁用，等待 |
| `0x01` | `COUNTDOWN` | 禁用，新局首次RESET |
| `0x02` | `MISSION_START` | 禁用 |
| `0x03` | `TAKEOFF_HOVER` | 禁用 |
| `0x04` | `RIGHT_SHIFT` | 禁用 |
| `0x05` | `FORWARD_CHASE` | SEARCH小车平台 |
| `0x06` | `SEARCH_CAR` | 保留；当前任务一实现不单独进入 |
| `0x07` | `FOLLOW_CAR` | FOLLOW小车平台 |
| `0x10` | `TASK1_ALIGN_DROP` | 对准投放中心，绝不驱动舵机 |
| `0x11` | `TASK1_DROP` | 合法条件满足时直接驱动舵机一次 |
| `0x12` | `TASK1_RETURN_H` | 禁用 |
| `0x13` | `TASK1_LAND_H` | 禁用 |
| `0x14` | `TASK1_COMPLETE` | 禁用，等待复位/下一局 |

### 8.7 各阶段 `VISION_COMMAND` 快速表

| 全局阶段 | enable | vision_mode | target_type | 摄像头主要任务 |
|---|---:|---:|---:|---|
| READY～RIGHT_SHIFT `0x00～0x04` | 0 | 0 | 0 | 仅保持通信 |
| FORWARD_CHASE `0x05` | 1 | 1 | 1 | 搜索小车 |
| FOLLOW_CAR `0x07` | 1 | 2 | 1 | 伴飞相对定位 |
| TASK1_ALIGN_DROP `0x10` | 1 | 3 | 3 | 对准投放中心，不投放 |
| TASK1_DROP `0x11` | 1 | 3 | 3 | 只执行一次舵机动作 |
| RETURN/LAND/COMPLETE `0x12～0x14` | 0 | 0 | 0 | 停止视觉控制 |

所有行的 `task_mode=0x01`、`requested_rate_hz=20`；`mission_phase` 填表中对应值。

### 8.8 `VISION_DATA(0x21)` 完整负载

| payload偏移 | 字段 | 类型 | 说明 |
|---:|---|---|---|
| 0 | `mission_phase_echo` | `uint8` | 最近执行的有效摄像头命令阶段 |
| 1 | `vision_state` | `uint8` | 见 8.9 |
| 2 | `target_type` | `uint8` | 当前实际处理的目标类型 |
| 3 | `vision_flags` | `uint8` | 见 8.10 |
| 4～7 | `relative_x_mm` | `int32 BE` | BODY x，前为正；无效时为0 |
| 8～11 | `relative_y_mm` | `int32 BE` | BODY y，左为正；无效时为0 |

摄像头没有 z 轴字段，也不发送速度。

### 8.9 `vision_state` 完整对应表

| 值 | 名称 | 含义 |
|---:|---|---|
| `0x00` | `DISABLED` | 视觉未启用 |
| `0x01` | `READY` | 摄像头已准备好 |
| `0x02` | `SEARCHING` | 正在搜索指定目标 |
| `0x03` | `TRACKING` | 正在跟踪并可输出有效偏移 |
| `0x04` | `DROP_ALIGNED` | 已满足投放中心对准条件，但尚未投放 |
| `0x05` | `LAND_ALIGNED` | 任务二降落对准；任务一不使用 |
| `0x06` | `TARGET_LOST` | 目标丢失，旧x/y不可继续使用 |
| `0x07` | `VISION_DROP_EXECUTING` | 摄像头正在执行舵机投放序列 |
| `0x08` | `VISION_DROP_COMPLETED` | 舵机投放序列已经正常结束并锁存 |
| `0x09` | `VISION_DROP_FAILED` | 投放执行失败、超时或非法直入投放阶段 |
| `0xF0` | `VISION_ERROR` | 摄像头内部视觉错误 |

`0x07～0x09` 已正式并入统一设备协议v1.7，帧长度不变。

### 8.10 `vision_flags` 位定义

| 位 | 掩码 | 名称 | 含义 |
|---:|---:|---|---|
| 0 | `0x01` | `TARGET_VALID` | 当前识别到指定目标 |
| 1 | `0x02` | `XY_VALID` | 本帧x/y可用于控制 |
| 2 | `0x04` | `DROP_ALIGNED` | 满足投放对准条件 |
| 3 | `0x08` | `LAND_ALIGNED` | 满足任务二降落对准条件 |
| 4 | `0x10` | `IMAGE_FRESH` | 本帧图像处理结果新鲜 |
| 5 | `0x20` | `VISION_ERROR` | 摄像头内部视觉错误 |
| 6～7 | — | 保留 | 必须为0；投放执行结果放在 `vision_state`，不占新标志位 |

常用组合：

| 场景 | flags |
|---|---:|
| 禁用 | `0x00` |
| 图像新鲜但未找到目标 | `0x10` |
| 目标、x/y和图像均有效 | `0x13` |
| 已对准投放中心 | `0x17` |
| 图像新鲜且发生视觉错误 | `0x30` |

### 8.11 与任务一投放有关的小车标志

小车 `CAR_STATUS` 中必须能让树莓派判断：

| 信息 | 所在字段 | 用途 |
|---|---|---|
| `task_mode=TASK1` | `CAR_STATUS` payload偏移0 | 确认本局执行第一题 |
| `session_id=1` | 通用帧头偏移6～7 | KEY1固定会话，防止使用上一局数据 |
| 最近经过点=C | `CAR_STATUS` payload偏移3的`last_checkpoint=3` | 表示已经进入C点后的投放区段且尚未到D |
| `C_DECEL_COMPLETE=1` | `CAR_STATUS` payload偏移5的`car_flags` bit6 | 表示小车已减速到约定速度，可以准备投放 |

“经过 C”与“C 点后减速完成”不是同一个条件；必须以后者作为进入投放对准的最终小车锚点。投放执行中，树莓派在 `MISSION_SYNC.coordination_flags` 置 bit2 `DROP_IN_PROGRESS`；收到摄像头完成状态后置 bit3 `DROP_COMPLETED` 和 bit9 `CAR_ACCELERATE_ALLOWED`，通知小车恢复加速。

---

## 9. 固定字节示例

### 9.1 摄像头报告“正在投放”

条件：`session=1`、`sequence=0`、`phase=0x11`、`state=0x07`、`target=0x03`、`flags=0x17`、x/y均为0。

```text
DA 7E 01 21 00 0C 00 01 11 07 03 17
00 00 00 00 00 00 00 00 61 0D 0A
```

校验和：`0x61`。

### 9.2 摄像头报告“投放完成”

条件：`session=1`、`sequence=1`、`phase=0x11`、`state=0x08`、`target=0x03`、`flags=0x10`、x/y均为0。

```text
DA 7E 01 21 01 0C 00 01 11 08 03 10
00 00 00 00 00 00 00 00 5C 0D 0A
```

校验和：`0x5C`。

---

## 10. 最小联调验收顺序

所有测试先拆桨或确保舵机机构不会造成危险。

1. 上电：确认摄像头收到 `READY`，回 `DISABLED`。
2. KEY1：确认 session 为1，摄像头只在新局第一次 RESET。
3. 模拟阶段 `0x05`：确认摄像头搜索平台并输出 `0x13 + x/y`。
4. 模拟阶段 `0x07`：移动图案，确认 x前正、y左正，目标丢失后 x/y立即清零。
5. 模拟 C 点但不置 `C_DECEL_COMPLETE`：树莓派必须保持伴飞。
6. 置 `C_DECEL_COMPLETE`：树莓派进入 `0x10`，摄像头对准但舵机绝不能动。
7. 未经过 `0x10` 直接发 `0x11`：舵机不得动，应报告 `0x09`。
8. 合法经过 `0x10`、完成对准，再发 `0x11`：舵机只动作一次，先报 `0x07`，后报 `0x08`。
9. 持续重发 `0x11`：舵机不得第二次动作。
10. 树莓派收到 `0x08`：确认 `/payload/drop_completed=true`，随后进入 `0x12` 返回 H。
11. 新 session 或 RESET：确认四个投放锁存全部清除，可以开始下一局。

---

## 11. 实施结论

这套流程中，投放权限关系固定为：

```text
小车C点后减速完成
→ 树莓派允许进入投放对准
→ 摄像头报告已对准
→ 树莓派发送TASK1_DROP正式授权
→ 摄像头只驱动舵机一次
→ 摄像头报告舵机序列完成
→ 树莓派控制返航H点
```

这样既保留树莓派对全局状态机的统一控制，也允许摄像头直接控制舵机，减少投放执行链路延迟，同时避免摄像头一看到目标就误投或因重复命令重复投放。
