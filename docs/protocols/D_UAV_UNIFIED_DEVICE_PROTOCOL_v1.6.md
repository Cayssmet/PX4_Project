# D题陆空协同系统统一设备通信协议 v1.6

日期：2026-07-29  
适用设备：小车、树莓派、摄像头、地面站  
设计目标：状态统一、字段固定、容易抓串口调试；只保留必要的校验和超时

## 0. 阅读说明与系统背景

本文档是四端共同实现依据，不要求读者了解树莓派现有ROS 2工程。小车端、摄像头端和地面站端AI应直接按本文档的字节偏移、单位、状态编号和清零规则实现，不能自行改变字段顺序或数值含义。

### 0.1 各端重点阅读位置

- 小车端：第2～8节、第11～16节；实现CAR_STATUS发送和MISSION_SYNC接收。
- 摄像头端：第2～7节、第9节、第12～16节；实现VISION_COMMAND接收和VISION_DATA发送。
- 地面站端：第2～7节、第8节、第10～16节；接收CAR_STATUS和UAV_STATUS并图形化显示。
- 树莓派端：全文；负责四端状态汇总、全局任务推进、协议与ROS 2之间转换。

### 0.2 赛题任务概述

系统由循线小车、无人机、树莓派、视觉摄像头和无线地面站组成。小车从A点出发，沿A→B→C→D→A顺时针行驶一圈，全程不能停车。无人机从H点起飞。

- task1：无人机起飞到1.5 m并稳定悬停总计3秒，在B点前追上小车并形成伴飞；在D点前完成物资投放；随后返回H点降落。
- task2：无人机起飞后直接追车，不执行task1的额外3秒悬停；在D点前降落到移动小车，停留5秒后从车顶复飞，再返回H点降落。
- C点是本系统选定的陆空联动锚点。小车到C后减速，达到设定动作速度才允许树莓派开始投放或车顶降落流程。

### 0.3 设备连接关系

```text
                         ┌──────────────┐
                         │   地面站 GCS  │
                         └──────▲───▲───┘
                                │   │
                     UAV_STATUS │   │ CAR_STATUS
                                │   │
┌──────────┐  CAR_STATUS   ┌──────┴───────┐  VISION_COMMAND  ┌──────────┐
│  小车 CAR │──────────────►│ 树莓派 RPI   │────────────────►│ 摄像头   │
│          │◄──────────────│ 全局状态主机  │◄────────────────│ CAMERA   │
└──────────┘ MISSION_SYNC  └──────┬───────┘   VISION_DATA    └──────────┘
                                  │
                                  │ 独立的飞控V1/V2协议
                                  ▼
                             ┌─────────┐
                             │  飞控 FC │
                             └─────────┘
```

CAR_STATUS发给树莓派和地面站时字节格式完全相同。摄像头不直接与小车或地面站通信。树莓派—飞控和N10雷达使用独立协议，不能把这些字节流混到本协议串口中。

### 0.4 四端职责边界

| 设备 | 负责决定 | 只负责上报/执行 | 不允许决定 |
|---|---|---|---|
| 小车 | KEY1/KEY2、3秒倒计时、A/B/C/D识别、车速变化 | 循线、分段距离、动作速度就绪 | 无人机是否投放、是否降落成功 |
| 树莓派 | START后的全局mission_phase、投放/降落/复飞流程 | 融合小车、视觉、定位和飞控状态 | 绕过飞控安全保护 |
| 摄像头 | 目标是否被识别、相对位移、视觉对准标志 | 按树莓派命令搜索/跟踪 | 自行触发投放、下降或任务完成 |
| 地面站 | 图形和文字显示、日志记录 | 回传可选在线心跳 | 正式比赛中发送飞行控制命令 |

### 0.5 三类状态不要混淆

一帧中可能同时出现以下状态，它们不是重复字段：

- `mission_phase`：整个系统当前进行到哪一步，由所有设备使用同一编号。
- `car_state/uav_state/vision_state`：某个设备此刻具体在做什么。
- `flags`：某个条件当前是否成立，或某个检查点在本轮是否已经经过。

例如全局处于TAKEOFF_HOVER时，无人机局部状态可以是UAV_HOVER，小车局部状态仍是CAR_RUN_AB；小车不会因为全局写着HOVER而停止。

### 0.6 简化原则

本协议有帧头、固定长度、简单checksum8和少量超时，但不做逐帧ACK、丢包重发、加密、严格sequence拒绝或复杂action_id。关键状态以10～20 Hz周期发送。这样既能发现明显坏帧，也便于串口助手观察和调试。

## 1. 链路与职责

| 链路 | 方向 | 主要数据 |
|---|---|---|
| 小车 ↔ 树莓派 | 双向 | 小车 START/位置/状态；树莓派全局任务状态 |
| 小车 → 地面站 | 单向为主 | 小车状态、赛道分段、相对 A/B/C/D 的里程 |
| 树莓派 ↔ 摄像头 | 双向串口 | 视觉工作模式；目标相对位移和视觉状态 |
| 树莓派 → 地面站 | 单向为主 | 无人机地图位置和全系统状态 |
| 地面站 → 树莓派/小车 | 可选 | 仅地面站在线状态，不参与正式比赛控制 |

树莓派—飞控 V1/V2、N10 雷达继续使用各自独立协议，不使用本协议。

若通信硬件工作在透明串口模式，四端默认统一使用115200、8N1、无流控。某条无线链路若受模块限制使用其他波特率，只要求链路两端一致，本文的帧内容和长度不变。

## 2. 统一状态原则

每台设备同时保存两个状态：

1. `mission_phase`：全系统统一，同一个数值在所有设备中含义相同；
2. 局部状态：`car_state`、`uav_state`、`vision_state`，表示本设备正在做什么。

树莓派是 START 之后 `mission_phase` 的唯一控制者。小车只负责产生按键启动、3 秒倒计时和 START；START 后由树莓派把权威任务状态发回小车并发送给地面站、摄像头。

小车处于无人机悬停阶段时不会停车，而是继续执行自己的 `car_state`。全局状态表示当前任务重点，局部状态表示各设备的并行动作。

## 3. 启动时序

```text
小车板载 KEY1 或 KEY2 按下
    ↓ KEY1 固定选择 session_id=1/task1；KEY2 固定选择 session_id=2/task2
    ↓ 立即发送 COUNTDOWN_3S，倒计时约 3000 ms
    ↓ 小车保持静止，无人机保持待机，地面站显示倒计时
倒计时到 0
    ↓ 小车电机启动并持续发送 START_LATCHED=1
    ↓ 树莓派收到后立即执行起飞
    ↓ 地面站从 MISSION_START 开始正式计时
```

通信不得等待 3 秒后才建立。倒计时期间就开始发送状态，方便确认所有设备在线。小车开始运动和树莓派收到 START 之间会有一个无线帧延迟，允许存在几毫秒至几十毫秒误差。

按键和任务固定映射：

| 小车按键 | session_id | task_mode | 任务 |
|---|---:|---:|---|
| KEY1 | 1 | 1 | task1 抛投 |
| KEY2 | 2 | 2 | task2 动态起降 |

session_id 现在表示按键/任务类型，不表示运行次数，也不再每次加 1。树莓派只在 READY 接受按键启动；同一次任务活动期间重复收到相同 START 不会重复起飞。任务结束并重新回到 READY 后，允许再次使用同一个按键和 session。

## 4. 通用帧格式

所有外部设备使用同一个帧头和帧尾，通过 `message_type` 区分消息。

```text
DA 7E | version | message_type | sequence | payload_length
session_id(2字节) | payload(N字节) | checksum8 | 0D 0A
```

一帧总长度为：

```text
11 + payload_length
```

| 偏移 | 长度 | 字段 | 说明 |
|---:|---:|---|---|
| 0 | 1 | header_1 | 固定 `0xDA` |
| 1 | 1 | header_2 | 固定 `0x7E` |
| 2 | 1 | version | 当前固定 `0x01` |
| 3 | 1 | message_type | 见第 7 节 |
| 4 | 1 | sequence | 每发一帧加 1，255 后回到 0 |
| 5 | 1 | payload_length | 必须等于消息定义长度 |
| 6 | 2 | session_id | 大端 `uint16`；KEY1固定为1，KEY2固定为2，0表示尚未选择 |
| 8 | N | payload | 消息数据 |
| 8+N | 1 | checksum8 | 简单累加校验 |
| 9+N | 1 | tail_1 | 固定 `0x0D` |
| 10+N | 1 | tail_2 | 固定 `0x0A` |

文档名称中的v1.6是说明书修订号，线缆内的version字节仍固定为 `0x01`。所有四端当前都发送和接受 `0x01`；只有未来出现不兼容的帧布局时才升级线缆version。

### 4.1 checksum8

从 `version` 到 payload 最后一个字节逐字节相加，只保留低 8 位：

```c
checksum = 0;
for (i = 2; i < 8 + payload_length; ++i) {
    checksum = (checksum + frame[i]) & 0xFF;
}
```

本协议不使用 CRC16、ACK、自动重发、加密或复杂的旧帧拒绝。校验错误或长度错误时丢弃当前帧，继续搜索下一组 `DA 7E`。

### 4.2 字节序和单位

- 多字节整数全部使用大端；
- 位置、距离：mm；
- 速度：mm/s；
- 角度：mrad；
- 时间：ms；
- 不在线缆中发送 float/double；
- `uint32 0xFFFFFFFF` 表示对应距离尚不可用；
- 有符号数必须结合相应 VALID 标志使用。

### 4.3 sequence 的简单用法

sequence 只用于地面站和日志观察是否丢帧。接收端不因为 sequence 重复、跳变或设备重启而锁死通信；校验和正确、session 合法且消息没有超时即可使用。

## 5. 全局任务状态 `mission_phase`

| 值 | 名称 | 全局含义 |
|---:|---|---|
| `0x00` | READY | 小车在 A、无人机在 H，等待按键 |
| `0x01` | COUNTDOWN_3S | 按键已按，约 3 秒倒计时 |
| `0x02` | MISSION_START | 小车开始行驶、无人机开始起飞 |
| `0x03` | TAKEOFF_HOVER | 无人机起飞/悬停，小车继续行驶 |
| `0x04` | RIGHT_SHIFT | 无人机右移 0.75 m，小车继续行驶 |
| `0x05` | FORWARD_CHASE | 无人机向前追车 |
| `0x06` | SEARCH_CAR | 摄像头搜索小车/平台 |
| `0x07` | FOLLOW_CAR | 已形成伴飞 |
| `0x10` | TASK1_ALIGN_DROP | task1：C-D 直道投放对准 |
| `0x11` | TASK1_DROP | task1：执行投放 |
| `0x12` | TASK1_RETURN_H | task1：返回 H |
| `0x13` | TASK1_LAND_H | task1：H 点降落 |
| `0x14` | TASK1_COMPLETE | task1 完成 |
| `0x20` | TASK2_ALIGN_LAND | task2：对准车顶 |
| `0x21` | TASK2_DESCEND | task2：视觉下降 |
| `0x22` | TASK2_ON_CAR | task2：已降落在小车平台 |
| `0x23` | TASK2_HOLD_5S | task2：车顶停留 5 秒 |
| `0x24` | TASK2_RETAKEOFF | task2：从小车再次起飞 |
| `0x25` | TASK2_RETURN_H | task2：返回 H |
| `0x26` | TASK2_LAND_H | task2：H 点降落 |
| `0x27` | TASK2_COMPLETE | task2 完成 |
| `0xF0` | SAFE_HOLD | 暂停推进，保持安全状态 |
| `0xF1` | ERROR | 任务错误 |
| `0xFF` | ABORTED | 任务终止，不自动重启 |

task1 在 `TAKEOFF_HOVER` 内完成赛题要求的总计 3 秒稳定悬停。task2 到高后不等待这 3 秒，直接进入 RIGHT_SHIFT。

## 6. 设备局部状态

### 6.1 `car_state`

| 值 | 名称 |
|---:|---|
| `0x00` | CAR_READY_A |
| `0x01` | CAR_COUNTDOWN |
| `0x02` | CAR_RUN_AB |
| `0x03` | CAR_RUN_BC |
| `0x04` | CAR_RUN_CD |
| `0x05` | CAR_RUN_DA |
| `0x06` | CAR_COMPLETE_A |
| `0xF0` | CAR_ERROR |

`last_checkpoint`：0 UNKNOWN、1 A、2 B、3 C、4 D。当前赛道段直接由 `car_state` 的 CAR_RUN_AB/BC/CD/DA 表示，不再单独发送segment_id。

### 6.2 `uav_state`

`uav_state` 表示无人机本体此刻正在执行的动作。它由树莓派根据全局任务阶段、PID和飞控反馈生成，再发给小车和地面站。它不是全局 `mission_phase`：同一全局阶段中，小车可能继续循线，而无人机处于悬停或视觉下降。

| 值 | 名称 | 解释 |
|---:|---|---|
| `0x00` | UAV_STANDBY | 无人机位于H点待机，尚未起飞 |
| `0x01` | UAV_TAKEOFF | 已收到启动要求，正在升至巡航高度 |
| `0x02` | UAV_HOVER | 已到达目标高度并保持定高 |
| `0x03` | UAV_MAP_NAVIGATION | 使用雷达地图坐标执行右移、追车或返航 |
| `0x04` | UAV_VISUAL_FOLLOW | 使用摄像头相对x/y位移跟随小车 |
| `0x05` | UAV_VISUAL_DESCENT | 使用视觉水平对准并执行车顶下降 |
| `0x06` | UAV_LANDED_ON_CAR | 已确认落在小车平台并保持稳定 |
| `0x07` | UAV_RETAKEOFF | 正在从移动小车平台再次起飞 |
| `0x08` | UAV_RETURNING_H | 正在返回H点 |
| `0x09` | UAV_LANDED_H | 已完成H点最终降落 |
| `0xF0` | UAV_FAILSAFE | 飞控或关键输入触发保护，不能按正常任务推进 |
| `0xF1` | UAV_ERROR | 无人机任务出现无法继续的错误 |

### 6.3 `vision_state`

`vision_state` 表示摄像头算法此刻处于什么工作状态，由摄像头产生并发给树莓派。该状态只供树莓派判断视觉是否可用于控制，不转发给地面站；只有树莓派有权根据视觉结果推进全局任务；摄像头不能自行宣布投放或降落完成。

| 值 | 名称 | 解释 |
|---:|---|---|
| `0x00` | VISION_DISABLED | 树莓派未要求视觉工作，摄像头不输出控制位移 |
| `0x01` | VISION_READY | 算法已启动并可接收模式命令，但尚未搜索目标 |
| `0x02` | VISION_SEARCHING | 正在寻找小车、投放中心或降落圆环，尚无稳定目标 |
| `0x03` | VISION_TRACKING | 已稳定识别目标并持续输出相对位移 |
| `0x04` | VISION_DROP_ALIGNED | 摄像头判断目标进入投放对准范围；是否投放仍由树莓派决定 |
| `0x05` | VISION_LAND_ALIGNED | 摄像头判断目标进入降落对准范围；是否下降仍由树莓派决定 |
| `0x06` | VISION_TARGET_LOST | 原目标丢失，相对位移不得继续用于控制 |
| `0xF0` | VISION_ERROR | 摄像头、图像或算法发生错误 |

`vision_mode`：0 DISABLED、1 SEARCH、2 FOLLOW、3 DROP_ALIGN、4 LAND_ALIGN。

## 7. 消息清单和总长度

| message_type | 消息 | 方向 | payload | 一帧总长 | 帧开头和结尾 |
|---:|---|---|---:|---:|---|
| `0x10` | CAR_STATUS | 小车→树莓派、小车→地面站 | 17 | 28 | `DA 7E 01 10 ... 0D 0A` |
| `0x11` | MISSION_SYNC | 树莓派→小车 | 12 | 23 | `DA 7E 01 11 ... 0D 0A` |
| `0x20` | VISION_COMMAND | 树莓派→摄像头 | 8 | 19 | `DA 7E 01 20 ... 0D 0A` |
| `0x21` | VISION_DATA | 摄像头→树莓派 | 12 | 23 | `DA 7E 01 21 ... 0D 0A` |
| `0x30` | UAV_STATUS | 树莓派→地面站 | 18 | 29 | `DA 7E 01 30 ... 0D 0A` |
| `0x31` | GCS_STATUS | 地面站→树莓派/小车，可选 | 4 | 15 | `DA 7E 01 31 ... 0D 0A` |

## 8. 小车 ↔ 树莓派、地面站

### 8.1 CAR_STATUS，message_type=`0x10`

payload固定17字节，小车以20 Hz发给树莓派，以10 Hz发给地面站。两端使用完全相同的数据格式。

| payload偏移 | 长度 | 字段 | 类型/说明 |
|---:|---:|---|---|
| 0 | 1 | task_mode | 0未选择，1 task1，2 task2 |
| 1 | 1 | mission_phase | 小车保存的统一状态 |
| 2 | 1 | car_state | 小车局部状态，同时表明当前AB/BC/CD/DA赛道段 |
| 3 | 1 | last_checkpoint | 最近经过点：0未知，1 A，2 B，3 C，4 D |
| 4 | 1 | car_error_code | 0正常；其他值见错误码表 |
| 5 | 2 | car_flags | `uint16`，见下表 |
| 7 | 2 | countdown_remaining_ms | 0～3000；其他阶段为0 |
| 9 | 2 | speed_mm_s | 小车当前速度 |
| 11 | 2 | action_target_speed_mm_s | C点后投放/降落使用的设定速度 |
| 13 | 4 | distance_since_checkpoint_mm | 从last_checkpoint重新累计的距离 |

`car_flags`：

| bit | 名称 | 说明 |
|---:|---|---|
| 0 | START_LATCHED | 3秒倒计时结束后持续为1 |
| 1 | MOVING | 小车正在行驶 |
| 2 | LINE_VALID | 小车本地循线输入有效；只报有效性，不传循线误差 |
| 3 | DISTANCE_VALID | 分段里程有效 |
| 4 | RPI_SYNC_OK | 500 ms内收到树莓派状态 |
| 5 | DECELERATING | C点后正在减速 |
| 6 | C_DECEL_COMPLETE | 经过C点后，已经减速到投放/降落设定速度；这是“C点减速完成”标志 |
| 7 | HOLD_ACTION_SPEED | 正在保持投放/降落速度 |
| 8 | ACCELERATING | 动作结束后正在加速 |
| 9 | CRUISE_SPEED_REACHED | 已恢复正常巡航速度 |
| 10 | LAP_COMPLETE | 已完成一圈并重新到达A |
| 11 | RESERVED | 必须0 |
| 12 | CAR_ERROR | 小车本地错误 |
| 13 | CHECKPOINT_ESTIMATED | 最近检查点由标定里程估算而非直接识别 |
| 14～15 | RESERVED | 必须0 |

`RPI_SYNC_OK`由小车本地计时：每收到一帧合法MISSION_SYNC就置1，连续500 ms没有收到则清零。当前协议不在线上传输等待年龄。

到达点信息不再使用单独的checkpoint_flags，而由持续发送的 `last_checkpoint` 表达：

- 起点准备时：last_checkpoint=A、car_state=CAR_READY_A、LAP_COMPLETE=0；
- 经过B后：last_checkpoint=B、car_state=CAR_RUN_BC，分段距离从0重新累计；
- 经过C后：last_checkpoint=C、car_state=CAR_RUN_CD，分段距离清零并开始减速；
- 经过D后：last_checkpoint=D、car_state=CAR_RUN_DA，分段距离清零；
- 完成一圈回A：last_checkpoint=A、car_state=CAR_COMPLETE_A、LAP_COMPLETE=1。

CAR_STATUS以10～20 Hz持续发送，所以接收端即使漏掉某一帧，后续帧仍会保持最新last_checkpoint。固定路线顺序使接收端可由car_state和last_checkpoint推知已经经过的点，不需要额外保存A/B/C/D四个锁存位。每次更新last_checkpoint时，distance_since_checkpoint_mm必须清零，旧赛道段误差不带入新赛道段。

C点联动规则：

1. last_checkpoint更新为C，小车置DECELERATING，并开始降低车速；
2. 当前速度进入本地设定速度容差后，清除DECELERATING，置C_DECEL_COMPLETE和HOLD_ACTION_SPEED；
3. 树莓派只有同时确认last_checkpoint=C和C_DECEL_COMPLETE，才进入投放或降落准备；其中前者表示“已经过C且尚未到D”，后者表示“C点后的减速已经完成”；
4. last_checkpoint更新为D后，树莓派不再允许开始新的投放或降落动作；若动作尚未完成，由树莓派状态机决定失败或退出。

`C_DECEL_COMPLETE`不是单帧脉冲：小车减速完成后应持续保持为1，直到小车开始重新加速或新一轮任务复位时清零。它只说明“小车在C点后已经完成减速”，不能脱离`last_checkpoint=C`单独作为开始动作的依据。

减速、保持低速和加速标志会发送给地面站用于日志，但地面站正式界面不显示这些内部速度阶段。

### 8.2 MISSION_SYNC，message_type=`0x11`

payload 固定12字节，树莓派以10 Hz发给小车。START后该消息中的 `mission_phase` 和动作标志是全系统权威结果。

| payload偏移 | 长度 | 字段 | 类型/说明 |
|---:|---:|---|---|
| 0 | 1 | task_mode | 当前锁定任务 |
| 1 | 1 | mission_phase | 树莓派权威统一状态 |
| 2 | 1 | uav_state | 无人机局部状态 |
| 3 | 1 | sync_flags | bit0有效，bit1任务活动，bit2完成，bit3错误 |
| 4 | 2 | countdown_remaining_ms | 倒计时回显；START后为0 |
| 6 | 2 | coordination_flags | `uint16`，陆空联动动作标志 |
| 8 | 2 | action_elapsed_ms | 当前投放/降落/复飞动作已用时间 |
| 10 | 1 | error_code | 0正常 |
| 11 | 1 | reserved | 必须0 |

`coordination_flags`：

| bit | 名称 | 说明 |
|---:|---|---|
| 0 | C_ANCHOR_ACCEPTED | 树莓派已接受C点联动锚点 |
| 1 | ACTION_PREPARING | 正在视觉对准或等待动作条件 |
| 2 | DROP_IN_PROGRESS | task1正在投放 |
| 3 | DROP_COMPLETED | task1投放机构确认完成 |
| 4 | LAND_IN_PROGRESS | task2正在车顶下降 |
| 5 | LANDED_ON_CAR | task2已确认落在小车上 |
| 6 | HOLD_5S_COMPLETE | task2车顶停留5秒完成 |
| 7 | RETAKEOFF_IN_PROGRESS | task2正在从车顶复飞 |
| 8 | RETAKEOFF_COMPLETED | task2已恢复巡航高度 |
| 9 | CAR_ACCELERATE_ALLOWED | 允许小车开始恢复巡航速度 |
| 10 | FINAL_LAND_IN_PROGRESS | 无人机正在H点最终降落 |
| 11 | TASK_COMPLETE | 当前任务完成 |
| 12 | ACTION_FAILED | 投放、降落或复飞失败 |
| 13～15 | RESERVED | 必须0 |

小车联动规则：

- task1：C_DECEL_COMPLETE后保持设定速度；收到DROP_COMPLETED和CAR_ACCELERATE_ALLOWED后，清除HOLD_ACTION_SPEED并置ACCELERATING；达到巡航速度后置CRUISE_SPEED_REACHED。
- task2：C_DECEL_COMPLETE后保持设定速度；LANDED_ON_CAR后速度不变；HOLD_5S_COMPLETE和RETAKEOFF_IN_PROGRESS期间仍保持；只有收到RETAKEOFF_COMPLETED和CAR_ACCELERATE_ALLOWED后才加速。
- ACTION_FAILED时小车不自行猜测后续动作，继续按自身循线策略运行并保留相关标志供日志分析。

小车超过500 ms未收到MISSION_SYNC时，只清除RPI_SYNC_OK，不停车、不自行改变投放/降落状态。

## 9. 树莓派 ↔ 摄像头

推荐独立串口：115200、8N1、无流控。最终设备名由接线后确定。

### 9.1 VISION_COMMAND，message_type=`0x20`

payload 固定 8 字节，树莓派以 10 Hz 发送。

| payload偏移 | 长度 | 字段 | 类型/说明 |
|---:|---:|---|---|
| 0 | 1 | enable | 0 关闭，1 开启 |
| 1 | 1 | vision_mode | SEARCH/FOLLOW/DROP_ALIGN/LAND_ALIGN |
| 2 | 1 | task_mode | 当前任务 |
| 3 | 1 | mission_phase | 统一状态 |
| 4 | 1 | target_type | 0无，1小车平台，2降落圆环，3投放中心 |
| 5 | 1 | command_flags | bit0 RESET_TRACKER |
| 6 | 2 | requested_rate_hz | 建议 20 |

重复收到相同命令不重启算法。只有 vision_mode 改变、任务从READY进入COUNTDOWN，或RESET_TRACKER=1时重置跟踪器。由于KEY1和KEY2会重复使用固定session，新一轮相同任务必须由树莓派主动置RESET_TRACKER。

### 9.2 VISION_DATA，message_type=`0x21`

payload固定12字节，摄像头以20～30 Hz发送。摄像头只发送平面相对位移，不发送高度、三维z或相对yaw。

| payload偏移 | 长度 | 字段 | 类型/单位 |
|---:|---:|---|---|
| 0 | 1 | mission_phase_echo | 最近收到的统一状态 |
| 1 | 1 | vision_state | 搜索、跟踪、对准、丢失或错误状态 |
| 2 | 1 | target_type | 当前识别目标 |
| 3 | 1 | vision_flags | 见下表 |
| 4 | 4 | relative_x_mm | `int32`，平面内目标在机体前方为正 |
| 8 | 4 | relative_y_mm | `int32`，平面内目标在机体左方为正 |

`vision_flags`：

| bit | 名称 | 说明 |
|---:|---|---|
| 0 | TARGET_VALID | 当前检测到可信目标 |
| 1 | XY_VALID | relative_x/y可用于平面控制 |
| 2 | DROP_ALIGNED | 满足摄像头侧投放对准范围 |
| 3 | LAND_ALIGNED | 满足摄像头侧降落对准范围 |
| 4 | IMAGE_FRESH | 图像和算法输出仍在刷新 |
| 5 | VISION_ERROR | 摄像头本地错误 |
| 6～7 | RESERVED | 必须0 |

摄像头必须把图像坐标转换为无人机BODY平面坐标后再发送：x向前、y向左。这里的x/y表示目标中心相对无人机还需要向哪个平面方向移动多少毫米。树莓派只用这两个量生成水平控制，yaw速度保持0；无人机高度完全来自飞控，不从摄像头协议获取。

没有可靠目标时，摄像头清除TARGET_VALID和XY_VALID，并把relative_x/y置0。树莓派超过200 ms没有收到合法VISION_DATA，或IMAGE_FRESH为0时，立即停止使用上一帧位移；不需要向摄像头发送ACK。

`vision_state`保留在摄像头→树莓派帧中，用于区分算法正在搜索、已经跟踪、目标丢失或视觉错误。它不再放入UAV_STATUS，也不发送给地面站。

## 10. 树莓派 ↔ 地面站

### 10.1 UAV_STATUS，message_type=`0x30`

payload固定18字节，树莓派以10 Hz发给地面站。该帧只发送无人机位置、无人机/任务状态和START后秒数，不发送速度、yaw、摄像头内部状态或各链路年龄。

| payload偏移 | 长度 | 字段 | 类型/单位 |
|---:|---:|---|---|
| 0 | 1 | task_mode | 当前任务 |
| 1 | 1 | mission_phase | 全系统权威统一状态 |
| 2 | 1 | uav_state | 无人机局部状态 |
| 3 | 1 | error_code | 树莓派汇总错误；0正常 |
| 4 | 2 | uav_flags | `uint16`，见下表 |
| 6 | 4 | map_x_mm | `int32`，无人机相对H点的平面x坐标 |
| 10 | 4 | map_y_mm | `int32`，无人机相对H点的平面y坐标 |
| 14 | 2 | current_height_mm | `uint16`，飞控提供的当前高度 |
| 16 | 2 | mission_elapsed_s | `uint16`，从MISSION_START起经过的整秒数 |

`uav_flags`：

| bit | 名称 |
|---:|---|
| 0 | POSE_VALID |
| 1 | FLIGHT_LINK_OK |
| 2 | CAMERA_LINK_OK |
| 3 | CAR_LINK_OK |
| 4 | ARMED |
| 5 | AIRBORNE |
| 6 | HEIGHT_VALID |
| 7 | VISION_CONTROL_ACTIVE |
| 8 | PAYLOAD_RELEASED |
| 9 | LANDED_ON_CAR |
| 10 | LANDED_AT_H |
| 11 | FAILSAFE |
| 12 | TASK_COMPLETE |
| 13～15 | 保留，必须0 |

无人机地图坐标是二维平面坐标：H点起飞位置为 `(0,0)`，x为初始机头方向，y向左。高度单独使用 `current_height_mm`，来源是飞控而不是摄像头。READY和COUNTDOWN阶段 `mission_elapsed_s=0`；进入MISSION_START时从0开始计时，超过65535秒后保持65535。

地面站通过独立CAR_STATUS获得小车局部状态，不需要树莓派重复转发car_state；摄像头vision_state属于树莓派内部控制输入，不发给地面站。

### 10.2 GCS_STATUS，message_type=`0x31`

payload 固定 4 字节，属于可选心跳，不参与飞行决策。

| payload偏移 | 长度 | 字段 | 类型/说明 |
|---:|---:|---|---|
| 0 | 1 | gcs_state | 0启动，1就绪，2正在显示，0xF0错误 |
| 1 | 1 | mission_phase_echo | 地面站最近显示的统一状态 |
| 2 | 1 | gcs_flags | bit0显示就绪，bit1记录中 |
| 3 | 1 | reserved | 必须 0 |

地面站离线不影响小车行驶和无人机控制。

## 11. 地面站小车位置显示

地面站保存 A/B/C/D 赛道几何和每段标定长度，通过以下数据绘制小车位置：

```text
car_state
last_checkpoint
distance_since_checkpoint_mm
```

地面站先使用 `last_checkpoint` 把小车位置校准到最近的A/B/C/D，再根据 `car_state` 判断当前AB/BC/CD/DA赛道段，并用 `distance_since_checkpoint_mm` 沿该段插值。到达下一检查点时重新校准并从零开始，旧赛道段的编码器累计误差不带入新赛道段。小车不能直接识别B/C/D时，可以按本段标定里程推算，同时置CHECKPOINT_ESTIMATED。

## 12. 最低限度的失效处理

只保留以下简单处理：

- checksum 或长度错误：丢弃该帧；
- 摄像头超过 200 ms：视觉无效，不使用旧位移；
- 小车状态超过 500 ms：CAR_LINK_OK=0，不根据旧的last_checkpoint或C_DECEL_COMPLETE开始动作；
- 树莓派状态超过 500 ms：小车清除 RPI_SYNC_OK，但继续循线；
- 地面站超过 1 s 没收到数据：界面显示链路离线；
- 当前任务活动期间，重复START不重复起飞；
- KEY1必须同时为session=1、task_mode=1，KEY2必须同时为session=2、task_mode=2；
- 同一个固定session只有在系统重新回到READY后才能再次启动。

明确不做：逐帧 ACK、丢包重发、加密、自动版本切换、严格 sequence 拒绝、复杂 action_id。所有关键状态通过 10～20 Hz 周期发送解决偶发丢帧。

## 13. 实现顺序

1. 四端复制相同的状态枚举、消息类型和 checksum8；
2. 先用串口助手逐条验证 6 种固定长度帧；
3. 小车实现 COUNTDOWN_3S、CAR_STATUS 和 MISSION_SYNC 接收；
4. 摄像头实现 VISION_COMMAND/VISION_DATA；
5. 树莓派编写统一通信桥并映射到 ROS 2；
6. 地面站实现 UAV_STATUS、CAR_STATUS 显示；
7. 验证A/B/C/D标志、C点减速、速度就绪、投放后加速、落车后保持和复飞后加速；
8. 最后做按键倒计时、同步启动和两项完整状态机测试。


## 14. 编码、发送与接收步骤

### 14.1 发送端组帧

每个发送端都按同一顺序组帧：

1. 写入帧头 `DA 7E`；
2. 写入version、message_type、sequence和固定payload_length；
3. 写入大端session_id；
4. 按本文档偏移依次写payload，不允许直接发送编译器未打包的C结构体；
5. 对偏移2到payload最后一字节计算checksum8；
6. 写入checksum和帧尾 `0D 0A`；
7. 一次发送完整帧。

推荐使用明确的写入函数：

```c
write_u16_be(buffer, offset, value);
write_s16_be(buffer, offset, value);
write_u32_be(buffer, offset, value);
write_s32_be(buffer, offset, value);
```

所有有符号字段使用二进制补码。接收端先按大端无符号数拼接，再转换为对应的 `int16_t/int32_t`。不要依赖不同单片机的结构体对齐、位域顺序或本机字节序。

### 14.2 接收端从连续字节流解析

建议使用以下简单状态机：

1. 逐字节寻找连续的 `0xDA 0x7E`；
2. 收满偏移2～7，读取message_type和payload_length；
3. 检查version必须为1，message_type必须是本链路允许的类型，长度必须等于第7节固定值；
4. 按 `11 + payload_length` 等待整帧；
5. 检查最后两字节必须为 `0D 0A`；
6. 重新计算checksum8并比较；
7. 检查session与task_mode固定映射；
8. 全部正确才更新本地数据和接收时间。

失败时丢弃当前候选帧，并从候选帧第二个字节之后重新寻找 `DA 7E`。不要只靠 `0D 0A` 判断结束，因为payload中允许自然出现相同字节。

### 14.3 物理链路如何区分设备

帧中没有source_id和destination_id，这是为了缩短帧。设备身份由物理链路和message_type共同确定：

- 小车链路只接受CAR_STATUS和MISSION_SYNC；
- 摄像头链路只接受VISION_COMMAND和VISION_DATA；
- 地面站链路接收CAR_STATUS和UAV_STATUS，可选发送GCS_STATUS。

如果以后多个设备共用同一总线，必须先升级协议增加设备地址，不能直接沿用当前点对点格式。

## 15. 标志位保持、清零和错误码

### 15.1 实时位和锁存位

实时位只描述当前这一刻，条件消失就清零：

- MOVING、LINE_VALID、DISTANCE_VALID、RPI_SYNC_OK；
- DECELERATING、C_DECEL_COMPLETE、HOLD_ACTION_SPEED；
- ACCELERATING、CRUISE_SPEED_REACHED；
- ACTION_PREPARING、DROP_IN_PROGRESS、LAND_IN_PROGRESS；
- RETAKEOFF_IN_PROGRESS、FINAL_LAND_IN_PROGRESS；
- TARGET_VALID、XY_VALID、IMAGE_FRESH。

锁存位表示本轮任务已经发生，置1后保持到系统重新进入READY：

- START_LATCHED；
- LAP_COMPLETE；
- C_ANCHOR_ACCEPTED、DROP_COMPLETED、LANDED_ON_CAR；
- HOLD_5S_COMPLETE、RETAKEOFF_COMPLETED、TASK_COMPLETE、ACTION_FAILED。

CAR_ACCELERATE_ALLOWED应保持到小车回报CRUISE_SPEED_REACHED，随后树莓派可以清零。DROP_ALIGNED和LAND_ALIGNED由摄像头按当前画面实时判断，不锁存。

### 15.2 新一轮任务复位

系统重新进入READY时：

- mission_phase回到READY，session_id回到0；
- 清除START、LAP_COMPLETE和全部动作完成标志；
- 小车确认位于A后设置last_checkpoint=A、car_state=CAR_READY_A，并将distance_since_checkpoint_mm清零；
- 树莓派向摄像头发送RESET_TRACKER；
- 小车和树莓派各自清零任务计时，不要求sequence回到0。

### 15.3 小车错误码 `car_error_code`

| 值 | 名称 | 说明 |
|---:|---|---|
| `0x00` | CAR_OK | 正常 |
| `0x01` | LINE_LOST | 循线输入丢失 |
| `0x02` | ENCODER_INVALID | 编码器或分段距离不可用 |
| `0x03` | MOTOR_ERROR | 电机或驱动异常 |
| `0x04` | CHECKPOINT_ERROR | 检查点顺序或识别异常 |
| `0x05` | KEY_TASK_MISMATCH | 按键、session与task_mode不匹配 |
| `0xFF` | CAR_INTERNAL_ERROR | 未分类错误 |

### 15.4 树莓派汇总错误码 `error_code/system_error_code`

| 值 | 名称 | 说明 |
|---:|---|---|
| `0x00` | SYSTEM_OK | 正常 |
| `0x01` | SESSION_TASK_MISMATCH | KEY、session和task_mode不一致 |
| `0x02` | CAR_TIMEOUT | 小车状态超过500 ms |
| `0x03` | VISION_TIMEOUT | 视觉数据超过200 ms |
| `0x04` | FLIGHT_LINK_TIMEOUT | 飞控状态过期 |
| `0x05` | POSE_INVALID | 无人机地图位置无效 |
| `0x06` | ACTION_WINDOW_MISSED | 到D仍未完成要求动作 |
| `0x07` | DROP_FAILED | 投放失败 |
| `0x08` | LAND_FAILED | 车顶降落失败 |
| `0x09` | RETAKEOFF_FAILED | 车顶复飞失败 |
| `0xFF` | SYSTEM_INTERNAL_ERROR | 未分类错误 |

错误码用于显示和日志。是否进入SAFE_HOLD、ERROR或ABORTED仍由树莓派任务状态机决定。

## 16. 各设备端实现清单

### 16.1 小车端必须实现

1. READY时以session=0周期发送CAR_STATUS；
2. KEY1固定选择session=1/task1，KEY2固定选择session=2/task2；
3. 按键后立即进入COUNTDOWN_3S并发送剩余时间，小车暂不运动；
4. 倒计时结束时启动电机、置START_LATCHED并进入CAR_RUN_AB；
5. 识别或估算A/B/C/D，更新last_checkpoint和car_state，并将分段距离清零；
6. last_checkpoint更新为C后减速，到设定速度后置C_DECEL_COMPLETE和HOLD_ACTION_SPEED；
7. 解析树莓派MISSION_SYNC并保存权威mission_phase；
8. task1只在DROP_COMPLETED和CAR_ACCELERATE_ALLOWED后加速；
9. task2落车后保持速度，只在RETAKEOFF_COMPLETED和CAR_ACCELERATE_ALLOWED后加速；
10. 同一份CAR_STATUS分别发给树莓派和地面站，地面站链路可以降低到10 Hz。

### 16.2 摄像头端必须实现

1. 解析VISION_COMMAND并核对固定19字节整帧；
2. 根据vision_mode选择搜索、跟随、投放对准或降落对准算法；
3. 把图像坐标转换为无人机BODY平面坐标：x前、y左；
4. 在VISION_DATA中回显最近收到的mission_phase和相同session；
5. 没有可靠目标时清除TARGET_VALID和XY_VALID，把相对位移字段置0；
6. 新一轮相同session仍可能重复使用，收到RESET_TRACKER必须清除旧目标和旧对准状态；
7. VISION_DROP_ALIGNED/VISION_LAND_ALIGNED只表示视觉条件满足，不直接控制投放或下降；
8. 以20～30 Hz发送固定23字节VISION_DATA。

### 16.3 地面站端必须实现

1. 同时解析CAR_STATUS和UAV_STATUS；
2. 使用mission_phase表显示统一中文状态；
3. 使用UAV_STATUS中的uav_state显示无人机状态，并使用独立CAR_STATUS中的car_state显示小车状态；
4. 使用H点为原点的map_x/y绘制无人机，并单独显示current_height_mm；
5. 使用car_state、last_checkpoint和distance_since_checkpoint_mm绘制小车；
6. 每次last_checkpoint改变时，把显示位置校准到对应A/B/C/D；
7. 保存减速、低速和加速标志到日志，但正式界面不显示这些内部速度阶段；
8. 超时只显示离线，不向小车或无人机发送控制命令。

### 16.4 树莓派端必须实现

1. 接收CAR_STATUS并检查KEY/session/task固定映射；
2. START后成为mission_phase唯一主机，并以10 Hz发送MISSION_SYNC；
3. 根据任务阶段发送VISION_COMMAND；
4. 将VISION_DATA转换为ROS 2视觉相对位移，过期后立即撤销；
5. 只有last_checkpoint=C和C_DECEL_COMPLETE同时成立才开始动作准备；
6. 根据真实投放机构、视觉、PID和飞控反馈设置coordination_flags；
7. 向地面站发送只包含无人机二维坐标、高度、状态和START后秒数的UAV_STATUS；
8. 不把地面站断联当成飞行故障。

## 17. 固定测试帧

下面是一条完整MISSION_SYNC测试帧，所有实现应得到完全相同的23字节。

条件：

```text
message_type = 0x11 MISSION_SYNC
sequence = 0
payload_length = 12
session_id = 1
task_mode = 1
mission_phase = 0x10 TASK1_ALIGN_DROP
uav_state = 0x04 UAV_VISUAL_FOLLOW
sync_flags = 0x03 状态有效+任务活动
countdown_remaining_ms = 0
coordination_flags = 0x0003 C锚点已接受+动作准备中
action_elapsed_ms = 0
error_code = 0
reserved = 0
checksum8 = 0x3A
```

完整字节：

```text
DA 7E 01 11 00 0C 00 01 01 10 04 03 00 00 00 03 00 00 00 00 3A 0D 0A
```

校验覆盖范围从第3个字节 `01` 到payload最后一个 `00`，不包括帧头、checksum和帧尾。收到该帧后，小车应保存task1、TASK1_ALIGN_DROP、UAV_VISUAL_FOLLOW，并识别C_ANCHOR_ACCEPTED与ACTION_PREPARING。
