# 飞控通信部分分析报告（面向 PX4 1.17.0 + uXRCE-DDS Offboard 迁移）

分析对象：
- `D:\09_PX4\pi_ros2_ws\src\flight_controller_bridge\`
- `D:\09_PX4\pi_ros2_ws\src\uav_interfaces\`
- `D:\09_PX4\pi_ros2_ws\docs\protocols\D_UAV_UNIFIED_DEVICE_PROTOCOL_v1.7.md`

> **⚠ 更正说明（追加，未改动下文原文）**
> 本文 §6.1 与 §7.2 写的 `/fmu/in/vehicle_odometry` **有误**。PX4 的 `dds_topics.yaml` 里 `vehicle_odometry` 只出现在 **publications**（即 `/fmu/out/vehicle_odometry`）；派 → 飞控方向能收 `VehicleOdometry` 的话题只有 **`/fmu/in/vehicle_visual_odometry`** 和 `/fmu/in/vehicle_mocap_odometry`，官方原文 "EKF2 only subscribes to `vehicle_visual_odometry` topics"。发到 `/fmu/in/vehicle_odometry` **不报错但完全无效**（不在白名单的话题在 DDS 里根本不存在）。
> 另两处口径：§7.2 的 `OffboardControlMode` 50 Hz 建议按官方示例改为 **10 Hz**（硬门槛 ≥2 Hz）；§6.3 / §7.5 的"串级 PID 冲突"应读作**一个必须拍死的架构选择**（offboard 用速度模式则 PX4 位置环不参与，树莓派 PID 是正常分工）。
> 详见 `PX4伴飞工程计划书.md` §8.5 与 §9.4。

---

## 0. 最重要的一条结论（先看这个）

**`D_UAV_UNIFIED_DEVICE_PROTOCOL_v1.7.md` 不是飞控协议。**

它是「小车 / 树莓派 / 摄像头 / 地面站」四端之间的**任务协同协议**。飞控在这份文档里被明确排除：

- 文档第 0.3 节连接图把飞控单列：`树莓派 RPI ◄─独立飞控V2协议─► 飞控 FC`；
- 文档第 1 节末尾原话：**"树莓派—飞控 V1/V2、N10 雷达继续使用各自独立协议，不使用本协议"**。

真正承载"控制量"的协议是另外两份（同目录下）：

| 文件 | 线缆 version | 帧长 | 用途 |
|---|---|---|---|
| `D:\09_PX4\pi_ros2_ws\src\flight_controller_bridge\docs\serial_protocol.md` | 无 version 字段（`AA 55`） | 命令 16B / 状态 8B | V1 旧基线 |
| `D:\09_PX4\pi_ros2_ws\src\flight_controller_bridge\docs\serial_protocol_v2.md` | `0x02`（`A5 5A`） | 命令 30B / 状态 30B | **V2，当前生产配置** |

当前生产参数已显式切到 V2（`config\flight_controller_bridge_params.yaml:6` → `protocol_version: 2`）。
**迁移到 PX4 时，被推翻的是 V1/V2 这两份线缆协议；四端协同协议 v1.7 要完整保留。**

---

## 1. 问题 1：`D_UAV_UNIFIED_DEVICE_PROTOCOL_v1.7.md` 完整协议规格

文件：`D:\09_PX4\pi_ros2_ws\docs\protocols\D_UAV_UNIFIED_DEVICE_PROTOCOL_v1.7.md`（57606 B，1191 行，日期 2026-07-30）

### 1.1 串口参数

| 项 | 值 | 出处 |
|---|---|---|
| 波特率 | 115200（透明串口模式下四端统一默认） | 行79 |
| 数据位/校验/停止位 | 8N1 | 行79 |
| 流控 | 无 | 行79 |
| 摄像头链路 | 同样推荐 115200 / 8N1 | 行815 |
| 多字节整数字节序 | **全部大端（Big-Endian）** | 行595-602 |
| 单位 | 位置 mm；速度 mm/s；角度 mrad；时间 ms | 行595-602 |
| 线缆是否传 float | **否**，只用整数定点 | 行595-602 |
| 0xFFFFFFFF | 表示"距离不可用" | 行595-602 |

### 1.2 帧格式（原文逐字段）

原文（行553-556）：

```
DA 7E | version | message_type | sequence | payload_length
session_id(2字节) | payload(N字节) | checksum8 | 0D 0A
```

帧总长 = `11 + payload_length`（行561）。

| 偏移 | 长度 | 字段名 | 原文说明 |
|---|---|---|---|
| 0 | 1 | `header_1` | 固定 `0xDA` |
| 1 | 1 | `header_2` | 固定 `0x7E` |
| 2 | 1 | `version` | 固定 `0x01`（**文档 v1.7 是说明书修订号，线缆 version 永远是 0x01**，行578） |
| 3 | 1 | `message_type` | 见 §1.3 命令表 |
| 4 | 1 | `sequence` | 每帧 +1，255 回 0 |
| 5 | 1 | `payload_length` | payload 字节数 N |
| 6 | 2 | `session_id` | 大端 uint16；KEY1=1，KEY2=2，0=未选择 |
| 8 | N | `payload` | 见 §1.3 |
| 8+N | 1 | `checksum8` | 见下方算法 |
| 9+N | 1 | `tail_1` | 固定 `0x0D` |
| 10+N | 1 | `tail_2` | 固定 `0x0A` |

**checksum8**（行584-589，原文 C 代码）：

```c
checksum = 0;
for (i = 2; i < 8 + payload_length; ++i) {
    checksum = (checksum + frame[i]) & 0xFF;
}
```

即：**从 `version` 累加到 payload 最后一个字节**（不含 `DA 7E`，不含 checksum 自身和帧尾）。

**可靠性设计（原文明确的"不做"清单，行591/606/945-958）**：
- 不做逐帧 ACK、不做丢包重发、不做加密、不做自动版本切换、不做严格 sequence 拒绝、不引入复杂 action_id；
- checksum 错误 → 直接丢帧，继续搜索 `DA 7E`；
- sequence 只用于地面站/日志观察丢帧；接收端不因重复、跳变、重启而锁死。

### 1.3 消息类型完整列表（编号 + 载荷字段）

出处：行699-706（清单）、行714-929（逐字段）。

#### 0x10 `CAR_STATUS`（小车 → 树莓派 / 地面站，payload 17 B，帧总长 28 B，20 Hz 发树莓派 / 10 Hz 发地面站）

| 偏移 | 长度 | 字段 | 说明 |
|---|---|---|---|
| 0 | 1 | `task_mode` | 0 未选择 / 1 task1 / 2 task2 |
| 1 | 1 | `mission_phase` | 见 §1.4 |
| 2 | 1 | `car_state` | 见 §1.4 |
| 3 | 1 | `last_checkpoint` | 0 UNKNOWN / 1 A / 2 B / 3 C / 4 D |
| 4 | 1 | `car_error_code` | 见 §1.6 |
| 5-6 | 2 | `car_flags` | uint16，位定义见下 |
| 7-8 | 2 | `countdown_remaining_ms` | 0~3000，其他阶段为 0 |
| 9-10 | 2 | `speed_mm_s` | 当前速度 |
| 11-12 | 2 | `action_target_speed_mm_s` | 动作目标速度 |
| 13-16 | 4 | `distance_since_checkpoint_mm` | uint32，自上个检查点距离 |

`car_flags` 位（行729-745）：
```
bit0  START_LATCHED        bit1  MOVING
bit2  LINE_VALID           bit3  DISTANCE_VALID
bit4  RPI_SYNC_OK          bit5  DECELERATING
bit6  C_DECEL_COMPLETE     bit7  HOLD_ACTION_SPEED
bit8  ACCELERATING         bit9  CRUISE_SPEED_REACHED
bit10 LAP_COMPLETE         bit11 RESERVED(必须0)
bit12 CAR_ERROR            bit13 CHECKPOINT_ESTIMATED
bit14~15 RESERVED(必须0)
```
`bit4 RPI_SYNC_OK` 的判定 = 500 ms 内收到过树莓派状态。

#### 0x11 `MISSION_SYNC`（树莓派 → 小车，payload 12 B，帧总长 23 B，**树莓派 10 Hz 发送**）

| 偏移 | 长度 | 字段 | 说明 |
|---|---|---|---|
| 0 | 1 | `task_mode` | 0/1/2 |
| 1 | 1 | `mission_phase` | **权威阶段**，树莓派是唯一主机 |
| 2 | 1 | `uav_state` | 见 §1.4 |
| 3 | 1 | `sync_flags` | bit0 有效 / bit1 任务活动 / bit2 完成 / bit3 错误 |
| 4-5 | 2 | `countdown_remaining_ms` | |
| 6-7 | 2 | `coordination_flags` | uint16，位定义见下 |
| 8-9 | 2 | `action_elapsed_ms` | |
| 10 | 1 | `error_code` | 见 §1.6 |
| 11 | 1 | `reserved` | **必须 0** |

`coordination_flags` 位（行788-803）：
```
bit0  C_ANCHOR_ACCEPTED      bit1  ACTION_PREPARING
bit2  DROP_IN_PROGRESS       bit3  DROP_COMPLETED
bit4  LAND_IN_PROGRESS       bit5  LANDED_ON_CAR
bit6  HOLD_5S_COMPLETE       bit7  RETAKEOFF_IN_PROGRESS
bit8  RETAKEOFF_COMPLETED    bit9  CAR_ACCELERATE_ALLOWED
bit10 FINAL_LAND_IN_PROGRESS bit11 TASK_COMPLETE
bit12 ACTION_FAILED          bit13~15 RESERVED(必须0)
```

#### 0x20 `VISION_COMMAND`（树莓派 → 摄像头，payload 8 B，帧总长 19 B，树莓派 10 Hz）

| 偏移 | 长度 | 字段 | 说明 |
|---|---|---|---|
| 0 | 1 | `enable` | 使能 |
| 1 | 1 | `vision_mode` | 0 DISABLED / 1 SEARCH / 2 FOLLOW / 3 DROP_ALIGN / 4 LAND_ALIGN |
| 2 | 1 | `task_mode` | |
| 3 | 1 | `mission_phase` | |
| 4 | 1 | `target_type` | 0 无 / 1 小车平台 / 2 降落圆环 / 3 投放中心 |
| 5 | 1 | `command_flags` | bit0 `RESET_TRACKER` |
| 6-7 | 2 | `requested_rate_hz` | 建议 20 |

#### 0x21 `VISION_DATA`（摄像头 → 树莓派，payload 12 B，帧总长 23 B，20~30 Hz）

| 偏移 | 长度 | 字段 | 说明 |
|---|---|---|---|
| 0 | 1 | `mission_phase_echo` | |
| 1 | 1 | `vision_state` | 见 §1.4 |
| 2 | 1 | `target_type` | |
| 3 | 1 | `vision_flags` | 位定义见下 |
| 4-7 | 4 | `relative_x_mm` | int32，**机体前方为正** |
| 8-11 | 4 | `relative_y_mm` | int32，**机体左方为正** |

`vision_flags`（行848-856）：`bit0 TARGET_VALID` / `bit1 XY_VALID` / `bit2 DROP_ALIGNED` / `bit3 LAND_ALIGNED` / `bit4 IMAGE_FRESH` / `bit5 VISION_ERROR` / `bit6~7 RESERVED(必须0)`

**注意：只发平面相对位移，不发高度 / z / yaw。**

#### 0x30 `UAV_STATUS`（树莓派 → 地面站，payload 18 B，帧总长 29 B，树莓派 10 Hz）

| 偏移 | 长度 | 字段 | 说明 |
|---|---|---|---|
| 0 | 1 | `task_mode` | |
| 1 | 1 | `mission_phase` | |
| 2 | 1 | `uav_state` | |
| 3 | 1 | `error_code` | |
| 4-5 | 2 | `uav_flags` | uint16，位定义见下 |
| 6-9 | 4 | `map_x_mm` | int32，相对 H 点平面 x |
| 10-13 | 4 | `map_y_mm` | int32，相对 H 点平面 y |
| 14-15 | 2 | `current_height_mm` | uint16，**由飞控提供** |
| 16-17 | 2 | `mission_elapsed_s` | uint16，MISSION_START 起整秒，超 65535 保持 65535 |

`uav_flags`（行899-914）：
```
bit0  POSE_VALID            bit1  FLIGHT_LINK_OK
bit2  CAMERA_LINK_OK        bit3  CAR_LINK_OK
bit4  ARMED                 bit5  AIRBORNE
bit6  HEIGHT_VALID          bit7  VISION_CONTROL_ACTIVE
bit8  PAYLOAD_RELEASED      bit9  LANDED_ON_CAR
bit10 LANDED_AT_H           bit11 FAILSAFE
bit12 TASK_COMPLETE         bit13~15 保留(必须0)
```
**不发速度 / yaw / 摄像头状态 / 链路年龄。**

#### 0x31 `GCS_STATUS`（地面站 → 树莓派 / 小车，可选，payload 4 B，帧总长 15 B）

| 偏移 | 长度 | 字段 | 说明 |
|---|---|---|---|
| 0 | 1 | `gcs_state` | 0 启动 / 1 就绪 / 2 正在显示 / 0xF0 错误 |
| 1 | 1 | `mission_phase_echo` | |
| 2 | 1 | `gcs_flags` | bit0 显示就绪 / bit1 记录中 |
| 3 | 1 | `reserved` | **必须 0** |

可选心跳，**不参与飞行决策**。

### 1.4 状态枚举（原文）

`mission_phase`（行610-635）：
```
0x00 READY              0x01 COUNTDOWN_3S      0x02 MISSION_START
0x03 TAKEOFF_HOVER      0x04 RIGHT_SHIFT       0x05 FORWARD_CHASE
0x06 SEARCH_CAR         0x07 FOLLOW_CAR
0x10 TASK1_ALIGN_DROP   0x11 TASK1_DROP        0x12 TASK1_RETURN_H
0x13 TASK1_LAND_H       0x14 TASK1_COMPLETE
0x20 TASK2_ALIGN_LAND   0x21 TASK2_DESCEND     0x22 TASK2_ON_CAR
0x23 TASK2_HOLD_5S      0x24 TASK2_RETAKEOFF   0x25 TASK2_RETURN_H
0x26 TASK2_LAND_H       0x27 TASK2_COMPLETE
0xF0 SAFE_HOLD          0xF1 ERROR             0xFF ABORTED
```

`car_state`（行645-652）：
```
0x00 CAR_READY_A  0x01 CAR_COUNTDOWN  0x02 CAR_RUN_AB  0x03 CAR_RUN_BC
0x04 CAR_RUN_CD   0x05 CAR_RUN_DA     0x06 CAR_COMPLETE_A  0xF0 CAR_ERROR
```

`uav_state`（行662-673）：**由树莓派根据阶段/PID/飞控反馈生成**
```
0x00 UAV_STANDBY  0x01 UAV_TAKEOFF   0x02 UAV_HOVER   0x03 UAV_MAP_NAVIGATION
0x04 UAV_VISUAL_FOLLOW  0x05 UAV_VISUAL_DESCENT  0x06 UAV_LANDED_ON_CAR
0x07 UAV_RETAKEOFF  0x08 UAV_RETURNING_H  0x09 UAV_LANDED_H
0xF0 UAV_FAILSAFE  0xF1 UAV_ERROR
```

`vision_state`（行681-691）：
```
0x00 VISION_DISABLED  0x01 READY  0x02 SEARCHING  0x03 TRACKING
0x04 VISION_DROP_ALIGNED  0x05 VISION_LAND_ALIGNED  0x06 VISION_TARGET_LOST
0x07 VISION_DROP_EXECUTING  0x08 VISION_DROP_COMPLETED  0x09 VISION_DROP_FAILED
0xF0 VISION_ERROR
```

### 1.5 上下行方向

| 方向 | 消息 |
|---|---|
| 树莓派下行 | `0x11 MISSION_SYNC`（→小车）、`0x20 VISION_COMMAND`（→摄像头）、`0x30 UAV_STATUS`（→地面站） |
| 树莓派上行接收 | `0x10 CAR_STATUS`（←小车）、`0x21 VISION_DATA`（←摄像头）、`0x31 GCS_STATUS`（←地面站，可选） |

### 1.6 错误码（原文）

`car_error_code`（行1058-1066）：
```
0x00 CAR_OK  0x01 LINE_LOST  0x02 ENCODER_INVALID  0x03 MOTOR_ERROR
0x04 CHECKPOINT_ERROR  0x05 KEY_TASK_MISMATCH  0xFF CAR_INTERNAL_ERROR
```

树莓派汇总 `error_code`（行1070-1082）：
```
0x00 SYSTEM_OK  0x01 SESSION_TASK_MISMATCH  0x02 CAR_TIMEOUT
0x03 VISION_TIMEOUT  0x04 FLIGHT_LINK_TIMEOUT  0x05 POSE_INVALID
0x06 ACTION_WINDOW_MISSED  0x07 DROP_FAILED  0x08 LAND_FAILED
0x09 RETAKEOFF_FAILED  0xFF SYSTEM_INTERNAL_ERROR
```
> **`0x04 FLIGHT_LINK_TIMEOUT` / `0x05 POSE_INVALID` 与飞控强相关，迁移后必须仍然能被触发。**

### 1.7 心跳 / 超时机制（第 12 节，行945-958）

| 链路 | 超时门限 | 超时后处理 |
|---|---|---|
| 摄像头 | 200 ms | 视觉判定无效，**不再使用旧位移** |
| 小车状态 | 500 ms | `CAR_LINK_OK=0`，不再使用旧 `last_checkpoint` / `C_DECEL_COMPLETE` |
| 树莓派状态 | 500 ms | 小车清 `RPI_SYNC_OK`，但**继续循线** |
| 地面站 | 1 s | 界面显示离线 |

其他行为：任务活动期间重复 START 不重复起飞；`KEY1` 必须 `session_id=1 且 task_mode=1`，`KEY2` 必须 `session_id=2 且 task_mode=2`。

### 1.8 关键任务时序（与飞控能力直接相关）

- task1 起飞后要求 `TAKEOFF_HOVER` 在 150±10 cm 稳定悬停**总计 3 s**；现场策略参数为"成功固定等 2.0 秒直接 RIGHT_SHIFT"，**不满足官方 3 秒**（行251/267）。
- `RIGHT_SHIFT` = 向右约 **0.375 m**，对应**机体系 vy < 0**（行277-279）。
- task1 从 `MISSION_START` 到 `TASK1_COMPLETE` 必须 **≤ 90 s**（行535）。
- 投放由摄像头直接驱动舵机（v1.7 变更），但 **`0x10` 只表示对准，只有 `0x11` 是授权**（第 9.3 节，行864-877）。
- 起飞/降落不是本协议命令，而是"树莓派按 mission_phase 状态机 + 独立飞控 V2 协议"发出（行219-220、行507）。

### 1.9 固定测试帧（第 17 节，行1148-1188）

```
MISSION_SYNC(task1, phase=0x10):
DA 7E 01 11 00 0C 00 01 01 10 04 03 00 00 00 03 00 00 00 00 3A 0D 0A   (checksum8=0x3A)

VISION_DATA(投放执行中):
DA 7E 01 21 00 0C 00 01 11 07 03 17 00 00 00 00 00 00 00 00 61 0D 0A   (checksum8=0x61)

VISION_DATA(投放完成):
DA 7E 01 21 01 0C 00 01 11 08 03 10 00 00 00 00 00 00 00 00 5C 0D 0A   (checksum8=0x5C)
```

---

## 1b. 附带：真正控制飞控的 V1 / V2 线缆协议（迁移时必须推翻的部分）

### V2（`docs\serial_protocol_v2.md`，version=0x02，30 字节冻结布局，当前生产）

物理层（行11）：115200 / 8N1 / 无流控；**命令 50 Hz**；**飞控命令看门狗 200 ms**；3.3V TTL；大端；
CRC16-CCITT-FALSE（poly=0x1021，init=0xFFFF，不反射，xorout=0，"123456789" 检查值 0x29B1）；
**CRC 覆盖偏移 2~25 共 24 字节**，CRC 在 26~27，帧尾 28~29；
命令帧头 `A5 5A`，状态帧头 `5A A5`，两种帧均固定 30 字节；
机体坐标系：x 前、y 左、z 上，yaw 俯视逆时针为正。

命令帧（行26-43）：

| 偏移 | 字段 | 类型 | 说明 |
|---|---|---|---|
| 0 | header | u8 | `0xA5` |
| 1 | header | u8 | `0x5A` |
| 2 | `protocol_version` | u8 | `0x02` |
| 3 | `message_type` | u8 | `0x01` |
| 4 | `transport_sequence` | u8 | 自然回绕 |
| 5 | `control_mode` | u8 | 见下表 |
| 6 | `command_flags` | u8 | bit0 = `control_valid`，其余必须 0 |
| 7 | `reserved` | u8 | **必须 0** |
| 8-11 | `session_id` | u32 | |
| 12-15 | `action_id` | u32 | |
| 16-17 | `velocity_x_mm_s` | i16 | BODY 前向 |
| 18-19 | `velocity_y_mm_s` | i16 | BODY 左向 |
| 20-21 | `velocity_z_mm_s` | i16 | **向上为正** |
| 22-23 | `yaw_rate_mrad_s` | i16 | 逆时针为正 |
| 24-25 | `target_height_mm` | u16 | |
| 26-27 | `CRC16` | u16 | |
| 28-29 | tail | | `0x0D 0x0A` |

`control_mode` 表（行47-54）：

| 值 | 模式 | 约束 |
|---|---|---|
| 0 | `STANDBY` | vx/vy/yaw=0，vz=0，h=0，action=0 |
| 1 | `TAKEOFF` | 水平必须 0，vz 必须 0，h=300~1700 mm，action>0 |
| 2 | `HEIGHT_HOLD` | vx/vy/yaw 有效，vz 必须 0，h=100~1700 mm，action=0 |
| 3 | `PRECISION_DESCENT` | 水平有效，vz 有符号，h=100~1700 mm（下降安全下限），action=0 |
| 4 | `FINAL_LAND` | 全 0，action>0 |
| 5 | `RESET_MISSION` | 全 0，action>0 |

状态帧（行60-80）：

| 偏移 | 字段 | 说明 |
|---|---|---|
| 0-3 | `5A A5 02 02` | 头 + version + message_type=0x02 |
| 4 | `acknowledged_sequence` | |
| 5 | `current_mode` | 0~5 |
| 6 | `status_flags` | 见下 |
| 7 | `error_code` | 见下 |
| 8-11 | `active_session_id` | |
| 12-15 | `active_action_id` | |
| 16-17 | `current_height_mm` | |
| 18-19 | `vertical_velocity_mm_s` | |
| 20-21 | `height_age_ms` | |
| 22-23 | `command_age_ms` | |
| 24 | `action_result` | |
| 25 | `reserved` | 必须 0 |
| 26-27 | `CRC16` | |
| 28-29 | tail | |

`status_flags`（行82）：`bit0 height_valid` / `bit1 height_reached`（**仅兼容/诊断，树莓派不再用它推进任务**）/ `bit2 armed` / `bit3 airborne` / `bit4 landed` / `bit5 action_accepted` / `bit6 action_completed` / `bit7 failsafe`

`action_result`（行90）：`0 NONE=(0,0)` / `1 PENDING=(1,0)` / `2 SUCCEEDED=(1,1)` / `3 REJECTED=(0,0)` / `4 FAILED=(1,0)`（括号 = accepted, completed）

高度无效时**必须**发 `current_height_mm=0`、`height_valid=0`、`height_age_ms=65535`（行92）。

`error_code` 冻结映射（行98-106）：
```
0x00 OK              0x01 RC_FAILSAFE        0x02 COMMAND_TIMEOUT
0x03 HEIGHT_TIMEOUT  0x04 HEIGHT_OUT_OF_RANGE 0x05 FRAME_ERROR(预留)
0x06 SEQUENCE_ERROR(预留)  0x07 NOT_AUTHORIZED  0x08 INVALID_MODE
0x09 INVALID_SESSION 0x0A DUPLICATE_ACTION  0x0B ACTION_NOT_ALLOWED
0x0C TAKEOFF_TIMEOUT(保留不主动产生)  0x0D LAND_TIMEOUT  0x0E RESET_NOT_ALLOWED
0x0F INTERNAL_ERROR(预留)  0x10 INVALID_FIELD  0x11 OLD_ACTION
```

安全上限（行112）：水平合速度 **1500 mm/s**、垂直 **|500| mm/s**、yaw **|2000| mrad/s**；生产高度上限 **1700 mm**。

sequence 语义（行119-125）：`delta = uint8(new - last)`；`0` 重复帧、`1..127` 合法新帧、`128..255` 旧帧；**只有合法新帧**可更新控制量/看门狗/ACK/离散动作；树莓派用 `lag = uint8(last_sent - acknowledged)` 判 ACK，默认仅接受 `lag <= 10`。

session/action 规则（行127-133）：新 session **必须先发一帧无物理动作的 STANDBY**（mode=0、action=0、valid=1、全 0），飞控回显 `active_session_id` 后树莓派才可发 TAKEOFF；飞行中出现不同 session 须返回 `INVALID_SESSION`；离散动作唯一键 = `session_id + action_id + control_mode`；同 session 新 action 必须严格递增；完全相同重发返回缓存结果；同 action_id 改参数返回 `DUPLICATE_ACTION`。

断联行为（行139-146）：
- `HEIGHT_HOLD` 断联 → 水平/yaw 归零，高度有效时保持最后安全目标；
- `PRECISION_DESCENT` 断联或 valid=0 → 水平/yaw/vz 归零，保持当前安全高度；
- `TAKEOFF` 只执行一次，到高后转定高；到高条件 = 目标 ±100 mm 连续 100 ms（5 个 20 ms 周期），**无固定 20 s 超时**；
- `FINAL_LAND` 一旦 accepted 就**脱离树莓派重发依赖**；
- `RESET_MISSION` 只复位任务状态，不解锁、不起飞；
- 遥控失联与主飞控保护**始终更高优先级**。

固定测试向量（行150-162）：
```
命令: A5 5A 02 01 FE 03 01 00 01 02 03 04 00 00 00 00 03 E8 FE 0C FF 88 00 32 01 2C D0 7B 0D 0A
状态: 5A A5 02 02 7B 02 6F 00 11 22 33 44 00 00 00 07 05 DC FF 85 00 14 00 28 02 00 08 A6 0D 0A
```

### V1（`docs\serial_protocol.md`，旧基线，仍可 `protocol_version: 1` 配置）

- 115200 / 8N1 / 无流控、命令 50 Hz、看门狗 200 ms；飞控物理接口 = 底板串口 2（`DrvUart2`），串口 4 专供匿名光流（行15）。
- 命令帧固定 **16 字节**，帧头 `AA 55`（行34-46）：`0 AA` / `1 55` / `2 sequence` / `3 flags` / `4-5 velocity_x(i16,mm/s,前方为正)` / `6-7 velocity_y(i16,mm/s,左侧为正)` / `8-9 yaw_rate(i16,mrad/s,逆时针为正)` / `10-11 target_height(u16,mm,离地)` / `12-13 CRC16(大端，覆盖偏移 2~11)` / `14 0x0D` / `15 0x0A`。
- 命令 flags（行50-54）：`bit0 control_valid`（0 时飞控必须输出安全零速度）/ `bit1 land_request` / `bit2~7 保留必须 0`。
  有效组合（行58-63）：`(0,0)` 不执行新运动目标，x/y/z/yaw 安全置零；`(1,0)` 执行 x/y/yaw 并用 `target_height` 做本地 z PID；`(1,1)` x/y/yaw 先置零、忽略 `target_height`、调用独立降落流程；`(0,1)` **非法，不得触发降落**。
- 状态帧固定 **8 字节**，帧头 `55 AA`（行69-77）：`0 55` / `1 AA` / `2 ack_sequence` / `3 flags` / `4-5 CRC16(覆盖 2~3)` / `6 0D` / `7 0A`。
- 状态 flags（行81-87）：`bit0 height_reached` / `bit1 land_accepted` / `bit2 land_completed` / `bit3 failsafe` / `bit4~7 保留`。
- 飞控端强制安全规则（行136-143）：帧头/长度/帧尾/CRC 全对才更新命令与看门狗；`control_valid=0` 立即清零；**连续 200 ms 无合法新命令**清零并退出树莓派控制；坏 CRC / flags / 重复或旧序号最迟下一 20 ms 周期清零并保持 failsafe；遥控授权开关关闭或遥控 failsafe 时树莓派命令无法取得控制权；相同目标高度重复到达不得每 20 ms 重置 z PID 积分滤波。
- 旧 `TMD_Set()` 单位换算（行147-156）：`vx_cm_s = round(vx_mm_s/10)`、`vy_cm_s = round(vy_mm_s/10)`、`yaw_deg_s = round(yaw_mrad_s*180/(1000*pi))`、`height_cm = round(target_height_mm/10)`。
- V1 测试向量（行120-131）：
```
命令: AA 55 01 01 03 E8 FE 0C 00 64 04 B0 90 E4 0D 0A
状态: 55 AA 01 01 3E 1F 0D 0A
```

---

## 2. 问题 2：`uav_interfaces` 定义了什么

路径：`D:\09_PX4\pi_ros2_ws\src\uav_interfaces\`
`CMakeLists.txt:16-29` — `rosidl_generate_interfaces` 注册 **10 个 msg + 1 个 srv**，`DEPENDENCIES builtin_interfaces std_msgs`，`VERSION 1.1.0`，C++17。
**没有任何 `.action` 文件。**

### msg 完整清单与字段原文

#### `msg\UavStatus.msg`（81 行）
常量：`TASK_NONE=0 / TASK1=1 / TASK2=2`；
`PHASE_READY=0` … `PHASE_TASK1_COMPLETE=20`、`PHASE_TASK2_*=32..39`、`PHASE_SAFE_HOLD=240 / ERROR=241 / ABORTED=255`；
`UAV_STANDBY=0` … `UAV_LANDED_H=9`、`FAILSAFE=240 / ERROR=241`；
`SYSTEM_OK=0` … `RETAKEOFF_FAILED=9 / INTERNAL_ERROR=255`；
`FLAG_POSE_VALID=1` … `FLAG_TASK_COMPLETE=4096`（对应协议 `uav_flags` bit0..12）。

字段：
```
std_msgs/Header header
uint8  sequence
uint16 session_id
uint8  task_mode
uint8  mission_phase
uint8  uav_state
uint8  error_code
uint16 flags
int32  map_x_mm
int32  map_y_mm
uint16 current_height_mm
uint16 mission_elapsed_s
```

#### `msg\CarStatus.msg`（44 行）
常量：`CAR_READY_A=0` … `CAR_COMPLETE_A=6`、`CAR_ERROR=240`；`CHECKPOINT_UNKNOWN/A/B/C/D=0..4`；`FLAG_START_LATCHED=1` … `FLAG_CHECKPOINT_ESTIMATED=8192`。
```
std_msgs/Header header
bool   communication_ok
uint8  sequence
uint16 session_id
uint8  task_mode
uint8  mission_phase
uint8  car_state
uint8  last_checkpoint
uint8  error_code
uint16 flags
uint16 countdown_remaining_ms
uint16 speed_mm_s
uint16 action_target_speed_mm_s
uint32 distance_since_checkpoint_mm
```

#### `msg\MissionSync.msg`（29 行）
常量：`SYNC_VALID=1 / TASK_ACTIVE=2 / TASK_COMPLETE=4 / ERROR=8`；`C_ANCHOR_ACCEPTED=1` … `ACTION_FAILED=4096`。
```
std_msgs/Header header
uint8  sequence
uint16 session_id
uint8  task_mode
uint8  mission_phase
uint8  uav_state
uint8  sync_flags
uint16 countdown_remaining_ms
uint16 coordination_flags
uint16 action_elapsed_ms
uint8  error_code
```

#### `msg\GcsStatus.msg`（15 行）
常量：`GCS_STARTING=0 / READY=1 / DISPLAYING=2 / ERROR=240`；`FLAG_DISPLAY_READY=1 / FLAG_RECORDING=2`。
```
std_msgs/Header header
bool   communication_ok
uint8  sequence
uint16 session_id
uint8  gcs_state
uint8  mission_phase_echo
uint8  flags
```

#### `msg\FlightCommand.msg`（43 行）
注释原文：**"这不是实际串口帧，帧头/帧尾/sequence/CRC16属于 flight_controller_bridge 内部线缆协议，不放在 ROS 2 消息中"**。
常量：`PROTOCOL_V1=1 / PROTOCOL_V2=2`；`STANDBY=0` … `RESET_MISSION=5`。
```
std_msgs/Header header        # frame_id 必须 base_link
uint8  protocol_version
uint8  control_mode
uint32 session_id
uint32 action_id
float32 velocity_x
float32 velocity_y
float32 yaw_rate
float32 velocity_z            # 仅V2精密下降，向上为正，m/s
float32 target_height         # 米，飞控本地激光做 z PID
bool   control_valid
bool   land_request
```

#### `msg\FlightStatus.msg`（59 行）
常量：`PROTOCOL_V1/V2`；`STANDBY..RESET_MISSION`；`ACTION_NONE=0 / PENDING=1 / SUCCEEDED=2 / REJECTED=3 / FAILED=4`。
```
std_msgs/Header header
uint8  protocol_version
bool   communication_ok
bool   height_reached         # 注释：兼容/诊断，树莓派任务状态机不得使用本字段推进任务
bool   land_accepted
bool   land_completed
bool   failsafe
uint8  current_mode
uint8  error_code
uint32 active_session_id
uint32 active_action_id
float32 current_height
float32 vertical_velocity
uint16 height_age_ms
uint16 command_age_ms
uint8  action_result
bool   height_valid
bool   armed
bool   airborne
bool   landed
bool   action_accepted
bool   action_completed
```
注释原文：**"本消息不包含飞控姿态"**。

#### `msg\FlightDirective.msg`（29 行）
常量：`STANDBY..RESET_MISSION`（**注意：本文件没有定义 `PROTOCOL_*`，与 `FlightCommand` 不同**）。
```
std_msgs/Header header
uint8  control_mode
uint32 session_id
uint32 action_id
float32 vertical_velocity
float32 target_height
bool   control_valid
```
注释：水平速度仍由 PID / control_mux 提供；连续模式 `action_id` 必须为 0，离散动作同 session 严格递增且非零。

#### `msg\Waypoint.msg`（26 行）
```
uint32  id
float64 x                 # map 系，m
float64 y                 # map 系，m
float64 yaw               # rad，逆时针为正
float64 target_height     # 激光基准正数离地高度，m，0 不代表降落
float64 hold_time         # s，>= 0
```

#### `msg\WaypointList.msg`（11 行）
```
std_msgs/Header header    # frame_id 必须 map
uint32 route_id
uav_interfaces/Waypoint[] waypoints
```

#### `msg\MissionStatus.msg`（36 行）
常量：`WAITING_FOR_ROUTE=0 / WAITING_FOR_PID=1 / TRACKING=2 / DWELLING=3 / COMPLETED=4 / ERROR=5`。
```
std_msgs/Header header
uint32 route_id
uint32 current_index
uint32 current_waypoint_id
uint32 waypoint_count
uint8  state
bool   route_valid
bool   target_sent
bool   mission_completed
```

#### `srv\SetWaypointList.srv`（9 行）
```
uav_interfaces/WaypointList waypoint_list
---
bool   accepted
string message
```

> **未确认**：`uav_interfaces\README.md`（11587 B）、`test\test_generated_interfaces.cpp`、`package.xml` 未逐字通读；如需接口约束细节需补读。

---

## 3. 问题 3：`flight_controller_bridge` 实现

### 3.1 节点结构 / 可执行文件

`CMakeLists.txt:20-43`：
- 静态库 `flight_controller_bridge_core`（只有 `src/protocol.cpp`，解码与坐标变换不依赖 ROS，便于单测复用）；
- **唯一 C++ 可执行节点：`flight_controller_bridge_node`**（`src/flight_controller_bridge_node.cpp`，1062 行）；
- `install(PROGRAMS ...)`（`CMakeLists.txt:52-56`）安装 3 个 Python 工具到 `lib/flight_controller_bridge`：
  `tools/fake_flight_controller.py`、`tools/fake_flight_controller_v2.py`、`tools/manual_v2_acceptance_publisher.py`（**伪飞控，不是生产节点**）；
- 测试：`test_protocol_and_coordinates`、`test_protocol_v2`（gtest）+ `test_bridge_launch.py`、`test_bridge_v2_launch.py`（`add_launch_test`，用伪终端模拟真实串口）。
- 包版本：`CMakeLists.txt:2` → `VERSION 1.1.0`，`package.xml:5` → `1.1.1`。

节点类 `FlightControllerBridgeNode : public rclcpp::Node`，构造于 `src/flight_controller_bridge_node.cpp:264-365`，节点名 `"flight_controller_bridge"`（`:265`）。
`main()` 在 `:1048-1062`，**单线程 `rclcpp::spin`**。

### 3.2 订阅的 ROS 话题 → 打包成的串口帧

| 话题 | 消息类型 | QoS | 代码位置 | 进入线缆帧的字段 |
|---|---|---|---|---|
| `/pid/cmd_vel` | `geometry_msgs/msg/TwistStamped` | `QoS(10).reliable()` | `:277-283` | `linear.x/y` → `velocity_x/y_mm_s`（经 map→body 旋转）；`angular.z` → `yaw_rate_mrad_s` |
| `/pid/control_valid` | `std_msgs/msg/Bool` | reliable | `:284-290` | 门控：无效则不发非零速度 |
| `/map/current_pose` | `geometry_msgs/msg/PoseStamped` | reliable | `:291-297` | 取 yaw 用于 map→body 旋转 |
| `/map/current_pose_valid` | `std_msgs/msg/Bool` | reliable | `:298-304` | 门控 |
| `/waypoint/current` | `uav_interfaces/msg/Waypoint` | `QoS(1).reliable().transient_local()` | `:305-316` | `target_height` → `target_height_mm`（V1）；变化时 `arm_new_target_guard()` |
| `/mission/status` | `uav_interfaces/msg/MissionStatus` | transient_local | `:317-329` | 航点/任务有效性门控 |
| `/mission/land_request` | `std_msgs/msg/Bool` | transient_local | `:330-336` | V1 `land_request` flag |
| `/flight/directive` | `uav_interfaces/msg/FlightDirective` | transient_local | `:337-354` | **V2 全部离散动作与垂直控制来源** |

**发布的话题**（`:270-273`）：

| 话题 | 类型 | QoS |
|---|---|---|
| `/flight/command` | `uav_interfaces/msg/FlightCommand` | `QoS(10).reliable()` — 每个定时周期都发（`:914`、`:921`），**这是给上层看的"我实际发了什么"镜像** |
| `/flight/status` | `uav_interfaces/msg/FlightStatus` | `QoS(10).reliable()` — 每周期发（`:938`） |

发送路径：`timer_callback()`（`:901-959`）→ `build_v1_command()`（`:697-738`）或 `build_v2_command()`（`:740-860`）→ `encode_command_frame()` / `encode_v2_command_frame()`（`src/protocol.cpp:86-104`、`:142-165`）→ `serial_.write_all()`。

### 3.3 收到的串口帧 → 解包成的 ROS 话题

- 读串口：`read_flight_status()`（`:506-529`），逐字节喂给 `StatusStreamParser::push()` / `V2StatusStreamParser::push()`（`src/protocol.cpp:279-349`；这两段用 `std::search` 搜帧头，即**抗错位重同步**）。
- **ACK 窗口校验**：`accept_acknowledged_status()`（`:487-504`）要求 `sent_sequence_available_` 且
  `acknowledgement_is_recent(last_sent_sequence_, acknowledged_sequence, max_ack_lag_frames_)`（`src/protocol.cpp:260-266`，实现即 `uint8(last_sent - ack) <= max_lag`）。
  不符合的帧被丢弃并计 `rejected_ack_frame_count_`；合法的才置 `feedback_received_ = true`。
- 解包结果**只落到** `/flight/status`（`publish_flight_status()`，`:862-899`）。**桥不发任何"原始飞控数据"话题**——姿态、位置、电池、GPS 全都没有。

### 3.4 串口设备名：参数，不是硬编码

`declare_and_load_parameters()`（`:368-393`），默认值原文（`:370-373`）：

```cpp
protocol_version_ = declare_parameter<int>("protocol_version", 1);
serial_device_    = declare_parameter<std::string>("serial_device", "/dev/ttyUSB0");
baud_rate_        = declare_parameter<int>("baud_rate", 115200);
send_rate_hz_     = declare_parameter<double>("send_rate_hz", 50.0);
```

实际生产参数文件覆盖（`config\flight_controller_bridge_params.yaml`）：
```yaml
protocol_version: 2                                        # :6
serial_device: "/dev/flight_controller"                    # :10  （udev 固定 CH340；注释警告不要用 /dev/ttyACM0——雷达占用）
baud_rate: 115200                                          # :11
send_rate_hz: 50.0                                         # :14
reconnect_interval_ms: 1000                                # :15
flight_feedback_timeout_ms: 300                            # :18
max_ack_lag_frames: 10                                     # :22
require_flight_feedback_before_control: true               # :26
map_frame: "odom"                                          # :31
body_frame: "base_link"                                    # :32
pid_command_timeout_ms: 300                                # :33
new_target_pid_guard_ms: 100                               # :35
pose_timeout_ms: 300                                       # :36
mission_status_timeout_ms: 500                             # :37
land_request_timeout_ms: 500                               # :38
directive_timeout_ms: 500                                  # :40
max_linear_speed: 1.5                                      # :47
max_vertical_speed: 0.5                                    # :48
max_yaw_rate: 2.0                                          # :49
min_target_height: 0.1                                     # :50
max_target_height: 1.7                                     # :51  （注释：与飞控V2生产上限1700mm一致）
print_status: true                                         # :57
```
验收专用参数 `config\flight_controller_bridge_v2_acceptance.yaml:5`：
`serial_device: "/dev/serial/by-path/platform-xhci-hcd.1-usb-0:1:1.0-port0"`。

串口初始化在 `SerialPort::open_port()`（`:46-89`）：`cfmakeraw` + `CLOCAL|CREAD` + `~CSTOPB` + `~PARENB` + `CS8` + `~CRTSCTS`，`VMIN=0/VTIME=0`，`tcflush(TCIOFLUSH)`，随后清掉 `O_NONBLOCK`。
**参数校验**（`:395-432`）：`protocol_version` 只能 1 或 2；`send_rate_hz` ∈ [1,200]；`max_ack_lag_frames` ∈ [0,127]；`new_target_pid_guard_ms` ∈ [0,1000]；高度上下限受 `kV2ContinuousMinHeightM=0.1` / `kV2MaxTargetHeightM=1.7` 约束。

### 3.5 发送频率与看门狗

- **树莓派侧发送频率 = `send_rate_hz_` = 50 Hz**（`config\...params.yaml:14`，默认也是 50.0）。
  定时器创建于 `:356-359`：`period = 1.0 / send_rate_hz_`，`create_wall_timer(...)`。
- **飞控侧命令看门狗 = 200 ms**（由对端实现，见 `docs\serial_protocol_v2.md:11`；桥只用 50 Hz 持续刷新）。
- **桥侧反馈超时 = 300 ms**（`flight_feedback_timeout_ms`，`:375`），超时 → `communication_ok=false`（`:867`）。
- 桥**不自行解锁/解锁由飞控端遥控开关决定**（`config\...params.yaml:59-60` 注释原文："本包不会自行解锁飞控；是否允许树莓派接管仍应由飞控端遥控开关决定。"）。
- 参数非法时构造抛异常 → `main()` 捕获并 `RCLCPP_FATAL "飞控桥启动失败：%s"`，返回 1（`:1052-1059`）。

### 3.6 安全门（迁移时必须保留的逻辑）

1. `require_flight_feedback_before_control_`（`:722-725`）：没有新鲜飞控反馈 → 只发无效零速帧。
2. `feedback_failsafe()`（`:537-540`、`:726-729`、`:806-809`）：飞控报 failsafe → 不发控制量。
3. 超范围**不截断**，而是整帧 `control_valid=false`（`:656-659`）。
4. `arm_new_target_guard()`（`:441-447`）：目标身份（waypoint id / target_height / route_id / current_waypoint_id / directive 模式与 session/action）变化后，**只接受保护时间之后新产生的 PID 命令**（`new_target_pid_guard_ms=100`），防止新目标配旧速度。
5. V2 额外门（`:768-823`）：`control_valid` 必须为真；`session_id != 0`；连续模式 `action_id` 必须 0、离散模式必须非 0；`v2_mode_uses_height()` 时高度必须合法；`mode != STANDBY` 时必须已有新鲜反馈且 `active_session_id` 已回显一致，否则只发无效帧（`reason = "V2 session尚未回显确认"`）。
6. V2 `HEIGHT_HOLD` 下若定位/PID 无效，则降级为 **水平零速但 control_valid=true**（`:830-841`，`reason = "V2定高有效，定位/PID无效时水平零速："`）。

---

## 4. 问题 4：状态反馈——飞控回传了什么

**飞控回传的全部内容 = V1 的 4 个 bool，或 V2 的下列字段。桥把它打包成 `/flight/status`（`uav_interfaces/msg/FlightStatus`）。**

V2 状态帧字段（`protocol.hpp:147-167` 结构体 `V2WireStatus`；解析在 `src/protocol.cpp:192-224`）：

| 字段 | 类型 | 映射到 `/flight/status` |
|---|---|---|
| `acknowledged_sequence` | u8 | 不映射（仅用于 ACK 窗口判定） |
| `current_mode` | 0~5 | `current_mode`（`node.cpp:875`） |
| `height_valid` | bool | `height_valid`（`:886`） |
| `height_reached` | bool | `height_reached`（`:887`；注释明确**树莓派任务状态机不得用它推进任务**） |
| `armed` | bool | `armed`（`:888`） |
| `airborne` | bool | `airborne`（`:889`） |
| `landed` | bool | `landed`（`:890`） |
| `action_accepted` | bool | `action_accepted`（`:891`） |
| `action_completed` | bool | `action_completed`（`:892`） |
| `failsafe` | bool | `failsafe`（`:893`） |
| `error_code` | 0x00~0x11 | `error_code`（`:876`） |
| `active_session_id` | u32 | `active_session_id`（`:877`） |
| `active_action_id` | u32 | `active_action_id`（`:878`） |
| `current_height_mm` | u16 | `current_height`（÷1000 转 m；`height_valid=false` 时强制 0.0，`:879-880`） |
| `vertical_velocity_mm_s` | i16 | `vertical_velocity`（÷1000，`:881-882`） |
| `height_age_ms` | u16 | `height_age_ms`（`:883`） |
| `command_age_ms` | u16 | `command_age_ms`（`:884`） |
| `action_result` | 0~4 | `action_result`（`:885`） |

派生量（`:894-896`）：
```cpp
status.land_accepted  = status.action_accepted && latest_v2_status_.current_mode == V2ControlMode::kFinalLand;
status.land_completed = status.landed && status.action_completed;
```

`communication_ok` **不在线缆里**，是本地推断的（`:867`）：`feedback_is_fresh(now)` = 300 ms 内收到过通过 ACK 窗口校验的状态帧。

### 明确"没有"的状态（这是 PX4 对照的关键）

- **没有姿态**（`FlightStatus.msg` 注释原文"本消息不包含飞控姿态"，即无 roll/pitch/yaw、无角速度）；
- **没有飞控解算的位置**（无 x/y/z 除了 `current_height`；map 位置由树莓派 SLAM 提供，见 `/map/current_pose`）；
- **没有电池**（电压/电流/剩余 %）；
- **没有 GPS / 卫星数 / 定位类型**；
- **没有 RC 遥控信号质量**；
- **没有水平速度反馈**（只有 `vertical_velocity`）；
- V1 分支下 `current_height`、`vertical_velocity`、`height_age_ms`、`command_age_ms`、`action_result`、`armed/airborne/landed` **全部不填**（`:869-873` 只填 4 个 bool + `height_reached`），`current_mode`/`error_code` 在 V1 下也未赋值 → **默认 0**。

---

## 5. 问题 5：解锁 / 模式切换 / 起飞 / 降落

### 5.1 四端协议 v1.7 里：没有

`D_UAV_UNIFIED_DEVICE_PROTOCOL_v1.7.md` **完全没有**解锁、切模式、起飞、降落的遥控命令。起飞/降落是树莓派按 `mission_phase` 状态机、通过**独立飞控 V2 协议**发出的：
- 行219-220："通过独立树莓派—飞控V2协议发送 TAKEOFF，目标高度 1500 mm"；
- 行507："向飞控发送最终降落动作"。

### 5.2 V1/V2 线缆协议里：有，但语义受限

| 能力 | V1 | V2 |
|---|---|---|
| 起飞 | 无独立起飞命令。靠 `control_valid=1` + `target_height`，由飞控本地 z PID 爬到该高度 | `control_mode=1 TAKEOFF`，`action_id>0`，高度必须 300~1700 mm |
| 高度保持 | `control_valid=1` + `target_height` | `control_mode=2 HEIGHT_HOLD`，`action_id=0` |
| 精密下降 | 无 | `control_mode=3 PRECISION_DESCENT`，带 `velocity_z` 与下降安全下限 |
| 降落 | `flags.bit1 land_request=1`（必须同时 `control_valid=1`，`(0,1)` 组合非法） | `control_mode=4 FINAL_LAND`，全零 + `action_id>0`；**一旦 accepted 就脱离树莓派重发依赖** |
| 复位任务 | 无 | `control_mode=5 RESET_MISSION`，**只复位任务状态，不解锁不起飞** |
| 解锁 arm | **没有任何命令**。`status_flags.bit2 armed` 只是飞控**上报**的状态 | 同 V1，只有 `status_flags.bit2 armed` 上报 |
| 模式切换 | 隐式：靠 `control_valid` + `land_request` 组合 | 显式 6 个 `control_mode` |

**关键点：整条链路里没有"解锁"命令。** `docs\serial_protocol_v2.md:139-146` 明确"遥控失联和主飞控保护始终更高优先级"；`config\flight_controller_bridge_params.yaml:59-60` 明确"本包不会自行解锁飞控"。解锁由飞控端遥控开关决定，树莓派只是**请求控制权**并读取 `armed`。

### 5.3 桥里的实现

- **V1 降落闩锁** `update_v1_land_latch()`（`:542-567`）：必须同时满足"任务状态新鲜（500 ms）+ 降落请求新鲜（500 ms）+ `route_valid` + `mission_completed`"，才置 `land_latched_`；再要求 `feedback_is_fresh` 且状态帧晚于闩锁时刻、且 `land_completed` 才置 `land_completed_latched_`。`route_id` 变化会复位两个闩锁。
  满足后 `build_v1_command()`（`:713-721`）强制 `control_valid=true, land_request=true`，`control_mode=kFinalLand`。
- **V2 模式**：完全由 `/flight/directive`（`uav_interfaces/msg/FlightDirective`）驱动（`:752-860`），桥不做状态机；session 握手规则见 §3.6 第 5 条。
- 桥**不产生** `TAKEOFF`/`FINAL_LAND` 本身，只翻译上层指令 → 由 `waypoint_manager` 包的 `competition_flight_directive_node`（`src\competition_flight_directive_node.cpp:38` 发 `/flight/directive`）决定。

---

## 6. 问题 6：与 PX4 1.17.0 + uXRCE-DDS Offboard 逐条对照

> 本机已有权威参考（**就是这套 PX4 1.17 的现场笔记**）：
> `docs/PX4树莓派伴飞实战路线.md`、`docs/PX4学习记录总结_至2026-10-06.md`（已随 2026-08-04 的整理并入本仓库 `docs/`）。
> 下列 PX4 话题名与消息类型与这两份笔记一致（uORB 消息名 PascalCase，ROS 话题名 snake_case）。

### 6.1 PX4 有直接对应（给出 ROS 2 话题）

| 平台现有能力 | PX4 对应 ROS 2 话题 | PX4 消息类型 | 说明 / 差异 |
|---|---|---|---|
| 起飞到指定高度（V2 `TAKEOFF`） | `/fmu/in/trajectory_setpoint`（NED，z 为负） | `TrajectorySetpoint` | PX4 无"高度目标 + 自己爬"的独立命令；要么靠位置设定点，要么发 `VEHICLE_CMD_NAV_TAKEOFF`（22，见 6.2 注释） |
| 高度保持（V2 `HEIGHT_HOLD`） | `/fmu/in/trajectory_setpoint` + `/fmu/in/offboard_control_mode` | `TrajectorySetpoint` / `OffboardControlMode` | 保持位置时 `TrajectorySetpoint` 发一次即可，但 `OffboardControlMode` **必须持续流** |
| 机体系水平速度控制（vx/vy） | `/fmu/in/trajectory_setpoint`（`velocity` 字段） | `TrajectorySetpoint` | **坐标系变了**：现协议 x 前 / y 左；PX4 NED 是 x 前 / **y 右**。y 要取反，并且要先把 `base_link` 航向转成 NED 朝向 |
| yaw 速率控制（V2 `yaw_rate_mrad_s`） | `/fmu/in/trajectory_setpoint`（`yawspeed`）或 `TrajectorySetpoint.yaw` | `TrajectorySetpoint` | 现协议逆时针为正（z 上）；NED 下 z 向下，符号需核对 |
| 垂直速度控制（V2 `PRECISION_DESCENT` 的 `velocity_z`） | `/fmu/in/trajectory_setpoint`（`velocity[2]`） | `TrajectorySetpoint` | 现协议"向上为正"，NED 是"向下为正" → **z 速度取反** |
| 最终降落（V2 `FINAL_LAND`） | `/fmu/in/vehicle_command`（`VEHICLE_CMD_NAV_LAND`=21，或 DO_SET_MODE→AUTO.LAND） | `VehicleCommand` | PX4 有原生降落检测（`/fmu/out/vehicle_land_detected`），不需要自己判"落地" |
| 解锁 / 上锁 | `/fmu/in/vehicle_command`（`VEHICLE_CMD_COMPONENT_ARM_DISARM`=400，`param1=1/0`） | `VehicleCommand` | **新增能力**：现协议只能被动读 `armed` |
| 进入 offboard 模式 | `/fmu/in/vehicle_command`（`MAV_CMD_DO_SET_MODE`=176，`param1=1`，`param2=6`） | `VehicleCommand` | DO_SET_MODE 176 / p1=1 / p2=6 |
| 系统 armed 状态（V2 `status_flags.bit2`） | `/fmu/out/vehicle_status` | `VehicleStatus` | 字段名与取值需按 `px4_msgs` 实际版本核对 |
| airborne / landed（V2 `bit3/bit4`） | `/fmu/out/vehicle_land_detected` | `VehicleLandDetected` | |
| failsafe（V2 `bit7`） | `/fmu/out/failsafe_flags` | `FailsafeFlags` | |
| 当前高度（V2 `current_height_mm`） | `/fmu/out/vehicle_local_position`（`z`，NED 向下为正；`dist_bottom` 为对地距离） | `VehicleLocalPosition` | 现协议是"激光基准离地高度"；PX4 里 `dist_bottom` 才是离地距离，还带 `dist_bottom_valid` / `dist_bottom_sensor_bitfield` |
| 垂直速度（V2 `vertical_velocity_mm_s`） | `/fmu/out/vehicle_local_position`（`vz`） | `VehicleLocalPosition` | 符号需转换 |
| "命令看门狗 200 ms / 50 Hz 重发" | `/fmu/in/offboard_control_mode` 持续流 + 参数 `COM_OF_LOSS_T`（默认 0.5 s） | `OffboardControlMode` | 机制等价但**由飞控执行**：断流即自动退出 offboard，不需要在飞控里写看门狗 |
| "链路是否新鲜"（V2 `command_age_ms` / `height_age_ms`） | 无直接对应 | — | 可用本地"上次收到 `/fmu/out/*` 的时间"推算（见 6.2） |
| 外部位姿回灌（现工程靠 SLAM 出 `/map/current_pose`） | `/fmu/in/vehicle_odometry` | `VehicleOdometry` | 这是把 SLAM 位置喂给 EKF2 的**唯一正确通道**；不发就"目标−当前"没有反馈量 |

### 6.2 PX4 没有直接对应、必须自己实现

1. **session_id / action_id 幂等与去重**。
   V2 规定"离散动作唯一键 = session_id + action_id + control_mode""同 action 重发返缓存""同 action 改参返 DUPLICATE_ACTION"。
   PX4 只有 `VehicleCommand` + `/fmu/out/vehicle_command_ack`，**没有 action 幂等层**。→ 必须在树莓派侧保留一个薄的 action 管理层。
2. **应用层 ACK / 结果语义（`action_result` = NONE/PENDING/SUCCEEDED/REJECTED/FAILED）**。
   PX4 是 `vehicle_command_ack`（result 值取自 MAVLink `MAV_RESULT`），语义粒度不同。→ 需要自己做映射与超时判定。
3. **"相对激光基准的目标高度"三层语义**。
   现协议：`min_target_height=0.1`（连续模式）/ `300 mm`（起飞）/ `1700 mm`（生产上限），且明确"有符号数须结合 VALID 标志"。
   PX4 的 `TrajectorySetpoint.z` 是 NED 绝对高度（相对 EKF2 原点），**没有"目标离地高度"字段**。→ 必须自己把"离地高度"转成 `z = -(高度)` 或 `dist_bottom` 闭环，并实现上下限与 `height_valid` 的等价物。
4. **数据年龄（`height_age_ms` / `command_age_ms`）**。
   PX4 不提供"该值多旧"。→ 用订阅回调时间戳自己算。
5. **响应式限幅 vs 直接拒绝**。
   现桥的做法是"超范围 → 整帧 `control_valid=false` 并保持零速"（`config\...params.yaml:45-46` 注释原文）。
   PX4 的位置/速度限幅在飞控内（MPC 参数），树莓派侧若还要"越界即拒绝"，必须自己写。
6. **`RESET_MISSION` 的独立语义**（只复位任务状态，不解锁不起飞）。
   PX4 无对应命令。→ 变成纯粹的树莓派侧任务状态机复位 + 发一个零设定点。
7. **"起飞到高"的到高判据**：现协议是目标 ±100 mm 连续 100 ms（5 个 20 ms 周期）。
   PX4 的 NAV_TAKEOFF 走自己的 takeoff 逻辑。→ 若沿用"到高"语义，需自己用 `z` 做窗口判断。
8. **舵机投放（PAYLOAD_RELEASED）**：四端协议 v1.7 里由摄像头直驱舵机；若要交给 PX4，`/fmu/in/actuator_servos` 存在，但需先在 PX4 侧配置好对应功能与输出通道——**未确认 MicoAir H743 v2 该版本固件的具体配置方式**。

### 6.3 在 PX4 下必须推翻的设计

| 必须推翻的东西 | 位置 | 原因 |
|---|---|---|
| **整个 V1/V2 线缆协议层**：帧头、CRC16-CCITT-FALSE、30 字节定长、`0x0D 0x0A` 帧尾 | `src\protocol.cpp` 全文（351 行）、`include\flight_controller_bridge\protocol.hpp` | 被 uXRCE-DDS 取代，串口不再有自定义帧 |
| **`SerialPort`（termios 手写串口）与 `reconnect_interval_ms`、`baud_rate`、`serial_device` 三个参数** | `node.cpp:38-120`、`:370-372`、`:457-485` | DDS Agent 走 `/dev/ttyACM0` 或 UDP，波特率/设备名由 agent 与 `UXRCE_DDS_CFG` 决定，不是 ROS 节点参数 |
| **ACK 窗口机制 `max_ack_lag_frames` / `acknowledgement_is_recent` / `rejected_ack_frame_count_`** | `node.cpp:487-504`、`protocol.cpp:260-266` | DDS 是可靠传输（QoS 层），没有"ACK 落后 N 帧"概念 |
| **sequence 分类 `classify_transport_sequence`（重复/新/旧）** | `protocol.cpp:268-277` | 同上 |
| **`control_valid` 作为"控制权开关"的整体设计** | `node.cpp` 多处 | PX4 的控制权由**模式**决定：必须进 offboard + 持续流 `OffboardControlMode`；没有 `control_valid` 位 |
| **"200 ms 命令看门狗"要在飞控端实现**的假设 | `docs\serial_protocol_v2.md:11` | PX4 用 `COM_OF_LOSS_T`（默认 0.5 s）自动退出 offboard |
| **飞控端 z 环 / 降落流程由凌霄固件实现** | `docs\serial_protocol_v2.md:139-146` | 换成 PX4 的 EKF2 + 位置环 + land detector |
| **坐标系假设"x 前 y 左 z 上，yaw 逆时针为正"** | `coordinates.hpp:9-20`、`docs\serial_protocol_v2.md` | PX4 是 NED（x 前 y 右 z 下）；`map_velocity_to_body()` 的旋转公式必须重写或改写调用点 |
| **`FlightCommand.protocol_version`（PROTOCOL_V1/V2）与 `current_mode`（STANDBY..RESET_MISSION 0~5）** | `uav_interfaces\msg\FlightCommand.msg`、`FlightStatus.msg` | 这两个枚举直接复刻线缆协议，迁移后语义不存在 |
| **"树莓派外环 PID + 飞控内环"的双环组合** | 现工程 `position_pid_controller` → `/pid/cmd_vel` → 桥 | ⚠ **高风险**：PX4 的位置环已经在飞控内，若树莓派再叠一层位置 PID 输出速度，会形成串级冲突/震荡。必须二选一（建议：树莓派只出速度或位置设定点，位置环交给 PX4） |
| **舵机/投放由摄像头直驱的那部分**（如决定改由 PX4 出 PWM） | 四端协议 v1.7 第 9.3 节 | PX4 不鼓励直接发 PWM；要用 `actuator_servos` 或 MAVLink 命令 |

### 6.4 PX4 侧必须补齐的三件事（不补就飞不起来）

1. **位姿源**：offboard 位置模式必须有 position/pose 估计。无 GPS 的室内必须先上光流+测距（走 `dist_bottom`），或把 SLAM 位姿用 `/fmu/in/vehicle_odometry` 回灌 EKF2。
2. **QoS 必须改**（沿用现工程的 reliable QoS **一定收不到数据**）：
   - 订阅 PX4 发布 → `BEST_EFFORT` + `TRANSIENT_LOCAL` + `KEEP_LAST(1)`；
   - 向 PX4 发布 → `BEST_EFFORT` + `VOLATILE` + `KEEP_LAST(1)`。
3. **`timestamp` 字段必填**（微秒级）：`msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)`。

**另外三个必须改的参数**（出自本机笔记 `PX4树莓派伴飞实战路线.md:600-608`）：
`COM_OF_LOSS_T`（默认 0.5 s）、`COM_OBL_RC_ACT`（默认 Position）、`COM_RC_OVERRIDE`（**offboard 下默认不生效，打杆抢不回控制权**）。

---

## 7. 迁移到 PX4 的改造方案建议

### 7.1 结论：`flight_controller_bridge` 整体替换，不改造

`flight_controller_bridge` 的存在理由是"把 ROS 速度翻译成凌霄自定义串口帧"。PX4 下这个理由消失（uXRCE-DDS 直接把 uORB 映射成 ROS 话题）。**建议新写一个 `px4_offboard_bridge` 包，旧包保留只读不改**，理由是：旧包有完整测试与验收记录，且 V1/V2 伪飞控工具可用于回归；迁移期两套并行便于对比。

**删除/停止安装的**：
- `src\flight_controller_bridge_node.cpp`（1062 行）、`src\protocol.cpp`（351 行）、`include\...\protocol.hpp`；
- `config\flight_controller_bridge_params.yaml` 中 `serial_device` / `baud_rate` / `reconnect_interval_ms` / `max_ack_lag_frames` / `protocol_version` 五项；
- `tools\fake_flight_controller*.py`（改为用 PX4 SITL 或 `px4_msgs` 假发布者）。

### 7.2 新节点建议：`px4_offboard_bridge`

**订阅（保留原有上游契约，一行不改上层代码）**：
| 话题 | 类型 | 用途 |
|---|---|---|
| `/pid/cmd_vel` | `TwistStamped` | 水平速度来源（**需确认是否仍要保留外环 PID，见 6.3 最后一行**） |
| `/pid/control_valid` | `Bool` | 门控 |
| `/map/current_pose` + `/map/current_pose_valid` | `PoseStamped` + `Bool` | 位姿（→ 回灌 `/fmu/in/vehicle_odometry`） |
| `/waypoint/current`、`/mission/status`、`/mission/land_request` | 现有 `uav_interfaces` 类型 | 任务语义 |
| `/flight/directive` | `uav_interfaces/msg/FlightDirective` | **离散动作入口，建议保留此接口** |

**发布（保持 `/flight/status` 的话题名与消息类型不变）**：
`/flight/status`（`uav_interfaces/msg/FlightStatus`）——语义重映射为：

| `FlightStatus` 字段 | PX4 来源 |
|---|---|
| `communication_ok` | 本地：最近 N ms 内是否收到 `/fmu/out/vehicle_status` |
| `armed` / `airborne` / `landed` | `/fmu/out/vehicle_status`、`/fmu/out/vehicle_land_detected` |
| `failsafe` | `/fmu/out/failsafe_flags` |
| `current_height` | `/fmu/out/vehicle_local_position` 的 `dist_bottom`（离地）或 `-z` |
| `height_valid` | `dist_bottom_valid` 或 `z_valid` |
| `vertical_velocity` | `vehicle_local_position.vz`（符号按需转换） |
| `height_age_ms` / `command_age_ms` | 本地用回调时间戳算（PX4 不提供） |
| `current_mode` | `/fmu/out/vehicle_status` 的模式字段（需按 `px4_msgs` 版本核对具体字段名） |
| `error_code` / `action_result` | 本地维护的薄 action 管理层（见 6.2 第 1、2 条） |
| `active_session_id` / `active_action_id` | 本地维护（PX4 无此概念） |
| `height_reached` | 本地窗口判断（沿用 ±100 mm / 100 ms 逻辑，如果仍需） |

**新发布（PX4 侧）**：
- `/fmu/in/offboard_control_mode`（`OffboardControlMode`）——**50 Hz 持续发，这是心跳，断流 = 退出 offboard**；
- `/fmu/in/trajectory_setpoint`（`TrajectorySetpoint`）——NED；未用字段填 NaN；
- `/fmu/in/vehicle_command`（`VehicleCommand`）——arm/disarm(400)、DO_SET_MODE(176,p1=1,p2=6)、NAV_LAND(21)；
- `/fmu/in/vehicle_odometry`（`VehicleOdometry`）——把 SLAM/雷达位姿回灌（**最关键的一条**）。

### 7.3 必须保留的接口与逻辑

**保留的 ROS 接口**（这样 `waypoint_manager`、`competition_mission_node`、`car_serial_bridge`、`gcs_serial_bridge` 全部不用改）：
1. `/flight/status`（类型不变）——被 `src\car_serial_bridge_node.cpp:162`、`src\gcs_serial_bridge_node.cpp:268`、`waypoint_manager`、`competition_mission_node:131`、`competition_flight_directive_node:49` 消费；
2. `/flight/directive`（类型不变）——由 `src\competition_flight_directive_node.cpp:38` 发布；
3. `/pid/cmd_vel`、`/pid/control_valid`、`/map/current_pose`、`/map/current_pose_valid`、`/waypoint/current`、`/mission/status`、`/mission/land_request`。

**保留的逻辑（原封搬过来，只是执行者从"串口帧"变成"PX4 话题"）**：
1. 三道安全门：`require_flight_feedback_before_control`、`failsafe` 短路、**超范围整帧置零而不是截断**；
2. 全部超时参数（`pid_command_timeout_ms=300`、`pose_timeout_ms=300`、`mission_status_timeout_ms=500`、`land_request_timeout_ms=500`、`directive_timeout_ms=500`）；
3. `arm_new_target_guard_ms=100` 的"新目标不接受旧速度"保护；
4. 速度/高度上限：`max_linear_speed=1.5`、`max_vertical_speed=0.5`、`max_yaw_rate=2.0`、`max_target_height=1.7`；
5. `/flight/command` 这个"实际发出的控制量镜像"话题——**建议保留并改名语义**（现在是 `FlightCommand`，含 `protocol_version`），迁移后建议改为发布 `px4_msgs/TrajectorySetpoint` 或新增一个纯净的 `OffboardCommand` 消息，便于地面站/日志对齐。

### 7.4 分阶段落地

| 阶段 | 内容 | 验收 |
|---|---|---|
| S0 | 建 `px4_msgs`（对应 1.17 的分支）+ 新桥骨架，只订阅 `/fmu/out/*` 并填 `/flight/status` | 无桨：`ros2 topic echo /flight/status` 能看到 `armed/height_valid` 随晃动变化 |
| S1 | 上位姿回灌 `/fmu/in/vehicle_odometry` | `ros2 topic echo /fmu/out/vehicle_local_position` 的 x/y 稳定、`dead_reckoning=false` |
| S2 | 打通 offboard：50 Hz `OffboardControlMode` + `TrajectorySetpoint` + DO_SET_MODE | 无桨：能进 offboard、能 arm、z 随指令变化、**拔串口线自动退出 offboard** |
| S3 | 接回任务层：`/flight/directive` → 离散动作，`/mission/land_request` → NAV_LAND | 桌面：起飞→悬停→降落 全链路无桨通过 |
| S4 | 恢复四端协同协议 v1.7 的 `FLIGHT_LINK_OK` / `HEIGHT_VALID` / `ARMED` 位 | 地面站看到 `uav_flags` 与飞控真实状态一致 |
| S5 | 带桨飞行 + 90 s 任务时序回归（含 3 s 悬停、0.375 m 右移） | 电赛 D 题计时达标 |

### 7.5 风险清单（按严重度）

| 风险 | 说明 |
|---|---|
| **串级 PID 冲突（高）** | 树莓派 `position_pid_controller` 出速度 + PX4 内部位置环，双层闭环极易震荡。必须先决定"位置环放哪一边" |
| **坐标系符号（高）** | 现协议 y 左 / z 上 / yaw 逆时针；NED y 右 / z 下。y 速度、z 速度、yaw 速率三处符号必须逐项核对，`coordinates.hpp:34-43` 的旋转公式不再适用 |
| **高度语义（高）** | 现协议 `target_height` 是"激光基准离地高度"；PX4 `TrajectorySetpoint.z` 是 NED 绝对高度。"0.1~1.7 m 越界拒绝"逻辑不能直接平移 |
| **QoS 不匹配（中高）** | 沿用现在 `QoS(10).reliable()` 会完全收不到 PX4 数据（静默失败，最难查） |
| **timestamp 未填（中高）** | 未填微秒时间戳的消息会被 PX4 丢弃 |
| **`COM_RC_OVERRIDE` 默认关闭（中）** | 首飞前不改，打杆抢不回控制权 |
| **无位置源就进不了 offboard（中）** | 室内必须先上光流+测距或回灌 odometry，否则 offboard 位置模式根本不生效 |
| **`FLIGHT_LINK_TIMEOUT` 语义变化（中）** | 现在 = 300 ms 没有合法 ACK 帧；迁移后 = 300 ms 没有 `/fmu/out/vehicle_status`。门限需重新标定 |
| **V1 分支的 `FlightStatus` 字段空洞（低，但会误导上云）** | `node.cpp:869-873` 在 V1 下只填 4 个 bool，`current_height`/`current_mode`/`error_code` 为默认 0；地面站可能把"0"当成真实值。迁移时不要复制这个模式 |

---

## 8. 明确标注"未确认"的项

1. `flight_controller_bridge\README.md`（15321 B）、`docs\2026-07-28_flight_controller_integration_test_report.md`（19410 B）、`docs\old_flight_controller_findings.md` **未逐字通读**——如需实测数据/历史故障细节需补读。
2. `uav_interfaces\README.md`、`test\test_generated_interfaces.cpp`、`package.xml` **未读**。
3. `flight_controller_bridge_node.cpp:121-184`（SerialPort 其余部分与 `finite_twist`/`quaternion_to_yaw` 辅助函数）**未逐行读**，但已由 `:185-215` 的定点换算与 `:623-695` 调用点覆盖其行为。
4. PX4 1.17 `px4_msgs` 中 `VehicleStatus` 表示飞行模式/arming 状态的**确切字段名与取值枚举未逐字核对**（本机笔记只列到话题级与 `VehicleLocalPosition` 字段级）。
5. MicoAir H743 v2 上 PX4 1.17.0 固件里 uXRCE-DDS 的**默认串口与 `UXRCE_DDS_CFG` 取值未确认**。
6. 本机 **未通过 ssh 连树莓派查运行时状态**（用户提示该命令可用，但本次分析完全基于源码与文档即可回答全部 6 问；运行时的 `ros2 topic list` / `dmesg` 结果可另行补充）。
