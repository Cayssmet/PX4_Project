# D题陆空协同系统统一设备通信协议（v0.1-draft）

> 已废止：本讨论稿已被 `D_UAV_UNIFIED_DEVICE_PROTOCOL_v1.7.md` 取代。实现时不得继续使用本稿的通用长帧和 CRC16。

日期：2026-07-29  
适用设备：循线小车、树莓派、视觉摄像头、地面站  
状态：字段讨论稿；四端确认后再冻结为 v1.0 并生成固定字节向量

## 1. 目标与边界

本协议统一四类设备的任务编号、任务阶段、坐标、单位、帧头、序号和 CRC。各设备不再用不同数字表示同一个阶段。

本协议只用于以下链路：

1. 小车 → 树莓派：任务选择、START、小车状态、赛道分段和行驶距离；
2. 小车 → 地面站：与发给树莓派相同的小车遥测；
3. 树莓派 ↔ 摄像头：视觉模式请求、相对位移和视觉状态；
4. 树莓派 → 地面站：无人机地图坐标、全局任务阶段和各链路状态；
5. 各设备 → 地面站：可选心跳和版本信息。

它不替换以下现有协议：

- 树莓派—凌霄飞控继续使用独立的 `D-UAV UART V1/V2`；
- N10 雷达继续使用厂商协议；
- 飞控调试文本不得混入任何二进制控制串口。

## 2. 状态机职责

### 2.1 唯一裁决原则

树莓派是全局 `mission_phase` 的唯一裁决者。

- 小车产生一次任务的 `session_id`、`task_mode` 和持续置位的 START，并上报自身 `car_state`；
- 摄像头上报 `vision_state`、相对位移和对准标志，但不能自行宣布抛投或降落完成；
- 树莓派综合小车、视觉、SLAM、PID 和飞控状态后推进全局任务阶段；
- 地面站只融合显示，不在正式比赛中发送起飞、投放或降落指令；
- 飞控仍执行自身安全状态机，全局任务阶段不能越过飞控保护。

这样可以避免小车、摄像头和树莓派同时推进同一个状态而发生竞争。

### 2.2 任务编号 `task_mode`

| 值 | 名称 | 含义 |
|---:|---|---|
| `0x00` | NONE | 尚未选择任务 |
| `0x01` | TASK1_DROP | 抛投任务，主要得分项 |
| `0x02` | TASK2_DYNAMIC_LAND | 动态起降任务 |

`task_mode` 由小车在起动前选择。树莓派只在 READY/WAIT_START 接受新任务；飞行中收到不同任务或不同 session 不切换当前任务。

### 2.3 全局任务阶段 `mission_phase`

| 值 | 名称 | 说明 |
|---:|---|---|
| `0x00` | BOOT | 上电启动 |
| `0x01` | SELF_CHECK | 自检和等待各链路 |
| `0x02` | READY | 设备就绪、允许选择任务 |
| `0x03` | WAIT_START | 已获得合法任务，等待小车 START |
| `0x04` | START_ACCEPTED | START 已按 session 锁存 |
| `0x10` | TAKEOFF | 垂直起飞到 1.5 m |
| `0x11` | TASK1_HOVER_3S | 仅 task1：到高后累计稳定悬停 3 s |
| `0x12` | RIGHT_SHIFT | 右移 0.75 m；BODY `vy<0` |
| `0x13` | FORWARD_CHASE | 沿初始机头方向前追 |
| `0x14` | SEARCH_CAR | 请求视觉搜索小车/平台 |
| `0x15` | FOLLOW_CAR | 已形成视觉伴飞 |
| `0x20` | TASK1_ALIGN_DROP | task1：C-D 直道投放窗口内对准 |
| `0x21` | TASK1_DROP | task1：执行一次投放 |
| `0x22` | TASK1_DROP_DONE | task1：投放机构确认完成 |
| `0x23` | TASK1_RETURN_H | task1：巡航高度返航 H |
| `0x24` | TASK1_LAND_H | task1：在 H 点最终降落 |
| `0x25` | TASK1_COMPLETE | task1 完成 |
| `0x30` | TASK2_ALIGN_LAND | task2：对准车顶降落图案 |
| `0x31` | TASK2_DESCEND | task2：视觉精密下降 |
| `0x32` | TASK2_ON_CAR | task2：已稳定落在移动小车上 |
| `0x33` | TASK2_HOLD_5S | task2：车顶停留累计 5 s |
| `0x34` | TASK2_RETAKEOFF | task2：从小车平台再次起飞 |
| `0x35` | TASK2_RETURN_H | task2：巡航高度返航 H |
| `0x36` | TASK2_LAND_H | task2：在 H 点最终降落 |
| `0x37` | TASK2_COMPLETE | task2 完成 |
| `0xE0` | SAFE_HOLD | 输入暂时失效，安全定高等待 |
| `0xF0` | FAILSAFE | 飞控或关键链路保护 |
| `0xF1` | ERROR | 任务不可继续的错误 |
| `0xFF` | ABORTED | 任务已终止，不自动重启 |

task1 必须在小车到 B 前形成伴飞、D 前完成投放；task2 必须在 D 前完成车顶降落，停留 5 s 后再次起飞返航。上述赛题条件由树莓派结合小车新鲜状态判断。

## 3. 通用二进制帧

### 3.1 字节序和单位

- 多字节整数全部大端；
- 禁止在线缆中发送 `float/double`；
- 距离和位置：mm；
- 速度：mm/s；
- 角度和角速度：mrad、mrad/s；
- 时间：ms；
- 无符号字段的无效哨兵为该类型最大值，例如 `uint32 0xFFFFFFFF`；
- 有符号数只有在对应 valid 位为 1 时才可使用。

### 3.2 帧布局

`N` 为 payload 长度，范围 0～64 字节；一帧总长为 `22 + N` 字节。

| 偏移 | 长度 | 字段 | 说明 |
|---:|---:|---|---|
| 0 | 1 | header_1 | `0xD5` |
| 1 | 1 | header_2 | `0x7A` |
| 2 | 1 | protocol_version | 当前为 `0x01` |
| 3 | 1 | message_type | 见第 5 节 |
| 4 | 1 | source_id | 发送设备 |
| 5 | 1 | destination_id | 接收设备或广播 |
| 6 | 1 | sequence | 每条物理链路独立递增，自然回绕 |
| 7 | 1 | common_flags | 见下表 |
| 8 | 1 | payload_length | `N`，不得超过 64 |
| 9 | 1 | reserved | 必须为 0 |
| 10 | 4 | session_id | 一次小车起动对应一个非零任务会话 |
| 14 | 4 | sender_uptime_ms | 发送端本次开机运行时间，自然回绕 |
| 18 | N | payload | 由 message_type 决定 |
| 18+N | 2 | CRC16 | 大端，覆盖偏移 2～17+N |
| 20+N | 1 | tail_1 | `0x0D` |
| 21+N | 1 | tail_2 | `0x0A` |

`common_flags`：

| bit | 名称 | 含义 |
|---:|---|---|
| 0 | PAYLOAD_VALID | 该帧载荷格式和设备状态可解释 |
| 1 | SENDER_ERROR | 发送设备存在本地错误，详见载荷状态 |
| 2～7 | RESERVED | 必须为 0 |

视觉“没有发现目标”是一条合法状态帧，因此此时 PAYLOAD_VALID 仍为 1，目标是否有效由视觉载荷的 TARGET_VALID 表示。

### 3.3 CRC

使用 CRC16-CCITT-FALSE：

```text
poly   = 0x1021
init   = 0xFFFF
refin  = false
refout = false
xorout = 0x0000
```

### 3.4 设备编号

| ID | 设备 |
|---:|---|
| `0x01` | CAR，小车 |
| `0x02` | RPI，树莓派 |
| `0x03` | CAMERA，视觉摄像头 |
| `0x04` | GCS，地面站 |
| `0x05` | FLIGHT_CONTROLLER，保留；本协议不用于飞控 UART2 |
| `0xFF` | BROADCAST，广播 |

### 3.5 sequence 与重启

```text
delta = uint8(new_sequence - last_sequence)
delta == 0       重复帧
delta 1..127     合法新帧
delta 128..255   旧帧
```

- 重复帧和旧帧不能刷新输入新鲜度；
- session 改变，或链路连续超时 1 s 后，接收端可以重新建立 sequence 基准；
- CRC、版本、长度、设备 ID、reserved 或帧尾错误时丢弃，并从下一字节重新搜索帧头；
- 本协议不为连续遥测增加逐帧 ACK。START 等关键状态必须持续重复发送，通过 session 实现幂等，避免单次事件丢包。

## 4. 设备局部状态

### 4.1 通用 `device_state`

| 值 | 名称 |
|---:|---|
| `0x00` | BOOT |
| `0x01` | SELF_CHECK |
| `0x02` | READY |
| `0x03` | RUNNING |
| `0x04` | COMPLETE |
| `0xE0` | DEGRADED |
| `0xF0` | ERROR |

### 4.2 小车状态 `car_state`

| 值 | 名称 | 含义 |
|---:|---|---|
| `0x00` | CAR_BOOT | 启动 |
| `0x01` | CAR_READY_A | 位于 A，等待一键启动 |
| `0x02` | CAR_RUN_AB | A→B |
| `0x03` | CAR_RUN_BC | B→C |
| `0x04` | CAR_RUN_CD | C→D，默认投放窗口 |
| `0x05` | CAR_RUN_DA | D→A |
| `0x06` | CAR_COMPLETE_A | 完成一圈并在 A 停止 |
| `0xF0` | CAR_ERROR | 循线、里程或驱动错误 |

赛道分段 `segment_id`：0 UNKNOWN、1 AB、2 BC、3 CD、4 DA。检查点 `checkpoint_id`：0 UNKNOWN、1 A、2 B、3 C、4 D、5 H。

### 4.3 视觉状态 `vision_state`

| 值 | 名称 |
|---:|---|
| `0x00` | VISION_DISABLED |
| `0x01` | VISION_BOOT |
| `0x02` | VISION_READY |
| `0x03` | VISION_SEARCHING |
| `0x04` | VISION_ACQUIRED |
| `0x05` | VISION_TRACKING |
| `0x06` | VISION_DROP_ALIGNED |
| `0x07` | VISION_LAND_ALIGNED |
| `0x08` | VISION_TARGET_LOST |
| `0xF0` | VISION_ERROR |

视觉请求模式 `vision_mode`：0 DISABLED、1 SEARCH、2 FOLLOW、3 DROP_ALIGN、4 LAND_ALIGN。

### 4.4 无人机显示状态 `uav_state`

| 值 | 名称 |
|---:|---|
| `0x00` | UAV_STANDBY |
| `0x01` | UAV_ARMED_GROUND |
| `0x02` | UAV_TAKEOFF |
| `0x03` | UAV_HOVER |
| `0x04` | UAV_MAP_NAVIGATION |
| `0x05` | UAV_VISUAL_FOLLOW |
| `0x06` | UAV_VISUAL_DESCENT |
| `0x07` | UAV_LANDED_ON_CAR |
| `0x08` | UAV_RETAKEOFF |
| `0x09` | UAV_RETURNING_H |
| `0x0A` | UAV_LANDED_H |
| `0xF0` | UAV_FAILSAFE |
| `0xF1` | UAV_ERROR |

## 5. 消息类型与载荷

| message_type | 名称 | 方向 | payload长度 | 推荐频率 |
|---:|---|---|---:|---:|
| `0x01` | DEVICE_HEARTBEAT | 任意设备 → GCS/对端 | 8 | 1 Hz |
| `0x10` | CAR_TELEMETRY | CAR → RPI、CAR → GCS | 32 | 20 Hz / 10 Hz |
| `0x20` | VISION_COMMAND | RPI → CAMERA | 8 | 10 Hz |
| `0x21` | VISION_TARGET | CAMERA → RPI | 32 | 20～30 Hz |
| `0x30` | UAV_TELEMETRY | RPI → GCS | 36 | 10 Hz |

同一份 CAR_TELEMETRY 分别发给树莓派和地面站，不定义两套字段。

### 5.1 DEVICE_HEARTBEAT，8 字节

| payload偏移 | 长度 | 字段 | 类型 |
|---:|---:|---|---|
| 0 | 1 | device_state | `uint8` |
| 1 | 1 | last_mission_phase | `uint8`；不知道时为 `0x00` |
| 2 | 1 | local_error_code | `uint8`；0 表示无错误 |
| 3 | 1 | health_flags | bit0 ready，bit1 running，bit2 degraded，bit3 error |
| 4 | 1 | software_major | `uint8` |
| 5 | 1 | software_minor | `uint8` |
| 6 | 1 | software_patch | `uint8` |
| 7 | 1 | reserved | 必须 0 |

### 5.2 CAR_TELEMETRY，32 字节

| payload偏移 | 长度 | 字段 | 类型/单位 |
|---:|---:|---|---|
| 0 | 1 | task_mode | `uint8` |
| 1 | 1 | car_state | `uint8` |
| 2 | 1 | segment_id | 0～4 |
| 3 | 1 | last_checkpoint | A/B/C/D |
| 4 | 2 | car_flags | `uint16` |
| 6 | 2 | speed_mm_s | `uint16` |
| 8 | 4 | lap_distance_mm | 从本次 A 点启动累计的总路程 |
| 12 | 4 | distance_from_A_mm | 当前圈从 A 已走距离 |
| 16 | 4 | distance_from_B_mm | 尚未经过 B 时为 `0xFFFFFFFF` |
| 20 | 4 | distance_from_C_mm | 尚未经过 C 时为 `0xFFFFFFFF` |
| 24 | 4 | distance_from_D_mm | 尚未经过 D 时为 `0xFFFFFFFF` |
| 28 | 2 | line_error_mm | `int16`，黑线相对小车中心；左正 |
| 30 | 2 | battery_mv | `uint16` |

`car_flags`：

| bit | 名称 | 说明 |
|---:|---|---|
| 0 | START_LATCHED | 一键启动后持续为 1，直到整次任务复位 |
| 1 | MOVING | 小车正在运动 |
| 2 | LINE_VALID | 循线传感器有效 |
| 3 | DISTANCE_VALID | 距离数据可用 |
| 4 | CHECKPOINT_ESTIMATED | B/C/D 是按标定里程推算而非直接识别 |
| 5 | DROP_WINDOW | C-D 直道投放窗口 |
| 6 | LAP_COMPLETE | 已回到 A 完成一圈 |
| 7 | CAR_ERROR | 小车本地错误 |
| 8～15 | RESERVED | 必须 0 |

距离语义：经过某检查点的瞬间，对应 `distance_from_X_mm=0`；之后持续累加。尚未经过 B/C/D 使用 `0xFFFFFFFF`，禁止用 0 同时表示“未经过”和“刚经过”。任务完成后保持最终值，直到下一次 session。

session 规则：小车每次物理一键启动生成新的非零 `session_id`，同一圈内保持不变。START_LATCHED 和 task_mode 在该 session 内持续重复，不发送一次性脉冲。

### 5.3 VISION_COMMAND，8 字节

| payload偏移 | 长度 | 字段 | 类型/说明 |
|---:|---:|---|---|
| 0 | 1 | enable | 0 禁用，1 启用 |
| 1 | 1 | requested_vision_mode | SEARCH/FOLLOW/DROP_ALIGN/LAND_ALIGN |
| 2 | 1 | task_mode | 当前任务 |
| 3 | 1 | mission_phase | 树莓派权威阶段 |
| 4 | 1 | target_type | 0 NONE，1 CAR_PLATFORM，2 LANDING_PATTERN，3 DROP_CENTER |
| 5 | 1 | command_flags | bit0 reset_tracker；其余保留 |
| 6 | 2 | requested_rate_hz | `uint16`，建议 20 |

树莓派以 10 Hz 重发当前命令。摄像头不得因为重复命令反复初始化；只有 session、模式或 reset_tracker 发生变化时才重置追踪器。

### 5.4 VISION_TARGET，32 字节

| payload偏移 | 长度 | 字段 | 类型/单位 |
|---:|---:|---|---|
| 0 | 1 | vision_state | `uint8` |
| 1 | 1 | vision_mode_echo | 当前实际模式 |
| 2 | 1 | task_mode_echo | 摄像头收到的任务 |
| 3 | 1 | mission_phase_echo | 摄像头收到的树莓派阶段 |
| 4 | 1 | target_type | 当前目标类型 |
| 5 | 1 | reserved | 必须 0 |
| 6 | 2 | vision_flags | `uint16` |
| 8 | 4 | relative_x_mm | `int32`，目标在机体前方为正 |
| 12 | 4 | relative_y_mm | `int32`，目标在机体左方为正 |
| 16 | 4 | relative_z_mm | `int32`，目标在机体上方为正 |
| 20 | 2 | relative_yaw_mrad | `int16`，俯视逆时针为正 |
| 22 | 2 | confidence_permille | 0～1000 |
| 24 | 2 | sample_age_ms | 从成像到发送的样本年龄 |
| 26 | 2 | processing_time_ms | 本帧处理耗时 |
| 28 | 4 | frame_counter | 相机图像帧计数 |

`vision_flags`：

| bit | 名称 | 说明 |
|---:|---|---|
| 0 | TARGET_VALID | 检出目标且通过基本质量检查 |
| 1 | XY_VALID | relative_x/y 可用于水平控制 |
| 2 | Z_VALID | relative_z 可用；不可用时必须为 0 |
| 3 | YAW_VALID | relative_yaw 可用；当前控制可选择忽略 |
| 4 | DROP_ALIGNED | 满足摄像头侧投放对准阈值 |
| 5 | LAND_ALIGNED | 满足摄像头侧下降对准阈值 |
| 6 | IMAGE_FRESH | 图像和算法没有卡住 |
| 7 | VISION_ERROR | 摄像头本地错误 |
| 8～15 | RESERVED | 必须 0 |

相对位移必须由摄像头端转换到无人机 BODY 坐标，不得直接发送图像像素轴。定义的是“从无人机参考点指向目标中心”的向量：x 前、y 左、z 上。目标在无人机下方时 z 为负。树莓派当前可只使用 x/y，yaw 控制保持 0；z 和 yaw 仍保留供显示与后续验证。

树莓派只接受：session 一致、echo 模式一致、TARGET_VALID、XY_VALID、IMAGE_FRESH、sample_age_ms 合法且串口帧新鲜的位移。任何一项失败都不能继续使用上一帧位移。

### 5.5 UAV_TELEMETRY，36 字节

| payload偏移 | 长度 | 字段 | 类型/单位 |
|---:|---:|---|---|
| 0 | 1 | task_mode | 当前锁定任务 |
| 1 | 1 | mission_phase | 树莓派权威全局阶段 |
| 2 | 1 | uav_state | 地面站显示用无人机状态 |
| 3 | 1 | vision_state | 最近摄像头状态 |
| 4 | 1 | car_state | 最近小车状态 |
| 5 | 1 | system_error_code | 树莓派汇总主错误；0 为正常 |
| 6 | 2 | uav_flags | `uint16` |
| 8 | 4 | map_x_mm | `int32` |
| 12 | 4 | map_y_mm | `int32` |
| 16 | 4 | map_z_mm | `int32` |
| 20 | 2 | yaw_mrad | `int16` |
| 22 | 2 | velocity_x_mm_s | `int16`，MAP x |
| 24 | 2 | velocity_y_mm_s | `int16`，MAP y |
| 26 | 2 | velocity_z_mm_s | `int16`，MAP z |
| 28 | 2 | camera_age_ms | 最近合法视觉帧年龄 |
| 30 | 2 | car_age_ms | 最近合法小车帧年龄 |
| 32 | 2 | flight_status_age_ms | 最近飞控状态年龄 |
| 34 | 2 | mission_elapsed_ds | 从 START 起经过的 0.1 s 数 |

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
| 13～15 | RESERVED |

## 6. 地图和赛道坐标

### 6.1 无人机 MAP 坐标

- 测评前手动放置的 H 点中心定义为 `(0,0,0)`；
- x 轴为无人机初始机头方向，同时应与小车 A 点初始朝向一致；
- y 轴向左；
- z 轴向上；
- yaw 俯视逆时针为正；
- UAV_TELEMETRY 使用 mm/mrad，ROS 2 内部仍使用 m/rad。

若 Cartographer 原生 map 原点不同，树莓派在任务开始时锁存 H 点位姿并变换后再发给地面站。地面站不得直接假设 Cartographer 的原始原点就是 H。

### 6.2 小车图形化位置

地面站保存赛道 A/B/C/D 的标定几何和各段长度，使用 `segment_id + distance_from_最近检查点` 在对应轨迹段插值显示小车位置。

- 小车无需同时计算一套可能与地面站不同的二维坐标；
- 若 B/C/D 没有硬件标记，允许按编码器累计里程推算，同时置 CHECKPOINT_ESTIMATED；
- 图形显示应标出“测量”或“估算”，避免把估算里程显示成绝对真值。

## 7. 推荐物理参数和超时

| 链路 | 物理层 | 推荐频率 | 接收超时 | 失效处理 |
|---|---|---:|---:|---|
| CAMERA ↔ RPI | 独立 3.3 V TTL/USB-TTL，115200，8N1 | 视觉20～30 Hz，命令10 Hz | 200 ms | 立即撤销视觉位移，不沿用旧值 |
| CAR → RPI | 无线串口/数传，硬件待定 | 20 Hz | 300 ms | 禁止依据旧 DROP_WINDOW 推进任务 |
| CAR → GCS | 无线串口/数传，硬件待定 | 10 Hz | 500 ms | 显示小车离线和最后更新时间 |
| RPI → GCS | 无线串口、UDP或数传，硬件待定 | 10 Hz | 500 ms | 显示无人机链路离线；不影响飞控安全 |
| 心跳 | 同各自链路 | 1 Hz | 3 s | 显示设备离线 |

各物理链路必须使用不同串口或明确的无线通道，不能把 CAMERA、飞控 V2 和雷达数据接入同一字节流。

## 8. 关键状态推进规则

### 8.1 START

1. 小车在 A 点 READY 时周期发送 task_mode，START_LATCHED=0；
2. 物理按钮按下后，小车生成新 session，START_LATCHED=1 并持续发送；
3. 树莓派首次接受该 session 后锁存任务，只执行一次起飞；
4. 同 session 的重复 START 只用于抗丢包，不重复起飞；
5. 飞行中出现新 session 只记录错误，不切换任务。

### 8.2 task1

```text
START → TAKEOFF → HOVER_3S → RIGHT_SHIFT → FORWARD_CHASE
      → SEARCH/FOLLOW（B前形成伴飞）
      → C-D DROP_WINDOW + 视觉连续对准 → DROP一次
      → RETURN_H → LAND_H → COMPLETE
```

DROP_WINDOW 只是允许投放的必要条件之一。树莓派还必须要求视觉帧新鲜、XY 有效、视觉控制已接管且对准条件连续满足。投放机构确认后才能进入 DROP_DONE。

### 8.3 task2

```text
START → TAKEOFF → RIGHT_SHIFT → FORWARD_CHASE → SEARCH/FOLLOW
      → ALIGN_LAND → DESCEND → ON_CAR（D前）
      → HOLD_5S → RETAKEOFF → RETURN_H → LAND_H → COMPLETE
```

task2 不执行 task1 的 3 s 悬停。视觉丢失时不得继续使用旧相对位移下降；应停止外部下降并由飞控保持安全状态。

## 9. 地面站最低显示内容

地面站至少显示：

- 场地图、H 点、A/B/C/D 和赛道；
- 无人机 `(x,y,z,yaw)`、轨迹和 POSE_VALID；
- 小车所在 AB/BC/CD/DA 段、距最近检查点距离、整圈里程和速度；
- task1/task2、全局 mission_phase、uav_state、car_state、vision_state；
- START 后计时；
- 飞控、摄像头、小车、定位链路是否新鲜；
- 抛投完成、车顶着陆、最终 H 点降落和 failsafe/error。

正式比赛界面不得提供会改变任务的手动按钮。调试按钮必须有独立的“测试模式”开关，并在比赛构建中禁用。

## 10. 实现前必须共同确认

以下四项不会改变上层状态字段，但会影响最终 v1.0：

1. CAMERA ↔ RPI 是否确认 115200、8N1，以及最终设备名；
2. CAR → RPI、CAR → GCS、RPI → GCS 使用哪种无线模块，是否都是透明串口；
3. 小车能否直接识别 B/C/D，还是只能用编码器按标定里程推算；
4. 摄像头能否提供 z 和相对 yaw；若不能，只需清除 Z_VALID/YAW_VALID，不修改帧布局。

四端确认后应完成：

- 冻结协议为 v1.0；
- 为每种消息生成至少一条固定字节向量；
- 四端使用同一 CRC 和枚举头文件；
- 做坏 CRC、丢帧、重复帧、sequence 回绕、设备重启和 session 重复测试；
- 最后再接入树莓派 ROS 2 话题和总启动包。
