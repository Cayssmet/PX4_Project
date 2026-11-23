# 树莓派—凌霄飞控串口协议 V2.0（revision A）

日期：2026-08-02  
状态：30字节布局保持冻结；HR位仅兼容/诊断，树莓派任务推进不再使用  
线上版本：`0x02`

V1 仍作为独立基线保留。V1 与 V2 只能由配置显式选择，不得在飞行中自动猜测、切换或回退。飞控已经刷入V2，树莓派生产参数现固定为 `protocol_version: 2`；以后若切回V1，双方必须同时显式修改并重新构建烧录。

## 1. 物理层与通用规则

- 串口：115200、8N1、无流控，50 Hz 命令；飞控命令看门狗 200 ms。
- 电气：3.3 V TTL，TX/RX 交叉并共地，禁止直接接 RS-232 电平。
- 多字节整数：大端。
- CRC：CRC16-CCITT-FALSE，`poly=0x1021`、`init=0xFFFF`、不反射、`xorout=0`。
- CRC 覆盖偏移 2～25，共 24 字节；CRC 位于 26～27，帧尾位于 28～29。
- V2 命令帧头 `A5 5A`，状态帧头 `5A A5`；两种帧固定 30 字节。
- 机体系：x 向前、y 向左、z 向上、yaw 俯视逆时针为正。
- 速度单位为 mm/s，yaw 为 mrad/s，高度为 mm；ROS 2 内部仍使用 m、rad、s。

解析器必须先验证帧头、固定长度、版本、消息类型、保留位、flags、CRC 和帧尾，再更新控制、ACK、session、动作缓存或看门狗。

## 2. 命令帧：树莓派到飞控

| 偏移 | 长度 | 字段 | 类型/说明 |
|---:|---:|---|---|
| 0 | 1 | 帧头1 | `0xA5` |
| 1 | 1 | 帧头2 | `0x5A` |
| 2 | 1 | protocol_version | `0x02` |
| 3 | 1 | message_type | `0x01` command |
| 4 | 1 | transport_sequence | `uint8`，自然回绕 |
| 5 | 1 | control_mode | 见下表 |
| 6 | 1 | command_flags | bit0=`control_valid`，其余必须0 |
| 7 | 1 | reserved | 必须0 |
| 8 | 4 | session_id | `uint32` |
| 12 | 4 | action_id | `uint32` |
| 16 | 2 | velocity_x_mm_s | `int16`，BODY前向 |
| 18 | 2 | velocity_y_mm_s | `int16`，BODY左向 |
| 20 | 2 | velocity_z_mm_s | `int16`，向上为正 |
| 22 | 2 | yaw_rate_mrad_s | `int16`，逆时针为正 |
| 24 | 2 | target_height_mm | `uint16` |
| 26 | 2 | CRC16 | 覆盖2～25，大端 |
| 28 | 1 | 帧尾1 | `0x0D` |
| 29 | 1 | 帧尾2 | `0x0A` |

模式与字段约束：

| 值 | 模式 | vx/vy/yaw | vz | target_height | action_id |
|---:|---|---|---|---|---:|
| 0 | STANDBY | 置0 | 置0 | 置0 | 0 |
| 1 | TAKEOFF | 必须0 | 必须0 | 300～1700 mm | >0 |
| 2 | HEIGHT_HOLD | 有效 | 必须0 | 100～1700 mm | 0 |
| 3 | PRECISION_DESCENT | 有效 | 有符号垂直速度 | 100～1700 mm下降安全下限 | 0 |
| 4 | FINAL_LAND | 必须0 | 必须0 | 置0 | >0 |
| 5 | RESET_MISSION | 必须0 | 必须0 | 置0 | >0 |

容器范围不是飞行安全范围。树莓派发送前限幅，飞控执行前仍必须独立限幅。

## 3. 状态帧：飞控到树莓派

| 偏移 | 长度 | 字段 | 类型/说明 |
|---:|---:|---|---|
| 0 | 1 | 帧头1 | `0x5A` |
| 1 | 1 | 帧头2 | `0xA5` |
| 2 | 1 | protocol_version | `0x02` |
| 3 | 1 | message_type | `0x02` status |
| 4 | 1 | acknowledged_sequence | `uint8` |
| 5 | 1 | current_mode | 0～5 |
| 6 | 1 | status_flags | 见下表 |
| 7 | 1 | error_code | 单一主错误码 |
| 8 | 4 | active_session_id | `uint32` |
| 12 | 4 | active_action_id | `uint32` |
| 16 | 2 | current_height_mm | `uint16` |
| 18 | 2 | vertical_velocity_mm_s | `int16` |
| 20 | 2 | height_age_ms | `uint16` |
| 22 | 2 | command_age_ms | `uint16` |
| 24 | 1 | action_result | 见下表 |
| 25 | 1 | reserved | 必须0 |
| 26 | 2 | CRC16 | 覆盖2～25，大端 |
| 28 | 1 | 帧尾1 | `0x0D` |
| 29 | 1 | 帧尾2 | `0x0A` |

`status_flags`：bit0 height_valid、bit1 height_reached、bit2 armed、bit3 airborne、bit4 landed、bit5 action_accepted、bit6 action_completed、bit7 failsafe。

自2026-08-02起，bit1仍保留在冻结布局中，但只作兼容/诊断。树莓派不再用
`height_reached/HR`启动悬停计时、推进RIGHT_SHIFT、判断航点完成、允许投放或判断复飞完成；
业务动作完成统一使用同session的`active_action_id + action_result=SUCCEEDED`。

当前飞控状态语义：`airborne` 需要主飞控已解锁、有效高度大于300 mm等条件共同成立；`landed` 是高度、垂直速度与动作阶段组合出的保守候选，需要连续满足约2 s，或由主飞控上锁辅助确认。无桨手举到目标高度时，TAKEOFF 可以到高成功，但仍可能看到 `armed=0、airborne=0、landed=0`，不得据此误判起飞动作失败。

`action_result`：0 NONE、1 PENDING、2 SUCCEEDED、3 REJECTED、4 FAILED。对应关系为：NONE=(0,0)，PENDING=(1,0)，SUCCEEDED=(1,1)，REJECTED=(0,0)，FAILED=(1,0)，括号内依次为 accepted/completed。

高度无效时必须发送：`current_height_mm=0`、`height_valid=0`、`height_age_ms=65535`。接收方必须先判断 height_valid，不得把数值0解释为着地。

### 3.1 error_code 冻结映射

| 值 | 名称 | 值 | 名称 |
|---:|---|---:|---|
| `0x00` | OK | `0x09` | INVALID_SESSION |
| `0x01` | RC_FAILSAFE | `0x0A` | DUPLICATE_ACTION |
| `0x02` | COMMAND_TIMEOUT | `0x0B` | ACTION_NOT_ALLOWED |
| `0x03` | HEIGHT_TIMEOUT | `0x0C` | TAKEOFF_TIMEOUT（保留，当前不主动产生） |
| `0x04` | HEIGHT_OUT_OF_RANGE | `0x0D` | LAND_TIMEOUT |
| `0x05` | FRAME_ERROR（预留） | `0x0E` | RESET_NOT_ALLOWED |
| `0x06` | SEQUENCE_ERROR（预留） | `0x0F` | INTERNAL_ERROR（预留） |
| `0x07` | NOT_AUTHORIZED | `0x10` | INVALID_FIELD |
| `0x08` | INVALID_MODE | `0x11` | OLD_ACTION |

`REJECTED` 不自动置 failsafe。遥控失联、命令超时以及当前控制所需的高度失效会置 failsafe；多错误并存时遥控失联和高度保护优先上报。

### 3.2 首轮双方安全上限

树莓派和飞控均按水平合速度 1500 mm/s、垂直速度绝对值 500 mm/s、yaw rate 绝对值 2000 mrad/s进行独立检查。生产目标高度上限统一为1700 mm：TAKEOFF 为300～1700 mm，HEIGHT_HOLD/PRECISION_DESCENT 为100～1700 mm，STANDBY/FINAL_LAND/RESET_MISSION 必须为0。

## 4. sequence、ACK、session 与 action

传输序号关系：

```text
delta = uint8(new_sequence - last_sequence)
delta == 0       重复帧
delta 1..127     合法新帧
delta 128..255   旧帧
```

只有合法新帧可以更新控制量、命令看门狗、ACK 和离散动作判断。重复帧与旧帧不能刷新 200 ms 看门狗。树莓派以 `lag=uint8(last_sent_sequence-acknowledged_sequence)` 判断 ACK，默认仅接受 `lag<=10`。

新 session 使用非零随机/递增 `session_id`，先发送无物理动作的 STANDBY：mode=0、action=0、valid=1、所有数值为0。飞控仅在待机未飞行，或可靠着地且上一 session 已结束时接受并回显该 session；树莓派确认回显后才发送 TAKEOFF。飞行中不同 session 必须返回 INVALID_SESSION，不能切换或刷新当前动作。

离散动作唯一键为 `session_id + action_id + control_mode`，摘要还包括目标高度等关键参数。同 session 中新离散 action 必须严格大于最后已处理 action，可跳号；正式 REJECTED 也算已处理。

- 完全相同的离散动作重发：不得重复执行，返回缓存结果，不报 DUPLICATE_ACTION。
- 相同 action_id 却改变模式或关键参数：不执行、不覆盖缓存，返回 DUPLICATE_ACTION。
- action 结果保持到下一合法离散动作、新 session 或安全复位；连续模式切换不清除上一动作结果。

## 5. 通信中断和坏帧

CRC、帧尾、版本或类型错误：丢弃，不更新控制、看门狗、ACK、session 和动作缓存。单个坏帧不立刻打断仍在 200 ms 有效期内的上一连续命令；持续坏帧最终由看门狗保护。

- HEIGHT_HOLD 断联：水平/yaw 立即归零，高度有效时保持最后安全目标。
- PRECISION_DESCENT 断联或 valid=0：水平/yaw/vz 归零，高度有效时停止下降并保持当前安全高度；高度无效则进入高度保护。
- TAKEOFF 接受后只执行一次，到高后转定高；断联不得产生水平运动。自2026-08-02起，通用到高条件为目标高度 ±100 mm 连续 100 ms（5个20 ms周期）。当前飞控不使用固定20 s超时主动把动作判失败；高度暂时无效时保持 PENDING、不给起飞油门，恢复有效后继续判断。
- V2只提供同session的`M=2/A=1/2`动作成功反馈，不强制任务层等待时长；实时HR不参与推进。官方task1要求稳定悬停3 s，当前树莓派现场策略参数为收到成功后固定等待2.0 s直接右移，正式按该得分项验收时应恢复3.0 s；task2收到成功后不额外等待。
- FINAL_LAND 一旦 accepted，脱离树莓派重发依赖，继续本地安全降落直至成功或失败。
- RESET 只复位任务状态，不解锁、不起飞、不产生运动。
- 遥控失联和主飞控自身保护始终拥有更高优先级。
- D题任务模式与 START 由小车—树莓派链路管理，不进入 V2 UART 帧。task1 投放期间使用 HEIGHT_HOLD；当前现场标定的起飞后右移量为0.375 m，对应 BODY `vy<0`。

## 6. 固定测试向量

向量1，命令：seq=FE、PRECISION_DESCENT、valid=1、session=01020304、action=0、vx=+1000、vy=-500、vz=-120 mm/s、yaw=+50 mrad/s、安全下限=300 mm：

```text
CRC：D0 7B
A5 5A 02 01 FE 03 01 00 01 02 03 04 00 00 00 00 03 E8 FE 0C FF 88 00 32 01 2C D0 7B 0D 0A
```

向量2，状态：ack=7B、HEIGHT_HOLD、flags=6F、session=11223344、action=7、高度1500 mm、vz=-123 mm/s、height_age=20 ms、command_age=40 ms、SUCCEEDED：

```text
CRC：08 A6
5A A5 02 02 7B 02 6F 00 11 22 33 44 00 00 00 07 05 DC FF 85 00 14 00 28 02 00 08 A6 0D 0A
```

## 7. 首轮无桨联合验收

按顺序验收：STANDBY建立/回显session；sequence跨255到0；坏CRC与200 ms看门狗；TAKEOFF首次接受、同动作幂等、冲突摘要拒绝、结果保持；HEIGHT_HOLD action=0；PRECISION_DESCENT负速度/安全下限/valid=0停止；FINAL_LAND断联继续；RESET不自动起飞；第二次TAKEOFF使用更大action；飞行中拒绝新session、着地后接受；V1/V2错配时无控制输出。
