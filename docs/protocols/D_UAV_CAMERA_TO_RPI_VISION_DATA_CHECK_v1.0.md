# 摄像头端向树莓派发送视觉数据核对说明 v1.0

日期：2026-08-02  
接收方：摄像头端 AI  
对应正式协议：`D_UAV_UNIFIED_DEVICE_PROTOCOL_v1.7.md`  
适用链路：摄像头 CAMERA ↔ 树莓派 RPI  

## 1. 本次问题与结论

最近一次实飞中，无人机已经正常完成起飞、右移，并进入：

```text
FORWARD_CHASE(0x05)
```

树莓派任务日志在整个前进阶段持续显示：

```text
视觉=false
/vision/link_ok=false
/vision/target_valid=false
```

无人机没有进入 `FOLLOW_CAR(0x07)`。

树莓派摄像头串口节点已经启动并成功打开：

```text
设备：/dev/ttyAMA0
串口：115200、8N1、无流控
```

因此当前可以确认：树莓派没有在200 ms内收到任何格式正确、可解析的
`VISION_DATA(0x21)`帧。

这和“摄像头暂时没有识别到目标”不是同一件事。即使没有找到目标，摄像头也必须周期发送`SEARCHING + IMAGE_FRESH`状态帧；只要该帧格式正确，树莓派的`/vision/link_ok`就应为`true`。

请摄像头端 AI 按本文逐项核对接收、组帧、发送和接线。

## 2. 物理串口要求

| 项目 | 要求 |
|---|---|
| 电平 | 3.3 V TTL；不得把5 V串口信号直接接入树莓派GPIO |
| 波特率 | 115200 |
| 数据格式 | 8位数据位、1位停止位、无校验、无流控 |
| 树莓派发送 | GPIO14 / TXD，物理引脚8 → 摄像头RX |
| 树莓派接收 | GPIO15 / RXD，物理引脚10 ← 摄像头TX |
| 地线 | 树莓派GND与摄像头GND必须共地 |
| 树莓派设备 | `/dev/ttyAMA0` |

摄像头和树莓派必须TX/RX交叉连接。摄像头向树莓派发送数据时，真正需要检查的是：

```text
摄像头TX → 树莓派GPIO15/RX（物理引脚10）
摄像头GND ↔ 树莓派GND
```

## 3. 通用二进制帧格式

摄像头必须发送原始二进制字节，不能发送以下文本：

```text
DA 7E 01 21 ...
```

上面只是人类查看时的十六进制写法。串口线上必须实际发送`0xDA、0x7E、0x01、0x21...`，不能发送字符`D`、`A`、空格和换行。

统一帧格式：

```text
DA 7E | version | message_type | sequence | payload_length |
session_id_hi session_id_lo | payload | checksum8 | 0D 0A
```

| 总偏移 | 长度 | 字段 | 摄像头回传要求 |
|---:|---:|---|---|
| 0～1 | 2 | 帧头 | 固定`DA 7E` |
| 2 | 1 | version | 固定`01`；文档v1.7不代表此处填`17` |
| 3 | 1 | message_type | 摄像头回传固定`21` |
| 4 | 1 | sequence | 每帧递增，`FF→00`；重复不致命，但应正常递增 |
| 5 | 1 | payload_length | 固定`0C`，即12字节 |
| 6～7 | 2 | session_id | 大端；任务一必须`00 01` |
| 8～19 | 12 | payload | 见第4节 |
| 20 | 1 | checksum8 | 从偏移2累加到偏移19，取低8位 |
| 21～22 | 2 | 帧尾 | 固定`0D 0A` |

每个`VISION_DATA`总长度必须恰好为23字节。

## 4. VISION_DATA(0x21)负载内容

| payload偏移 | 整帧偏移 | 长度 | 字段 | 类型与含义 |
|---:|---:|---:|---|---|
| 0 | 8 | 1 | `mission_phase_echo` | 回显最近收到并正在执行的树莓派阶段 |
| 1 | 9 | 1 | `vision_state` | 搜索、跟踪、丢失、对准或投放状态 |
| 2 | 10 | 1 | `target_type` | 当前处理的目标类型 |
| 3 | 11 | 1 | `vision_flags` | 当前帧有效标志 |
| 4～7 | 12～15 | 4 | `relative_x_mm` | `int32`大端；目标在无人机前方为正 |
| 8～11 | 16～19 | 4 | `relative_y_mm` | `int32`大端；目标在无人机左方为正 |

摄像头只发送平面x/y相对位移，单位必须为毫米。不得发送：

- 像素值；
- 速度；
- z或高度；
- yaw或角速度；
- C语言未打包结构体的原始内存。

有符号坐标使用32位二进制补码、大端顺序。例如：

```text
+500 mm  → 00 00 01 F4
-100 mm  → FF FF FF 9C
```

## 5. 前进搜索阶段必须执行的流程

无人机进入前进阶段后，树莓派以10 Hz发送`VISION_COMMAND(0x20)`：

```text
session_id      = 1
enable          = 1
vision_mode     = SEARCH(0x01)
task_mode       = TASK1(0x01)
mission_phase   = FORWARD_CHASE(0x05)
target_type     = CAR_PLATFORM(0x01)
command_flags   = 0；阶段刚切换时可能脉冲RESET_TRACKER=1
requested_rate  = 20 Hz
```

一条`sequence=0`、无RESET的标准命令示例为：

```text
DA 7E 01 20 00 08 00 01
01 01 01 05 01 00 00 14
47 0D 0A
```

摄像头收到后必须：

1. 确认帧头、版本、类型、长度、checksum和帧尾；
2. 保存`session_id=1`；
3. 进入SEARCH模式，识别`CAR_PLATFORM`；
4. 无论是否找到目标，都以20～30 Hz持续发送23字节`VISION_DATA`；
5. 不等待树莓派ACK，因为本协议没有逐帧ACK。

## 6. 三种必须正确发送的搜索结果

### 6.1 图像正常，但尚未找到小车

必须发送：

```text
mission_phase_echo = FORWARD_CHASE(0x05)
vision_state       = SEARCHING(0x02)
target_type        = CAR_PLATFORM(0x01)
vision_flags       = IMAGE_FRESH(0x10)
relative_x_mm      = 0
relative_y_mm      = 0
```

`session=1、sequence=0`完整固定帧：

```text
DA 7E 01 21 00 0C 00 01
05 02 01 10 00 00 00 00 00 00 00 00
47 0D 0A
```

树莓派预期：

```text
/vision/link_ok=true
/vision/target_valid=false
任务保持FORWARD_CHASE并继续向前搜索
```

### 6.2 已识别小车并得到有效平面偏移

必须发送：

```text
mission_phase_echo = FORWARD_CHASE(0x05)
vision_state       = TRACKING(0x03)
target_type        = CAR_PLATFORM(0x01)
vision_flags       = TARGET_VALID | XY_VALID | IMAGE_FRESH = 0x13
relative_x_mm      = 有效BODY x毫米值
relative_y_mm      = 有效BODY y毫米值
```

示例：目标在无人机前方500 mm、右方100 mm。BODY坐标y向左为正，因此右方100 mm应发送`y=-100 mm`。

`session=1、sequence=1`完整示例帧：

```text
DA 7E 01 21 01 0C 00 01
05 03 01 13 00 00 01 F4 FF FF FF 9C
DA 0D 0A
```

树莓派预期：

```text
/vision/link_ok=true
/vision/target_valid=true
/vision/relative_error.x=+0.500 m
/vision/relative_error.y=-0.100 m
```

目标连续有效后，树莓派将任务切换为`FOLLOW_CAR(0x07)`。

### 6.3 原来识别到目标，后来目标丢失

必须立即清除旧坐标并发送：

```text
mission_phase_echo = 当前树莓派阶段
vision_state       = TARGET_LOST(0x06)
target_type        = 当前要求的目标类型
vision_flags       = IMAGE_FRESH(0x10)，前提是图像仍在刷新
relative_x_mm      = 0
relative_y_mm      = 0
```

`FORWARD_CHASE、session=1、sequence=2`示例：

```text
DA 7E 01 21 02 0C 00 01
05 06 01 10 00 00 00 00 00 00 00 00
4D 0D 0A
```

不得继续重复上一次有效x/y。

## 7. 进入伴飞后的变化

树莓派收到连续有效目标后会进入`FOLLOW_CAR(0x07)`，并改发：

```text
enable        = 1
vision_mode   = FOLLOW(0x02)
task_mode     = TASK1(0x01)
mission_phase = FOLLOW_CAR(0x07)
target_type   = CAR_PLATFORM(0x01)
```

摄像头随后回传必须同步改为：

```text
mission_phase_echo = FOLLOW_CAR(0x07)
vision_state       = TRACKING(0x03)
target_type        = CAR_PLATFORM(0x01)
vision_flags       = 0x13
relative_x/y_mm    = 当前有效平面偏移
session_id         = 1
```

如果摄像头继续回显`0x05`，树莓派会把该帧判定为阶段不匹配，链路仍可能显示在线，但不会使用x/y控制飞机。

## 8. 树莓派把一帧认定为“可用于视觉控制”的完整条件

树莓派当前代码要求同一帧同时满足：

```text
帧头              = DA 7E
version           = 0x01
message_type      = 0x21
payload_length    = 0x0C
session_id        = 当前task；任务一必须1
checksum8         = 正确
帧尾              = 0D 0A
mission_phase_echo= 当前树莓派阶段
target_type       = 当前树莓派要求的目标
flags bit0        = TARGET_VALID=1
flags bit1        = XY_VALID=1
flags bit4        = IMAGE_FRESH=1
flags bit5        = VISION_ERROR=0
flags bit6～7     = 0
最近接收时间      ≤ 200 ms
```

以下任一错误都会使`/vision/target_valid=false`：

- phase仍回显旧阶段；
- target_type错误；
- 忘记设置0x13中的某一位；
- 设置了VISION_ERROR；
- 发送的是像素而不是毫米；
- 数据超过200 ms；
- session不是1。

以下错误会导致整帧根本无法解析，使`/vision/link_ok=false`：

- 发ASCII十六进制文本而不是二进制；
- 帧头、version或帧尾错误；
- message_type不是0x21；
- payload_length不是12；
- 总长度不是23字节；
- checksum覆盖范围或算法错误；
- 摄像头TX没有接到GPIO15/RX；
- 波特率或串口格式不一致；
- 摄像头没有周期发送任何状态帧。

## 9. checksum8算法

checksum不包含帧头`DA 7E`，也不包含checksum自身和帧尾`0D 0A`。

伪代码：

```c
uint8_t checksum = 0;
for (int i = 2; i <= 19; ++i) {
    checksum = (uint8_t)(checksum + frame[i]);
}
frame[20] = checksum;
```

注意事项：

- 每一帧都要把sequence和当前payload一起重新计算；
- 不能使用CRC16；
- 不能包含`DA 7E`；
- 不能把最终和取反；
- 只保留累加结果低8位。

## 10. 建议摄像头端立即提供的核对证据

请摄像头端 AI 回复以下内容：

1. 实际使用的串口号、波特率、TX/RX引脚和电平；
2. 是否已经成功解析树莓派发来的`VISION_COMMAND(0x20)`；
3. 进入前进阶段时解析到的session、enable、mode、task、phase和target；
4. 摄像头实际发送的是二进制字节还是ASCII十六进制字符串；
5. 未识别目标时实际发送的一条完整23字节HEX帧；
6. 识别目标时实际发送的一条完整23字节HEX帧；
7. VISION_DATA实际发送频率；
8. `relative_x_mm/relative_y_mm`是否已经从像素换算成毫米并转为BODY坐标；
9. checksum8计算代码；
10. 发送函数的实际调用位置，确认它在未识别目标时也会周期运行。

## 11. 本轮最短联调方法

本轮不需要起飞，也不需要启动飞控电机：

1. 树莓派和摄像头正常连接并共地；
2. 树莓派模拟进入task1的`FORWARD_CHASE`；
3. 摄像头先不放目标，确认树莓派得到：
   `/vision/link_ok=true、target_valid=false`；
4. 把小车平台图案放入视野，确认树莓派得到：
   `/vision/link_ok=true、target_valid=true`及变化的x/y；
5. 移走图案，确认target立即变false且x/y归零；
6. 连续通过后，再做真实前追和伴飞测试。

## 12. 当前树莓派端判断

目前树莓派端没有证据表明收到过合法的摄像头`VISION_DATA`：

```text
摄像头串口设备已打开：是
任务进入FORWARD_CHASE：是
树莓派向摄像头生成SEARCH命令：是
收到合法VISION_DATA：否
/vision/link_ok：始终false
收到有效目标x/y：否
进入FOLLOW_CAR：否
```

因此摄像头端当前首先应检查“串口回传与23字节组帧”，而不是先调整识别阈值或视觉PID。
