# 凌霄飞控旧代码审计与当前接入结果

## 1. 本次只读参考的工程

当前分析的工程位于：

```text
D:\Fly world\飞控资料\凌霄\5.飞控MCU源码工程\
ANO_LX_FC-2021-7-18 121043\ANO_LX_FC
```

统一项目整理时没有修改 D 盘原目录，并在项目的 `flight_controller/` 下保存了一份完整副本。
项目已在用户确认后改造飞控副本；D盘原目录保持不变，修改前备份位于
`flight_controller/backups/pre_rpi_bridge_2026-07-26/`。所有修改均使用下列注释包围：

```c
/* [树莓派新架构修改-开始] 修改目的：…… */
/* 新代码 */
/* [树莓派新架构修改-结束] */
```

## 2. 已确认可复用的部分

- 底板串口 2 对应 TM4C123 硬件 UART4，已经按 115200 8N1 初始化；
- `Ano_Scheduler.c` 的用户任务以 50 Hz（20 ms）运行；
- `ANO_LX.c` 已确认 `TMD_Val` 的方向与单位：x 前、y 左、z 上，单位 cm/s，yaw 逆时针为正，单位 deg/s；
- 旧 `Ano_ExtVelocity.c` 已有按字节重同步、CRC 检查、序号和 200 ms 超时清零思路；
- `OneKey_Land()` 可以作为独立降落入口；
- `HIGH` 是飞控侧收到的当前高度反馈。

## 3. 与新树莓派架构不兼容的部分

旧命令帧仍是 `vx、vy、vz、yaw`，而新方案必须是：

```text
vx、vy、yaw_rate、target_height、control_valid、land_request
```

因此不能把当前 `Ano_ExtVelocity.c` 原样用于新包。还存在以下差异：

- 旧帧是 15 字节 CRC8，新帧是 16 字节 CRC16；
- 旧 z 字段直接代表 z 速度，新 z 字段代表数值目标高度；
- 旧 `User_Task.c` 把 z=0/1/2 映射为 80/100/125 cm，新方案不再使用高度等级；
- 旧降落从另一个 UART3 的 `FD 01 0D` 接收，新方案改为同一 UART2 命令 flags；
- 旧代码没有向树莓派返回高度到达、降落接受/完成和 failsafe 状态帧；
- 旧代码只有收到过外部“位置”才开放速度接管，但新架构的飞控不接收树莓派地图位置。

## 4. 高度 PID 不能直接照搬的原因

`ProjectTM4C123/Src/control.c` 中现有 `keep_high()` 可以作为原理参考，但在上桨前必须修正并台架验证：

1. `PID_Calculate()` 先把 `feedback_old/expect_old` 写成当前值，再计算差分，导致 D 项恒为 0；
2. `centre_ok` 只在误差大于 5 cm 时清零，持续位于容差内时会溢出；
3. 注释写“40 次、0.8 秒”，代码实际判断 10 次、0.2 秒；
4. `User_Task.c` 每 20 ms 调用一次 `PID_Init()`，不利于明确管理 PID 状态；
5. `keep_high()` 对 z 输出使用了负号，但 `TMD_Set()` 注释又定义 z 上为正，必须结合真实高度反馈方向做无桨验证，不能仅凭注释猜测；
6. 目标高度变化、无效命令、遥控退出和降落时应有明确的 PID 重置规则。

所以本次树莓派包只负责可靠发送“目标高度”，不会假装旧飞控高度 PID 已经安全可用。

## 5. 本阶段已完成的飞控端修改

已经完成：

1. 新建 `Ano_RpiBridge.c/.h`：解析 16 字节命令、CRC16、看门狗、单位换算、状态帧；
2. `Drv_Uart.c`：令底板串口2的单字节入口指向 `AnoRpiBridgeReceiveByte()`；
3. `Drv_Uart.h`：保留并使用 `DrvUart2SendBuf()` 返回 8 字节状态；
4. `User_Task.c`：去掉旧位置/z 等级/UART3 降落流程，50 Hz 调用新桥更新函数；
5. Keil工程文件：使用 `Ano_RpiBridge.c/.h` 替换旧 `Ano_ExtVelocity.c` 编译项；
6. 飞控板串口4的匿名光流入口和500000波特率保持不变。

尚未完成的是 `control.c/.h` 的 z PID 重构、真实 `height_reached/land_completed` 判定以及Keil/无桨验证。在这些步骤完成前不能做真实闭环飞行。
