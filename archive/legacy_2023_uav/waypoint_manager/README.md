# waypoint_manager v1.5.0

本包包含：

- `waypoint_manager_node`：旧地面站/本地航线管理器，保留回归；
- `competition_mission_node`：D题任务状态机，总启动包使用它；
- `competition_flight_directive_node`：把比赛阶段转换为飞控V2的STANDBY、TAKEOFF、HEIGHT_HOLD和FINAL_LAND指令。

## 比赛任务选择

两项任务分开评测。正式运行时，小车必须先发送：

```text
/competition/task_mode  std_msgs/msg/UInt8
1 = 抛投任务（60分主线）
2 = 动态起降任务（20分）
```

随后再发送：

```text
/competition/start  std_msgs/msg/Bool  true
```

没收到合法task_mode时，即使START已经到达也只进入`WAIT_READY`，不会发送起飞航点。任务开始后忽略中途切换task_mode。当前执行任务会发布到`/competition/active_task_mode`，供地面站显示。

### READY复位

统一协议v1.7的合法READY为`session=0、task_mode=0、START=false`。收到READY后：

- 尚未开始任务时，清除已选择任务和未执行START，回到`WAIT_START`；
- 已活动或ERROR的任务，只有飞控状态新鲜、通信正常、无failsafe、未解锁、非空中且确认着地时才复位；
- 飞行中收到READY只限频告警并保持当前任务，不能清空航点或撤销任务状态；
- 成功复位后清除投放、视觉、圈完成和PID到达等上一轮锁存，固定session 1/2可再次使用；
- 周期重复READY不会重复打印拒绝日志。

## 共同起飞和截获流程

收到合法task_mode和START后，记录当时H点map位置与机头yaw，随后：

```text
H点升至1.23m（当前实飞激光参考高度，用于减小激光落到小车平台后继续爬升的风险）
→ 右移0.375m（当前实飞标定值）
→ 沿初始机头方向前追2.75m
→ 摄像头连续发现目标
→ 视觉相对位置控制接管
```

坐标约定为机头前方正、机体左侧正，所以右移使用：

```yaml
lateral_offset_m: -0.375
```

代码会拒绝非负横移参数，防止再次误配成左移。

悬停计时按任务区分：

- task1：同一flight session收到active_action_id=1和ACTION_SUCCEEDED后锁存TAKEOFF成功，并累计2秒；实时height_reached/HR不参与计时或阶段推进；
- task2：赛题没有额外3秒要求，飞控TAKEOFF成功且平面起点PID到达后立即右移追车；
- 两项任务的后续水平导航、投放和复飞均不再读取HR，但仍要求飞控链路安全且height_valid=true。

从进入`TAKEOFF_HOVER`起，树莓派最多等待20秒飞控给出合法的TAKEOFF成功反馈。
若反馈一直未完成，任务锁存在`SAFE_HOLD`、撤销路线输出且绝不强行进入右移；即使反馈随后恢复也不会突然续飞，
必须先安全着地，再由合法READY复位。这个看门狗只作用于树莓派—飞控内部状态，不改变小车、摄像头或地面站协议。

## task1抛投主线

```text
WAIT_START
→ WAIT_READY
→ TAKEOFF_HOVER
→ RIGHT_SHIFT
→ FORWARD_CHASE
→ TASK1_FOLLOW
→ TASK1_ALIGN_DROP
→ TASK1_DROP
→ TASK1_RETURN_HOME
→ TASK1_LAND_HOME
→ TASK1_COMPLETE
```

小车通信节点应在C→D直道期间持续发布：

```text
/car/drop_window = true
```

只有以下条件连续满足`drop_alignment_time_sec`后，任务管理器才发布一次投放请求：

- 小车投放窗口新鲜且为true；
- 视觉目标新鲜有效；
- `/pid/visual_control_active=true`且新鲜；
- 视觉PID报告对准；
- 飞控通信正常、无failsafe；
- 飞控高度数据有效。

投放接口：

| 话题 | 方向 | 含义 |
|---|---|---|
| `/payload/drop_request` Bool | 任务→投放节点 | true表示本轮一次性投放请求 |
| `/payload/drop_completed` Bool | 投放节点→任务 | 机构确认完成 |

旧的完成反馈不能提前完成新请求。进入TASK1_DROP后，摄像头明确失败、投放反馈超时、飞控异常或定位异常都会撤销本轮投放请求、禁止重复投放，并直接切换TASK1_RETURN_HOME；若飞控控制暂时不可用，则在链路恢复后继续返回H点。投放成功同样退出视觉控制并返回启动时记录的H点，在H点上方到达后通过`/mission/land_request`请求飞控最终降落。

## task2当前范围

当前比赛策略主动舍弃车顶复飞、返回H和第二次降落，只完成稳定落车：

```text
TASK2_VISUAL_APPROACH
→ TASK2_ALIGN_LAND
→ TASK2_DESCEND
→ TASK2_ON_CAR
→ TASK2_HOLD_5S
→ TASK2_COMPLETE
```

`TASK2_DESCEND`阶段使用视觉水平修正和固定-0.12 m/s下降；高度到0.20 m时锁存并发送现有
V2 `FINAL_LAND action=2`。下降中视觉丢失时锁存丢失瞬间高度、停止x/y/vz并等待重新对准，
不返回1.23 m巡航高度；视觉接管后Cartographer只记录，不再阻塞task2推进，yaw_rate保持0。
飞控必须同时报告同session的`action=2/SUCCEEDED`和`landed`后，任务才发布
`TASK2_ON_CAR`并进入`TASK2_HOLD_5S`，显示“已落车，稳定确认中”；落车状态持续稳定5秒后
进入`TASK2_COMPLETE`并显示“已稳定”。不会进入`TASK2_RETAKEOFF`、`TASK2_RETURN_H`或
`TASK2_LAND_H`。协议中的旧阶段编号继续保留，只用于兼容，不代表当前任务会执行。

## 主要输入输出

输入：`/competition/task_mode`、`/competition/start`、`/car/drop_window`、`/vision/target_valid`、`/pid/visual_control_active`、`/pid/target_reached`、当前位姿、飞控状态和投放完成反馈。

输出：PID目标、当前航点、任务状态、比赛阶段、视觉接管请求、投放请求、降落请求和当前任务编号。

V2飞控链路先周期发送STANDBY建立独立flight session；收到飞控正确回显后，
`TAKEOFF_HOVER`先产生一次TAKEOFF动作。飞控回报该动作成功后，全局阶段仍保持
`TAKEOFF_HOVER`，但飞控指令切换为HEIGHT_HOLD，使PID能够保持起飞点；`RIGHT_SHIFT`与
`FORWARD_CHASE`也均转换为HEIGHT_HOLD，水平速度来自PID。飞控状态短时超时、failsafe或错误时，
任务是否进入`SAFE_HOLD`由`enable_safe_hold_latch`决定；正式参数保持关闭。task session 1/2只用于
设备任务选择，不会直接用作flight session。
TAKEOFF首次生成时会锁存目标高度；同一session/action的后续重发不会跟随航点消息变化。

任务一在同一flight session收到`active_action_id=1 + ACTION_SUCCEEDED`后独立锁存TAKEOFF成功。
树莓派不会读取height_reached/HR决定是否开始计时、是否进入右移或是否完成后续航点。
成功锁存后累计2秒，到时进入右移。水平PID仍负责起飞点定点修正，但PID到达标志、
HR和Cartographer短时抖动都不再是阶段推进门槛；task2收到TAKEOFF成功后不额外等待。
该2秒不读取实时HR。
高度传感器失效仍由height_valid、飞控错误码和failsafe阻止任务继续，不等于删除高度安全保护。

当前右移、前追、视觉对准和下降链路已接入；task2以可靠落车并稳定停留5秒为终点，不再执行复飞和返航。真正触地高度仍需按激光到起落架的实测距离标定。

正式参数位于：

```text
/home/ljz/ros2_ws/src/waypoint_manager/config/competition_mission_params.yaml
```

总启动默认不会自动执行任务；正式比赛必须由小车发task_mode和START。摄像头、小车无线通信和投放机构节点仍需按上述接口实现并加入总启动包。
