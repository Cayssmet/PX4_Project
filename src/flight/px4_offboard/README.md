# px4_offboard

PX4 1.17 + uXRCE-DDS 的伴飞适配层。它**取代** `flight_controller_bridge` 在系统中的位置：
对上层（PID、任务状态机、建图链）保持 `/flight/command`、`/flight/status` 等话题接口不变，
只把下层从"凌霄飞控自定义串口协议"换成"uXRCE-DDS + PX4 offboard"。

> 设计依据见 `docs/PX4伴飞工程计划书.md`（尤其 §5–§9 的坐标系与链路设计、§10 的目录规划）。
> 施工顺序见 `docs/M1_操作手册_派能读飞控.md`。

## 版本状态

| 版本 | 内容 | 能否上机 |
| --- | --- | --- |
| **v2.0.0（当前）** | 只有 `frame_transforms`（纯数学坐标换算库）+ `qos_profiles`（PX4 专用 QoS 助手）+ 单元测试。**没有节点、不接任何运行中的链路。** | 只跑 `colcon test`，不涉及飞行 |
| 下一版 | `px4_pose_uplink_node`：把 `/map/current_pose` 转成 `/fmu/in/vehicle_visual_odometry`（对应里程碑 **M3**，室内 offboard 的前置条件） | 手持验证 |
| 再下一版 | `px4_offboard_bridge_node`：先只做"订阅 `/fmu/out/*` 填 `/flight/status`"（只读，零风险），再加 `vehicle_command`（M2），最后加 offboard 三件套（M4） | 拆桨 / 系留 |

## 为什么坐标换算要单独做成一个库

**坐标系换算是本项目炸机风险第一名。** 前后左右全反、或者姿态和速度用了两套不同源的
换算，在地面上完全看不出来，一起飞就往错误方向冲。

所以这里刻意：

- 不依赖 ROS、不依赖 Eigen（只需要 `<cmath>`），可以脱离 ROS 环境编译和测试；
- 单元测试里放了一条 `BodyVectorWorldConsistency`：它验证"姿态换算法则"和"机体矢量换算法则"
  **互相自洽**。单看任何一条都可能通过，两条不同源才会让飞机反向。
- 生产代码里**绝不用闭式**，一律用两次显式 Hamilton 四元数乘法，与 PX4 官方
  `px4_ros_com/src/lib/frame_transforms.cpp` 逐字一致；闭式只在测试里做交叉校验。

### 三条换算法则（互相不一样，混淆就出事）

| 对象 | ROS 侧 | PX4 侧 | 法则 |
| --- | --- | --- | --- |
| 世界系位置 / 速度 | ENU | NED | `(x, y, z) -> (y, x, -z)`（**自逆**） |
| 机体矢量 | FLU | FRD | `(x, y, z) -> (x, -y, -z)`（**自逆**，注意**不换位**） |
| 姿态 | ENU/FLU | NED/FRD | `q_px4 = NED_ENU_Q ⊗ q_enu ⊗ AIRCRAFT_BASELINK_Q` |
| 偏航角 | ENU | NED | `yaw_px4 = pi/2 - yaw_map`（**是反射，不是常数偏移**） |
| 偏航角速度 | ENU | NED | `yaw_rate_px4 = -yaw_rate_map`（纯变号） |

其中 `NED_ENU_Q = (0, √2/2, √2/2, 0)`、`AIRCRAFT_BASELINK_Q = (0, 1, 0, 0)`。

**偏航为什么不是加一个常数**：ENU 的 z 朝上、NED 的 z 朝下，两边绕 z 的正方向相反。
`yaw_map = 0` 对应 `yaw_px4 = 90°`，而 `yaw_map = -90°` 对应 `yaw_px4 = 180°`——
差值符号已经反过来了，所以不存在能对所有航向都成立的常数 offset。

## 为什么 QoS 要单独做成一个头文件

PX4 的 QoS **不是 ROS 2 默认值**。ROS 2 默认 `KEEP_LAST(10) + RELIABLE + VOLATILE`，
PX4 用的是：

| 方向 | 话题 | PX4 侧角色 | durability | reliability |
| --- | --- | --- | --- | --- |
| 飞控 → 派 | `/fmu/out/*` | 发布者 | `TRANSIENT_LOCAL` | `BEST_EFFORT` |
| 派 → 飞控 | `/fmu/in/*` | 订阅者 | `VOLATILE` | `BEST_EFFORT` |

不匹配时**不报错**，只表现为"一个话题都收不到 / 发不进去"，极易被误判成 Agent 或串口故障。
因此所有跟 PX4 有关的话题都必须走 `px4_subscription_qos()` / `px4_publication_qos()`。

## 编译与测试

```bash
# 1) 先准备 px4_msgs（当前 v2.0.0 还用不到，但下一版就需要）
bash tools/setup_px4_msgs.sh

# 2) 编译
cd ~/ros2_ws
colcon build --symlink-install --packages-select px4_offboard

# 3) 跑坐标系单元测试（这一步必须先通过，再谈任何飞行）
colcon test --packages-select px4_offboard --event-handlers console_direct+
colcon test-result --verbose
```

v2.0.0 只有纯数学库，**不装 px4_msgs 也能单独编译和测试**。

## 无 ROS 环境下的交叉校验

如果手边没有 ROS 2 环境（例如在 Windows 上），可以用 Python 版脚本做同一套随机位姿校验：

```bash
python3 tools/verify_frames.py
```

它和 `test/test_frame_transforms.cpp` 用的是同一套算法与随机位姿生成方式，
两边结果应当一致。改动 `frame_transforms.cpp` 后建议两边都跑一遍。
