# gcs_serial_bridge v1.0

树莓派—地面站只读状态桥，严格实现《D题统一设备通信协议 v1.7》。节点不发布飞行控制命令；地面站离线、心跳超时或串口拔出只会令 `/gcs/link_ok=false`，不会改变比赛任务和飞控输出。

## 设备拓扑

- 树莓派通过 `/dev/gcs_bluetooth` 向地面站发送 `UAV_STATUS(0x30)`。
- 小车通过自己的无线链路直接向地面站发送 `CAR_STATUS`。
- 本包不会把 `/car/status` 原样转发给地面站；它只把小车链路状态汇总进无人机状态标志和错误码。

串口固定为 115200、8N1、无软硬件流控。统一帧使用 `DA 7E` 帧头、线缆版本 `0x01`、大端字段、checksum8 和 `0D 0A` 帧尾。

## 线缆消息

### UAV_STATUS 0x30

树莓派以 10 Hz 发送固定 29 字节帧，18 字节载荷依次为：

1. `task_mode`、`mission_phase`、`uav_state`、`error_code`；
2. 16 位状态标志；
3. 相对 H 点的 `x/y`，单位 mm，x 为起飞初始机头前方，y 为左方；
4. 飞控当前高度，单位 mm；
5. START 后经过秒数。

READY 和倒计时阶段持续更新 H 点基准；START 后冻结。session、task 和投放/上车完成标志在任务期间锁存，只有合法 READY（session=0、task=0）复位。

错误优先级为：飞控失联或故障、定位无效、小车超时、视觉阶段视觉超时、任务/session错误。

### GCS_STATUS 0x31

地面站可选发送固定 15 字节心跳，4 字节载荷为：

1. `gcs_state`；
2. `mission_phase_echo`；
3. 标志位（显示就绪、正在记录）；
4. 保留字节 0。

超过 1 秒未收到合法心跳，仅报告地面站离线。重复 sequence 和 255→0 回绕都允许，不会锁死通信。

## ROS 2接口

输入：

- `/competition/phase`
- `/competition/active_task_mode`
- `/car/status`、`/car/link_ok`
- `/map/current_pose`、`/map/current_pose_valid`
- `/flight/status`
- `/vision/link_ok`
- `/pid/visual_control_active`
- `/payload/drop_completed`

输出：

- `/gcs/uav_status`：本次将要发送的完整 UAV_STATUS 快照；
- `/gcs/status`：最近一条合法 GCS_STATUS；
- `/gcs/link_ok`：1 秒内是否收到合法地面站心跳。

## 参数

默认参数位于 `config/gcs_serial_bridge_params.yaml`：

- `serial_device=/dev/gcs_bluetooth`
- `baud_rate=115200`
- `send_rate_hz=10.0`
- `heartbeat_timeout_ms=1000`
- `input_timeout_ms=500`
- `vision_timeout_ms=200`
- `reconnect_interval_ms=1000`

## 构建与测试

```bash
cd /home/ljz/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select uav_interfaces gcs_serial_bridge uav_bringup
colcon test --packages-select uav_interfaces gcs_serial_bridge uav_bringup
colcon test-result --verbose
```

伪串口测试覆盖固定 29 字节向量、GCS 心跳、分段输入、坏帧重同步、sequence 回绕和心跳离线后继续发送。

## 实机联调

1. 确认地面站模块指向 `/dev/gcs_bluetooth`，且没有第二个程序占用串口。
2. 总服务安全启动后，在地面站十六进制接收界面确认约 10 Hz 出现 `DA 7E 01 30`，每帧 29 字节。
3. 地面站按 v1.7 发送一条合法 `0x31` 心跳。
4. 树莓派检查 `/gcs/link_ok=true` 和 `/gcs/status` 字段。
5. 停止心跳超过 1 秒，确认只有 `/gcs/link_ok` 变为 false，任务阶段和飞控话题不变化。
