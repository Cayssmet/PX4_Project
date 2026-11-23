# car_serial_bridge v1.0.2

树莓派侧的小车通信桥，严格使用《D题陆空协同系统统一设备通信协议 v1.7》。

- 串口：默认 /dev/car_bluetooth，115200、8N1。
- 接收：28字节 CAR_STATUS(0x10)，重复sequence允许使用。
- 发送：10 Hz、23字节 MISSION_SYNC(0x11)。
- 超时：500 ms没有合法CAR_STATUS即撤销小车链路、C点动作窗口和旧位置使用，不会自行停止小车。
- START：在合法session/task和START_LATCHED存在时周期发布；任务管理器负责防重；READY/session0会复位。
- C点动作窗口：必须同时满足新鲜last_checkpoint=C和C_DECEL_COMPLETE=1。
- 简化task2完成：`TASK2_COMPLETE`继续报告`UAV_LANDED_ON_CAR`，并同时发送
  `LANDED_ON_CAR | HOLD_5S_COMPLETE | TASK_COMPLETE`，不会误报为H点降落。

主要ROS 2接口：

| 方向 | 话题 | 含义 |
|---|---|---|
| 发布 | /car/status | 完整CAR_STATUS结构化镜像 |
| 发布 | /car/link_ok | 500 ms新鲜度结果 |
| 发布 | /competition/task_mode | 小车KEY1/KEY2选择 |
| 发布 | /competition/start | 倒计时结束后的单次启动事件 |
| 发布 | /car/drop_window | C点已过且减速完成 |
| 发布 | /car/lap_complete | 小车完成一圈回到A |
| 发布 | /car/mission_sync | 实际发送给小车的MISSION_SYNC快照 |
| 订阅 | /competition/phase | 树莓派权威任务阶段 |
| 订阅 | /competition/active_task_mode | 已锁定任务 |
| 订阅 | /flight/status | 无人机局部状态来源 |
| 订阅 | /payload/drop_completed | 投放完成及允许小车加速的来源 |

单独启动：ros2 launch car_serial_bridge car_serial_bridge.launch.py
