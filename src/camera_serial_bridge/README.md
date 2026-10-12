# camera_serial_bridge v1.0

本包严格实现 `D_UAV_UNIFIED_DEVICE_PROTOCOL_v1.6.md` 的树莓派—摄像头链路：

- 10 Hz发送固定19字节 `VISION_COMMAND (0x20)`；
- 接收固定23字节 `VISION_DATA (0x21)`；
- 校验 `DA 7E`、version 1、固定长度、checksum8和 `0D 0A`；
- 将大端毫米位移转换成 `base_link` 下的米：x前、y左、z恒为0；
- 数据超过200 ms、`IMAGE_FRESH=0`、目标/XY无效或视觉错误时立即发布无效零位移；
- 摄像头未连接时节点保持运行并自动重连，不影响整套系统其他功能包。

## ROS 2接口

输入：`/competition/phase`、`/competition/active_task_mode`。

输出：`/vision/relative_error`、`/vision/target_valid`、`/vision/drop_aligned`、
`/vision/land_aligned`、`/vision/link_ok`、`/vision/state`、`/vision/target_type`。

串口默认 `/dev/ttyAMA0`、115200 8N1，使用树莓派 GPIO14/15 硬件串口。
