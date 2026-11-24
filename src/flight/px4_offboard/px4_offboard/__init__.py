# -*- coding: utf-8 -*-
"""px4_offboard —— 树莓派 ↔ PX4 的 MAVLink 桥。

模块划分
--------
* `mavlink_codec` —— 零依赖的 MAVLink v1 编解码（只实现本项目用到的消息）
* `frames`        —— ROS 2（ENU/FLU）↔ PX4（NED/FRD）的坐标与姿态换算
* `px4_node`      —— rclpy 节点（第一阶段：只读下行 → `/flight/status`）

`mavlink_codec` 与 `frames` 都不 import ROS，可以脱离 ROS 环境单独测试。
"""

__version__ = "3.1.0"
