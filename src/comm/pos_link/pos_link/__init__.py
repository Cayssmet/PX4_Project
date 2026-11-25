"""pos_link —— ESP32-S3 与树莓派之间的 TCP 定长帧链路（ROS 2 侧）。

对外只暴露两样东西：
  * ``frame`` —— 12 字节定长帧的编解码与 CRC 校验；
  * ``bridge_node`` —— 把收到的坐标帧发布成 ``PointStamped`` 的 ROS 2 节点。
"""

__version__ = "1.0.0"
