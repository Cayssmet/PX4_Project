"""把 ESP32-S3 通过 TCP 发来的坐标帧发布成 ROS 2 话题。

链路形状::

    ESP32-S3 (STA)  ──WiFi──►  树莓派 wlan0 (AP, 192.168.50.1)
                                   │
                                   └─ 本节点 TCP Server :5000 ──► /esp32/position

本节点只做三件事：

1. 收字节流，用帧头 ``AA 55`` 重新对齐，逐帧切出 12 字节；
2. 校验帧尾和 CRC16，坏的丢掉并记账（不重传 —— 1 Hz 的坐标丢一帧无所谓，
   下一帧 1 秒后就到，重传机制在 TCP 之上再叠一层反而更难维护）；
3. 把 x/y 发布成 ``geometry_msgs/PointStamped``。

TCP 由 ESP32 主动连过来，所以树莓派这边**不发起连接、也不需要知道 ESP32 的地址**：
ESP32 掉线重连、换 IP、重启，这边都只是「一个客户端断开、一会儿再来一个」。
"""

import socket
import threading

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from pos_link.frame import FRAME_LEN, HEAD, parse_frame


class BridgeNode(Node):
    """TCP 收帧 → ``/esp32/position``。"""

    def __init__(self) -> None:
        super().__init__("wifi_bridge")

        self.declare_parameter("listen_port", 5000)
        self.declare_parameter("frame_id", "esp32")
        self.declare_parameter("timeout_s", 3.0)

        self._port = int(self.get_parameter("listen_port").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._timeout = float(self.get_parameter("timeout_s").value)

        self._pub = self.create_publisher(PointStamped, "esp32/position", 10)

        self._lock = threading.Lock()
        self._n_frames = 0
        self._n_bad = 0
        self._n_gap = 0
        self._last_seq = None
        self._last_rx = None

        self._stop = threading.Event()
        self._srv = None
        self._thread = threading.Thread(
            target=self._accept_loop, name="pos_link-accept", daemon=True
        )
        self._thread.start()

        self.create_timer(1.0, self._watchdog)
        self.get_logger().info(f"TCP 监听 0.0.0.0:{self._port}")

    # ------------------------------------------------------------------ 接入

    def _accept_loop(self) -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # SO_REUSEADDR：桥接节点重启时 5000 端口可能还在 TIME_WAIT，
        # 没有这一行就要等 60 秒才能再起来。
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", self._port))
        srv.listen(4)
        srv.settimeout(1.0)
        self._srv = srv

        while not self._stop.is_set():
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            self.get_logger().info(f"客户端接入 {addr[0]}:{addr[1]}")
            # 每个客户端一个线程：ESP32 重连时旧线程可能还没退出，
            # 用线程而不是复用单连接，逻辑最简单。
            threading.Thread(
                target=self._serve,
                args=(conn, addr),
                name="pos_link-client",
                daemon=True,
            ).start()

    def _serve(self, conn: socket.socket, addr) -> None:
        conn.settimeout(1.0)
        buf = bytearray()
        try:
            while not self._stop.is_set():
                try:
                    chunk = conn.recv(256)
                except socket.timeout:
                    continue
                except OSError:
                    break
                if not chunk:
                    break
                buf.extend(chunk)
                self._drain(buf)
        finally:
            try:
                conn.close()
            except OSError:
                pass
            self.get_logger().info(f"客户端断开 {addr[0]}:{addr[1]}")

    # ------------------------------------------------------------------ 切帧

    def _drain(self, buf: bytearray) -> None:
        """从缓冲区头部反复切帧，直到凑不满一帧为止。"""
        while True:
            idx = buf.find(HEAD)
            if idx < 0:
                # 一个帧头都没有。留最后一个字节：0xAA 0x55 可能被切在两次 recv 之间。
                if len(buf) > 1:
                    del buf[:-1]
                return
            if idx:
                del buf[:idx]  # 帧头之前的垃圾直接丢
            if len(buf) < FRAME_LEN:
                return  # 半帧，等下一次 recv

            parsed = parse_frame(bytes(buf[:FRAME_LEN]))
            if parsed is None:
                with self._lock:
                    self._n_bad += 1
                del buf[0]  # 假帧头：退一格，重新找下一个 AA 55
                continue

            del buf[:FRAME_LEN]
            self._publish(*parsed)

    def _publish(self, seq: int, x: int, y: int) -> None:
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = 0.0
        self._pub.publish(msg)

        with self._lock:
            self._n_frames += 1
            if self._last_seq is not None and seq != (self._last_seq + 1) % 256:
                self._n_gap += 1  # 序号不连续 = 路上丢了，记一笔
            self._last_seq = seq
            self._last_rx = self.get_clock().now()
            total = self._n_frames

        self.get_logger().info(
            f"帧 seq={seq} x={x} y={y} 累计={total}",
            throttle_duration_sec=5.0,
        )

    # ---------------------------------------------------------------- 看门狗

    def _watchdog(self) -> None:
        with self._lock:
            last = self._last_rx
            n_frames, n_bad, n_gap = self._n_frames, self._n_bad, self._n_gap
        if last is None:
            return
        age = (self.get_clock().now() - last).nanoseconds / 1e9
        if age > self._timeout:
            self.get_logger().warning(
                f"{age:.0f}s 未收到帧 —— 总帧 {n_frames} 坏帧 {n_bad} 跳变 {n_gap}",
                throttle_duration_sec=5.0,
            )

    def destroy_node(self) -> bool:
        self._stop.set()
        if self._srv is not None:
            try:
                self._srv.close()
            except OSError:
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C 时 rclpy 自己已经 shutdown 过一次，这里不要再调，
        # 否则报 RCLError: failed to shutdown: rcl_shutdown already called.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
