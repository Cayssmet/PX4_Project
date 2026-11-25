"""pos_link 桥接节点 —— 把 ROS 2 里的坐标推给 ESP32-S3。

数据方向是 **树莓派 → ESP32**：

    坐标源 ──(ROS 2 话题)──► 本节点 ──(TCP, 12 字节帧, 1 Hz)──► ESP32-S3

**坐标从哪来**：本节点只订阅一个 `geometry_msgs/PointStamped`，话题名由参数
`input_topic` 决定（默认 `/pos_link/target`）。谁算出坐标谁就发这个话题——
定位节点、任务节点、甚至手敲 `ros2 topic pub` 都行，本节点不关心算法。
换坐标源时不用改一行代码，改参数即可（见 launch 文件）。

**为什么树莓派当服务端**：ESP32 的 IP 是 DHCP 分的，树莓派事先不知道；
树莓派在热点里的地址是固定的 192.168.50.1。所以 ESP32 主动连上来，
树莓派记住这条连接，之后往里推帧。ESP32 重启会自己重连，树莓派掉线
ESP32 也会重连——两边都不需要配对方的地址簿。
"""

from __future__ import annotations

import math
import socket
import threading
import time
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from pos_link.frame import build_frame

ACCEPT_POLL_S = 0.5  # accept() 的超时，用来定期检查退出标志
SEND_TIMEOUT_S = 1.0  # 单个客户端的发送超时：ESP32 卡死时不能把定时器拖住
INT16_MIN, INT16_MAX = -32768, 32767


class PosLinkBridge(Node):
    """订阅坐标话题，按固定周期把最新坐标打包成 12 字节帧推给已连接的 ESP32。"""

    def __init__(self) -> None:
        super().__init__("pos_link_bridge")

        self.declare_parameter("listen_port", 5000)
        self.declare_parameter("input_topic", "/pos_link/target")
        self.declare_parameter("send_period_ms", 1000)
        self.declare_parameter("hold_last_on_stale", True)
        self.declare_parameter("stale_warn_s", 5.0)

        self._port = int(self.get_parameter("listen_port").value)
        self._input_topic = str(self.get_parameter("input_topic").value)
        period_ms = max(50, int(self.get_parameter("send_period_ms").value))
        self._hold_last = bool(self.get_parameter("hold_last_on_stale").value)
        self._stale_warn_s = float(self.get_parameter("stale_warn_s").value)

        self._lock = threading.Lock()
        self._clients: List[socket.socket] = []
        self._target: Optional[Tuple[int, int]] = None
        self._target_time = 0.0

        self._seq = 0
        self._frames_sent = 0
        self._clients_dropped = 0
        self._running = True

        self._server = self._open_server()
        self._accept_thread = threading.Thread(
            target=self._accept_loop, name="pos_link_accept", daemon=True
        )
        self._accept_thread.start()

        self.create_subscription(PointStamped, self._input_topic, self._on_target, 10)
        self.create_timer(period_ms / 1000.0, self._tick)
        self.create_timer(10.0, self._report)

        self.get_logger().info(
            f"已监听 0.0.0.0:{self._port}（等 ESP32 连上来）；"
            f"坐标取自 {self._input_topic}；下发周期 {period_ms} ms"
        )

    # ---------------- TCP 服务端 ----------------

    def _open_server(self) -> socket.socket:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            srv.bind(("0.0.0.0", self._port))
        except OSError as exc:
            srv.close()
            raise RuntimeError(
                f"端口 {self._port} 绑定失败（是不是已经有一个 bridge 在跑？）：{exc}"
            ) from exc
        srv.listen(4)
        srv.settimeout(ACCEPT_POLL_S)
        return srv

    def _accept_loop(self) -> None:
        """后台线程：专门收 ESP32 的连接，收完交给 _tick 往里推帧。"""
        while self._running:
            try:
                conn, addr = self._server.accept()
            except socket.timeout:
                continue
            except OSError:
                break  # 服务端 socket 被关掉了，正常退出

            conn.settimeout(SEND_TIMEOUT_S)
            try:
                # 1 Hz 的小包，攒包只会让对端多等几十毫秒
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except OSError:
                pass

            with self._lock:
                self._clients.append(conn)
                count = len(self._clients)
            self.get_logger().info(f"ESP32 已接入 {addr[0]}:{addr[1]}（当前 {count} 个连接）")

    def _drop_client(self, conn: socket.socket, why: str) -> None:
        with self._lock:
            if conn in self._clients:
                self._clients.remove(conn)
                left = len(self._clients)
            else:
                left = len(self._clients)
        try:
            conn.close()
        except OSError:
            pass
        self._clients_dropped += 1
        self.get_logger().warning(f"断开一个 ESP32 连接（剩余 {left} 个）：{why}")

    # ---------------- 坐标来源 ----------------

    def _on_target(self, msg: PointStamped) -> None:
        """坐标源的回调。想接别的来源，改这里就行——比如从服务调用拿值。"""
        x, y = msg.point.x, msg.point.y
        if not (math.isfinite(x) and math.isfinite(y)):
            self.get_logger().warning("丢弃非法坐标（NaN/Inf）")
            return

        ix = max(INT16_MIN, min(INT16_MAX, int(round(x))))
        iy = max(INT16_MIN, min(INT16_MAX, int(round(y))))
        if (ix, iy) != (int(round(x)), int(round(y))):
            self.get_logger().warning(
                f"坐标 ({x:.1f}, {y:.1f}) 超出 int16，已夹到 ({ix}, {iy})",
                throttle_duration_sec=10.0,
            )

        with self._lock:
            self._target = (ix, iy)
            self._target_time = time.monotonic()

    # ---------------- 定时下发 ----------------

    def _tick(self) -> None:
        with self._lock:
            clients = list(self._clients)
            target = self._target
            age = time.monotonic() - self._target_time

        if not clients:
            self.get_logger().info("还没有 ESP32 连上来，等待中……", throttle_duration_sec=10.0)
            return
        if target is None:
            self.get_logger().info(
                f"{self._input_topic} 上还没有坐标，暂不发送", throttle_duration_sec=10.0
            )
            return
        if age > self._stale_warn_s:
            if not self._hold_last:
                self.get_logger().warning(
                    f"坐标已 {age:.1f}s 未更新，暂停发送", throttle_duration_sec=5.0
                )
                return
            # 保持最后值：对执行机构来说「知道位置但不动」比「突然没有指令」安全
            self.get_logger().warning(
                f"坐标已 {age:.1f}s 未更新，继续下发最后一次值", throttle_duration_sec=10.0
            )

        x, y = target
        frame = build_frame(x, y, self._seq)
        self._seq = (self._seq + 1) & 0xFF

        broken = []
        for conn in clients:
            try:
                conn.sendall(frame)
            except OSError as exc:
                broken.append((conn, exc))

        ok = len(clients) - len(broken)
        if ok > 0:
            self._frames_sent += ok
            self.get_logger().info(
                f"已下发 seq={frame[3]} x={x} y={y}（{ok} 个连接）",
                throttle_duration_sec=5.0,
            )
        for conn, exc in broken:
            self._drop_client(conn, str(exc))

    def _report(self) -> None:
        with self._lock:
            n = len(self._clients)
        self.get_logger().info(
            f"统计：已发 {self._frames_sent} 帧，在线连接 {n} 个，累计断开 {self._clients_dropped} 次"
        )

    # ---------------- 收尾 ----------------

    def destroy_node(self) -> None:
        self._running = False
        server = getattr(self, "_server", None)
        if server is not None:
            try:
                server.close()
            except OSError:
                pass
        with self._lock:
            clients = list(self._clients)
            self._clients.clear()
        for conn in clients:
            try:
                conn.close()
            except OSError:
                pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = PosLinkBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
