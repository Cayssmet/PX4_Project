"""按每次D题任务自动保存可读事件流和完整rosbag。"""

from __future__ import annotations

import json
import math
import os
import shutil
import signal
import subprocess
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Callable

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from rclpy.node import Node
from std_msgs.msg import Bool, String, UInt8
from uav_interfaces.msg import (
    CarStatus,
    FlightCommand,
    FlightDirective,
    FlightStatus,
    MissionStatus,
    MissionSync,
    Waypoint,
)


ACTIVE_PHASES = {
    "WAIT_READY",
    "TAKEOFF_HOVER",
    "RIGHT_SHIFT",
    "FORWARD_CHASE",
    "TASK1_FOLLOW",
    "TASK1_ALIGN_DROP",
    "TASK1_DROP",
    "TASK1_RETURN_HOME",
    "TASK1_LAND_HOME",
    "TASK2_VISUAL_APPROACH",
    "TASK2_ALIGN_LAND",
    "TASK2_DESCEND",
    "TASK2_ON_CAR",
    "TASK2_HOLD_5S",
    "TASK2_RETAKEOFF",
    "TASK2_RETURN_HOME",
    "TASK2_LAND_HOME",
}
TERMINAL_PHASES = {
    "TASK1_COMPLETE",
    "TASK2_COMPLETE",
    "ERROR",
    "ABORTED",
}

DEFAULT_BAG_TOPICS = [
    "/competition/phase",
    "/competition/active_task_mode",
    "/competition/task_mode",
    "/competition/start",
    "/map/current_pose",
    "/map/current_pose_valid",
    "/pid/target_pose",
    "/pid/cmd_vel",
    "/pid/control_valid",
    "/pid/target_reached",
    "/pid/visual_control_active",
    "/vision/relative_error",
    "/vision/target_valid",
    "/vision/drop_aligned",
    "/vision/land_aligned",
    "/vision/link_ok",
    "/vision/state",
    "/vision/target_type",
    "/car/status",
    "/car/mission_sync",
    "/car/link_ok",
    "/car/drop_window",
    "/car/lap_complete",
    "/flight/directive",
    "/flight/command",
    "/flight/status",
    "/waypoint/current",
    "/mission/status",
    "/mission/land_request",
    "/payload/drop_request",
    "/payload/drop_completed",
    "/tf",
    "/tf_static",
]


def finite(value: float) -> float | None:
    return float(value) if math.isfinite(value) else None


def pose_dict(message: PoseStamped) -> dict[str, Any]:
    pose = message.pose
    return {
        "frame": message.header.frame_id,
        "x_m": finite(pose.position.x),
        "y_m": finite(pose.position.y),
        "z_m": finite(pose.position.z),
        "qx": finite(pose.orientation.x),
        "qy": finite(pose.orientation.y),
        "qz": finite(pose.orientation.z),
        "qw": finite(pose.orientation.w),
    }


def twist_dict(message: TwistStamped) -> dict[str, Any]:
    return {
        "frame": message.header.frame_id,
        "vx_m_s": finite(message.twist.linear.x),
        "vy_m_s": finite(message.twist.linear.y),
        "vz_m_s": finite(message.twist.linear.z),
        "yaw_rate_rad_s": finite(message.twist.angular.z),
    }


class FlightDataRecorder(Node):
    def __init__(self) -> None:
        super().__init__("flight_data_recorder")
        self.output_root = Path(
            os.path.expanduser(
                self.declare_parameter(
                    "output_root", "/home/ljz/ros2_ws/logs/flights"
                ).value
            )
        )
        self.record_bag = bool(self.declare_parameter("record_bag", True).value)
        self.record_scan = bool(self.declare_parameter("record_scan", True).value)
        self.max_bag_size_mb = int(
            self.declare_parameter("max_bag_size_mb", 2048).value
        )
        self.post_roll_sec = float(self.declare_parameter("post_roll_sec", 2.0).value)

        self.active_task = 0
        self.phase = "WAIT_START"
        self.start_seen = False
        self.run_dir: Path | None = None
        self.event_file = None
        self.rosbag_log_file = None
        self.bag_process: subprocess.Popen | None = None
        self.started_unix = 0.0
        self.stop_deadline: float | None = None
        self.message_counts: dict[str, int] = {}
        self.phases_seen: list[str] = []
        self.flight_sessions: set[int] = set()

        self.output_root.mkdir(parents=True, exist_ok=True)
        self._create_subscriptions()
        self.create_timer(0.2, self._housekeeping)
        self.get_logger().info(
            f"飞行记录器已就绪：目录={self.output_root}，rosbag={self.record_bag}，"
            f"雷达scan={self.record_scan}；等待小车START，不影响飞控输出"
        )

    def _subscribe(self, message_type, topic: str, converter: Callable[[Any], dict]) -> None:
        self.create_subscription(
            message_type,
            topic,
            lambda message, t=topic, c=converter: self._record(t, c(message)),
            50,
        )

    def _create_subscriptions(self) -> None:
        self.create_subscription(String, "/competition/phase", self._phase_callback, 10)
        self.create_subscription(
            UInt8, "/competition/active_task_mode", self._task_callback, 10
        )
        self.create_subscription(Bool, "/competition/start", self._start_callback, 10)

        self._subscribe(PoseStamped, "/map/current_pose", pose_dict)
        self._subscribe(Bool, "/map/current_pose_valid", lambda m: {"value": m.data})
        self._subscribe(PoseStamped, "/pid/target_pose", pose_dict)
        self._subscribe(TwistStamped, "/pid/cmd_vel", twist_dict)
        for topic in (
            "/pid/control_valid",
            "/pid/target_reached",
            "/pid/visual_control_active",
            "/vision/target_valid",
            "/vision/drop_aligned",
            "/vision/land_aligned",
            "/vision/link_ok",
            "/car/link_ok",
            "/car/drop_window",
            "/car/lap_complete",
            "/mission/land_request",
            "/payload/drop_request",
            "/payload/drop_completed",
        ):
            self._subscribe(Bool, topic, lambda m: {"value": m.data})
        self._subscribe(
            Vector3Stamped,
            "/vision/relative_error",
            lambda m: {
                "frame": m.header.frame_id,
                "x_m": finite(m.vector.x),
                "y_m": finite(m.vector.y),
                "z_m": finite(m.vector.z),
            },
        )
        for topic in ("/vision/state", "/vision/target_type"):
            self._subscribe(UInt8, topic, lambda m: {"value": int(m.data)})
        self._subscribe(CarStatus, "/car/status", self._car_status_dict)
        self._subscribe(MissionSync, "/car/mission_sync", self._mission_sync_dict)
        self._subscribe(FlightDirective, "/flight/directive", self._directive_dict)
        self._subscribe(FlightCommand, "/flight/command", self._command_dict)
        self._subscribe(FlightStatus, "/flight/status", self._flight_status_dict)
        self._subscribe(Waypoint, "/waypoint/current", self._waypoint_dict)
        self._subscribe(MissionStatus, "/mission/status", self._mission_status_dict)

    def _task_callback(self, message: UInt8) -> None:
        self.active_task = int(message.data)
        self._record("/competition/active_task_mode", {"value": self.active_task})
        if self.start_seen and self.active_task in (1, 2) and self.run_dir is None:
            self._start_run("start_and_task_ready")

    def _start_callback(self, message: Bool) -> None:
        self._record("/competition/start", {"value": bool(message.data)})
        if not message.data:
            return
        self.start_seen = True
        if self.active_task in (1, 2) and self.run_dir is None:
            self._start_run("car_start")

    def _phase_callback(self, message: String) -> None:
        previous = self.phase
        self.phase = message.data
        if (
            self.run_dir is None
            and self.active_task in (1, 2)
            and (self.start_seen or self.phase in ACTIVE_PHASES)
        ):
            self._start_run("active_phase")
        self._record(
            "/competition/phase", {"value": self.phase, "previous": previous}
        )
        if self.run_dir is not None and self.phase not in self.phases_seen:
            self.phases_seen.append(self.phase)
        if self.run_dir is not None and self.phase in TERMINAL_PHASES:
            self.stop_deadline = time.monotonic() + max(0.0, self.post_roll_sec)
        if self.run_dir is not None and self.phase == "WAIT_START" and previous != "WAIT_START":
            self.stop_deadline = time.monotonic() + max(0.0, self.post_roll_sec)

    def _start_run(self, trigger: str) -> None:
        now = datetime.now().astimezone()
        label = now.strftime("%Y%m%d_%H%M%S_%f")[:-3]
        self.run_dir = self.output_root / f"{label}_task{self.active_task}"
        self.run_dir.mkdir(parents=True, exist_ok=False)
        self.started_unix = time.time()
        self.message_counts = {}
        self.phases_seen = []
        self.flight_sessions = set()
        self.stop_deadline = None
        self.event_file = (self.run_dir / "events.jsonl").open(
            "a", encoding="utf-8", buffering=1
        )
        manifest = {
            "format_version": 1,
            "created_local": now.isoformat(),
            "task_mode": self.active_task,
            "trigger": trigger,
            "contains": "ROS topic bag + readable JSONL event stream",
            "coordinate_convention": "map: x initial-forward, y left; BODY: x forward, y left",
            "protocol": "Unified device v1.7; flight-controller V2.0 revision A",
        }
        self._write_json(self.run_dir / "manifest.json", manifest)
        self._record("recorder/start", manifest)
        if self.record_bag:
            self._start_rosbag()
        self.get_logger().info(f"本轮飞行开始记录：{self.run_dir}")

    def _start_rosbag(self) -> None:
        assert self.run_dir is not None
        ros2 = shutil.which("ros2")
        if ros2 is None:
            self.get_logger().error("找不到ros2命令；本轮仅记录events.jsonl")
            return
        topics = list(DEFAULT_BAG_TOPICS)
        if self.record_scan:
            topics.append("/scan")
        command = [
            ros2,
            "bag",
            "record",
            "--output",
            str(self.run_dir / "bag"),
            "--storage",
            "mcap",
            "--storage-preset-profile",
            "zstd_fast",
            "--disable-keyboard-controls",
            "--max-bag-size",
            str(max(1, self.max_bag_size_mb) * 1024 * 1024),
            "--topics",
            *topics,
        ]
        self.rosbag_log_file = (self.run_dir / "rosbag.log").open(
            "w", encoding="utf-8"
        )
        try:
            self.bag_process = subprocess.Popen(
                command,
                stdout=self.rosbag_log_file,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except OSError as error:
            self.get_logger().error(f"rosbag启动失败：{error}；继续保存JSONL")
            self.bag_process = None

    def _record(self, topic: str, data: dict[str, Any]) -> None:
        if self.event_file is None:
            return
        item = {
            "time_unix": time.time(),
            "time_monotonic": time.monotonic(),
            "topic": topic,
            "data": data,
        }
        self.event_file.write(json.dumps(item, ensure_ascii=False) + "\n")
        self.message_counts[topic] = self.message_counts.get(topic, 0) + 1

    def _housekeeping(self) -> None:
        if self.bag_process is not None and self.bag_process.poll() is not None:
            return_code = self.bag_process.returncode
            self.get_logger().error(
                f"rosbag进程提前退出，返回码={return_code}；JSONL仍继续记录"
            )
            self.bag_process = None
        if self.stop_deadline is not None and time.monotonic() >= self.stop_deadline:
            self._stop_run(f"terminal_phase:{self.phase}")

    def _stop_run(self, reason: str) -> None:
        if self.run_dir is None:
            return
        self._record("recorder/stop", {"reason": reason})
        bag_ok = self._stop_rosbag()
        summary = {
            "started_unix": self.started_unix,
            "stopped_unix": time.time(),
            "duration_sec": max(0.0, time.time() - self.started_unix),
            "task_mode": self.active_task,
            "final_phase": self.phase,
            "stop_reason": reason,
            "flight_session_ids": sorted(self.flight_sessions),
            "phases_seen": self.phases_seen,
            "message_counts": self.message_counts,
            "rosbag_closed_cleanly": bag_ok,
        }
        self._write_json(self.run_dir / "summary.json", summary)
        if self.event_file is not None:
            self.event_file.flush()
            os.fsync(self.event_file.fileno())
            self.event_file.close()
        completed_dir = self.run_dir
        self.event_file = None
        self.run_dir = None
        self.started_unix = 0.0
        self.stop_deadline = None
        self.start_seen = False
        self.get_logger().info(f"本轮飞行日志已安全保存：{completed_dir}")

    def _stop_rosbag(self) -> bool:
        process = self.bag_process
        self.bag_process = None
        clean = True
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=12.0)
            except subprocess.TimeoutExpired:
                clean = False
                process.terminate()
                try:
                    process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3.0)
            except ProcessLookupError:
                pass
        if process is not None and process.returncode not in (None, 0):
            clean = False
        if self.rosbag_log_file is not None:
            self.rosbag_log_file.flush()
            self.rosbag_log_file.close()
            self.rosbag_log_file = None
        return clean

    @staticmethod
    def _write_json(path: Path, data: dict[str, Any]) -> None:
        with path.open("w", encoding="utf-8") as stream:
            json.dump(data, stream, ensure_ascii=False, indent=2)
            stream.write("\n")

    def _car_status_dict(self, m: CarStatus) -> dict[str, Any]:
        return {
            "communication_ok": m.communication_ok,
            "sequence": m.sequence,
            "session_id": m.session_id,
            "task_mode": m.task_mode,
            "mission_phase": m.mission_phase,
            "car_state": m.car_state,
            "last_checkpoint": m.last_checkpoint,
            "error_code": m.error_code,
            "flags": m.flags,
            "countdown_remaining_ms": m.countdown_remaining_ms,
            "speed_mm_s": m.speed_mm_s,
            "action_target_speed_mm_s": m.action_target_speed_mm_s,
            "distance_since_checkpoint_mm": m.distance_since_checkpoint_mm,
        }

    def _mission_sync_dict(self, m: MissionSync) -> dict[str, Any]:
        return {
            "sequence": m.sequence,
            "session_id": m.session_id,
            "task_mode": m.task_mode,
            "mission_phase": m.mission_phase,
            "uav_state": m.uav_state,
            "sync_flags": m.sync_flags,
            "countdown_remaining_ms": m.countdown_remaining_ms,
            "coordination_flags": m.coordination_flags,
            "action_elapsed_ms": m.action_elapsed_ms,
            "error_code": m.error_code,
        }

    def _directive_dict(self, m: FlightDirective) -> dict[str, Any]:
        return {
            "control_mode": m.control_mode,
            "session_id": m.session_id,
            "action_id": m.action_id,
            "vertical_velocity_m_s": finite(m.vertical_velocity),
            "target_height_m": finite(m.target_height),
            "control_valid": m.control_valid,
        }

    def _command_dict(self, m: FlightCommand) -> dict[str, Any]:
        return {
            "protocol_version": m.protocol_version,
            "control_mode": m.control_mode,
            "session_id": m.session_id,
            "action_id": m.action_id,
            "vx_m_s": finite(m.velocity_x),
            "vy_m_s": finite(m.velocity_y),
            "vz_m_s": finite(m.velocity_z),
            "yaw_rate_rad_s": finite(m.yaw_rate),
            "target_height_m": finite(m.target_height),
            "control_valid": m.control_valid,
            "land_request": m.land_request,
        }

    def _flight_status_dict(self, m: FlightStatus) -> dict[str, Any]:
        if m.active_session_id:
            self.flight_sessions.add(int(m.active_session_id))
        return {
            "protocol_version": m.protocol_version,
            "communication_ok": m.communication_ok,
            "current_mode": m.current_mode,
            "error_code": m.error_code,
            "active_session_id": m.active_session_id,
            "active_action_id": m.active_action_id,
            "action_result": m.action_result,
            "current_height_m": finite(m.current_height),
            "vertical_velocity_m_s": finite(m.vertical_velocity),
            "height_age_ms": m.height_age_ms,
            "command_age_ms": m.command_age_ms,
            "height_valid": m.height_valid,
            "height_reached": m.height_reached,
            "armed": m.armed,
            "airborne": m.airborne,
            "landed": m.landed,
            "failsafe": m.failsafe,
            "action_accepted": m.action_accepted,
            "action_completed": m.action_completed,
        }

    @staticmethod
    def _waypoint_dict(m: Waypoint) -> dict[str, Any]:
        return {
            "id": m.id,
            "x_m": finite(m.x),
            "y_m": finite(m.y),
            "yaw_rad": finite(m.yaw),
            "target_height_m": finite(m.target_height),
            "hold_time_sec": finite(m.hold_time),
        }

    @staticmethod
    def _mission_status_dict(m: MissionStatus) -> dict[str, Any]:
        return {
            "route_id": m.route_id,
            "current_index": m.current_index,
            "current_waypoint_id": m.current_waypoint_id,
            "waypoint_count": m.waypoint_count,
            "state": m.state,
            "route_valid": m.route_valid,
            "target_sent": m.target_sent,
            "mission_completed": m.mission_completed,
        }

    def destroy_node(self) -> bool:
        if self.run_dir is not None:
            self._stop_run("recorder_shutdown")
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FlightDataRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
