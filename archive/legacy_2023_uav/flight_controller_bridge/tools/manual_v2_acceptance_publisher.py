#!/usr/bin/env python3
# D-UAV UART V2.0真实飞控无桨验收的单一ROS控制发布器。

import argparse
import json
import math
from pathlib import Path
import secrets
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from uav_interfaces.msg import FlightCommand, FlightDirective, FlightStatus


MODE_BY_NAME = {
    "standby": FlightDirective.STANDBY,
    "takeoff": FlightDirective.TAKEOFF,
    "height_hold": FlightDirective.HEIGHT_HOLD,
    "precision_descent": FlightDirective.PRECISION_DESCENT,
    "final_land": FlightDirective.FINAL_LAND,
    "reset": FlightDirective.RESET_MISSION,
}
CONTINUOUS_MODES = {
    FlightDirective.STANDBY,
    FlightDirective.HEIGHT_HOLD,
    FlightDirective.PRECISION_DESCENT,
}
ZERO_MOTION_MODES = {
    FlightDirective.STANDBY,
    FlightDirective.TAKEOFF,
    FlightDirective.FINAL_LAND,
    FlightDirective.RESET_MISSION,
}


def parse_u32(value):
    parsed = int(value, 0)
    if not 0 < parsed <= 0xFFFFFFFF:
        raise argparse.ArgumentTypeError("session必须是1..0xFFFFFFFF")
    return parsed


def load_or_create_session(args):
    session_path = Path(args.session_file)
    if args.session is not None:
        session = args.session
    elif args.new_session:
        if args.mode != "standby":
            raise ValueError("只有standby允许使用--new-session")
        # 明确避开外部统一协议固定使用的1和2。
        session = secrets.randbelow(0xFFFFFFFD) + 3
        session_path.write_text(f"{session}\n", encoding="ascii")
    else:
        try:
            session = int(session_path.read_text(encoding="ascii").strip(), 0)
        except (FileNotFoundError, ValueError) as error:
            raise ValueError("未找到flight session；先运行standby --new-session") from error
    if not 0 < session <= 0xFFFFFFFF or session in (1, 2):
        raise ValueError("flight session必须为非零uint32，且不能直接使用外部session 1或2")
    return session


def validate(args, mode):
    if mode in CONTINUOUS_MODES and args.action_id != 0:
        raise ValueError("STANDBY/HEIGHT_HOLD/PRECISION_DESCENT的action-id必须为0")
    if mode not in CONTINUOUS_MODES and args.action_id <= 0:
        raise ValueError("TAKEOFF/FINAL_LAND/RESET的action-id必须为递增非零值")
    if math.hypot(args.vx, args.vy) > 1.5:
        raise ValueError("水平合速度不能超过1.5 m/s")
    if abs(args.yaw) > 2.0:
        raise ValueError("yaw速度绝对值不能超过2.0 rad/s")
    if abs(args.vz) > 0.5:
        raise ValueError("垂直速度绝对值不能超过0.5 m/s")
    if not 0.0 <= args.height <= 1.7:
        raise ValueError("高度必须位于0..1.7 m")
    if mode == FlightDirective.TAKEOFF and args.height < 0.3:
        raise ValueError("TAKEOFF目标高度必须位于0.3..1.7 m")
    if mode in (FlightDirective.HEIGHT_HOLD, FlightDirective.PRECISION_DESCENT) \
            and args.height < 0.1:
        raise ValueError("HEIGHT_HOLD/PRECISION_DESCENT目标高度必须位于0.1..1.7 m")
    if mode in ZERO_MOTION_MODES and any(
        abs(value) > 1e-9 for value in (args.vx, args.vy, args.vz, args.yaw)
    ):
        raise ValueError("该模式所有速度字段必须为0")
    if mode in (FlightDirective.STANDBY, FlightDirective.FINAL_LAND,
                FlightDirective.RESET_MISSION) and abs(args.height) > 1e-9:
        raise ValueError("该模式height必须为0")


class AcceptancePublisher(Node):
    def __init__(self, args, mode, session):
        super().__init__("manual_v2_acceptance_publisher")
        retained = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        normal = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.directive_pub = self.create_publisher(
            FlightDirective, "/flight/directive", retained)
        self.pid_pub = self.create_publisher(TwistStamped, "/pid/cmd_vel", normal)
        self.pid_valid_pub = self.create_publisher(Bool, "/pid/control_valid", normal)
        self.create_subscription(FlightCommand, "/flight/command", self.on_command, normal)
        self.create_subscription(FlightStatus, "/flight/status", self.on_status, normal)
        self.args = args
        self.mode = mode
        self.session = session
        self.last_print = 0.0
        self.log_handle = None
        if args.log_file:
            log_path = Path(args.log_file)
            log_path.parent.mkdir(parents=True, exist_ok=True)
            self.log_handle = log_path.open("a", encoding="utf-8")
        self.timer = self.create_timer(1.0 / args.rate, self.publish_setpoint)

    def write_event(self, kind, data):
        record = {"time_unix": time.time(), "kind": kind, **data}
        if self.log_handle is not None:
            self.log_handle.write(json.dumps(record, ensure_ascii=False) + "\n")
            self.log_handle.flush()
        now = time.monotonic()
        if not self.args.quiet and kind == "status" and now - self.last_print >= 0.5:
            self.last_print = now
            print(json.dumps(record, ensure_ascii=False), flush=True)

    def publish_setpoint(self):
        stamp = self.get_clock().now().to_msg()
        directive = FlightDirective()
        directive.header.stamp = stamp
        directive.control_mode = self.mode
        directive.session_id = self.session
        directive.action_id = self.args.action_id
        directive.vertical_velocity = self.args.vz
        directive.target_height = self.args.height
        directive.control_valid = not self.args.invalid
        self.directive_pub.publish(directive)

        twist = TwistStamped()
        twist.header.stamp = stamp
        twist.header.frame_id = "base_link"
        twist.twist.linear.x = self.args.vx
        twist.twist.linear.y = self.args.vy
        twist.twist.angular.z = self.args.yaw
        self.pid_pub.publish(twist)
        valid = Bool()
        valid.data = not self.args.invalid
        self.pid_valid_pub.publish(valid)

    def on_command(self, message):
        self.write_event("command", {
            "protocol": int(message.protocol_version),
            "mode": int(message.control_mode),
            "session": int(message.session_id),
            "action": int(message.action_id),
            "vx": float(message.velocity_x),
            "vy": float(message.velocity_y),
            "vz": float(message.velocity_z),
            "yaw": float(message.yaw_rate),
            "height": float(message.target_height),
            "valid": bool(message.control_valid),
        })

    def on_status(self, message):
        self.write_event("status", {
            "communication_ok": bool(message.communication_ok),
            "mode": int(message.current_mode),
            "error": int(message.error_code),
            "session": int(message.active_session_id),
            "action": int(message.active_action_id),
            "action_result": int(message.action_result),
            "height_valid": bool(message.height_valid),
            "height": float(message.current_height),
            "height_age_ms": int(message.height_age_ms),
            "command_age_ms": int(message.command_age_ms),
            "armed": bool(message.armed),
            "airborne": bool(message.airborne),
            "landed": bool(message.landed),
            "failsafe": bool(message.failsafe),
        })

    def close_log(self):
        if self.log_handle is not None:
            self.log_handle.close()
            self.log_handle = None


def build_parser():
    parser = argparse.ArgumentParser(description="D-UAV V2真实飞控无桨验收发布器")
    parser.add_argument("mode", choices=MODE_BY_NAME)
    parser.add_argument("--session", type=parse_u32)
    parser.add_argument("--new-session", action="store_true")
    parser.add_argument(
        "--session-file", default="/tmp/d_uav_v2_flight_session.txt")
    parser.add_argument("--action-id", type=int, default=0)
    parser.add_argument("--height", type=float, default=0.0, help="m")
    parser.add_argument("--vx", type=float, default=0.0, help="m/s，前为正")
    parser.add_argument("--vy", type=float, default=0.0, help="m/s，左为正")
    parser.add_argument("--vz", type=float, default=0.0, help="m/s，上为正")
    parser.add_argument("--yaw", type=float, default=0.0, help="rad/s，逆时针为正")
    parser.add_argument("--invalid", action="store_true")
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--duration", type=float, default=5.0, help="秒；0表示持续运行")
    parser.add_argument("--log-file", default="")
    parser.add_argument("--quiet", action="store_true")
    return parser


def main():
    args = build_parser().parse_args()
    if not 1.0 <= args.rate <= 50.0:
        raise SystemExit("rate必须位于1..50 Hz")
    if args.duration < 0.0:
        raise SystemExit("duration不能为负数")
    mode = MODE_BY_NAME[args.mode]
    try:
        session = load_or_create_session(args)
        validate(args, mode)
    except ValueError as error:
        raise SystemExit(str(error)) from error
    print(
        f"mode={args.mode} flight_session_id={session} (0x{session:08X}) "
        f"action_id={args.action_id} valid={not args.invalid}", flush=True)

    rclpy.init()
    node = AcceptancePublisher(args, mode, session)
    deadline = None if args.duration == 0.0 else time.monotonic() + args.duration
    try:
        while rclpy.ok() and (deadline is None or time.monotonic() < deadline):
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_log()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
