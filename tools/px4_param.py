#!/usr/bin/env python3
"""
px4_param.py -- 直接通过 MAVLink 串口读取/写入 PX4 参数，不依赖 QGC。

存在理由
--------
QGC 5.0.8 在这台 Windows 上有一个反复出现的故障：打开串口后占着不读数据
（表现：进程在、端口打不开、但没有新 tlog、ParamCache 不刷新）。
本脚本绕过 QGC 直接讲 MAVLink，用于「只想改几个参数」的场合。

用法
----
    set PYTHONPATH=D:\\09_PX4\\PX4_Project\\tools\\.pylibs
    python px4_param.py COM9 --get UXRCE_DDS_CFG,SER_TEL3_BAUD
    python px4_param.py COM9 --set UXRCE_DDS_CFG=103 --set UXRCE_DDS_SYNCT=1
    python px4_param.py COM9 --set UXRCE_DDS_CFG=103 --reboot
    python px4_param.py COM9 --list            # 打印全部参数名

★ 关键坑：PX4 把 INT32 参数的**位模式**当 float 发出去（denormal）
  例如 103 会显示成 1.4426950408889634e-43，57600 显示成 8.07148e-41。
  所以读取时要 struct.unpack('<i', pack('<f', raw))，写入时要反向打包。
  本脚本自动按 param_type 判断，不需要手工处理。

★ 本脚本会独占串口。运行前必须关掉 QGC 和任何桥接进程，否则报
  PermissionError(13, '拒绝访问。')。
"""

import argparse
import struct
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("缺少 pymavlink。先运行：\n"
             "  python -m pip install --target D:\\09_PX4\\PX4_Project\\tools\\.pylibs pymavlink\n"
             "并设置 PYTHONPATH=D:\\09_PX4\\PX4_Project\\tools\\.pylibs")

# MAV_PARAM_TYPE
PARAM_TYPE_UINT8 = 1
PARAM_TYPE_INT8 = 2
PARAM_TYPE_UINT16 = 3
PARAM_TYPE_INT16 = 4
PARAM_TYPE_UINT32 = 5
PARAM_TYPE_INT32 = 6
PARAM_TYPE_REAL32 = 9
INT_TYPES = {PARAM_TYPE_UINT8, PARAM_TYPE_INT8, PARAM_TYPE_UINT16,
             PARAM_TYPE_INT16, PARAM_TYPE_UINT32, PARAM_TYPE_INT32}


def float_to_int32(raw: float) -> int:
    """PX4 把 int32 的位模式当 float 发出，这里还原。"""
    return struct.unpack('<i', struct.pack('<f', raw))[0]


def int32_to_float(value: int) -> float:
    """写入时反向：把 int32 的位模式打包成 float 交给 PX4。"""
    return struct.unpack('<f', struct.pack('<i', value))[0]


def looks_denormal(raw: float) -> bool:
    """|x| < 1e-30 且非零，基本可以断定是 int32 位模式。"""
    return raw != 0.0 and abs(raw) < 1e-30


def decode(name: str, raw: float, ptype: int) -> str:
    if ptype in INT_TYPES or looks_denormal(raw):
        return f"{name} = {float_to_int32(raw)}  (int32, type={ptype})"
    return f"{name} = {raw:g}  (float, type={ptype})"


class Px4Param:
    def __init__(self, port: str, baud: int = 115200, timeout: float = 20.0):
        # mavutil.mavlink_connection('COM9') 会把 COM9 当日志文件去 open()，
        # 必须显式用 mavserial。
        self.m = mavutil.mavserial(port, baud=baud, autoreconnect=False)
        self.m.port.flushInput()
        print(f"[连接] {port} @ {baud}，等待飞控心跳 ...")
        hb = self.m.wait_heartbeat(timeout=int(timeout))
        if hb is None:
            sys.exit("[失败] 20 秒内没收到 HEARTBEAT。检查：飞控是否上电 / COM 口是否正确 / QGC 是否占着口。")
        self.ts = self.m.target_system
        self.tc = self.m.target_component
        print(f"[连接] 成功  system={self.ts} component={self.tc}")
        self._send_gcs_heartbeat()

    # ---------------- 底层 ----------------
    def _send_gcs_heartbeat(self):
        self.m.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def get(self, name: str, timeout: float = 4.0):
        self.m.mav.param_request_read_send(
            self.ts, self.tc, name.encode('utf-8'), -1)
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.m.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
            if msg is None:
                self._send_gcs_heartbeat()
                continue
            got = msg.param_id
            if isinstance(got, bytes):
                got = got.decode('utf-8', 'ignore')
            if got.split('\x00')[0] == name:
                return msg.param_value, msg.param_type
        return None

    def set(self, name: str, value: float, timeout: float = 4.0) -> bool:
        cur = self.get(name)
        if cur is None:
            print(f"[跳过] {name}: 该固件没有这个参数")
            return False
        raw, ptype = cur
        is_int = ptype in INT_TYPES or looks_denormal(raw)

        if is_int:
            send_val = int32_to_float(int(round(float(value))))
        else:
            send_val = float(value)

        send_type = PARAM_TYPE_INT32 if is_int else PARAM_TYPE_REAL32
        self.m.mav.param_set_send(
            self.ts, self.tc, name.encode('utf-8'), send_val, send_type)

        # 等回执并校验
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.m.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
            if msg is None:
                self._send_gcs_heartbeat()
                continue
            got = msg.param_id
            if isinstance(got, bytes):
                got = got.decode('utf-8', 'ignore')
            if got.split('\x00')[0] != name:
                continue
            now = msg.param_value
            ok = (float_to_int32(now) == int(round(float(value)))) if is_int \
                else abs(now - float(value)) < 1e-6
            print(("[成功] " if ok else "[失败] ") + decode(name, now, msg.param_type))
            return ok
        print(f"[超时] {name}: 发出去但没收到回执")
        return False

    def list_all(self, timeout: float = 30.0):
        self.m.mav.param_request_list_send(self.ts, self.tc)
        seen = {}
        expected = None
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.m.recv_match(type='PARAM_VALUE', blocking=True, timeout=1.0)
            if msg is None:
                if expected is not None and len(seen) >= expected:
                    break
                self._send_gcs_heartbeat()
                continue
            expected = msg.param_count
            n = msg.param_id
            if isinstance(n, bytes):
                n = n.decode('utf-8', 'ignore')
            seen[n.split('\x00')[0]] = (msg.param_value, msg.param_type)
            if expected is not None and len(seen) >= expected:
                break
        return seen

    def save(self, timeout: float = 2.5):
        """把 RAM 里的参数写进 flash。

        ★ 这一步不可省：PX4 的 PARAM_SET 只改 RAM，重启就丢。
          QGC 每次改参数都会替用户发 MAV_CMD_PREFLIGHT_STORAGE(param1=1)，
          自己用脚本时必须手工发，否则会以为改成功了、一重启全回去了。
        """
        print("[动作] 保存参数到 flash (MAV_CMD_PREFLIGHT_STORAGE, param1=1) ...")
        for _ in range(3):
            self.m.mav.command_long_send(
                self.ts, self.tc,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE, 0,
                1, 0, 0, 0, 0, 0, 0)
            time.sleep(0.4)
        time.sleep(timeout)
        print("[动作] 保存命令已发")

    def reboot(self, timeout: float = 3.0):
        print("[动作] 发送重启命令 (MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, param1=1) ...")
        for _ in range(5):
            self.m.mav.command_long_send(
                self.ts, self.tc,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
                1, 0, 0, 0, 0, 0, 0)
            time.sleep(0.3)
        time.sleep(timeout)
        print("[动作] 重启命令已发（USB 会短暂断开，属正常）")

    def close(self):
        try:
            self.m.close()
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser(description="直接通过 MAVLink 读写 PX4 参数（绕过 QGC）")
    ap.add_argument("port", help="串口，例如 COM9")
    ap.add_argument("--baud", type=int, default=115200, help="波特率，默认 115200")
    ap.add_argument("--get", help="读取参数，逗号分隔，例如 A,B,C")
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                    help="写入参数，可重复，例如 --set UXRCE_DDS_CFG=103")
    ap.add_argument("--save", action="store_true", default=True,
                    help="写入后保存到 flash（默认开启，不可省）")
    ap.add_argument("--no-save", dest="save", action="store_false",
                    help="只改 RAM 不保存（调试用，重启即失效）")
    ap.add_argument("--reboot", action="store_true", help="写完重启飞控")
    ap.add_argument("--list", action="store_true", help="拉取全部参数名")
    ap.add_argument("--show", default=None, help="拉取全部参数后只显示这些（逗号分隔子串）")
    args = ap.parse_args()

    px = Px4Param(args.port, args.baud)
    try:
        if args.list or args.show:
            params = px.list_all()
            print(f"[完成] 共收到 {len(params)} 个参数")
            if args.show:
                keys = [k.strip() for k in args.show.split(',')]
                for name in sorted(params):
                    if any(k.lower() in name.lower() for k in keys):
                        raw, pt = params[name]
                        print("  " + decode(name, raw, pt))
        if args.get:
            for name in [n.strip() for n in args.get.split(',') if n.strip()]:
                cur = px.get(name)
                if cur is None:
                    print(f"[缺失] {name}")
                else:
                    print("  " + decode(name, cur[0], cur[1]))
        for item in args.set:
            if '=' not in item:
                sys.exit(f"--set 格式错误：{item}（应为 NAME=VALUE）")
            name, val = item.split('=', 1)
            px.set(name.strip(), float(val.strip()))
        if args.set and args.save:
            px.save()
        if args.reboot:
            px.reboot()
    finally:
        px.close()


if __name__ == "__main__":
    main()
