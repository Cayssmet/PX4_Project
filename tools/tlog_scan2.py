# -*- coding: utf-8 -*-
"""离线扫 tlog：看 QGC 到底有没有请求组件信息、飞控有没有回、电机测试命令被怎么 ACK。"""
import sys, collections
from pymavlink import mavutil

LG = sys.argv[1] if len(sys.argv) > 1 else r"C:\Users\lenovo\Documents\QGroundControl\Telemetry\2026-11-08 20-20-38.tlog"

CMD_NAMES = {
    512: "MAV_CMD_REQUEST_MESSAGE",
    310: "MAV_CMD_ACTUATOR_TEST",
    209: "MAV_CMD_DO_MOTOR_TEST",
    511: "MAV_CMD_SET_MESSAGE_INTERVAL",
    519: "MAV_CMD_REQUEST_PROTOCOL_VERSION",
    246: "MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN",
    400: "MAV_CMD_COMPONENT_ARM_DISARM",
    176: "MAV_CMD_DO_SET_MODE",
    520: "MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES",
}
MSGID_NAMES = {
    395: "COMPONENT_INFORMATION",
    397: "COMPONENT_INFORMATION_BASIC",
    148: "AUTOPILOT_VERSION",
    300: "PROTOCOL_VERSION",
    110: "FILE_TRANSFER_PROTOCOL",
    76: "COMMAND_LONG",
    77: "COMMAND_ACK",
}
RESULT = {0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED", 3: "UNSUPPORTED",
          4: "FAILED", 5: "IN_PROGRESS", 6: "CANCELLED"}

def main():
    m = mavutil.mavlink_connection(LG)
    counts = collections.Counter()
    req_component = []          # QGC 请求组件信息
    got_component = []          # 飞控回的 COMPONENT_INFORMATION
    acks = []                   # 所有 COMMAND_ACK
    actuator_test_cmds = []
    n = 0
    while True:
        msg = m.recv_match(blocking=False)
        if msg is None:
            break
        t = msg.get_type()
        if t == 'BAD_DATA':
            continue
        n += 1
        counts[t] += 1

        if t == 'COMPONENT_INFORMATION':
            got_component.append(msg)
        elif t == 'COMMAND_LONG':
            cmd = msg.command
            if cmd == 512:
                req_component.append(msg)
            if cmd in (209, 310):
                actuator_test_cmds.append(msg)
        elif t == 'COMMAND_INT':
            cmd = msg.command
            if cmd == 512:
                req_component.append(msg)
            if cmd in (209, 310):
                actuator_test_cmds.append(msg)
        elif t == 'COMMAND_ACK':
            acks.append(msg)

    print(f"文件: {LG}")
    print(f"消息总数: {n}")
    print()
    print("=" * 80)
    print("A) 是否存在组件信息相关的消息")
    print("=" * 80)
    print(f"  COMPONENT_INFORMATION(395) 收到 : {counts.get('COMPONENT_INFORMATION', 0)}")
    print(f"  COMPONENT_INFORMATION_BASIC(397) : {counts.get('COMPONENT_INFORMATION_BASIC', 0)}")
    print(f"  FILE_TRANSFER_PROTOCOL(110)      : {counts.get('FILE_TRANSFER_PROTOCOL', 0)}")
    print(f"  COMMAND_LONG(76)                 : {counts.get('COMMAND_LONG', 0)}")
    print(f"  COMMAND_ACK(77)                  : {counts.get('COMMAND_ACK', 0)}")
    print()
    print(f"  QGC 发出的 MAV_CMD_REQUEST_MESSAGE 请求: {len(req_component)} 次")
    for r in req_component[:25]:
        mid = int(r.param1) if r.param1 == r.param1 else -1
        print(f"    -> 请求 msgid={mid} ({MSGID_NAMES.get(mid,'?')})  param2={r.param2} param3={r.param3}")
    print()
    for g in got_component[:6]:
        uri = bytes(g.general_metadata_uri).split(b'\x00')[0].decode('ascii', 'replace')
        print(f"    <- COMPONENT_INFORMATION: crc={g.general_metadata_file_crc} uri='{uri}' types={g.metadata_types}")
    print()
    print("=" * 80)
    print("B) 电机/执行器测试命令与 ACK")
    print("=" * 80)
    print(f"  发出 ACTUATOR_TEST(310) 或 DO_MOTOR_TEST(209): {len(actuator_test_cmds)} 次")
    for c in actuator_test_cmds[:15]:
        print(f"    -> cmd={c.command} ({CMD_NAMES.get(c.command,'?')}) p1={c.param1} p2={c.param2} p5={c.param5} p6={c.param6}")
    print()
    print(f"  全部 COMMAND_ACK（最多 40 条）:")
    for a in acks[:40]:
        cn = CMD_NAMES.get(a.command, str(a.command))
        rn = RESULT.get(a.result, str(a.result))
        print(f"    <- ack cmd={a.command} {cn:<32} result={a.result} ({rn})")
    print()
    print("=" * 80)
    print("C) 消息种类排行（前 25）")
    print("=" * 80)
    for t, c in counts.most_common(25):
        print(f"  {t:<32} {c}")

if __name__ == '__main__':
    main()
