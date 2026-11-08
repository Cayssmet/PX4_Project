# -*- coding: utf-8 -*-
"""
px4_quickread.py —— 一次性读取 PX4 飞控的关键状态，用于远程排查。

用途：
  QGC 的某个界面用不了、或者想知道"现在到底是不是解锁状态""某个参数到底存在不存在"时，
  不依赖 QGC，直接独占串口读一遍。

用法：
  python -u px4_quickread.py COM9            # 默认 115200
  python -u px4_quickread.py COM9 115200

依赖（装在 tools/.pylibs）：
  pip install --target tools/.pylibs pymavlink pyserial

输出：
  1) 心跳 / 解锁状态 / 飞行模式 / 落地状态
  2) 飞控自己打的 STATUSTEXT（含 Preflight Fail 之类的拒绝解锁原因）
  3) EKF_STATUS_REPORT（判断"heading estimate not stable"的直接依据）
  4) 指定前缀的参数清单（执行器配置、EKF、控制相关）
  5) 全部参数名写入 _params_all.txt，便于核对某个参数是否存在

注意：运行前必须先完全退出 QGroundControl，否则串口被它独占。
"""
import sys
import time
import struct

from pymavlink import mavutil

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM9"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

# ---------- 工具 ----------

def denorm(v):
    """PX4 把 INT32 参数按 float 位模式发送（denormal），这里还原成整数。"""
    try:
        if v != 0 and abs(v) < 1e-30:
            return struct.unpack('<i', struct.pack('<f', v))[0]
    except Exception:
        pass
    return v


def pname(raw):
    if isinstance(raw, bytes):
        return raw.split(b'\x00', 1)[0].decode('ascii', 'replace')
    return str(raw).split('\x00', 1)[0]


def connect():
    last = None
    for attempt in range(1, 7):
        try:
            m = mavutil.mavserial(PORT, baud=BAUD, autoreconnect=False)
            m.wait_heartbeat(timeout=10)
            return m
        except Exception as e:
            last = e
            print(f"  第 {attempt} 次连接失败：{e}")
            time.sleep(2)
    raise SystemExit(f"连不上 {PORT}：{last}（多半是 QGroundControl 还占着串口）")


def main():
    print(f"===== 连接 {PORT} @ {BAUD} =====")
    m = connect()
    print("  连上了\n")

    # 先做一次 GCS 握手，让飞控把状态流发过来
    def hb():
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                             mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    for _ in range(3):
        hb()
        time.sleep(0.1)

    def req_interval(msg_id, us):
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            msg_id, us, 0, 0, 0, 0, 0)

    # ---------- 1) 心跳 / 解锁状态 ----------
    print("=" * 78)
    print("1) 心跳 / 解锁状态 / 飞行模式 / 落地状态")
    print("=" * 78)
    hbeats, ext, atts = [], [], []
    t_end = time.time() + 4.0
    while time.time() < t_end:
        hb()
        msg = m.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        t = msg.get_type()
        if t == 'HEARTBEAT':
            hbeats.append(msg)
        elif t == 'EXTENDED_SYS_STATE':
            ext.append(msg)
        elif t == 'ATTITUDE':
            atts.append(msg)

    if hbeats:
        h = hbeats[-1]
        armed = bool(h.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        print(f"  飞控类型   : {h.type}")
        print(f"  autopilot  : {h.autopilot}")
        print(f"  base_mode  : 0x{h.base_mode:02X}")
        print(f"  custom_mode: {h.custom_mode}")
        print(f"  >>> 解锁状态: {'【已解锁 ARMED】' if armed else '【未解锁 DISARMED】'}")
    else:
        print("  !! 没收到 HEARTBEAT")

    if ext:
        e = ext[-1]
        names = {0: 'UNDEFINED', 1: 'ON_GROUND', 2: 'IN_AIR', 3: 'TAKEOFF',
                 4: 'LANDING', 5: 'IN_AIR(alt ok)'}
        print(f"  落地状态   : landed_state={e.landed_state} ({names.get(e.landed_state, '?')})")
    else:
        print("  落地状态   : （没收到 EXTENDED_SYS_STATE）")

    if atts:
        a = atts[-1]
        print(f"  当前姿态   : roll={a.roll*57.2958:+.2f}° "
              f"pitch={a.pitch*57.2958:+.2f}° yaw={a.yaw*57.2958:+.1f}°")
    print()

    # ---------- 2) STATUSTEXT ----------
    print("=" * 78)
    print("2) 飞控自己打的 STATUSTEXT（拒绝解锁的原因就在这里面）")
    print("=" * 78)
    req_interval(253, 200000)  # STATUSTEXT 5 Hz
    seen = {}
    t_end = time.time() + 5.0
    while time.time() < t_end:
        hb()
        msg = m.recv_match(type='STATUSTEXT', blocking=True, timeout=0.5)
        if msg is None:
            continue
        txt = msg.text if isinstance(msg.text, str) else msg.text.decode('utf-8', 'replace')
        txt = txt.strip()
        if txt:
            seen[txt] = seen.get(txt, 0) + 1
    if seen:
        for txt, c in seen.items():
            print(f"  x{c:<3} [{txt}]")
    else:
        print("  （这段时间内飞控没有主动打新消息）")
        print("  提示：试着在 QGC 里点一次 Arm，再回来看；或者直接看下面第 3 节的 EKF 数据。")
    print()

    # ---------- 3) EKF_STATUS_REPORT ----------
    print("=" * 78)
    print("3) EKF_STATUS_REPORT —— 判断 heading estimate not stable 的直接依据")
    print("=" * 78)
    req_interval(193, 200000)  # EKF_STATUS_REPORT 5 Hz
    FLAG = {0: 'ATTITUDE', 1: 'HORIZ_VEL', 2: 'VERT_VEL', 3: 'HORIZ_POS_ABS',
            4: 'VERT_POS_ABS', 5: 'TERRAIN_ALT', 6: 'CONST_POS_MODE'}
    got = 0
    t_end = time.time() + 5.0
    while time.time() < t_end and got < 6:
        hb()
        msg = m.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=0.5)
        if msg is None:
            continue
        got += 1
        flags = [n for b, n in FLAG.items() if msg.flags & (1 << b)]
        print(f"  flags=0x{msg.flags:04X} [{' '.join(flags)}]")
        print(f"    velocity_variance     = {msg.velocity_variance:.6f}")
        print(f"    pos_horiz_variance    = {msg.pos_horiz_variance:.6f}")
        print(f"    pos_vert_variance     = {msg.pos_vert_variance:.6f}")
        print(f"    compass_variance      = {msg.compass_variance:.6f}   <-- 航向方差，大就是 not stable")
        print(f"    terrain_alt_variance  = {msg.terrain_alt_variance:.6f}")
    if got == 0:
        print("  （没收到 EKF_STATUS_REPORT，该版本可能不主动发；可用 listener estimator_status_flags 代替）")
    print()

    # ---------- 4) 参数 ----------
    print("=" * 78)
    print("4) 参数（全量拉取后按前缀筛选）")
    print("=" * 78)
    m.param_fetch_all()
    deadline = time.time() + 25.0
    last_new = time.time()
    while time.time() < deadline:
        msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=1.0)
        if msg is None:
            if time.time() - last_new > 3.0:
                break
            continue
        n = pname(msg.param_id)
        m.params[n] = msg.param_value
        last_new = time.time()
    print(f"  共收到 {len(m.params)} 个参数\n")

    # 全部参数名落盘，便于核对"某个参数到底存不存在"
    try:
        with open("_params_all.txt", "w", encoding="utf-8") as f:
            for k in sorted(m.params):
                f.write(f"{k}={m.params[k]}\n")
        print("  全部参数已写入 _params_all.txt\n")
    except Exception as e:
        print(f"  写 _params_all.txt 失败：{e}\n")

    GROUPS = [
        ("执行器 / 电机配置", ('ACTUATOR_', 'CA_', 'PWM_', 'MOT_', 'SYS_AUTOSTART', 'SERVO_')),
        ("EKF2 高度与航向", ('EKF2_HGT_REF', 'EKF2_OF_CTRL', 'EKF2_RNG_CTRL', 'EKF2_EV_CTRL',
                             'EKF2_MAG_TYPE', 'EKF2_GPS_CTRL', 'EKF2_BARO_CTRL', 'EKF2_YAW_')),
        ("解锁与安全", ('COM_ARM', 'CBRK_', 'COM_RC_', 'NAV_DLL_ACT', 'COM_DISARM')),
        ("遥控 / 校准", ('CAL_MAG0_ROT', 'CAL_ACC0_ROT', 'CAL_GYRO0_ROT', 'SENS_BOARD_ROT',
                        'SENS_BOARD_X_OFF', 'RC_MAP_', 'RC1_', 'RC3_')),
        ("链路", ('MAV_PROTO_VER', 'SER_TEL', 'MAV_1_', 'MAV_0_')),
    ]
    for title, prefixes in GROUPS:
        print(f"  ---- {title} ----")
        keys = sorted(k for k in m.params if any(k.startswith(p) for p in prefixes))
        if not keys:
            print("    （一个都没有！）")
        for k in keys:
            v = denorm(m.params[k])
            if isinstance(v, float):
                print(f"    {k:<28} = {v:.6g}")
            else:
                print(f"    {k:<28} = {v}")
        print()

    print("=" * 78)
    print("读取完毕")
    print("=" * 78)


if __name__ == '__main__':
    main()
