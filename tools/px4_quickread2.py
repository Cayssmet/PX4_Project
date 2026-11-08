# -*- coding: utf-8 -*-
"""第二次读取：固件版本、传感器校准状态、航向稳定性。"""
import sys, time, struct, statistics
from pymavlink import mavutil

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM9"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

def pname(raw):
    if isinstance(raw, bytes):
        return raw.split(b'\x00', 1)[0].decode('ascii', 'replace')
    return str(raw).split('\x00', 1)[0]

def decode_ver(v):
    import math
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return "NaN"
    i = struct.unpack('<i', struct.pack('<f', float(v)))[0] if abs(v) < 1e-30 or (isinstance(v,float) and v!=v) else int(v)
    major = (i >> 24) & 0xFF
    minor = (i >> 16) & 0xFF
    patch = (i >> 8) & 0xFF
    ftype = i & 0xFF
    return f"{major}.{minor}.{patch} (type={ftype})"

def main():
    m = mavutil.mavserial(PORT, baud=BAUD, autoreconnect=False)
    m.wait_heartbeat(timeout=10)
    print(f"已连接 {PORT}\n")

    def hb():
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                             mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    for _ in range(3):
        hb(); time.sleep(0.1)

    # ---- AUTOPILOT_VERSION ----
    print("=" * 78)
    print("A) 固件版本")
    print("=" * 78)
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
                            148, 0, 0, 0, 0, 0, 0)
    ver = None
    t_end = time.time() + 5
    while time.time() < t_end:
        hb()
        msg = m.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=0.5)
        if msg:
            ver = msg
            break
    if ver:
        print(f"  flight_sw_version    = raw {ver.flight_sw_version}  ->  {decode_ver(ver.flight_sw_version)}")
        print(f"  middleware_sw_version= raw {ver.middleware_sw_version}")
        print(f"  os_sw_version        = raw {ver.os_sw_version}")
        print(f"  board_version        = raw {ver.board_version}")
        print(f"  vendor_id / product_id = {ver.vendor_id} / {ver.product_id}")
        fv = ver.flight_custom_version
        try:
            txt = bytes([b for b in fv if b != 0]).decode('ascii', 'replace')
            print(f"  flight_custom_version= {txt}")
        except Exception:
            pass
    else:
        print("  （没收到 AUTOPILOT_VERSION）")
    print()

    # ---- 传感器在位 ----
    print("=" * 78)
    print("B) SYS_STATUS —— 传感器是否在位")
    print("=" * 78)
    ss = m.recv_match(type='SYS_STATUS', blocking=True, timeout=3)
    if ss:
        BITS = {0: 'GYRO', 1: 'ACCEL', 2: 'MAG', 3: 'ABSOLUTE_PRESSURE', 4: 'DIFFERENTIAL_PRESSURE',
                5: 'GPS', 6: 'OPTICAL_FLOW', 7: 'VISION_POSITION', 8: 'LASER_POSITION',
                9: 'EXTERNAL_GROUND_TRUTH', 10: 'ANGULAR_RATE_CONTROL', 11: 'ATTITUDE_STABILIZATION',
                12: 'YAW_POSITION', 13: 'Z_ALTITUDE_CONTROL', 14: 'XY_POSITION_CONTROL',
                15: 'MOTOR_OUTPUTS', 16: 'RC_RECEIVER', 17: 'GYRO2', 18: 'ACCEL2', 19: 'MAG2',
                20: 'GEOFENCE', 21: 'AHRS', 22: 'TERRAIN', 23: 'REVERSE_MOTOR',
                24: 'OPTICAL_FLOW_SUBSYSTEM', 25: 'UNKNOWN'}
        pres = ss.onboard_control_sensors_present
        enab = ss.onboard_control_sensors_enabled
        heal = ss.onboard_control_sensors_health
        print(f"  电压: {ss.voltage_battery} mV   电流: {ss.current_battery} cA   电池余量: {ss.battery_remaining}%")
        print(f"  {'传感器':<28}{'在位':>6}{'启用':>6}{'健康':>6}")
        for b, n in BITS.items():
            p = bool(pres & (1 << b)); e = bool(enab & (1 << b)); h = bool(heal & (1 << b))
            if p or e or h:
                print(f"  {n:<28}{str(p):>6}{str(e):>6}{str(h):>6}")
    else:
        print("  （没收到 SYS_STATUS）")
    print()

    # ---- 航向稳定性 ----
    print("=" * 78)
    print("C) 航向稳定性（8 秒采样 ATTITUDE.yaw，看漂不漂）")
    print("=" * 78)
    yaws, pitches, rolls = [], [], []
    t_end = time.time() + 8
    while time.time() < t_end:
        hb()
        msg = m.recv_match(type='ATTITUDE', blocking=True, timeout=0.5)
        if msg:
            yaws.append(msg.yaw * 57.2958)
            pitches.append(msg.pitch * 57.2958)
            rolls.append(msg.roll * 57.2958)
    if len(yaws) > 5:
        print(f"  采样 {len(yaws)} 个")
        print(f"  yaw  : min={min(yaws):.2f}°  max={max(yaws):.2f}°  峰峰差={max(yaws)-min(yaws):.3f}°  标准差={statistics.pstdev(yaws):.3f}°")
        print(f"  pitch: min={min(pitches):.2f}°  max={max(pitches):.2f}°  峰峰差={max(pitches)-min(pitches):.3f}°")
        print(f"  roll : min={min(rolls):.2f}°  max={max(rolls):.2f}°  峰峰差={max(rolls)-min(rolls):.3f}°")
        print(f"  首尾 yaw 漂移 = {yaws[-1]-yaws[0]:+.3f}°")
        print()
        print("  判读：峰峰差 < 0.5° 说明航向很稳；> 2° 说明磁罗盘在抖（干扰/未校准）。")
    else:
        print("  采样太少")
    print()

    # ---- 校准状态 ----
    print("=" * 78)
    print("D) 校准状态（ID 为 0 就表示该传感器没校准过）")
    print("=" * 78)
    m.param_fetch_all()
    deadline = time.time() + 25
    last_new = time.time()
    while time.time() < deadline:
        msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=1.0)
        if msg is None:
            if time.time() - last_new > 3.0:
                break
            continue
        m.params[pname(msg.param_id)] = msg.param_value
        last_new = time.time()
    print(f"  共 {len(m.params)} 个参数\n")

    def show(prefix):
        keys = sorted(k for k in m.params if k.startswith(prefix))
        if not keys:
            print(f"    {prefix}* : 一个都没有")
            return
        for k in keys:
            v = m.params[k]
            if v != v:  # NaN
                print(f"    {k:<26} = NaN  (在 PX4 里通常意味着 int32 的 -1)")
            elif isinstance(v, float) and abs(v) < 1e-30:
                iv = struct.unpack('<i', struct.pack('<f', v))[0]
                print(f"    {k:<26} = {iv}  (还原自 denormal)")
            else:
                print(f"    {k:<26} = {v:.6g}")

    for p in ('CAL_MAG0_', 'CAL_MAG1_', 'CAL_ACC0_', 'CAL_GYRO0_', 'CAL_ACC1_', 'CAL_GYRO1_',
              'CAL_MAG_SIDES', 'CAL_MAG_ROT_AUTOSET', 'SENS_BOARD_', 'SENS_IMU_', 'SENS_MAG_'):
        print(f"  ---- {p}* ----")
        show(p)
        print()

    print("=" * 78)
    print("E) 关键项汇总")
    print("=" * 78)
    for k in ('COM_RC_OVERRIDE', 'EKF2_MAG_TYPE', 'EKF2_HGT_REF', 'EKF2_OF_CTRL', 'EKF2_RNG_CTRL',
              'SYS_AUTOSTART', 'CA_AIRFRAME', 'CA_METHOD', 'CA_ROTOR_COUNT',
              'MPC_THR_HOVER', 'MPC_THR_MIN', 'MPC_MANTHR_MIN', 'MAV_PROTO_VER'):
        if k in m.params:
            v = m.params[k]
            if v != v:
                print(f"  {k:<26} = NaN(int -1)")
            elif isinstance(v, float) and abs(v) < 1e-30:
                print(f"  {k:<26} = {struct.unpack('<i', struct.pack('<f', v))[0]}")
            else:
                print(f"  {k:<26} = {v:.6g}")
    print("  提示：查某个参数是否存在 -> 看 _params_all.txt")


if __name__ == '__main__':
    main()
