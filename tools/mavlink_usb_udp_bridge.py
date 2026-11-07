"""飞控 USB 串口 <-> UDP 桥

用途：绕过 QGC 5.0.8 在 Windows 上"打开串口却读不到数据"的故障。
工作原理：把飞控串口的原始字节原样转发到 UDP 127.0.0.1:14550（QGC 的 UDP 监听端口），
          同时把 QGC 发回来的字节原样写回串口。不需要解析 MAVLink —— MAVLink 只是字节流。

特性：
  * 启动时串口不在（飞控没插）不会崩，会一直等待出现
  * 飞控中途被拔掉 / 换口 / 重新枚举，会自动重连，不用重启脚本
  * 每 5 秒打印一次双向流量，可据此判断链路是否真的在通

用法：
    python mavlink_usb_udp_bridge.py [COM9] [115200] [127.0.0.1] [14550] [14551]

    COM9        串口名
    115200      波特率（CDC ACM 下通常无所谓，但写出来便于肉眼核对）
    127.0.0.1   目标地址（QGC 所在机器）
    14550       目标端口（QGC 的 UDP 监听端口）
    14551       本端绑定端口（QGC 会把回包发到这里）

依赖：
    python -m pip install --target <本脚本所在目录>/.pylibs pyserial
    set PYTHONPATH=<本脚本所在目录>/.pylibs

停止：Ctrl+C
"""

import socket
import sys
import threading
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM9"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
HOST = sys.argv[3] if len(sys.argv) > 3 else "127.0.0.1"
QGC_PORT = int(sys.argv[4]) if len(sys.argv) > 4 else 14550
LOCAL_PORT = int(sys.argv[5]) if len(sys.argv) > 5 else 14551

stats = {"ser2udp_bytes": 0, "ser2udp_pkts": 0, "udp2ser_bytes": 0, "udp2ser_pkts": 0}
stop = threading.Event()

ser_lock = threading.Lock()
ser = None
serial_ready = False  # 只在首次打开时打印一次总览


def open_serial():
    """尝试打开串口；成功返回 True。"""
    global ser, serial_ready
    try:
        s = serial.Serial(PORT, BAUD, timeout=0.2)
    except Exception:
        return False
    with ser_lock:
        ser = s
    if not serial_ready:
        serial_ready = True
        print(f"[bridge] 串口 {PORT}@{BAUD} 已打开", flush=True)
        print(f"[bridge] 转发 -> udp://{HOST}:{QGC_PORT}   回包监听 udp://{HOST}:{LOCAL_PORT}", flush=True)
        print("[bridge] Ctrl+C 停止", flush=True)
    else:
        print(f"[bridge] 串口 {PORT} 已重新连上", flush=True)
    return True


def drop_serial(reason):
    """关闭并丢弃当前串口句柄，之后由主循环重连。"""
    global ser
    with ser_lock:
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
            ser = None
    print(f"[bridge] 串口断开（{reason}），等待重新出现…", flush=True)


def current_serial():
    with ser_lock:
        return ser


def serial_to_udp():
    while not stop.is_set():
        s = current_serial()
        if s is None:
            time.sleep(0.2)
            continue
        try:
            data = s.read(4096)
        except Exception as exc:
            drop_serial(f"读取异常 {exc}")
            continue
        if data:
            try:
                sock.sendto(data, (HOST, QGC_PORT))
                stats["ser2udp_bytes"] += len(data)
                stats["ser2udp_pkts"] += 1
            except Exception as exc:
                print(f"[bridge] UDP 发送异常: {exc}", flush=True)


def udp_to_serial():
    while not stop.is_set():
        try:
            data, _addr = sock.recvfrom(4096)
        except socket.timeout:
            continue
        except Exception as exc:
            print(f"[bridge] UDP 接收异常: {exc}", flush=True)
            continue
        if not data:
            continue
        s = current_serial()
        if s is None:
            continue  # 串口还没好，丢掉这段时间的 UDP 包
        try:
            s.write(data)
            stats["udp2ser_bytes"] += len(data)
            stats["udp2ser_pkts"] += 1
        except Exception as exc:
            drop_serial(f"写入异常 {exc}")


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((HOST, LOCAL_PORT))
sock.settimeout(0.5)

print(f"[bridge] 等待串口 {PORT} 出现（飞控未插入时会一直等）…", flush=True)
while not stop.is_set() and not open_serial():
    time.sleep(1.0)

t1 = threading.Thread(target=serial_to_udp, daemon=True)
t2 = threading.Thread(target=udp_to_serial, daemon=True)
t1.start()
t2.start()

last = time.time()
last_wait_log = 0.0
try:
    while not stop.is_set():
        time.sleep(0.5)
        # 串口掉线时持续重连，但日志最多每 10 秒一条，避免刷屏
        if current_serial() is None:
            if not open_serial():
                if time.time() - last_wait_log >= 10.0:
                    last_wait_log = time.time()
                    print(f"[bridge] 仍在等待 {PORT} …（检查飞控是否插好、是否换了 USB 口）", flush=True)
            continue
        if time.time() - last >= 5.0:
            last = time.time()
            print(
                f"[bridge] 飞控->QGC {stats['ser2udp_bytes']:>9} B "
                f"({stats['ser2udp_pkts']} 包/5s) | "
                f"QGC->飞控 {stats['udp2ser_bytes']:>7} B "
                f"({stats['udp2ser_pkts']} 包/5s)",
                flush=True,
            )
            stats["ser2udp_bytes"] = stats["ser2udp_pkts"] = 0
            stats["udp2ser_bytes"] = stats["udp2ser_pkts"] = 0
except KeyboardInterrupt:
    pass
finally:
    stop.set()
    with ser_lock:
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
    sock.close()
    print("[bridge] 已退出", flush=True)
