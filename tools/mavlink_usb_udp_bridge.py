"""飞控 USB 串口 <-> UDP 桥（临时工具）

用途：绕过 QGC 5.0.8 在 Windows 上"打开串口却读不到数据"的故障。
工作原理：把 COM9 的原始字节原样转发到 UDP 127.0.0.1:14550（QGC 的 UDP 监听端口），
          同时把 QGC 发回来的字节原样写回串口。不需要解析 MAVLink，MAVLink 只是字节流。

用法：
    python _mavlink_bridge.py [COM9] [115200] [127.0.0.1] [14550] [14551]

    COM9        串口名
    115200      波特率（CDC ACM 下无所谓，但要和 QGC 一致）
    127.0.0.1   目标地址（QGC 所在机器）
    14550       目标端口（QGC 的 UDP 监听端口）
    14551       本端绑定端口（QGC 会把回包发到这里）

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

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((HOST, LOCAL_PORT))
sock.settimeout(0.5)

ser = serial.Serial(PORT, BAUD, timeout=0.2)
print(f"[bridge] 串口 {PORT}@{BAUD} 已打开")
print(f"[bridge] 转发 -> udp://{HOST}:{QGC_PORT}  回包监听 udp://{HOST}:{LOCAL_PORT}")
print("[bridge] Ctrl+C 停止", flush=True)


def serial_to_udp():
    while not stop.is_set():
        try:
            data = ser.read(4096)
        except Exception as exc:  # 串口拔出等
            print(f"[bridge] 串口读取异常: {exc}", flush=True)
            stop.set()
            return
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
            data, addr = sock.recvfrom(4096)
        except socket.timeout:
            continue
        except Exception as exc:
            print(f"[bridge] UDP 接收异常: {exc}", flush=True)
            continue
        if data:
            try:
                ser.write(data)
                stats["udp2ser_bytes"] += len(data)
                stats["udp2ser_pkts"] += 1
            except Exception as exc:
                print(f"[bridge] 串口写入异常: {exc}", flush=True)
                stop.set()
                return


t1 = threading.Thread(target=serial_to_udp, daemon=True)
t2 = threading.Thread(target=udp_to_serial, daemon=True)
t1.start()
t2.start()

last = time.time()
try:
    while not stop.is_set():
        time.sleep(0.5)
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
    try:
        ser.close()
    except Exception:
        pass
    sock.close()
    print("[bridge] 已退出", flush=True)
