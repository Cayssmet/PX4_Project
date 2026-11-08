# -*- coding: utf-8 -*-
r"""
直接从飞控读取组件元数据文件，判定「执行器元数据」到底在飞控里还是在云端。

关键问题只有一个：
    /etc/extras/actuators.json.xz 到底存不存在于飞控的文件系统里？

  * 存在  -> 固件是 mftp:// 方案，QGC 理论上能自己取；它没取到就是 QGC 的 FTP 环节问题。
  * 不存在 -> 固件编的是 https://px4-travis.s3.amazonaws.com/... 方案（且没有 fallback），
              能不能显示就完全取决于那个 URL 是否 200。

用法：
    python tools\px4_metadata_probe.py COM9 115200

前置：QGC 必须完全退出（它会独占串口）。
"""
import sys
import time
import json
import lzma

from pymavlink import mavutil
from pymavlink.mavftp import MAVFTP

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM9"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

TYPE_NAMES = {0: "GENERAL", 1: "PARAMETER", 2: "COMMANDS", 3: "PERIPHERALS",
              4: "EVENTS", 5: "ACTUATORS"}

CANDIDATES = [
    "/etc/extras/actuators.json.xz",
    "/etc/extras/actuators.json",
    "/etc/extras/component_general.json.xz",
    "/etc/extras/component_general.json",
]


def cstr(raw):
    if isinstance(raw, bytes):
        return raw.split(b"\x00", 1)[0].decode("ascii", "replace")
    return str(raw).split("\x00", 1)[0]


def entry_name(e):
    for attr in ("name", "Name"):
        if hasattr(e, attr):
            return getattr(e, attr)
    return str(e)


def try_decompress(raw):
    """返回 (text, 是否压缩)。"""
    try:
        return lzma.decompress(raw).decode("utf-8", "replace"), True
    except Exception:  # noqa: BLE001
        pass
    try:
        return raw.decode("utf-8", "replace"), False
    except Exception:  # noqa: BLE001
        return None, False


def main():
    m = mavutil.mavserial(PORT, baud=BAUD, autoreconnect=False)
    m.wait_heartbeat(timeout=15)
    # PX4 的自动驾驶仪组件固定是 sysid=1 / compid=1；不要用 wait_heartbeat 拿到的 compid（可能是 0）
    m.target_system = 1
    m.target_component = 1
    print(f"已连接 {PORT}，目标 system=1 component=1")

    def hb():
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                             mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    hb()
    time.sleep(0.3)
    hb()

    ftp = MAVFTP(m, 1, 1)

    # ---------- A) 列目录 ----------
    for d in ("/etc", "/etc/extras"):
        print()
        print("=" * 78)
        print(f"A) MAVFTP 列目录 {d}")
        print("=" * 78)
        try:
            ret = ftp.cmd_list([d])
            try:
                ret.display_message()
            except Exception:  # noqa: BLE001
                print(f"  cmd_list 返回: {ret}")
            lst = getattr(ftp, "list_result", None)
            if lst:
                print(f"  共 {len(lst)} 个条目：")
                for e in lst:
                    print(f"    - {entry_name(e)}")
            else:
                print("  （list_result 为空）")
        except Exception as exc:  # noqa: BLE001
            print(f"  × cmd_list 异常: {exc}")

    # ---------- B) 逐个尝试读文件 ----------
    found = {}
    for path in CANDIDATES:
        print()
        print("=" * 78)
        print(f"B) 尝试读 {path}")
        print("=" * 78)
        try:
            raw = ftp.read(path, 400000)
        except Exception as exc:  # noqa: BLE001
            print(f"  × read 异常: {exc}")
            continue
        if not raw:
            print("  × 读不到（文件不存在或 FTP 拒绝）")
            continue
        print(f"  ✅ 读到 {len(raw)} 字节")
        text, was_xz = try_decompress(raw)
        print(f"  解压: {'xz ✅' if was_xz else '按明文'}")
        found[path] = (raw, text, was_xz)

    # ---------- C) 解析 general metadata ----------
    print()
    print("=" * 78)
    print("C) 解析 general metadata 的 metadataTypes")
    print("=" * 78)
    gen = found.get("/etc/extras/component_general.json.xz") or \
          found.get("/etc/extras/component_general.json")
    if not gen:
        print("  × 没拿到 general metadata 文件，无法解析 URI 列表。")
    else:
        text = gen[1]
        try:
            doc = json.loads(text)
        except Exception as exc:  # noqa: BLE001
            print(f"  × JSON 解析失败: {exc}")
            print(text[:600])
            doc = None
        if doc:
            print(f"  顶层键: {list(doc.keys())}")
            for k in ("version", "metadataVersion", "vendorName", "modelName"):
                if k in doc:
                    print(f"    {k} = {doc[k]}")
            types = doc.get("metadataTypes") or []
            print(f"  metadataTypes 共 {len(types)} 条：")
            for t in types:
                mt = t.get("metadataType")
                print(f"    ── type {mt} = {TYPE_NAMES.get(mt, 'UNKNOWN')}")
                print(f"         uri         = {t.get('uri')}")
                if "uriFallback" in t:
                    print(f"         uriFallback = {t.get('uriFallback')!r}")

    # ---------- D) 结论 ----------
    print()
    print("=" * 78)
    print("D) 结论")
    print("=" * 78)
    act = [p for p in ("/etc/extras/actuators.json.xz", "/etc/extras/actuators.json") if p in found]
    if act:
        print(f"  ★★ 飞控里【有】执行器元数据文件：{act[0]}（{len(found[act[0]][0])} 字节）")
        print("     ⇒ 固件是 mftp:// 方案。QGC 若仍打不开，问题在 QGC 的 MAVFTP 客户端或缓存。")
        text = found[act[0]][1]
        if text:
            try:
                d = json.loads(text)
                print(f"     文件顶层键: {list(d.keys())}")
            except Exception:  # noqa: BLE001
                pass
    else:
        print("  ★★ 飞控里【没有】执行器元数据文件。")
        print("     ⇒ 固件编的是 https://px4-travis.s3.amazonaws.com/... 方案，")
        print("        能不能显示完全取决于那个 URL 是否 200；若 404，QGC 侧无法修。")


if __name__ == "__main__":
    main()
