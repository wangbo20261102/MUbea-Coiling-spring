import ctypes
import math
import re
import socket
import time
from datetime import datetime
from pathlib import Path

from rtde_receive import RTDEReceiveInterface


ROBOT_IP = "192.168.1.10"
DASHBOARD_PORT = 29999
CHECK_INTERVAL = 0.02  # 秒
MIN_Z_CM = 0.0

TIME_FORMAT = "%Y-%m-%d %H:%M:%S"
NUMBER = r"[+-]?\d+(?:\.\d+)?"
RECORD = re.compile(
    r"^保存时间：[ \t]*(?P<saved_at>\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2})"
    r"[ \t]*（北京时间，UTC\+8）[ \t]*\n"
    rf"X[ \t]*=[ \t]*(?P<x>{NUMBER})[ \t]*cm[ \t]*\n"
    rf"Y[ \t]*=[ \t]*(?P<y>{NUMBER})[ \t]*cm[ \t]*\n"
    rf"Z[ \t]*=[ \t]*(?P<z>{NUMBER})[ \t]*cm[ \t]*$",
    re.MULTILINE,
)


def cross(a, b, p):
    """计算向量 AB 和 AP 的二维叉积，用于判断点在边的哪一侧。"""
    return (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])


def load_boundary():
    """读取四条记录，按时间连接 XY 顶点，并验证四边形。"""
    desktop = ctypes.create_unicode_buffer(260)
    if ctypes.windll.shell32.SHGetFolderPathW(None, 0x10, None, 0, desktop):
        raise OSError("无法获取 Windows 桌面路径。")

    folder = Path(desktop.value) / "坐标"
    records = []
    for path in sorted(folder.glob("*.txt")):
        content = path.read_text(encoding="utf-8-sig")
        matches = list(RECORD.finditer(content))
        if not matches or len(matches) != content.count("保存时间："):
            raise ValueError(f"{path.name} 中有记录不符合坐标文件格式。")

        for match in matches:
            saved_at = datetime.strptime(match["saved_at"], TIME_FORMAT)
            xy = (float(match["x"]), float(match["y"]))
            if not all(math.isfinite(value) for value in xy):
                raise ValueError(f"{path.name} 中有无效坐标。")
            records.append((saved_at, xy))

    if len(records) != 4 or len({stamp for stamp, _ in records}) != 4:
        raise ValueError(f"{folder} 中必须有四条保存时间各不相同的坐标记录。")

    records.sort(key=lambda record: record[0])
    vertices = [xy for _, xy in records]
    turns = [
        cross(vertices[i], vertices[(i + 1) % 4], vertices[(i + 2) % 4])
        for i in range(4)
    ]
    if all(turn > 0 for turn in turns):
        direction = 1
    elif all(turn < 0 for turn in turns):
        direction = -1
    else:
        raise ValueError("按时间连接的四个点必须组成凸四边形。")

    return vertices, direction


def check_tcp(pose, vertices, direction):
    """将实际 TCP 坐标转为厘米，检查 Z 下限和四条 XY 边。"""
    if len(pose) != 6 or not all(math.isfinite(value) for value in pose):
        raise RuntimeError("未收到有效的 TCP 位姿。")

    x, y, z = (value * 100.0 for value in pose[:3])
    location = f"X={x:+.2f} cm，Y={y:+.2f} cm，Z={z:+.2f} cm"

    if z < MIN_Z_CM:
        raise RuntimeError(f"TCP 低于底座 Z 下限。{location}")

    for i, vertex in enumerate(vertices):
        next_vertex = vertices[(i + 1) % 4]
        if direction * cross(vertex, next_vertex, (x, y)) < -1e-9:
            raise RuntimeError(f"TCP 越过第 {i + 1} 条 XY 边界。{location}")


def monitor():
    """循环读取 TCP。发现越界或 RTDE 读取失败时，请求停止程序。"""
    vertices, direction = load_boundary()
    print(f"底座 XY 边界（cm）：{vertices}")
    print(f"底座 Z 下限：{MIN_Z_CM:.2f} cm")

    with socket.create_connection((ROBOT_IP, DASHBOARD_PORT), timeout=2.0) as dashboard:
        with dashboard.makefile("rb") as replies:
            greeting = replies.readline().decode("utf-8", errors="replace").strip()
            if "Dashboard Server" not in greeting:
                raise RuntimeError(f"Dashboard 握手失败：{greeting!r}")

            receiver = RTDEReceiveInterface(ROBOT_IP)
            try:
                print("正在监控实际 TCP 坐标。")
                while True:
                    try:
                        if not receiver.isConnected():
                            raise RuntimeError("RTDE 连接已断开。")
                        check_tcp(receiver.getActualTCPPose(), vertices, direction)
                    except Exception as fault:
                        print(f"触发停止：{fault}", flush=True)
                        try:
                            dashboard.sendall(b"stop\n")
                            reply = replies.readline().decode(
                                "utf-8", errors="replace"
                            ).strip()
                            if reply.lower() != "stopped":
                                raise RuntimeError(reply or "未收到 Dashboard 回复")
                        except Exception as stop_error:
                            raise RuntimeError(
                                f"无法确认停止命令已被接受：{stop_error}。"
                                "请立即查看示教器并按现场规程停机。"
                            ) from stop_error

                        print("Dashboard 已接受停止命令。请在示教器上确认机械臂已静止。")
                        return 1

                    time.sleep(CHECK_INTERVAL)
            finally:
                receiver.disconnect()


if __name__ == "__main__":
    try:
        raise SystemExit(monitor())
    except KeyboardInterrupt:
        print("\n监控已退出。")
        raise SystemExit(130)
    except (OSError, ValueError, RuntimeError) as error:
        print(f"监控程序错误：{error}")
        raise SystemExit(1)