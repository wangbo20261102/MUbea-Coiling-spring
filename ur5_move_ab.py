"""Move the active TCP between A and B with a 100 mm lift in base +Z."""

# 使用方法（PowerShell，先进入本文件所在的文件夹）：
#   python .\ur5_move_ab.py --preview   在电脑上预览生成的指令
#   python .\ur5_move_ab.py             输入 YES 后才向控制器发送指令
#
# 程序分成两部分：Python 在电脑上检查参数并发送指令；
# URScript 在机械臂控制器上读取当前位置、决定方向并执行运动。
# 下文的 TCP 指工具中心点，例如夹爪工作位置；socket 则用于网络通信。

import argparse
import math
import socket
import sys

from ur5_limit_safety import protect_script, require_inside_mm


# 控制器的 IP 地址与接收 URScript 的端口。
ROBOT_IP = "192.168.1.10"
ROBOT_PORT = 30002

# A、B 是用同一个工具 TCP 实测的基座坐标，顺序为 (X, Y, Z)，单位为 mm。
# 基座坐标系固定在机械臂底座上，抬升沿基座 +Z 方向。
# 下面的 Rx、Ry、Rz 是采点时记录的旋转向量，单位为 rad，仅供对照。
# 本程序运动时会保持发送后读取到的当前工具姿态，A、B 只指定位置。
A_XYZ_MM = (24.30, -555.04, 67.03)
# A 点实测姿态 (Rx, Ry, Rz)：(-1.84513, 2.50242, 0.10301)。
B_XYZ_MM = (-340.24, -250.33, 140.17)
# B 点实测姿态 (Rx, Ry, Rz)：(0.35360, -3.11672, -0.03994)。

LIFT_MM = 100.0  # 从选中的起点沿基座 +Z 抬升 100 mm。
NEAR_MM = 20.0  # TCP 到起点的三维直线距离不超过 20 mm 才允许启动。
TIE_MM = 1.0  # 同时靠近两点时，两边距离相差不超过 1 mm 则取消。
APPROACH_SPEED_M_S = 0.005  # 对准起点时用 5 mm/s。
TRANSFER_SPEED_M_S = 0.025  # 抬升、平移和下降时用 25 mm/s。
ACCEL_M_S2 = 0.10  # TCP 加速度，单位为 m/s²。


def build_script(a_mm, b_mm):
    """检查 A、B 参数并生成交给机械臂控制器的 URScript 文本。"""
    # 每个点需要三个有限数值。未填写、坐标不足或无效数值都会在电脑端报错。
    for name, xyz in (("A", a_mm), ("B", b_mm)):
        if xyz is None or len(xyz) != 3 or not all(map(math.isfinite, xyz)):
            raise ValueError(f"Set {name}_XYZ_MM to three base-frame coordinates in mm.")
        require_inside_mm(xyz, name)

    # 两点本身必须能区分。高度差也必须小于抬升距离，
    # 这样从任意一点出发，平移后的高度都严格高于另一点，最后一步才是下降。
    if math.dist(a_mm, b_mm) <= TIE_MM:
        raise ValueError(f"A and B must be more than {TIE_MM:g} mm apart.")
    if abs(a_mm[2] - b_mm[2]) >= LIFT_MM:
        raise ValueError("A/B height difference must be less than 100 mm for both directions.")

    # URScript 使用米，填在文件开头的是毫米，代入指令前要除以 1000。
    # f 字符串中的大括号会被 Python 换成具体数值，例如 100 mm 会写成 0.100 m。
    ax, ay, az = (f"{value / 1000:.9f}" for value in a_mm)
    bx, by, bz = (f"{value / 1000:.9f}" for value in b_mm)

    # 下方 URScript 的阅读顺序：
    # 1. is_steady() 确认机械臂静止，再读当前工具的基座位姿 actual。
    #    p[X, Y, Z, Rx, Ry, Rz] 的前三项是位置，后三项是旋转向量。
    #    下标从 0 开始，所以 actual[3] 到 actual[5] 就是当前工具姿态。
    # 2. point_dist() 只比较三维位置。只靠近一个点时就从该点出发；
    #    两点都近时选更近的点，差距太小或两点都太远就写日志并取消。
    # 3. travel_z = 起点 Z + 100 mm。lifted 在起点正上方，
    #    across 在终点正上方。两者 Z 相同，所以中间这段是水平移动。
    # 4. 先检查四个目标位姿是否在控制器安全限位内。这里检查的是目标位姿，
    #    实际运行前还要确认各段路径和机械臂扫过的空间没有障碍。
    # 5. 四次 movel() 依次对准起点、抬升、平移、下降。a 是加速度，
    #    v 是速度，r=0.0 表示不做圆角过渡，机械臂会依次到达各路点。
    #    textmsg() 把开始、完成或取消原因写入示教器日志。
    return protect_script(f"""def move_tcp_between_ab():
  if is_steady():
    actual = get_actual_tcp_pose()
    a_pose = p[{ax}, {ay}, {az}, actual[3], actual[4], actual[5]]
    b_pose = p[{bx}, {by}, {bz}, actual[3], actual[4], actual[5]]
    dist_a = point_dist(actual, a_pose)
    dist_b = point_dist(actual, b_pose)
    near_a = dist_a <= {NEAR_MM / 1000:.3f}
    near_b = dist_b <= {NEAR_MM / 1000:.3f}

    if not near_a and not near_b:
      textmsg("A/B move cancelled: TCP is outside both start regions")
    elif near_a and near_b and norm(dist_a - dist_b) <= {TIE_MM / 1000:.3f}:
      textmsg("A/B move cancelled: start point is ambiguous")
    else:
      if near_a and (not near_b or dist_a < dist_b):
        start_pose = a_pose
        end_pose = b_pose
        direction = "A to B"
      else:
        start_pose = b_pose
        end_pose = a_pose
        direction = "B to A"
      end

      travel_z = start_pose[2] + {LIFT_MM / 1000:.3f}
      lifted = p[start_pose[0], start_pose[1], travel_z, actual[3], actual[4], actual[5]]
      across = p[end_pose[0], end_pose[1], travel_z, actual[3], actual[4], actual[5]]
      lift_ok = is_within_safety_limits(start_pose) and is_within_safety_limits(lifted)
      transfer_ok = is_within_safety_limits(across) and is_within_safety_limits(end_pose)

      if lift_ok and transfer_ok:
        textmsg("A/B move starting: ", direction)
        movel(start_pose, a={ACCEL_M_S2:.2f}, v={APPROACH_SPEED_M_S:.3f}, r=0.0)
        movel(lifted, a={ACCEL_M_S2:.2f}, v={TRANSFER_SPEED_M_S:.3f}, r=0.0)
        movel(across, a={ACCEL_M_S2:.2f}, v={TRANSFER_SPEED_M_S:.3f}, r=0.0)
        movel(end_pose, a={ACCEL_M_S2:.2f}, v={TRANSFER_SPEED_M_S:.3f}, r=0.0)
        textmsg("A/B move completed: ", direction)
      else:
        textmsg("A/B move cancelled: a waypoint is outside safety limits")
      end
    end
  else:
    textmsg("A/B move cancelled: robot is not stationary")
  end
end
""")


def main():
    """处理命令行参数、操作员确认和发送结果。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preview", action="store_true", help="print the URScript locally")
    args = parser.parse_args()

    try:
        script = build_script(A_XYZ_MM, B_XYZ_MM)
    except (TypeError, ValueError) as error:
        print(f"Coordinate setup error: {error}", file=sys.stderr)
        return 2

    # 预览只在终端打印 URScript，直接返回，不建立网络连接。
    if args.preview:
        print(script, end="")
        return 0

    print(f"Controller: {ROBOT_IP}:{ROBOT_PORT}")
    print(f"Base-frame TCP points in mm: A={A_XYZ_MM}, B={B_XYZ_MM}")
    print(f"Start within {NEAR_MM:g} mm of A or B. Lift: {LIFT_MM:g} mm.")
    # strip() 去掉首尾空白，仍要求输入大写 YES 才会继续。
    confirm = input("Confirm the TCP and path are clear, then type YES to send: ").strip()
    if confirm != "YES":
        print("Cancelled.")
        return 0

    # 把脚本文本转成字节发给控制器。with 块结束时会自动关闭连接。
    # 如果发送过程报错，先看示教器状态再决定是否重试。
    try:
        with socket.create_connection((ROBOT_IP, ROBOT_PORT), timeout=5.0) as connection:
            connection.sendall(script.encode("ascii"))
    except OSError as error:
        print(f"Communication error: {error}", file=sys.stderr)
        print("Check the teach pendant before retrying.", file=sys.stderr)
        return 1

    # 这条提示只说明电脑完成了发送。是否启动、完成或取消，要看示教器日志。
    print("Script sent. Check motion and the log on the teach pendant.")
    return 0


# 直接运行文件时才进入 main()。导入文件用于检查参数时不会连接机械臂。
if __name__ == "__main__":
    raise SystemExit(main())
