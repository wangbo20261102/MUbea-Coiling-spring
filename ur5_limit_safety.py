"""UR5 TCP 工作区软件限位。默认仅本地检查/预览，不连接机器人。

单位：外部输入 mm，控制器位姿 m。Base-Z >= 0，无 Z 上限。
仅约束本模块包装的固定姿态、无圆角 movel 程序，不是安全认证功能。
"""

import argparse
import math
import re
import socket
import textwrap


VERTICES_MM = ((153.3, -654.5), (-434.2, -664.4),
               (-448.2, 185.4), (175.9, 209.3))
# 四个外法线半平面：a*x + b*y <= c。不是轴对齐包围框。
PLANES = ((9.9, -587.5, 386036.42),
          (-849.8, -14.0, 378284.76),
          (-23.9, 624.1, 126420.12),
          (863.8, -22.6, 147212.24))
# 只吸收浮点舍入误差，相当于小于 1e-9 mm 的水平距离。
ROUNDING_EPS = 1e-7
STOP_DECEL_M_S2 = 0.5


def contains_mm(xyz):
    """包含边界；无效值拒绝；Z 不使用放宽容差。"""
    try:
        if len(xyz) != 3 or not all(math.isfinite(v) for v in xyz):
            return False
        x, y, z = xyz
        return z >= 0 and all(
            a*x + b*y <= c + ROUNDING_EPS for a, b, c in PLANES
        )
    except (TypeError, ValueError, OverflowError):
        return False


def require_inside_mm(xyz, name="target"):
    if not contains_mm(xyz):
        raise ValueError(f"{name} outside TCP workspace: {xyz}; Base Z must be >= 0 mm")


def protect_script(script):
    """包装本目录已有的 movel 程序；不是任意 URScript 的安全沙箱。

    运动放在子线程。主线程每个 sync 周期检查实际 TCP；越界时先 kill
    运动线程，再 stopl，避免两个线程同时发运动指令。结束后不继续监控。
    """
    match = re.match(r"def ([A-Za-z_][A-Za-z_0-9]*)\(\):\n", script)
    if not match or not script.rstrip().endswith("end"):
        raise ValueError("Expected a complete no-argument URScript function")
    if re.search(r"\b(movej|movep|movec|speedj|speedl|servoj|servoc)\s*\(", script):
        raise ValueError("Only fixed-orientation, unblended movel programs are supported")
    original_name = match.group(1)
    guarded_body = re.sub(r"\bmovel\(", "limit_movel(", script)
    checks = [f"({a:.10g}*x + ({b:.10g})*y <= {c:.10g} + 0.0000001)"
              for a, b, c in PLANES]
    condition = " and ".join(checks)
    return f'''def tcp_limited_program():
  global limit_motion_done = False

  def inside_tcp_workspace(pose):
    local x = pose[0] * 1000.0
    local y = pose[1] * 1000.0
    return pose[2] >= 0.0 and {condition}
  end

  def limit_movel(target, a=0.10, v=0.025, r=0.0):
    if r != 0.0 or a <= 0.0 or v <= 0.0:
      textmsg("LIMIT: invalid motion parameters or blending requested")
      halt
    end
    if not inside_tcp_workspace(get_actual_tcp_pose()) or not inside_tcp_workspace(target):
      textmsg("LIMIT: current TCP or target outside workspace; move rejected")
      halt
    end
    if not is_within_safety_limits(target):
      textmsg("LIMIT: target rejected by controller limits")
      halt
    end
    movel(target, a=a, v=v, r=0.0)
  end

{textwrap.indent(guarded_body.rstrip(), '  ')}

  thread limit_motion():
    {original_name}()
    global limit_motion_done = True
    # Keep the handle alive until the supervising thread kills it.
    while True:
      sync()
    end
  end

  if not is_steady():
    textmsg("LIMIT: robot must be stationary before starting")
    halt
  end
  if not inside_tcp_workspace(get_actual_tcp_pose()):
    textmsg("LIMIT: starting TCP is outside workspace; no automatic recovery")
    halt
  end
  local motion_handle = run limit_motion()
  while not limit_motion_done:
    if not inside_tcp_workspace(get_actual_tcp_pose()):
      kill motion_handle
      stopl({STOP_DECEL_M_S2})
      textmsg("LIMIT: actual TCP outside workspace; program stopped")
      halt
    end
    sync()
  end
  kill motion_handle
  if not inside_tcp_workspace(get_actual_tcp_pose()):
    stopl({STOP_DECEL_M_S2})
    textmsg("LIMIT: final TCP outside workspace")
    halt
  end
end
'''


def build_target_script(xyz_mm):
    """直线到指定位置，保持启动时的 TCP 姿态。"""
    require_inside_mm(xyz_mm)
    x, y, z = (format(v / 1000, '.12g') for v in xyz_mm)
    return protect_script(f'''def move_to_limited_target():
  actual = get_actual_tcp_pose()
  target = p[{x}, {y}, {z}, actual[3], actual[4], actual[5]]
  movel(target, a=0.10, v=0.025, r=0.0)
end
''')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check-mm', nargs=3, type=float, metavar=('X', 'Y', 'Z'))
    mode.add_argument('--target-mm', nargs=3, type=float, metavar=('X', 'Y', 'Z'))
    parser.add_argument('--execute', metavar='ROBOT_IP', help='send target motion after confirmation')
    args = parser.parse_args()
    if args.execute and args.target_mm is None:
        parser.error('--execute requires --target-mm')
    if args.check_mm is not None:
        ok = contains_mm(args.check_mm)
        print('INSIDE' if ok else 'OUTSIDE')
        return 0 if ok else 2
    if args.target_mm is None:
        parser.print_help()
        return 0
    try:
        script = build_target_script(args.target_mm)
    except ValueError as error:
        parser.error(str(error))
    if not args.execute:
        print(script, end='')
        return 0
    print(f'Target Base XYZ (mm): {args.target_mm}; controller {args.execute}:30002')
    if input('Confirm active TCP, controller version and clear path; type YES: ').strip() != 'YES':
        print('Cancelled.')
        return 0
    try:
        with socket.create_connection((args.execute, 30002), timeout=5) as connection:
            connection.sendall(script.encode('ascii'))
    except OSError as error:
        print(f'Send failed: {error}. Check the pendant before retrying.')
        return 1
    print('Script sent; check the pendant for execution status.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
