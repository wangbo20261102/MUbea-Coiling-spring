"""Move the active TCP 100 mm along the UR5 base frame's positive Z axis."""

import argparse
import socket
import sys

from ur5_limit_safety import protect_script


ROBOT_PORT = 30002

UR_SCRIPT = protect_script("""def move_tcp_up_base_z():
  if is_steady():
    target = get_actual_tcp_pose()
    target[2] = target[2] + 0.100
    if is_within_safety_limits(target):
      movel(target, a=0.10, v=0.025, r=0.0)
      textmsg("Base Z +100 mm: move completed")
    else:
      textmsg("Base Z +100 mm: target outside safety limits")
    end
  else:
    textmsg("Base Z +100 mm: robot is not stationary")
  end
end
""")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--execute", metavar="ROBOT_IP", help="send the script to this controller IP"
    )
    args = parser.parse_args()

    if args.execute is None:
        print(UR_SCRIPT, end="")
        return 0

    print(f"Controller: {args.execute}:{ROBOT_PORT}")
    print("Active TCP: base Z +100 mm, orientation held, speed 25 mm/s.")
    if input("Confirm the path is clear, then type YES to send: ").strip() != "YES":
        print("Cancelled.")
        return 0

    try:
        with socket.create_connection((args.execute, ROBOT_PORT), timeout=5.0) as conn:
            conn.sendall(UR_SCRIPT.encode("ascii"))
    except OSError as error:
        print(f"Communication error: {error}", file=sys.stderr)
        print("Check the teach pendant before retrying.", file=sys.stderr)
        return 1

    print("Script sent. Check motion and the log on the teach pendant.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
