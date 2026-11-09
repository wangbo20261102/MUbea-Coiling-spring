import socket
import sys

from ur5_limit_safety import protect_script


ROBOT_IP = "192.168.1.10"
ROBOT_PORT = 30002

Z_OFFSET_M = 0.100
ACCEL_M_S2 = 0.10
SPEED_M_S = 0.025

script = protect_script(f"""def move_tcp_up_base_z():
  if is_steady():
    target = get_actual_tcp_pose()
    target[2] = target[2] + {Z_OFFSET_M:.3f}

    if is_within_safety_limits(target):
      movel(target, a={ACCEL_M_S2:.2f}, v={SPEED_M_S:.3f}, r=0.0)
      textmsg("Base Z +100 mm: move completed")
    else:
      textmsg("Base Z move cancelled: target outside safety limits")
    end
  else:
    textmsg("Base Z move cancelled: robot is not stationary")
  end
end
""")


def main():
    print("UR5 TCP linear move")
    print(f"Controller: {ROBOT_IP}:{ROBOT_PORT}")
    print(f"Base Z: +{Z_OFFSET_M * 1000:g} mm; tool orientation unchanged.")
    print(f"Speed: {SPEED_M_S * 1000:g} mm/s.")

    confirm = input("Confirm the path is clear, then type YES to send: ").strip()
    if confirm != "YES":
        print("Cancelled.")
        return 0

    try:
        with socket.create_connection((ROBOT_IP, ROBOT_PORT), timeout=5.0) as connection:
            connection.sendall(script.encode("ascii"))
    except OSError as error:
        print(f"Communication error: {error}", file=sys.stderr)
        print("Check the teach pendant before retrying.", file=sys.stderr)
        return 1

    print("Script sent. Check motion and the log on the teach pendant.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
