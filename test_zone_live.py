"""Read-only TCP region test. Never uploads scripts or sends motion/stop."""

import argparse
from contextlib import suppress
import math
from pathlib import Path
import socket
import time

from safety_zone import DEFAULT_CONFIG, SafetyZone, vector


def evaluate(zone, pose):
    pose = vector(pose, 6)
    xyz = [value * 1000 for value in pose[:3]]
    distances = zone.distances_mm(xyz)
    nearest = min(distances, key=distances.get)
    inside = all(value >= zone.margin for value in distances.values())
    return inside, xyz, nearest, distances[nearest]


def self_test(zone):
    center = [sum(p[i] for p in zone.vertices) / 4 for i in range(2)]
    center.append((zone.z_min + zone.z_max) / 2)
    zone.require_xyz_mm(center)
    cases = [("Inside", center, True)]
    for i, plane in enumerate(zone.planes):
        if i < 4:
            p, q = zone.vertices[i], zone.vertices[(i + 1) % 4]
            point = [(p[0] + q[0]) / 2, (p[1] + q[1]) / 2, center[2]]
            point = [value - normal for value, normal in zip(point, plane.normal)]
        else:
            point = center.copy()
            point[2] = zone.z_min - 1 if i == 4 else zone.z_max + 1
        cases.append((plane.name, point, False))
    for name, xyz, expected in cases:
        inside, _, _, _ = evaluate(zone, [v / 1000 for v in xyz] + [0, 0, 0])
        if inside != expected:
            raise RuntimeError(f"Self-test failed: {name}")
        print(f"PASS {name}: {'INSIDE' if inside else 'OUTSIDE'}", flush=True)
    print("7/7 geometry tests passed. No robot connection or motion.", flush=True)


class Freshness:
    def __init__(self, timeout=1.0):
        self.timeout = timeout
        self.previous = None
        self.last_update = None

    def check(self, stamp, now):
        if isinstance(stamp, bool) or not isinstance(stamp, (float, int)):
            raise RuntimeError("Invalid robot timestamp; position is UNKNOWN.")
        if not math.isfinite(stamp):
            raise RuntimeError("Invalid robot timestamp; position is UNKNOWN.")
        advanced = self.previous is not None and stamp > self.previous
        if self.previous is not None and stamp < self.previous:
            raise RuntimeError("Robot timestamp went backwards; position is UNKNOWN.")
        if self.previous is None or advanced:
            self.previous, self.last_update = stamp, now
        if now - self.last_update >= self.timeout:
            raise RuntimeError("RTDE data is stale; position is UNKNOWN.")
        return advanced


def monitor(zone, ip, duration):
    print(f"Read-only monitor: {ip}:30004; Ctrl+C to exit.", flush=True)
    print("Reports TCP only. Does NOT stop or constrain any robot motion.", flush=True)
    # Probe first so a disconnected robot fails promptly, before RTDE construction.
    with socket.create_connection((ip, 30004), timeout=2.0):
        pass
    import rtde_receive

    receiver = None
    any_outside = False
    try:
        receiver = rtde_receive.RTDEReceiveInterface(
            ip, 20.0, ["timestamp", "actual_TCP_pose"]
        )
        start = time.monotonic()
        freshness = Freshness()
        verified = False
        samples = 0
        while duration == 0 or time.monotonic() - start < duration:
            if not receiver.isConnected():
                raise RuntimeError("RTDE disconnected; position is UNKNOWN.")
            advanced = freshness.check(receiver.getTimestamp(), time.monotonic())
            verified = verified or advanced
            if verified:
                inside, xyz, name, distance = evaluate(zone, receiver.getActualTCPPose())
                any_outside = any_outside or not inside
                samples += 1
                print(
                    f"{time.strftime('%H:%M:%S')} "
                    f"{'INSIDE' if inside else 'OUTSIDE'} "
                    f"XYZ(mm)=({xyz[0]:.2f}, {xyz[1]:.2f}, {xyz[2]:.2f}) "
                    f"nearest={name}: {distance:.2f} mm; margin={zone.margin:g} mm",
                    flush=True,
                )
            time.sleep(0.1)
        if not samples:
            raise RuntimeError("No fresh position samples; test is inconclusive.")
        print(f"Read {samples} samples. No motion or stop commands were sent.", flush=True)
        return 2 if any_outside else 0
    except KeyboardInterrupt:
        print("\nMonitor closed. No robot stop command was sent.", flush=True)
        return 130
    finally:
        if receiver is not None:
            with suppress(Exception):
                receiver.disconnect()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--ip", default="192.168.1.10")
    parser.add_argument("--duration", type=float, default=0,
                        help="Seconds to monitor; 0 = until Ctrl+C.")
    parser.add_argument("--self-test", action="store_true",
                        help="Simulate inside/outside points without connecting.")
    args = parser.parse_args()
    try:
        if not math.isfinite(args.duration) or args.duration < 0:
            raise ValueError("Duration must be finite and >= 0.")
        zone = SafetyZone.load(args.config)
        if args.self_test:
            self_test(zone)
            return 0
        return monitor(zone, args.ip, args.duration)
    except Exception as error:
        print(f"TEST FAILED / UNKNOWN: {error}", flush=True)
        print("Monitoring is unavailable. Do not treat this as active protection.", flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
