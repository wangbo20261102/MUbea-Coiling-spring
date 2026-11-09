"""Execute script control flow with mocked UR APIs, NOT a URScript simulator.

Only translates the small Python-like URScript subset used by this script.
Does not validate controller parsing, physical IK, safety or collisions.
"""
import math
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "ur5_rectangle_once.script").read_text(encoding="utf-8")


class Halt(Exception):
    pass


class PoseFactory:
    def __getitem__(self, values):
        return list(values)


def translate(source):
    lines = []
    depth = 0
    for raw in source.splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if line == "end":
            depth -= 1
            continue
        if line.startswith("local "):
            line = line[6:]
        if line == "halt":
            line = "raise Halt()"
        lines.append("    " * depth + line)
        if line.endswith(":"):
            depth += 1
    assert depth == 0
    return compile("\n".join(lines), "<translated-control-flow>", "exec")


class RobotMock:
    def __init__(self, z=0.200, fail_ik=None, safety=True, joint_jump=False, drop=False):
        self.origin = [0.20, -0.30, z, 0.2, 2.8, -0.1]
        self.actual = self.origin.copy()
        self.moves = []
        self.samples = []
        self.messages = []
        self.fail_ik = fail_ik or (lambda pose: False)
        self.safety = safety
        self.joint_jump = joint_jump
        self.drop = drop

    def ik_ok(self, pose, qnear):
        self.samples.append(pose.copy())
        return not self.fail_ik(pose)

    def move(self, pose, **kwargs):
        self.moves.append((pose.copy(), kwargs))
        self.actual = pose.copy()
        if self.drop:
            self.actual[2] = 0.099

    def run(self, source=SOURCE):
        env = {
            "Halt": Halt, "p": PoseFactory(), "ceil": math.ceil,
            "norm": lambda v: math.sqrt(sum(x*x for x in v)), "length": len,
            "interpolate_pose": lambda a, b, t: [x + (y-x)*t for x, y in zip(a, b)],
            "get_actual_tcp_pose": lambda: self.actual.copy(),
            "get_actual_joint_positions": lambda: [0.0]*6,
            "get_inverse_kin_has_solution": self.ik_ok,
            "get_inverse_kin": lambda pose, qnear: [1.0]*6 if self.joint_jump else qnear.copy(),
            "is_within_safety_limits": lambda q: self.safety,
            "movel": self.move, "sync": lambda: None,
            "textmsg": self.messages.append, "popup": lambda *args, **kwargs: None,
        }
        exec(translate(source), env)


class CycleTests(unittest.TestCase):
    def test_extremes_come_from_coordinate_files(self):
        coords = []
        for path in sorted((ROOT / "坐标").glob("*.txt")):
            values = dict(re.findall(r"([XYZ])\s*=\s*([+-]?\d+\.\d+)\s*cm", path.read_text(encoding="utf-8")))
            coords.append({k: float(v)/100 for k, v in values.items()})
        self.assertEqual(len(coords), 4)
        for axis in ("x", "y"):
            for suffix, operation in (("min", min), ("max", max)):
                value = float(re.search(rf"local {axis}_{suffix} = ([-.\d]+)", SOURCE)[1])
                self.assertAlmostEqual(value, operation(c[axis.upper()] for c in coords))

    def test_one_cycle_and_exact_start_pose_return(self):
        for z in (0.100, 0.110, 0.200):
            with self.subTest(z=z):
                robot = RobotMock(z=z)
                robot.run()
                self.assertEqual(robot.actual, robot.origin)
                expected = [[.1759, -.6644], [-.4482, -.6644], [-.4482, .2093], [.1759, .2093]]
                corners = [pose[:2] for pose, _ in robot.moves
                           if abs(pose[2]-.110) < 1e-9 and pose[:2] in expected]
                # Duplicated raised/lowered waypoints are skipped by the script.
                self.assertEqual(corners, expected)
                for pose, settings in robot.moves:
                    self.assertGreaterEqual(pose[2], .100)
                    self.assertEqual(pose[3:], robot.origin[3:])
                    self.assertEqual(settings, {"a": .050, "v": .020, "r": 0})
                self.assertTrue(all(p[2] >= .100 for p in robot.samples))
                if z > .110:
                    self.assertEqual(robot.moves[0][0][2], z)

    def test_start_below_floor_no_motion(self):
        robot = RobotMock(z=.0999)
        with self.assertRaises(Halt):
            robot.run()
        self.assertEqual(robot.moves, [])

    def test_invalid_config_no_motion(self):
        robot = RobotMock()
        with self.assertRaises(Halt):
            robot.run(SOURCE.replace("z_run = 0.110", "z_run = 0.099"))
        self.assertEqual(robot.moves, [])

    def test_unreachable_return_fails_before_first_motion(self):
        # Reject an interior sample unique to the raised corner4 -> origin leg.
        robot = RobotMock(fail_ik=lambda p: .179 < p[0] < .196 and p[1] > -.25 and p[2] > .19)
        with self.assertRaises(Halt):
            robot.run()
        self.assertEqual(robot.moves, [])

    def test_configured_safety_limit_no_motion(self):
        robot = RobotMock(safety=False)
        with self.assertRaises(Halt):
            robot.run()
        self.assertEqual(robot.moves, [])

    def test_joint_discontinuity_no_motion(self):
        robot = RobotMock(joint_jump=True)
        with self.assertRaises(Halt):
            robot.run()
        self.assertEqual(robot.moves, [])

    def test_measured_low_z_prevents_next_move(self):
        robot = RobotMock(drop=True)
        with self.assertRaises(Halt):
            robot.run()
        self.assertEqual(len(robot.moves), 1)


if __name__ == "__main__":
    unittest.main()
