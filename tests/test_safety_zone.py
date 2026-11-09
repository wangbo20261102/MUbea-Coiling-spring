import copy
import json
import math
from pathlib import Path
import re
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from safety_zone import SafetyZone, ZoneViolation


class ZoneTests(unittest.TestCase):
    def setUp(self):
        self.config = json.loads(
            (ROOT / "safety_zone.json").read_text(encoding="utf-8")
        )
        self.zone = SafetyZone(self.config)

    def test_source_coordinates_match_config(self):
        paths = sorted((ROOT / "坐标").glob("*.txt"))
        self.assertEqual(len(paths), 4)
        for path, xy in zip(paths, self.zone.vertices):
            text = path.read_text(encoding="utf-8-sig")
            parsed = dict(re.findall(
                r"([XYZ])\s*=\s*([+-]?\d+(?:\.\d+)?)\s*cm", text
            ))
            for axis, value in zip("XY", xy):
                self.assertAlmostEqual(float(parsed[axis]) * 10, value)

    def test_six_unit_normals_point_inwards(self):
        self.assertEqual(len(self.zone.planes), 6)
        for plane in self.zone.planes:
            self.assertAlmostEqual(math.hypot(*plane.normal), 1)
            self.assertGreater(plane.clearance_mm((0, 0, 175)), 0)

    def test_bottom_and_top_are_zero_and_350(self):
        for z in (0, 175, 350):
            self.zone.require_xyz_mm((0, 0, z))
        for z in (-0.001, 350.001):
            with self.subTest(z=z), self.assertRaises(ZoneViolation):
                self.zone.require_xyz_mm((0, 0, z))

    def test_each_side_rejects_points_outward(self):
        for i, plane in enumerate(self.zone.planes[:4]):
            p, q = self.zone.vertices[i], self.zone.vertices[(i + 1) % 4]
            xyz = [(p[0] + q[0]) / 2, (p[1] + q[1]) / 2, 175]
            xyz[0] -= plane.normal[0]
            xyz[1] -= plane.normal[1]
            with self.subTest(side=i), self.assertRaises(ZoneViolation):
                self.zone.require_xyz_mm(xyz)

    def test_bounding_box_corner_is_not_allowed(self):
        # Inside min/max rectangle but outside the actual slanted quadrilateral.
        with self.assertRaises(ZoneViolation):
            self.zone.require_xyz_mm((175, -660, 175))

    def test_known_a_b_and_their_approaches(self):
        for xyz in ([24.259, -555.011, 67.030], [24.259, -555.011, 167.030],
                    [-338.066, -252.792, 141.738],
                    [-338.066, -252.792, 241.738]):
            self.zone.require_xyz_mm(xyz)

    def test_metres_are_converted_once(self):
        self.zone.require_pose_m([0, 0, 0.350, 0, 0, 0])
        with self.assertRaises(ZoneViolation):
            self.zone.require_pose_m([0, 0, 350, 0, 0, 0])

    def test_linear_inside_and_outside(self):
        a = [0.024259, -0.555011, 0.16703, 0, 0, 0]
        b = [-0.338066, -0.252792, 0.241738, 0, 0, math.pi]
        self.zone.require_path_m(a, b, "moveL")
        outside = b.copy()
        outside[2] = 0.351
        with self.assertRaises(ZoneViolation):
            self.zone.require_path_m(a, outside, "moveL")
        with self.assertRaises(ZoneViolation):
            self.zone.require_path_m(outside, b, "moveL")

    def test_joint_and_blended_paths_fail_closed(self):
        pose = [0, 0, 0.1, 0, 0, 0]
        for kind in ("moveJ", "moveP", "blended", ""):
            with self.subTest(kind=kind), self.assertRaises(ZoneViolation):
                self.zone.require_path_m(pose, pose, kind)

    def test_nonzero_margin_applies_to_all_six_faces(self):
        config = copy.deepcopy(self.config)
        config["tcp_margin_mm"] = 10
        zone = SafetyZone(config)
        zone.require_xyz_mm((0, 0, 100))
        for z in (9, 341):
            with self.assertRaises(ZoneViolation):
                zone.require_xyz_mm((0, 0, z))
        for i, plane in enumerate(zone.planes[:4]):
            p, q = zone.vertices[i], zone.vertices[(i + 1) % 4]
            xyz = [(p[0] + q[0]) / 2 + 5 * plane.normal[0],
                   (p[1] + q[1]) / 2 + 5 * plane.normal[1], 175]
            with self.assertRaises(ZoneViolation):
                zone.require_xyz_mm(xyz)

    def test_clockwise_and_counterclockwise_give_same_checks(self):
        reverse = copy.deepcopy(self.config)
        reverse["xy_vertices"].reverse()
        zone = SafetyZone(reverse)
        for z in (0, 350):
            zone.require_xyz_mm((0, 0, z))
        with self.assertRaises(ZoneViolation):
            zone.require_xyz_mm((175, -660, 100))

    def test_invalid_polygons_and_units_are_rejected(self):
        changes = [
            ("xy_vertices", [[0, 0], [1, 1], [0, 1], [1, 0]]),
            ("xy_vertices", [[0, 0], [0, 0], [1, 1], [0, 1]]),
            ("xy_vertices", [[0, 0], [2, 0], [1, 0.1], [0, 2]]),
            ("coordinate_frame", "tool"), ("units", "m"),
            ("allowed_region", "outside"), ("z_min", 350),
            ("z_max", float("inf")), ("tcp_margin_mm", -1),
            ("tcp_margin_mm", True),
        ]
        for key, value in changes:
            config = copy.deepcopy(self.config)
            config[key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                SafetyZone(config)

    def test_invalid_input_is_rejected(self):
        for xyz in ([0, 0], [0, 0, float("nan")], [0, 0, True],
                    [0, 0, "0"]):
            with self.subTest(xyz=xyz), self.assertRaises(ValueError):
                self.zone.require_xyz_mm(xyz)


if __name__ == "__main__":
    unittest.main()
