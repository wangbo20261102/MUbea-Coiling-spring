"""Offline geometry and integration tests; never connect to a robot."""
import math
import unittest
from unittest.mock import patch

import ur5_limit_safety as limits
import ur5_move_ab
import ur5_move_base_z
import main


class WorkspaceTests(unittest.TestCase):
    def test_vertices_and_edge_midpoints_are_included(self):
        vertices = limits.VERTICES_MM
        for index, (x, y) in enumerate(vertices):
            xx, yy = vertices[(index + 1) % 4]
            self.assertTrue(limits.contains_mm((x, y, 0)))
            self.assertTrue(limits.contains_mm(((x+xx)/2, (y+yy)/2, 0)))

    def test_each_edge_inside_and_outside(self):
        vertices = limits.VERTICES_MM
        for index, (x, y) in enumerate(vertices):
            xx, yy = vertices[(index+1) % 4]
            a, b, _ = limits.PLANES[index]
            length = math.hypot(a, b)
            mx, my = (x+xx)/2, (y+yy)/2
            # Move 0.01 mm perpendicular to each sloping edge.
            self.assertFalse(limits.contains_mm((mx+a/length*0.01, my+b/length*0.01, 10)))
            self.assertTrue(limits.contains_mm((mx-a/length*0.01, my-b/length*0.01, 10)))

    def test_z_floor_and_unspecified_ceiling(self):
        self.assertTrue(limits.contains_mm((0, 0, 0)))
        self.assertFalse(limits.contains_mm((0, 0, -0.000001)))
        self.assertTrue(limits.contains_mm((0, 0, 10000)))

    def test_bounding_box_does_not_replace_polygon(self):
        self.assertFalse(limits.contains_mm((175.9, -664.4, 10)))

    def test_invalid_inputs_fail_closed(self):
        for xyz in (None, (), (0, 0), ('a', 0, 0),
                    (math.nan, 0, 0), (0, math.inf, 0), (0, 0, math.nan)):
            self.assertFalse(limits.contains_mm(xyz))
        with self.assertRaises(ValueError):
            limits.build_target_script((0, 0, -1))
        with self.assertRaises(ValueError):
            ur5_move_ab.build_script((900, 0, 67), ur5_move_ab.B_XYZ_MM)

    def test_existing_paths_and_convex_segments(self):
        a, b = ur5_move_ab.A_XYZ_MM, ur5_move_ab.B_XYZ_MM
        for start, end in ((a,b),(b,a)):
            z = start[2] + ur5_move_ab.LIFT_MM
            points = [start, (start[0],start[1],z), (end[0],end[1],z), end]
            for p, q in zip(points, points[1:]):
                for step in range(101):
                    t = step / 100
                    self.assertTrue(limits.contains_mm(tuple((1-t)*v+t*w for v,w in zip(p,q))))

    def test_all_existing_entrypoints_have_guard(self):
        with patch('socket.create_connection', side_effect=AssertionError('Unexpected network')):
            scripts = [main.script, ur5_move_base_z.UR_SCRIPT,
                       ur5_move_ab.build_script(ur5_move_ab.A_XYZ_MM, ur5_move_ab.B_XYZ_MM),
                       limits.build_target_script((0, -300, 100))]
        for script in scripts:
            self.assertTrue(script.startswith('def tcp_limited_program():'))
            # Only the checked helper is allowed to contain a raw motion command.
            self.assertEqual(script.count('\n    movel('), 1)
            self.assertIn('kill motion_handle\n      stopl(', script)
            self.assertIn('if not inside_tcp_workspace(get_actual_tcp_pose()):', script)
            self.assertIn('if not is_within_safety_limits(target):', script)
            script.encode('ascii')


if __name__ == '__main__':
    unittest.main()
