import unittest

from safety_zone import SafetyZone
from test_zone_live import Freshness, evaluate, self_test


class LiveMonitorTests(unittest.TestCase):
    def test_simulated_six_boundaries(self):
        self_test(SafetyZone.load())

    def test_live_pose_conversion_and_outside(self):
        zone = SafetyZone.load()
        inside, xyz, _, distance = evaluate(zone, [0, 0, 0.351, 0, 0, 0])
        self.assertFalse(inside)
        self.assertAlmostEqual(xyz[2], 351)
        self.assertAlmostEqual(distance, -1)
        self.assertTrue(evaluate(zone, [0, 0, 0.1, 0, 0, 0])[0])

    def test_freshness_requires_advancing_robot_time(self):
        check = Freshness()
        self.assertFalse(check.check(10, 100))
        self.assertFalse(check.check(10, 100.5))
        self.assertTrue(check.check(10.1, 100.6))
        with self.assertRaises(RuntimeError):
            check.check(10.1, 101.7)

    def test_backwards_and_invalid_robot_time_fail(self):
        for stamp in (9, float("nan"), float("inf"), None, True):
            check = Freshness()
            check.check(10, 100)
            with self.subTest(stamp=stamp), self.assertRaises(RuntimeError):
                check.check(stamp, 100.1)


if __name__ == "__main__":
    unittest.main()
