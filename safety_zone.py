"""TCP 工作区域的离线几何检查，不连接或驱动机器人。"""

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path


DEFAULT_CONFIG = Path(__file__).resolve().with_name("safety_zone.json")


class ZoneViolation(ValueError):
    """TCP 点或路径未通过区域检查。"""


def finite_number(value):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value)):
        raise ValueError("坐标和配置必须为有限数值。")
    return float(value)


def vector(values, length):
    if not isinstance(values, (list, tuple)) or len(values) != length:
        raise ValueError(f"需要 {length} 个坐标值。")
    return tuple(finite_number(v) for v in values)


@dataclass(frozen=True)
class Plane:
    name: str
    normal: tuple
    offset_mm: float

    def clearance_mm(self, xyz):
        # 单位内法向量定义允许侧：normal · xyz >= offset_mm。
        return sum(n * v for n, v in zip(self.normal, xyz)) - self.offset_mm


class SafetyZone:
    def __init__(self, config):
        if not isinstance(config, dict):
            raise ValueError("区域配置必须是 JSON 对象。")
        required = {
            "schema_version": 1,
            "coordinate_frame": "base",
            "units": "mm",
            "allowed_region": "inside",
        }
        if any(config.get(k) != v for k, v in required.items()):
            raise ValueError("需要版本 1、base 坐标、mm 单位、内部允许区配置。")
        vertices = config.get("xy_vertices")
        if not isinstance(vertices, list) or len(vertices) != 4:
            raise ValueError("需要按周界顺序排列的四个顶点。")
        self.vertices = tuple(vector(p, 2) for p in vertices)
        self.z_min = finite_number(config.get("z_min"))
        self.z_max = finite_number(config.get("z_max"))
        self.margin = finite_number(config.get("tcp_margin_mm"))
        if self.z_min >= self.z_max or self.margin < 0:
            raise ValueError("Z 上限必须大于下限，内缩余量不得为负数。")

        # 要求严格凸四边形且顺序正确，拒绝交叉、重复、共线及凹边界。
        turns = []
        for i in range(4):
            p, q, r = [self.vertices[(i + j) % 4] for j in range(3)]
            turns.append((q[0] - p[0]) * (r[1] - q[1])
                         - (q[1] - p[1]) * (r[0] - q[0]))
        if not (all(v > 0 for v in turns) or all(v < 0 for v in turns)):
            raise ValueError("四点必须构成按边界顺序排列的严格凸四边形。")
        ccw = turns[0] > 0
        planes = []
        for i in range(4):
            p, q = self.vertices[i], self.vertices[(i + 1) % 4]
            dx, dy = q[0] - p[0], q[1] - p[1]
            length = math.hypot(dx, dy)
            nx, ny = (-dy / length, dx / length) if ccw else (
                dy / length, -dx / length
            )
            planes.append(Plane(
                f"P{i + 1}-P{(i + 1) % 4 + 1}",
                (nx, ny, 0.0),
                nx * p[0] + ny * p[1],
            ))
        planes.extend([
            Plane("Z 下限", (0.0, 0.0, 1.0), self.z_min),
            Plane("Z 上限", (0.0, 0.0, -1.0), -self.z_max),
        ])
        self.planes = tuple(planes)

    @classmethod
    def load(cls, path=DEFAULT_CONFIG):
        return cls(json.loads(Path(path).read_text(encoding="utf-8")))

    def distances_mm(self, xyz_mm):
        xyz = vector(xyz_mm, 3)
        return {p.name: p.clearance_mm(xyz) for p in self.planes}

    def require_xyz_mm(self, xyz_mm):
        """检查 TCP 点；margin 对六个面分别向内收缩，不给出物理安全保证。"""
        distances = self.distances_mm(xyz_mm)
        violations = {
            name: value for name, value in distances.items()
            if value < self.margin
        }
        if violations:
            details = ", ".join(
                f"{name}: {value:.3f} mm" for name, value in violations.items()
            )
            raise ZoneViolation(
                f"TCP 不在允许区：{details}；需要至少 {self.margin:.3f} mm 内距。"
            )
        return distances

    def require_pose_m(self, pose):
        """UR/RTDE 六维位姿：米、弧度旋转向量；这里只检查 TCP 位置。"""
        values = vector(pose, 6)
        return self.require_xyz_mm([v * 1000 for v in values[:3]])

    def require_linear_path_m(self, start_pose, target_pose):
        """仅适用于无圆滑过渡的 TCP 直线位置轨迹。"""
        # 允许区是凸多面体，两端都满足同样的半空间约束时，
        # 理想直线位置轨迹也在内部；不包含跟踪误差、工具旋转扫掠及停止距离。
        self.require_pose_m(start_pose)
        self.require_pose_m(target_pose)

    def require_path_m(self, start_pose, target_pose, motion_type):
        # moveJ 即使起终点在区内，中间 TCP 也可能越界，不能按直线验证。
        if motion_type != "moveL":
            raise ZoneViolation(
                "只支持无圆滑过渡的 moveL 几何检查；"
                "moveJ/混合轨迹需要单独验证关节路径及控制器安全限制。"
            )
        self.require_linear_path_m(start_pose, target_pose)


def inspect_points(zone, path, above_mm):
    """离线检查已记录的 A/B 及上方点；不把原点关节路径视为已验证。"""
    above_mm = finite_number(above_mm)
    if above_mm <= 0:
        raise ValueError("上方高度必须大于零。")
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if (not isinstance(data, dict)
            or data.get("coordinate_frame") != "base"
            or data.get("pose_units") != "m_rad_rotation_vector"):
        raise ValueError("点位文件必须明确使用 base 坐标和 m_rad_rotation_vector 单位。")
    points = data.get("points")
    if not isinstance(points, dict):
        raise ValueError("缺少 points 对象。")
    poses = {}
    for name in ("A", "B"):
        if not isinstance(points.get(name), dict):
            raise ValueError(f"缺少 {name} 点。")
        pose = list(vector(points[name].get("tcp_pose"), 6))
        zone.require_pose_m(pose)
        poses[name] = pose
        upper = pose.copy()
        upper[2] += above_mm / 1000
        zone.require_pose_m(upper)
        poses[f"{name}上方"] = upper
        print(f"{name} / {name}上方：TCP 点在允许区。")
    order = ["A上方", "A", "A上方", "B上方", "B", "B上方"]
    for start, target in zip(order, order[1:]):
        zone.require_path_m(poses[start], poses[target], "moveL")
    print("A/B 中间五段：理想 TCP 直线位置轨迹在区内。")
    print("原点 TCP、原点连接段、工具及机械臂本体路径尚未验证。")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--xyz-mm", type=float, nargs=3, metavar=("X", "Y", "Z"))
    parser.add_argument("--points-file", type=Path)
    parser.add_argument("--above-mm", type=float, default=100.0)
    args = parser.parse_args()
    try:
        zone = SafetyZone.load(args.config)
        print(f"底座坐标系允许区：四边形内部，Z={zone.z_min:g}~{zone.z_max:g} mm。")
        print(f"TCP 点内缩余量：{zone.margin:g} mm。")
        print("以下为单位内法向量 n 与偏移 d，允许侧为 n·[X,Y,Z] >= d：")
        for plane in zone.planes:
            print(f"  {plane.name}: n={plane.normal}, d={plane.offset_mm:.6f} mm")
        if args.xyz_mm is not None:
            distances = zone.require_xyz_mm(args.xyz_mm)
            print(f"TCP 点通过，最小边界内距 {min(distances.values()):.3f} mm。")
        if args.points_file is not None:
            inspect_points(zone, args.points_file, args.above_mm)
        print("此程序不连接机器人，不配置控制器安全平面，不执行运动。")
        return 0
    except (OSError, ValueError, TypeError) as error:
        print(f"检查未通过：{error}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
