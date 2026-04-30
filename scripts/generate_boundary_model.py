#!/usr/bin/env python3

import argparse
import math
from pathlib import Path


MODEL_NAME = "boundary_points"


def parse_args():
    script_path = Path(__file__).resolve()
    package_root = script_path.parent.parent
    default_input = Path("/home/chensi/boundary_points.txt")
    default_output = package_root / "models" / MODEL_NAME / "model.sdf"

    parser = argparse.ArgumentParser(
        description="Generate mower_gazebo/models/boundary_points/model.sdf from ENU boundary points."
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=default_input,
        help="Path to ENU boundary point file. Default: %(default)s",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=default_output,
        help="Output model.sdf path. Default: %(default)s",
    )
    parser.add_argument(
        "--stride",
        type=int,
        default=8,
        help="Keep every Nth source point when generating visual segments. Default: %(default)s",
    )
    parser.add_argument(
        "--radius",
        type=float,
        default=0.025,
        help="Cylinder radius for each boundary segment. Default: %(default)s",
    )
    parser.add_argument(
        "--z-offset",
        type=float,
        default=0.035,
        help="Lift visual segments slightly above ground. Default: %(default)s",
    )
    return parser.parse_args()


def load_enu_points(path: Path):
    points = []
    for lineno, line in enumerate(path.read_text().splitlines(), start=1):
        parts = line.split()
        if len(parts) < 3:
            continue
        try:
            east, north, up = (float(value) for value in parts[:3])
        except ValueError as exc:
            raise ValueError(f"invalid point at {path}:{lineno}: {line}") from exc
        points.append((east, north, up))
    if not points:
        raise ValueError(f"no valid points found in {path}")
    return points


def enu_to_map(point):
    east, north, up = point
    return (north, -east, up)


def sample_points(points, stride: int):
    sampled = points[::stride]
    if points and sampled[-1] != points[-1]:
        sampled.append(points[-1])
    return sampled


def build_segments(points, z_offset: float):
    segments = []
    for start, end in zip(points, points[1:]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        dz = end[2] - start[2]
        length = math.sqrt(dx * dx + dy * dy + dz * dz)
        if length < 1e-4:
            continue
        mx = (start[0] + end[0]) * 0.5
        my = (start[1] + end[1]) * 0.5
        mz = (start[2] + end[2]) * 0.5 + z_offset
        yaw = math.atan2(dy, dx)
        segments.append((mx, my, mz, length, yaw))
    return segments


def render_sdf(segments, radius: float):
    lines = [
        '<?xml version="1.0"?>',
        '<sdf version="1.6">',
        f'  <model name="{MODEL_NAME}">',
        "    <static>true</static>",
        '    <link name="boundary_visual_link">',
    ]

    for index, (mx, my, mz, length, yaw) in enumerate(segments):
        lines.extend(
            [
                f'      <visual name="boundary_segment_{index}">',
                f"        <pose>{mx:.6f} {my:.6f} {mz:.6f} 0 1.570796 {yaw:.6f}</pose>",
                "        <geometry>",
                "          <cylinder>",
                f"            <radius>{radius:.6f}</radius>",
                f"            <length>{length:.6f}</length>",
                "          </cylinder>",
                "        </geometry>",
                "        <material>",
                "          <ambient>1.0 0.05 0.02 1</ambient>",
                "          <diffuse>1.0 0.05 0.02 1</diffuse>",
                "          <emissive>0.35 0.02 0.0 1</emissive>",
                "        </material>",
                "      </visual>",
            ]
        )

    lines.extend(
        [
            "    </link>",
            "  </model>",
            "</sdf>",
            "",
        ]
    )
    return "\n".join(lines)


def main():
    args = parse_args()

    if args.stride <= 0:
        raise ValueError("--stride must be > 0")

    enu_points = load_enu_points(args.input)
    map_points = [enu_to_map(point) for point in enu_points]
    sampled_points = sample_points(map_points, args.stride)
    segments = build_segments(sampled_points, args.z_offset)
    sdf = render_sdf(segments, args.radius)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(sdf)

    xs = [point[0] for point in map_points]
    ys = [point[1] for point in map_points]
    print(f"generated {args.output}")
    print(f"source ENU points: {len(enu_points)}")
    print(f"sampled points: {len(sampled_points)}")
    print(f"visual segments: {len(segments)}")
    print(f"map bounds x=[{min(xs):.3f}, {max(xs):.3f}], y=[{min(ys):.3f}, {max(ys):.3f}]")


if __name__ == "__main__":
    main()
