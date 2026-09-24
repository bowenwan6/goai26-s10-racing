#!/usr/bin/env python3
"""Regenerate the racing course from the contest track overlay.

The scored course lives in the upstream MJCF as a set of visual marker geoms. Rather than
transcribing coordinates by hand, this script reads them straight from the scene, so the
course we follow can never silently drift from the course we are scored on.

Usage:
    scripts/extract_waypoints.py                       # write the default config
    scripts/extract_waypoints.py --check               # verify the config is current
    scripts/extract_waypoints.py -o /tmp/course.yaml
"""

from __future__ import annotations

import argparse
import math
import re
import sys
from itertools import pairwise
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_OVERLAY = (
    "upstream/goai_embodied_future_material/src/S10_sdk_deploy/"
    "S10_description/s10_mjcf/mjcf/track_overlay.xml"
)
DEFAULT_OUTPUT = REPO_ROOT / "src/s10_bringup/config/course.yaml"

WAYPOINT_PATTERN = re.compile(r'name="track_waypoint_(\d+)_(\w+)"\s+type="sphere"\s+pos="([^"]+)"')


def parse_overlay(path: Path) -> list[tuple[int, str, tuple[float, float, float]]]:
    text = path.read_text()
    waypoints = [
        (int(index), kind, tuple(float(v) for v in pos.split()))
        for index, kind, pos in WAYPOINT_PATTERN.findall(text)
    ]
    if not waypoints:
        raise SystemExit(f"No waypoints found in {path}; has the overlay format changed?")
    waypoints.sort(key=lambda w: w[0])
    return waypoints


def render(waypoints, source: str) -> str:
    length = sum(math.dist(a[2][:2], b[2][:2]) for a, b in pairwise(waypoints))
    climb = sum(max(0.0, b[2][2] - a[2][2]) for a, b in pairwise(waypoints))

    lines = [
        "# S10 perception racing course.",
        "#",
        "# Extracted from the contest track overlay by scripts/extract_waypoints.py.",
        "# Do not hand-edit: regenerate instead, so the course always matches the scored scene.",
        "#",
        f"# Waypoints:      {len(waypoints)}",
        f"# Course length:  {length:.1f} m (horizontal)",
        f"# Total climb:    {climb:.2f} m",
        "#",
        "# The scorer checks a 0.2 m horizontal radius and requires waypoints in order.",
        "",
        "metadata:",
        f"  source: {source}",
        f"  count: {len(waypoints)}",
        f"  length_m: {length:.2f}",
        f"  climb_m: {climb:.2f}",
        "  reach_radius_m: 0.2",
        "",
        "waypoints:",
    ]
    for index, kind, (x, y, z) in waypoints:
        lines.append(f"  - index: {index}")
        lines.append(f"    position: [{x:.4f}, {y:.4f}, {z:.4f}]")
        lines.append(f"    kind: {kind}")
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--overlay",
        type=Path,
        default=REPO_ROOT / DEFAULT_OVERLAY,
        help="Path to the upstream track_overlay.xml",
    )
    parser.add_argument("-o", "--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--check",
        action="store_true",
        help="Exit non-zero if the output file is missing or out of date",
    )
    args = parser.parse_args()

    if not args.overlay.is_file():
        raise SystemExit(f"Overlay not found: {args.overlay}\nRun scripts/setup_upstream.sh first.")

    source = "S10_sdk_deploy/S10_description/s10_mjcf/mjcf/track_overlay.xml"
    content = render(parse_overlay(args.overlay), source)

    if args.check:
        if not args.output.is_file() or args.output.read_text() != content:
            print(
                f"{args.output} is out of date; run scripts/extract_waypoints.py",
                file=sys.stderr,
            )
            return 1
        print(f"{args.output} is up to date")
        return 0

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(content)
    print(f"Wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
