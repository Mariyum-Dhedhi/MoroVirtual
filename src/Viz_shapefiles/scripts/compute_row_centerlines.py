from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]

Point = tuple[float, float]

ROW_SPACING = 1.5


def _dist(p: Point, q: Point) -> float:
    return math.hypot(q[0] - p[0], q[1] - p[1])


def analyze_plot(corners: list[Point], row_spacing: float = ROW_SPACING) -> dict[str, Any]:
    """Derive row centerlines for one plot's 4 corners (in polygon order).

    1. Edge lengths are computed for all 4 sides; the two opposite-side pairs
       are averaged to get the long-edge and short-edge lengths.
    2. The long edge sets the row direction; the short edge sets the spacing
       direction (perpendicular-ish, following the actual quad geometry).
    3. row_count = round(short_edge_length / row_spacing).
    4. Centerlines run parallel to the long edge, spaced row_spacing apart
       along the short edge, inset by half row_spacing from each short-edge
       boundary.
    """
    edge_len = [_dist(corners[i], corners[(i + 1) % 4]) for i in range(4)]
    pair_a = (edge_len[0] + edge_len[2]) / 2  # edges c0-c1 and c2-c3
    pair_b = (edge_len[1] + edge_len[3]) / 2  # edges c1-c2 and c3-c0

    if pair_a >= pair_b:
        origin_idx, long_idx, short_idx = 0, 1, 3
        long_length, short_length = pair_a, pair_b
    else:
        origin_idx, long_idx, short_idx = 1, 2, 0
        long_length, short_length = pair_b, pair_a

    origin = corners[origin_idx]
    long_pt = corners[long_idx]
    short_pt = corners[short_idx]

    dir_short = (
        (short_pt[0] - origin[0]) / short_length,
        (short_pt[1] - origin[1]) / short_length,
    )

    row_count = round(short_length / row_spacing)

    centerlines = []
    for i in range(row_count):
        offset = (i + 0.5) * row_spacing
        start = (origin[0] + dir_short[0] * offset, origin[1] + dir_short[1] * offset)
        end = (long_pt[0] + dir_short[0] * offset, long_pt[1] + dir_short[1] * offset)
        centerlines.append((start, end))

    return {
        "long_edge_length": long_length,
        "short_edge_length": short_length,
        "row_count": row_count,
        "centerlines": centerlines,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate per-row centerlines for each plot's local-frame corners."
    )
    parser.add_argument(
        "--corners",
        type=Path,
        default=REPO_ROOT / "outputs" / "plot_corners_local.json",
        help="Path to the plot ID -> local corners JSON.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=REPO_ROOT / "outputs" / "row_centerlines.json",
        help="Path to save the ID -> row-centerline data as JSON.",
    )
    parser.add_argument(
        "--row-spacing",
        type=float,
        default=ROW_SPACING,
        help="Row spacing in meters.",
    )
    parser.add_argument(
        "--sanity-id",
        type=int,
        default=162,
        help="Plot ID to print in detail as a sanity check.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    with open(args.corners) as f:
        corners_by_id: dict[str, list[Point]] = json.load(f)

    results: dict[str, dict[str, Any]] = {}
    for plot_id, corners in corners_by_id.items():
        corners = [tuple(pt) for pt in corners]
        results[plot_id] = analyze_plot(corners, args.row_spacing)

    sanity_key = str(args.sanity_id)
    if sanity_key in results:
        r = results[sanity_key]
        print(f"Plot ID={sanity_key}")
        print(f"  long edge length:  {r['long_edge_length']:.3f} m")
        print(f"  short edge length: {r['short_edge_length']:.3f} m")
        print(f"  row_count:         {r['row_count']}")
        print("  centerlines:")
        for start, end in r["centerlines"]:
            print(f"    ({start[0]:.3f}, {start[1]:.3f}) -> ({end[0]:.3f}, {end[1]:.3f})")
    else:
        print(f"WARNING: plot ID={sanity_key} not found in corners data")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with open(args.output, "w") as f:
        json.dump(results, f, indent=2)

    print(f"\nComputed row centerlines for {len(results)} plots")
    print(f"Saved to: {args.output}")


if __name__ == "__main__":
    main()
