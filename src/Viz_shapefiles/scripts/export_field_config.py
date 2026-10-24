from __future__ import annotations

import argparse
import json
from collections import Counter
from difflib import unified_diff
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]

DATUM = [50.6259768055, 6.9836962966, -0.067526]
ROW_DIRECTION = -0.067526
ROW_SPACING = 1.5
ROW_START_OFFSET = 0.0


def compute_values(row_centerlines_path: Path) -> tuple[int, float]:
    with open(row_centerlines_path) as f:
        data = json.load(f)

    row_counts = [r["row_count"] for r in data.values()]
    long_edges = [r["long_edge_length"] for r in data.values()]

    modal_row_count = Counter(row_counts).most_common(1)[0][0]
    mean_long_edge = round(sum(long_edges) / len(long_edges), 3)

    return modal_row_count, mean_long_edge


def render_config(row_count: int, row_length: float) -> str:
    datum_str = ", ".join(str(v) for v in DATUM)
    return (
        f"# datum: [{datum_str}]\n"
        "gps_row_error_node:\n"
        "  ros__parameters:\n"
        f"    row_count: {row_count}\n"
        f"    row_spacing: {ROW_SPACING}\n"
        f"    row_length: {row_length}\n"
        f"    row_start_offset: {ROW_START_OFFSET}\n"
        f"    row_direction: {ROW_DIRECTION}\n"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export computed field row geometry into field_config.yaml."
    )
    parser.add_argument(
        "--row-centerlines",
        type=Path,
        default=REPO_ROOT / "outputs" / "row_centerlines.json",
        help="Path to the plot ID -> row centerlines JSON.",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=REPO_ROOT.parent / "p23_bringup" / "config" / "field_config.yaml",
        help="Path to field_config.yaml to overwrite.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    row_count, row_length = compute_values(args.row_centerlines)
    new_content = render_config(row_count, row_length)
    old_content = args.config.read_text() if args.config.exists() else ""

    diff_text = "".join(
        unified_diff(
            old_content.splitlines(keepends=True),
            new_content.splitlines(keepends=True),
            fromfile=str(args.config),
            tofile=str(args.config),
        )
    )
    print(diff_text if diff_text else "No changes.")

    args.config.write_text(new_content)
    print(f"\nWrote: {args.config}")


if __name__ == "__main__":
    main()
