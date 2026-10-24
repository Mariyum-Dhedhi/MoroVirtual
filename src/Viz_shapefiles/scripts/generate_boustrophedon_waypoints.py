from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]

Point = tuple[float, float]

BAND_TOLERANCE = 3.0

# MUST-C dataset has 17 crop columns; the shapefile-derived grid has 20, so
# the 3 westernmost columns (smallest along_coord per band) are dropped.
DROPPED_WESTERN_COLUMNS = 3

# Datum yaw (radians) describing the field's row direction, from the SW->SE
# bearing computed off the shapefile's minimum-area bounding rectangle.
DATUM_YAW = -0.067526

# Distance (m) the path extends past a row's south end before entering it,
# and past its north end before leaving it for the next plot in the pass.
OVERSHOOT_DISTANCE = 2.0

# Extra distance (m) a headland corner sits beyond the overshoot point,
# continuing in the same exit direction, before turning toward the next
# column -- keeps the turn outside the field rather than cutting across it.
HEADLAND_DISTANCE = 1.0

# Distance (m) row_start/row_end are each extended past the plot boundary,
# along the row's own direction, so the robot is already straight and
# aligned when it crosses into/out of the crop -- no in-crop turning.
ROW_EXTENSION = 0.75


def _centroid(centerlines: list[list[Point]]) -> Point:
    mids = [((s[0] + e[0]) / 2, (s[1] + e[1]) / 2) for s, e in centerlines]
    cx = sum(m[0] for m in mids) / len(mids)
    cy = sum(m[1] for m in mids) / len(mids)
    return cx, cy


def _row_and_perp_dir(angle: float) -> tuple[Point, Point]:
    row_dir = (math.cos(angle), math.sin(angle))
    perp_dir = (-math.sin(angle), math.cos(angle))
    return row_dir, perp_dir


def _row_traversal(
    centerlines: list[list[Point]], row_idx: int, north_to_south: bool
) -> tuple[Point, Point, float]:
    """Return (start_pt, end_pt, signed_uy) for traversing one centerline.

    start_pt/end_pt are the physical south/north ends, swapped when the
    pass direction is north->south so the plot is always entered from
    whichever side faces the direction of travel. signed_uy is the
    unit-y component in the direction of travel (positive = north,
    negative = south), used to compute the entry/overshoot y-offsets.
    """
    (sx, sy), (ex, ey) = centerlines[row_idx]
    dy = ey - sy
    length = math.hypot(ex - sx, dy)
    uy = dy / length
    if north_to_south:
        return (ex, ey), (sx, sy), -uy
    return (sx, sy), (ex, ey), uy


def _cluster_bands(
    plots: list[tuple[str, float, float, list[list[Point]]]], tolerance: float
) -> list[list[tuple[str, float, float, list[list[Point]]]]]:
    """Greedy 1D clustering on perp_coord, sorted ascending (south to north
    along the field's local rotated axis).

    Uses consecutive-gap chaining (compare each point to the previous one in
    sorted order, not to the band's first member): once projected onto the
    perpendicular axis, plots within a physical row sit ~0.2-0.3m apart while
    the gap between physical rows is 5m+, so a single gap threshold cleanly
    separates the rows. (Anchor-to-first-member would incorrectly fragment a
    row, since a row's total perp_coord spread exceeds the tolerance.)
    """
    ordered = sorted(plots, key=lambda p: p[2])

    bands: list[list[tuple[str, float, float, list[list[Point]]]]] = []
    current = [ordered[0]]
    for plot in ordered[1:]:
        if plot[2] - current[-1][2] <= tolerance:
            current.append(plot)
        else:
            bands.append(current)
            current = [plot]
    bands.append(current)
    return bands


def generate_snake_waypoints(
    centerlines_by_id: dict[str, dict[str, Any]],
    tolerance: float = BAND_TOLERANCE,
    angle: float = DATUM_YAW,
    overshoot: float = OVERSHOOT_DISTANCE,
    headland: float = HEADLAND_DISTANCE,
) -> list[dict[str, Any]]:
    """Column-major snake traversal.

    The field is a grid of (band, column) plots, bands south->north and
    columns west->east. For a fixed column, all 4 bands' row-idx-th
    centerline are visited before moving to the next row within that
    column. Direction alternates by tracked continuity, not parity: the
    robot starts at the south, so pass 1 goes S->N; every pass after that
    goes the opposite way from the one before it, so each new pass always
    starts on the exact side of the field the previous pass ended on --
    both within a column and across a column-to-column headland turn.

    Each centerline's "start" is its south end and "end" its north end
    (row_centerlines.json convention). Every centerline is entered
    `overshoot` meters before its start and, if more bands remain in the
    current pass, exited `overshoot` meters past its end before the next
    plot's entry point.
    """
    row_dir, perp_dir = _row_and_perp_dir(angle)

    plots = []
    for plot_id, r in centerlines_by_id.items():
        cx, cy = _centroid(r["centerlines"])
        along_coord = cx * row_dir[0] + cy * row_dir[1]
        perp_coord = cx * perp_dir[0] + cy * perp_dir[1]
        plots.append((plot_id, along_coord, perp_coord, r["centerlines"]))

    bands_raw = _cluster_bands(plots, tolerance)  # south -> north

    bands: list[list[tuple[str, float, float, list[list[Point]]]]] = []
    for band in bands_raw:
        band_sorted = sorted(band, key=lambda p: p[1])[DROPPED_WESTERN_COLUMNS:]
        bands.append(band_sorted)  # west -> east

    num_bands = len(bands)
    num_cols = min(len(b) for b in bands)

    waypoints: list[dict[str, Any]] = []
    # Robot starts at the field's south edge, so the very first pass goes
    # S->N; every pass after that flips direction from the one before it,
    # so each new pass always starts on the side the previous one ended on.
    current_exit_side = "south"
    for col_idx in range(num_cols):
        column_group = [bands[b][col_idx] for b in range(num_bands)]
        # row_count is not perfectly uniform across bands: floating-point
        # noise in short_edge_length (from upstream row_centerlines.json,
        # left unmodified) puts some plots at 5.999999... instead of
        # exactly 6.0, so floor(short_edge / row_spacing) yields 3 rows
        # there instead of 4. Iterate the union of row indices present in
        # any band, and skip a band from a given pass if it lacks that row
        # rather than truncating the whole column to the shortest plot.
        row_count = max(len(cg[3]) for cg in column_group)

        for row_idx in range(row_count):
            band_order = [b for b in range(num_bands) if row_idx < len(column_group[b][3])]
            north_to_south = current_exit_side == "north"
            current_exit_side = "south" if north_to_south else "north"
            if north_to_south:
                band_order = band_order[::-1]

            for pass_pos, band_i in enumerate(band_order):
                plot_id, _along, _perp, centerlines = column_group[band_i]
                start_pt, end_pt, signed_uy = _row_traversal(centerlines, row_idx, north_to_south)

                meta = {
                    "col_idx": col_idx,
                    "row_idx": row_idx,
                    "band": band_i + 1,
                    "plot_id": plot_id,
                }

                # Entry/overshoot only bracket a whole pass now -- the first
                # band gets an entry before its row_start, the last band
                # gets an overshoot after its row_end. Bands in between
                # connect row_end(plot N) -> row_start(plot N+1) directly:
                # the robot is already heading the right way and the soil
                # gap between plots is open, so no stop/reverse is needed.
                if pass_pos == 0:
                    waypoints.append({
                        **meta,
                        "x": start_pt[0], "y": start_pt[1] - overshoot * signed_uy,
                        "point_type": "entry",
                    })

                # row_start/row_end are extended along the row's own
                # direction (not just its y-component) so the robot is
                # already aligned with the crop rows on entry/exit --
                # entry/overshoot below stay anchored to the *original*
                # start_pt/end_pt, since they're turn/approach points, not
                # row endpoints.
                row_dx, row_dy = end_pt[0] - start_pt[0], end_pt[1] - start_pt[1]
                row_length = math.hypot(row_dx, row_dy)
                row_ux, row_uy = row_dx / row_length, row_dy / row_length

                ext_start = (
                    start_pt[0] - ROW_EXTENSION * row_ux,
                    start_pt[1] - ROW_EXTENSION * row_uy,
                )
                ext_end = (
                    end_pt[0] + ROW_EXTENSION * row_ux,
                    end_pt[1] + ROW_EXTENSION * row_uy,
                )

                waypoints.append({
                    **meta, "x": ext_start[0], "y": ext_start[1], "point_type": "row_start",
                })
                waypoints.append({
                    **meta, "x": ext_end[0], "y": ext_end[1], "point_type": "row_end",
                })

                is_last_of_column = (
                    row_idx == row_count - 1 and pass_pos == len(band_order) - 1
                )

                if pass_pos == len(band_order) - 1:
                    overshoot_pt = (end_pt[0], end_pt[1] + overshoot * signed_uy)
                    waypoints.append({
                        **meta, "x": overshoot_pt[0], "y": overshoot_pt[1],
                        "point_type": "overshoot",
                    })

                if is_last_of_column and col_idx < num_cols - 1:
                    # Headland turn into the next column: a rectangular
                    # detour (2 corners) outside the field edge instead of
                    # cutting diagonally across it.
                    y_headland = overshoot_pt[1] + headland * signed_uy

                    next_column_group = [bands[b][col_idx + 1] for b in range(num_bands)]
                    # current_exit_side was already advanced above for this
                    # (last) pass of the column, so it already holds the
                    # side the *next* pass (row_idx=0 of the next column)
                    # must start on -- reusing it here (rather than an
                    # independent formula) is what guarantees the headland
                    # always connects two passes on the same field edge.
                    next_north_to_south = current_exit_side == "north"
                    next_band_order = [
                        b for b in range(num_bands) if 0 < len(next_column_group[b][3])
                    ]
                    if next_north_to_south:
                        next_band_order = next_band_order[::-1]
                    _, _, _, next_centerlines = next_column_group[next_band_order[0]]
                    next_start_pt, _, _ = _row_traversal(next_centerlines, 0, next_north_to_south)
                    x_next = next_start_pt[0]

                    waypoints.append({
                        **meta, "x": overshoot_pt[0], "y": y_headland,
                        "point_type": "headland",
                    })
                    waypoints.append({
                        **meta, "x": x_next, "y": y_headland,
                        "point_type": "headland",
                    })

    return waypoints


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a column-major snake waypoint list covering all plots."
    )
    parser.add_argument(
        "--row-centerlines",
        type=Path,
        default=REPO_ROOT / "outputs" / "row_centerlines.json",
        help="Path to the plot ID -> row centerlines JSON.",
    )
    parser.add_argument(
        "--yaml-output",
        type=Path,
        default=REPO_ROOT / "outputs" / "waypoints.yaml",
    )
    parser.add_argument(
        "--csv-output",
        type=Path,
        default=REPO_ROOT / "outputs" / "waypoints.csv",
    )
    parser.add_argument(
        "--band-tolerance",
        type=float,
        default=BAND_TOLERANCE,
        help="perp_coord tolerance (meters) for grouping plots into row-bands.",
    )
    parser.add_argument(
        "--datum-yaw",
        type=float,
        default=DATUM_YAW,
        help="Field row-direction angle (radians) defining row_dir/perp_dir.",
    )
    parser.add_argument(
        "--overshoot",
        type=float,
        default=OVERSHOOT_DISTANCE,
        help="Entry/exit overshoot distance (meters) past each row's ends.",
    )
    parser.add_argument(
        "--headland",
        type=float,
        default=HEADLAND_DISTANCE,
        help="Extra distance (meters) a headland corner sits past the overshoot point.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    with open(args.row_centerlines) as f:
        centerlines_by_id = json.load(f)

    waypoints = generate_snake_waypoints(
        centerlines_by_id, args.band_tolerance, args.datum_yaw, args.overshoot, args.headland
    )

    args.yaml_output.parent.mkdir(parents=True, exist_ok=True)
    with open(args.yaml_output, "w") as f:
        yaml.safe_dump(
            {
                "num_waypoints": len(waypoints),
                "waypoints": [
                    {
                        "index": i,
                        "col_idx": wp["col_idx"],
                        "row_idx": wp["row_idx"],
                        "band": wp["band"],
                        "plot_id": wp["plot_id"],
                        "x": wp["x"],
                        "y": wp["y"],
                        "point_type": wp["point_type"],
                    }
                    for i, wp in enumerate(waypoints)
                ],
            },
            f,
            sort_keys=False,
        )

    with open(args.csv_output, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            ["index", "col_idx", "row_idx", "band", "plot_id", "x", "y", "point_type"]
        )
        for i, wp in enumerate(waypoints):
            writer.writerow([
                i, wp["col_idx"], wp["row_idx"], wp["band"], wp["plot_id"],
                wp["x"], wp["y"], wp["point_type"],
            ])

    print(f"Total waypoints: {len(waypoints)}")
    print(f"\nSaved to: {args.yaml_output}")
    print(f"Saved to: {args.csv_output}")


if __name__ == "__main__":
    main()
