#!/usr/bin/env python3
"""Offline accuracy analysis for a waypoint_follower + accuracy_logger run.

Computes three error metrics from the CSVs produced by accuracy_logger.py
(accuracy_log.csv, reached_events.csv) plus a row-centerline reference file:

  1. Terminal waypoint error — how far the robot actually was from each
     waypoint's target coordinates at the moment /waypoint_reached fired.
  2. Localization error — how far the EKF-filtered pose estimate is from
     ground truth, over the whole run.
  3. Cross-track error — perpendicular distance from ground truth to the
     nearest crop row's centerline, per row and overall.

Usage:
    python3 analyze_accuracy.py \\
        --accuracy-log accuracy_log.csv \\
        --reached-events reached_events.csv \\
        --row-centerlines row_centerlines.csv \\
        --output summary_stats.csv

row_centerlines.csv is expected to have one row per centerline segment:
    row_id,x1,y1,x2,y2
where (x1,y1)-(x2,y2) are the two endpoints of that row's centerline.
"""

import argparse

import numpy as np
import pandas as pd


def _euclidean(dx, dy):
    return np.hypot(dx, dy)


def _rms(values):
    return float(np.sqrt(np.mean(np.square(values))))


def _error_stats(values, n_total, n_dropped=0):
    return {
        "mean": float(np.mean(values)),
        "rms": _rms(values),
        "max": float(np.max(values)),
        "n": int(len(values)),
        "n_dropped": int(n_dropped),
        "n_total": int(n_total),
    }


def terminal_waypoint_error(reached_events: pd.DataFrame) -> dict:
    n_total = len(reached_events)
    missing = reached_events["target_x"].isna() | reached_events["target_y"].isna()
    n_dropped = int(missing.sum())
    if n_dropped:
        dropped_idx = reached_events.loc[missing, "waypoint_idx"].tolist()
        print(
            f"[terminal_waypoint_error] dropped {n_dropped} row(s) with missing "
            f"target_x/target_y (waypoint_idx={dropped_idx}) — these are late-QoS-join "
            f"gaps where /current_target hadn't been received yet when the reached "
            f"event fired, not real errors."
        )

    clean = reached_events.loc[~missing]
    dist = _euclidean(
        clean["x_gt"] - clean["target_x"], clean["y_gt"] - clean["target_y"])
    return _error_stats(dist.to_numpy(), n_total, n_dropped)


def localization_error(accuracy_log: pd.DataFrame) -> dict:
    n_total = len(accuracy_log)
    required = ["x_gt", "y_gt", "x_ekf", "y_ekf"]
    missing = accuracy_log[required].isna().any(axis=1)
    n_dropped = int(missing.sum())
    if n_dropped:
        print(
            f"[localization_error] dropped {n_dropped} row(s) with missing "
            f"x_gt/y_gt/x_ekf/y_ekf (typically the first tick(s) before odometry "
            f"was first received)."
        )

    clean = accuracy_log.loc[~missing]
    dist = _euclidean(clean["x_gt"] - clean["x_ekf"], clean["y_gt"] - clean["y_ekf"])
    return _error_stats(dist.to_numpy(), n_total, n_dropped)


def _point_to_segment_distance(px, py, x1, y1, x2, y2):
    """Distance from points (px, py) to segments (x1,y1)-(x2,y2), clamped to
    the segment. All args broadcast: points are (N,), segments are (M,)."""
    px = px[:, None]
    py = py[:, None]
    x1 = x1[None, :]
    y1 = y1[None, :]
    x2 = x2[None, :]
    y2 = y2[None, :]

    dx = x2 - x1
    dy = y2 - y1
    seg_len_sq = dx * dx + dy * dy
    seg_len_sq = np.where(seg_len_sq == 0, np.finfo(float).eps, seg_len_sq)

    t = ((px - x1) * dx + (py - y1) * dy) / seg_len_sq
    t = np.clip(t, 0.0, 1.0)

    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    return _euclidean(px - proj_x, py - proj_y)


def _perpendicular_line_distance(px, py, x1, y1, x2, y2):
    """Perpendicular distance from points to the *infinite* line through each
    segment's endpoints (the standard cross-track-error definition — a point
    past the end of a row still has a well-defined lateral offset)."""
    dx = x2 - x1
    dy = y2 - y1
    seg_len = np.hypot(dx, dy)
    seg_len = np.where(seg_len == 0, np.finfo(float).eps, seg_len)
    # |cross product| / |line vector|
    return np.abs(dx * (y1 - py) - (x1 - px) * dy) / seg_len


CROSS_TRACK_MAX_ASSIGNMENT_DISTANCE = 1.0  # m


def cross_track_error(accuracy_log: pd.DataFrame, centerlines: pd.DataFrame) -> dict:
    n_total = len(accuracy_log)
    missing = accuracy_log[["x_gt", "y_gt"]].isna().any(axis=1)
    n_dropped = int(missing.sum())
    if n_dropped:
        print(
            f"[cross_track_error] dropped {n_dropped} row(s) with missing x_gt/y_gt.")

    clean = accuracy_log.loc[~missing].reset_index(drop=True)
    px = clean["x_gt"].to_numpy(dtype=float)
    py = clean["y_gt"].to_numpy(dtype=float)

    x1 = centerlines["x1"].to_numpy(dtype=float)
    y1 = centerlines["y1"].to_numpy(dtype=float)
    x2 = centerlines["x2"].to_numpy(dtype=float)
    y2 = centerlines["y2"].to_numpy(dtype=float)
    row_ids = centerlines["row_id"].to_numpy()

    # Assign each point to its nearest row by distance to the finite segment
    # (a point off the end of a row shouldn't snap to that row's line).
    seg_dist = _point_to_segment_distance(px, py, x1, y1, x2, y2)  # (N, M)
    nearest_idx = np.argmin(seg_dist, axis=1)
    nearest_seg_dist = seg_dist[np.arange(len(px)), nearest_idx]
    assigned_row_id = row_ids[nearest_idx]

    # A point can be closer to a real row than to any other real row and
    # still not actually be ON a row — e.g. the robot rotating in place near
    # its spawn point before entering the field. Force-assigning those to
    # "nearest real row anyway" corrupts that row's stats (this is exactly
    # what happened when bogus plot 239 was removed and its nearby points
    # landed on 238_0 instead). Exclude anything too far from every segment.
    excluded = nearest_seg_dist > CROSS_TRACK_MAX_ASSIGNMENT_DISTANCE
    n_excluded = int(excluded.sum())
    if n_excluded:
        print(
            f"[cross_track_error] excluded {n_excluded} point(s) more than "
            f"{CROSS_TRACK_MAX_ASSIGNMENT_DISTANCE} m from their nearest centerline "
            f"segment — not force-assigned to a row (pre-engagement/startup points, "
            f"turn/headland transit, etc. — not actually on any real row):")
        for rid in np.unique(assigned_row_id[excluded]):
            n = int((assigned_row_id[excluded] == rid).sum())
            print(f"    nearest-but-excluded row {rid}: {n} point(s)")

    # The reported error is the perpendicular distance to the assigned row's
    # infinite centerline (standard cross-track-error definition).
    perp_dist = _perpendicular_line_distance(
        px, py, x1[nearest_idx], y1[nearest_idx], x2[nearest_idx], y2[nearest_idx])

    kept = ~excluded
    per_row = {}
    for rid in np.unique(assigned_row_id[kept]):
        row_mask = kept & (assigned_row_id == rid)
        d = perp_dist[row_mask]
        n_row_excluded = int((~kept & (assigned_row_id == rid)).sum())
        per_row[rid] = _error_stats(d, n_total=int(row_mask.sum()) + n_row_excluded)
        per_row[rid]["n_excluded"] = n_row_excluded

    overall = _error_stats(perp_dist[kept], n_total, n_dropped)
    overall["n_excluded"] = n_excluded
    return {"overall": overall, "per_row": per_row}


def build_summary_table(term_wp, loc_err, cte):
    rows = []

    def add(metric, row_id, stats):
        rows.append({
            "metric": metric,
            "row_id": row_id,
            "mean": stats["mean"],
            "rms": stats["rms"],
            "max": stats["max"],
            "n": stats["n"],
            "n_dropped": stats["n_dropped"],
            "n_excluded": stats.get("n_excluded", 0),
        })

    add("terminal_waypoint_error", "ALL", term_wp)
    add("localization_error", "ALL", loc_err)
    add("cross_track_error", "ALL", cte["overall"])
    for row_id, stats in sorted(cte["per_row"].items(), key=lambda kv: str(kv[0])):
        add("cross_track_error", row_id, stats)

    return pd.DataFrame(rows, columns=[
        "metric", "row_id", "mean", "rms", "max", "n", "n_dropped", "n_excluded"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--accuracy-log", default="accuracy_log.csv")
    parser.add_argument("--reached-events", default="reached_events.csv")
    parser.add_argument("--row-centerlines", default="row_centerlines.csv")
    parser.add_argument("--output", default="summary_stats.csv")
    args = parser.parse_args()

    accuracy_log = pd.read_csv(args.accuracy_log)
    reached_events = pd.read_csv(args.reached_events)
    centerlines = pd.read_csv(args.row_centerlines)

    term_wp = terminal_waypoint_error(reached_events)
    loc_err = localization_error(accuracy_log)
    cte = cross_track_error(accuracy_log, centerlines)

    summary = build_summary_table(term_wp, loc_err, cte)

    pd.set_option("display.float_format", lambda v: f"{v:.4f}")
    print()
    print(summary.to_string(index=False))

    summary.to_csv(args.output, index=False)
    print(f"\nSaved summary to {args.output}")


if __name__ == "__main__":
    main()
