from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from orthophoto_reader import OrthophotoReader

DATUM_EASTING = 357388.0403
DATUM_NORTHING = 5610173.5711


def load_filtered_plots(path: Path) -> list[dict]:
    with open(path) as f:
        return json.load(f)["features"]


def load_waypoints(path: Path) -> list[dict]:
    waypoints = []
    with open(path) as f:
        for row in csv.DictReader(f):
            waypoints.append(
                {
                    "index": int(row["index"]),
                    "col_idx": int(row["col_idx"]),
                    "row_idx": int(row["row_idx"]),
                    "band": int(row["band"]),
                    "plot_id": row["plot_id"],
                    "x": float(row["x"]) + DATUM_EASTING,
                    "y": float(row["y"]) + DATUM_NORTHING,
                    "point_type": row["point_type"],
                }
            )
    return waypoints


def plot_check(
    orthophoto_path: Path,
    plots_path: Path,
    waypoints_path: Path,
    output_path: Path,
    show: bool = False,
) -> None:
    orthophoto = OrthophotoReader(orthophoto_path).read()
    plots = load_filtered_plots(plots_path)
    waypoints = load_waypoints(waypoints_path)

    fig, ax = plt.subplots(figsize=(14, 10))
    ax.imshow(orthophoto.image, extent=orthophoto.extent, origin="upper")

    for feature in plots:
        ring = feature["geometry"]["coordinates"][0]
        xs = [p[0] for p in ring]
        ys = [p[1] for p in ring]
        ax.plot(xs, ys, color="yellow", linewidth=1.0, zorder=4)

    xs = [wp["x"] for wp in waypoints]
    ys = [wp["y"] for wp in waypoints]
    ax.plot(xs, ys, color="blue", linewidth=0.5, zorder=5)

    # Direction arrows along the path, spread out to stay legible.
    n = len(waypoints)
    arrow_step = max(1, n // 60)
    for i in range(0, n - 1, arrow_step):
        x0, y0 = xs[i], ys[i]
        x1, y1 = xs[i + 1], ys[i + 1]
        if (x0, y0) == (x1, y1):
            continue
        ax.annotate(
            "",
            xy=(x1, y1),
            xytext=(x0, y0),
            arrowprops=dict(arrowstyle="->", color="blue", lw=0.8),
            zorder=6,
        )

    entry_overshoot = [wp for wp in waypoints if wp["point_type"] in ("entry", "overshoot")]
    row_ends = [wp for wp in waypoints if wp["point_type"] in ("row_start", "row_end")]
    headland = [wp for wp in waypoints if wp["point_type"] == "headland"]
    ax.scatter(
        [wp["x"] for wp in entry_overshoot], [wp["y"] for wp in entry_overshoot],
        color="orange", s=8, zorder=7, label="entry/overshoot",
    )
    ax.scatter(
        [wp["x"] for wp in row_ends], [wp["y"] for wp in row_ends],
        color="red", s=5, zorder=8, label="row_start/row_end",
    )
    ax.scatter(
        [wp["x"] for wp in headland], [wp["y"] for wp in headland],
        color="green", s=8, zorder=9, label="headland",
    )
    ax.legend(loc="upper right", fontsize=9)

    ax.set_title("Boustrophedon Waypoint Path Check")
    ax.set_xlabel("Easting (m, EPSG:32632)")
    ax.set_ylabel("Northing (m, EPSG:32632)")
    ax.set_aspect("equal", adjustable="box")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=300, bbox_inches="tight")
    print(f"Saved figure to: {output_path}")

    if show:
        plt.show()
    else:
        plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Overlay filtered plot boundaries and the boustrophedon "
        "waypoint path on the orthophoto for a visual sanity check."
    )
    parser.add_argument(
        "--orthophoto",
        type=Path,
        default=REPO_ROOT
        / "orthophoto"
        / "raster_data_UAV3-RGB"
        / "raster_data"
        / "UAV3-RGB"
        / "230607_ortho.tif",
        help="Path to the orthophoto GeoTIFF.",
    )
    parser.add_argument(
        "--plots",
        type=Path,
        default=REPO_ROOT / "outputs" / "filtered_plots_230607.geojson",
        help="Path to the filtered plots GeoJSON (EPSG:32632).",
    )
    parser.add_argument(
        "--waypoints",
        type=Path,
        default=REPO_ROOT / "outputs" / "waypoints.csv",
        help="Path to the waypoints CSV (local meters, offset by the datum).",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=REPO_ROOT / "outputs" / "waypoints_check.png",
        help="Path to save the resulting figure.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Open an interactive plot window.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    plot_check(
        orthophoto_path=args.orthophoto,
        plots_path=args.plots,
        waypoints_path=args.waypoints,
        output_path=args.output,
        show=args.show,
    )


if __name__ == "__main__":
    main()
