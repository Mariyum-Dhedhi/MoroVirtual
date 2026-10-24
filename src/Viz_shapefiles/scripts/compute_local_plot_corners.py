from __future__ import annotations

import argparse
import json
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]

DATUM_EASTING = 357388.0403
DATUM_NORTHING = 5610173.5711


def load_local_corners(geojson_path: Path) -> dict[int, list[tuple[float, float]]]:
    with open(geojson_path) as f:
        data = json.load(f)

    corners_by_id: dict[int, list[tuple[float, float]]] = {}
    for feature in data["features"]:
        plot_id = feature["properties"]["ID"]
        ring = feature["geometry"]["coordinates"][0]
        # Ring is closed (first point repeats as last); keep the 4 unique corners.
        corners = [
            (easting - DATUM_EASTING, northing - DATUM_NORTHING)
            for easting, northing in ring[:-1]
        ]
        corners_by_id[plot_id] = corners

    return corners_by_id


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Express each filtered plot's corners as local meters (X=east, "
            "Y=north) relative to the datum SW corner."
        )
    )
    parser.add_argument(
        "--geojson",
        type=Path,
        default=REPO_ROOT / "outputs" / "filtered_plots_230607.geojson",
        help="Path to the filtered plots GeoJSON (EPSG:32632).",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=REPO_ROOT / "outputs" / "plot_corners_local.json",
        help="Path to save the ID -> local corner dict as JSON.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    corners_by_id = load_local_corners(args.geojson)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with open(args.output, "w") as f:
        json.dump({str(k): v for k, v in corners_by_id.items()}, f, indent=2)

    print(f"Computed local-frame corners for {len(corners_by_id)} plots")
    print(f"Saved to: {args.output}")

    if 0 in corners_by_id:
        sanity_id = 0
    else:
        sanity_id = min(corners_by_id)
        print(
            f"NOTE: plot ID=0 is not in the filtered subset "
            f"(kept IDs range {min(corners_by_id)}-{max(corners_by_id)}); "
            f"using ID={sanity_id} for the sanity check instead."
        )

    print(f"\nCorners of plot ID={sanity_id} (local meters, X=east, Y=north from datum):")
    for corner in corners_by_id[sanity_id]:
        print(f"  ({corner[0]:.3f}, {corner[1]:.3f})")


if __name__ == "__main__":
    main()
