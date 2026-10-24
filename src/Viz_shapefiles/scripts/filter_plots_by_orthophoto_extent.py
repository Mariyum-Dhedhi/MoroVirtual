from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import rasterio
from pyproj import Transformer

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from shapefile_reader import ShapefileReader

Ring = list[tuple[float, float]]
Polygon = list[Ring]


def _reproject_polygon(polygon: Polygon, transformer: Transformer) -> Polygon:
    return [[transformer.transform(x, y) for x, y in ring] for ring in polygon]


def _polygon_within_bounds(
    polygon: Polygon, bounds: tuple[float, float, float, float]
) -> bool:
    left, bottom, right, top = bounds
    for ring in polygon:
        for x, y in ring:
            if not (left <= x <= right and bottom <= y <= top):
                return False
    return True


def filter_plots(
    shapefile_path: Path,
    orthophoto_path: Path,
    output_path: Path,
) -> int:
    shapes = ShapefileReader(shapefile_path).read()

    with rasterio.open(orthophoto_path) as dataset:
        raster_crs = dataset.crs
        raster_bounds = dataset.bounds

    target_crs = "EPSG:32632"
    transformer = Transformer.from_crs("EPSG:4326", target_crs, always_xy=True)
    bounds = (
        raster_bounds.left,
        raster_bounds.bottom,
        raster_bounds.right,
        raster_bounds.top,
    )

    features = []
    for polygon, record in zip(shapes.polygons, shapes.records):
        projected_polygon = _reproject_polygon(polygon, transformer)
        if not _polygon_within_bounds(projected_polygon, bounds):
            continue

        features.append(
            {
                "type": "Feature",
                "properties": {"ID": record["ID"]},
                "geometry": {
                    "type": "Polygon",
                    "coordinates": [
                        [[x, y] for x, y in ring] for ring in projected_polygon
                    ],
                },
            }
        )

    geojson = {
        "type": "FeatureCollection",
        "crs": {
            "type": "name",
            "properties": {"name": f"urn:ogc:def:crs:{target_crs.replace(':', '::')}"},
        },
        "features": features,
    }

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w") as f:
        json.dump(geojson, f, indent=2)

    return len(features)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Reproject shapefile plots to EPSG:32632 and keep only those "
            "fully within an orthophoto's bounding box."
        )
    )
    parser.add_argument(
        "--shapefile",
        type=Path,
        default=REPO_ROOT / "shapefiles" / "2023" / "PhenorobField_2023_lla.shp",
        help="Path to the source shapefile (.shp), in EPSG:4326.",
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
        help="Path to the orthophoto GeoTIFF whose bounds define the filter.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=REPO_ROOT / "outputs" / "filtered_plots_230607.geojson",
        help="Path to save the filtered subset as GeoJSON.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    total = len(ShapefileReader(args.shapefile).read().shapes)
    kept = filter_plots(args.shapefile, args.orthophoto, args.output)
    print(f"Kept {kept} of {total} plots fully within the orthophoto extent")
    print(f"Saved to: {args.output}")
    print(f"Kept {kept} plots within orthophoto extent.")


if __name__ == "__main__":
    main()
