from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
from pyproj import Transformer

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from orthophoto_reader import OrthophotoReader
from shapefile_reader import ShapefileReader


def _looks_like_lon_lat(bounds: tuple[float, float, float, float]) -> bool:
    xmin, ymin, xmax, ymax = bounds
    return all(
        [
            -180.0 <= xmin <= 180.0,
            -180.0 <= xmax <= 180.0,
            -90.0 <= ymin <= 90.0,
            -90.0 <= ymax <= 90.0,
        ]
    )


def _reproject_polygons(
    polygons: list[list[list[tuple[float, float]]]],
    source_crs: str,
    target_crs: str,
) -> list[list[list[tuple[float, float]]]]:
    transformer = Transformer.from_crs(source_crs, target_crs, always_xy=True)
    projected: list[list[list[tuple[float, float]]]] = []

    for polygon in polygons:
        projected_polygon: list[list[tuple[float, float]]] = []
        for ring in polygon:
            projected_ring = [transformer.transform(x, y) for x, y in ring]
            projected_polygon.append(projected_ring)
        projected.append(projected_polygon)

    return projected


def plot_overlay(
    orthophoto_path: Path,
    shapefile_path: Path,
    output_path: Path | None = None,
    show: bool = True,
) -> None:
    orthophoto = OrthophotoReader(orthophoto_path).read()
    shapes = ShapefileReader(shapefile_path).read()

    if orthophoto.image is None or orthophoto.extent is None:
        raise ValueError("Orthophoto could not be loaded")

    polygons_to_plot = shapes.polygons
    if orthophoto.crs and _looks_like_lon_lat(shapes.bounds):
        polygons_to_plot = _reproject_polygons(shapes.polygons, "EPSG:4326", orthophoto.crs)
        print(f"Reprojected shapefile coordinates from EPSG:4326 to {orthophoto.crs}")

    fig, ax = plt.subplots(figsize=(12, 9))

    ax.imshow(
        orthophoto.image,
        extent=orthophoto.extent,
        origin="upper",
    )

    for polygon in polygons_to_plot:
        for ring in polygon:
            if len(ring) < 2:
                continue
            xs = [point[0] for point in ring]
            ys = [point[1] for point in ring]
            ax.plot(xs, ys, color="yellow", linewidth=1.3)

    ax.set_title("Orthophoto with Shapefile Overlay")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_aspect("equal", adjustable="box")

    if output_path is not None:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_path, dpi=300, bbox_inches="tight")
        print(f"Saved figure to: {output_path}")

    if show:
        plt.show()
    else:
        plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot an orthophoto with a shapefile overlay."
    )
    parser.add_argument(
        "--orthophoto",
        type=Path,
        default=REPO_ROOT / "orthophoto" / "raster_data_UAV3-RGB" / "raster_data" / "UAV3-RGB" / "230607_ortho.tif",
        help="Path to the orthophoto GeoTIFF.",
    )
    parser.add_argument(
        "--shapefile",
        type=Path,
        default=REPO_ROOT / "shapefiles" / "2023" / "PhenorobField_2023_lla.shp",
        help="Path to the shapefile (.shp).",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=REPO_ROOT / "outputs" / "orthophoto_with_shapefile.png",
        help="Path to save the resulting figure.",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not open an interactive plot window.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    plot_overlay(
        orthophoto_path=args.orthophoto,
        shapefile_path=args.shapefile,
        output_path=args.output,
        show=not args.no_show,
    )


if __name__ == "__main__":
    main()
