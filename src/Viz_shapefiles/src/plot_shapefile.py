from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt

from shapefile_reader import ShapefileReader


def plot_field(shapefile_path: Path) -> None:
    reader = ShapefileReader(shapefile_path).read()

    fig, ax = plt.subplots(figsize=(10, 8))

    for index, shape in enumerate(reader.shapes, start=1):
        xs = [point[0] for point in shape]
        ys = [point[1] for point in shape]
        ax.plot(xs, ys, color="black", linewidth=1)
        ax.fill(xs, ys, alpha=0.08, color="tab:green")

        centroid_x = sum(xs[:-1]) / max(len(xs) - 1, 1)
        centroid_y = sum(ys[:-1]) / max(len(ys) - 1, 1)
        ax.text(centroid_x, centroid_y, str(index), fontsize=8, ha="center", va="center")

    ax.set_title(shapefile_path.stem)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.4)
    plt.tight_layout()
    plt.show()


def main() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    default_shapefile = repo_root / "shapefiles" / "2023" / "PhenorobField_2023_lla.shp"

    shapefile_path = Path(sys.argv[1]) if len(sys.argv) > 1 else default_shapefile
    plot_field(shapefile_path)


if __name__ == "__main__":
    main()