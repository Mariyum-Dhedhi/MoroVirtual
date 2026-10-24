from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rasterio


@dataclass
class OrthophotoReader:
    """Read a georeferenced orthophoto and prepare it for plotting."""

    path: Path
    image: np.ndarray | None = None
    extent: tuple[float, float, float, float] | None = None
    crs: str | None = None

    def read(self) -> "OrthophotoReader":
        with rasterio.open(self.path) as dataset:
            raster = dataset.read()
            bounds = dataset.bounds
            self.extent = (bounds.left, bounds.right, bounds.bottom, bounds.top)
            self.crs = str(dataset.crs) if dataset.crs else None

        # Convert raster bands (C, H, W) into a Matplotlib-friendly format.
        if raster.shape[0] == 1:
            image = raster[0]
        else:
            image = np.moveaxis(raster[:3], 0, -1)

        self.image = self._normalize_for_display(image)
        return self

    @staticmethod
    def _normalize_for_display(image: np.ndarray) -> np.ndarray:
        if image.dtype == np.uint8:
            return image

        image_float = image.astype(np.float32)
        max_value = float(np.nanmax(image_float)) if image_float.size else 0.0
        if max_value <= 0.0:
            return image_float

        return np.clip(image_float / max_value, 0.0, 1.0)
