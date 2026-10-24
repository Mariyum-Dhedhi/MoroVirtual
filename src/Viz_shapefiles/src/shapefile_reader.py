from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import shapefile


@dataclass
class ShapefileReader:
    """Read polygon shapefiles and expose geometry and attributes."""

    path: Path
    shapes: list[list[tuple[float, float]]] = field(default_factory=list)
    polygons: list[list[list[tuple[float, float]]]] = field(default_factory=list)
    records: list[dict[str, Any]] = field(default_factory=list)

    def read(self) -> "ShapefileReader":
        reader = shapefile.Reader(str(self.path))
        fields = [field[0] for field in reader.fields[1:]]

        self.shapes.clear()
        self.polygons.clear()
        self.records.clear()

        for shape_record in reader.shapeRecords():
            points = [(float(x), float(y)) for x, y in shape_record.shape.points]
            parts = list(shape_record.shape.parts) + [len(shape_record.shape.points)]

            polygon_parts: list[list[tuple[float, float]]] = []
            for index in range(len(parts) - 1):
                start = parts[index]
                end = parts[index + 1]
                polygon_parts.append(points[start:end])

            self.shapes.append(points)
            self.polygons.append(polygon_parts)
            self.records.append(dict(zip(fields, shape_record.record)))

        return self

    @property
    def bounds(self) -> tuple[float, float, float, float]:
        if not self.shapes:
            raise ValueError("No shapes loaded")

        xs = [x for shape in self.shapes for x, _ in shape]
        ys = [y for shape in self.shapes for _, y in shape]
        return min(xs), min(ys), max(xs), max(ys)
