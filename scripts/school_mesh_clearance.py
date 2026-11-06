"""Conservative world-space height raster for the unchanged binary campus STL."""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Final

XY = tuple[float, float]
Point = tuple[float, float, float]
Cell = tuple[int, int]
RESOLUTION: Final = 0.2


class UnsafeRoute(ValueError):
    """Geometry cannot authorize a route or landing column."""


@dataclass(frozen=True, slots=True)
class MeshPlacement:
    path: Path
    scale: Point
    translation: Point


@dataclass(frozen=True, slots=True)
class HeightMap:
    """Triangle bounding boxes overestimate occupied space, including thin walls."""

    heights: dict[Cell, float]

    def height_near(self, position: XY, radius: float) -> float:
        """Maximum triangle top in any raster square touching the XY disk."""
        x, y = position
        highest = -math.inf
        for i in range(
            math.floor((x - radius) / RESOLUTION),
            math.floor((x + radius) / RESOLUTION) + 1,
        ):
            dx = max(i * RESOLUTION - x, x - (i + 1) * RESOLUTION, 0)
            for j in range(
                math.floor((y - radius) / RESOLUTION),
                math.floor((y + radius) / RESOLUTION) + 1,
            ):
                dy = max(j * RESOLUTION - y, y - (j + 1) * RESOLUTION, 0)
                if dx * dx + dy * dy <= radius * radius:
                    highest = max(highest, self.heights.get((i, j), -math.inf))
        return highest


def read_height_map(placement: MeshPlacement) -> HeightMap:
    """Rasterize every triangle conservatively from the current source bytes."""
    raw = placement.path.read_bytes()
    if len(raw) < 84 or len(raw) != 84 + 50 * struct.unpack_from("<I", raw, 80)[0]:
        raise UnsafeRoute("Campus collision mesh must be a complete binary STL")
    heights: dict[Cell, float] = {}
    for triangle in struct.iter_unpack("<12fH", memoryview(raw)[84:]):
        vertices = tuple(
            tuple(
                triangle[3 + vertex * 3 + axis] * placement.scale[axis]
                + placement.translation[axis]
                for axis in range(3)
            )
            for vertex in range(3)
        )
        if any(not math.isfinite(value) for vertex in vertices for value in vertex):
            raise UnsafeRoute("Campus collision mesh contains non-finite coordinates")
        top = max(vertex[2] for vertex in vertices)
        for i in range(
            math.floor(min(v[0] for v in vertices) / RESOLUTION),
            math.floor(max(v[0] for v in vertices) / RESOLUTION) + 1,
        ):
            for j in range(
                math.floor(min(v[1] for v in vertices) / RESOLUTION),
                math.floor(max(v[1] for v in vertices) / RESOLUTION) + 1,
            ):
                heights[i, j] = max(top, heights.get((i, j), -math.inf))
    return HeightMap(heights)


def projected_distance(vertices: tuple[Point, ...], point: XY) -> float:
    """Distance to a projected triangle, including degenerate vertical faces."""
    crosses = []
    distance = math.inf
    for start, end in zip(vertices, (*vertices[1:], vertices[0]), strict=True):
        dx, dy = end[0] - start[0], end[1] - start[1]
        px, py = point[0] - start[0], point[1] - start[1]
        crosses.append(dx * py - dy * px)
        denominator = dx * dx + dy * dy
        fraction = (
            min(1, max(0, (px * dx + py * dy) / denominator)) if denominator else 0
        )
        distance = min(distance, math.hypot(px - fraction * dx, py - fraction * dy))
    a, b, c = vertices
    area = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    if abs(area) > 1e-12 and (min(crosses) >= 0 or max(crosses) <= 0):
        return 0.0
    return distance


def column_height(placement: MeshPlacement, position: XY, radius: float) -> float:
    """Conservative triangle top in an exact projected disk, without bbox false hits."""
    raw = placement.path.read_bytes()
    highest = -math.inf
    for triangle in struct.iter_unpack("<12fH", memoryview(raw)[84:]):
        vertices = tuple(
            tuple(
                triangle[3 + vertex * 3 + axis] * placement.scale[axis]
                + placement.translation[axis]
                for axis in range(3)
            )
            for vertex in range(3)
        )
        if any(
            min(v[axis] for v in vertices) > position[axis] + radius
            or max(v[axis] for v in vertices) < position[axis] - radius
            for axis in (0, 1)
        ):
            continue
        if projected_distance(vertices, position) <= radius:
            highest = max(highest, max(v[2] for v in vertices))
    return highest
