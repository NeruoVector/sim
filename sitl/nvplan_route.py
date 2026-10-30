"""Read selected route from the exact native-validated NVPLAN2 text."""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class GeoWaypoint:
    lat: float
    lon: float
    altitude_m: float


def selected_route(plan_text: str) -> tuple[list[GeoWaypoint], list[GeoWaypoint], int]:
    lines = iter(plan_text.splitlines())
    try:
        header = next(lines).split()
        if len(header) != 6 or header[0] != "NVPLAN2":
            raise ValueError("unsupported plan header")
        selected, candidate_count = int(header[4]), int(header[5])
        if not 0 <= selected < candidate_count <= 16:
            raise ValueError("invalid candidate selection")
        next(lines)  # Native validator checks the constraint record.
        next(lines)  # Native validator checks the optional window record.
        def read_points(count: int) -> list[GeoWaypoint]:
            if not 2 <= count <= 256:
                raise ValueError("invalid waypoint count")
            result = []
            for _ in range(count):
                parts = next(lines).split()
                if len(parts) != 3:
                    raise ValueError("invalid waypoint record")
                lat, lon, alt = map(float, parts)
                if not all(math.isfinite(v) for v in (lat, lon, alt)):
                    raise ValueError("non-finite waypoint")
                result.append(GeoWaypoint(lat, lon, alt))
            return result
        required = read_points(int(next(lines)))
        candidates = [read_points(int(next(lines))) for _ in range(candidate_count)]
        return required, candidates[selected], selected
    except (StopIteration, IndexError) as exc:
        raise ValueError("truncated mission") from exc


def required_indices(required: list[GeoWaypoint], route: list[GeoWaypoint]) -> tuple[int, ...]:
    """Locate every required point in order using native validator tolerances."""
    positions = []
    cursor = 0
    for target in required:
        while cursor < len(route):
            candidate = route[cursor]
            index = cursor
            cursor += 1
            if (abs(target.lat - candidate.lat) < 1e-10
                    and abs(target.lon - candidate.lon) < 1e-10
                    and abs(target.altitude_m - candidate.altitude_m) < 1e-6):
                positions.append(index)
                break
        else:
            raise ValueError("candidate omits or reorders a required stop")
    if not positions or positions[0] != 0 or positions[-1] != len(route) - 1:
        raise ValueError("candidate endpoints differ from required route")
    return tuple(positions)
