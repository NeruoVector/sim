"""Convert local ENU and PX4 NED using an explicit synthetic geographic origin."""
from __future__ import annotations

import math
from dataclasses import dataclass

EARTH_RADIUS_M = 6_371_000.0


@dataclass(frozen=True, slots=True)
class Enu:
    east_m: float
    north_m: float
    up_m: float


@dataclass(frozen=True, slots=True)
class Ned:
    north_m: float
    east_m: float
    down_m: float


@dataclass(frozen=True, slots=True)
class Geographic:
    latitude_deg: float
    longitude_deg: float
    altitude_m: float


@dataclass(frozen=True, slots=True)
class Origin:
    """Origin altitude and input altitude must share the same declared datum."""

    latitude_deg: float
    longitude_deg: float
    altitude_m: float

    def __post_init__(self) -> None:
        if not all(math.isfinite(v) for v in (self.latitude_deg, self.longitude_deg, self.altitude_m)):
            raise ValueError("origin must be finite")
        if abs(self.latitude_deg) >= 89 or abs(self.longitude_deg) > 180:
            raise ValueError("origin outside local projection range")

    def to_enu(self, point: Geographic) -> Enu:
        if not all(math.isfinite(v) for v in (point.latitude_deg, point.longitude_deg, point.altitude_m)):
            raise ValueError("position must be finite")
        if abs(point.latitude_deg) > 90 or abs(point.longitude_deg) > 180:
            raise ValueError("invalid geographic coordinates")
        delta_lon = (point.longitude_deg - self.longitude_deg + 180) % 360 - 180
        return Enu(
            math.radians(delta_lon) * EARTH_RADIUS_M * math.cos(math.radians(self.latitude_deg)),
            math.radians(point.latitude_deg - self.latitude_deg) * EARTH_RADIUS_M,
            point.altitude_m - self.altitude_m,
        )

    def to_geographic(self, point: Enu) -> Geographic:
        if not all(math.isfinite(v) for v in (point.east_m, point.north_m, point.up_m)):
            raise ValueError("position must be finite")
        lon = self.longitude_deg + math.degrees(
            point.east_m / (EARTH_RADIUS_M * math.cos(math.radians(self.latitude_deg)))
        )
        return Geographic(
            self.latitude_deg + math.degrees(point.north_m / EARTH_RADIUS_M),
            (lon + 180) % 360 - 180,
            self.altitude_m + point.up_m,
        )


def enu_to_ned(point: Enu) -> Ned:
    return Ned(point.north_m, point.east_m, -point.up_m)


def ned_to_enu(point: Ned) -> Enu:
    return Enu(point.east_m, point.north_m, -point.down_m)


def require_synthetic_start(point: Enu) -> None:
    """Require a landed vehicle within two metres of the known fixture origin."""
    if (not all(math.isfinite(v) for v in (point.east_m, point.north_m, point.up_m))
            or math.hypot(point.east_m, point.north_m) > 2.0 or abs(point.up_m) > 0.75):
        raise ValueError("PX4 start does not match the synthetic origin")


def local_target(current_local: Ned, current_enu: Enu, target_enu: Enu) -> Ned:
    """Translate a target by current local/global delta, avoiding a stale origin offset."""
    return Ned(current_local.north_m + target_enu.north_m - current_enu.north_m,
               current_local.east_m + target_enu.east_m - current_enu.east_m,
               current_local.down_m - (target_enu.up_m - current_enu.up_m))


def horizontal_segment_error(point: Enu, start: Enu, end: Enu) -> float:
    """Distance to the finite verified horizontal segment, in metres."""
    dx, dy = end.east_m - start.east_m, end.north_m - start.north_m
    length2 = dx * dx + dy * dy
    fraction = max(0.0, min(1.0, ((point.east_m-start.east_m)*dx
                                   + (point.north_m-start.north_m)*dy) / length2)) if length2 else 0.0
    return math.hypot(point.east_m - (start.east_m + fraction*dx),
                      point.north_m - (start.north_m + fraction*dy))


def require_route_corridor(route: list[Enu], index: int, position: Enu, *, landing: bool) -> None:
    if landing:
        return
    if not 0 <= index < len(route):
        raise ValueError("invalid active route index")
    if (index > 0 and abs(route[index].up_m - route[index - 1].up_m) < 0.1
            and abs(position.up_m - route[index].up_m) > 1.0):
        raise ValueError("altitude departed the level-route safety corridor")
    if index > 0 and horizontal_segment_error(position, route[index - 1], route[index]) > 2.0:
        raise ValueError("position departed the verified segment safety corridor")
