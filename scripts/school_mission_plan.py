"""Read and validate offline-verified campus flight plans."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

from school_route_manifest import route_signature

Point = tuple[float, float, float]


class InvalidMission(ValueError):
    """A fixture route cannot safely authorize flight."""


def point(value: list[float]) -> Point:
    """Parse a finite three-dimensional fixture position."""
    if len(value) != 3 or any(
        isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v)
        for v in value
    ):
        raise InvalidMission("Expected three finite position coordinates")
    return float(value[0]), float(value[1]), float(value[2])


@dataclass(frozen=True, slots=True)
class MissionPlan:
    destination_id: str
    route: tuple[Point, ...]
    speed_mps: float
    arrival_hold_seconds: float
    delivery_index: int

    @property
    def return_takeoff_index(self) -> int:
        return self.delivery_index + 1

    @property
    def home_approach_index(self) -> int:
        return len(self.route) - 2

    @classmethod
    def read(cls, fixture: Path, destination_id: str | None) -> MissionPlan:
        """Reject missing destinations, unsafe cruise and points outside the campus."""
        try:
            data = json.loads(fixture.read_text())
            config = data["delivery_missions"]
            destinations = config["destinations"]
            if data["synthetic"] is not True or destination_id not in destinations:
                raise InvalidMission("Unmapped synthetic destination")
            home = point(data["takeoff_enu"])
            destination = point(destinations[destination_id])
            landing = point(config["landing_points"][destination_id])
            altitude = destination[2]
            minimum_altitude = config["cruise_altitude_m"]
            speed = config["speed_mps"]
            dwell = config["arrival_hold_seconds"]
            if any(
                isinstance(v, bool)
                or not isinstance(v, (int, float))
                or not math.isfinite(v)
                for v in (minimum_altitude, altitude, speed, dwell)
            ):
                raise InvalidMission("Non-finite mission configuration")
            if (
                not 3
                < minimum_altitude
                <= altitude
                < min(120, data["drone"]["max_altitude_m"])
            ):
                raise InvalidMission("Cruise altitude is outside the planning envelope")
            if (
                not 0 < speed <= data["drone"]["cruise_speed_mps"]
                or not 0 <= dwell <= 60
            ):
                raise InvalidMission("Invalid mission speed or hover duration")
            center = point(data["campus"]["ground_center_enu"])
            size = data["campus"]["size_m"]
            if destination[:2] != landing[:2] or not 0 <= landing[2] < altitude:
                raise InvalidMission("Landing must be below the destination approach")
            if any(
                abs(p[axis] - center[axis]) > size[axis] / 2
                for p in (home, destination, landing)
                for axis in (0, 1)
            ):
                raise InvalidMission("Route leaves the synthetic campus")
            mesh_uri = data["campus"]["mesh_uri"]
            if mesh_uri != "model://school-demo/meshes/school_map.stl":
                raise InvalidMission("Unrecognized campus collision mesh")
            mesh = (
                Path(__file__).resolve().parents[1]
                / "models/environments/school-demo/meshes/school_map.stl"
            )
            signed_config = {
                key: value for key, value in config.items() if key != "route_signature"
            }
            manifest = json.dumps(
                [
                    data["campus"],
                    data["buildings"],
                    home,
                    data["drone"]["radius_m"],
                    signed_config,
                ],
                sort_keys=True,
                separators=(",", ":"),
            )
            if route_signature(mesh, manifest) != config["route_signature"]:
                raise InvalidMission(
                    "Offline route verification is stale; regenerate routes"
                )
            outbound = tuple(point(p) for p in config["routes_enu"][destination_id])
            if (
                len(outbound) < 2
                or 2 * len(outbound) + 2 > 32
                or outbound[0] != (home[0], home[1], altitude)
                or outbound[-1] != destination
                or any(p[2] != altitude for p in outbound)
                or any(
                    abs(p[axis] - center[axis]) > size[axis] / 2
                    for p in outbound
                    for axis in (0, 1)
                )
            ):
                raise InvalidMission("Invalid verified outbound route")
            route = (*outbound, landing, *reversed(outbound), home)
            return cls(
                str(destination_id), route, float(speed), float(dwell), len(outbound)
            )
        except (OSError, KeyError, TypeError, ValueError) as error:
            raise InvalidMission(str(error)) from error
