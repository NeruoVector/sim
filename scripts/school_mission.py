"""Validated campus hover-delivery missions, independent of ROS and PX4."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Literal, TypedDict

Point = tuple[float, float, float]
Phase = Literal["outbound", "arrival_hold", "returning", "landing", "completed"]


class MissionSnapshot(TypedDict):
    order_id: str
    destination_id: str
    phase: Phase
    route_enu: list[Point]
    waypoint_index: int
    remaining_distance_m: float
    eta_seconds: float | None
    arrived: bool
    delivery_distance_m: float
    delivery_eta_seconds: float | None
    cruise_speed_mps: float


class InvalidMission(ValueError):
    """A fixture route cannot safely authorize flight."""


def enu_to_ned(point: Point) -> Point:
    return point[1], point[0], -point[2]


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
            altitude = config["cruise_altitude_m"]
            speed = config["speed_mps"]
            dwell = config["arrival_hold_seconds"]
            if any(
                isinstance(v, bool)
                or not isinstance(v, (int, float))
                or not math.isfinite(v)
                for v in (altitude, speed, dwell)
            ):
                raise InvalidMission("Non-finite mission configuration")
            ceiling = max(b["no_fly_ceiling_m"] for b in data["buildings"])
            if not ceiling < altitude < min(120, data["drone"]["max_altitude_m"]):
                raise InvalidMission("Cruise altitude must clear the campus ceiling")
            if (
                not 0 < speed <= data["drone"]["cruise_speed_mps"]
                or not 0 <= dwell <= 60
            ):
                raise InvalidMission("Invalid mission speed or hover duration")
            center = point(data["campus"]["ground_center_enu"])
            size = data["campus"]["size_m"]
            route = (
                (home[0], home[1], float(altitude)),
                destination,
                (home[0], home[1], float(altitude)),
                home,
            )
            if destination[2] != altitude:
                raise InvalidMission("Destination must remain at safe cruise altitude")
            if any(
                abs(p[axis] - center[axis]) > size[axis] / 2
                for p in route
                for axis in (0, 1)
            ):
                raise InvalidMission("Route leaves the synthetic campus")
            # The launch column must be clear of the known school building footprint.
            for building in data["buildings"]:
                if all(
                    abs(home[i] - building["center_enu"][i])
                    <= building["size_m"][i] / 2 + 1
                    for i in (0, 1)
                ):
                    raise InvalidMission("Launch column intersects a building")
            return cls(str(destination_id), route, float(speed), float(dwell))
        except (OSError, KeyError, TypeError, ValueError) as error:
            raise InvalidMission(str(error)) from error


@dataclass(slots=True)
class Mission:
    """Own waypoint and dwell progress for exactly one order."""

    order_id: str
    plan: MissionPlan
    waypoint_index: int = 0
    phase: Phase = "outbound"
    arrived: bool = False
    dwell_elapsed: float = 0
    last_step: float | None = None

    @property
    def target(self) -> Point:
        return self.plan.route[self.waypoint_index]

    def step(
        self, position: Point, *, armed: bool, authorized: bool, now: float
    ) -> None:
        """Advance only with live authorization and measured proximity to each target."""
        elapsed = 0 if self.last_step is None else max(0, min(now - self.last_step, 1))
        self.last_step = now
        if not authorized or not armed:
            return
        match self.phase:
            case "arrival_hold":
                if math.dist(position, self.target) <= 0.6:
                    self.dwell_elapsed += elapsed
                    if self.dwell_elapsed >= self.plan.arrival_hold_seconds:
                        self.arrived = True
                        self.phase = "returning"
                        self.waypoint_index = 2
                else:
                    self.dwell_elapsed = 0
            case "outbound":
                if math.dist(position, self.target) <= 0.6:
                    if self.waypoint_index == 0:
                        self.waypoint_index = 1
                    else:
                        self.phase = "arrival_hold"
                        self.dwell_elapsed = 0
            case "returning":
                if math.dist(position, self.target) <= 0.6:
                    self.phase = "landing"
                    self.waypoint_index = 3
            case "landing" | "completed":
                pass

    def complete_landing(self, position: Point, *, armed: bool) -> bool:
        """Disarming elsewhere or before arrival never completes this mission."""
        if (
            self.phase == "landing"
            and self.arrived
            and not armed
            and (
                math.dist(position[:2], self.plan.route[-1][:2]) <= 1
                and abs(position[2] - self.plan.route[-1][2]) <= 0.7
            )
        ):
            self.phase = "completed"
            return True
        return False

    def snapshot(self, position: Point, *, authorized: bool) -> MissionSnapshot:
        remaining = (
            0.0
            if self.phase == "completed"
            else math.dist(position, self.target)
            + sum(
                math.dist(a, b)
                for a, b in zip(
                    self.plan.route[self.waypoint_index :],
                    self.plan.route[self.waypoint_index + 1 :],
                    strict=False,
                )
            )
        )
        dwell = (
            0
            if self.arrived
            else max(0, self.plan.arrival_hold_seconds - self.dwell_elapsed)
        )
        eta = remaining / self.plan.speed_mps + dwell if authorized else None
        if self.phase == "completed":
            eta = 0.0
        delivery_distance = (
            0.0
            if self.arrived
            else math.dist(position, self.target)
            + sum(
                math.dist(a, b)
                for a, b in zip(
                    self.plan.route[self.waypoint_index : 2],
                    self.plan.route[self.waypoint_index + 1 : 2],
                    strict=False,
                )
            )
        )
        delivery_eta = delivery_distance / self.plan.speed_mps if authorized else None
        if self.phase == "completed":
            delivery_eta = 0.0
        return MissionSnapshot(
            order_id=self.order_id,
            destination_id=self.plan.destination_id,
            phase=self.phase,
            route_enu=list(self.plan.route),
            waypoint_index=self.waypoint_index,
            remaining_distance_m=remaining,
            eta_seconds=eta,
            arrived=self.arrived,
            delivery_distance_m=delivery_distance,
            delivery_eta_seconds=delivery_eta,
            cruise_speed_mps=self.plan.speed_mps,
        )
