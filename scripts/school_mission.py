"""Measured landing, recipient handoff and return missions, independent of ROS/PX4."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal, TypedDict, assert_never

from school_mission_plan import InvalidMission, MissionPlan, Point, point

__all__ = [
    "InvalidMission",
    "Mission",
    "MissionPlan",
    "MissionSnapshot",
    "Phase",
    "Point",
    "enu_to_ned",
    "point",
]

Phase = Literal[
    "outbound",
    "delivery_landing",
    "awaiting_receipt",
    "return_takeoff",
    "returning",
    "landing",
    "completed",
]


class MissionSnapshot(TypedDict):
    order_id: str
    destination_id: str
    destination_enu: Point
    phase: Phase
    route_enu: list[Point]
    waypoint_index: int
    remaining_distance_m: float
    eta_seconds: float | None
    arrived: bool
    received: bool
    delivery_distance_m: float
    delivery_eta_seconds: float | None
    cruise_speed_mps: float


def enu_to_ned(point: Point) -> Point:
    return point[1], point[0], -point[2]


@dataclass(slots=True)  # noqa: MUTABLE_OK
class Mission:
    """Mutate measured waypoint and recipient receipt progress for one order."""

    order_id: str
    plan: MissionPlan
    waypoint_index: int = 0
    phase: Phase = "outbound"
    arrived: bool = False
    received: bool = False

    @property
    def target(self) -> Point:
        return self.plan.route[self.waypoint_index]

    def step(
        self,
        position: Point,
        *,
        armed: bool,
        authorized: bool,
        now: float,
        received: bool = False,
    ) -> None:
        """Advance only with live authorization and measured proximity to each target."""
        if not authorized:
            return
        match self.phase:
            case "delivery_landing":
                if not armed and math.dist(position, self.target) <= 0.7:
                    self.arrived = True
                    self.phase = "awaiting_receipt"
            case "awaiting_receipt":
                if not armed and received and math.dist(position, self.target) <= 0.7:
                    self.received = True
                    self.phase = "return_takeoff"
                    self.waypoint_index = self.plan.return_takeoff_index
            case "outbound":
                if armed and math.dist(position, self.target) <= 0.6:
                    self.waypoint_index += 1
                    if self.waypoint_index == self.plan.delivery_index:
                        self.phase = "delivery_landing"
            case "return_takeoff":
                if armed and math.dist(position, self.target) <= 0.6:
                    self.phase = "returning"
                    self.waypoint_index += 1
            case "returning":
                if armed and math.dist(position, self.target) <= 0.6:
                    self.waypoint_index += 1
                    if self.waypoint_index == len(self.plan.route) - 1:
                        self.phase = "landing"
            case "landing" | "completed":
                pass
            case unreachable:
                assert_never(unreachable)

    def complete_landing(self, position: Point, *, armed: bool) -> bool:
        """Disarming elsewhere or before arrival never completes this mission."""
        if (
            self.phase == "landing"
            and self.arrived
            and self.received
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
        # Human handoff has no predictable duration; never invent a return ETA.
        eta = remaining / self.plan.speed_mps if authorized and self.received else None
        if self.phase == "completed":
            eta = 0.0
        delivery_distance = (
            0.0
            if self.arrived
            else math.dist(position, self.target)
            + sum(
                math.dist(a, b)
                for a, b in zip(
                    self.plan.route[self.waypoint_index : self.plan.delivery_index + 1],
                    self.plan.route[
                        self.waypoint_index + 1 : self.plan.delivery_index + 1
                    ],
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
            destination_enu=self.plan.route[self.plan.delivery_index],
            phase=self.phase,
            route_enu=list(self.plan.route),
            waypoint_index=self.waypoint_index,
            remaining_distance_m=remaining,
            eta_seconds=eta,
            arrived=self.arrived,
            received=self.received,
            delivery_distance_m=delivery_distance,
            delivery_eta_seconds=delivery_eta,
            cruise_speed_mps=self.plan.speed_mps,
        )
