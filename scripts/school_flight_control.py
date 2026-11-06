"""One bounded simulation flight per live API order; idle without an assignment."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from uuid import uuid4
from school_mission import Point, enu_to_ned


def position_setpoint_ned(reference: Point, measured: Point, local: Point) -> Point:
    """Align a world target to the current estimator frame, including EKF drift."""
    error = enu_to_ned(tuple(a - b for a, b in zip(reference, measured, strict=True)))
    return tuple(a + b for a, b in zip(local, error, strict=True))


def route_heading(target: Point, position: Point, previous: float) -> float:
    """Face the horizontal travel direction; keep heading during a vertical leg."""
    east, north = target[0] - position[0], target[1] - position[1]
    return math.atan2(east, north) if math.hypot(east, north) >= 0.5 else previous


def landing_aligned(position: Point, target: Point) -> bool:
    """Require a centered landing column before handing descent to PX4."""
    return math.dist(position[:2], target[:2]) <= 0.08


@dataclass(frozen=True, slots=True)
class OrderAssignment:
    timestamp: float
    order_id: str | None
    destination_id: str | None = None
    received: bool = False

    @classmethod
    def read(cls, path: Path) -> OrderAssignment:
        raw = json.loads(path.read_text())
        if not isinstance(raw, dict):
            raise TypeError("Order assignment must be an object")
        stamp, order = raw.get("timestamp"), raw.get("order_id")
        if not isinstance(stamp, (int, float)) or isinstance(stamp, bool):
            raise TypeError("Assignment timestamp must be numeric")
        if order is not None and not isinstance(order, str):
            raise ValueError("Order ID must be a string or null")
        destination = raw.get("destination_id")
        if destination is not None and not isinstance(destination, str):
            raise ValueError("Destination ID must be a string or null")
        received = raw.get("received", False)
        if not isinstance(received, bool):
            raise TypeError("Receipt must be boolean")
        return cls(float(stamp), order, destination, received)


@dataclass(slots=True)
class FlightControl:
    """Own mutable assignment/completion state; repeated assignments never repeat a flight."""

    controller_id: str = field(default_factory=lambda: str(uuid4()))
    order_id: str | None = None
    completed_order_id: str | None = None
    mode: str = "waiting_for_order"

    def update(
        self,
        assignment: OrderAssignment | None,
        *,
        fresh: bool,
        blocked: bool,
        now: float,
    ) -> bool:
        if assignment is None or not 0 <= now - assignment.timestamp <= 3:
            self.mode = "hold_order_stale" if self.order_id else "waiting_for_order"
            return False
        self.order_id = assignment.order_id
        if self.order_id is None:
            self.mode = "waiting_for_order"
            return False
        if self.order_id == self.completed_order_id:
            self.mode = "order_completed"
            return False
        if not fresh or blocked:
            self.mode = "hold_vision_stale" if not fresh else "hold_obstacle"
            return False
        self.mode = "flying"
        return True

    def complete(self) -> None:
        self.completed_order_id = self.order_id
        self.mode = "order_completed"
