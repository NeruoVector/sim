"""Recognize verified static-building clearance without inventing camera distances."""

from __future__ import annotations

import math
from pathlib import Path

from school_mission import InvalidMission, MissionPlan, Point
from school_relay_runtime import SimState


def segment_distance(position: Point, start: Point, end: Point) -> float:
    delta = tuple(b - a for a, b in zip(start, end, strict=True))
    length_squared = sum(value * value for value in delta)
    projection = sum((p - a) * d for p, a, d in zip(position, start, delta, strict=True))
    fraction = min(1.0, max(0.0, projection / length_squared)) if length_squared else 0.0
    nearest = tuple(a + fraction * d for a, d in zip(start, delta, strict=True))
    return math.dist(position, nearest)


def checked_building_corridor(state: SimState, fixture: Path) -> bool:
    """Use static mesh clearance only inside the exact certified leg's 0.6m envelope."""
    mission = state.mission
    if mission is None or mission.phase not in {"outbound", "returning", "delivery_landing", "landing"} or state.position_enu is None:
        return False
    try:
        plan = MissionPlan.read(fixture, mission.destination_id)
    except InvalidMission:
        return False
    if mission.route_enu is None or tuple(mission.route_enu) != plan.route:
        return False
    if not 0 < state.waypoint < len(plan.route):
        return False
    if mission.phase in {"delivery_landing", "landing"}:
        return math.dist(state.position_enu, plan.route[state.waypoint - 1]) <= 0.6
    return segment_distance(state.position_enu, plan.route[state.waypoint - 1], plan.route[state.waypoint]) <= 0.6
