"""Translate a native proceed or slow decision into a bounded lookahead."""
from __future__ import annotations

import math

from coordinates import Enu


def movement_target(position: Enu, goal: Enu, action: str, desired_speed: float,
                    measured_speed: float, *, level_leg: bool) -> Enu | None:
    if action not in ("proceed", "slow"):
        return None
    if action == "slow":
        if desired_speed <= 0.05 or measured_speed > desired_speed + 0.15:
            return None
        # At this PX4 tuning, an 0.8 m position lead produces about 0.66 m/s.
        # Feedback brakes if the measured speed exceeds the requested ceiling.
        lookahead = 0.8 * min(1.0, (desired_speed + 0.15) / 0.66)
    else:
        lookahead = 0.8
    dx, dy, dz = goal.east_m-position.east_m, goal.north_m-position.north_m, goal.up_m-position.up_m
    length = math.sqrt(dx*dx + dy*dy + dz*dz)
    scale = min(1.0, lookahead / length) if length else 0.0
    return Enu(position.east_m + dx*scale, position.north_m + dy*scale,
               goal.up_m if level_leg else position.up_m + dz*scale)
