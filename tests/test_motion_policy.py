"""Native zero speed and excessive measured speed must stop movement."""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sitl"))
from coordinates import Enu
from motion_policy import movement_target


def test_positive_slow_uses_reduced_lookahead_and_zero_holds() -> None:
    pos, goal = Enu(0, 0, 8), Enu(0, 30, 8)
    assert movement_target(pos, goal, "slow", 0, 0, level_leg=True) is None
    assert movement_target(pos, goal, "slow", 0.3, 0.5, level_leg=True) is None
    target = movement_target(pos, goal, "slow", 0.3, 0.3, level_leg=True)
    assert target is not None and target.north_m == pytest.approx(0.8 * 0.45 / 0.66)
    assert target.up_m == 8
    assert movement_target(pos, goal, "request_replan", 1, 0, level_leg=True) is None
