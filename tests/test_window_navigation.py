"""Behavioral checks for the isolated window navigation controller."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'sitl'))
from coordinates import Enu
from window_navigation import Hold, facade_clearance, progress_target


def test_facade_clearance_measures_exterior_plane() -> None:
    assert facade_clearance(Enu(30, 36, 8)) == pytest.approx(4)
    assert facade_clearance(Enu(30, 40.2, 8)) < 0


def test_hold_requires_continuous_in_tolerance_samples() -> None:
    hold = Hold()
    assert not hold.update(10, True, 30)
    assert not hold.update(39, True, 30)
    assert not hold.update(40, False, 30)
    assert not hold.update(41, True, 30)
    assert hold.update(71, True, 30)


def test_level_leg_target_does_not_sag_with_altitude_error() -> None:
    target = progress_target(Enu(0, 0, 7.5), Enu(10, 0, 8), True)
    assert target.up_m == 8
    assert 0 < target.east_m <= 0.8
