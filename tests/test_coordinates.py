"""Local conversion uses an explicit origin and preserves PX4 axis signs."""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sitl"))
from coordinates import (
    Enu,
    Geographic,
    Ned,
    Origin,
    enu_to_ned,
    horizontal_segment_error,
    ned_to_enu,
    require_route_corridor,
    require_synthetic_start,
)


def test_enu_ned_axes_and_round_trip() -> None:
    point = Enu(3.0, 5.0, 8.0)
    assert enu_to_ned(point) == Ned(5.0, 3.0, -8.0)
    assert ned_to_enu(enu_to_ned(point)) == point


def test_geographic_conversion_uses_origin_altitude_and_round_trip() -> None:
    origin = Origin(37.0, 127.0, 225.0)
    target = Geographic(37.000045, 127.0, 233.0)
    local = origin.to_enu(target)
    assert local.north_m == pytest.approx(5.00377, abs=0.001)
    assert local.east_m == pytest.approx(0.0, abs=0.001)
    assert local.up_m == 8.0
    restored = origin.to_geographic(local)
    assert restored.latitude_deg == pytest.approx(target.latitude_deg, abs=1e-10)
    assert restored.longitude_deg == pytest.approx(target.longitude_deg, abs=1e-10)
    assert restored.altitude_m == target.altitude_m


def test_invalid_coordinates_fail_closed() -> None:
    with pytest.raises(ValueError):
        Origin(float("nan"), 127.0, 0.0)
    with pytest.raises(ValueError):
        Origin(37.0, 127.0, 0.0).to_enu(Geographic(91.0, 127.0, 8.0))


def test_start_rejects_distant_or_airborne_state() -> None:
    require_synthetic_start(Enu(0.2, -0.1, 0.05))
    with pytest.raises(ValueError):
        require_synthetic_start(Enu(2.1, 0, 0))
    with pytest.raises(ValueError):
        require_synthetic_start(Enu(0, 0, 1))


def test_local_target_is_relative_to_current_frame_after_altitude_reset() -> None:
    from coordinates import local_target

    target = Enu(0, 30, 8)
    before_reset = local_target(Ned(10, 0, -7.5), Enu(0, 10, 7.1), target)
    after_reset = local_target(Ned(10, 0, -7.9), Enu(0, 10, 7.1), target)
    assert before_reset.down_m == pytest.approx(-8.4)
    assert after_reset.down_m == pytest.approx(-8.8)
    assert before_reset.north_m == after_reset.north_m == 30


def test_segment_error_checks_cross_track_and_endpoints() -> None:
    start, end = Enu(0, 0, 8), Enu(0, 30, 8)
    assert horizontal_segment_error(Enu(1.5, 15, 8), start, end) == pytest.approx(1.5)
    assert horizontal_segment_error(Enu(0, 33, 8), start, end) == pytest.approx(3)
    assert horizontal_segment_error(Enu(3, 4, 8), start, start) == pytest.approx(5)


def test_corridor_rejects_deviation_but_allows_post_route_landing() -> None:
    route = [Enu(0, 0, 8), Enu(0, 30, 8)]
    require_route_corridor(route, 1, Enu(0, 15, 8), landing=False)
    with pytest.raises(ValueError):
        require_route_corridor(route, 1, Enu(0, 15, 6), landing=False)
    with pytest.raises(ValueError):
        require_route_corridor(route, 1, Enu(3, 15, 8), landing=False)
    require_route_corridor(route, len(route), Enu(0, 30, 0), landing=True)
