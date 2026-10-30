"""The flown route must come from the exact independently verified wire bytes."""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sitl"))
from nvplan_route import GeoWaypoint, selected_route


def test_selects_wire_candidate_and_required_stops() -> None:
    text = ("NVPLAN2 id drone 2000000000 1 2\n1 2 3 4 5 6 7\n0\n2\n"
            "37 127 8\n37.1 127 8\n2\n37 127 8\n37.1 127 8\n"
            "3\n37 127 8\n37.05 127 8\n37.1 127 8\n0\n")
    required, route, index = selected_route(text)
    assert index == 1
    assert required == [GeoWaypoint(37, 127, 8), GeoWaypoint(37.1, 127, 8)]
    assert route[1] == GeoWaypoint(37.05, 127, 8)


def test_rejects_truncated_wire_plan() -> None:
    with pytest.raises(ValueError):
        selected_route("NVPLAN2 id drone 2000000000 0 1\n1 2 3 4 5 6 7\n0\n2\n")


def test_required_indices_preserve_order_and_distinct_repeated_home() -> None:
    from nvplan_route import required_indices

    home = GeoWaypoint(37, 127, 8)
    first = GeoWaypoint(37.1, 127, 8)
    second = GeoWaypoint(37.2, 127, 8)
    detour = GeoWaypoint(37.05, 127.01, 8)
    assert required_indices([home, first, second, home],
                            [home, detour, first, second, home]) == (0, 2, 3, 4)
    with pytest.raises(ValueError):
        required_indices([home, first, second, home], [home, second, first, home])


def test_colocated_customers_require_two_distinct_waypoint_occurrences() -> None:
    from nvplan_route import required_indices

    home = GeoWaypoint(37, 127, 8)
    pickup = GeoWaypoint(37.00027, 127, 8)
    assert required_indices([home, pickup, pickup, home], [home, pickup, pickup, home]) == (0, 1, 2, 3)
    with pytest.raises(ValueError):
        required_indices([home, pickup, pickup, home], [home, pickup, home])
