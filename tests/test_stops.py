"""Customer stop identity and completion must preserve exact order."""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sitl"))
from stops import bind_stops, stop_is_complete


def test_customer_stops_bind_required_waypoint_indices() -> None:
    rows = [{"order_id":"first", "stop_sequence":1, "status":"scheduled"},
            {"order_id":"second", "stop_sequence":2, "status":"scheduled"}]
    stops = bind_stops(rows, (0, 2, 4, 5))
    assert [s.waypoint_index for s in stops] == [2, 4]
    assert not stop_is_complete(rows, stops, 0)
    rows[0]["status"] = "completed"
    assert stop_is_complete(rows, stops, 0)
    assert not stop_is_complete(rows, stops, 1)


def test_later_completion_and_missing_stops_fail_closed() -> None:
    rows = [{"order_id":"first", "stop_sequence":1, "status":"scheduled"},
            {"order_id":"second", "stop_sequence":2, "status":"scheduled"}]
    stops = bind_stops(rows, (0, 1, 2, 3))
    rows[1]["status"] = "completed"
    with pytest.raises(ValueError):
        stop_is_complete(rows, stops, 0)
    with pytest.raises(ValueError):
        bind_stops(rows[:1], (0, 1, 2, 3))


def test_colocated_orders_keep_distinct_indices_and_sequential_completion() -> None:
    rows = [{"order_id":"first", "stop_sequence":1, "status":"scheduled"},
            {"order_id":"second", "stop_sequence":2, "status":"scheduled"}]
    stops = bind_stops(rows, (0, 1, 2, 3))
    assert [(s.order_id, s.waypoint_index) for s in stops] == [("first", 1), ("second", 2)]
    rows[1]["status"] = "completed"
    with pytest.raises(ValueError):
        stop_is_complete(rows, stops, 0)
    rows[0]["status"], rows[1]["status"] = "completed", "scheduled"
    assert not stop_is_complete(rows, stops, 1)
