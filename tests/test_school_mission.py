"""Campus deliveries use measured route progress and authorize home landing only."""

import json
import math
from pathlib import Path

import pytest
from school_mission import InvalidMission, Mission, MissionPlan, enu_to_ned

FIXTURE = Path(__file__).parents[1] / "fixtures/school.json"


@pytest.mark.parametrize(
    "destination,expected",
    [
        ("lobby", (30, 37, 18)),
        ("gym", (60, 20, 18)),
        ("back", (30, 63, 18)),
        ("playground", (25, 12, 18)),
    ],
)
def test_destination_selects_real_campus_route(destination, expected):
    # Given the shared synthetic fixture, when a known delivery is selected.
    plan = MissionPlan.read(FIXTURE, destination)
    # Then cruise clears the mesh and destination replaces the old tiny square.
    assert plan.route == ((0, 0, 18), expected, (0, 0, 18), (0, 0, 0.15))
    assert plan.speed_mps == 1


@pytest.mark.parametrize("destination", [None, "unknown", "", "1층 현관"])
def test_unmapped_destination_cannot_authorize_route(destination):
    with pytest.raises(InvalidMission):
        MissionPlan.read(FIXTURE, destination)


@pytest.mark.parametrize(
    "field,value",
    [
        ("cruise_altitude_m", 8),
        ("cruise_altitude_m", 120),
        ("speed_mps", 0),
        ("arrival_hold_seconds", math.nan),
    ],
)
def test_invalid_route_configuration_is_rejected(tmp_path, field, value):
    data = json.loads(FIXTURE.read_text())
    data["delivery_missions"][field] = value
    fixture = tmp_path / "fixture.json"
    fixture.write_text(json.dumps(data))
    with pytest.raises(InvalidMission):
        MissionPlan.read(fixture, "lobby")


def test_route_outside_campus_is_rejected(tmp_path):
    data = json.loads(FIXTURE.read_text())
    data["delivery_missions"]["destinations"]["lobby"] = [1000, 0, 18]
    fixture = tmp_path / "fixture.json"
    fixture.write_text(json.dumps(data))
    with pytest.raises(InvalidMission):
        MissionPlan.read(fixture, "lobby")


def test_enu_ned_conversion_uses_north_east_down():
    assert enu_to_ned((30, 37, 18)) == (37, 30, -18)


def test_remaining_distance_uses_actual_pose_not_elapsed_time():
    mission = Mission("one", MissionPlan.read(FIXTURE, "lobby"))
    far = mission.snapshot((0, 0, 0), authorized=True)
    near = mission.snapshot((0, 0, 10), authorized=True)
    assert far["remaining_distance_m"] - near["remaining_distance_m"] == pytest.approx(
        10
    )
    assert far["delivery_distance_m"] - near["delivery_distance_m"] == pytest.approx(10)
    assert far["eta_seconds"] - near["eta_seconds"] == pytest.approx(10)
    assert far["delivery_distance_m"] == pytest.approx(18 + math.hypot(30, 37))
    assert far["remaining_distance_m"] == pytest.approx(
        18 + 2 * math.hypot(30, 37) + 17.85
    )


def test_hold_preserves_waypoint_and_removes_eta():
    mission = Mission("one", MissionPlan.read(FIXTURE, "lobby"))
    mission.step((0, 0, 18), armed=True, authorized=False, now=1)
    snapshot = mission.snapshot((0, 0, 18), authorized=False)
    assert mission.waypoint_index == 0
    assert snapshot["eta_seconds"] is None
    assert snapshot["delivery_eta_seconds"] is None
    assert snapshot["remaining_distance_m"] > 0


def test_disarmed_vehicle_cannot_advance_waypoint():
    mission = Mission("one", MissionPlan.read(FIXTURE, "lobby"))
    mission.step((0, 0, 18), armed=False, authorized=True, now=1)
    assert mission.waypoint_index == 0
    assert not mission.complete_landing((0, 0, 0.15), armed=False)


def arrived_mission():
    mission = Mission("own-order", MissionPlan.read(FIXTURE, "lobby"))
    mission.step((0, 0, 18), armed=True, authorized=True, now=0)
    mission.step((30, 37, 18), armed=True, authorized=True, now=1)
    for second in range(2, 7):
        mission.step((30, 37, 18), armed=True, authorized=True, now=second)
    return mission


def test_arrival_requires_measured_hover_then_returns_home():
    mission = arrived_mission()
    assert mission.arrived
    assert mission.phase == "returning"
    assert mission.target == (0, 0, 18)
    snapshot = mission.snapshot((30, 37, 18), authorized=True)
    assert snapshot["delivery_distance_m"] == 0
    assert snapshot["delivery_eta_seconds"] == 0
    assert snapshot["remaining_distance_m"] > 0


def test_hold_time_does_not_count_as_arrival_hover():
    mission = Mission("one", MissionPlan.read(FIXTURE, "lobby"))
    mission.step((0, 0, 18), armed=True, authorized=True, now=0)
    mission.step((30, 37, 18), armed=True, authorized=True, now=1)
    mission.step((30, 37, 18), armed=True, authorized=False, now=50)
    mission.step((30, 37, 18), armed=True, authorized=True, now=51)
    assert not mission.arrived
    assert mission.dwell_elapsed == 1


def test_only_home_landing_after_arrival_completes_own_mission():
    mission = arrived_mission()
    mission.step((0, 0, 18), armed=True, authorized=True, now=7)
    assert mission.phase == "landing"
    assert not mission.complete_landing((30, 37, 0.15), armed=False)
    assert not mission.complete_landing((0, 0, 0.15), armed=True)
    assert not mission.complete_landing((0, 0, 18), armed=False)
    assert mission.complete_landing((0, 0, 0.15), armed=False)
    snapshot = mission.snapshot((0, 0, 0.15), authorized=False)
    assert snapshot["order_id"] == "own-order"
    assert snapshot["remaining_distance_m"] == 0
    assert snapshot["eta_seconds"] == 0
