"""Measured delivery and explicit receipt lifecycle over obstacle-aware routes."""

import json
import math
from pathlib import Path

import pytest
from school_mission import InvalidMission, Mission, MissionPlan, enu_to_ned

FIXTURE = Path(__file__).parents[1] / "fixtures/school.json"
DESTINATIONS = ["lobby", "gym", "back", "playground"]


def outbound_mission(destination="lobby"):
    mission = Mission("own-order", MissionPlan.read(FIXTURE, destination))
    while mission.phase == "outbound":
        mission.step(mission.target, armed=True, authorized=True, now=0)
    return mission


def arrived_mission(destination="lobby"):
    mission = outbound_mission(destination)
    mission.step(mission.target, armed=False, authorized=True, now=1)
    return mission


@pytest.mark.parametrize("destination", DESTINATIONS)
def test_destination_selects_lower_verified_route_with_safe_return(destination):
    data = json.loads(FIXTURE.read_text())
    plan = MissionPlan.read(FIXTURE, destination)
    approach = tuple(data["delivery_missions"]["destinations"][destination])
    assert 0 < approach[2] <= 8 < data["buildings"][0]["no_fly_ceiling_m"]
    assert plan.route[0] == (0, 0, approach[2])
    assert plan.route[plan.delivery_index - 1] == approach
    assert plan.route[plan.delivery_index] == tuple(
        data["delivery_missions"]["landing_points"][destination]
    )
    assert plan.route[plan.return_takeoff_index] == approach
    assert plan.route[plan.return_takeoff_index : -1] == tuple(
        reversed(plan.route[: plan.delivery_index])
    )
    assert plan.route[-1] == (0, 0, 0.15)
    assert 6 < len(plan.route) <= 32


@pytest.mark.parametrize("destination", [None, "unknown", "", "1층 현관"])
def test_unmapped_destination_cannot_authorize_route(destination):
    with pytest.raises(InvalidMission):
        MissionPlan.read(FIXTURE, destination)


@pytest.mark.parametrize(
    "field,value",
    [
        ("cruise_altitude_m", 0),
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


@pytest.mark.parametrize("mutation", ["outside", "building", "route", "pad", "mesh"])
def test_stale_or_modified_geometry_cannot_authorize_offline_route(tmp_path, mutation):
    data = json.loads(FIXTURE.read_text())
    if mutation == "outside":
        data["delivery_missions"]["destinations"]["lobby"] = [1000, 0, 5]
    elif mutation == "building":
        data["buildings"][0]["center_enu"] = [20, 20, 6]
    elif mutation == "route":
        data["delivery_missions"]["routes_enu"]["lobby"][1] = [5, 8, 5]
    elif mutation == "pad":
        data["delivery_missions"]["landing_points"]["gym"][0] = 50
    else:
        data["campus"]["mesh_pose_enu"][2] = 1
    fixture = tmp_path / "fixture.json"
    fixture.write_text(json.dumps(data))
    with pytest.raises(InvalidMission):
        MissionPlan.read(fixture, "lobby")


def test_enu_ned_conversion_uses_north_east_down():
    assert enu_to_ned((30, 37, 5)) == (37, 30, -5)


@pytest.mark.parametrize("destination", DESTINATIONS)
def test_every_detour_waypoint_precedes_delivery_landing(destination):
    mission = Mission("route-order", MissionPlan.read(FIXTURE, destination))
    for index in range(mission.plan.delivery_index):
        assert mission.phase == "outbound"
        assert mission.waypoint_index == index
        mission.step(mission.target, armed=True, authorized=True, now=1)
    assert mission.phase == "delivery_landing"
    assert not mission.arrived
    assert mission.waypoint_index == mission.plan.delivery_index


def test_remaining_distance_uses_actual_pose_not_elapsed_time():
    mission = Mission("one", MissionPlan.read(FIXTURE, "lobby"))
    far = mission.snapshot((0, 0, 0), authorized=True)
    near = mission.snapshot((0, 0, 2), authorized=True)
    assert far["remaining_distance_m"] - near["remaining_distance_m"] == pytest.approx(
        2
    )
    assert far["delivery_distance_m"] - near["delivery_distance_m"] == pytest.approx(2)
    assert far["remaining_distance_m"] > far["delivery_distance_m"] > math.hypot(30, 37)
    assert far["eta_seconds"] is None
    assert far["delivery_eta_seconds"] == pytest.approx(
        far["delivery_distance_m"] / mission.plan.speed_mps
    )
    assert far["destination_enu"] == (30, 37, 0.59)


def test_hold_and_disarm_preserve_progress_and_remove_eta():
    mission = Mission("one", MissionPlan.read(FIXTURE, "lobby"))
    mission.step(mission.target, armed=True, authorized=False, now=1)
    mission.step(mission.target, armed=False, authorized=True, now=2)
    snapshot = mission.snapshot(mission.target, authorized=False)
    assert mission.waypoint_index == 0
    assert snapshot["eta_seconds"] is None
    assert snapshot["delivery_eta_seconds"] is None
    assert not mission.complete_landing(mission.plan.route[-1], armed=False)


def test_waypoint_acceptance_remains_fixed_at_point_six_metres():
    mission = Mission("one", MissionPlan.read(FIXTURE, "lobby"))
    x, y, z = mission.target
    mission.step((x + 0.61, y, z), armed=True, authorized=True, now=0)
    assert mission.waypoint_index == 0
    mission.step((x + 0.59, y, z), armed=True, authorized=True, now=1)
    assert mission.waypoint_index == 1


def test_arrival_requires_landing_and_waits_indefinitely_for_explicit_receipt():
    mission = arrived_mission()
    assert mission.arrived
    mission.step(mission.target, armed=False, authorized=True, now=10000)
    assert mission.phase == "awaiting_receipt"
    snapshot = mission.snapshot(mission.target, authorized=True)
    assert snapshot["delivery_distance_m"] == snapshot["delivery_eta_seconds"] == 0
    assert snapshot["remaining_distance_m"] > 0
    assert snapshot["eta_seconds"] is None


def test_hover_or_wrong_ground_position_never_counts_as_delivery():
    mission = outbound_mission()
    mission.step(mission.target, armed=True, authorized=True, now=500)
    mission.step((0, 0, 0.15), armed=False, authorized=True, now=501)
    assert not mission.arrived
    assert mission.phase == "delivery_landing"


@pytest.mark.parametrize("destination", DESTINATIONS)
def test_explicit_receipt_returns_through_every_detour_then_completes_at_home(
    destination,
):
    mission = arrived_mission(destination)
    home = mission.plan.route[-1]
    assert not mission.complete_landing(home, armed=False)
    mission.step(mission.target, armed=False, authorized=True, received=True, now=3)
    assert mission.phase == "return_takeoff"
    mission.step(mission.target, armed=True, authorized=True, now=4)
    assert mission.phase == "returning"
    while mission.phase == "returning":
        mission.step(mission.target, armed=True, authorized=True, now=5)
    assert mission.phase == "landing"
    assert mission.target == home
    assert not mission.complete_landing(
        mission.plan.route[mission.plan.delivery_index], armed=False
    )
    assert not mission.complete_landing(home, armed=True)
    assert not mission.complete_landing((home[0], home[1], 5), armed=False)
    assert mission.complete_landing(home, armed=False)
    snapshot = mission.snapshot(home, authorized=False)
    assert snapshot["order_id"] == "own-order"
    assert snapshot["remaining_distance_m"] == snapshot["eta_seconds"] == 0
    assert snapshot["received"]


def test_receipt_without_current_authorization_or_while_armed_cannot_take_off():
    mission = arrived_mission()
    mission.step(mission.target, armed=False, authorized=False, received=True, now=3)
    mission.step(mission.target, armed=True, authorized=True, received=True, now=4)
    assert mission.phase == "awaiting_receipt"
    assert not mission.received
