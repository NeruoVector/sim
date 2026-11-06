"""Measured static-mesh clearance must not become an unrestricted vision bypass."""
from pathlib import Path

import pytest

from school_mission import MissionPlan
from school_relay_runtime import MissionContext, SimState
from school_vision_route import checked_building_corridor, segment_distance

FIXTURE = Path(__file__).parents[1] / 'fixtures/school.json'


def current_state():
    plan = MissionPlan.read(FIXTURE, 'lobby')
    return SimState(timestamp=0, camera_timestamp=0, position_enu=(30.3,17.3,5.03),
                    mode='hold_obstacle', armed=True, drone_id='school-drone-001',waypoint=4,
                    mission=MissionContext(phase='outbound',destination_id='lobby',route_enu=list(plan.route)))


def test_actual_far_facade_hold_is_inside_the_signed_clear_leg():
    assert checked_building_corridor(current_state(), FIXTURE)


@pytest.mark.parametrize('position',[(31.0,17.3,5),(30.3,17.3,4.3),(30,39,5)])
def test_outside_planned_error_envelope_fails_closed(position):
    assert not checked_building_corridor(current_state().model_copy(update={'position_enu':position}),FIXTURE)


def test_wrong_route_destination_phase_and_missing_mesh_fail_closed(tmp_path):
    state=current_state()
    assert state.mission is not None
    for update in [{'route_enu':[(0,0,18),(30,37,18)]},{'destination_id':'gym'},{'phase':'awaiting_receipt'}]:
        assert not checked_building_corridor(state.model_copy(update={'mission':state.mission.model_copy(update=update)}),FIXTURE)
    assert not checked_building_corridor(state,FIXTURE.with_name('missing.json'))
    assert not checked_building_corridor(state.model_copy(update={'waypoint':0}),FIXTURE)
    assert not checked_building_corridor(state.model_copy(update={'position_enu':None}),FIXTURE)


def test_distance_uses_bounded_segment_and_all_three_dimensions():
    assert segment_distance((0,0,0),(1,0,0),(2,0,0))==1
    assert segment_distance((1.5,0,.7),(1,0,0),(2,0,0))==.7
