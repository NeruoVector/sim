"""The simulated aircraft holds for actual high-priority model detections."""

import importlib.util
from pathlib import Path

from neurovector_vision.models import BoundingBox, Detection, DetectResponse

spec = importlib.util.spec_from_file_location(
    "school_relay", Path(__file__).parents[1] / "scripts/school-vision-relay.py"
)
assert spec is not None and spec.loader is not None
relay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(relay)


def result(priority: int, distance: float | None = None, confidence: float = 0.9) -> DetectResponse:
    return DetectResponse(
        drone_id="school-drone-001",
        forwarded=1,
        elapsed_ms=1,
        detections=[
            Detection(
                label="obstacle",
                priority_level=priority,
                confidence=confidence,
                distance_m=distance,
                box=BoundingBox(x1=1, y1=1, x2=20, y2=20),
            )
        ],
    )


def test_critical_detection_holds_even_when_distance_is_unknown():
    observed = result(1)
    blocked = relay.blocked_by(observed)
    assert blocked


def test_nearby_object_holds_and_far_object_does_not():
    near, far = result(2, 2), result(2, 20)
    decisions = relay.blocked_by(near), relay.blocked_by(far)
    assert decisions == (True, False)


def test_leaf_and_low_confidence_detection_do_not_stop_the_demo():
    leaf, uncertain = result(3, 1), result(1, 1, 0.2)
    decisions = relay.blocked_by(leaf), relay.blocked_by(uncertain)
    assert decisions == (False, False)


def test_school_infrastructure_in_the_image_corridor_holds_without_fabricated_distance():
    observed = result(2)
    obstacle = observed.detections[0].model_copy(
        update={
            "label": "cable",
            "box": BoundingBox(x1=300, y1=100, x2=340, y2=200),
        }
    )
    observed = observed.model_copy(update={"detections": [obstacle]})
    assert relay.blocked_by(observed, 640)
    assert not relay.blocked_by(observed, 1600)


def test_validated_vertical_takeoff_is_not_blocked_by_a_distant_facade_in_front():
    observed = result(2)
    facade = observed.detections[0].model_copy(update={"label": "building_region", "box": BoundingBox(x1=0,y1=0,x2=640,y2=360)})
    observed = observed.model_copy(update={"detections": [facade]})
    assert relay.blocked_by(observed,640)
    assert not relay.blocked_by(observed,640,allow_unknown_building_in_column=True)
    assert relay.blocked_by(result(1),640,True,allow_unknown_building_in_column=True)
    nearby = facade.model_copy(update={"distance_m": 1})
    assert relay.blocked_by(observed.model_copy(update={"detections":[nearby]}),640,True,
                            allow_unknown_building_in_column=True)


def test_disarmed_handoff_uses_checked_column_without_ignoring_forward_obstacles():
    from school_relay_runtime import MissionContext, SimState

    state = SimState(timestamp=0, camera_timestamp=0, position_enu=(30,37,.43),
                     mode='hold_obstacle', armed=False, drone_id='school-drone-001',
                     mission=MissionContext(phase='delivery_landing'), waypoint=5)
    assert relay.in_landing_column(state)
    assert relay.in_landing_column(state.model_copy(update={'mission':MissionContext(phase='awaiting_receipt')}))
    assert not relay.in_landing_column(state.model_copy(update={'armed':True}))
    assert not relay.in_landing_column(state.model_copy(update={'mission':MissionContext(phase='returning')}))
    assert not relay.in_landing_column(state.model_copy(update={'mission':None}))
    observed = result(2)
    cable = observed.detections[0].model_copy(update={'label':'cable','box':BoundingBox(x1=0,y1=0,x2=640,y2=360)})
    assert relay.blocked_by(observed.model_copy(update={'detections':[cable]}),640,True,
                            allow_unknown_building_in_column=True)


def test_signed_building_corridor_can_approach_facade_without_ignoring_near_hazards():
    observed=result(2)
    facade=observed.detections[0].model_copy(update={'label':'building_region','distance_m':3})
    observed=observed.model_copy(update={'detections':[facade]})
    assert relay.blocked_by(observed)
    assert not relay.blocked_by(observed,640,True)
    near=facade.model_copy(update={'distance_m':1.4})
    assert relay.blocked_by(observed.model_copy(update={'detections':[near]}),640,True)
    person=facade.model_copy(update={'label':'person','priority_level':1})
    assert relay.blocked_by(observed.model_copy(update={'detections':[person]}),640,True)


def test_checked_horizontal_route_holds_unknown_or_stale_building_range(tmp_path):
    # Given an on-screen facade and a certified horizontal flight corridor,
    from school_depth_ranges import with_depth_ranges
    import numpy as np

    facade = result(2).detections[0].model_copy(update={
        'label': 'building_region',
        'box': BoundingBox(x1=280, y1=140, x2=360, y2=220),
    })
    unknown = result(2).model_copy(update={'detections': [facade]})
    path = tmp_path / 'depth.npz'
    np.savez(path, depth=np.full((480,640),3,dtype=np.float32),
             timestamp=95, near_m=.2, far_m=19.1)

    # When depth is missing or stale, it cannot authorize facade clearance.
    stale = with_depth_ranges(unknown, path, (360,640), 100, 100)

    # Then the signed horizontal route holds; only the separate column flag exempts unknown.
    assert relay.blocked_by(unknown, 640, checked_building_clearance=True)
    assert relay.blocked_by(stale, 640, checked_building_clearance=True)
