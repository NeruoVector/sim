"""Orders authorize bounded flights; inference alone never starts the drone."""

from school_flight_control import FlightControl, OrderAssignment
from school_flight_control import position_setpoint_ned
from school_flight_control import route_heading
from school_flight_control import landing_aligned
import pytest


def test_world_target_accounts_for_current_estimator_drift():
    # Given a world-frame residual and an estimator with a different origin.
    target = position_setpoint_ned((30,37,5),(29.8,37.1,4.9),(-2,3,-4))
    # Then the NED command closes the measured world error, rather than preserving drift.
    assert target == pytest.approx((-2.1,3.2,-4.1))
    assert position_setpoint_ned((30,37,5),(30,37,5),(-2,3,-4)) == (-2,3,-4)


def test_camera_follows_travel_direction_and_keeps_vertical_leg_heading():
    assert route_heading((0,10,5),(0,0,5),1) == 0
    assert route_heading((10,0,5),(0,0,5),0) == pytest.approx(1.5707963267948966)
    assert route_heading((0,0,5),(0,0,0),1) == 1


def test_landing_waits_for_precise_horizontal_alignment():
    assert not landing_aligned((0.5,0,5),(0,0,.15))
    assert not landing_aligned((.08,.08,5),(0,0,.15))
    assert landing_aligned((.05,.05,5),(0,0,.15))


def test_healthy_model_without_order_stays_idle():
    control = FlightControl()
    assert not control.update(None, fresh=True, blocked=False, now=100)
    assert control.mode == "waiting_for_order"


def test_live_order_starts_and_completed_order_does_not_repeat():
    control = FlightControl()
    assignment = OrderAssignment(100, "order-one")
    assert control.update(assignment, fresh=True, blocked=False, now=100)
    control.complete()
    assert not control.update(assignment, fresh=True, blocked=False, now=101)
    assert control.mode == "order_completed"


def test_backend_or_inference_outage_holds_active_order():
    control = FlightControl()
    assignment = OrderAssignment(100, "order-one")
    assert control.update(assignment, fresh=True, blocked=False, now=100)
    assert not control.update(assignment, fresh=False, blocked=False, now=101)
    assert control.mode == "hold_vision_stale"
    assert not control.update(assignment, fresh=True, blocked=False, now=104)
    assert control.mode == "hold_order_stale"


def test_cancelled_assignment_cannot_fly_and_next_order_can():
    control = FlightControl()
    control.update(OrderAssignment(100, "one"), fresh=True, blocked=False, now=100)
    assert not control.update(
        OrderAssignment(101, None), fresh=True, blocked=False, now=101
    )
    assert control.update(
        OrderAssignment(102, "two"), fresh=True, blocked=False, now=102
    )


def test_obstacle_blocks_order_until_safe():
    control = FlightControl()
    assignment = OrderAssignment(100, "order")
    assert not control.update(assignment, fresh=True, blocked=True, now=100)
    assert control.mode == "hold_obstacle"
    assert control.update(assignment, fresh=True, blocked=False, now=101)


def test_restart_with_completed_order_does_not_fly_again():
    control = FlightControl(completed_order_id="completed")
    assert not control.update(
        OrderAssignment(100, "completed"), fresh=True, blocked=False, now=100
    )
    assert control.mode == "order_completed"


def test_assignment_parses_destination_id(tmp_path):
    import json

    path = tmp_path / "assignment.json"
    path.write_text(
        json.dumps({"timestamp": 100, "order_id": "one", "destination_id": "back"})
    )
    assignment = OrderAssignment.read(path)
    assert assignment.destination_id == "back"


def test_assignment_rejects_non_string_destination(tmp_path):
    import json

    import pytest

    path = tmp_path / "assignment.json"
    path.write_text(
        json.dumps({"timestamp": 100, "order_id": "one", "destination_id": 42})
    )
    with pytest.raises(ValueError, match="Destination"):
        OrderAssignment.read(path)
