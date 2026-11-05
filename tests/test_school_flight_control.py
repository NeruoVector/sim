"""Orders authorize bounded flights; inference alone never starts the drone."""

from school_flight_control import FlightControl, OrderAssignment


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
