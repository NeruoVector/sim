"""Real order responses drive assignments, cursor recovery, cancellation and completion."""

import json
import time
from datetime import UTC, datetime
from uuid import uuid4

import httpx2
from school_order_relay import OrderRelay


def controller(root, completed=None):
    (root / "state.json").write_text(
        json.dumps({"timestamp": time.time(), "completed_order_id": completed})
    )


def test_empty_feed_does_not_assign_and_cursor_survives_restart(tmp_path):
    relay = OrderRelay(tmp_path, "http://backend")
    controller(tmp_path)
    with httpx2.Client(
        transport=httpx2.MockTransport(lambda request: httpx2.Response(200, json=[]))
    ) as client:
        relay.poll_once(client)
    assignment = json.loads((tmp_path / "order-assignment.json").read_text())
    assert assignment["order_id"] is None
    assert OrderRelay(tmp_path, "http://backend").cursor == relay.cursor


def test_new_order_assigns_once_then_completion_advances_cursor(tmp_path):
    relay = OrderRelay(tmp_path, "http://backend")
    order = {
        "id": str(uuid4()),
        "status": "pending_payment",
        "created_at": datetime.now(UTC).isoformat(),
    }
    controller(tmp_path)
    seen = []

    def receive(request):
        seen.append(request.url.params["after_id"])
        return httpx2.Response(
            200, json=[order] if request.url.params["after_id"] != order["id"] else []
        )

    with httpx2.Client(transport=httpx2.MockTransport(receive)) as client:
        relay.poll_once(client)
        assert (
            json.loads((tmp_path / "order-assignment.json").read_text())["order_id"]
            == order["id"]
        )
        controller(tmp_path, order["id"])
        relay.poll_once(client)
        relay.poll_once(client)
    assert str(OrderRelay(tmp_path, "http://backend").cursor.after_id) == order["id"]
    assert (
        json.loads((tmp_path / "order-assignment.json").read_text())["order_id"] is None
    )
    assert seen[-1] == order["id"]
    assert (
        json.loads((tmp_path / "orders" / f"{order['id']}.json").read_text())[
            "order_id"
        ]
        == order["id"]
    )


def test_cancelled_order_is_skipped_before_following_order(tmp_path):
    relay = OrderRelay(tmp_path, "http://backend")
    controller(tmp_path)
    orders = [
        {
            "id": str(uuid4()),
            "status": status,
            "created_at": datetime.now(UTC).isoformat(),
        }
        for status in ("cancelled", "paid")
    ]
    with httpx2.Client(
        transport=httpx2.MockTransport(
            lambda request: httpx2.Response(200, json=orders)
        )
    ) as client:
        relay.poll_once(client)
    assert str(relay.cursor.after_id) == orders[0]["id"]
    assert (
        json.loads((tmp_path / "order-assignment.json").read_text())["order_id"]
        == orders[1]["id"]
    )


def test_destination_is_forwarded_without_customer_details(tmp_path):
    relay = OrderRelay(tmp_path, "http://backend")
    controller(tmp_path)
    order = {
        "id": str(uuid4()),
        "status": "paid",
        "created_at": datetime.now(UTC).isoformat(),
        "destination_id": "gym",
        "delivery_address": "must not forward",
        "received": True,
    }
    with httpx2.Client(
        transport=httpx2.MockTransport(
            lambda request: httpx2.Response(200, json=[order])
        )
    ) as client:
        relay.poll_once(client)
    assignment = json.loads((tmp_path / "order-assignment.json").read_text())
    assert assignment["destination_id"] == "gym"
    assert assignment["received"] is True
    assert "delivery_address" not in assignment


def test_unmapped_destination_stays_explicitly_unavailable(tmp_path):
    relay = OrderRelay(tmp_path, "http://backend")
    controller(tmp_path)
    order = {
        "id": str(uuid4()),
        "status": "paid",
        "created_at": datetime.now(UTC).isoformat(),
        "destination_id": None,
    }
    with httpx2.Client(
        transport=httpx2.MockTransport(
            lambda request: httpx2.Response(200, json=[order])
        )
    ) as client:
        relay.poll_once(client)
    assignment = json.loads((tmp_path / "order-assignment.json").read_text())
    assert assignment["order_id"] == order["id"]
    assert assignment["destination_id"] is None
