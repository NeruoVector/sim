"""Telemetry remains independent and cannot re-enable inference across an outage."""

import json
import threading
import time
from pathlib import Path

import httpx2
import pytest
from school_relay_runtime import InferencePublisher, SimState, TelemetryReporter

from neurovector_vision.models import DetectResponse


def source(root: Path, **changes) -> SimState:
    state = SimState(
        timestamp=time.time(),
        camera_timestamp=time.time(),
        position_enu=(1, 2, 3),
        mode="flying",
        armed=True,
        drone_id="school-drone-001",
    ).model_copy(update=changes)
    (root / "state.json").write_text(state.model_dump_json())
    return state


def empty_result() -> DetectResponse:
    return DetectResponse(drone_id="school-drone-001", detections=[], forwarded=0, elapsed_ms=1)


def test_failure_recovery_cannot_promote_inference_started_before_outage(tmp_path):
    publisher = InferencePublisher(tmp_path)
    state = source(tmp_path)
    publisher.telemetry_succeeded(state)
    generation = publisher.begin(state)
    publisher.telemetry_failed("backend disconnected")
    publisher.telemetry_succeeded(state)
    assert not json.loads((tmp_path / "inference.json").read_text())["ok"]
    with pytest.raises(RuntimeError, match="while inference"):
        publisher.publish(empty_result(), state, generation, time.time(), False)
    assert not json.loads((tmp_path / "inference.json").read_text())["ok"]
    publisher.publish(empty_result(), state, publisher.begin(state), time.time(), False)
    assert json.loads((tmp_path / "inference.json").read_text())["ok"]


def test_background_failure_invalidates_while_inference_waits(tmp_path):
    publisher = InferencePublisher(tmp_path)
    state = source(tmp_path)
    publisher.telemetry_succeeded(state)
    generation = publisher.begin(state)
    publisher.publish(empty_result(), state, generation, time.time(), False)
    observed = threading.Event()
    calls = []

    def receive(request):
        calls.append(json.loads(request.content))
        observed.set()
        return httpx2.Response(503)

    client = httpx2.Client(transport=httpx2.MockTransport(receive))
    with TelemetryReporter(publisher, "http://backend.test", lambda: client):
        assert observed.wait(2)
        deadline = time.monotonic() + 2
        while json.loads((tmp_path / "inference.json").read_text())["ok"]:
            assert time.monotonic() < deadline
            time.sleep(0.005)
        with pytest.raises(RuntimeError):
            publisher.publish(empty_result(), state, generation, time.time(), False)
    assert client.is_closed
    assert calls[0]["altitude_m"] == 3
    assert calls[0]["status"] == "flying"


def test_reporter_keeps_sending_without_inference_and_closes_pool(tmp_path):
    publisher = InferencePublisher(tmp_path)
    source(tmp_path)
    received = threading.Event()
    calls = []

    def receive(request):
        calls.append(time.monotonic())
        if len(calls) >= 2:
            received.set()
        return httpx2.Response(200)

    client = httpx2.Client(transport=httpx2.MockTransport(receive))
    with TelemetryReporter(publisher, "http://backend.test", lambda: client):
        assert received.wait(3)
        assert 0.8 < calls[1] - calls[0] < 1.8
        assert not (tmp_path / "inference.json").exists()
    assert client.is_closed


@pytest.mark.parametrize(
    "changes", [dict(timestamp=0), dict(camera_timestamp=0), dict(position_enu=None)]
)
def test_unusable_source_never_posts_telemetry(tmp_path, changes):
    publisher = InferencePublisher(tmp_path)
    source(tmp_path, **changes)

    def unexpected(request):
        pytest.fail("Invalid simulator state reached backend")

    with httpx2.Client(transport=httpx2.MockTransport(unexpected)) as client:
        TelemetryReporter(publisher, "http://backend.test").report_once(client)
    assert not json.loads((tmp_path / "inference.json").read_text())["ok"]


def test_late_acknowledgement_is_rejected(tmp_path, monkeypatch):
    publisher = InferencePublisher(tmp_path)
    state = source(tmp_path)

    def delayed(request):
        monkeypatch.setattr("school_relay_runtime.time.time", lambda: state.timestamp + 6)
        return httpx2.Response(200)

    with httpx2.Client(transport=httpx2.MockTransport(delayed)) as client:
        TelemetryReporter(publisher, "http://backend.test").report_once(client)
    with pytest.raises((RuntimeError, TimeoutError)):
        publisher.begin(state)
    assert not json.loads((tmp_path / "inference.json").read_text())["ok"]


def test_changed_identity_or_stale_camera_cannot_publish(tmp_path):
    publisher = InferencePublisher(tmp_path)
    state = source(tmp_path)
    publisher.telemetry_succeeded(state)
    generation = publisher.begin(state)
    source(tmp_path, drone_id="another-drone")
    with pytest.raises(RuntimeError, match="identity"):
        publisher.publish(empty_result(), state, generation, time.time(), False)
    source(tmp_path, camera_timestamp=0)
    with pytest.raises(TimeoutError, match="camera"):
        publisher.publish(empty_result(), state, generation, time.time(), False)


def test_failure_after_publication_overwrites_success(tmp_path):
    publisher = InferencePublisher(tmp_path)
    state = source(tmp_path)
    publisher.telemetry_succeeded(state)
    publisher.publish(empty_result(), state, publisher.begin(state), time.time(), False)
    publisher.telemetry_failed("connection lost after inference")
    result = json.loads((tmp_path / "inference.json").read_text())
    assert result["ok"] is False and result["blocked"] is True
