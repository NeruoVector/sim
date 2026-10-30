"""Replacement activation must use fresh telemetry from a held aircraft."""
from __future__ import annotations

import json
import sys
import time
import urllib.error
from datetime import UTC, datetime, timedelta
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sitl"))
import auto_replan
from auto_replan import _fresh_held


def test_requires_recent_accepted_slow_armed_sample() -> None:
    row = {"utc": datetime.now(UTC).isoformat(), "local_age": 0.1, "gps_age": 0.1,
           "armed": True, "vn": 0.1, "ve": 0.1, "vd": 0.1,
           "latitude":37.0, "longitude":127.0, "relative_altitude":8.0}
    snapshot = (123, row, time.monotonic())
    assert _fresh_held(snapshot)[0] == 123
    with pytest.raises(RuntimeError):
        _fresh_held((123, row | {"vn": 0.4}, time.monotonic()))
    with pytest.raises(RuntimeError):
        _fresh_held((123, row | {"vn": float("nan")}, time.monotonic()))
    with pytest.raises(RuntimeError):
        _fresh_held((123, row | {"utc": (datetime.now(UTC)-timedelta(seconds=4)).isoformat()},
                     time.monotonic()))
    with pytest.raises(RuntimeError):
        _fresh_held(None)


def test_automatic_replan_preserves_stops_and_retries_cas(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    stops = [{"lat":37.0,"lon":127.0,"altitude_m":8},
             {"lat":37.00027,"lon":127.0,"altitude_m":8},
             {"lat":37.0,"lon":127.0,"altitude_m":8}]
    original = tmp_path / "old.json"
    original.write_text(json.dumps({"payload":{"altitude_reference":"home_relative", "required_stops":stops,
                                                "constraints":{"max_altitude_m":8}}}))
    requests = []
    attempts = 0
    def request(_admin: object, path: str, body: dict) -> dict:
        nonlocal attempts
        requests.append((path, body))
        if path == "routes/no-fly-zones":
            return {"id":"zone"}
        if path == "routes/plan":
            return {"id":"plan", "route_id":"route", "route_fingerprint":"hash", "plan_text":"wire"}
        if path == "routes/plans/plan/cross-check":
            return {"id":"plan", "route_id":"route", "status":"verified"}
        attempts += 1
        if attempts == 1:
            raise urllib.error.HTTPError("", 409, "conflict", {}, None)
        return {"plan_id":"plan", "route_id":"route"}
    monkeypatch.setattr(auto_replan, "_request", request)
    monkeypatch.setattr(auto_replan.subprocess, "run", lambda *a, **k: SimpleNamespace(stdout='{"valid":true}'))
    monkeypatch.setattr(auto_replan.time, "sleep", lambda _: None)
    row = {"utc":datetime.now(UTC).isoformat(), "local_age":0.0, "gps_age":0.0, "armed":True,
           "vn":0, "ve":0, "vd":0, "latitude":37.00015, "longitude":127.0, "relative_altitude":8.002}
    contract = {"QA_DELIVERY":"delivery", "QA_ROUTE":"old"}
    result, saved, zone = auto_replan.activate_global_replan(None, contract, original, Path("checker"),
        lambda: (123, row, time.monotonic()), tmp_path)
    assert zone == "zone" and result["QA_ROUTE"] == "route" and saved.is_file() and attempts == 2
    plan_request = next(body for path, body in requests if path == "routes/plan")
    assert plan_request["altitude_reference"] == "home_relative"
    assert plan_request["max_altitude_m"] == 8 and plan_request["waypoints"][0]["altitude_m"] == 8
    assert plan_request["waypoints"][1:] == stops[1:]
    assert requests[-1][1]["expected_telemetry_id"] == 123
