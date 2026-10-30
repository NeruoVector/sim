"""Private synthetic-backend global replan, executed only during a measured hold."""
from __future__ import annotations

import json
import math
import subprocess
import time
import urllib.error
import urllib.request
from collections.abc import Callable
from datetime import UTC, datetime
from pathlib import Path

from coordinates import Enu, Origin
from mission_backend import API, DRONE, AdminStops

Snapshot = tuple[int, dict, float]


def _request(admin: AdminStops, path: str, body: dict) -> dict:
    req = urllib.request.Request(f"{API}/api/v1/{path}", data=json.dumps(body).encode(),
        headers={"Content-Type": "application/json"}, method="POST")
    with admin.opener.open(req, timeout=5) as response:
        result = json.load(response)
    if not isinstance(result, dict):
        raise TypeError("invalid backend response")
    return result


def _fresh_held(snapshot: Snapshot | None) -> tuple[int, dict]:
    if snapshot is None:
        raise RuntimeError("no accepted telemetry available for held replan")
    identity, row, accepted_at = snapshot
    age = (datetime.now(UTC) - datetime.fromisoformat(row["utc"])).total_seconds()
    if (not all(math.isfinite(row[key]) for key in ("vn", "ve", "vd", "latitude", "longitude", "relative_altitude"))
            or time.monotonic() - accepted_at > 3 or not 0 <= age <= 3
            or row["local_age"] > 0.8 or row["gps_age"] > 0.8
            or not row["armed"] or math.hypot(row["vn"], row["ve"]) > 0.3 or abs(row["vd"]) > 0.3):
        raise RuntimeError("replan telemetry is stale or aircraft is not holding")
    return identity, row


def activate_global_replan(admin: AdminStops, contract: dict[str, str], old_plan: Path,
                           checker: Path, latest: Callable[[], Snapshot | None],
                           evidence: Path) -> tuple[dict[str, str], Path, str]:
    """Create fixture zone, global route, native cross-check and admin CAS activation."""
    _, pose = _fresh_held(latest())
    old = json.loads(old_plan.read_text())
    if old["payload"].get("altitude_reference") != "home_relative":
        raise ValueError("original plan has no verified home-relative altitude reference")
    required = old["payload"]["required_stops"]
    if len(required) < 3:
        raise ValueError("original route has no ordered delivery stops")
    ceiling = float(old["payload"]["constraints"]["max_altitude_m"])
    level = float(required[0]["altitude_m"])
    next_level = float(required[1]["altitude_m"])
    if (not all(math.isfinite(x) for x in (ceiling, level, next_level)) or not 0 < level <= ceiling
            or abs(next_level - level) > 0.1
            or abs(pose["relative_altitude"] - level) > 1.0):
        raise ValueError("held altitude is outside the verified level and fleet ceiling")
    origin = Origin(37.0, 127.0, 0.0)
    corners = [origin.to_geographic(Enu(east, north, 0))
               for east, north in ((-2, 23), (2, 23), (2, 27), (-2, 27))]
    zone = _request(admin, "routes/no-fly-zones", {"name": "synthetic-sitl-sphere-exclusion",
        "polygon": [{"lat": p.latitude_deg, "lon": p.longitude_deg, "altitude_m": 0} for p in corners]})
    zone_id = str(zone["id"])
    (evidence / "auto-obstacle-zone.json").write_text(json.dumps(zone, indent=2))
    request = {"name": "synthetic-auto-held-replacement", "drone_id": DRONE,
        "cruise_speed_mps": 5, "max_altitude_m": ceiling, "expires_in_seconds": 600,
        "altitude_reference": "home_relative",
        "waypoints": [{"lat": pose["latitude"], "lon": pose["longitude"],
                       "altitude_m": level}, *required[1:]]}
    plan = _request(admin, "routes/plan", request)
    native = subprocess.run([str(checker), "--drone-id", DRONE, "--fingerprint",
        plan["route_fingerprint"], "--min-power-w", "120", "--battery-wh", "500"],
        input=plan["plan_text"], text=True, capture_output=True, check=True, timeout=10)
    report = json.loads(native.stdout)
    verified = _request(admin, f"routes/plans/{plan['id']}/cross-check", report)
    if verified.get("status") != "verified":
        raise ValueError("global replacement was not native verified")
    path = evidence / "auto-obstacle-activated-plan.json"
    path.write_text(json.dumps(verified, indent=2))
    for _ in range(20):
        telemetry_id, _ = _fresh_held(latest())
        try:
            activated = _request(admin, f"admin/deliveries/{contract['QA_DELIVERY']}/activate-plan", {
                "plan_id": plan["id"], "expected_route_id": contract["QA_ROUTE"],
                "expected_telemetry_id": telemetry_id, "altitude_reference": "plan"})
            break
        except urllib.error.HTTPError as exc:
            if exc.code != 409:
                raise
            time.sleep(0.2)
    else:
        raise RuntimeError("backend activation compare-and-swap did not converge")
    if activated.get("route_id") != plan["route_id"] or activated.get("plan_id") != plan["id"]:
        raise ValueError("activation did not bind the replacement plan")
    (evidence / "auto-obstacle-activation.json").write_text(json.dumps(activated, indent=2))
    new_contract = contract | {"QA_PLAN": plan["id"], "QA_ROUTE": plan["route_id"],
                               "QA_FINGERPRINT": plan["route_fingerprint"]}
    return new_contract, path, zone_id
