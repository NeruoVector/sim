"""Backend access for the isolated synthetic mission controller."""
from __future__ import annotations

import http.cookiejar
import json
import math
import subprocess
import urllib.request
from datetime import UTC, datetime
from pathlib import Path

from coordinates import Enu, Geographic, Origin
from nvplan_route import GeoWaypoint, required_indices, selected_route
from stops import Stop, StopRecord

API = "http://127.0.0.1:8083"
DRONE = "school-drone-001"


def require_plan_reference(expected: dict, payload: dict) -> None:
    if (payload.get("altitude_reference") != "home_relative"
            or expected.get("payload", {}).get("altitude_reference") != "home_relative"):
        raise ValueError("verified plan does not declare the calibrated home-relative altitude datum")


def fetch_plan(path: Path, checker: Path, contract: dict[str, str]) -> tuple[list[Enu], datetime, tuple[int, ...]]:
    expected = json.loads(path.read_text())
    route_id = expected["route_id"]
    if (contract.get("QA_API") != API or contract.get("QA_DRONE") != DRONE
            or contract.get("QA_PLAN") != expected["id"] or contract.get("QA_ROUTE") != route_id
            or contract.get("QA_FINGERPRINT") != expected["route_fingerprint"]):
        raise ValueError("synthetic contract does not match mission")
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open(f"{API}/api/v1/routes/plans/by-route/{route_id}", timeout=3) as reply:
        plan = json.load(reply)
    with opener.open(f"{API}/api/v1/deliveries/{contract['QA_DELIVERY']}", timeout=3) as reply:
        delivery = json.load(reply)
    if (delivery["status"] != "in_flight" or delivery["route_id"] != route_id
            or delivery["drone_id"] != DRONE):
        raise ValueError("delivery is not authorized for this active route")
    if (plan["status"] != "verified" or plan["id"] != expected["id"]
            or plan["route_fingerprint"] != expected["route_fingerprint"]
            or plan["drone_id"] != DRONE):
        raise ValueError("backend mission identity or verification differs")
    expiry = datetime.fromisoformat(plan["expires_at"])
    if expiry <= datetime.now(UTC):
        raise ValueError("mission expired")
    checked = subprocess.run([str(checker), "--drone-id", DRONE, "--fingerprint",
        plan["route_fingerprint"], "--min-power-w", "120", "--battery-wh", "500"],
        input=plan["plan_text"], text=True, capture_output=True, timeout=10, check=True)
    report = json.loads(checked.stdout)
    selected = plan["payload"]["selected_index"]
    if not report["valid"] or report["selected_index"] != selected:
        raise ValueError("native route validator rejected mission")
    required, wire_route, wire_index = selected_route(plan["plan_text"])
    payload = plan["payload"]
    require_plan_reference(expected, payload)
    encoded_route = [GeoWaypoint(p["lat"], p["lon"], p["altitude_m"])
                     for p in payload["candidates"][selected]["waypoints"]]
    encoded_required = [GeoWaypoint(p["lat"], p["lon"], p["altitude_m"])
                        for p in payload["required_stops"]]
    if wire_index != selected or wire_route != encoded_route or required != encoded_required:
        raise ValueError("mission payload differs from native-validated wire bytes")
    if payload.get("window_target") is not None:
        raise ValueError("window missions require a separate facade adapter")
    positions = required_indices(required, wire_route)
    origin = Origin(37.0, 127.0, 0.0)
    result = [origin.to_enu(Geographic(p.lat, p.lon, p.altitude_m)) for p in wire_route]
    if len(result) < 2 or any(math.hypot(p.east_m, p.north_m) > 100 or not 0 < p.up_m <= 10
                              for p in result):
        raise ValueError("mission exceeds isolated school test bounds")
    return result, expiry, positions


def telemetry_payload(row: dict, *, home_verified: bool = False) -> dict:
    payload = {"ts": row["utc"], "latitude": row["latitude"], "longitude": row["longitude"],
               "altitude_m": row["relative_altitude"], "speed_mps": math.hypot(row["vn"], row["ve"]),
               "vertical_speed_mps": -row["vd"],
               "battery_pct": row["battery"], "heading_deg": row["heading"],
               "status": "flying" if row["armed"] else "idle"}
    if home_verified:
        payload["altitude_reference"] = "home_relative"
    return payload


def post_telemetry(row: dict, *, home_verified: bool = False) -> tuple[int, int]:
    payload = telemetry_payload(row, home_verified=home_verified)
    request = urllib.request.Request(f"{API}/api/v1/flights/telemetry/{DRONE}",
        data=json.dumps(payload).encode(), headers={"Content-Type": "application/json"}, method="POST")
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open(request, timeout=1) as reply:
        result = json.load(reply)
        return reply.status, int(result["id"])



class AdminStops:
    """Authenticated stop status reads for one isolated backend."""

    def __init__(self, contract: dict[str, str]) -> None:
        jar = http.cookiejar.CookieJar()
        self.opener = urllib.request.build_opener(
            urllib.request.ProxyHandler({}), urllib.request.HTTPCookieProcessor(jar))
        request = urllib.request.Request(f"{API}/api/v1/admin/auth/login",
            data=json.dumps({"username": contract["QA_ADMIN_USER"],
                             "password": contract["QA_ADMIN_PASSWORD"]}).encode(),
            headers={"Content-Type": "application/json"}, method="POST")
        with self.opener.open(request, timeout=3) as reply:
            if reply.status != 200:
                raise RuntimeError("admin authentication failed")
        self.delivery_id = contract["QA_DELIVERY"]

    def read(self) -> list[StopRecord]:
        with self.opener.open(f"{API}/api/v1/admin/deliveries/{self.delivery_id}/stops",
                              timeout=1) as reply:
            records = json.load(reply)
        if not isinstance(records, list):
            raise TypeError("invalid admin stop response")
        return records

    def verify_locations(self, stops: tuple[Stop, ...], route: list[Enu]) -> None:
        origin = Origin(37.0, 127.0, 0.0)
        for stop in stops:
            with self.opener.open(f"{API}/api/v1/orders/{stop.order_id}", timeout=3) as reply:
                order = json.load(reply)
            if order["id"] != stop.order_id or order["delivery_id"] != self.delivery_id:
                raise ValueError("order identity differs from active delivery")
            point = origin.to_enu(Geographic(order["latitude"], order["longitude"], 0.0))
            target = route[stop.waypoint_index]
            if math.hypot(point.east_m-target.east_m, point.north_m-target.north_m) > 0.2:
                raise ValueError("order coordinates differ from verified required waypoint")
