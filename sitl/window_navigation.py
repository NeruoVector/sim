#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
# Run: python3 sim/sitl/window_navigation.py PLAN CHECKER BRIDGE --synthetic-only
"""Independent synthetic facade navigation; never contacts the backend."""
from __future__ import annotations

import json
import math
import queue
import subprocess
import sys
import threading
import time
from datetime import UTC, datetime
from pathlib import Path

from coordinates import (
    Enu,
    Geographic,
    Ned,
    Origin,
    local_target,
    require_route_corridor,
    require_synthetic_start,
)
from fixture_avoidance import FixtureAvoidance
from nvplan_route import GeoWaypoint, required_indices, selected_route

ROOT = Path(__file__).resolve().parents[2]
CONTAINER = "nv-window-sitl-b730"
ORIGIN = Origin(37.0, 127.0, 0.0)


def facade_clearance(position: Enu) -> float:
    return 40.0 - position.north_m


def progress_target(position: Enu, goal: Enu, level: bool) -> Enu:
    dx, dy, dz = goal.east_m-position.east_m, goal.north_m-position.north_m, goal.up_m-position.up_m
    length = math.sqrt(dx*dx + dy*dy + dz*dz)
    scale = min(1.0, 0.8 / length) if length else 0.0
    return Enu(position.east_m + dx*scale, position.north_m + dy*scale,
               goal.up_m if level else position.up_m + dz*scale)


class Hold:
    """Mutable timer resets whenever any measured hold condition fails."""
    def __init__(self) -> None:
        self.since: float | None = None

    def update(self, now: float, valid: bool, duration: float) -> bool:
        if not valid:
            self.since = None
            return False
        if self.since is None:
            self.since = now
        return now - self.since >= duration


def log(kind: str, **fields: float | str | bool) -> None:
    print(json.dumps({"kind": kind, "monotonic_s": time.monotonic(), **fields}), flush=True)


def verified_route(plan_path: Path, checker: Path) -> tuple[list[Enu], datetime, float]:
    plan = json.loads(plan_path.read_text())
    if plan["status"] != "verified" or plan["drone_id"] != "school-drone-001":
        raise RuntimeError("plan is not a verified synthetic window plan")
    expiry = datetime.fromisoformat(plan["expires_at"])
    if expiry <= datetime.now(UTC):
        raise RuntimeError("plan expired")
    wire = plan["plan_text"]
    check = subprocess.run([str(checker), "--drone-id", plan["drone_id"], "--fingerprint",
        plan["route_fingerprint"], "--min-power-w", "120", "--battery-wh", "500"],
        input=wire, text=True, capture_output=True, check=True, timeout=10)
    result = json.loads(check.stdout)
    log("native_check", report=check.stdout.strip())
    required, candidate, selected = selected_route(wire)
    payload = plan["payload"]
    encoded = [GeoWaypoint(p["lat"], p["lon"], p["altitude_m"]) for p in payload["candidates"][selected]["waypoints"]]
    encoded_required = [GeoWaypoint(p["lat"], p["lon"], p["altitude_m"]) for p in payload["required_stops"]]
    if not result["valid"] or result["selected_index"] != selected or selected != payload["selected_index"] or candidate != encoded or required != encoded_required:
        raise RuntimeError("wire candidate or native validation mismatch")
    required_indices(required, candidate)
    window = payload["window_target"]
    wire_window = wire.splitlines()[2].split()
    expected_window = [window["window"]["lat"], window["window"]["lon"],
                       window["window"]["altitude_m"], window["outward_bearing_deg"],
                       window["standoff_m"], window["approach_distance_m"], window["hold_seconds"]]
    if len(wire_window) != 8 or wire_window[0] != "1" or any(
            not math.isclose(float(a), b, abs_tol=1e-10, rel_tol=0.0)
            for a, b in zip(wire_window[1:], expected_window)):
        raise RuntimeError("payload window differs from native-validated wire")
    point = ORIGIN.to_enu(Geographic(window["window"]["lat"], window["window"]["lon"], window["window"]["altitude_m"]))
    if math.dist((point.east_m, point.north_m, point.up_m), (30, 40, 8)) > 0.05 or window["outward_bearing_deg"] != 180 or window["standoff_m"] != 3 or window["hold_seconds"] != 30:
        raise RuntimeError("window does not match procedural facade")
    route = [ORIGIN.to_enu(Geographic(p.lat, p.lon, p.altitude_m)) for p in candidate]
    if len(route) < 2 or any(math.hypot(p.east_m, p.north_m) > 100 or not 0 < p.up_m <= 10 or facade_clearance(p) < 2.99 for p in route):
        raise RuntimeError("route exceeds synthetic bounds")
    return route, expiry, float(window["hold_seconds"])


def run(plan: Path, checker: Path, bridge: Path) -> int:
    isolation = subprocess.run(["docker", "inspect", "--format", "{{.HostConfig.NetworkMode}} {{.HostConfig.IpcMode}} {{len .HostConfig.Devices}} {{len .HostConfig.PortBindings}}", CONTAINER], capture_output=True, text=True, check=True, timeout=5).stdout.strip()
    if isolation != "none private 0 0":
        raise RuntimeError(f"unsafe container isolation: {isolation}")
    route, expiry, duration = verified_route(plan, checker)
    policy = FixtureAvoidance(ROOT / "sim/fixtures/school.json", bridge)
    vehicle = subprocess.Popen(["docker", "exec", "-i", "-e", "NV_SITL_ONLY=network-none", CONTAINER, "python3", "/nv-sim/sitl/offboard_agent.py", "--synthetic-mission"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
    if vehicle.stdin is None or vehicle.stdout is None:
        raise RuntimeError("vehicle transport unavailable")
    events: queue.Queue[str] = queue.Queue()
    def reader() -> None:
        for line in vehicle.stdout:
            events.put(line)
        events.put("")
    thread = threading.Thread(target=reader, daemon=True)
    thread.start()
    started = time.monotonic()
    state, index, checked_start = "waiting", 0, False
    active = route
    timer = Hold()
    hold_point: Enu | None = None
    try:
        while time.monotonic() - started < 480:
            try:
                line = events.get(timeout=35 if state == "waiting" else 1)
            except queue.Empty:
                raise TimeoutError("vehicle telemetry timed out") from None
            if not line:
                raise RuntimeError("vehicle transport closed")
            row = json.loads(line)
            log("agent", event=line.strip())
            if row["kind"] != "telemetry":
                if row["kind"] == "failsafe":
                    raise RuntimeError("vehicle agent failsafe")
                if row.get("state") == "landed":
                    if state != "landing":
                        raise RuntimeError("unexpected landing")
                    log("mission_complete", outcome="success")
                    return 0
                continue
            now = time.monotonic()
            if max(row["local_age"], row["gps_age"], now - row["monotonic_s"]) > 0.8:
                raise RuntimeError("stale PX4 position")
            if row["battery"] is None and state == "waiting":
                continue
            if row["battery"] is None or not 20 <= row["battery"] <= 100 or datetime.now(UTC) >= expiry:
                raise RuntimeError("battery or expiry guard")
            pos = ORIGIN.to_enu(Geographic(row["latitude"], row["longitude"], row["relative_altitude"]))
            velocity = Enu(row["ve"], row["vn"], -row["vd"])
            speed = math.sqrt(velocity.east_m**2 + velocity.north_m**2 + velocity.up_m**2)
            clearance = facade_clearance(pos)
            log("position", east=pos.east_m, north=pos.north_m, up=pos.up_m, clearance=clearance, speed=speed, phase=state, waypoint=index)
            if clearance < 3.0 or math.hypot(velocity.east_m, velocity.north_m) > 2 or abs(velocity.up_m) > 2:
                raise RuntimeError("facade clearance or velocity guard")
            require_route_corridor(active, index, pos, landing=state == "landing")
            if state == "landing":
                continue
            if not checked_start:
                if row["armed"]:
                    raise RuntimeError("vehicle already armed")
                require_synthetic_start(pos)
                checked_start = True
            decision = policy.decide(now, pos, velocity, active[index:])
            action = decision["action"]
            log("decision", action=action, reason=decision["reason"], waypoint=index)
            if state == "waiting":
                if action != "proceed":
                    raise RuntimeError("native policy blocked takeoff")
                vehicle.stdin.write('{"op":"start"}\n')
                vehicle.stdin.flush()
                state = "outbound"
            goal = active[index]
            # Aim half a metre outside the plane while remaining within waypoint tolerance.
            if state == "outbound" and index == len(active)-1:
                goal = Enu(goal.east_m, goal.north_m - 0.5, goal.up_m)
            distance = math.dist((goal.east_m, goal.north_m, goal.up_m), (pos.east_m, pos.north_m, pos.up_m))
            stable = row["armed"] and distance < 0.35 and speed < 0.5 and action == "proceed"
            needed = duration if state == "outbound" and index == len(active)-1 else 1.0
            if timer.update(now, stable, needed):
                log("waypoint_reached", waypoint=index, duration=needed, phase=state, clearance=clearance)
                timer = Hold()
                index += 1
                if index == len(active):
                    if state == "outbound":
                        state, active, index = "return", list(reversed(route)), 1
                        log("postmission_return", detail="reverse route is separate from wire candidate")
                    else:
                        vehicle.stdin.write('{"op":"land"}\n')
                        vehicle.stdin.flush()
                        state, index = "landing", len(active)-1
                        continue
                goal = active[index]
            if action != "proceed":
                hold_point = hold_point or pos
                target = hold_point
            else:
                hold_point = None
                target = progress_target(pos, goal, index > 0 and abs(goal.up_m-active[index-1].up_m) < 0.1)
            ned = local_target(Ned(row["north"], row["east"], row["down"]), pos, target)
            command = {"op": "target", "north": ned.north_m, "east": ned.east_m, "down": ned.down_m}
            log("target", command=json.dumps(command))
            vehicle.stdin.write(json.dumps(command) + "\n")
            vehicle.stdin.flush()
        raise TimeoutError("mission deadline exceeded")
    finally:
        vehicle.stdin.close()
        try:
            vehicle.wait(timeout=8)
        except subprocess.TimeoutExpired:
            vehicle.terminate()
            vehicle.wait(timeout=5)
        thread.join(timeout=2)
        policy.close()


def main() -> int:
    if len(sys.argv) != 5 or sys.argv[4] != "--synthetic-only":
        raise RuntimeError("usage: window_navigation.py PLAN CHECKER BRIDGE --synthetic-only")
    return run(Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3]))


if __name__ == "__main__":
    raise SystemExit(main())
