"""Execute one verified synthetic delivery through isolated PX4 and native avoidance."""
from __future__ import annotations

import argparse
import json
import math
import queue
import subprocess
import threading
import time
from concurrent.futures import Future, ThreadPoolExecutor
from datetime import UTC, datetime
from pathlib import Path

from altitude_reference import require_running_world
from auto_replan import Snapshot
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
from gazebo_obstacle import CONTAINERS as OBSTACLE_CONTAINERS
from mission_backend import AdminStops, fetch_plan, post_telemetry
from motion_policy import movement_target
from replan_coordinator import ReplanCoordinator
from stops import bind_stops, stop_is_complete

SIM = Path(__file__).resolve().parents[1]



def log(kind: str, **values: float | str | bool) -> None:
    print(json.dumps({"kind": kind, "monotonic_s": time.monotonic(), **values}), flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plan", type=Path)
    parser.add_argument("checker", type=Path)
    parser.add_argument("bridge", type=Path)
    parser.add_argument("contract", type=Path)
    parser.add_argument("--container", choices=("nv-autonomy-sitl-7bd1", "nv-autonomy-sitl-retry-7bd1",
                                               "nv-colocated-sitl-ad1", *OBSTACLE_CONTAINERS),
                        default="nv-autonomy-sitl-7bd1")
    parser.add_argument("--synthetic-only", action="store_true", required=True)
    parser.add_argument("--exercise-stale-observation", action="store_true")
    parser.add_argument("--exercise-replan-hold", action="store_true")
    parser.add_argument("--exercise-physical-obstacle", action="store_true")
    parser.add_argument("--auto-global-replan", action="store_true")
    args = parser.parse_args()
    if args.auto_global_replan and not args.exercise_physical_obstacle:
        raise ValueError("automatic global replan requires the physical synthetic obstacle")
    if args.exercise_physical_obstacle and (args.container not in OBSTACLE_CONTAINERS or args.exercise_replan_hold):
        raise ValueError("physical obstacle test requires its own isolated container")
    isolation = subprocess.run(["docker", "inspect", "--format",
        "{{.HostConfig.NetworkMode}} {{.HostConfig.IpcMode}} {{len .HostConfig.Devices}}",
        args.container], text=True, capture_output=True, check=True, timeout=5).stdout.strip()
    if isolation != "none private 0":
        raise RuntimeError(f"unsafe SITL isolation: {isolation}")
    require_running_world(args.container,
        SIM.parent / ".omo/evidence/drone-autonomy/sim/school_procedural.sdf")
    contract = dict(line.split("=", 1) for line in args.contract.read_text().splitlines()
                    if line and not line.startswith("#") and "=" in line)
    fixture = json.loads((SIM / "fixtures/school.json").read_text())
    if fixture.get("synthetic") is not True or fixture.get("origin") != {"lat":37.0,"lon":127.0}:
        raise ValueError("unexpected synthetic school origin")
    route, expiry, indices = fetch_plan(args.plan, args.checker, contract)
    admin = AdminStops(contract)
    stops = bind_stops(admin.read(), indices)
    admin.verify_locations(stops, route)
    customer_by_waypoint = {stop.waypoint_index: index for index, stop in enumerate(stops)}
    replanner = ReplanCoordinator(args, admin, SIM.parent, log)
    policy = FixtureAvoidance(SIM / "fixtures/school.json", args.bridge)
    vehicle = subprocess.Popen(["docker", "exec", "-i", "-e", "NV_SITL_ONLY=network-none",
        args.container, "python3", "/nv-sim/sitl/offboard_agent.py", "--synthetic-mission"],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
    if vehicle.stdin is None or vehicle.stdout is None:
        raise RuntimeError("SITL transport unavailable")
    events: queue.Queue[str] = queue.Queue()
    def reader() -> None:
        for line in vehicle.stdout:
            events.put(line)
        events.put("")
    thread = threading.Thread(target=reader, daemon=True)
    thread.start()
    index, state = 0, "waiting"
    start_verified = False
    hover_since: float | None = None
    stale_started: float | None = None
    stale_complete = False
    hold_point: Enu | None = None
    last_post = 0.0
    last_frame = time.monotonic()
    started = last_frame
    future: Future[tuple[int, int]] | None = None
    posted_row: dict | None = None
    latest_accepted: list[Snapshot | None] = [None]
    tracking_future: Future[list] | None = None
    tracking_at = 0.0
    handoff_complete = False
    hover_announced = False
    pickup_started: float | None = None
    aborted = False
    origin = Origin(37.0, 127.0, 0.0)
    with ThreadPoolExecutor(max_workers=3) as executor:
        try:
            while time.monotonic() - started < (480 if args.exercise_replan_hold or args.exercise_physical_obstacle else 300):
                try:
                    line = events.get(timeout=35 if state == "waiting" else 1)
                except queue.Empty:
                    raise TimeoutError("no current vehicle telemetry") from None
                if not line:
                    raise RuntimeError("SITL transport closed")
                row = json.loads(line)
                if row["kind"] != "telemetry":
                    log("vehicle", event=json.dumps(row))
                    if row["kind"] == "failsafe":
                        raise RuntimeError("SITL agent reported failsafe")
                    if row.get("state") == "landed":
                        log("mission_complete", visited_waypoints=index, aborted=aborted)
                        return 2 if aborted else 0
                    continue
                now = time.monotonic()
                log("px4_telemetry", sample=json.dumps(row))
                if row["local_age"] > 0.8 or row["gps_age"] > 0.8 or now - row["monotonic_s"] > 0.8:
                    raise RuntimeError("stale PX4 telemetry")
                if row["battery"] is None and state == "waiting":
                    continue
                if row["battery"] is None or not 20 <= row["battery"] <= 100:
                    raise RuntimeError("simulated battery reserve unavailable")
                if datetime.now(UTC) >= expiry:
                    raise RuntimeError("mission expired during execution")
                if tracking_future is not None and tracking_future.done():
                    handoff_complete = stop_is_complete(tracking_future.result(), stops, customer_by_waypoint[index])
                    tracking_future = None
                    if handoff_complete:
                        log("handoff_confirmed", waypoint=index)
                if future is not None and future.done():
                    status, telemetry_id = future.result()
                    if status != 201:
                        raise RuntimeError("backend telemetry failed")
                    if posted_row is None:
                        raise RuntimeError("accepted telemetry sample is missing")
                    latest_accepted[0] = telemetry_id, posted_row, now
                    log("backend_telemetry_accepted", telemetry_id=telemetry_id)
                    future = None
                if now - last_post >= 1 and future is None:
                    posted_row = row.copy()
                    future = executor.submit(post_telemetry, posted_row, home_verified=start_verified)
                    last_post = now
                geographic = Geographic(row["latitude"], row["longitude"], row["relative_altitude"])
                pos = origin.to_enu(geographic)
                vel = Enu(row["ve"], row["vn"], -row["vd"])
                if math.hypot(vel.east_m, vel.north_m) > 2.0 or abs(vel.up_m) > 2.0:
                    raise RuntimeError("actual PX4 velocity exceeded the safety bound")
                require_route_corridor(route, index, pos, landing=state == "landing")
                if state == "landing":
                    continue
                if not start_verified:
                    if row["armed"]:
                        raise ValueError("refusing to start with an armed vehicle")
                    require_synthetic_start(pos)
                    start_verified = True
                if args.exercise_stale_observation and index == 1 and not stale_complete:
                    if stale_started is None:
                        stale_started = now
                        log("synthetic_sensor_blackout", state="begin")
                    if now - stale_started < 3:
                        observation_time = stale_started - 2
                    else:
                        stale_complete = True
                        observation_time = now
                        log("synthetic_sensor_blackout", state="end")
                else:
                    observation_time = now
                replanner.observe(index, pos, executor, policy)
                decision = policy.decide(now, pos, vel, route[index:], observation_time=observation_time)
                action = decision["action"]
                log("decision", action=action, reason=decision["reason"], waypoint=index,
                    east=pos.east_m, north=pos.north_m, up=pos.up_m,
                    desired_speed_mps=decision["desired_speed_mps"])
                action, replacement = replanner.decide(now, pos, vel, index, decision,
                    route, indices, stops, contract, executor, lambda: latest_accepted[0])
                if replacement is not None:
                    route, expiry, indices, stops = replacement
                    customer_by_waypoint = {stop.waypoint_index: i for i, stop in enumerate(stops)}
                    index, hold_point = 1, pos
                if state == "waiting":
                    if action != "proceed":
                        raise RuntimeError("native safety decision blocks takeoff")
                    vehicle.stdin.write('{"op":"start"}\n')
                    vehicle.stdin.flush()
                    state = "flying"
                goal = route[index]
                distance = math.sqrt((goal.east_m-pos.east_m)**2 + (goal.north_m-pos.north_m)**2 +
                                     (goal.up_m-pos.up_m)**2)
                speed = math.sqrt(vel.east_m**2 + vel.north_m**2 + vel.up_m**2)
                if row["armed"] and distance < 0.75 and speed < 0.7 and action == "proceed":
                    if hover_since is None:
                        hover_since = now
                    customer_stop = index in customer_by_waypoint
                    duration = 20 if customer_stop else 1
                    if customer_stop and now - hover_since >= 3 and not hover_announced:
                        log("destination_hover", order_id=stops[customer_by_waypoint[index]].order_id,
                            stop_sequence=stops[customer_by_waypoint[index]].sequence,
                            east=pos.east_m, north=pos.north_m, up=pos.up_m)
                        hover_announced = True
                        pickup_started = now
                    if (customer_stop and pickup_started is not None
                            and now - pickup_started > 120 and not handoff_complete):
                        vehicle.stdin.write('{"op":"land"}\n')
                        vehicle.stdin.flush()
                        state, aborted = "landing", True
                        log("pickup_timeout", outcome="land_without_handoff")
                        continue
                    if (customer_stop and now - hover_since >= 15 and not handoff_complete
                            and now - tracking_at >= 2 and tracking_future is None):
                        tracking_future = executor.submit(admin.read)
                        tracking_at = now
                    if now - hover_since >= duration and (not customer_stop or handoff_complete):
                        index += 1
                        hover_since = None
                        handoff_complete = False
                        hover_announced = False
                        pickup_started = None
                        log("waypoint_reached", index=index, duration=duration)
                        if index == len(route):
                            vehicle.stdin.write('{"op":"land"}\n')
                            vehicle.stdin.flush()
                            state = "landing"
                            continue
                        goal = route[index]
                else:
                    hover_since = None
                move = movement_target(pos, goal, action, decision["desired_speed_mps"], speed,
                    level_leg=index > 0 and abs(goal.up_m - route[index - 1].up_m) < 0.1)
                if move is None:
                    if hold_point is None:
                        hold_point = pos
                    target = hold_point
                else:
                    hold_point = None
                    target = move
                ned_target = local_target(Ned(row["north"], row["east"], row["down"]), pos, target)
                vehicle.stdin.write(json.dumps({"op":"target", "north":ned_target.north_m,
                    "east":ned_target.east_m, "down":ned_target.down_m}) + "\n")
                vehicle.stdin.flush()
                last_frame = now
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


if __name__ == "__main__":
    raise SystemExit(main())
