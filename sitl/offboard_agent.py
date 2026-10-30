"""Loopback-only PX4 SITL control transport; host must enforce mission safety."""
from __future__ import annotations

import json
import math
import os
import select
import socket
import sys
import time
from datetime import UTC, datetime

from pymavlink import mavutil


def emit(kind: str, **fields: float | str | bool | None) -> None:
    print(json.dumps({"kind": kind, "monotonic_s": time.monotonic(), **fields}), flush=True)


def main() -> int:
    if os.environ.get("NV_SITL_ONLY") != "network-none" or socket.if_nameindex() != [(1, "lo")]:
        raise RuntimeError("refusing to command a non-isolated SITL network")
    if sys.argv[1:] != ["--synthetic-mission"]:
        raise RuntimeError("explicit synthetic mission opt-in is required")
    vehicle = mavutil.mavlink_connection("udpin:127.0.0.1:14540", source_system=245)
    if vehicle.wait_heartbeat(timeout=30) is None or vehicle.target_system != 1:
        raise RuntimeError("isolated PX4 heartbeat missing")
    position = None
    gps = None
    battery = None
    armed = False
    custom_mode = 0
    state = "standby"
    setpoint = None
    last_position = 0.0
    last_gps = 0.0
    gps_received_utc = None
    last_command = time.monotonic()
    next_tick = last_command
    stats_at = last_command
    last_setpoint = 0.0
    last_target_log = 0.0
    setpoint_count = 0
    max_gap = 0.0
    next_heartbeat = last_command
    last_report = 0.0
    start_at = 0.0
    landed_requested = False
    drop_local = False
    pending = b""
    try:
        while True:
            now = time.monotonic()
            while True:
                message = vehicle.recv_match(blocking=False)
                if message is None:
                    break
                kind = message.get_type()
                if kind == "LOCAL_POSITION_NED" and not drop_local:
                    position = message
                    last_position = now
                elif kind == "GLOBAL_POSITION_INT":
                    gps = message
                    last_gps = now
                    gps_received_utc = datetime.now(UTC).isoformat()
                elif kind == "SYS_STATUS" and message.battery_remaining >= 0:
                    battery = float(message.battery_remaining)
                elif kind == "HEARTBEAT" and message.get_srcSystem() == 1:
                    armed = bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    custom_mode = message.custom_mode
                elif kind == "COMMAND_ACK":
                    emit("ack", command=message.command, result=message.result)
                elif kind == "STATUSTEXT":
                    emit("status", text=message.text)
            if position is not None and gps is not None and now - last_report >= 0.2:
                emit("telemetry", north=position.x, east=position.y, down=position.z,
                     vn=position.vx, ve=position.vy, vd=position.vz,
                     latitude=gps.lat / 1e7, longitude=gps.lon / 1e7,
                     altitude_msl=gps.alt / 1000, relative_altitude=gps.relative_alt / 1000,
                     heading=gps.hdg / 100, battery=battery, armed=armed,
                     px4_custom_mode=custom_mode,
                     utc=gps_received_utc,
                     local_age=now - last_position, gps_age=now - last_gps)
                last_report = now
            if now >= next_heartbeat:
                vehicle.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                    mavutil.mavlink.MAV_STATE_ACTIVE)
                next_heartbeat = now + 1.0
            ready, _, _ = select.select([sys.stdin], [], [], 0)
            if ready:
                chunk = os.read(sys.stdin.fileno(), 4096)
                if not chunk:
                    emit("failsafe", reason="host_eof")
                    break
                pending += chunk
                if len(pending) > 16384:
                    emit("failsafe", reason="host_input_too_large")
                    break
            if b"\n" in pending:
                line, _, pending = pending.partition(b"\n")
                try:
                    command = json.loads(line)
                    operation = command["op"]
                    if operation == "target":
                        xyz = tuple(float(command[key]) for key in ("north", "east", "down"))
                        if not all(math.isfinite(v) for v in xyz):
                            raise ValueError("non-finite target")
                        if state not in ("starting", "flying"):
                            raise ValueError("target while not flying")
                        setpoint = xyz
                        last_command = now
                    elif operation == "start" and state == "standby":
                        if position is None or gps is None or now - last_position > 1 or now - last_gps > 1:
                            raise ValueError("fresh PX4 position and GPS are required")
                        setpoint = (position.x, position.y, position.z)
                        state, start_at, last_command = "starting", now, now
                        emit("transition", state=state)
                    elif (operation == "inject-local-loss" and armed
                          and os.environ.get("NV_SITL_FAULT_TEST") == "1"):
                        drop_local = True
                        emit("fault_injection", source="local_position_suppressed")
                    elif operation == "land" and state in ("starting", "flying"):
                        vehicle.mav.command_long_send(vehicle.target_system, vehicle.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
                        landed_requested = True
                        state = "landing"
                        emit("transition", state=state)
                    else:
                        raise ValueError("unsupported control transition")
                except (ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
                    emit("failsafe", reason=f"invalid_host_command: {exc}")
                    break
            if state in ("starting", "flying"):
                if now - last_position > 1 or now - last_gps > 1:
                    emit("failsafe", reason="px4_telemetry_stale")
                    break
                if now - last_command > 1.5:
                    emit("failsafe", reason="host_decision_stale")
                    break
                if setpoint is not None and now >= next_tick:
                    vehicle.mav.set_position_target_local_ned_send(
                        0, vehicle.target_system, vehicle.target_component,
                        mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0x0DF8,
                        *setpoint, 0, 0, 0, 0, 0, 0, 0, 0)
                    if now - last_target_log >= 0.2:
                        emit("setpoint", north=setpoint[0], east=setpoint[1], down=setpoint[2])
                        last_target_log = now
                    if last_setpoint:
                        max_gap = max(max_gap, now - last_setpoint)
                    last_setpoint = now
                    setpoint_count += 1
                    next_tick += 0.05
                    if next_tick < now:
                        next_tick = now + 0.05
                    if now - stats_at >= 1:
                        emit("setpoint_stats", count=setpoint_count,
                             interval_s=now - stats_at, max_gap_s=max_gap)
                        stats_at, setpoint_count, max_gap = now, 0, 0.0
                if state == "starting" and now - start_at >= 2:
                    vehicle.mav.set_mode_send(vehicle.target_system,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6 << 16)
                    vehicle.mav.command_long_send(vehicle.target_system, vehicle.target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
                    state = "flying"
                    emit("transition", state=state)
            if state == "landing" and not armed:
                emit("transition", state="landed")
                return 0
            time.sleep(0.005)
    finally:
        if armed and not landed_requested:
            vehicle.mav.command_long_send(vehicle.target_system, vehicle.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
            emit("failsafe", reason="land_on_exit")
        vehicle.close()
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
