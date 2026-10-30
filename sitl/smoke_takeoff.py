"""Bounded PX4 position-control smoke, only inside a network-none SITL container."""
from __future__ import annotations

import json
import math
import os
import socket
import sys
import time

from pymavlink import mavutil


def event(kind: str, **fields: float | str | bool) -> None:
    print(json.dumps({"kind": kind, "monotonic_s": time.monotonic(), **fields}), flush=True)


def main() -> int:
    if os.environ.get("NV_SITL_ONLY") != "network-none" or socket.if_nameindex() != [(1, "lo")]:
        raise RuntimeError("refusing to command a non-isolated SITL network")
    if sys.argv[1:] not in (["--synthetic-takeoff-smoke"], ["--synthetic-offboard-loss"]):
        raise RuntimeError("explicit synthetic test opt-in is required")
    loss_test = sys.argv[1:] == ["--synthetic-offboard-loss"]
    vehicle = mavutil.mavlink_connection("udpin:127.0.0.1:14540", source_system=245)
    heartbeat = vehicle.wait_heartbeat(timeout=30)
    if heartbeat is None or heartbeat.get_srcSystem() != 1:
        raise RuntimeError("expected isolated PX4 heartbeat not received")
    event("heartbeat", system=heartbeat.get_srcSystem(), mode=heartbeat.base_mode)
    deadline = time.monotonic() + 30
    position = None
    while time.monotonic() < deadline:
        message = vehicle.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1)
        if message and all(math.isfinite(v) for v in (message.x, message.y, message.z)):
            position = message
            break
    if position is None:
        raise RuntimeError("no finite PX4 local position")
    north, east, ground_down = position.x, position.y, position.z
    target_down = ground_down - 2.0
    event("origin", north=north, east=east, down=ground_down, target_down=target_down)
    start = time.monotonic()
    last_position = start
    next_setpoint = start
    next_heartbeat = start
    offboard_sent = False
    arm_sent = False
    armed = False
    hover_since = None
    landed_command = False
    loss_started = None
    offboard_left = False
    last_log = 0.0
    try:
        while time.monotonic() - start < 75:
            now = time.monotonic()
            while True:
                message = vehicle.recv_match(blocking=False)
                if message is None:
                    break
                kind = message.get_type()
                if kind == "LOCAL_POSITION_NED":
                    position = message
                    last_position = now
                    if now - last_log >= 0.2:
                        event("position", north=message.x, east=message.y, down=message.z,
                              vn=message.vx, ve=message.vy, vd=message.vz)
                        last_log = now
                elif kind == "HEARTBEAT" and message.get_srcSystem() == 1:
                    armed = bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    if loss_started is not None and (message.custom_mode >> 16) & 0xff != 6:
                        offboard_left = True
                        event("offboard_loss_mode", custom_mode=message.custom_mode)
                elif kind == "COMMAND_ACK":
                    event("ack", command=message.command, result=message.result)
                elif kind == "STATUSTEXT":
                    event("status", text=message.text, severity=message.severity)
            if now - last_position > 1.5:
                event("failsafe", reason="local_position_stale")
                return 2
            if now >= next_heartbeat:
                vehicle.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                    mavutil.mavlink.MAV_STATE_ACTIVE)
                next_heartbeat = now + 1.0
            if not landed_command and loss_started is None and now >= next_setpoint:
                vehicle.mav.set_position_target_local_ned_send(
                    0, vehicle.target_system, vehicle.target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0x0DF8,
                    north, east, target_down, 0, 0, 0, 0, 0, 0, 0, 0,
                )
                next_setpoint += 0.05
                if next_setpoint < now:
                    next_setpoint = now + 0.05
            elapsed = now - start
            if elapsed >= 2 and not offboard_sent:
                vehicle.mav.set_mode_send(vehicle.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6 << 16)
                offboard_sent = True
                event("command", name="offboard")
            if elapsed >= 3 and not arm_sent:
                vehicle.mav.command_long_send(vehicle.target_system, vehicle.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
                arm_sent = True
                event("command", name="arm_without_force")
            if arm_sent and not armed and elapsed > 12 and not landed_command and loss_started is None:
                event("failsafe", reason="arm_not_confirmed")
                return 3
            if armed and not landed_command and loss_started is None and abs(position.z - target_down) < 0.35:
                if hover_since is None:
                    hover_since = now
                if now - hover_since >= 3:
                    if loss_test:
                        loss_started = now
                        event("synthetic_setpoint_loss", state="begin")
                    else:
                        vehicle.mav.command_long_send(vehicle.target_system, vehicle.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
                        landed_command = True
                        event("command", name="land")
            elif not landed_command:
                hover_since = None
            if loss_started is not None and not armed:
                event("completed", offboard_loss_disarmed=offboard_left)
                return 0 if offboard_left else 5
            if loss_started is not None and now - loss_started > 30:
                event("failsafe", reason="offboard_loss_did_not_land")
                return 6
            if landed_command and not armed:
                event("completed", takeoff_hover_land=True)
                return 0
            time.sleep(0.01)
        event("failsafe", reason="timeout")
        return 4
    finally:
        if armed and not landed_command:
            vehicle.mav.command_long_send(vehicle.target_system, vehicle.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
            event("command", name="land_on_exit")
        vehicle.close()


if __name__ == "__main__":
    raise SystemExit(main())
