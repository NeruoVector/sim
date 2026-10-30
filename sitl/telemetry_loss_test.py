"""Inject loss of local PX4 samples in a network-none SITL and observe landing."""
from __future__ import annotations

import json
import os
import queue
import socket
import subprocess
import sys
import threading
import time

from pymavlink import mavutil


def main() -> int:
    if (os.environ.get("NV_SITL_ONLY") != "network-none"
            or socket.if_nameindex() != [(1, "lo")]
            or sys.argv[1:] not in (["--synthetic-local-loss"], ["--synthetic-partial-command"])):
        raise RuntimeError("isolated synthetic fault-test opt-in required")
    partial_test = sys.argv[1:] == ["--synthetic-partial-command"]
    expected_failure = "host_decision_stale" if partial_test else "px4_telemetry_stale"
    env = os.environ | {"NV_SITL_FAULT_TEST": "1"}
    agent = subprocess.Popen([sys.executable, "/nv-sim/sitl/offboard_agent.py", "--synthetic-mission"],
                             stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True,
                             bufsize=1, env=env)
    if agent.stdin is None or agent.stdout is None:
        raise RuntimeError("agent pipes unavailable")
    events: queue.Queue[str] = queue.Queue()
    def reader() -> None:
        for line in agent.stdout:
            events.put(line)
        events.put("")
    thread = threading.Thread(target=reader, daemon=True)
    thread.start()
    target = None
    observed_armed = False
    fault = False
    stale_seen = False
    fallback_seen = False
    deadline = time.monotonic() + 60
    try:
        while time.monotonic() < deadline:
            try:
                line = events.get(timeout=3)
            except queue.Empty:
                raise TimeoutError("agent event timeout") from None
            if not line:
                break
            print(line, end="", flush=True)
            row = json.loads(line)
            if row["kind"] == "failsafe":
                stale_seen = stale_seen or row.get("reason") == expected_failure
                fallback_seen = fallback_seen or row.get("reason") == "land_on_exit"
            if row["kind"] != "telemetry":
                continue
            if target is None:
                target = (row["north"], row["east"], row["down"] - 2)
                agent.stdin.write('{"op":"start"}\n')
                agent.stdin.flush()
            observed_armed = observed_armed or row["armed"]
            if observed_armed and abs(row["down"] - target[2]) < 0.35 and not fault:
                agent.stdin.write('{"op":' if partial_test else '{"op":"inject-local-loss"}\n')
                agent.stdin.flush()
                fault = True
            if not fault:
                agent.stdin.write(json.dumps({"op":"target", "north":target[0],
                                               "east":target[1], "down":target[2]}) + "\n")
                agent.stdin.flush()
        if not fault:
            raise RuntimeError("fault injection did not occur")
        code = agent.wait(timeout=5)
        if code != 2 or not stale_seen or not fallback_seen:
            raise RuntimeError("agent did not complete the expected explicit landing fallback")
    finally:
        agent.stdin.close()
        if agent.poll() is None:
            agent.terminate()
            agent.wait(timeout=5)
        thread.join(timeout=2)
    monitor = mavutil.mavlink_connection("udpin:127.0.0.1:14540", source_system=245)
    try:
        heartbeat = monitor.wait_heartbeat(timeout=10)
        if heartbeat is None:
            raise RuntimeError("PX4 heartbeat missing after watchdog")
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            message = monitor.recv_match(type=["HEARTBEAT", "LOCAL_POSITION_NED", "STATUSTEXT"],
                                         blocking=True, timeout=1)
            if message is None:
                continue
            kind = message.get_type()
            print(json.dumps({"kind":"monitor", "type":kind, "data":message.to_dict()}), flush=True)
            if (kind == "HEARTBEAT" and message.get_srcSystem() == 1
                    and not message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                print(json.dumps({"kind":"completed", "watchdog_disarmed":True}), flush=True)
                return 0
        return 3
    finally:
        monitor.close()


if __name__ == "__main__":
    raise SystemExit(main())
