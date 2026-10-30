"""Post one actual disarmed home sample from the isolated procedural PX4."""
from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path

from altitude_reference import require_running_world
from coordinates import Geographic, Origin, require_synthetic_start
from mission_backend import post_telemetry


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--container", choices=("nv-autonomy-auto-7bd1", "nv-colocated-sitl-ad1"), required=True)
    parser.add_argument("--synthetic-only", action="store_true", required=True)
    args = parser.parse_args()
    isolated = subprocess.run(["docker", "inspect", "--format",
        "{{.HostConfig.NetworkMode}} {{.HostConfig.IpcMode}} {{len .HostConfig.Devices}}", args.container],
        capture_output=True, text=True, check=True, timeout=5).stdout.strip()
    if isolated != "none private 0":
        raise ValueError("unsafe synthetic container isolation")
    root = Path(__file__).resolve().parents[2]
    require_running_world(args.container, root / ".omo/evidence/drone-autonomy/sim/school_procedural.sdf")
    process = subprocess.Popen(["docker", "exec", "-i", "-e", "NV_SITL_ONLY=network-none", args.container,
        "python3", "/nv-sim/sitl/offboard_agent.py", "--synthetic-mission"],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
    if process.stdin is None or process.stdout is None:
        raise RuntimeError("synthetic transport unavailable")
    try:
        for line in process.stdout:
            row = json.loads(line)
            if (row.get("kind") != "telemetry" or row["battery"] is None
                    or row["local_age"] > 0.8 or row["gps_age"] > 0.8):
                continue
            if row["armed"]:
                raise ValueError("home calibration requires disarmed PX4")
            require_synthetic_start(Origin(37.0, 127.0, 0.0).to_enu(
                Geographic(row["latitude"], row["longitude"], row["relative_altitude"])))
            status, telemetry_id = post_telemetry(row, home_verified=True)
            print(json.dumps({"sample": row, "status": status, "telemetry_id": telemetry_id}))
            return 0
        raise RuntimeError("synthetic telemetry stream closed before home calibration")
    finally:
        process.stdin.close()
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.wait(timeout=5)


if __name__ == "__main__":
    raise SystemExit(main())
