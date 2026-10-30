"""Encode the complete static procedural fixture for the native bridge."""
from __future__ import annotations

import json
import math
import os
import select
import subprocess
import time
from pathlib import Path

from coordinates import Enu


class FixtureAvoidance:
    """Synthetic scene geometry is ground truth, not camera perception."""

    def __init__(self, fixture_path: Path, bridge_path: Path) -> None:
        self.fixture = json.loads(fixture_path.read_text())
        if self.fixture.get("synthetic") is not True:
            raise ValueError("only the explicit synthetic fixture is permitted")
        self.bridge = subprocess.Popen([str(bridge_path)], stdin=subprocess.PIPE,
                                       stdout=subprocess.PIPE, bufsize=0)

    def decide(self, now: float, position: Enu, velocity: Enu,
               remaining: list[Enu], *, observation_time: float | None = None) -> dict:
        observed = now if observation_time is None else observation_time
        tokens = ["V1", now, 1, observed, position.east_m, position.north_m, position.up_m,
                  velocity.east_m, velocity.north_m, velocity.up_m, len(remaining)]
        for point in remaining:
            tokens.extend((point.east_m, point.north_m, point.up_m))
        obstacles = self.fixture["obstacles"]
        tokens.append(len(obstacles))
        for obstacle in obstacles:
            tokens.extend((obstacle["priority"], 1, observed, 1, *obstacle["position_enu"],
                           1, 0, 0, 0, 0, 0, obstacle["radius_m"]))
        buildings = self.fixture["buildings"]
        tokens.append(len(buildings))
        for building in buildings:
            center = building["center_enu"]
            size = building["size_m"]
            tokens.extend(center[i] - size[i] / 2 for i in range(3))
            tokens.extend((center[0] + size[0] / 2, center[1] + size[1] / 2,
                           building["no_fly_ceiling_m"]))
        tokens.extend((1, -100, -100, -2, 150, 150, 120, observed, 0, 0))
        if self.bridge.stdin is None or self.bridge.stdout is None:
            raise RuntimeError("native avoidance bridge unavailable")
        payload = (" ".join(map(str, tokens)) + "\n").encode()
        deadline = time.monotonic() + 0.3
        offset = 0
        os.set_blocking(self.bridge.stdin.fileno(), False)
        while offset < len(payload):
            remaining_time = deadline - time.monotonic()
            if remaining_time <= 0:
                raise TimeoutError("native avoidance request timed out")
            _, writable, _ = select.select([], [self.bridge.stdin], [], remaining_time)
            if not writable:
                raise TimeoutError("native avoidance request timed out")
            try:
                offset += os.write(self.bridge.stdin.fileno(), payload[offset:])
            except BlockingIOError:
                continue
        data = b""
        while b"\n" not in data:
            remaining_time = deadline - time.monotonic()
            if remaining_time <= 0:
                raise TimeoutError("native avoidance decision timed out")
            ready, _, _ = select.select([self.bridge.stdout], [], [], remaining_time)
            if not ready:
                raise TimeoutError("native avoidance decision timed out")
            chunk = os.read(self.bridge.stdout.fileno(), 4096)
            if not chunk:
                raise RuntimeError("native avoidance bridge closed")
            data += chunk
            if len(data) > 65536:
                raise ValueError("native avoidance response too large")
        line, extra = data.split(b"\n", 1)
        if extra:
            raise ValueError("unexpected additional native bridge output")
        response = json.loads(line)
        if (not isinstance(response, dict)
                or response.get("action") not in ("proceed", "slow", "hold", "request_replan")
                or not isinstance(response.get("reason"), str)
                or isinstance(response.get("desired_speed_mps"), bool)
                or not isinstance(response.get("desired_speed_mps"), (int, float))
                or not math.isfinite(response["desired_speed_mps"])
                or not 0 <= response["desired_speed_mps"] <= 30
                or isinstance(response.get("braking_distance_m"), bool)
                or not isinstance(response.get("braking_distance_m"), (int, float))
                or not math.isfinite(response["braking_distance_m"])
                or response["braking_distance_m"] < 0
                or not isinstance(response.get("threat_order"), list)
                or any(type(index) is not int or index < 0 for index in response["threat_order"])
                or not isinstance(response.get("detour"), list)
                or len(response["detour"]) > 256):
            raise ValueError("native avoidance returned invalid decision")
        for point in response["detour"]:
            if (not isinstance(point, list) or len(point) != 3
                    or any(isinstance(v, bool) or not isinstance(v, (int, float))
                           or not math.isfinite(v) for v in point)):
                raise ValueError("native avoidance returned invalid detour")
        return response

    def close(self) -> None:
        if self.bridge.stdin is not None:
            self.bridge.stdin.close()
        try:
            self.bridge.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.bridge.kill()
            self.bridge.wait(timeout=2)
