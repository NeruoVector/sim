"""Run the actual native bridge against synthetic fresh and stale observations."""
from __future__ import annotations

import os
import sys
from pathlib import Path

import pytest

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM / "sitl"))
from coordinates import Enu
from fixture_avoidance import FixtureAvoidance


def test_native_bridge_uses_fixture_route() -> None:
    bridge_path = Path(os.environ.get("NV_AVOIDANCE_BRIDGE", "/tmp/orca-drone-release/nv_avoidance_bridge"))
    if not bridge_path.is_file():
        pytest.skip("native bridge executable unavailable")
    bridge = FixtureAvoidance(SIM / "fixtures/school.json", bridge_path)
    try:
        decision = bridge.decide(100.0, Enu(0, 0, 0), Enu(0, 0, 0),
                                 [Enu(0, 0, 8), Enu(0, 5, 8)])
        assert decision["action"] == "proceed"
        stale = bridge.decide(102.0, Enu(0, 0, 0), Enu(0, 0, 0),
                              [Enu(0, 0, 8)], observation_time=100.0)
        assert stale["action"] == "hold"
    finally:
        bridge.close()


@pytest.mark.parametrize("reply", ["partial", "invalid", "exit"])
def test_broken_bridge_fails_with_bounded_wait(tmp_path: Path, reply: str) -> None:
    bridge_path = tmp_path / "bridge"
    bridge_path.write_text(
        "#!/usr/bin/env python3\nimport sys,time\n"
        "sys.stdin.readline()\n"
        f"mode={reply!r}\n"
        "if mode=='partial':\n sys.stdout.write('{'); sys.stdout.flush(); time.sleep(1)\n"
        "elif mode=='invalid':\n print('not-json', flush=True)\n"
    )
    bridge_path.chmod(0o755)
    bridge = FixtureAvoidance(SIM / "fixtures/school.json", bridge_path)
    try:
        with pytest.raises((ValueError, TimeoutError, RuntimeError)):
            bridge.decide(100.0, Enu(0, 0, 0), Enu(0, 0, 0), [Enu(0, 0, 8)])
    finally:
        bridge.close()


def test_bridge_rejects_nonfinite_or_unbounded_decision(tmp_path: Path) -> None:
    bridge_path = tmp_path / "bridge"
    response = '{"action":"proceed","reason":"x","desired_speed_mps":1e999,"braking_distance_m":1,"threat_order":[],"detour":[]}'
    bridge_path.write_text("#!/usr/bin/env python3\nimport sys\nsys.stdin.readline()\n"
                           + f"print({response!r}, flush=True)\n")
    bridge_path.chmod(0o755)
    bridge = FixtureAvoidance(SIM / "fixtures/school.json", bridge_path)
    try:
        with pytest.raises(ValueError):
            bridge.decide(100.0, Enu(0, 0, 0), Enu(0, 0, 0), [Enu(0, 0, 8)])
    finally:
        bridge.close()


def test_bridge_request_timeout_when_pipe_is_full(tmp_path: Path) -> None:
    import time

    bridge_path = tmp_path / "bridge"
    bridge_path.write_text("#!/usr/bin/env python3\nimport time\ntime.sleep(2)\n")
    bridge_path.chmod(0o755)
    bridge = FixtureAvoidance(SIM / "fixtures/school.json", bridge_path)
    assert bridge.bridge.stdin is not None
    fd = bridge.bridge.stdin.fileno()
    os.set_blocking(fd, False)
    try:
        while True:
            try:
                os.write(fd, b"x" * 4096)
            except BlockingIOError:
                break
        started = time.monotonic()
        with pytest.raises(TimeoutError):
            bridge.decide(100.0, Enu(0, 0, 0), Enu(0, 0, 0), [Enu(0, 0, 8)])
        assert time.monotonic() - started < 1.5
    finally:
        bridge.close()
