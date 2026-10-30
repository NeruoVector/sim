"""An obstacle-triggered hold survives later native decisions until replacement."""
from __future__ import annotations

import argparse
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sitl"))
from coordinates import Enu
from replan_coordinator import ReplanCoordinator


def test_native_replan_stays_held_without_activated_replacement(tmp_path: Path) -> None:
    args = argparse.Namespace(exercise_physical_obstacle=False, exercise_replan_hold=False,
                              auto_global_replan=False)
    events = []
    coordinator = ReplanCoordinator(args, None, tmp_path, lambda kind, **data: events.append((kind, data)))
    args.exercise_physical_obstacle = True
    coordinator.obstacle_present = True
    coordinator.marker = tmp_path / "missing"
    decision = {"action":"request_replan", "detour":[]}
    pos, vel = Enu(0, 17, 8), Enu(0, 0, 0)
    with ThreadPoolExecutor(max_workers=1) as executor:
        action, replacement = coordinator.decide(100, pos, vel, 1, decision, [], (), (), {}, executor, lambda: None)
        assert action == "hold" and replacement is None
        action, replacement = coordinator.decide(104, pos, vel, 1, {"action":"proceed", "detour":[]},
                                                  [], (), (), {}, executor, lambda: None)
        assert action == "hold" and replacement is None
    assert [data["phase"] for kind, data in events if kind == "synthetic_replan_hold"] == ["begin", "stable"]
