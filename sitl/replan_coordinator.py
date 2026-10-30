"""Coordinate a measured synthetic hold and independently verified replacement."""
from __future__ import annotations

import argparse
import json
import math
from collections.abc import Callable
from concurrent.futures import Executor, Future
from pathlib import Path

from auto_replan import Snapshot, activate_global_replan
from coordinates import Enu
from fixture_avoidance import FixtureAvoidance
from gazebo_obstacle import spawn_obstacle
from mission_backend import AdminStops, fetch_plan
from replacement import verify_replacement
from stops import bind_stops


class ReplanCoordinator:
    def __init__(self, args: argparse.Namespace, admin: AdminStops, root: Path, log: Callable) -> None:
        self.args, self.admin, self.log = args, admin, log
        self.point: Enu | None = None
        self.stable: float | None = None
        self.announced = self.complete = self.obstacle_present = False
        self.obstacle_future: Future | None = None
        self.auto_future: Future | None = None
        self.replan_future: Future | None = None
        self.new_contract: dict[str, str] | None = None
        self.plan = root / (".omo/evidence/drone-autonomy/backend/sitl-obstacle-activated-plan.json"
                            if args.exercise_physical_obstacle else
                            ".omo/evidence/drone-autonomy/backend/sitl-activated-plan.json")
        self.marker = Path("/tmp/qa-autonomy-obstacle-activated-contract.env" if args.exercise_physical_obstacle
                           else "/tmp/qa-autonomy-activated-contract.env")
        self.evidence = root / ".omo/evidence/drone-autonomy/sim"
        if not args.auto_global_replan and (args.exercise_replan_hold or args.exercise_physical_obstacle) and (
                self.marker.exists() or self.plan.exists()):
            raise ValueError("remove stale task-owned replacement marker before starting")
        if args.auto_global_replan and (self.evidence / "auto-obstacle-activated-plan.json").exists():
            raise ValueError("automatic replacement evidence already exists")

    def observe(self, index: int, pos: Enu, executor: Executor, policy: FixtureAvoidance) -> None:
        if self.args.exercise_physical_obstacle and index == 1 and pos.north_m >= 5 and self.obstacle_future is None:
            self.obstacle_future = executor.submit(spawn_obstacle, self.args.container)
            self.log("physical_obstacle", phase="spawn_requested")
        if self.obstacle_future is not None and self.obstacle_future.done() and not self.obstacle_present:
            obstacle = self.obstacle_future.result()
            policy.fixture["obstacles"].append(obstacle)
            self.obstacle_present = True
            self.log("physical_obstacle", phase="scene_confirmed", observation=json.dumps(obstacle))

    def _load(self, contract: dict[str, str], path: Path, route: list[Enu], indices: tuple[int, ...],
              stops: tuple, point: Enu) -> tuple:
        new_route, expiry, new_indices = fetch_plan(path, self.args.checker, contract)
        aligned = verify_replacement(route, indices, new_route, new_indices, point)
        new_stops = bind_stops(self.admin.read(), aligned)
        if [s.order_id for s in new_stops] != [s.order_id for s in stops]:
            raise ValueError("replacement changed backend customer ordering")
        self.admin.verify_locations(new_stops, new_route)
        return new_route, expiry, new_indices, new_stops

    def decide(self, now: float, pos: Enu, vel: Enu, index: int, decision: dict,
               route: list[Enu], indices: tuple[int, ...], stops: tuple, contract: dict[str, str],
               executor: Executor, latest: Callable[[], Snapshot | None]) -> tuple[str, tuple | None]:
        action = decision["action"]
        if self.obstacle_future is not None and not self.obstacle_present and pos.north_m >= 12:
            action = "hold"
        if (not self.complete and index == 1 and self.point is None and
            ((self.args.exercise_replan_hold and pos.north_m >= 15.0) or
             (self.args.exercise_physical_obstacle and self.obstacle_present and action == "request_replan"))):
            self.point = pos
            self.log("synthetic_replan_hold", phase="begin", north=pos.north_m, east=pos.east_m,
                     up=pos.up_m, native_action=decision["action"], native_detour=json.dumps(decision["detour"]))
        if self.point is None or self.complete:
            return action, None
        speed = math.sqrt(vel.east_m**2 + vel.north_m**2 + vel.up_m**2)
        if speed < 0.2 and math.dist((pos.east_m, pos.north_m, pos.up_m),
                                     (self.point.east_m, self.point.north_m, self.point.up_m)) < 0.6:
            self.stable = self.stable or now
        else:
            self.stable = None
        if self.stable is not None and now - self.stable >= 3 and not self.announced:
            self.announced = True
            self.log("synthetic_replan_hold", phase="stable", north=pos.north_m, east=pos.east_m, up=pos.up_m)
        if self.announced and self.stable is not None and now - self.stable > 120:
            raise TimeoutError("held replacement was not activated in time")
        if self.announced and self.args.auto_global_replan and self.auto_future is None:
            self.auto_future = executor.submit(activate_global_replan, self.admin, contract, self.args.plan,
                self.args.checker, latest, self.evidence)
            self.log("automatic_global_replan", phase="begin")
        if self.auto_future is not None and self.auto_future.done() and self.replan_future is None:
            self.new_contract, path, zone_id = self.auto_future.result()
            self.log("automatic_global_replan", phase="activated", zone_id=zone_id)
            self.replan_future = executor.submit(self._load, self.new_contract, path, route, indices, stops, self.point)
        if (self.announced and not self.args.auto_global_replan and self.marker.exists()
                and self.replan_future is None):
            self.new_contract = dict(line.split("=", 1) for line in self.marker.read_text().splitlines()
                                     if line and not line.startswith("#") and "=" in line)
            if self.new_contract.get("QA_DELIVERY") != contract.get("QA_DELIVERY"):
                raise ValueError("replacement delivery identity changed")
            self.replan_future = executor.submit(self._load, self.new_contract, self.plan, route, indices, stops, self.point)
        if self.replan_future is not None and self.replan_future.done():
            new_route, expiry, new_indices, new_stops = self.replan_future.result()
            verify_replacement(route, indices, new_route, new_indices, pos)
            if self.new_contract is None:
                raise RuntimeError("replacement contract missing")
            self.complete = True
            self.log("replacement_accepted", route=self.new_contract["QA_ROUTE"], waypoint=1)
            return "hold", (new_route, expiry, new_indices, new_stops)
        return "hold", None
