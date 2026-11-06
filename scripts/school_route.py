"""Deterministic low-cruise A* routes with conservative segment clearance."""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from functools import cache
from typing import Final

from school_mesh_clearance import XY, HeightMap, UnsafeRoute

STEP: Final = 0.5
SAMPLE_STEP: Final = 0.25
TARGET_TOLERANCE: Final = 0.6
SAFETY_MARGIN: Final = 0.25


@dataclass(frozen=True, slots=True)
class Building:
    center: XY
    size: XY
    ceiling: float


@dataclass(frozen=True, slots=True)
class RouteSpace:
    terrain: HeightMap
    bounds: tuple[XY, XY]
    buildings: tuple[Building, ...]
    altitude: float
    radius: float

    @property
    def clearance(self) -> float:
        return self.radius + TARGET_TOLERANCE + SAFETY_MARGIN

    def clear(self, position: XY, padding: float = 0) -> bool:
        """Protect the airframe, acceptance tolerance and margin in every direction."""
        radius = self.clearance + padding
        if any(
            not self.bounds[0][axis] + radius
            <= position[axis]
            <= self.bounds[1][axis] - radius
            for axis in (0, 1)
        ):
            return False
        if self.terrain.height_near(position, radius) >= self.altitude - self.clearance:
            return False
        return all(
            self.altitude - self.clearance > b.ceiling
            or any(
                abs(position[axis] - b.center[axis]) > b.size[axis] / 2 + radius
                for axis in (0, 1)
            )
            for b in self.buildings
        )

    def segment_clear(self, start: XY, end: XY) -> bool:
        """Half a sampling step of extra radius covers the continuous segment."""
        count = max(1, math.ceil(math.dist(start, end) / SAMPLE_STEP))
        return all(
            self.clear(
                tuple(a + (b - a) * n / count for a, b in zip(start, end, strict=True)),
                SAMPLE_STEP / 2,
            )
            for n in range(count + 1)
        )


def plan_route(space: RouteSpace, start: XY, end: XY) -> tuple[XY, ...]:
    """Find a safe horizontal route, then remove only visibility-proven waypoints."""
    if not space.segment_clear(start, start) or not space.segment_clear(end, end):
        raise UnsafeRoute("Cruise endpoint has insufficient obstacle clearance")
    if space.segment_clear(start, end):
        return start, end
    origin = (round(start[0] / STEP), round(start[1] / STEP))
    goal = (round(end[0] / STEP), round(end[1] / STEP))

    def position(cell: tuple[int, int]) -> XY:
        return cell[0] * STEP, cell[1] * STEP

    if not space.segment_clear(start, position(origin)) or not space.segment_clear(
        position(goal), end
    ):
        raise UnsafeRoute("Endpoint cannot safely connect to the planning grid")

    @cache
    def clear(cell: tuple[int, int]) -> bool:
        # This padding also covers each 8-connected edge between clear nodes.
        return space.clear(position(cell), STEP / math.sqrt(2))

    queue = [(math.dist(origin, goal), 0.0, origin)]
    costs = {origin: 0.0}
    parents: dict[tuple[int, int], tuple[int, int]] = {}
    while queue:
        _, cost, cell = heapq.heappop(queue)
        if cost > costs[cell]:
            continue
        if cell == goal:
            break
        for dx, dy in (
            (1, 0),
            (0, 1),
            (-1, 0),
            (0, -1),
            (1, 1),
            (-1, 1),
            (-1, -1),
            (1, -1),
        ):
            neighbor = cell[0] + dx, cell[1] + dy
            distance = cost + math.hypot(dx, dy)
            safe = (
                space.segment_clear(position(cell), end)
                if neighbor == goal
                else clear(neighbor)
            )
            if distance >= costs.get(neighbor, math.inf) or not safe:
                continue
            if cell == origin and not space.segment_clear(start, position(neighbor)):
                continue
            costs[neighbor] = distance
            parents[neighbor] = cell
            heapq.heappush(
                queue, (distance + math.dist(neighbor, goal), distance, neighbor)
            )
    else:
        raise UnsafeRoute("No obstacle-clear campus route exists")
    cells = [goal]
    while cells[-1] != origin:
        cells.append(parents[cells[-1]])
    path = [start, *(position(cell) for cell in reversed(cells)), end]
    simplified = [start]
    index = 0
    while index < len(path) - 1:
        furthest = next(
            (
                n
                for n in range(len(path) - 1, index, -1)
                if space.segment_clear(path[index], path[n])
            ),
            None,
        )
        if furthest is None:
            raise UnsafeRoute("Planning grid produced an unsafe segment")
        simplified.append(path[furthest])
        index = furthest
    if 2 * len(simplified) + 2 > 32:
        raise UnsafeRoute("Safe mission exceeds the 32-point route limit")
    return tuple(simplified)
