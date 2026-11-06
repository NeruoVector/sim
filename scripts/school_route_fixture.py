"""Offline route compilation; live control only checks the resulting signature."""

from __future__ import annotations

import json
from pathlib import Path

from school_mesh_clearance import (
    MeshPlacement,
    UnsafeRoute,
    column_height,
    read_height_map,
)
from school_route import SAFETY_MARGIN, Building, RouteSpace, plan_route
from school_route_manifest import route_signature


def compile_routes(fixture: Path) -> str:
    """Return a fixture with verified route points; never modify the source mesh."""
    data = json.loads(fixture.read_text())
    config = data["delivery_missions"]
    config.pop("route_signature", None)
    mesh = (
        Path(__file__).resolve().parents[1]
        / "models/environments/school-demo/meshes/school_map.stl"
    )
    campus = data["campus"]
    placement = MeshPlacement(
        mesh, tuple(campus["mesh_scale"]), tuple(campus["mesh_pose_enu"])
    )
    terrain = read_height_map(placement)
    center, size = campus["ground_center_enu"], campus["size_m"]
    home = tuple(data["takeoff_enu"])
    radius = data["drone"]["radius_m"]
    bounds = (
        (center[0] - size[0] / 2, center[1] - size[1] / 2),
        (center[0] + size[0] / 2, center[1] + size[1] / 2),
    )
    buildings = tuple(
        Building(
            tuple(b["center_enu"][:2]), tuple(b["size_m"][:2]), b["no_fly_ceiling_m"]
        )
        for b in data["buildings"]
    )
    for landing in (home, *(tuple(p) for p in config["landing_points"].values())):
        if (
            column_height(placement, landing[:2], radius + SAFETY_MARGIN)
            > landing[2] + 0.05
        ):
            raise UnsafeRoute(
                "Landing column intersects mesh above the support surface"
            )
        if any(
            all(
                abs(landing[axis] - b.center[axis])
                <= b.size[axis] / 2 + radius + SAFETY_MARGIN
                for axis in (0, 1)
            )
            for b in buildings
        ):
            raise UnsafeRoute("Landing column intersects a building")
    routes = {}
    for destination_id, destination in config["destinations"].items():
        space = RouteSpace(terrain, bounds, buildings, destination[2], radius)
        horizontal = plan_route(space, home[:2], tuple(destination[:2]))
        routes[destination_id] = [(x, y, destination[2]) for x, y in horizontal]
    config["routes_enu"] = routes
    manifest = json.dumps(
        [campus, data["buildings"], home, radius, config],
        sort_keys=True,
        separators=(",", ":"),
    )
    config["route_signature"] = route_signature(mesh, manifest)
    return json.dumps(data, ensure_ascii=False, indent=2) + "\n"
