"""Pure route planning covers obstacles, continuous segments and failure closure."""

import json
import math
import struct
from itertools import pairwise
from pathlib import Path

import pytest
from school_mesh_clearance import (
    HeightMap,
    MeshPlacement,
    UnsafeRoute,
    projected_distance,
    read_height_map,
)
from school_route import Building, RouteSpace, plan_route
from school_route_fixture import compile_routes
from school_route_manifest import route_signature

FIXTURE = Path(__file__).parents[1] / "fixtures/school.json"


def empty_space(buildings=()):
    return RouteSpace(HeightMap({}), ((-10, -10), (10, 10)), buildings, 5, 0.5)


def test_clear_space_uses_direct_route():
    assert plan_route(empty_space(), (-5, 0), (5, 0)) == ((-5, 0), (5, 0))


def test_low_route_detours_around_building_instead_of_lowering_through_it():
    space = empty_space((Building((0, 0), (2, 6), 15),))
    route = plan_route(space, (-5, 0), (5, 0))
    assert len(route) > 2
    assert any(abs(y) > 4.35 for _, y in route)
    assert all(space.segment_clear(a, b) for a, b in pairwise(route))
    assert route == plan_route(space, (-5, 0), (5, 0))


def test_tall_mesh_cells_are_obstacles_even_without_building_metadata():
    terrain = HeightMap({(0, y): 8 for y in range(-15, 16)})
    space = RouteSpace(terrain, ((-10, -10), (10, 10)), (), 5, 0.5)
    route = plan_route(space, (-5, 0), (5, 0))
    assert len(route) > 2
    assert not space.segment_clear((-5, 0), (5, 0))
    assert all(space.segment_clear(a, b) for a, b in pairwise(route))


def test_blocked_campus_cannot_fall_back_to_unsafe_direct_route():
    space = empty_space((Building((0, 0), (2, 30), 15),))
    with pytest.raises(UnsafeRoute, match="No obstacle-clear"):
        plan_route(space, (-5, 0), (5, 0))


def test_blocked_endpoint_is_rejected():
    space = empty_space((Building((0, 0), (2, 2), 15),))
    with pytest.raises(UnsafeRoute, match="endpoint"):
        plan_route(space, (0, 0), (5, 0))


def test_segment_interior_is_checked_even_when_both_endpoints_are_clear():
    space = empty_space((Building((0, 0), (0.1, 0.1), 15),))
    assert space.clear((-5, 0)) and space.clear((5, 0))
    assert not space.segment_clear((-5, 0), (5, 0))


def test_projected_vertical_face_does_not_claim_far_away_point_is_inside():
    triangle = ((43, 43, 4), (42.5, 42.5, 4), (43, 43, 5))
    assert projected_distance(triangle, (0, 0)) == pytest.approx(math.hypot(42.5, 42.5))
    assert projected_distance(((0, 0, 1), (2, 0, 1), (0, 2, 1)), (0.5, 0.5)) == 0


def test_binary_stl_scale_translation_and_triangle_interior_are_rasterized(tmp_path):
    mesh = tmp_path / "triangle.stl"
    mesh.write_bytes(
        bytes(80)
        + struct.pack("<I", 1)
        + struct.pack("<12fH", 0, 0, 1, 0, 0, 4, 2, 0, 4, 0, 2, 4, 0)
    )
    terrain = read_height_map(MeshPlacement(mesh, (2, 2, 2), (10, 20, 1)))
    assert terrain.height_near((11, 21), 0) == 9
    assert terrain.height_near((0, 0), 0) == -math.inf


def test_truncated_stl_cannot_authorize_a_route(tmp_path):
    mesh = tmp_path / "broken.stl"
    mesh.write_bytes(bytes(80) + struct.pack("<I", 1))
    with pytest.raises(UnsafeRoute, match="complete binary STL"):
        read_height_map(MeshPlacement(mesh, (1, 1, 1), (0, 0, 0)))


def test_route_signature_detects_changed_mesh_bytes(tmp_path):
    mesh = tmp_path / "mesh.stl"
    mesh.write_bytes(b"original")
    original = route_signature(mesh, "same fixture")
    mesh.write_bytes(b"modified")
    assert route_signature(mesh, "same fixture") != original


def test_checked_in_routes_reproduce_from_the_unchanged_mesh():
    original = json.loads(FIXTURE.read_text())
    rebuilt = json.loads(compile_routes(FIXTURE))
    assert rebuilt["delivery_missions"] == original["delivery_missions"]
