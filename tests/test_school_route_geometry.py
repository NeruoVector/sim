"""Independent continuous segment/rectangle proof against every raw STL triangle."""

import json
import math
import struct
from pathlib import Path

import pytest
from school_mesh_clearance import MeshPlacement, column_height
from school_mission import MissionPlan

FIXTURE = Path(__file__).parents[1] / "fixtures/school.json"
MESH = (
    Path(__file__).parents[1] / "models/environments/school-demo/meshes/school_map.stl"
)


def point_segment_distance(point, start, end):
    delta = (end[0] - start[0], end[1] - start[1])
    denominator = delta[0] ** 2 + delta[1] ** 2
    factor = (
        min(
            1,
            max(0, sum((point[i] - start[i]) * delta[i] for i in (0, 1)) / denominator),
        )
        if denominator
        else 0
    )
    return math.hypot(*(point[i] - start[i] - factor * delta[i] for i in (0, 1)))


def rectangle_distance(start, end, rectangle):
    """Exact 2D segment distance using slab intersection and edge endpoints."""
    lower, upper = rectangle
    enter, leave = 0.0, 1.0
    intersects = True
    for axis in (0, 1):
        delta = end[axis] - start[axis]
        if delta == 0:
            if not lower[axis] <= start[axis] <= upper[axis]:
                intersects = False
                break
        else:
            first, last = sorted(
                (
                    (lower[axis] - start[axis]) / delta,
                    (upper[axis] - start[axis]) / delta,
                )
            )
            enter, leave = max(enter, first), min(leave, last)
            if enter > leave:
                intersects = False
                break
    if intersects:
        return 0.0
    corners = [(x, y) for x in (lower[0], upper[0]) for y in (lower[1], upper[1])]
    return min(
        *(point_segment_distance(corner, start, end) for corner in corners),
        *(
            math.hypot(
                *(max(lower[i] - point[i], point[i] - upper[i], 0) for i in (0, 1))
            )
            for point in (start, end)
        ),
    )


@pytest.fixture(scope="module")
def raw_geometry():
    data = json.loads(FIXTURE.read_text())
    placement = data["campus"]
    raw = MESH.read_bytes()
    rectangles = []
    for values in struct.iter_unpack("<12fH", memoryview(raw)[84:]):
        vertices = [
            tuple(
                values[3 + n * 3 + i] * placement["mesh_scale"][i]
                + placement["mesh_pose_enu"][i]
                for i in range(3)
            )
            for n in range(3)
        ]
        rectangles.append(
            (
                (
                    tuple(min(v[i] for v in vertices) for i in (0, 1)),
                    tuple(max(v[i] for v in vertices) for i in (0, 1)),
                ),
                max(v[2] for v in vertices),
            )
        )
    assert len(rectangles) == 159573
    return data, rectangles


def clearance_metrics(plan, geometry):
    data, triangles = geometry
    envelope = data["drone"]["radius_m"] + 0.6 + 0.25
    segments = list(
        zip(plan.route[: plan.delivery_index - 1], plan.route[1 : plan.delivery_index])
    )
    high = [
        rectangle for rectangle, top in triangles if top >= plan.route[0][2] - envelope
    ]
    mesh_clearance = min(
        rectangle_distance(a, b, rectangle) for a, b in segments for rectangle in high
    )
    building_clearance = math.inf
    for building in data["buildings"]:
        if plan.route[0][2] - envelope > building["no_fly_ceiling_m"]:
            continue
        center, size = building["center_enu"], building["size_m"]
        rectangle = (
            tuple(center[i] - size[i] / 2 for i in (0, 1)),
            tuple(center[i] + size[i] / 2 for i in (0, 1)),
        )
        building_clearance = min(
            building_clearance,
            *(rectangle_distance(a, b, rectangle) for a, b in segments),
        )
    return mesh_clearance, building_clearance


@pytest.mark.parametrize("destination", ["lobby", "gym", "back", "playground"])
def test_entire_route_clears_raw_mesh_and_building_with_body_and_corner_tolerance(
    raw_geometry, destination
):
    plan = MissionPlan.read(FIXTURE, destination)
    mesh, building = clearance_metrics(plan, raw_geometry)
    required = raw_geometry[0]["drone"]["radius_m"] + 0.6 + 0.25
    assert mesh >= required
    assert building >= required
    # Return retraces the same proven horizontal segments in reverse.
    assert plan.route[plan.return_takeoff_index : -1] == tuple(
        reversed(plan.route[: plan.delivery_index])
    )


def test_original_low_straight_routes_would_intersect_tall_mesh(raw_geometry):
    data, triangles = raw_geometry
    envelope = data["drone"]["radius_m"] + 0.6 + 0.25
    for destination in data["delivery_missions"]["destinations"].values():
        assert any(
            top >= destination[2] - envelope
            and rectangle_distance((0, 0), destination, rectangle) < envelope
            for rectangle, top in triangles
        )


@pytest.mark.parametrize("pad", ["home", "lobby", "gym", "back", "playground"])
def test_exact_projected_pad_column_is_clear_above_flat_support(raw_geometry, pad):
    data, _ = raw_geometry
    campus = data["campus"]
    placement = MeshPlacement(
        MESH, tuple(campus["mesh_scale"]), tuple(campus["mesh_pose_enu"])
    )
    landing = (
        data["takeoff_enu"]
        if pad == "home"
        else data["delivery_missions"]["landing_points"][pad]
    )
    assert (
        column_height(placement, landing[:2], data["drone"]["radius_m"] + 0.25)
        <= landing[2] + 0.05
    )


def test_gym_relocation_avoids_old_pad_slope(raw_geometry):
    data, _ = raw_geometry
    campus = data["campus"]
    placement = MeshPlacement(
        MESH, tuple(campus["mesh_scale"]), tuple(campus["mesh_pose_enu"])
    )
    assert column_height(placement, (50, 20), 0.5) > 0.59
    assert column_height(placement, (48, 20), 1.45) == pytest.approx(0.443476)
