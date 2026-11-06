"""The generated school world must provide PX4 a physical heading reference."""

import math
import struct
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path
import runpy


def test_school_world_has_a_nonzero_magnetic_field_for_ekf_heading():
    world = ET.parse(Path(__file__).parents[1] / "worlds/school.sdf").find("world")
    assert world is not None
    field = tuple(float(x) for x in world.findtext("magnetic_field", "").split())
    assert len(field) == 3
    assert all(math.isfinite(x) for x in field)
    assert math.sqrt(sum(x * x for x in field)) > 0


def test_colored_school_visual_preserves_every_source_triangle():
    mesh_dir = Path(__file__).parents[1] / "models/environments/school-demo/meshes"
    original = (mesh_dir / "school_map.stl").read_bytes()
    expected = Counter(
        tuple(tuple(record[index:index + 3]) for index in (3, 6, 9))
        for record in struct.iter_unpack("<12fH", memoryview(original)[84:])
    )
    colored = ET.parse(mesh_dir / "school_map_colored.dae")
    ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
    array = colored.find(".//c:source[@id='positions']/c:float_array", ns)
    assert array is not None and array.text is not None
    values = [float(value) for value in array.text.split()]
    vertices = [tuple(values[index:index + 3]) for index in range(0, len(values), 3)]
    actual = Counter()
    groups = colored.findall(".//c:triangles", ns)
    for group in groups:
        indices = group.find("c:p", ns)
        assert indices is not None and indices.text is not None
        stride = 1 + max(int(item.attrib["offset"]) for item in group.findall("c:input", ns))
        positions = [int(value) for value in indices.text.split()][::stride]
        actual.update(tuple(vertices[index] for index in positions[offset:offset + 3])
                      for offset in range(0, len(positions), 3))
    assert len(groups) > 1
    assert actual == expected


def test_facade_windows_follow_continuous_coordinates_across_mesh_triangles():
    # Given two facade triangles spanning several intended window bays,
    root = Path(__file__).parents[1]
    material_for = runpy.run_path(str(root / "scripts/color-school-mesh.py"))["material_for"]
    wall = (((158, -20, 4), (148, -20, 20), (168, -20, 28)),
            ((148, -20, 20), (158, -20, 4), (148, -20, 4)))

    # When exporting their shared facade into the real render asset,
    assert {material_for(triangle, (0, 1, 0)) for triangle in wall} == {"facade"}
    mesh_dir = root / "models/environments/school-demo/meshes"
    colored = ET.parse(mesh_dir / "school_map_colored.dae")
    ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
    texture = colored.find(".//c:effect[@id='facade-effect']//c:diffuse/c:texture", ns)

    # Then color comes from continuous wall UVs rather than per-face glass colors.
    assert texture is not None
    assert (mesh_dir / "school_facade.png").read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    group = colored.find(".//c:triangles[@material='facade']", ns)
    assert group is not None
    inputs = group.findall("c:input", ns)
    uv_input = next(item for item in inputs if item.attrib["semantic"] == "TEXCOORD")
    stride = 1 + max(int(item.attrib["offset"]) for item in inputs)
    indices = [int(value) for value in group.findtext("c:p", "", ns).split()]
    uv_values = [float(value) for value in colored.findtext(
        ".//c:source[@id='texcoords']/c:float_array", "", ns).split()]
    position_values = [float(value) for value in colored.findtext(
        ".//c:source[@id='positions']/c:float_array", "", ns).split()]
    uv_offset = int(uv_input.attrib["offset"])
    for offset in range(0, len(indices), stride):
        position = indices[offset] * 3
        uv = indices[offset + uv_offset] * 3
        x, y, z = position_values[position:position + 3]
        s, t, _ = uv_values[uv:uv + 3]
        assert math.isclose(t, z / 8, abs_tol=1e-7)
        assert math.isclose(s, x / 12, abs_tol=1e-7) or math.isclose(s, y / 12, abs_tol=1e-7)


def test_cv_targets_use_hydrated_assets_instead_of_lfs_pointers():
    # Given the pinned Fuel identities, when inspecting local render resources,
    # then every resource must contain the exact original bytes.
    import hashlib
    import json

    root = Path(__file__).parents[1]
    assets = json.loads((root / "scripts/school-assets.json").read_text())
    for asset in assets:
        content = (root / "models/props" / asset["path"]).read_bytes()
        assert len(content) == asset["size"], asset["path"]
        assert hashlib.sha256(content).hexdigest() == asset["sha256"], asset["path"]


def test_cv_scene_keeps_ground_truth_proxies_out_of_camera_images():
    # Given the generated scene, when inspecting its rendered geometry,
    # then the legacy risk volumes cannot be mistaken for detected objects.
    model = ET.parse(Path(__file__).parents[1] / "models/environments/school-demo/model.sdf")
    proxies = [link for link in model.findall("model/link") if link.attrib["name"].startswith("risk_")]
    assert proxies
    assert all(link.find("visual") is None and link.find("collision") is not None for link in proxies)
    car = model.find("model/model[@name='cv_parked_car']")
    assert car is not None and car.findtext("static") == "true"
    assert car.find("joint") is None
    assert car.findall("link/visual/geometry/mesh")
