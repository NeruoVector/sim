"""The generated school world must provide PX4 a physical heading reference."""

import math
import struct
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path


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
        positions = [int(value) for value in indices.text.split()][::2]
        actual.update(tuple(vertices[index] for index in positions[offset:offset + 3])
                      for offset in range(0, len(positions), 3))
    assert len(groups) > 1
    assert actual == expected


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
