"""Imported drone appearance must remain above the physical landing skids."""

import hashlib
import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path


def test_imported_drone_mesh_clears_ground_when_landing_skids_touch():
    # Given the actual imported vertices and the stock physical contact boxes,
    root = Path(__file__).parents[1] / "models/drones/x500_base"
    model = ET.parse(root / "model.sdf")
    visual = model.find("model/link/visual[@name='neurovector_drone_visual']")
    assert visual is not None
    pose = [float(value) for value in visual.findtext("pose", "").split()]
    scale = [float(value) for value in visual.findtext("geometry/mesh/scale", "").split()]
    assert pose[3:] == [0, 0, 0]
    source = (root / "meshes/drone.stl").read_bytes()
    assert hashlib.sha256(source).hexdigest() == (
        "390d7ee9e4abf178b0ed0b7bbc2f7616e122afb2686cf78101567df05f0cd153"
    )
    assert struct.unpack_from("<I", source, 80)[0] == 318391
    vertices = (record[index:index + 3]
                for record in struct.iter_unpack("<12fH", memoryview(source)[84:])
                for index in (3, 6, 9))
    visual_bottom = min(vertex[2] * scale[2] + pose[2] for vertex in vertices)

    # When the lowest rotated collision box rests on the ground plane,
    collision_bottoms: list[float] = []
    for collision in model.findall("model/link[@name='base_link']/collision"):
        position = [float(value) for value in collision.findtext("pose", "").split()]
        size = [float(value) for value in collision.findtext("geometry/box/size", "").split()]
        assert position[4:] == [0, 0]
        half_height = (abs(math.sin(position[3])) * size[1]
                       + abs(math.cos(position[3])) * size[2]) / 2
        collision_bottoms.append(position[2] - half_height)

    # Then every visible vertex stays above contact, without clamping telemetry.
    clearance = visual_bottom - min(collision_bottoms)
    assert 0.001 <= clearance <= 0.003, f"Mesh ground clearance is {clearance:.6f} m"
