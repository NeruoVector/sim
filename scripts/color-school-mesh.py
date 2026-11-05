#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = []
# ///
# Run: python3 sim/scripts/color-school-mesh.py [--check]
"""Give the imported school visual semantic materials without changing its mesh."""

import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Final

ROOT: Final = Path(__file__).resolve().parents[1]
SOURCE: Final = ROOT / "models/environments/school-demo/meshes/school_map.stl"
TARGET: Final = SOURCE.with_name("school_map_colored.dae")
Vertex = tuple[float, float, float]
PALETTE: Final = {
    "pavement": (0.52, 0.57, 0.61, 1),
    "sports_turf": (0.25, 0.48, 0.29, 1),
    "sports_court": (0.66, 0.39, 0.24, 1),
    "facade": (0.91, 0.81, 0.61, 1),
    "roof": (0.63, 0.23, 0.16, 1),
    "glass": (0.18, 0.43, 0.59, 1),
    "foliage": (0.22, 0.42, 0.18, 1),
    "foliage_light": (0.35, 0.54, 0.23, 1),
    "trunk": (0.39, 0.28, 0.18, 1),
}
# Imported STL coordinates, before the existing campus scale and translation.
BUILDING_ZONES: Final = (
    (-270, -234, -18, 123),
    (-270, -126, 90, 123),
    (-220, -146, -18, 49),
    (-114, 115, 58, 96),
    (122, 233, -78, -14),
)
NS: Final = "http://www.collada.org/2005/11/COLLADASchema"


class MeshFormatError(Exception):
    """The source is not a complete binary STL."""


def material_for(vertices: tuple[Vertex, Vertex, Vertex], normal: Vertex) -> str:
    x, y, z = (sum(point[axis] for point in vertices) / 3 for axis in range(3))
    if z <= 6:
        if -112 <= x <= 109 and -119 <= y <= 58:
            return "sports_turf"
        if -258 <= x <= -128 and -123 <= y <= -23:
            return "sports_court"
        return "pavement"
    if any(left <= x <= right and bottom <= y <= top
           for left, right, bottom, top in BUILDING_ZONES):
        if normal[2] > 0.55 and z >= 16:
            return "roof"
        along_wall = y if abs(normal[0]) > abs(normal[1]) else x
        if abs(normal[2]) < 0.25 and 10 < z < 29:
            if int(z) // 4 % 2 == 0 and int(along_wall) % 12 < 8:
                return "glass"
        return "facade"
    if z < 10 and abs(normal[2]) < 0.3:
        return "trunk"
    return "foliage_light" if normal[2] > 0.35 else "foliage"


def node(parent: ET.Element, tag: str, text: str | None = None,
         **attributes: str) -> ET.Element:
    child = ET.SubElement(parent, tag, attributes)
    child.text = text
    return child


def source_node(mesh: ET.Element, name: str, values: list[Vertex]) -> None:
    source = node(mesh, "source", id=name)
    node(source, "float_array", " ".join(f"{v:.9g}" for row in values for v in row),
         id=f"{name}-array", count=str(len(values) * 3))
    accessor = node(node(source, "technique_common"), "accessor",
                    source=f"#{name}-array", count=str(len(values)), stride="3")
    for axis in ("X", "Y", "Z"):
        node(accessor, "param", name=axis, type="float")


def generate() -> str:
    data = SOURCE.read_bytes()
    if len(data) < 84:
        raise MeshFormatError("Missing binary STL header")
    face_count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + 50 * face_count:
        raise MeshFormatError("STL triangle count does not match its length")
    positions: dict[Vertex, int] = {}
    normals: dict[Vertex, int] = {}
    groups: dict[str, list[int]] = {name: [] for name in PALETTE}
    for record in struct.iter_unpack("<12fH", memoryview(data)[84:]):
        normal: Vertex = (record[0], record[1], record[2])
        triangle = ((record[3], record[4], record[5]),
                    (record[6], record[7], record[8]),
                    (record[9], record[10], record[11]))
        normal_id = normals.setdefault(normal, len(normals))
        indices = groups[material_for(triangle, normal)]
        for position in triangle:
            indices.extend((positions.setdefault(position, len(positions)), normal_id))
    root = ET.Element("COLLADA", xmlns=NS, version="1.4.1")
    asset = node(root, "asset")
    node(asset, "unit", name="source_mesh_unit", meter="1")
    node(asset, "up_axis", "Z_UP")
    effects = node(root, "library_effects")
    materials = node(root, "library_materials")
    for name, color in PALETTE.items():
        effect = node(effects, "effect", id=f"{name}-effect")
        phong = node(node(node(effect, "profile_COMMON"), "technique", sid="common"), "phong")
        for channel in ("ambient", "diffuse"):
            node(node(phong, channel), "color", " ".join(str(v) for v in color))
        node(node(phong, "specular"), "color", "0.06 0.06 0.06 1")
        node(node(phong, "shininess"), "float", "12")
        node(node(materials, "material", id=name, name=name), "instance_effect", url=f"#{name}-effect")
    mesh = node(node(node(root, "library_geometries"), "geometry", id="school"), "mesh")
    source_node(mesh, "positions", list(positions))
    source_node(mesh, "normals", list(normals))
    node(node(mesh, "vertices", id="vertices"), "input", semantic="POSITION", source="#positions")
    for name, indices in groups.items():
        if not indices:
            continue
        triangles = node(mesh, "triangles", count=str(len(indices) // 6), material=name)
        node(triangles, "input", semantic="VERTEX", source="#vertices", offset="0")
        node(triangles, "input", semantic="NORMAL", source="#normals", offset="1")
        node(triangles, "p", " ".join(str(index) for index in indices))
    scene = node(node(root, "library_visual_scenes"), "visual_scene", id="campus")
    instance = node(node(scene, "node", id="campus-node"), "instance_geometry", url="#school")
    binding = node(node(instance, "bind_material"), "technique_common")
    for name in groups:
        node(binding, "instance_material", symbol=name, target=f"#{name}")
    node(node(root, "scene"), "instance_visual_scene", url="#campus")
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode", xml_declaration=True) + "\n"


def main() -> None:
    arguments = sys.argv[1:]
    if arguments not in ([], ["--check"]):
        raise SystemExit("Usage: color-school-mesh.py [--check]")
    content = generate()
    if arguments == ["--check"]:
        if not TARGET.exists() or TARGET.read_text(encoding="utf-8") != content:
            raise SystemExit("Colored school mesh is stale; run color-school-mesh.py")
    else:
        TARGET.write_text(content, encoding="utf-8")
    print("School visual materials generated; original STL geometry preserved")


if __name__ == "__main__":
    main()
