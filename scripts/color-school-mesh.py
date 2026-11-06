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
import zlib
from pathlib import Path
from typing import Final

ROOT: Final = Path(__file__).resolve().parents[1]
SOURCE: Final = ROOT / "models/environments/school-demo/meshes/school_map.stl"
TARGET: Final = SOURCE.with_name("school_map_colored.dae")
TEXTURE: Final = SOURCE.with_name("school_facade.png")
Vertex = tuple[float, float, float]
PALETTE: Final = {
    "pavement": (0.52, 0.57, 0.61, 1),
    "sports_turf": (0.25, 0.48, 0.29, 1),
    "sports_court": (0.66, 0.39, 0.24, 1),
    "facade": (0.91, 0.81, 0.61, 1),
    "roof": (0.63, 0.23, 0.16, 1),
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
    for axis in (("S", "T", "P") if name == "texcoords" else ("X", "Y", "Z")):
        node(accessor, "param", name=axis, type="float")


def facade_texture() -> bytes:
    """Make a repeating rectangular window bay, independent of triangulation."""
    def chunk(kind: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data))

    rows = bytearray()
    for y in range(64):
        rows.append(0)
        for x in range(64):
            color = (46, 110, 150) if 12 <= x < 52 and 16 <= y < 48 else (232, 207, 156)
            if 30 <= x < 34 or 30 <= y < 34:
                color = (232, 207, 156)
            rows.extend(color)
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", 64, 64, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(rows)) + chunk(b"IEND", b""))


def generate() -> str:
    data = SOURCE.read_bytes()
    if len(data) < 84:
        raise MeshFormatError("Missing binary STL header")
    face_count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + 50 * face_count:
        raise MeshFormatError("STL triangle count does not match its length")
    positions: dict[Vertex, int] = {}
    normals: dict[Vertex, int] = {}
    texcoords: dict[Vertex, int] = {}
    groups: dict[str, list[int]] = {name: [] for name in PALETTE}
    for record in struct.iter_unpack("<12fH", memoryview(data)[84:]):
        normal: Vertex = (record[0], record[1], record[2])
        triangle = ((record[3], record[4], record[5]),
                    (record[6], record[7], record[8]),
                    (record[9], record[10], record[11]))
        normal_id = normals.setdefault(normal, len(normals))
        indices = groups[material_for(triangle, normal)]
        for position in triangle:
            along_wall = position[1] if abs(normal[0]) > abs(normal[1]) else position[0]
            uv: Vertex = (along_wall / 12, position[2] / 8, 0)
            indices.extend((positions.setdefault(position, len(positions)), normal_id,
                            texcoords.setdefault(uv, len(texcoords))))
    root = ET.Element("COLLADA", xmlns=NS, version="1.4.1")
    asset = node(root, "asset")
    node(asset, "unit", name="source_mesh_unit", meter="1")
    node(asset, "up_axis", "Z_UP")
    images = node(root, "library_images")
    node(node(images, "image", id="facade-image"), "init_from", TEXTURE.name)
    effects = node(root, "library_effects")
    materials = node(root, "library_materials")
    for name, color in PALETTE.items():
        effect = node(effects, "effect", id=f"{name}-effect")
        profile = node(effect, "profile_COMMON")
        if name == "facade":
            surface = node(node(profile, "newparam", sid="facade-surface"), "surface", type="2D")
            node(surface, "init_from", "facade-image")
            sampler = node(node(profile, "newparam", sid="facade-sampler"), "sampler2D")
            node(sampler, "source", "facade-surface")
            node(sampler, "wrap_s", "WRAP")
            node(sampler, "wrap_t", "WRAP")
        phong = node(node(profile, "technique", sid="common"), "phong")
        for channel in ("ambient", "diffuse"):
            channel_node = node(phong, channel)
            if name == "facade":
                node(channel_node, "texture", texture="facade-sampler", texcoord="UVMap")
            else:
                node(channel_node, "color", " ".join(str(v) for v in color))
        node(node(phong, "specular"), "color", "0.06 0.06 0.06 1")
        node(node(phong, "shininess"), "float", "12")
        node(node(materials, "material", id=name, name=name), "instance_effect", url=f"#{name}-effect")
    mesh = node(node(node(root, "library_geometries"), "geometry", id="school"), "mesh")
    source_node(mesh, "positions", list(positions))
    source_node(mesh, "normals", list(normals))
    source_node(mesh, "texcoords", list(texcoords))
    node(node(mesh, "vertices", id="vertices"), "input", semantic="POSITION", source="#positions")
    for name, indices in groups.items():
        if not indices:
            continue
        triangles = node(mesh, "triangles", count=str(len(indices) // 9), material=name)
        node(triangles, "input", semantic="VERTEX", source="#vertices", offset="0")
        node(triangles, "input", semantic="NORMAL", source="#normals", offset="1")
        node(triangles, "input", semantic="TEXCOORD", source="#texcoords", offset="2", set="0")
        node(triangles, "p", " ".join(str(index) for index in indices))
    scene = node(node(root, "library_visual_scenes"), "visual_scene", id="campus")
    instance = node(node(scene, "node", id="campus-node"), "instance_geometry", url="#school")
    binding = node(node(instance, "bind_material"), "technique_common")
    for name in groups:
        material = node(binding, "instance_material", symbol=name, target=f"#{name}")
        node(material, "bind_vertex_input", semantic="UVMap", input_semantic="TEXCOORD", input_set="0")
    node(node(root, "scene"), "instance_visual_scene", url="#campus")
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode", xml_declaration=True) + "\n"


def main() -> None:
    arguments = sys.argv[1:]
    if arguments not in ([], ["--check"]):
        raise SystemExit("Usage: color-school-mesh.py [--check]")
    content = generate()
    texture = facade_texture()
    if arguments == ["--check"]:
        if not TARGET.exists() or TARGET.read_text(encoding="utf-8") != content:
            raise SystemExit("Colored school mesh is stale; run color-school-mesh.py")
        if not TEXTURE.exists() or TEXTURE.read_bytes() != texture:
            raise SystemExit("School facade texture is stale; run color-school-mesh.py")
    else:
        TARGET.write_text(content, encoding="utf-8")
        TEXTURE.write_bytes(texture)
    print("School visual materials generated; original STL geometry preserved")


if __name__ == "__main__":
    main()
