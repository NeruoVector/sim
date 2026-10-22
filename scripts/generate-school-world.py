#!/usr/bin/env python3
"""Generate the static Gazebo scene from the shared synthetic school fixture."""

import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "fixtures/school.json"


def vector(value, count=3, positive=False):
    if len(value) != count or any(
        isinstance(x, bool)
        or not isinstance(x, (int, float))
        or not math.isfinite(x)
        or (positive and x <= 0)
        for x in value
    ):
        raise ValueError("invalid finite fixture vector")
    return value


def element(parent, tag, value=None, **attributes):
    node = ET.SubElement(parent, tag, attributes)
    if value is not None:
        node.text = str(value)
    return node


def numbers(values):
    return " ".join(format(x, ".12g") for x in values)


def box(model, name, position, size, color, *, collision=True, visual=True, yaw=0):
    vector(position)
    vector(size, positive=True)
    link = element(model, "link", name=name)
    element(link, "pose", numbers([*position, 0, 0, yaw]))
    for kind in (["collision"] if collision else []) + (["visual"] if visual else []):
        node = element(link, kind, name=f"{name}_{kind}")
        element(element(element(node, "geometry"), "box"), "size", numbers(size))
        if kind == "visual":
            material = element(node, "material")
            element(material, "ambient", color)
            element(material, "diffuse", color)


def mesh(model, name, uri, pose, scale, color):
    vector(pose)
    vector(scale, positive=True)
    link = element(model, "link", name=name)
    element(link, "pose", numbers([*pose, 0, 0, 0]))
    for kind in ("collision", "visual"):
        node = element(link, kind, name=f"{name}_{kind}")
        geometry = element(node, "geometry")
        mesh_node = element(geometry, "mesh")
        element(mesh_node, "uri", uri)
        element(mesh_node, "scale", numbers(scale))
        if kind == "visual":
            material = element(node, "material")
            element(material, "ambient", color)
            element(material, "diffuse", color)


def xml(root):
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode", xml_declaration=True) + "\n"


def generate(data):
    if data["synthetic"] is not True or data["drone"]["sim_model"] != "gz_x500_depth":
        raise ValueError(
            "scene requires synthetic data and the existing gz_x500_depth aircraft"
        )
    root = ET.Element("sdf", {"version": "1.9"})
    model = element(root, "model", name="school-demo")
    element(model, "static", "true")
    campus = data["campus"]
    center = vector(campus["ground_center_enu"])
    box(
        model,
        "campus_ground",
        [center[0], center[1], center[2] - 0.05],
        [*vector(campus["size_m"], 2, positive=True), 0.1],
        "0.34 0.48 0.30 1",
    )
    mesh(
        model,
        "campus_map",
        campus["mesh_uri"],
        campus["mesh_pose_enu"],
        campus["mesh_scale"],
        "0.68 0.70 0.68 1",
    )
    for building in data["buildings"]:
        box(
            model,
            building["id"],
            building["center_enu"],
            building["size_m"],
            "0.75 0.64 0.47 1",
            visual=False,
        )
    playground = data["playground"]
    box(
        model,
        "playground",
        playground["center_enu"],
        playground["size_m"],
        "0.67 0.33 0.23 1",
        visual=False,
    )
    window = data["window"]
    position = vector(window["position_enu"])
    bearing = math.radians(window["outward_bearing_deg"])
    marker = [
        position[0] + 0.04 * math.sin(bearing),
        position[1] + 0.04 * math.cos(bearing),
        position[2],
    ]
    box(
        model,
        "window_target_marker",
        marker,
        [1.5, 0.04, 1.5],
        "0.1 0.65 0.95 1",
        collision=False,
        yaw=math.pi - bearing,
    )
    takeoff = vector(data["takeoff_enu"])
    box(
        model,
        "takeoff_marker",
        [takeoff[0], takeoff[1], center[2] + 0.015],
        [1.5, 1.5, 0.02],
        "0.95 0.95 0.85 1",
        collision=False,
    )
    # shortcut: risk-class boxes; use person/car assets before image-based detector benchmarks.
    for obstacle in data["obstacles"]:
        priority = obstacle["priority"]
        if priority not in (1, 2, 3):
            raise ValueError("invalid obstacle priority")
        radius = obstacle["radius_m"]
        vector([radius], 1, positive=True)
        color = {1: "0.95 0.1 0.1 1", 2: "1 0.65 0.05 1", 3: "0.2 0.7 0.25 1"}[priority]
        box(
            model,
            f"risk_{priority}_{obstacle['id']}",
            obstacle["position_enu"],
            [2 * radius] * 3,
            color,
        )

    world_root = ET.Element("sdf", {"version": "1.9"})
    world = element(world_root, "world", name="school")
    physics = element(world, "physics", name="default_physics", type="ignored")
    element(physics, "max_step_size", "0.004")
    element(physics, "real_time_factor", "1")
    element(world, "gravity", "0 0 -9.81")
    for filename, name in (
        ("gz-sim-physics-system", "gz::sim::systems::Physics"),
        ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
        ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster"),
        ("gz-sim-sensors-system", "gz::sim::systems::Sensors"),
        ("gz-sim-imu-system", "gz::sim::systems::Imu"),
        ("gz-sim-magnetometer-system", "gz::sim::systems::Magnetometer"),
        ("gz-sim-navsat-system", "gz::sim::systems::NavSat"),
        ("gz-sim-air-pressure-system", "gz::sim::systems::AirPressure"),
        ("gz-sim-contact-system", "gz::sim::systems::Contact"),
    ):
        plugin = element(world, "plugin", filename=filename, name=name)
        if name.endswith("::Sensors"):
            element(plugin, "render_engine", "ogre2")
    origin = data["origin"]
    if not (-90 <= origin["lat"] <= 90 and -180 <= origin["lon"] <= 180):
        raise ValueError("invalid synthetic origin")
    coordinates = element(world, "spherical_coordinates")
    element(coordinates, "surface_model", "EARTH_WGS84")
    element(coordinates, "world_frame_orientation", "ENU")
    element(coordinates, "latitude_deg", origin["lat"])
    element(coordinates, "longitude_deg", origin["lon"])
    element(coordinates, "elevation", "0")
    element(coordinates, "heading_deg", "0")
    light = element(world, "light", name="sun", type="directional")
    element(light, "pose", "0 0 50 0 0 0")
    element(light, "diffuse", "0.9 0.9 0.9 1")
    element(light, "specular", "0.2 0.2 0.2 1")
    element(light, "direction", "-0.5 0.2 -1")
    element(light, "cast_shadows", "true")
    element(element(world, "include"), "uri", "model://school-demo")

    config = ET.Element("model")
    element(config, "name", "Synthetic school demo with STL campus")
    element(config, "version", "1.0")
    element(config, "sdf", "model.sdf", version="1.9")
    element(
        config,
        "description",
        "Synthetic school fixture with an STL campus mesh; no real recipients.",
    )
    return {
        ROOT / "models/environments/school-demo/model.sdf": xml(root),
        ROOT / "models/environments/school-demo/model.config": xml(config),
        ROOT / "worlds/school.sdf": xml(world_root),
    }


def validate(outputs, data):
    model = ET.fromstring(outputs[ROOT / "models/environments/school-demo/model.sdf"])
    world = ET.fromstring(outputs[ROOT / "worlds/school.sdf"])
    links = {link.attrib["name"]: link for link in model.findall("model/link")}
    map_link = links["campus_map"]
    for kind in ("collision", "visual"):
        mesh_node = map_link.find(f"{kind}/geometry/mesh")
        assert mesh_node.findtext("uri") == data["campus"]["mesh_uri"]
        assert mesh_node.findtext("scale") == numbers(data["campus"]["mesh_scale"])
    assert (ROOT / "models/environments/school-demo/meshes/school_map.stl").is_file()
    drone_root = ET.parse(ROOT / "models/drones/x500_base/model.sdf").getroot()
    drone_model = drone_root.find("model")
    assert drone_model is not None and drone_model.attrib["name"] == "x500_base"
    drone_visuals = drone_model.findall("link/visual/geometry/mesh")
    assert len(drone_visuals) == 1
    assert drone_visuals[0].findtext("uri") == "model://x500_base/meshes/drone.stl"
    assert (ROOT / "models/drones/x500_base/meshes/drone.stl").is_file()
    assert len(drone_model.findall("link")) == 5
    assert len(drone_model.findall("joint")) == 4
    assert len(drone_model.findall("link/sensor")) == 4
    for building in data["buildings"]:
        link = links[building["id"]]
        assert link.findtext("pose") == numbers([*building["center_enu"], 0, 0, 0])
        assert link.findtext("collision/geometry/box/size") == numbers(
            building["size_m"]
        )
        assert link.find("visual") is None
    assert model.findtext("model/static") == "true"
    assert world.find("world").attrib["name"] == "school"
    assert world.findtext("world/include/uri") == "model://school-demo"
    assert len(world.findall("world/include")) == 1
    assert world.find("world/model") is None  # PX4 alone spawns its existing aircraft.
    assert len(links) == len(data["buildings"]) + len(data["obstacles"]) + 5
    assert links["window_target_marker"].find("collision") is None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--check",
        action="store_true",
        help="check XML and detect stale generated files",
    )
    args = parser.parse_args()
    data = json.loads(FIXTURE.read_text(encoding="utf-8"))
    outputs = generate(data)
    validate(outputs, data)
    for path, content in outputs.items():
        if args.check:
            if not path.exists() or path.read_text(encoding="utf-8") != content:
                raise SystemExit(
                    f"stale generated scene: {path}; run {Path(__file__).name}"
                )
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content, encoding="utf-8")
    print("school scene XML and fixture consistency: OK")


if __name__ == "__main__":
    main()
