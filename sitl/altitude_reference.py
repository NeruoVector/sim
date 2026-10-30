"""Verify the disposable procedural world's calibrated altitude origin."""
from __future__ import annotations

import hashlib
import json
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path


def require_fixture_origin(world: Path) -> str:
    root = ET.fromstring(world.read_bytes())
    scene = root.find("world")
    if scene is None or scene.get("name") != "school_procedural":
        raise ValueError("unexpected procedural world")
    coordinates = scene.find("spherical_coordinates")
    expected = {"surface_model": "EARTH_WGS84", "world_frame_orientation": "ENU",
                "latitude_deg": "37.0", "longitude_deg": "127.0",
                "elevation": "0", "heading_deg": "0"}
    if coordinates is None or any(coordinates.findtext(key) != value for key, value in expected.items()):
        raise ValueError("synthetic geographic and altitude origin is not calibrated")
    return hashlib.sha256(world.read_bytes()).hexdigest()


def require_running_world(container: str, world: Path) -> None:
    digest = require_fixture_origin(world)
    env = subprocess.run(["docker", "inspect", "--format", "{{json .Config.Env}}", container],
                         check=True, capture_output=True, text=True, timeout=5)
    values = dict(item.split("=", 1) for item in json.loads(env.stdout) if "=" in item)
    if values.get("PX4_GZ_WORLD") != "school_procedural" or values.get("PX4_SIM_MODEL") != "gz_x500":
        raise ValueError("PX4 container is not using the synthetic fixture")
    remote = subprocess.run(["docker", "exec", container, "sha256sum",
        "/opt/px4-gazebo/share/gz/worlds/school_procedural.sdf"],
        check=True, capture_output=True, text=True, timeout=5)
    if remote.stdout.split()[0] != digest:
        raise ValueError("running Gazebo world differs from the calibrated fixture")
