"""Insert one named synthetic collision object in the task-owned Gazebo world."""
from __future__ import annotations

import json
import subprocess

NAME = "nv_synthetic_obstacle_7bd1"
CONTAINERS = ("nv-autonomy-obstacle-7bd1", "nv-autonomy-auto-7bd1")


def spawn_obstacle(container: str) -> dict:
    if container not in CONTAINERS:
        raise ValueError("obstacle insertion requires its dedicated synthetic container")
    sdf = (f'<sdf version="1.9"><model name="{NAME}"><static>true</static>'
           '<pose>0 25 8 0 0 0</pose><link name="obstacle"><collision name="collision">'
           '<geometry><sphere><radius>0.7</radius></sphere></geometry></collision>'
           '<visual name="visual"><geometry><sphere><radius>0.7</radius></sphere>'
           '</geometry></visual></link></model></sdf>')
    base = ["docker", "exec", container, "gz", "service", "-s"]
    created = subprocess.run(base + ["/world/school_procedural/create", "--reqtype",
        "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean", "--timeout", "5000",
        "--req", "sdf: " + json.dumps(sdf)], capture_output=True, text=True, check=True, timeout=15)
    if "data: true" not in created.stdout:
        raise RuntimeError("Gazebo did not confirm collision model creation")
    scene = subprocess.run(base + ["/world/school_procedural/scene/info", "--reqtype",
        "gz.msgs.Empty", "--reptype", "gz.msgs.Scene", "--timeout", "5000", "--req", ""],
        capture_output=True, text=True, check=True, timeout=15).stdout
    index = scene.find(f'name: "{NAME}"')
    # The task-owned model has an identity pose except for north=25 and up=8.
    if index < 0 or "y: 25" not in scene[index:index+200] or "z: 8" not in scene[index:index+200]:
        raise RuntimeError("Gazebo did not report the expected named obstacle pose")
    return {"id": NAME, "priority": 1, "position_enu": [0.0, 25.0, 8.0], "radius_m": 0.7}
