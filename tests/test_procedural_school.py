"""Procedural fixture must retain physical school boxes without missing meshes."""
from __future__ import annotations

import json
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

SIM = Path(__file__).resolve().parents[1]


def test_procedural_world_uses_fixture_collision_without_mesh(tmp_path: Path) -> None:
    output = tmp_path / "school_procedural.sdf"
    subprocess.run([sys.executable, str(SIM / "scripts/generate-procedural-school.py"),
                    str(output)], check=True)
    root = ET.parse(output).getroot()
    fixture = json.loads((SIM / "fixtures/school.json").read_text())
    world = root.find("world")
    assert world is not None and world.attrib["name"] == "school_procedural"
    assert world.find(".//mesh") is None
    assert world.find("include") is None
    for building in fixture["buildings"]:
        link = world.find(f"model/link[@name='{building['id']}']")
        assert link is not None
        assert link.findtext("collision/geometry/box/size") == " ".join(
            format(value, ".12g") for value in building["size_m"]
        )
        assert link.find("visual/geometry/box") is not None
