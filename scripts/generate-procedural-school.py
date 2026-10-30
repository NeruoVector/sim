#!/usr/bin/env python3
"""Build an explicitly procedural school world from the synthetic fixture."""

from __future__ import annotations

import argparse
import importlib.util
import json
import xml.etree.ElementTree as ET
from pathlib import Path

SIM = Path(__file__).resolve().parents[1]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="destination SDF file")
    args = parser.parse_args()
    module_spec = importlib.util.spec_from_file_location(
        "school_world", SIM / "scripts/generate-school-world.py"
    )
    if module_spec is None or module_spec.loader is None:
        raise RuntimeError("school world generator unavailable")
    generator = importlib.util.module_from_spec(module_spec)
    module_spec.loader.exec_module(generator)
    fixture = json.loads((SIM / "fixtures/school.json").read_text())
    outputs = generator.generate(fixture)
    model_root = ET.fromstring(outputs[SIM / "models/environments/school-demo/model.sdf"])
    world_root = ET.fromstring(outputs[SIM / "worlds/school.sdf"])
    model = model_root.find("model")
    world = world_root.find("world")
    if model is None or world is None:
        raise RuntimeError("generated school is missing model or world")
    model.set("name", "procedural_school_fixture")
    for link in list(model.findall("link")):
        if link.get("name") == "campus_map":
            model.remove(link)
        elif link.get("name") in {building["id"] for building in fixture["buildings"]}:
            collision = link.find("collision")
            if collision is None:
                raise RuntimeError("fixture building is missing its collision box")
            visual = ET.SubElement(link, "visual", name="building_visual")
            geometry = collision.find("geometry")
            if geometry is None:
                raise RuntimeError("fixture building is missing collision geometry")
            visual.append(ET.fromstring(ET.tostring(geometry)))
    world.set("name", "school_procedural")
    include = world.find("include")
    if include is None:
        raise RuntimeError("generated school is missing model include")
    world.remove(include)
    world.append(model)
    ET.indent(world_root, space="  ")
    args.output.write_text(
        ET.tostring(world_root, encoding="unicode", xml_declaration=True) + "\n"
    )


if __name__ == "__main__":
    main()
