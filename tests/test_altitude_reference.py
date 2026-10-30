"""Altitude datum is never asserted for an unknown world origin."""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sitl"))
from altitude_reference import require_fixture_origin


def test_world_origin_must_match_calibrated_fixture(tmp_path: Path) -> None:
    world = tmp_path / "world.sdf"
    world.write_text('<sdf><world name="school_procedural"><spherical_coordinates>'
        '<surface_model>EARTH_WGS84</surface_model><world_frame_orientation>ENU</world_frame_orientation>'
        '<latitude_deg>37.0</latitude_deg><longitude_deg>127.0</longitude_deg>'
        '<elevation>0</elevation><heading_deg>0</heading_deg></spherical_coordinates></world></sdf>')
    assert len(require_fixture_origin(world)) == 64
    world.write_text(world.read_text().replace('<elevation>0</elevation>', '<elevation>1</elevation>'))
    with pytest.raises(ValueError):
        require_fixture_origin(world)
