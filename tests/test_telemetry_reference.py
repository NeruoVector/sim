"""Only an explicitly verified home may assert a telemetry altitude reference."""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sitl"))
from mission_backend import require_plan_reference, telemetry_payload


def test_unknown_home_omits_reference() -> None:
    sample = {"utc":"2026-01-01T00:00:00+00:00", "latitude":37.0, "longitude":127.0,
              "relative_altitude":8, "vn":0, "ve":0, "vd":0, "battery":50,
              "heading":0, "armed":True}
    assert "altitude_reference" not in telemetry_payload(sample)
    assert telemetry_payload(sample, home_verified=True)["altitude_reference"] == "home_relative"


def test_unknown_or_changed_plan_reference_fails_closed() -> None:
    typed = {"altitude_reference": "home_relative"}
    require_plan_reference({"payload": typed}, typed)
    with pytest.raises(ValueError):
        require_plan_reference({"payload": {}}, typed)
    with pytest.raises(ValueError):
        require_plan_reference({"payload": typed}, {"altitude_reference": "mean_sea_level"})
