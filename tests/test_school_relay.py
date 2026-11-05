"""The simulated aircraft holds for actual high-priority model detections."""

import importlib.util
from pathlib import Path

from neurovector_vision.models import BoundingBox, Detection, DetectResponse

spec = importlib.util.spec_from_file_location(
    "school_relay", Path(__file__).parents[1] / "scripts/school-vision-relay.py"
)
assert spec is not None and spec.loader is not None
relay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(relay)


def result(priority: int, distance: float | None = None, confidence: float = 0.9) -> DetectResponse:
    return DetectResponse(
        drone_id="school-drone-001",
        forwarded=1,
        elapsed_ms=1,
        detections=[
            Detection(
                label="obstacle",
                priority_level=priority,
                confidence=confidence,
                distance_m=distance,
                box=BoundingBox(x1=1, y1=1, x2=20, y2=20),
            )
        ],
    )


def test_critical_detection_holds_even_when_distance_is_unknown():
    observed = result(1)
    blocked = relay.blocked_by(observed)
    assert blocked


def test_nearby_object_holds_and_far_object_does_not():
    near, far = result(2, 2), result(2, 20)
    decisions = relay.blocked_by(near), relay.blocked_by(far)
    assert decisions == (True, False)


def test_leaf_and_low_confidence_detection_do_not_stop_the_demo():
    leaf, uncertain = result(3, 1), result(1, 1, 0.2)
    decisions = relay.blocked_by(leaf), relay.blocked_by(uncertain)
    assert decisions == (False, False)


def test_school_infrastructure_in_the_image_corridor_holds_without_fabricated_distance():
    observed = result(2)
    obstacle = observed.detections[0].model_copy(
        update={
            "label": "cable",
            "box": BoundingBox(x1=300, y1=100, x2=340, y2=200),
        }
    )
    observed = observed.model_copy(update={"detections": [obstacle]})
    assert relay.blocked_by(observed, 640)
    assert not relay.blocked_by(observed, 1600)
