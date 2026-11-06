# /// script
# requires-python = ">=3.13"
# dependencies = ["httpx2[http2,brotli,zstd]", "opencv-python-headless", "pydantic"]
# ///
# Run with vision/.venv/bin/python sim/scripts/school-vision-relay.py
"""Relay real simulated frames to the configured vision API and publish telemetry."""

from __future__ import annotations

import logging
from io import BytesIO
import time
from pathlib import Path
from typing import Final

import cv2
import httpx2
import numpy as np
from school_order_relay import OrderRelay
from school_relay_runtime import InferencePublisher, SimState, TelemetryReporter, atomic_write, http_client
from school_vision_route import checked_building_corridor
from school_depth_ranges import with_depth_ranges

from neurovector_vision.models import DetectResponse

ROOT: Final = Path(__file__).resolve().parents[2] / "runtime/school"
BACKEND: Final = "http://127.0.0.1:18081"
VISION: Final = "http://127.0.0.1:18082"
LOG: Final = logging.getLogger("school-relay")


def blocked_by(result: DetectResponse, frame_width: int = 640, checked_building_clearance: bool = False,
               *, allow_unknown_building_in_column: bool = False) -> bool:
    """Conservative demo hold: critical objects, or nearby non-low-priority objects."""
    infrastructure = {
        "cable",
        "tower_lattice",
        "tower_tucohy",
        "tower_wooden",
        "tree_region",
        "building_region",
        "fence_region",
        "rock_region",
        "pole_region",
    }
    return any(
        d.confidence >= 0.5
        and (
            d.priority_level == 1
            or (
                d.priority_level == 2 and d.distance_m is not None and d.distance_m < 5
                and not (checked_building_clearance and d.label == "building_region" and d.distance_m >= 1.45)
            )
            or (
                d.label in infrastructure
                and not (allow_unknown_building_in_column and d.label == "building_region")
                and d.distance_m is None
                and d.box.x1 < frame_width * 0.6
                and d.box.x2 > frame_width * 0.4
            )
        )
        for d in result.detections
    )


def in_landing_column(state: SimState) -> bool:
    """Separate stationary handoff and checked vertical legs from forward travel."""
    return state.mission is not None and (
        state.mission.phase == "return_takeoff"
        or (state.mission.phase == "outbound" and state.waypoint == 0)
        or (not state.armed and state.mission.phase in {"delivery_landing", "awaiting_receipt"})
    )


def run() -> None:
    publisher = InferencePublisher(ROOT)
    with TelemetryReporter(publisher, BACKEND), OrderRelay(ROOT, BACKEND), http_client() as client:
        last_stamp = 0.0
        while True:
            try:
                state = publisher.read_state()
                generation = publisher.begin(state)
                if state.camera_timestamp == last_stamp:
                    time.sleep(0.2)
                    continue
                frame = (ROOT / "camera.jpg").read_bytes()
                image = cv2.imdecode(np.frombuffer(frame, dtype=np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    raise RuntimeError("Cannot decode the simulator camera JPEG")
                started = time.time()
                try:
                    depth_frame = BytesIO((ROOT / 'depth.npz').read_bytes())
                except OSError:
                    depth_frame = BytesIO()
                response = client.post(
                    VISION + "/detect", files={"file": ("camera.jpg", frame, "image/jpeg")}
                )
                response.raise_for_status()
                result = DetectResponse.model_validate(response.json())
                if result.forwarded != len(result.detections):
                    raise RuntimeError("Backend did not acknowledge every detection")
                result = with_depth_ranges(result, depth_frame, image.shape[:2], state.camera_timestamp, time.time())
                landing_column = in_landing_column(state)
                building_clear = landing_column or checked_building_corridor(
                    state, ROOT.parents[1] / "sim/fixtures/school.json"
                )
                blocked = blocked_by(result, image.shape[1], building_clear,
                                     allow_unknown_building_in_column=landing_column)
                publisher.publish(result, state, generation, started, blocked)

                for d in result.detections:
                    a, b, c, e = (int(d.box.x1), int(d.box.y1), int(d.box.x2), int(d.box.y2))
                    cv2.rectangle(image, (a, b), (c, e), (0, 80, 255), 2)
                    cv2.putText(
                        image,
                        f"{d.label} {d.confidence:.2f}",
                        (a, max(20, b - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 255, 255),
                        1,
                    )
                ok, encoded = cv2.imencode(".jpg", image)
                if ok:
                    atomic_write(ROOT, "detections.jpg", encoded.tobytes())
                LOG.info(
                    "frame detections=%d forwarded=%d blocked=%s elapsed_ms=%.0f",
                    len(result.detections),
                    result.forwarded,
                    blocked,
                    result.elapsed_ms,
                )
                last_stamp = state.camera_timestamp
                time.sleep(1)
            except (
                OSError,
                ValueError,
                KeyError,
                httpx2.HTTPError,
                TimeoutError,
                RuntimeError,
            ) as error:
                publisher.inference_failed(str(error))
                LOG.warning("relay unavailable: %s", error)
                time.sleep(2)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    run()
