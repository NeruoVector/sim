"""Project co-located Gazebo depth onto RGB boxes using the real camera FOVs."""
from __future__ import annotations

import math
from io import BytesIO
from pathlib import Path

import numpy as np

from neurovector_vision.models import DetectResponse


def with_depth_ranges(result: DetectResponse, path: Path | BytesIO, rgb_shape: tuple[int, int], camera_timestamp: float, now: float) -> DetectResponse:
    try:
        with np.load(path, allow_pickle=False) as frame:
            stamp = float(frame['timestamp'])
            depth = frame['depth']
            near_m, far_m = float(frame['near_m']), float(frame['far_m'])
    except (OSError, ValueError, KeyError, EOFError):
        return result
    if not 0 <= now - stamp <= 4 or abs(camera_timestamp - stamp) > .75:
        return result
    if depth.ndim != 2 or depth.dtype != np.float32 or min(depth.shape) < 2:
        return result
    if not 0 < near_m < 5 < far_m <= 100:
        return result
    rgb_height, rgb_width = rgb_shape
    height, width = depth.shape
    rgb_focal = rgb_width / (2 * math.tan(1.204 / 2))
    depth_focal = width / (2 * math.tan(1.274 / 2))
    ratio = depth_focal / rgb_focal
    detections = []
    for detection in result.detections:
        box = detection.box
        x1 = max(0, math.floor(((box.x1 * .8 + box.x2 * .2) - rgb_width / 2) * ratio + width / 2))
        x2 = min(width, math.ceil(((box.x1 * .2 + box.x2 * .8) - rgb_width / 2) * ratio + width / 2))
        y1 = max(0, math.floor(((box.y1 * .8 + box.y2 * .2) - rgb_height / 2) * ratio + height / 2))
        y2 = min(height, math.ceil(((box.y1 * .2 + box.y2 * .8) - rgb_height / 2) * ratio + height / 2))
        patch = depth[y1:y2, x1:x2]
        measured = np.where(np.isposinf(patch), far_m, np.where(np.isneginf(patch), near_m, patch))
        valid = measured[np.isfinite(measured) & (measured >= near_m) & (measured <= far_m)]
        if detection.distance_m is None and valid.size >= 12:
            distance = float(np.quantile(valid, .1))
            lower_bound = math.isclose(distance, far_m, rel_tol=1e-6)
            upper_bound = math.isclose(distance, near_m, rel_tol=1e-6) and bool(np.isneginf(patch).any())
            if lower_bound:
                distance = far_m
            if upper_bound:
                distance = near_m
            detection = detection.model_copy(update={
                'distance_m': distance,
                'distance_source': 'gazebo_depth_lower_bound' if lower_bound else 'gazebo_depth_upper_bound' if upper_bound else 'gazebo_depth',
                'distance_note': 'no_return_before_far_clip' if lower_bound else 'return_inside_near_clip' if upper_bound else 'measured_depth_rgb_fov_projection_p10',
                'distance_confidence': None,
            })
        detections.append(detection)
    return result.model_copy(update={'detections': detections})
