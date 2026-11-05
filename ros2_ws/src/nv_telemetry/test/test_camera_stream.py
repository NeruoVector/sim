import json
import subprocess
import time
from pathlib import Path

import pytest
import rclpy
from sensor_msgs.msg import Image

from nv_telemetry.camera_stream import CameraStream, StreamError, encoder_command, pack_image


def image() -> Image:
    return Image(width=2, height=2, step=8, encoding="bgr8",
                 data=bytes([1, 2, 3, 4, 5, 6, 99, 99] * 2))


def test_packs_padded_bgr_rows_without_swapping_channels():
    frame = pack_image(image())
    assert frame.pixel_format == "bgr24"
    assert frame.data == bytes([1, 2, 3, 4, 5, 6] * 2)


@pytest.mark.parametrize("field,value", [("encoding", "16UC1"), ("width", 0),
                                       ("step", 5), ("data", b"short")])
def test_rejects_bad_frame(field, value):
    message = image()
    setattr(message, field, value)
    with pytest.raises(StreamError):
        pack_image(message)


def test_replayed_frame_cannot_refresh_live_stream():
    rclpy.init()
    node = CameraStream()
    try:
        message = image()
        message.header.stamp.sec = 3
        node.on_image(message)
        first = node._frame
        node.on_image(message)
        assert node._frame is first
        message.header.stamp.sec = 2
        node.on_image(message)
        assert node._frame is first
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


def test_encoder_timestamps_follow_elapsed_time_when_frame_delivery_slows(tmp_path: Path) -> None:
    frame = pack_image(image())
    recording = tmp_path / "camera.ts"
    with subprocess.Popen(encoder_command(frame, 5.0, str(recording)),
                          stdin=subprocess.PIPE, stdout=subprocess.DEVNULL) as encoder:
        assert encoder.stdin is not None
        started_at = time.monotonic()
        for index in range(8):
            if index:
                time.sleep(0.4)
            encoder.stdin.write(frame.data)
            encoder.stdin.flush()
        elapsed = time.monotonic() - started_at
        encoder.stdin.close()
        assert encoder.wait(timeout=10) == 0
    probe = subprocess.run([
        "ffprobe", "-v", "error", "-select_streams", "v:0", "-show_packets",
        "-show_entries", "packet=pts_time", "-of", "json", str(recording),
    ], check=True, capture_output=True, text=True)
    packets = json.loads(probe.stdout)["packets"]
    duration = float(packets[-1]["pts_time"]) - float(packets[0]["pts_time"])
    assert duration == pytest.approx(elapsed, abs=0.4)
