import pytest
import rclpy
from sensor_msgs.msg import Image

from nv_telemetry.camera_stream import CameraStream, StreamError, pack_image


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
