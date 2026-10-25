"""Relay fresh ROS RGB frames through H.264/SRT to the WebRTC gateway."""

import math
import re
import subprocess
import threading
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class StreamError(RuntimeError):
    """Invalid stream configuration or camera frame."""


@dataclass(frozen=True, slots=True)
class Frame:
    width: int
    height: int
    pixel_format: str
    data: bytes
    received_at: float


def pack_image(message: Image) -> Frame:
    """Remove row padding and preserve the source colour channel order."""
    pixel_format = {"rgb8": "rgb24", "bgr8": "bgr24"}.get(message.encoding)
    if pixel_format is None:
        raise StreamError("camera encoding must be rgb8 or bgr8")
    width, height, step = message.width, message.height, message.step
    if width <= 0 or height <= 0 or step < width * 3 or len(message.data) != step * height:
        raise StreamError("invalid camera dimensions, stride or buffer length")
    data = memoryview(message.data)
    packed = b"".join(data[row * step:row * step + width * 3] for row in range(height))
    return Frame(width, height, pixel_format, packed, time.monotonic())


class CameraStream(Node):
    """Keep encoding and network backpressure off the ROS subscription thread."""

    def __init__(self) -> None:
        super().__init__("camera_stream")
        for name, value in [("drone_id", "drone-001"), ("srt_host", "127.0.0.1"),
                            ("srt_port", 8890), ("fps", 5.0), ("frame_timeout_s", 1.0)]:
            self.declare_parameter(name, value)
        drone_id = str(self.get_parameter("drone_id").value)
        host = str(self.get_parameter("srt_host").value)
        port = int(self.get_parameter("srt_port").value)
        self.fps = float(self.get_parameter("fps").value)
        self.frame_timeout = float(self.get_parameter("frame_timeout_s").value)
        if not re.fullmatch(r"[A-Za-z0-9_-]{1,128}", drone_id):
            raise StreamError("invalid drone_id")
        if not re.fullmatch(r"[A-Za-z0-9.-]+", host) or not 1 <= port <= 65535:
            raise StreamError("invalid SRT host or port")
        if not math.isfinite(self.fps) or not 1 <= self.fps <= 60:
            raise StreamError("fps must be between 1 and 60")
        if not math.isfinite(self.frame_timeout) or self.frame_timeout <= 0:
            raise StreamError("frame_timeout_s must be positive")
        self.url = f"srt://{host}:{port}?streamid=publish:{drone_id}&pkt_size=1316"
        self._frame: Frame | None = None
        self._stamp = -1
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._encoder: subprocess.Popen[bytes] | None = None
        self._failure: OSError | StreamError | None = None
        self.create_subscription(Image, "camera/image_raw", self.on_image, qos_profile_sensor_data)
        self.create_timer(0.2, self.check_worker)
        self._thread = threading.Thread(target=self.send_frames, name="camera-encoder", daemon=True)
        self._thread.start()

    def on_image(self, message: Image) -> None:
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if stamp <= self._stamp:
            return
        try:
            frame = pack_image(message)
        except StreamError as error:
            self.get_logger().warning(str(error))
            return
        with self._lock:
            self._frame = frame
            self._stamp = stamp

    def stop_encoder(self) -> None:
        encoder = self._encoder
        if encoder is None:
            return
        if encoder.poll() is None:
            encoder.terminate()
            try:
                encoder.wait(timeout=2)
            except subprocess.TimeoutExpired:
                encoder.kill()
                encoder.wait()
        if encoder.stdin is not None:
            encoder.stdin.close()
        self._encoder = None

    def send_frames(self) -> None:
        geometry: tuple[int, int, str] | None = None
        try:
            while not self._stop.wait(1 / self.fps):
                with self._lock:
                    frame = self._frame
                if frame is None or time.monotonic() - frame.received_at > self.frame_timeout:
                    self.stop_encoder()
                    continue
                current = (frame.width, frame.height, frame.pixel_format)
                if self._encoder is None or geometry != current:
                    self.stop_encoder()
                    self._encoder = subprocess.Popen([
                        "ffmpeg", "-hide_banner", "-loglevel", "warning", "-f", "rawvideo",
                        "-pixel_format", frame.pixel_format, "-video_size", f"{frame.width}x{frame.height}",
                        "-framerate", str(self.fps), "-i", "pipe:0", "-an", "-c:v", "libx264",
                        "-preset", "ultrafast", "-tune", "zerolatency", "-profile:v", "baseline",
                        "-pix_fmt", "yuv420p", "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2",
                        "-g", str(max(1, round(self.fps))), "-bf", "0", "-f", "mpegts", self.url,
                    ], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL)
                    geometry = current
                encoder = self._encoder
                if encoder.poll() is not None or encoder.stdin is None:
                    raise StreamError("FFmpeg stopped; check SRT gateway and restart camera-stream")
                encoder.stdin.write(frame.data)
                encoder.stdin.flush()
        except (OSError, StreamError) as error:
            if not self._stop.is_set():
                self._failure = error
        finally:
            self.stop_encoder()

    def check_worker(self) -> None:
        if self._failure is not None:
            raise StreamError("camera encoder failed") from self._failure

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2)
        encoder = self._encoder
        if self._thread.is_alive() and encoder is not None and encoder.poll() is None:
            encoder.kill()
        self._thread.join(timeout=3)


def main() -> None:
    rclpy.init()
    node = CameraStream()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping camera stream")
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
