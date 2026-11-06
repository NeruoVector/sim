"""Export actual aligned OakD depth for simulation obstacle range checks."""
from __future__ import annotations

import os
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class SchoolDepth(Node):
    def __init__(self) -> None:
        super().__init__('school_depth')
        if os.environ.get('ROS_DOMAIN_ID') != '73' or os.environ.get('NV_SCHOOL_SCENE') != '1':
            raise RuntimeError('Depth export requires isolated school SITL')
        self.last_frame = 0.0
        model = ET.parse('/tmp/nv-school-models/OakD-Lite/model.sdf')
        camera = model.find(".//sensor[@type='depth_camera']/camera")
        self.near_m = float(camera.findtext('clip/near'))
        self.far_m = float(camera.findtext('clip/far'))
        self.create_subscription(Image, '/camera/depth/image_raw', self.on_depth, qos_profile_sensor_data)

    def on_depth(self, message: Image) -> None:
        now = time.time()
        if now - self.last_frame < 0.2 or message.encoding != '32FC1':
            return
        dtype = np.dtype('>f4' if message.is_bigendian else '<f4')
        rows = np.frombuffer(bytes(message.data), dtype=dtype).reshape(message.height, message.step // 4)
        depth = rows[:, :message.width].astype(np.float32)
        target = Path('/runtime/depth.npz')
        temporary = target.with_suffix('.tmp')
        with temporary.open('wb') as stream:
            np.savez(stream, depth=depth, timestamp=now, near_m=self.near_m, far_m=self.far_m)
        temporary.replace(target)
        self.last_frame = now


def main() -> None:
    rclpy.init()
    node = SchoolDepth()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
