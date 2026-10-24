#!/usr/bin/env python3
"""Read-only integration check. Run inside the sourced simulation container."""

import json
import math
import time

import rclpy
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray


def main():
    rclpy.init()
    node = Node("verify_school_stack")
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    counts, latest, subscriptions = {}, {}, []

    def subscribe(topic, kind, qos):
        def receive(msg):
            counts[topic] = counts.get(topic, 0) + 1
            latest[topic] = msg

        subscriptions.append(node.create_subscription(kind, topic, receive, qos))

    for topic, kind in [
        ("/clock", Clock),
        ("/camera/image_raw", Image),
        ("/camera/camera_info", CameraInfo),
        ("/camera/depth/image_raw", Image),
        ("/camera/depth/points", PointCloud2),
    ]:
        subscribe(topic, kind, qos_profile_sensor_data)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscribe("/school/scene", MarkerArray, latched)
    subscribe("/school/planned_route", Path, latched)
    deadline = time.monotonic() + 120
    status_topic = None
    required = list(subscriptions)
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if status_topic is None:
                for topic, types in node.get_topic_names_and_types():
                    if topic == "/fmu/out/vehicle_status" or topic.startswith(
                        "/fmu/out/vehicle_status_v"
                    ):
                        status_topic = topic
                        subscribe(topic, get_message(types[0]), qos_profile_sensor_data)
                        break
            if (
                len(counts) >= len(required) + 1
                and all(n >= 2 for n in counts.values())
                and buffer.can_transform("map", "base_link", rclpy.time.Time())
            ):
                break
        assert status_topic is not None, "PX4 vehicle_status was not discovered"
        expected = [
            "/clock",
            "/camera/image_raw",
            "/camera/camera_info",
            "/camera/depth/image_raw",
            "/camera/depth/points",
            "/school/scene",
            "/school/planned_route",
            status_topic,
        ]
        missing = [t for t in expected if counts.get(t, 0) < 2]
        assert not missing, f"missing repeated messages: {missing}"
        assert node.count_publishers("/clock") == 1, "duplicate simulation clocks"
        image, info, depth, points = [latest[t] for t in expected[1:5]]
        assert image.width == info.width and image.height == info.height, (
            "RGB calibration dimensions disagree"
        )
        assert image.width > 0 and len(image.data) == image.step * image.height
        assert (
            depth.encoding == "32FC1" and len(depth.data) == depth.step * depth.height
        )
        assert points.width * points.height > 0 and len(points.data) > 0
        status = latest[status_topic]
        assert status.timestamp > 0 and status.arming_state != 2, (
            "expected a disarmed PX4"
        )
        markers = latest["/school/scene"].markers
        assert {0, 1, 300, 301, 400}.issubset({m.id for m in markers}), (
            "missing school/drone/window/route markers"
        )
        drone = next(m for m in markers if m.id == 1)
        assert buffer.can_transform(
            "map", "base_link", rclpy.time.Time.from_msg(drone.header.stamp)
        ), "drone marker timestamp has no matching TF"
        route = latest["/school/planned_route"]
        assert route.header.frame_id == "map" and len(route.poses) >= 2
        assert all(
            math.isfinite(v)
            for p in route.poses
            for v in (p.pose.position.x, p.pose.position.y, p.pose.position.z)
        )
        transform = buffer.lookup_transform(
            "map", "base_link", rclpy.time.Time(), timeout=Duration(seconds=1)
        )
        p = transform.transform.translation
        assert all(math.isfinite(v) for v in (p.x, p.y, p.z))
        print(
            json.dumps(
                {
                    "status": "passed",
                    "messages": counts,
                    "rgb": [image.width, image.height],
                    "depth": [depth.width, depth.height],
                    "points": points.width * points.height,
                    "px4_status_topic": status_topic,
                    "px4_disarmed": True,
                    "map_to_base_link": [p.x, p.y, p.z],
                    "route_waypoints": len(route.poses),
                    "scene_markers": len(markers),
                    "physical_flight": False,
                },
                indent=2,
            )
        )
    finally:
        del listener
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
