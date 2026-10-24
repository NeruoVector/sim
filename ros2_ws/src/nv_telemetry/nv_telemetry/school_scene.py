"""Publish the school fixture, stored route, and live Gazebo pose; no flight commands."""

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path as FilePath

import rclpy
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


class SchoolScene(Node):
    def __init__(self):
        super().__init__("school_scene")
        self.declare_parameter("mission", "window")
        mission = self.get_parameter("mission").value
        if mission not in ("window", "outdoor"):
            raise ValueError("mission must be window or outdoor")
        self.fixture = json.loads(FilePath("/fixtures/school.json").read_text())
        if self.fixture.get("synthetic") is not True:
            raise ValueError("school scene requires the synthetic fixture")
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers = self.create_publisher(MarkerArray, "/school/scene", qos)
        self.route = self.create_publisher(Path, "/school/planned_route", qos)
        self.tf = TransformBroadcaster(self)
        self.live_drone = None
        base_sdf = ET.parse("/models/drones/x500_base/model.sdf")
        base_pose = [float(v) for v in base_sdf.findtext("model/pose").split()]
        if len(base_pose) != 6 or any(base_pose[3:]):
            raise ValueError(
                "base model pose changed; update the visualization transform"
            )
        self.base_offset = base_pose[:3]
        self.create_subscription(
            Odometry, "/sim/odometry", self.on_pose, qos_profile_sensor_data
        )
        folder = FilePath("/school-results/school-tests")
        offline = "school-window" if mission == "window" else "school-detour"
        paths = [folder / f"school-api-{mission}.json", folder / f"{offline}.json"]
        self.plan = next(
            (json.loads(p.read_text())["payload"] for p in paths if p.is_file()), None
        )
        if self.plan is None:
            self.get_logger().warning(
                "No stored route; run make school-test and restart this node"
            )
        self.scene, self.planned_path = self.build_scene()
        self.create_timer(1.0, self.publish_scene)

    def marker(self, identity, kind, xyz, size, color):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type = "school", identity, kind
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, xyz)
        m.scale.x, m.scale.y, m.scale.z = map(float, size)
        m.color.r, m.color.g, m.color.b, m.color.a = map(float, color)
        return m

    def on_pose(self, msg):
        p = msg.pose.pose.position
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            return
        root = TransformStamped()
        root.header = msg.header
        root.header.frame_id = "map"
        root.child_frame_id = "x500_depth_0"
        root.transform.translation.x = p.x
        root.transform.translation.y = p.y
        root.transform.translation.z = p.z
        root.transform.rotation = msg.pose.pose.orientation
        base = TransformStamped()
        base.header.stamp = msg.header.stamp
        base.header.frame_id, base.child_frame_id = "x500_depth_0", "base_link"
        (
            base.transform.translation.x,
            base.transform.translation.y,
            base.transform.translation.z,
        ) = self.base_offset
        base.transform.rotation.w = 1.0
        self.tf.sendTransform([root, base])
        m = self.marker(
            1, Marker.MESH_RESOURCE, [0, 0, 0], [0.01] * 3, [0.85, 0.87, 0.9, 1]
        )
        m.header.frame_id = "base_link"
        m.header.stamp = msg.header.stamp
        m.mesh_resource = "file:///models/drones/x500_base/meshes/drone.stl"
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (
            -0.00257,
            0.00702,
            -0.25,
        )
        self.live_drone = m

    def geo(self, w):
        origin = self.fixture["origin"]
        r = 6371000.0
        return Point(
            x=math.radians(w["lon"] - origin["lon"])
            * r
            * math.cos(math.radians(origin["lat"])),
            y=math.radians(w["lat"] - origin["lat"]) * r,
            z=float(w["altitude_m"]),
        )

    def build_scene(self) -> tuple[MarkerArray, Path | None]:
        f = self.fixture
        campus = f["campus"]
        m = self.marker(
            0,
            Marker.MESH_RESOURCE,
            campus["mesh_pose_enu"],
            campus["mesh_scale"],
            [0.55, 0.65, 0.75, 1],
        )
        m.mesh_resource = (
            "file:///models/environments/school-demo/meshes/school_map.stl"
        )
        markers = [m]
        # shortcut: dummy no-fly boxes need STL alignment before terrain collision trials.
        for i, b in enumerate(f["buildings"]):
            ceiling = b["no_fly_ceiling_m"]
            markers.append(
                self.marker(
                    10 + i,
                    Marker.CUBE,
                    [*b["center_enu"][:2], ceiling / 2],
                    [*b["size_m"][:2], ceiling],
                    [1, 0.3, 0.2, 0.18],
                )
            )
        for i, o in enumerate(f["obstacles"]):
            color = [(1, 0.2, 0.2, 1), (1, 0.7, 0.1, 1), (0.7, 0.4, 1, 1)][
                o["priority"] - 1
            ]
            markers.append(
                self.marker(
                    100 + i,
                    Marker.SPHERE,
                    o["position_enu"],
                    [max(0.3, o["radius_m"] * 2)] * 3,
                    color,
                )
            )
            label = self.marker(
                200 + i,
                Marker.TEXT_VIEW_FACING,
                [*o["position_enu"][:2], o["position_enu"][2] + 1.5],
                [0, 0, 1],
                color,
            )
            label.text = f"P{o['priority']} {o['id']} (fixture)"
            markers.append(label)
        w = f["window"]
        bearing = math.radians(w["outward_bearing_deg"])
        pos = w["position_enu"]
        hover = [
            pos[0] + math.sin(bearing) * w["standoff_m"],
            pos[1] + math.cos(bearing) * w["standoff_m"],
            pos[2],
        ]
        markers.append(
            self.marker(300, Marker.CUBE, pos, [1, 0.15, 1], [0.3, 0.6, 1, 1])
        )
        markers.append(
            self.marker(301, Marker.SPHERE, hover, [0.7] * 3, [0.3, 0.8, 1, 1])
        )
        path = None
        if self.plan is not None:
            p = self.plan
            selected = p["selected_index"]
            if not p["validations"][selected]["valid"]:
                raise ValueError("stored selected route is invalid")
            path = Path()
            path.header.frame_id = "map"
            path.header.stamp = self.get_clock().now().to_msg()
            for point in p["candidates"][selected]["waypoints"]:
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position = self.geo(point)
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
            line = self.marker(
                400, Marker.LINE_STRIP, [0, 0, 0], [0.2, 0, 0], [0.1, 1, 0.7, 1]
            )
            line.points = [pose.pose.position for pose in path.poses]
            markers.append(line)
        return MarkerArray(markers=markers), path

    def publish_scene(self) -> None:
        stamp = self.get_clock().now().to_msg()
        for marker in self.scene.markers:
            marker.header.stamp = stamp
        if self.planned_path is not None:
            self.planned_path.header.stamp = stamp
            for pose in self.planned_path.poses:
                pose.header.stamp = stamp
            self.route.publish(self.planned_path)
        markers = list(self.scene.markers)
        if self.live_drone is not None:
            markers.append(self.live_drone)
        self.markers.publish(MarkerArray(markers=markers))


def main():
    rclpy.init()
    node = SchoolScene()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
