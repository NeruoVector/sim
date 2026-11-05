# /// script
# requires-python = ">=3.12"
# dependencies = []
# ///
# Run inside nv-sim: python3 /scripts/school-live-node.py
"""Export real Gazebo frames/state and fly an isolated PX4 SITL inspection loop."""

from __future__ import annotations

import json
import math
import os
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from school_flight_control import FlightControl, OrderAssignment
from school_mission import InvalidMission, Mission, MissionPlan, Point, enu_to_ned
from sensor_msgs.msg import Image

ROOT = Path("/runtime")


def atomic_write(path: Path, content: bytes) -> None:
    temp = path.with_suffix(".tmp")
    temp.write_bytes(content)
    temp.replace(path)


class SchoolLive(Node):
    """Own mutable sensor snapshots and a bounded, simulation-only flight controller."""

    def __init__(self) -> None:
        super().__init__("school_live")
        if (
            os.environ.get("ROS_DOMAIN_ID") != "73"
            or os.environ.get("NV_SCHOOL_SCENE") != "1"
        ):
            raise RuntimeError(
                "This controller requires the isolated school SITL domain"
            )
        ROOT.mkdir(exist_ok=True)
        self.local: VehicleLocalPosition | None = None
        self.status: VehicleStatus | None = None
        self.pose: Odometry | None = None
        self.last_frame = 0.0
        self.last_pose = 0.0
        self.started = time.monotonic()
        self.tick = 0
        self.waypoint = 0
        self.hold: tuple[float, float, float] | None = None
        self.mode = "waiting_for_sensors"
        self.flight_control = FlightControl()
        try:
            previous = json.loads((ROOT / "state.json").read_text())
            completed = previous.get("completed_order_id")
            if isinstance(completed, str):
                self.flight_control.completed_order_id = completed
        except (OSError, ValueError, TypeError):
            self.get_logger().info("Starting without a prior completed order")
        self.mission: Mission | None = None
        self.mission_authorized = False
        self.ned_offset: Point | None = None
        self.reference: Point | None = None
        self.last_control_time = time.monotonic()
        self.last_phase: str | None = None
        self.route_error: tuple[str, str | None] | None = None
        self.setpoint = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10
        )
        self.offboard = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", 10
        )
        self.commands = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", 10
        )
        self.create_subscription(
            Image, "/camera/image_raw", self.on_image, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, "/sim/odometry", self.on_pose, qos_profile_sensor_data
        )
        self.create_subscription(
            VehicleLocalPosition,
            f"/fmu/out/vehicle_local_position_v{VehicleLocalPosition.MESSAGE_VERSION}",
            self.on_local,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VehicleStatus,
            f"/fmu/out/vehicle_status_v{VehicleStatus.MESSAGE_VERSION}",
            self.on_status,
            qos_profile_sensor_data,
        )
        self.create_timer(0.1, self.control)
        self.create_timer(1.0, self.export)

    def on_local(self, message: VehicleLocalPosition) -> None:
        self.local = message

    def on_status(self, message: VehicleStatus) -> None:
        self.status = message

    def on_pose(self, message: Odometry) -> None:
        self.pose = message
        self.last_pose = time.monotonic()

    def on_image(self, message: Image) -> None:
        now = time.time()
        if now - self.last_frame < 0.5:
            return
        if message.encoding not in ("rgb8", "bgr8"):
            self.get_logger().error(
                "Unsupported RGB camera encoding: " + message.encoding
            )
            return
        pixels = np.frombuffer(bytes(message.data), dtype=np.uint8).reshape(
            message.height, message.step
        )
        image = pixels[:, : message.width * 3].reshape(message.height, message.width, 3)
        if message.encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        ok, encoded = cv2.imencode(".jpg", image)
        if ok:
            atomic_write(ROOT / "camera.jpg", encoded.tobytes())
            self.last_frame = now

    def command(self, command: int, param1: float, param2: float = 0.0) -> None:
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.command, msg.param1, msg.param2 = command, param1, param2
        msg.target_system, msg.target_component = 1, 1
        msg.source_system, msg.source_component, msg.from_external = 1, 1, True
        self.commands.publish(msg)

    def control(self) -> None:
        self.tick += 1
        self.mission_authorized = False
        if (
            self.local is None
            or self.status is None
            or not self.local.xy_valid
            or not self.local.z_valid
        ):
            return
        try:
            vision = json.loads((ROOT / "inference.json").read_text())
            fresh = 0 <= time.time() - vision["timestamp"] < 8 and vision["ok"]
            blocked = vision["blocked"]
        except (OSError, ValueError, KeyError):
            fresh, blocked = False, True
        try:
            assignment = OrderAssignment.read(ROOT / "order-assignment.json")
        except (OSError, ValueError, KeyError, TypeError):
            assignment = None
        can_fly = self.flight_control.update(
            assignment, fresh=fresh, blocked=blocked, now=time.time()
        )
        armed = self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.mode = self.flight_control.mode
        self.mission_authorized = False
        if self.pose is None or time.monotonic() - self.last_pose > 3:
            self.mode = "hold_position_unavailable"
            return
        p = self.pose.pose.pose.position
        position = (p.x, p.y, p.z)
        measured_ned = enu_to_ned(position)
        if self.ned_offset is None:
            if armed:
                self.mode = "hold_origin_unavailable"
                return
            self.ned_offset = tuple(
                a - b
                for a, b in zip(
                    (self.local.x, self.local.y, self.local.z),
                    measured_ned,
                    strict=True,
                )
            )
        if can_fly and assignment is not None:
            replace = (
                self.mission is None or self.mission.order_id != assignment.order_id
            )
            if replace and armed:
                can_fly = False
                self.mode = "hold_order_changed"
            elif replace:
                try:
                    plan = MissionPlan.read(
                        Path("/fixtures/school.json"), assignment.destination_id
                    )
                    home = plan.route[-1]
                    if (
                        math.dist(position[:2], home[:2]) > 1.5
                        or abs(position[2] - home[2]) > 0.7
                    ):
                        raise InvalidMission(
                            "Aircraft must start disarmed on the launch pad"
                        )
                    self.mission = Mission(str(assignment.order_id), plan)
                    self.reference = None
                    self.route_error = None
                except InvalidMission as error:
                    can_fly = False
                    self.mode = "hold_route_unavailable"
                    key = (str(assignment.order_id), assignment.destination_id)
                    if self.route_error != key:
                        self.get_logger().error(
                            f"Mission unavailable for {key}: {error}"
                        )
                        self.route_error = key
            elif self.mission.plan.destination_id != assignment.destination_id:
                can_fly = False
                self.mode = "hold_route_unavailable"
        now = time.monotonic()
        dt = max(0, min(now - self.last_control_time, 0.3))
        self.last_control_time = now
        if self.mission is not None and self.mission.complete_landing(
            position, armed=armed
        ):
            self.flight_control.order_id = self.mission.order_id
            self.flight_control.complete()
            self.mode = "order_completed"
            can_fly = False
        self.mission_authorized = can_fly
        if self.mission is not None:
            self.mission.step(position, armed=armed, authorized=can_fly, now=now)
            self.waypoint = self.mission.waypoint_index
            if self.last_phase != self.mission.phase:
                self.get_logger().info(
                    f"Mission {self.mission.order_id}: {self.mission.phase}, waypoint {self.waypoint}"
                )
                self.last_phase = self.mission.phase
            if self.mission.phase == "landing" and armed:
                # Landing was authorized only after arrival and measured return to home.
                self.mode = "landing"
                if self.tick % 10 == 0:
                    self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND, 0.0)
                return
        if not can_fly and not armed:
            return
        stamp = self.get_clock().now().nanoseconds // 1000
        msg = OffboardControlMode()
        msg.timestamp, msg.position = stamp, True
        self.offboard.publish(msg)
        if not can_fly or self.mission is None:
            if self.hold is None:
                self.hold = (self.local.x, self.local.y, self.local.z)
            target = self.hold
            self.reference = None
        else:
            self.hold = None
            if self.reference is None or not armed:
                self.reference = position
            desired = self.mission.target
            distance = math.dist(self.reference, desired)
            fraction = (
                min(1, self.mission.plan.speed_mps * dt / distance) if distance else 1
            )
            self.reference = tuple(
                a + fraction * (b - a)
                for a, b in zip(self.reference, desired, strict=True)
            )
            target = tuple(
                a + b
                for a, b in zip(
                    enu_to_ned(self.reference), self.ned_offset, strict=True
                )
            )
        trajectory = TrajectorySetpoint()
        trajectory.timestamp, trajectory.position = stamp, list(target)
        trajectory.velocity = [math.nan] * 3
        trajectory.acceleration = [math.nan] * 3
        trajectory.yaw = 0.0
        self.setpoint.publish(trajectory)
        if self.tick % 30 == 0 and time.monotonic() - self.started > 30 and can_fly:
            self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            if not armed:
                self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def export(self) -> None:
        xyz = None
        if self.pose is not None:
            p = self.pose.pose.pose.position
            xyz = [p.x, p.y, p.z]
        atomic_write(
            ROOT / "state.json",
            json.dumps(
                {
                    "timestamp": time.time(),
                    "camera_timestamp": self.last_frame,
                    "source": "Gazebo Harmonic + PX4 SITL",
                    "drone_id": "school-drone-001",
                    "position_enu": xyz,
                    "mode": self.mode,
                    "waypoint": self.waypoint,
                    "controller_id": self.flight_control.controller_id,
                    "order_id": self.flight_control.order_id,
                    "completed_order_id": self.flight_control.completed_order_id,
                    "mission": self.mission.snapshot(
                        tuple(xyz), authorized=self.mission_authorized
                    )
                    if self.mission is not None and xyz is not None
                    else None,
                    "armed": self.status is not None
                    and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED,
                }
            ).encode(),
        )


def main() -> None:
    rclpy.init()
    node = SchoolLive()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
