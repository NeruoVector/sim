"""PX4 텔레메트리 요약 노드.

PX4 는 uXRCE-DDS 를 통해 `/fmu/out/*` 토픽으로 상태를 발행하고,
publish QoS 는 best-effort 입니다. 그래서 구독도 best-effort 로 맞춥니다.
"""

from __future__ import annotations

import rclpy
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

# VehicleStatus.nav_state 값 중 사람이 읽기 위한 최소 매핑
NAV_STATE_NAMES = {
    0: "MANUAL",
    1: "ALTCTL",
    2: "POSCTL",
    3: "AUTO_MISSION",
    4: "AUTO_LOITER",
    5: "AUTO_RTL",
    14: "OFFBOARD",
}


class TelemetryNode(Node):
    def __init__(self) -> None:
        super().__init__("nv_telemetry")

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self._armed = False
        self._nav_state = 0
        self._position = (0.0, 0.0, 0.0)

        self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status", self._on_status, qos
        )
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self._on_local_position,
            qos,
        )

        self.create_timer(2.0, self._report)
        self.get_logger().info("nv_telemetry 시작 (PX4 토픽 대기 중)")

    def _on_status(self, msg: VehicleStatus) -> None:
        self._armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self._nav_state = msg.nav_state

    def _on_local_position(self, msg: VehicleLocalPosition) -> None:
        self._position = (msg.x, msg.y, msg.z)

    def _report(self) -> None:
        x, y, z = self._position
        self.get_logger().info(
            f"armed={self._armed} "
            f"nav_state={NAV_STATE_NAMES.get(self._nav_state, self._nav_state)} "
            f"pos=({x:.2f}, {y:.2f}, {z:.2f})"
        )


def main() -> None:
    rclpy.init()
    node = TelemetryNode()
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
