from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="nv_telemetry",
                executable="telemetry",
                name="nv_telemetry",
                output="screen",
            ),
        ]
    )
