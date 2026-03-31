import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bridge_config = os.path.join(
        get_package_share_directory("stm32_firmware_driver"),
        "config",
        "bridge.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="stm32_firmware_driver",
                executable="stm32_bridge_node",
                name="stm32_firmware_bridge",
                output="screen",
                parameters=[bridge_config],
            )
        ]
    )
