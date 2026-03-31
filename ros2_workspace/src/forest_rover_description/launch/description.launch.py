import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("forest_rover_description")
    urdf_file = os.path.join(pkg_share, "urdf", "forest_rover.urdf.xacro")

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": Command([FindExecutable(name="xacro"), " ", urdf_file])}],
            )
        ]
    )
