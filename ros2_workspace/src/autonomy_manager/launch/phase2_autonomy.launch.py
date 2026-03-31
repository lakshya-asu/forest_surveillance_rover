import os
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("autonomy_manager")
    pkg_prefix = get_package_prefix("autonomy_manager")
    libexec_dir = os.path.join(pkg_prefix, "lib", "autonomy_manager")
    ekf_cfg = os.path.join(pkg_share, "config", "ekf.yaml")
    smooth_cfg = os.path.join(pkg_share, "config", "cmd_vel_smoother.yaml")
    use_ekf = LaunchConfiguration("use_ekf")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_ekf",
                default_value="false",
                description="Enable robot_localization EKF node",
            ),
            ExecuteProcess(
                cmd=[sys.executable, os.path.join(libexec_dir, "odometry_publisher_node.py")],
                output="screen",
            ),
            ExecuteProcess(
                cmd=[
                    sys.executable,
                    os.path.join(libexec_dir, "cmd_vel_smoother_node.py"),
                    "--ros-args",
                    "--params-file",
                    smooth_cfg,
                ],
                output="screen",
            ),
            ExecuteProcess(
                cmd=[sys.executable, os.path.join(libexec_dir, "patrol_manager_node.py")],
                output="screen",
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[ekf_cfg],
                condition=IfCondition(use_ekf),
            ),
        ]
    )
