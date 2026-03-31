import os
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description() -> LaunchDescription:
    pkg_prefix = get_package_prefix("data_logger_node")
    libexec_dir = os.path.join(pkg_prefix, "lib", "data_logger_node")
    config_path = os.path.join(
        get_package_share_directory("data_logger_node"),
        "config",
        "data_logger.yaml",
    )
    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=[
                    sys.executable,
                    os.path.join(libexec_dir, "data_logger.py"),
                    "--ros-args",
                    "--params-file",
                    config_path,
                ],
                output="screen",
            )
        ]
    )
