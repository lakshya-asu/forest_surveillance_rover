from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    description_launch = PathJoinSubstitution(
        [FindPackageShare("forest_rover_description"), "launch", "description.launch.py"]
    )
    bridge_launch = PathJoinSubstitution(
        [FindPackageShare("stm32_firmware_driver"), "launch", "stm32_bridge.launch.py"]
    )
    autonomy_launch = PathJoinSubstitution(
        [FindPackageShare("autonomy_manager"), "launch", "phase2_autonomy.launch.py"]
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(description_launch)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(bridge_launch)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(autonomy_launch)),
        ]
    )
