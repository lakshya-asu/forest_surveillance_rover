from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    yolo_launch = PathJoinSubstitution(
        [FindPackageShare("yolo_detector_node"), "launch", "yolo_detector.launch.py"]
    )
    color_launch = PathJoinSubstitution(
        [FindPackageShare("color_tracker_node"), "launch", "color_tracker.launch.py"]
    )
    smoke_launch = PathJoinSubstitution(
        [FindPackageShare("smoke_detection_node"), "launch", "smoke_detection.launch.py"]
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(yolo_launch)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(color_launch)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(smoke_launch)),
        ]
    )
