"""
Complete Launch - Phase 0-4 composition for fully integrated rover.

Launches:
- Phase 2: Hardware bridge + autonomy base
- Phase 3: Perception (YOLOv8, color tracker, smoke detection)
- Phase 4: Data logging
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    desc_dir = get_package_share_directory("forest_rover_description")

    # Phase 2: Hardware bridge + autonomy
    phase2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f"{desc_dir}/launch/phase2_bringup.launch.py"
        ),
    )

    # Phase 3: Perception stack
    phase3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f"{desc_dir}/launch/perception_phase3.launch.py"
        ),
    )

    # Phase 4: Data logging
    phase4_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f"{desc_dir}/launch/phase4_logging.launch.py"
        ),
    )

    return LaunchDescription([
        phase2_launch,
        phase3_launch,
        phase4_launch,
    ])
