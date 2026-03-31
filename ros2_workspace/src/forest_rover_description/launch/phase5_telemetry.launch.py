"""
Phase 5 LoRa Telemetry Launch - autonomous rover with long-range telemetry.

Includes:
- Phase 2 autonomy + hardware bridge
- Phase 3 perception (YOLOv8, color tracker, smoke detection)
- Phase 4 data logging
- Phase 5 telemetry gateway (LoRa heartbeat transmission)
- Phase 5 base station receiver (ground station monitoring)
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    desc_dir = get_package_share_directory("forest_rover_description")

    # Include Phase 2 (autonomy + hardware bridge)
    phase2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f"{desc_dir}/launch/phase2_bringup.launch.py"
        ),
    )

    # Include Phase 3 (perception)
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f"{desc_dir}/launch/perception_phase3.launch.py"
        ),
    )

    # Include Phase 4 (logging) launch
    logging_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f"{desc_dir}/launch/phase4_logging.launch.py"
        ),
    )

    # Phase 5: Telemetry Gateway Node
    telemetry_gateway_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f"{desc_dir}/launch/telemetry_gateway.launch.py"
        ),
    )

    # Phase 5: Base Station Receiver
    basestation_receiver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f"{desc_dir}/launch/base_station_receiver.launch.py"
        ),
    )

    return LaunchDescription(
        [
            phase2_launch,
            perception_launch,
            logging_launch,
            telemetry_gateway_launch,
            basestation_receiver_launch,
        ]
    )
