"""Launch base station receiver node."""

import os
import sys
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    #Try multiple possible locations for the base_station_receiver_main.py script
    src_path = Path(__file__).parent.parent.parent / "base_station_receiver_node" / "base_station_receiver_node" / "base_station_receiver_main.py"
    
    # Fallback to installed location
    if not src_path.exists():
        from ament_index_python.packages import get_package_prefix
        pkg_prefix = get_package_prefix("base_station_receiver_node")
        src_path = Path(pkg_prefix) / "lib" / "base_station_receiver_node" / "base_station_receiver_main.py"
    
    if not src_path.exists():
        raise RuntimeError(f"Cannot find base_station_receiver_main.py at {src_path}")
    
    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=[
                    sys.executable,
                    str(src_path),
                ],
                output="screen",
            )
        ]
    )
