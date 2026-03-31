"""Launch telemetry gateway node."""

import os
import sys
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    # Try multiple possible locations for the telemetry_gateway_main.py script
    src_path = Path(__file__).parent.parent.parent / "telemetry_gateway_node" / "telemetry_gateway_node" / "telemetry_gateway_main.py"
    
    # Fallback to installed location
    if not src_path.exists():
        from ament_index_python.packages import get_package_prefix
        pkg_prefix = get_package_prefix("telemetry_gateway_node")
        src_path = Path(pkg_prefix) / "lib" / "telemetry_gateway_node" / "telemetry_gateway_main.py"
    
    if not src_path.exists():
        raise RuntimeError(f"Cannot find telemetry_gateway_main.py at {src_path}")
    
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
