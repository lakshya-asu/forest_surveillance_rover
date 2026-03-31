# Deployment and Operations Guide

## 1. Build and Source

```bash
cd ros2_workspace
colcon build --symlink-install
source install/setup.bash
```

## 2. Full System Bringup

```bash
ros2 launch forest_rover_description complete.launch.xml
```

This launch starts:

- robot state publisher and TF model,
- STM32 bridge (simulation mode by default),
- autonomy manager stack (odometry, smoother, patrol),
- YOLO detection, red-ball tracking, smoke detection,
- mission data logger.

## 3. Key Runtime Topics

- `/odometry/raw`
- `/environmental/data`
- `/perception/detections`
- `/perception/ball_centroid`
- `/alerts/smoke_detected`
- `/events/rover`

## 4. Safety Controls

Emergency stop service:

```bash
ros2 service call /emergency_stop forest_rover_msgs/srv/EmergencyStop "{stop: true, reason: 'operator'}"
```

Clear emergency stop:

```bash
ros2 service call /emergency_stop forest_rover_msgs/srv/EmergencyStop "{stop: false, reason: 'clear'}"
```

## 5. Logging and Analysis

Mission telemetry is written to SQLite:

- default path: `mission_logs/mission_events.db`

Run post-mission summary:

```bash
python3 install/data_logger_node/lib/data_logger_node/analyze_mission.py mission_logs/mission_events.db
```

## 6. Troubleshooting

1. No perception output:
- verify camera topic exists: `ros2 topic list | grep camera`
- keep YOLO in simulation mode until model/runtime is available.

2. No STM32 data:
- check bridge parameters in `stm32_firmware_driver/config/bridge.yaml`
- verify serial device and baudrate.

3. Launch fails on optional packages:
- install missing ROS dependencies in Humble environment,
- re-run `colcon build --symlink-install`.

4. No logs generated:
- ensure process has write permissions to `mission_logs/`.

## 7. Field Checklist

- battery voltage above threshold,
- emergency-stop call tested,
- odometry and TF visible,
- smoke alert pipeline tested with synthetic values,
- mission logging active before deployment.
