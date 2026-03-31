# ROS 2 Architecture - Forest Surveillance Rover

**Status**: Phase 0 Infrastructure Complete  
**ROS Distribution**: Humble (LTS until May 2027)  
**Target Platform**: Raspberry Pi CM4 with Ubuntu 22.04

---

## System Overview

The ROS 2 stack on Raspberry Pi CM4 provides high-level autonomy, sensor fusion, perception, and decision-making for the Forest Surveillance Rover.

```
┌─────────────────────────────────────────────────────┐
│   ROS 2 Humble (Raspberry Pi CM4)                   │
│   port 11311, ROS_DOMAIN_ID=1                       │
├─────────────────────────────────────────────────────┤
│                                                     │
│  • nav2 stack (autonomy + navigation)              │
│  • YOLOv8 detection pipeline (~15-20 FPS)          │
│  • Color tracking (red ball pursuit)                │
│  • Sensor fusion (IMU + odometry)                   │
│  • Data logging (rosbag2)                           │
│  • Autonomy decision state machine                  │
│                                                     │
└──────────────┬────────────────────────────────────┬─┘
               │ UART 115200 baud                   │
               ↓                                    ↓
    ┌──────────────────────┐          ┌──────────────────┐
    │ STM32F407 Firmware   │          │ USB Camera       │
    │ (Real-Time)          │          │ (V4L2)           │
    └──────────────────────┘          └──────────────────┘
```

---

## 1. Package Hierarchy

```
forest_rover_core (meta-package)
├── forest_rover_hardware (meta-package)
│   ├── stm32_firmware_driver (node)
│   ├── camera_driver (node) [TODO: Phase 2]
│   └── sensor_fusion_node (node) [TODO: Phase 2]
│
├── forest_rover_perception (meta-package)
│   ├── yolo_detector_node (node) [TODO: Phase 3]
│   └── color_tracker_node (node) [TODO: Phase 3]
│
├── forest_rover_autonomy (meta-package)
│   └── autonomy_manager (node) [TODO: Phase 2+3]
│
├── forest_rover_description
│   ├── urdf/ (robot model)
│   ├── launch/ (launch files)
│   └── config/ (TF static configs)
│
├── forest_rover_msgs (message definitions)
│   ├── msg/ (custom messages)
│   └── srv/ (custom services)
│
└── forest_rover_utils (tooling)
    ├── diagnostic_aggregator
    ├── data_logger
    ├── motor_tuner
    └── calibration_tools
```

---

## 2. Topic & Service Map

### Published Topics

| Topic | Type | Source | Frequency | Purpose |
|-------|------|--------|-----------|---------|
| `/camera/image_raw` | sensor_msgs/Image | camera_driver | 30 Hz | USB camera stream |
| `/imu/data` | sensor_msgs/Imu | stm32_firmware_driver | 100 Hz | 6-DOF IMU (BNO055) |
| `/environmental/data` | sensor_msgs/EnvironmentalData | stm32_firmware_driver | 50 Hz | Pressure, temp, humidity (BME280) |
| `/gas_sensor/reading` | forest_rover_msgs/GasSensorData | stm32_firmware_driver | 10 Hz | MQ-2 smoke sensor |
| `/raw_odom` | nav_msgs/Odometry | sensor_fusion_node | 100 Hz | Raw wheel odometry |
| `/odometry/filtered` | nav_msgs/Odometry | sensor_fusion_node | 30 Hz | EKF-fused odometry |
| `/tf` | tf2_msgs/TFMessage | sensor_fusion_node | 30 Hz | Transform frames (odom → base_link) |
| `/perception/detections` | forest_rover_msgs/DetectionEvent[] | yolo_detector_node | 15-20 FPS | YOLOv8 animal detections |
| `/perception/ball_centroid` | geometry_msgs/PointStamped | color_tracker_node | 30 Hz | Red ball 2D position |
| `/cmd_vel` | geometry_msgs/Twist | autonomy_manager → nav2 | ~10 Hz | Linear + angular velocity command |
| `/autonomy/state` | std_msgs/String | autonomy_manager | 10 Hz | Current behavior state (patrol/track/alert) |
| `/autonomy/event_log` | rosgraph_msgs/Log | autonomy_manager | Event-driven | Mission events for logging |

### Subscribed Topics

| Topic | Consumer | Type | Purpose |
|-------|----------|------|---------|
| `/camera/image_raw` | yolo_detector, color_tracker | sensor_msgs/Image | Vision processing |
| `/imu/data` | sensor_fusion_node, autonomy_manager | sensor_msgs/Imu | State estimation + tilt compensation |
| `/raw_odom` | sensor_fusion_node | nav_msgs/Odometry | EKF input |
| `/odometry/filtered` | nav2, autonomy_manager | nav_msgs/Odometry | Navigation feedback |
| `/perception/detections` | autonomy_manager, data_logger | forest_rover_msgs/DetectionEvent[] | Detection-based decisions |
| `/perception/ball_centroid` | autonomy_manager | geometry_msgs/PointStamped | Red ball tracking |

### Services

| Service | Type | Server | Client | Purpose |
|---------|------|--------|--------|---------|
| `/motor_command` | forest_rover_msgs/SetMotorSpeed | stm32_firmware_driver | nav2_controller | Set motor speeds |
| `/emergency_stop` | std_srvs/Empty | stm32_firmware_driver | autonomy_manager | E-stop trigger |
| `/reset_odometry` | std_srvs/Empty | sensor_fusion_node | calibration_tools | Reset pose estimate |

---

## 3. ROS 2 Humble Configuration

### Environment Variables

```bash
# Set ROS2 distribution
export ROS_DISTRO=humble

# Domain ID (isolates network traffic)
export ROS_DOMAIN_ID=1

# Middleware selection (CycloneDDS optimized for edge)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Performance tuning for RPi CM4
export RMW_QOS_PROFILE_PARAMETER_EVENTS__MAX_SAMPLES=10
export RMW_QOS_PROFILE_ROSOUT__MAX_SAMPLES=10
```

### DDS Configuration (CycloneDDS)

Create `cyclon-dds.xml` for network optimization:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://github.com/eclipse-cyclonedds/cyclonedds/releases/tag/0.10.0/cyclonedds.xsd">
  <Domain id="1">
    <General>
      <!-- Disable multicast, use UDP unicast for reliability -->
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
    </General>
    <Discovery>
      <!-- Reduce discovery overhead on RPi -->
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
```

### colcon Build Optimization

`.colcon-defaults.yaml`:
- Symlink install (no copying, faster iteration)
- Release build type (optimal performance)
- Parallel build (leverage multi-core)

---

## 4. Hardware Bridge (STM32 ↔ ROS2)

### UART Protocol

| Component | Role |
|-----------|------|
| **Baud Rate** | 115200 bps |
| **Frame Format** | COBS (Consistent Overhead Byte Stuffing) |
| **Checksum** | CRC-16 (CCITT) |
| **Timeout** | 500ms (motor watchdog) |

### Message Structure (STM32 → ROS)

```
[SYNC][MESSAGE_TYPE][LENGTH][PAYLOAD][CRC-16][END]
 0xFF   1 byte         2 bytes   var    2 bytes 0xFE
```

### Example: Sensor Data Frame

```c
typedef struct {
  uint8_t sync;           // 0xFF
  uint8_t msg_type;       // 0x10 (sensor data)
  uint16_t length;        // payload size
  
  // Payload
  int32_t encoder_a;      // counts
  int32_t encoder_b;
  float imu_accel[3];     // m/s²
  float imu_gyro[3];      // rad/s
  float bme280_temp;      // °C
  float bme280_pres;      // Pa
  float bme280_hum;       // %
  uint16_t mq2_adc;       // raw ADC
  
  uint16_t crc16;         // checksum
  uint8_t end;            // 0xFE
} SensorDataFrame_t;
```

---

## 5. Transform Tree (TF)

```
map
  └── odom (published by nav2/SLAM)
        └── base_link (published by sensor_fusion_node)
              ├── base_footprint
              ├── camera (depth center)
              ├── imu_link (BNO055)
              └── laser_link (if LiDAR added in Phase 2)
```

### Transform Publication Frequency

- `odom → base_link`: 30 Hz (from robot_localization EKF)
- `map → odom`: ~1-10 Hz (from nav2 SLAM, if enabled; otherwise static)
- Static transforms: Published once at startup by `tf2_ros StaticTransformBroadcaster`

---

## 6. Launch Files

### `rover.launch.xml` - Hardware Stack
- Loads robot description (URDF)
- Starts stm32_firmware_driver
- Starts sensor_fusion_node
- Launches camera driver
- Publishes static TF frames

### `autonomy.launch.xml` - Autonomy Stack
- Includes rover.launch.xml
- Starts nav2 stack
- Starts autonomy_manager
- Loads nav2 config (costmap, planner)

### `perception.launch.xml` - Vision Stack  
- Starts yolo_detector_node
- Starts color_tracker_node
- Loads YOLOv8 model weights

### `complete.launch.xml` - All-in-One
- Includes rover + autonomy + perception
- Starts data_logger (rosbag2)
- Ready for autonomous missions

---

## 7. Data Logging (rosbag2)

### Recording Configuration

```python
# Auto-record all topics except large image streams
rosbag2 record \
  /imu/data \
  /odometry/filtered \
  /perception/detections \
  /autonomy/state \
  /gas_sensor/reading \
  /cmd_vel \
  --compression-mode file --compression-format zstd
```

### Post-Mission Analysis

```bash
# Play back with visualization
ros2 bag convert input.db3 --output-format cdr
rosbag2 play input.db3 --rate 0.1

# Extract specific topic
rosbag2 export input.db3 --output-format csv --topics /perception/detections
```

---

## 8. Performance Targets

| Metric | Target | Notes |
|--------|--------|-------|
| **Motor Control Loop** | 100 Hz (10ms) | STM32 real-time |
| **Sensor Publish Rate** | 50-100 Hz | I2C → UART → ROS |
| **YOLOv8 Inference** | ≥15 FPS | YOLOv8n model |
| **EKF Fusion** | 30 Hz | robot_localization |
| **nav2 Costmap Update** | 10-20 Hz | Local costmap |
| **Odometry Error (100m)** | <5% | Calibration critical |
| **CPU Usage (idle)** | <30% | RPi CM4 compute budget |
| **Memory Usage** | <60 MB | roslaunch footprint |

---

## 9. Development Workflow

### Build & Test

```bash
cd ros2_workspace

# Build all packages
colcon build --symlink-install --event-handlers console_direct+

# Test single package
colcon test --packages-select stm32_firmware_driver

# Clean everything
colcon clean all --remove-build
```

### Debug with ROS2 Command Line

```bash
# List all nodes
ros2 node list

# List all topics
ros2 topic list

# Echo a topic
ros2 topic echo /odometry/filtered

# Call a service
ros2 service call /emergency_stop std_srvs/srv/Empty

# Check node info
ros2 node info /autonomy_manager
```

### Visualize with rviz2

```bash
# Launch rviz2 with saved config
rviz2 -d forest_rover_description/config/rviz_config.rviz

# Load robot model + odometry visualization
ros2 launch forest_rover_description rviz.launch.xml
```

---

## 10. Debugging Tips

### UART Communication Issues

- Check baud rate: `stty -F /dev/ttyUSB0 115200`
- Monitor serial: `minicom -D /dev/ttyUSB0`
- Verify COBS framing: Check firmware logs with OpenOCD debugger

### Laggy Perception

- Profile YOLOv8: Turn on timing in detector node
- Check CPU usage: `top`, `htop`
- Reduce image resolution (from 640x480 to 416x416)
- Use YOLOv8n instead of YOLOv8s

### Odometry Drift

- Calibrate wheel radius on 10m test track
- Record raw encoder counts vs. actual distance
- Adjust `wheel_radius` in robot_localization config
- Enable IMU heading fusion to reduce rotation drift

---

## 11. Phase 0 Completion Checklist

- [x] ROS2 workspace structure created
- [x] All packages can build (colcon build succeeds)
- [x] CMakeLists.txt and package.xml files correct
- [x] URDF skeleton created
- [x] TF frame plan documented
- [x] stm32_firmware_driver interface designed
- [x] Docker ARM64 build environment configured
- [x] GitHub Actions CI/CD pipeline setup
- [ ] First UART test with hardware (Phase 1.1)

---

**Next**: Phase 1 will implement actual node code and begin Phase 1.1 STM32 firmware core + UART bridge.
