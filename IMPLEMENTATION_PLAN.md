# Forest Surveillance Rover - Comprehensive Implementation Plan

**Project**: Autonomous Forest Surveillance Rover with ROS-based High-Level Control and STM32 Real-Time Firmware  
**Status**: Planning Phase  
**Last Updated**: 2026-03-31

---

## Executive Overview

Transform the existing Forest Surveillance Rover PCB project into a fully functional autonomous rover system integrating:
- **ROS 2 on Raspberry Pi CM4** for autonomous navigation, sensor fusion, and decision making
- **STM32F407 Real-Time Firmware** for motor control, encoder feedback, and sensor acquisition
- **YOLOv8 Vision Pipeline** for animal detection with transfer learning
- **Fire/Smoke Detection** via integrated gas sensor with alert system
- **Color-Based Tracking** for red ball following and investigation

---

## 1. Requirements Analysis & Scope

### Functional Requirements

| Requirement | Category | Description | Priority |
|------------|----------|-------------|----------|
| **Autonomous Navigation** | Core Autonomy | ROS nav2 stack with SLAM-based or waypoint-based navigation | High |
| **Motor Control** | Real-Time | Closed-loop velocity control with encoder feedback on STM32 | High |
| **Sensor Fusion** | Core Autonomy | IMU + odometry + (optional) LiDAR integration | High |
| **YOLOv8 Detection** | Vision | Real-time animal detection (deer, boar, etc.) with confidence filtering | High |
| **Smoke Detection** | Safety | MQ-2 or similar gas sensor with alert thresholds and logging | Medium |
| **Color Tracking** | Extension | Red ball tracking and pursuit behavior | Medium |
| **LoRa Telemetry** | Communication | Long-range status reporting and emergency alerts (optional for Phase 1) | Low |
| **Data Logging** | Operations | Onboard logging of video, sensor data, detection events for post-mission analysis | Medium |

### Non-Functional Requirements

| Requirement | Specification |
|------------|--------------|
| **Real-Time Constraints** | STM32 motor control loop ≤ 10ms ; ROS sensor publishing ≤ 100ms |
| **Vision Performance** | YOLOv8n or YOLOv8s for ~15-20 FPS edge inference on RPi CM4 |
| **Autonomy Duration** | 2-4 hours with 3S LiPo + cruising motors (no stall) |
| **LoRa Range** | 2+ km line-of-sight (characterization required post-deployment) |
| **Robustness** | Graceful degradation if camera fails; autonomous operation on motor+IMU alone |
| **Safety** | E-stop via wireless or local interrupt; motor timeout safety handlers |

### Scope Boundaries

**✅ In Scope**
- ROS 2 core autonomy and navigation stack
- STM32F407 firmware for motor/sensor control
- YOLOv8 animal detection pipeline
- Smoke sensor integration
- Color-based tracking behavior
- Basic SLAM or waypoint navigation
- Data logging to onboard storage or to external base station over LoRa

**❌ Out of Scope (Phase 1)**
- LiDAR-based SLAM (reserve for Phase 2 if needed; use odometry + IMU + optional camera-based SLAM)
- Arm or manipulation subsystem
- Advanced path planning (A*, RRT*) beyond nav2 defaults
- Distributed multi-robot coordination
- Hardware fabrication/assembly (assumes Rev A is already fabricated)

---

## 2. Technical Architecture

### 2.1 System-Level Block Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                      Forest Surveillance Rover                       │
├─────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │       Raspberry Pi CM4 (High-Level Autonomy & Vision)        │   │
│  ├──────────────────────────────────────────────────────────────┤   │
│  │ • ROS 2 (Humble/Iron) Master                                 │   │
│  │ • nav2 Stack (costmap, planner, behavior tree)               │   │
│  │ • YOLOv8 Animal Detection Node                               │   │
│  │ • Color Tracking Node                                        │   │
│  │ • Autonomy Decision Logic (state machine)                    │   │
│  │ • Data Logger Node (rosbag + custom logging)                 │   │
│  └──────────────────────────────────────────────────────────────┘   │
│                           ↕ UART/I2C/USB                            │
│                                                                       │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │        STM32F407 (Real-Time Embedded Control)                │   │
│  ├──────────────────────────────────────────────────────────────┤   │
│  │ • FreeRTOS or HAL-based scheduler (10ms loop)                │   │
│  │ • Motor Control & PWM (DRV8871 drivers)                      │   │
│  │ • Encoder Feedback & Odometry Calculation                    │   │
│  │ • Sensor Acquisition (BME280, BNO055, MQ-2, analog PIR)      │   │
│  │ • Firmware Bootloader & Flash Manager                        │   │
│  │ • ROS Serial Bridge (rosserial or custom protocol)           │   │
│  └──────────────────────────────────────────────────────────────┘   │
│                                                                       │
│  ┌──────────┬──────────┬──────────┬──────────┬──────────┐            │
│  │ USB Cam  │ BME280   │ BNO055   │ MQ-2 Gas │ Encoder  │            │
│  │ (USB on  │ (I2C)    │ (I2C)    │ (ADC)    │ Inputs   │            │
│  │ RPi)     │          │          │          │          │            │
│  └──────────┴──────────┴──────────┴──────────┴──────────┘            │
│                                                                       │
│  ┌──────────┬──────────┬──────────┬──────────┐                       │
│  │ Motor A  │ Motor B  │ LoRa TX  │ Power M. │                       │
│  │ (DRV)    │ (DRV)    │ (SPI)    │ (LDO/BK) │                       │
│  └──────────┴──────────┴──────────┴──────────┘                       │
│                                                                       │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │              3S LiPo Battery + Solar Charging                 │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                                                                       │
└─────────────────────────────────────────────────────────────────────┘
```

### 2.2 Hardware-Software Integration Map

| Hardware Component | Interface | ROS Node / Driver | Assigned MCU | Frequency |
|--------------------|-----------|------------------|--------------|-----------|
| USB Camera | USB 2.0 | /camera/image_raw (V4L2) | RPi CM4 | 30 FPS |
| BME280 Sensor | I2C1 | environmental_node (pressure, temp, humidity) | STM32 | 50 Hz |
| BNO055 IMU | I2C1 | imu_node (accel, gyro, mag) | STM32 | 100 Hz |
| MQ-2 Gas Sensor | ADC (PA0) | smoke_detection_node | STM32 | 10 Hz |
| Motor A + Encoder A | PWM + GPIO | motor_controller_node (motor_a) | STM32 | 100 Hz (loop) |
| Motor B + Encoder B | PWM + GPIO | motor_controller_node (motor_b) | STM32 | 100 Hz (loop) |
| LoRa RFM95W | SPI2 | telemetry_node (optional Phase 1) | STM32 | 10 Hz (heartbeat) |

### 2.3 Recommended Technology Stack

#### ROS 2 Ecosystem (RPi CM4)

| Component | Recommendation | Rationale | Maturity | Stars |
|-----------|---|-----------|----------|-------|
| **OS** | Ubuntu 22.04 LTS Arm64 | Official RPi CM4 support; LTS for 5-year support | Stable | - |
| **ROS Distribution** | ROS 2 Humble | LTS release (until May 2027); good balance of performance and features | Very Stable | ~6.5K |
| **Nav2 Stack** | nav2 Humble | Mature SLAM/path-planning; handles odometry + IMU well | Mature | ~2.5K |
| **Camera Driver** | ros2_v4l2_camera | Simple, lightweight USB camera bridge; no external deps | Stable | ~300 |
| **YOLOv8 Inference** | ultralytics python library + custom ROS2 node | Official, well-maintained repository; TFLite or ONNX export for edge | Very Active | ~25K+ |
| **Data Logging** | rosbag2 (official) | Native ROS2 logging format; mature tooling | Stable | - |
| **Serial Bridge** | custom ROS2 node (handwritten UART protocol) vs micro_ros_agent | micro_ros_agent more flexible; custom simpler for deterministic timing | Stable | ~1.5K |

#### STM32 Firmware (Real-Time Control)

| Component | Recommendation | Rationale | Notes |
|-----------|---|-----------|-------|
| **HAL/Middleware** | STM32CubeIDE + STM32CubeMX for init; HAL libraries for driver abstraction | Official toolchain; good for UART, SPI, PWM, ADC | Free, but VSCode plugins available too |
| **RTOS** | FreeRTOS or bare-metal with interrupts | LMS-class task scheduling for motor loop (must be ≤10ms deterministic) | FreeRTOS adds minor overhead but worth the safety; optional for bare-metal experts |
| **Motor Control Library** | HAL Timer/PWM + custom PID controller (no external lib initially) | STM32 timers are excellent; PID tuning onboard from ROS params | Custom is lean; can migrate to HAL abstraction later |
| **IMU/Sensor Drivers** | I2C HAL + simple state machines or lightweight FreeRTOS tasks | BME280/BNO055 have open-source Arduino/STM32 libraries; use or port to HAL | Bosch datasheets are good; avoid copy-paste |
| **Serial Protocol** | Custom COBS/SLIP framing + ROS message serialization (or micro-ROS) | COBS is robust, lightweight, proven for embedded <→ ROS bridges | micro-ROS option if standardization is priority |
| **Bootloader & Flash** | STMicrobootloader via UART1 or custom UART DFU | Allows firmware updates over serial from base station | Essential for long-term field operation |

#### Computer Vision & ML

| Component | Recommendation | Version | Rationale |
|-----------|---|---------|-----------|
| **YOLOv8 Base** | Ultralytics YOLOv8n or YOLOv8s | Latest (2024+) | Small model suitable for RPi CM4 (~15-20 FPS inference) |
| **Export Format** | ONNX or TensorFlow Lite | Latest | TFLite for maximum RPi efficiency; ONNX for flexibility |
| **Inference Runtime** | onnxruntime-arm64 or tflite-runtime | Latest | onnxruntime faster; tflite smaller footprint |
| **Color Detection** | OpenCV (built-in HSV color space) | 4.8+ | No extra dependency; mature HSV tracking |
| **ROS Bridge** | Custom Python node + asyncio for async processing | - | Keep vision separate from motor loop determinism |

#### Database & Logging

| Component | Recommendation | Use Case |
|-----------|---|----------|
| **Onboard Storage** | rosbag2 (native ROS) + SQLite for structured events | Backup recording; post-mission analysis |
| **LoRa Telemetry** | Optional msgpack or Protocol Buffers for compact serialization | Long-range heartbeat and alert transmission |

---

## 3. ROS 2 Package Architecture

### 3.1 Package Organization

```
forest_rover_ros2/
├── src/
│   ├── forest_rover_core/            # Meta-package
│   │   └── package.xml
│   │
│   ├── forest_rover_hardware/        # Hardware abstraction layer
│   │   ├── stm32_firmware_driver/    # UART bridge to STM32
│   │   │   ├── include/
│   │   │   ├── src/
│   │   │   ├── test/
│   │   │   ├── package.xml
│   │   │   └── CMakeLists.txt
│   │   │
│   │   ├── sensor_fusion_node/       # IMU + odometry fusion (optional: robot_localization)
│   │   │   ├── include/
│   │   │   ├── src/
│   │   │   ├── test/
│   │   │   ├── package.xml
│   │   │   └── CMakeLists.txt
│   │   │
│   │   ├── camera_driver/            # USB camera aggregation node
│   │   │   ├── src/
│   │   │   ├── package.xml
│   │   │   └── CMakeLists.txt
│   │   │
│   │   └── package.xml (meta)
│   │
│   ├── forest_rover_perception/      # Vision & detection pipeline
│   │   ├── yolo_detector_node/
│   │   │   ├── yolo_detector/
│   │   │   │   ├── __init__.py
│   │   │   │   ├── detector.py       # YOLOv8 inference wrapper
│   │   │   │   └── msg_converters.py
│   │   │   ├── src/
│   │   │   │   └── yolo_detector_node.py
│   │   │   ├── config/
│   │   │   │   └── yolo_detector.yaml
│   │   │   ├── test/
│   │   │   ├── package.xml
│   │   │   └── CMakeLists.txt
│   │   │
│   │   ├── color_tracker_node/
│   │   │   ├── src/
│   │   │   │   └── color_tracker_node.py
│   │   │   ├── config/
│   │   │   │   └── color_tracker.yaml
│   │   │   ├── package.xml
│   │   │   └── CMakeLists.txt
│   │   │
│   │   └── package.xml (meta)
│   │
│   ├── forest_rover_autonomy/        # Decision logic & behavior
│   │   ├── autonomy_manager/
│   │   │   ├── include/
│   │   │   ├── src/
│   │   │   │   ├── autonomy_manager_node.cpp
│   │   │   │   ├── state_machine.cpp
│   │   │   │   └── behaviors/
│   │   │   │       ├── patrol_behavior.cpp
│   │   │   │       ├── track_behavior.cpp
│   │   │   │       └── investigate_behavior.cpp
│   │   │   ├── config/
│   │   │   │   └── autonomy.yaml
│   │   │   ├── test/
│   │   │   ├── package.xml
│   │   │   └── CMakeLists.txt
│   │   │
│   │   └── package.xml (meta)
│   │
│   ├── forest_rover_description/     # URDF, TF frames, launch files
│   │   ├── urdf/
│   │   │   └── forest_rover.urdf.xacro
│   │   ├── meshes/
│   │   ├── config/
│   │   │   └── tf_broadcast.yaml
│   │   ├── launch/
│   │   │   ├── rover.launch.xml      # Core hardware stack
│   │   │   ├── autonomy.launch.xml   # Autonomy + nav2
│   │   │   ├── sim.launch.xml        # Gazebo simulation (optional)
│   │   │   └── complete.launch.xml   # All-in-one
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   │
│   ├── forest_rover_msgs/            # Custom message & service definitions
│   │   ├── msg/
│   │   │   ├── MotorCommand.msg
│   │   │   ├── EncoderData.msg
│   │   │   ├── DetectionEvent.msg    # YOLOv8 detections
│   │   │   ├── SensorData.msg
│   │   │   └── RoverStatus.msg
│   │   ├── srv/
│   │   │   ├── SetMotorSpeed.srv
│   │   │   ├── EmergencyStop.srv
│   │   │   └── ResetOdometry.srv
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   │
│   └── forest_rover_utils/            # Utilities, calibration, diagnostics
│       ├── motor_tuner_node/         # PID parameter sweep tool
│       ├── calibration/
│       │   └── camera_calibration.py
│       ├── diagnostic_aggregator/    # Health monitoring
│       ├── telemetry_gateway/        # LoRa heartbeat aggregation
│       ├── package.xml
│       └── CMakeLists.txt
│
├── docker/
│   └── Dockerfile.arm64              # Container for RPi CM4 deployment
│
├── docs/
│   ├── ROS_ARCHITECTURE.md
│   ├── MOTOR_TUNING.md
│   ├── YOLO_FINETUNING.md
│   └── DEPLOYMENT.md
│
├── workspace_setup.sh                # Quick start script
└── .colcon-defaults.yaml             # Colcon config for faster builds
```

### 3.2 Node Communication Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        ROS 2 Topic/Service Graph                     │
└─────────────────────────────────────────────────────────────────────┘

HARDWARE LAYER (Low-Level Drivers)
──────────────────────────────────

[stm32_firmware_driver_node]
  ├─ ~/raw_motor_feedback       (MessageType: forest_rover_msgs::EncoderData)
  ├─ ~/imu/data               (MessageType: sensor_msgs/Imu)
  ├─ ~/environmental/data     (MessageType: sensor_msgs/Temperature + sensor_msgs/Humidity + ...)
  ├─ ~/gas_sensor/reading     (MessageType: forest_rover_msgs/GasSensorData)
  ├─ /motor_command           (Service: SetMotorSpeed) ←─ [autonomy_manager]
  └─ /emergency_stop          (Service: EmergencyStop) ←─ [autonomy_manager]

[camera_driver_node]
  ├─ /camera/image_raw        (MessageType: sensor_msgs/Image)
  └─ /camera/camera_info      (MessageType: sensor_msgs/CameraInfo)


PERCEPTION LAYER (Vision Processing)
─────────────────────────────────────

[yolo_detector_node]
  ├─ Sub: /camera/image_raw
  ├─ Pub: /perception/detections          (MessageType: forest_rover_msgs/DetectionEvent)
  └─ Pub: /perception/debug/annotated_img (optional debug visualization)

[color_tracker_node]
  ├─ Sub: /camera/image_raw
  ├─ Sub: /tf (for camera frame transforms)
  ├─ Pub: /perception/ball_centroid       (MessageType: geometry_msgs/PointStamped)
  └─ Pub: /perception/tracking_confidence (MessageType: std_msgs/Float32)


ESTIMATION LAYER (State Fusion)
──────────────────────────────

[sensor_fusion_node] (using robot_localization or custom)
  ├─ Sub: ~/raw_motor_feedback
  ├─ Sub: ~/imu/data
  ├─ Pub: /odometry/filtered            (MessageType: nav_msgs/Odometry)
  ├─ Pub: /tf (base_link → odom)
  └─ Service: /reset_odometry           ←─ [autonomy_manager]


AUTONOMY LAYER (Decision & Behavior)
────────────────────────────────────

[autonomy_manager_node]
  ├─ Sub: /perception/detections
  ├─ Sub: /perception/ball_centroid
  ├─ Sub: /odometry/filtered
  ├─ Sub: /tf
  ├─ Sub: /tf_static
  ├─ Pub: /cmd_vel                      (MessageType: geometry_msgs/Twist)
  │        [nav2_controller or direct motor command routing]
  ├─ Pub: /autonomy/state               (MessageType: std_msgs/String)
  ├─ Pub: /autonomy/event_log           (MessageType: rosgraph_msgs/Log)
  ├─ Service: /set_mission_waypoints    ←─ [operator or scheduler]
  └─ Service: /request_e_stop           ←─ [safety/operator]


NAVIGATION LAYER (Optional nav2 stack)
─────────────────────────────────────

[nav2_controller]
  ├─ Sub: /cmd_vel                      (from autonomy_manager)
  ├─ Sub: /costmap/costmap              (local/global cost maps)
  └─ Pub: /cmd_vel_smoothed             (velocity ramping + collision check)
          └─ → [stm32_firmware_driver via /motor_command service]


LOGGING & TELEMETRY
───────────────────

[data_logger_node]
  ├─ Sub: /camera/image_raw
  ├─ Sub: /perception/detections
  ├─ Sub: /odometry/filtered
  ├─ Sub: /autonomy/event_log
  ├─ Sub: /gas_sensor/reading
  └─ Pub: rosbag2 (.db3 files)

[telemetry_gateway_node] (Optional, Phase 2)
  ├─ Sub: /autonomy/state
  ├─ Sub: /perception/detections (filtered subset)
  ├─ Sub: /rover_status
  └─ → LoRa TX (via STM32 SPI)


TIME SYNCHRONIZATION
────────────────────
- ROS 2 clock on RPi CM4 is system clock (synchronized via NTP if available)
- STM32 uses local monotonic clock; offset calibrated on boot via /sync_clocks service
```

### 3.3 Message Definitions (Custom)

```protobuf
// forest_rover_msgs/msg/DetectionEvent.msg
std_msgs/Header header
int32 class_id          # e.g., 0=deer, 1=boar, 2=person
string class_name       # Human-readable label
float32 confidence      # 0.0-1.0 confidence score
geometry_msgs/BoundingBox bbox    # 2D bounding box in image
geometry_msgs/PointStamped world_position  # Estimated 3D position (if depth available)
int32 frame_count       # Unique detection ID within session

---

// forest_rover_msgs/msg/GasSensorData.msg
std_msgs/Header header
float32 adc_value       # Raw ADC reading (0-1023)
float32 ppm_co          # Carbon monoxide ppm estimate
float32 ppm_smoke       # Smoke ppm estimate
bool alert              # True if exceeds threshold
uint32 alert_level      # 0=nominal, 1=warning, 2=critical

---

// forest_rover_msgs/srv/SetMotorSpeed.srv
Request:
  float32 motor_a_speed  # -1.0 to +1.0 (reverse to forward)
  float32 motor_b_speed  # -1.0 to +1.0
  uint32 duration_ms     # How long to hold this speed
---
Response:
  bool success
  string message
```

---

## 4. Arduino/STM32 Firmware Architecture (Real-Time System)

### 4.1 Firmware Structure (FreeRTOS or Bare-Metal Scheduler)

```
firmware/
├── STM32_FirmwareProject/
│   ├── Core/
│   │   ├── Inc/
│   │   │   ├── main.h
│   │   │   ├── config.h                   # Build-time config
│   │   │   ├── stm32f4xx_it.h             # Interrupt handlers
│   │   │   ├── pid_controller.h
│   │   │   ├── motor_driver.h
│   │   │   ├── encoder.h
│   │   │   ├── i2c_sensor_manager.h
│   │   │   ├── adc_reader.h
│   │   │   ├── uart_bridge.h              # ROS serial protocol
│   │   │   └── safety_manager.h
│   │   │
│   │   └── Src/
│   │       ├── main.c                     # Entry point + FreeRTOS scheduler
│   │       ├── stm32f4xx_it.c             # ISR implementations
│   │       ├── stm32f4xx_hal_msp.c        # HAL peripheral init
│   │       ├── pid_controller.c           # PID state machines
│   │       ├── motor_driver.c             # DRV8871 PWM + safety
│   │       ├── encoder.c                  # Quadrature decoder & odometry
│   │       ├── i2c_sensor_manager.c       # BME280, BNO055 drivers
│   │       ├── adc_reader.c               # MQ-2 gas sensor polling
│   │       ├── uart_bridge.c              # UART1 ROS serial protocol
│   │       ├── safety_manager.c           # Watchdog, timeout handlers
│   │       └── freertos.c                 # FreeRTOS config (if used)
│   │
│   ├── Drivers/
│   │   ├── STM32F4xx_HAL_Driver/          # Official STM32 HAL
│   │   └── sensors/
│   │       ├── bme280.c                   # Bosch BME280 driver
│   │       └── bno055.c                   # Bosch BNO055 driver
│   │
│   ├── Middlewares/
│   │   └── Third_Party/
│   │       └── FreeRTOS/                  # FreeRTOS kernel (if used)
│   │
│   ├── STM32CubeMX_Config/
│   │   └── forest_rover.ioc               # CubeMX project file for reference
│   │
│   ├── CMakeLists.txt                     # ARM-GCC cross-compile
│   └── STM32F407VGTx_FLASH.ld             # Linker script
│
├── Tools/
│   ├── openocd/
│   │   └── forest-rover-stm32f407.cfg     # OpenOCD config for debugging
│   └── uart_bootloader/
│       ├── bootloader.c                   # UART DFU bootloader
│       └── README.md
│
└── Tests/
    ├── motor_test/
    │   ├── motor_sweep_test.c
    │   └── PID_step_response_log.txt
    ├── sensor_test/
    │   └── i2c_bus_scan.c
    └── integration_test/
        └── full_system_test.c
```

### 4.2 Firmware Task Breakdown (FreeRTOS)

```c
// Pseudo-code task structure
FreeRTOS Tasks:

Task 1: MOTOR_CONTROL_TASK (Priority: 3, Frequency: 100 Hz / 10ms)
├─ Read encoder counts (ISR-captured)
├─ Calculate velocity (m/s from rpm)
├─ Run PID controller for each motor
├─ Update PWM duty cycle (DRV8871)
├─ Check for stall condition (timeout)
├─ Publish /raw_motor_feedback to STM32 FIFO

Task 2: SENSOR_ACQUISITION_TASK (Priority: 2, Frequency: 50 Hz / 20ms)
├─ Poll I2C1 bus (BME280, BNO055)
├─ Read ADC for gas sensor + voltage dividers
├─ Gather raw sensor payload
├─ Timestamp data
├─ Queue to UART TX buffer

Task 3: UART_ROS_BRIDGE_TASK (Priority: 1, Frequency: ~Variable)
├─ RX: Listen for motor commands from RPi
├─ TX: Send sensor data frames to RPi
├─ Implement COBS or SLIP framing
├─ Validate checksums
├─ Route to appropriate handler task

Task 4: SAFETY_WATCHDOG_TASK (Priority: 4, Frequency: 10 Hz / 100ms)
├─ Detect lost ROS heartbeat (timeout → stop motors)
├─ Check battery voltage (low battery warning)
├─ Verify motor health (stall detection, current overload)
├─ Emergency stop if triggered
├─ Log safety events to flash memory

Task 5: LORA_TELEMETRY_TASK (Priority: 0, Frequency: 1 Hz / 1s) [Optional Phase 2]
├─ Aggregate status data (voltage, temperature, detection count)
├─ Serialize compact message
├─ SPI to RFM95W
├─ Await TX complete
```

### 4.3 Key Subsystem Interfaces

#### Motor Control Loop

```c
// pseudocode
typedef struct {
  float target_speed_mps;      // m/s
  float actual_speed_mps;
  float accumulated_error;
  int16_t pwm_duty;
  uint32_t last_encoder_count;
  uint32_t current_encoder_count;
} MotorState_t;

void MotorControlTask(void *pvParameters) {
  for(;;) {
    // Read encoder (captured by ISR)
    uint32_t counts = GetEncoderCounts(MOTOR_A);
    float rpm = (counts * ENCODER_CPR) / SAMPLE_TIME_S;
    float speed_mps = rpm * WHEEL_RADIUS / 60.0;
    
    // PID update
    float error = target_speed - speed_mps;
    float pwm = PID_Update(&pid, error);
    
    // Saturate & drive
    SetMotorPWM(MOTOR_A, pwm);
    
    // Telemetry
    PublishMotorFeedback(MOTOR_A, speed_mps, pwm);
    
    vTaskDelay(pdMS_TO_TICKS(10)); // 10ms loop
  }
}
```

#### Sensor Acquisition (I2C Non-Blocking State Machine)

```c
// Pseudo-code non-blocking I2C polling
typedef enum {
  I2C_IDLE,
  I2C_BME280_REQ,
  I2C_BME280_WAIT,
  I2C_BNO055_REQ,
  I2C_BNO055_WAIT,
  I2C_DONE
} I2CState_t;

void SensorAcquisitionTask(void *pvParameters) {
  for(;;) {
    switch(i2c_state) {
      case I2C_IDLE:
        i2c_state = I2C_BME280_REQ;
        HAL_I2C_Mem_Read_IT(&hi2c1, BME280_ADDR, BME280_DATA_REG, 8, sensor_buf, 8);
        break;
      case I2C_BME280_WAIT:
        if (i2c_done_flag) {
          ParseBME280(sensor_buf, &bme280_data);
          i2c_state = I2C_BNO055_REQ;
        }
        break;
      // ... similar for BNO055
      case I2C_DONE:
        PublishSensorData(&bme280_data, &bno055_data);
        i2c_state = I2C_IDLE;
        break;
    }
    vTaskDelay(pdMS_TO_TICKS(20)); // 50 Hz
  }
}
```

---

## 5. Computer Vision Pipeline (YOLOv8 Integration)

### 5.1 CV Architecture on RPi CM4

```
┌──────────────────────────────────────────────────────┐
│         USB Camera (V4L2 driver)                     │
│         ROS 2 Image Topic (/camera/image_raw)       │
└────────────────────┬─────────────────────────────────┘
                     │
                     ↓
        ┌────────────────────────────┐
        │   Image Preprocessing      │
        │ • Resize to YOLOv8 input   │
        │ • Normalize (0-1)          │
        │ • 416x416 or 640x640       │
        └────────┬───────────────────┘
                 │
                 ↓
    ┌────────────────────────────────────┐
    │  YOLOv8n/s Inference Engine        │
    │  (ONNX Runtime or TFLite)          │
    │  Target: ~15-20 FPS on CM4         │
    └──────┬─────────────┬──────────────┘
           │             │
           ↓             ↓
     ┌──────────┐  ┌─────────────────┐
     │Detections│  │Class Confidence │
     │(boxes)   │  │Filtering        │
     └────┬─────┘  └────────┬────────┘
          │                 │
          └────────┬────────┘
                   ↓
        ┌──────────────────────────┐
        │ ROS Message Conversion   │
        │ DetectionEvent[] msg     │
        │ Timestamp + publish      │
        └──────────┬───────────────┘
                   │
                   ↓
     [autonomy_manager_node] ← /perception/detections
     [color_tracker_node]    ← /perception/detections
     [data_logger_node]      ← /perception/detections
```

### 5.2 YOLOv8 Fine-Tuning Workflow

**Phase 1a: Pre-trained Model Selection**
- Download YOLOv8n (nano) weights from Ultralytics
- Test on representative forest images
- Benchmark FPS on RPi CM4 (target ≥ 15 FPS)

**Phase 1b: Transfer Learning (if custom animals are priority)**
- Collect ~500-1000 labeled images of target animals (deer, boar, etc.)
- Annotate in COCO format (use Roboflow or Label Studio)
- Fine-tune YOLOv8s on custom dataset (10-20 epochs, low learning rate)
- Test mAP on validation set; target ≥ 60% mAP@0.5

**Phase 1c: Model Export & Optimization**
- Export to ONNX format (best for edge inference)
- Quantize to int8 if necessary (minor accuracy loss for speed gain)
- Validate latency on RPi CM4 (should be <50ms per frame)

**Phase 2: Deployment**
- Package model + ROS2 node into Docker image
- Deploy to RPi CM4
- Run continuous evaluation on field data

### 5.3 YOLOv8 ROS2 Node Pseudocode

```python
# yolo_detector_node.py (ROS2 Python Node)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np
from forest_rover_msgs.msg import DetectionEvent

class YOLODetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        
        # Load YOLOv8 model
        self.model = YOLO('yolov8s.pt')  # or custom fine-tuned model
        self.model.to('cuda' if torch.cuda.is_available() else 'cpu')
        
        # ROS2 setup
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.detection_pub = self.create_publisher(
            DetectionEvent, '/perception/detections', 10)
        
        self.cv_bridge = CvBridge()
        self.frame_count = 0
        self.get_logger().info('YOLOv8 detector initialized')
    
    def image_callback(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # YOLOv8 inference
            results = self.model(cv_image, conf=0.5)  # confidence threshold
            
            # Parse detections
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = box.conf[0].cpu().numpy()
                    cls_id = int(box.cls[0])
                    cls_name = result.names[cls_id]
                    
                    # Convert to ROS message
                    det_event = DetectionEvent()
                    det_event.header = msg.header
                    det_event.class_id = cls_id
                    det_event.class_name = cls_name
                    det_event.confidence = float(conf)
                    # ... populate bounding box, world_position
                    det_event.frame_count = self.frame_count
                    
                    self.detection_pub.publish(det_event)
            
            self.frame_count += 1
        except Exception as e:
            self.get_logger().error(f'Detection error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = YOLODetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 6. Sensor Interface Abstraction Layer

### 6.1 Abstraction Pattern (Adapter/Facade)

```c
// sensor_interface.h (HAL abstraction)
typedef struct {
  float temperature_c;
  float humidity_rh;
  float pressure_pa;
} EnvironmentalData_t;

typedef struct {
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
  float mag_x, mag_y, mag_z;
  float roll, pitch, yaw;  // Computed orientation
} IMUData_t;

typedef struct {
  float adc_raw_value;
  float ppm_co;
  float ppm_smoke;
  bool alert;
} GasSensorData_t;

// Abstract interfaces
HAL_StatusTypeDef ReadEnvironmental(EnvironmentalData_t *out);
HAL_StatusTypeDef ReadIMU(IMUData_t *out);
HAL_StatusTypeDef ReadGasSensor(GasSensorData_t *out);

// Implementation (sensor-agnostic from application perspective)
// bme280.c, bno055.c, mq2.c implement these functions
```

### 6.2 Deployment Path: Hardware to ROS

```
┌────────────────────┐
│  Physical Sensor   │
│  (BME280, BNO055)  │
└────────┬───────────┘
         │ I2C Protocol
         ↓
┌────────────────────────────┐
│  STM32 HAL I2C Driver      │
│  Polling or Interrupt-Driven│
└────────┬───────────────────┘
         │ Raw ADC/I2C Data
         ↓
┌────────────────────────────┐
│  Sensor-Specific Decoder   │
│  (bme280.c, bno055.c)      │
└────────┬───────────────────┘
         │ Structured Sensor Data
         ↓
┌────────────────────────────┐
│  STM32 FreeRTOS Task       │
│  (SensorAcquisitionTask)   │
└────────┬───────────────────┘
         │ UART Frame (COBS)
         ↓
┌────────────────────────────┐
│  UART1 Link to RPi CM4     │
│  (115200 baud, full-duplex)│
└────────┬───────────────────┘
         │ ROS Serial Protocol
         ↓
┌────────────────────────────┐
│  stm32_firmware_driver_node│
│  (ROS2 Python/C++)         │
└────────┬───────────────────┘
         │ Sensor ROS Topics
         ↓
┌────────────────────────────────────────┐
│  ROS2 Topic Subscribers                │
│  /environmental/data                   │
│  /imu/data                             │
│  /gas_sensor/reading                   │
│  → sensor_fusion_node → nav2            │
│  → autonomy_manager_node                │
└────────────────────────────────────────┘
```

---

## 7. Development Phases, Milestones & Timeline

### 7.1 Git Branching Strategy

```
main (production/stable)
  ↑
  ├─ release/v1.0 ────→ [Release candidate, final testing]
  │
develop (integration branch)
  ↑
  ├─ feature/phase-1-firmware
  ├─ feature/phase-1-ros-hardware-stack
  ├─ feature/phase-2-autonomy
  ├─ feature/phase-2-yolo-detector
  ├─ feature/phase-3-color-tracker
  └─ feature/phase-3-integration-testing
```

**Branch Management Rules:**
- `main`: Tagged releases only (signed tags); hotfixes only
- `develop`: Integration point; all feature branches PR into develop
- `feature/*`: Individual developer branches; prefix with phase and feature name
- Protected branch rules: Require 2 code reviews, passing CI/CD, branch must be up-to-date

### 7.2 Detailed Phase Breakdown

---

#### **PHASE 0: Setup & Infrastructure (Weeks 1-2, 2 weeks)**

##### Milestone 0.1: Repository & Toolchain Setup (Week 1)

| Task | Effort | Assignee | Dependencies | Acceptance Criteria |
|------|--------|----------|--------------|-------------------|
| **0.1.1** Reorganize repo structure (migrate PCB to side branch `hardware/rev-a`) | 4h | DevOps | - | `hardware/rev-a` branch exists with full PCB project; `main` branch is clean for SW |
| **0.1.2** Set up ROS2 Humble workspace with colcon | 8h | Infrastructure | - | Workspace builds clean; colcon profile set in `.colcon-defaults.yaml` |
| **0.1.3** Configure STM32CubeIDE project + OpenOCD debugger | 6h | Firmware Lead | - | STM32 project compiles; can flash and debug via OpenOCD |
| **0.1.4** Set up Docker environment for RPi CM4 arm64 cross-compilation | 8h | Infrastructure | - | Dockerfile builds successfully; can compile ROS2 packages for arm64 |
| **0.1.5** Configure GitHub Actions CI/CD pipeline (ROS build + linting) | 6h | CI/CD | - | GitHub Actions workflow runs on every PR; compiles firmware + ROS packages |
| **0.1.6** Create project documentation skeleton (README, CONTRIBUTING, arch docs) | 4h | Tech Lead | - | Markdown files exist; CI checks for broken links |

**Phase 0 Deliverables:**
- ✅ Clean main branch with proper ROS2 workspace structure
- ✅ `hardware/rev-a` branch preserves all existing PCB work
- ✅ STM32 build toolchain functional
- ✅ CI/CD pipeline operational
- ✅ Docker environment tested

---

#### **PHASE 1: Hardware Driver Layer & Real-Time Firmware (Weeks 3-6, 4 weeks)**

##### Milestone 1.1: STM32 Core Firmware Foundation (Week 3)

| Task | Effort | Assignee | Dependencies | Acceptance Criteria |
|------|--------|----------|--------------|-------------------|
| **1.1.1** Initialize STM32F407 HAL with clock, GPIO, UART1/I2C1/SPI2 | 10h | Firmware | 0.1-0.1.3 | CubeMX config exported; clocks verified with scope (HSE/LSE) |
| **1.1.2** Implement FreeRTOS scheduler + task creation framework | 8h | Firmware | 1.1.1 | FreeRTOS kernel running; tasks spawn and idle successfully |
| **1.1.3** Develop UART1 interrupt handler + COBS framing layer | 12h | Firmware | 1.1.1, 1.1.2 | UART RX/TX tested with loopback; COBS frames verified |
| **1.1.4** Create interrupt-driven encoder ISR for both motors | 8h | Firmware | 1.1.1 | Encoder counts increment correctly on bench rotation test |

**Phase 1.1 Deliverables:**
- ✅ Functional STM32 core with real-time scheduler
- ✅ UART/I2C/SPI communication stacks
- ✅ Encoder feedback path verified

##### Milestone 1.2: Motor Control Subsystem (Week 4)

| Task | Effort | Assignee | Dependencies | Acceptance Criteria |
|------|--------|----------|--------------|-------------------|
| **1.2.1** Implement DRV8871 PWM driver + safety interlocks (current monitoring) | 12h | Firmware | 1.1.1, 1.1.4 | Motors spin at variable PWM; current sense reads correctly on scope |
| **1.2.2** Develop PID controller (tuning interface for Kp/Ki/Kd from UART) | 10h | Firmware | 1.1.3, 1.2.1 | PID parameters adjustable via ROS param server (mockup); step response logged |
| **1.2.3** Integrate encoder feedback into velocity estimation | 8h | Firmware | 1.1.4, 1.2.2 | Odometry calc verified: 1m wheel travel = X encoder counts |
| **1.2.4** Implement motor timeout safety handler (watchdog for lost ROS heartbeat) | 6h | Firmware | 1.1.2, 1.2.1 | Motors stop after 500ms of no command; logs safety event |

**Phase 1.2 Deliverables:**
- ✅ Full motor control loop at 100 Hz
- ✅ PID tuning capability
- ✅ Safety interlocks functional

##### Milestone 1.3: Sensor Drivers (Week 5)

| Task | Effort | Assignee | Dependencies | Acceptance Criteria |
|------|--------|----------|--------------|-------------------|
| **1.3.1** Port BME280 driver (Bosch reference code → HAL I2C) | 10h | Firmware | 1.1.1 | Temperature, humidity, pressure read correctly; compare to reference sensor |
| **1.3.2** Port BNO055 IMU driver (quaternion + Euler angles) | 12h | Firmware | 1.1.1, 1.3.1 | Accel/gyro/mag readings verified with static test; orientation near 0°,0°,0° at rest |
| **1.3.3** Implement MQ-2 gas sensor ADC reading + calibration | 8h | Firmware | 1.1.1 | Raw ADC values map to ppm range; comparison with reference sensor |
| **1.3.4** Integrate PIR motion sensor (analog input + threshold) | 4h | Firmware | 1.1.1 | PIR output toggles on motion; threshold adjustable |
| **1.3.5** Create non-blocking sensor acquisition task (round-robin I2C polling) | 8h | Firmware | 1.3.1-1.3.4 | No I2C deadlock; all sensors read every 20-50ms without blocking motor loop |

**Phase 1.3 Deliverables:**
- ✅ All onboard sensors reading correctly
- ✅ Non-blocking acquisition (no motor jitter)
- ✅ Sensor data packaged for ROS export

##### Milestone 1.4: ROS2 Hardware Bridge (Week 6)

| Task | Effort | Assignee | Dependencies | Acceptance Criteria |
|------|--------|----------|--------------|-------------------|
| **1.4.1** Design STM32 ↔ ROS2 serial protocol (COBS + message IDs) | 6h | Lead | 1.1.3 | Protocol doc written; message types enumerated (motor cmd, sensor data, status) |
| **1.4.2** Implement stm32_firmware_driver ROS2 node (C++ or Python) | 16h | ROS Dev | 1.1-1.3.5,0.1.2 | Node compiles; publishes /imu/data, /environmental/data, /raw_motor_feedback<br/>Subscribes to /motor_command service |
| **1.4.3** Test end-to-end UART bridge on bench (RPi ↔ STM32) | 8h | Integration | 1.4.1-1.4.2 | rostopic echo receives sensor updates; ros service call sets motor speed successfully |
| **1.4.4** Safety handler: implement E-stop service & watchdog timeout | 6h | Firmware | 1.2.4, 1.4.2 | /emergency_stop service stops motors; 500ms no-heartbeat also triggers |

**Phase 1.4 Deliverables:**
- ✅ Reliable STM32 ↔ ROS communication
- ✅ All hardware sensors exposed as ROS topics
- ✅ Motor commands flowing ROS → STM32
- ✅ Safety interlocks verified end-to-end

---

#### **PHASE 2: Autonomy & Navigation Stack (Weeks 7-10, 4 weeks)**

##### Milestone 2.1: Odometry & Sensor Fusion (Week 7)

| Task | Effort | Assignee | Dependencies | Acceptance Criteria |
|------|--------|----------|--------------|-------------------|
| **2.1.1** Implement odometry publisher on ROS2 (wheel-based dead reckoning) | 10h | ROS Dev | 1.4.3 | /odometry/raw topic published at 100 Hz; velocity covariance estimated |
| **2.1.2** Configure robot_localization EKF for IMU + odometry fusion | 12h | ROS Dev | 2.1.1 | /odometry/filtered published; IMU accel helps correct odometry drift |
| **2.1.3** Calibrate wheel radius & encoder CPR on test track (1m baseline) | 6h | Testing | 2.1.1 | Odometry error < 5% over 10m straight-line test |
| **2.1.4** Publish TF frames (odom → base_link, base_link → camera) | 8h | ROS Dev | 2.1.1, 2.1.2 | tf2 tree valid; rviz2 can display rover pose + sensor frame |
| **2.1.5** Create URDF model of rover geometry (wheel base, camera height) | 6h | ROS Dev | 2.1.4 | URDF valid; meshes or simple boxes represent physical layout |

**Phase 2.1 Deliverables:**
- ✅ Odometry fusion robust to mild drift
- ✅ TF frames properly defined
- ✅ URDF package complete
- ✅ rviz2 visualization operational

##### Milestone 2.2: Navigation Stack Integration (Week 8)

| Task | Effort | Assignee | Dependencies | Acceptance Criteria |
|------|--------|----------|--------------|-------------------|
| **2.2.1** Research & select baseline SLAM approach (wheel odometry + IMU only, no LiDAR) | 8h | Lead | 2.1-Milestone | Decision doc: Cartographer (odometry-only) vs nav2 with odometry-only costmap |
| **2.2.2** Configure nav2 behavior tree for simple waypoint patrol | 12h | ROS Dev | 2.1-Milestone, 1.4-Milestone | nav2 controller publishes /cmd_vel; rover follows straight-line waypoints ±0.5m |
| **2.2.3** Implement cmd_vel → motor command bridge (smooth acceleration ramps) | 10h | ROS Dev | 1.4.3, 2.2.2 | RPi publishes /cmd_vel; motors execute smoothly (no jerky acceleration) |
| **2.2.4** Manual remote operation mode (gamepad or web telemetry UI) [Optional] | 8h | ROS Dev | 2.2.3 | Operator can steer rover manually via ROS service or Joy node topic |
| **2.2.5** Integration test: autonomy stack end-to-end on hardware | 12h | Testing | 2.2.1-2.2.4 | Rover navigates 10m waypoint route autonomously; logs odometry & detections |

**Phase 2.2 Deliverables:**
- ✅ nav2 stack operational
- ✅ Waypoint-based autonomy functional
- ✅ End-to-end autonomous movement verified

---

#### **PHASE 3: Computer Vision & Detection (Weeks 11-14, 4 weeks)**

##### Milestone 3.1: YOLOv8 Integration & Detector Node (Weeks 11-12)

| Task | Effort | Assignee | Dependencies | Acceptance Criteria |
|------|--------|----------|--------------|-------------------|
| **3.1.1** Set up YOLOv8 pre-trained model (YOLOv8n download + ARM64 verification) | 6h | CV Lead | 0.1.4 | yolo.pt loads in Python; runs on /camera/image_raw in VM first |
| **3.1.2** Benchmark YOLOv8n inference latency on RPi CM4 | 8h | CV Lead | 3.1.1 | FPS measured: target ≥15 FPS on 640x480 camera input (document in PR) |
| **3.1.3** Implement yolo_detector_node (ROS2 Python + OpenCV) | 14h | ROS Dev | 1.4.3, 3.1.1 | yolo_detector_node publishes /perception/detections; compiles & runs on CM4 |
| **3.1.4** Configure confidence threshold & NMS parameters (tuning doc) | 6h | CV Lead | 3.1.3 | Confidence threshold = 0.5, NMS IOU = 0.45 documented; no duplicate detections |
| **3.1.5** (Optional Phase 1b) Collect & annotate custom animal dataset (~500 images) | 40h | Annotation Team | - | COCO format dataset with ≥3 animal classes; images labeled in Roboflow |

**Phase 3.1 Deliverables:**
- ✅ YOLOv8 detector running on RPi CM4 at ~15-20 FPS
- ✅ /perception/detections ROS topic published
- ✅ Detections logged for analysis
- ✅ (Optional) Custom fine-tuned model available

##### Milestone 3.2: Red Ball Color Tracking (Week 13)

| Task | Effort | Assignee | Dependencies | Acceptance Criteria |
|------|--------|----------|--------------|-------------------|
| **3.2.1** Implement color_tracker_node (HSV red color space, contour-based) | 12h | ROS Dev | 1.4.3 | color_tracker_node runs; outputs /perception/ball_centroid when red object present |
| **3.2.2** Calibrate HSV thresholds for red under varied lighting | 8h | Testing | 3.2.1 | Red ball detected reliably indoors and outdoors (90%+ success over 100 frames) |
| **3.2.3** Estimate 3D position from 2D centroid (pinhole camera model) | 6h | ROS Dev | 3.2.1 | Estimated distance to ball within ±0.2m (tested at 1, 2, 3m distances) |
| **3.2.4** Publish tracking confidence & debug visualization image (annotated frame) | 6h | ROS Dev | 3.2.1 | /perception/tracking_confidence float; /perception/debug/ball_image published |

**Phase 3.2 Deliverables:**
- ✅ Red ball tracking functional
- ✅ Confidence-based gating (ignore low-confidence detections)
- ✅ 3D position estimation

##### Milestone 3.3: Smoke Detection with Alerts (Week 14)

| Task | Effort | Assignee | Dependencies | Acceptance Criteria |
|------|--------|----------|--------------|-------------------|
| **3.3.1** Establish MQ-2 gas sensor baseline & alarm thresholds (lab calibration) | 8h | Hardware | 1.3.3 | Threshold document: 300 ppm CO = warning, 500 ppm CO = critical |
| **3.3.2** Implement smoke_detection_node (ROS publisher + alert system) | 8h | ROS Dev | 1.4.3, 3.3.1 | /gas_sensor/reading published; /alerts/smoke_detected topic when threshold exceeded |
| **3.3.3** Integrate alert triggering into autonomy_manager (behavior change) | 8h | Autonomy | 2.2.1-Milestone, 3.3.2 | On smoke alert: rover logs event, changes to "investigate_fire" behavior, or backs away |
| **3.3.4** Test sensor in controlled smoke/CO scenarios (safety-compliant lab test) | 6h | Testing | 3.3.1-3.3.3 | MQ-2 triggers alert at expected ppm; false alarm rate < 5% over 10min baseline |

**Phase 3.3 Deliverables:**
- ✅ Smoke/fire detection operational
- ✅ Alert handling integrated into autonomy
- ✅ Calibration data documented

---

#### **PHASE 4: Integration, Testing & Hardening (Weeks 15-18, 4 weeks)**

##### Milestone 4.1: System Integration Testing (Week 15)

| Task | Effort | Assignee | Dependencies | Acceptance Criteria |
|------|--------|----------|--------------|-------------------|
| **4.1.1** End-to-end launch script (single `ros2 launch` command) | 8h | Infrastructure | All Phase 1-3 | `ros2 launch forest_rover_description complete.launch.xml` runs all nodes |
| **4.1.2** Stress test: simultaneous motor control + vision + sensor polling (HW bench) | 12h | Testing | 2.2-Milestone, 3.1-Milestone | No deadlocks; motor timing jitter < 2ms; frame drop rate < 5% |
| **4.1.3** IMU vibration & noise characterization during motor operation | 8h | Testing | 2.1-Milestone | Accel noise < 0.1g RMS; rate offset < 1°/s during continuous operation |
| **4.1.4** Field trial: autonomous patrol + detection on test track (1000m²) | 12h | Testing | 4.1.1-4.1.3 | Rover completes 3 patrol circuits; detects 5/5 planted test objects; no crashes |
| **4.1.5** Document integration issues + fixes in postmortem analysis | 6h | Lead | 4.1.1-4.1.4 | Postmortem doc: top 5 issues, root causes, solutions |

**Phase 4.1 Deliverables:**
- ✅ System boots reliably in field
- ✅ No major timing or stability issues
- ✅ Field trial data collected

##### Milestone 4.2: Data Logging & Analysis (Week 16)

| Task | Effort | Assignee | Dependencies | Acceptance Criteria |
|------|--------|----------|--------------|-------------------|
| **4.2.1** Implement data_logger_node (rosbag2 continuous recording) | 8h | ROS Dev | All nodes | rosbag2 files created; can play back all sensor streams |
| **4.2.2** Add structured event logging (detection events, state transitions, alerts) | 6h | ROS Dev | 4.2.1 | SQLite database of events; can query by timestamp or event type |
| **4.2.3** Develop post-mission analysis script (detection statistics, odometry accuracy) | 8h | Data | 4.2.1-4.2.2 | Script outputs: detection count/confidence histogram, odometry drift estimate |
| **4.2.4** Dashboard for real-time monitoring (rviz2 + custom plugin or web UI) | 12h | ROS Dev | 4.2.1-4.2.3 | Real-time display of rover location, detected animals, alerts, battery voltage |

**Phase 4.2 Deliverables:**
- ✅ Complete telemetry recorded for each mission
- ✅ Analysis tools ready for post-hoc evaluation
- ✅ Operator dashboard functional

##### Milestone 4.3: Robustness & Edge Cases (Week 17)

| Task | Effort | Assignee | Dependencies | Acceptance Criteria |
|------|--------|----------|--------------|-------------------|
| **4.3.1** Camera failure graceful degradation (navigate on odometry+IMU alone) | 8h | Autonomy | 2.2-Milestone | If camera feed loses 3+ seconds: autonomy switches to odometry-only mode |
| **4.3.2** Low battery detection & safe return-to-base behavior | 8h | Hardware | 1.4.4, 2.2.1-Milestone | Voltage threshold at 9.0V nominal (3S LiPo); triggers return-to-start waypoint |
| **4.3.3** IMU drift compensation over long missions (>1 hour) | 6h | Autonomy | 2.1.3 | Over 2-hour mission: odometry error < 10m (5% of patrol area) |
| **4.3.4** WiFi/LoRa connectivity loss handling (queued telemetry) | 8h | Infrastructure | - | Messages queued locally if LoRa unavailable; sent on reconnect |
| **4.3.5** Environmental stress testing (rain, snow, temperature swings) | 8h | Testing | All milestones | Operate successfully in 0-35°C range; PCB survives light rain (water ingress < acceptable level) |

**Phase 4.3 Deliverables:**
- ✅ System tolerates component failures gracefully
- ✅ Multi-hour autonomous missions achievable
- ✅ Environmental robustness verified

##### Milestone 4.4: Performance Optimization & Documentation (Week 18)

| Task | Effort | Assignee | Dependencies | Acceptance Criteria |
|------|--------|----------|--------------|-------------------|
| **4.4.1** Profile & optimize motor latency (target: <5ms end-to-end cmd→motor response) | 10h | Firmware | 1.2-Milestone, 1.4-Milestone | Scope measurements: UART RX latency + STM32 processing + PWM update < 5ms |
| **4.4.2** Memory optimization: ROS node heap footprint (target: <300MB per node) | 8h | Infrastructure | All nodes | Measured with `ps` / `valgrind`; no leaks detected over 1-hour uptime |
| **4.4.3** Thermal analysis: identify hotspots during full-load autonomy | 6h | Hardware | 4.1.1-4.1.4 | Thermal camera scan; max temps on MCU, buck converter, motors ≤ spec limits |
| **4.4.4** Create comprehensive field operations guide + troubleshooting FAQ | 12h | Tech Writer | All doc sources | Guide covers: deployment checklist, calibration, common issues & recovery |
| **4.4.5** Finalize architecture documentation (auto-generated from comments + diagrams) | 8h | Tech Lead | All docs | docs/ folder has: ROS_NODE_ARCHITECTURE.md, FIRMWARE_STRUCTURE.md, DEPLOYMENT.md |

**Phase 4.4 Deliverables:**
- ✅ System meets latency/memory targets
- ✅ Comprehensive documentation ready
- ✅ Operator guide complete

---

#### **PHASE 5: LoRa Telemetry & Remote Operations (Weeks 19-20, 2 weeks) [OPTIONAL - PHASE 2]**

##### Milestone 5.1: LoRa Heartbeat & Status Reporting

| Task | Effort | Assignee | Dependencies | Acceptance Criteria |
|------|--------|----------|--------------|-------------------|
| **5.1.1** Configure RFM95W radio parameters (frequency, bandwidth, spread factor) | 6h | Hardware | 1.4.1 | LoRa radio transmits/receives test packets; range > 500m in-field |
| **5.1.2** Implement telemetry_gateway_node (heartbeat + alert transmission) | 10h | ROS Dev | 1.4.1, 4.1.1 | Heartbeat every 10s: position, battery, detection count |
| **5.1.3** Build base station ROS2 node to receive & log LoRa telemetry | 8h | ROS Dev | 5.1.2 | Base station displays live rover location on map; alerts trigger notification |
| **5.1.4** Range characterization test (measure max comms distance in field) | 6h | Testing | 5.1.1-5.1.3 | Document: "Effective range: 1.2 km line-of-sight; 400m through light forest" |

**Phase 5 is deferred to Phase 2 (if required).**

---

### 7.3 Overall Timeline Summary

```
Week  1-2:  Phase 0: Setup & Infrastructure
Week  3-6:  Phase 1: Hardware Driver Layer & Firmware
Week  7-10: Phase 2: Autonomy & Navigation
Week 11-14: Phase 3: Vision & Detection
Week 15-18: Phase 4: Integration & Hardening
Week 19-20: Phase 5: LoRa [OPTIONAL]

Total: 18-20 weeks (~4-5 months)
```

---

## 8. Risk Assessment & Mitigation

| Risk | Probability | Impact | Mitigation Strategy | Owner |
|------|-------------|--------|-------------------|-------|
| **RPi CM4 thermal throttling during YOLOv8 inference** | Medium | High | Pre-benchmark YOLOv8n latency early (Week 11); consider YOLOv8n-edge quantization; active heatsink | CV Lead |
| **UART serial communication dropout under high motor transients** | Medium | Medium | Add error correction (CRC16); hardware loopback test; shielded cable; ferrite on UART | Firmware |
| **Encoder resolution insufficient for accurate odometry** | Low | Medium | Calibrate on 10m baseline early (Phase 2.1); consider gear ratio boost; sensor fusion with IMU | Firmware |
| **YOLOv8 false positives in outdoor lighting variations** | Medium | Low | Collect diverse training data; confidence threshold tuning; run-time augmentation | ML/CV |
| **LiPo battery under-capacity during long missions** | Low | Medium | Power budget analysis (already done); test 2-hour mission; plan battery pack upgrade for Phase 2 | Hardware |
| **nav2 path planning collisions with unmapped obstacles** | Medium | High | Fallback to simple reactive obstacle avoidance; beep/buzzer on approach | Autonomy |
| **IMU drift accumulation over hours** | High | Low | Continuous wheel odometry + EKF fusion; consider GPS for Phase 2 | Software |
| **Firmware OTA update fails; rover becomes bricked** | Low | Critical | Implement bootloader with fallback to previous version; test update mechanism thoroughly | Firmware |
| **LoRa range inadequate for field deployment** | Medium | Low | Early range test (Phase 5.1); contingency: local only, no LoRa requirement for Phase 1 | Hardware |

---

## 9. Testing & Verification Strategy

### 9.1 Unit Testing

| Component | Testing Approach | Coverage Target |
|-----------|-----------------|-----------------|
| **PID Controller (Firmware)** | Step response test; verify settling time < 2s for 0.5 m/s command | 100% |
| **Encoder Feedback** | Bench rotation with known distance; verify CPR calibration | 100% |
| **Sensor Drivers** | I2C loopback test; verify data format & CRC | 95%+ |
| **UART Protocol** | Send/receive COBS frames; simulate packet loss and corruption | 100% |
| **ROS Message Serialization** | Unit test message encoding/decoding | 95%+ |

### 9.2 Integration Testing

| Scenario | Duration | Success Criteria |
|----------|----------|-----------------|
| **Motor + Encoder + PID Loop** | 10 min | Velocity feedback smooth; reaching setpoint in <1s |
| **Multi-Sensor Fusion (IMU + Odometry)** | 5 min | No topic deadlocks; timestamp alignment < 10ms |
| **Camera + YOLOv8 + Detection Node** | 10 min | Detections published every frame; latency < 100ms |
| **Autonomy + nav2 + Motor Control** | 20 min | Rover follows 10m waypoint course ±0.5m accuracy |
| **Full System End-to-End (Field)** | 30 min | Autonomous patrol, detect 5 targets, return safely |

### 9.3 System Testing

| Test | Acceptance Criteria | Phase |
|------|-------------------|-------|
| **Straight-Line Odometry** | Error < 5% over 10m | 2.1 |
| **Circular Track Following** | Error < 10cm radius | 2.2 |
| **Detection Accuracy** | 5/5 targets detected in field | 3.1 |
| **Smoke Alert Response** | From gas threshold to autonomy behavior < 2s | 3.3 |
| **Battery Endurance** | 2+ hours autonomous operation | 4.1 |
| **Temperature Range** | Functional 0-35°C | 4.3 |
| **Rain Ingress** | Survive light rain; no electrical shorts | 4.3 |

---

## 10. File Structure & Project Layout

### 10.1 Final Repository Structure

```
forest_surveillance_rover/
├── .github/
│   └── workflows/
│       ├── firmware-build.yml           # STM32 CMake build on push
│       ├── ros-build.yml                # ROS2 colcon build on push
│       └── lint-and-test.yml            # Code style checks
│
├── firmware/
│   ├── STM32F407_Project/
│   │   ├── Core/Inc/
│   │   │   ├── main.h
│   │   │   ├── config.h
│   │   │   ├── pid_controller.h
│   │   │   ├── motor_driver.h
│   │   │   ├── encoder.h
│   │   │   ├── i2c_sensor_manager.h
│   │   │   ├── uart_bridge.h
│   │   │   └── safety_manager.h
│   │   ├── Core/Src/
│   │   │   ├── main.c
│   │   │   ├── pid_controller.c
│   │   │   ├── motor_driver.c
│   │   │   ├── encoder.c
│   │   │   ├── i2c_sensor_manager.c
│   │   │   ├── uart_bridge.c
│   │   │   ├── safety_manager.c
│   │   │   └── freertos.c
│   │   ├── Drivers/
│   │   │   ├── STM32F4xx_HAL_Driver/
│   │   │   └── sensors/
│   │   │       ├── bme280.c
│   │   │       └── bno055.c
│   │   ├── CMakeLists.txt
│   │   └── STM32F407VGTx_FLASH.ld
│   │
│   ├── Tools/
│   │   ├── openocd/
│   │   │   └── forest-rover-stm32f407.cfg
│   │   └── uart_bootloader/
│   │       └── bootloader.c
│   │
│   └── Tests/
│       ├── motor_test/
│       ├── sensor_test/
│       └── integration_test/
│
├── ros2_ws/
│   ├── src/
│   │   ├── forest_rover_core/
│   │   ├── forest_rover_hardware/
│   │   │   ├── stm32_firmware_driver/
│   │   │   ├── sensor_fusion_node/
│   │   │   └── camera_driver/
│   │   ├── forest_rover_perception/
│   │   │   ├── yolo_detector_node/
│   │   │   └── color_tracker_node/
│   │   ├── forest_rover_autonomy/
│   │   │   └── autonomy_manager/
│   │   ├── forest_rover_description/
│   │   ├── forest_rover_msgs/
│   │   └── forest_rover_utils/
│   │
│   ├── docker/
│   │   └── Dockerfile.arm64
│   │
│   └── workspace_setup.sh
│
├── docs/
│   ├── IMPLEMENTATION_PLAN.md            # This file
│   ├── ROS2_ARCHITECTURE.md
│   ├── FIRMWARE_STRUCTURE.md
│   ├── MOTOR_TUNING.md
│   ├── YOLO_FINETUNING.md
│   ├── DEPLOYMENT.md
│   ├── TROUBLESHOOTING.md
│   └── API_REFERENCE.md
│
├── hardware/ [from hardware/rev-a branch]
│   ├── kicad/
│   ├── gerbers/
│   └── README.md
│
├── simulation/
│   ├── ngspice/
│   └── qucs-s/
│
├── docker/
│   └── Dockerfile.arm64                 # Full development environment
│
├── scripts/
│   ├── setup_rpi_cm4.sh                 # Ubuntu 22.04 setup script
│   ├── calibrate_motors.py              # PID tuning interface
│   ├── calibrate_camera.py              # Camera intrinsics
│   ├── run_field_trial.sh               # Mission launcher
│   └── analyze_mission_data.py           # Post-hoc analysis
│
├── .colcon-defaults.yaml                # Colcon build config
├── .flake8                              # Python linting
├── .clang-format                        # C/C++ formatting
├── CONTRIBUTING.md
├── LICENSE
└── README.md                            # Project overview
```

---

## 11. Library & Framework Recommendations

### 11.1 Curated Technology Stack (2024-2025)

#### ROS 2 Core Libraries

| Library | Purpose | Latest Version | Stars | Status | License |
|---------|---------|----------------|-------|--------|---------|
| **ROS 2 Humble** | Middleware for robotics | 2024.04 LTS | - | Stable | Apache 2.0 |
| **nav2** | Navigation & path planning | 1.1.x | ~2.5K | Production-Ready | Apache 2.0 |
| **robot_localization** | EKF/UKF sensor fusion | 0.4.x | ~1.2K | Stable | BSD |
| **micro_ros_agent** | Middleware for embedded ↔ ROS | 2.0.x | ~500 | Active | Apache 2.0 |
| **ros2_v4l2_camera** | USB camera driver | Latest | ~300 | Works well | Apache 2.0 |
| **image_transpose** | Image rotation (if needed) | Latest | - | Maintained | BSD |

#### Computer Vision & ML

| Library | Purpose | Latest Version | Stars | Status | License |
|---------|---------|----------------|-------|--------|---------|
| **Ultralytics YOLOv8** | Object detection | 8.x (2024) | ~25K+ | Very Active | AGPL3 (MIT available) |
| **OpenCV** | Image processing | 4.8+ | ~35K+ | Stable | Apache 2.0 |
| **onnxruntime** | ONNX model inference | 1.17+ | ~9K+ | Active | MIT |
| **tflite-runtime** | TensorFlow Lite edge inference | Latest | - | Maintained | Apache 2.0 |

#### Embedded Systems (STM32)

| Library | Purpose | Source | Status | License |
|---------|---------|--------|--------|---------|
| **STM32CubeF4** | Official HAL & middleware | STMicroelectronics | Stable | BSD |
| **FreeRTOS** | Real-time OS | Official / GitHub | Stable | MIT |
| **ChibiOS** | Alternative RTOS (lighter) | GitHub | Maintained | Apache 2.0 |

#### Development & Deployment

| Tool | Purpose | Version | Stars | Status |
|------|---------|---------|-------|--------|
| **VSCode** | Editor (STM32 extension + ROS extension) | Latest | - | Standard |
| **STM32CubeIDE** | Official STM32 IDE (optional) | 1.13+ | - | Official |
| **CMake** | Build system (firmware + ROS) | 3.22+ | - | Standard |
| **colcon** | ROS2 build tool | Latest | - | Official |
| **Docker** | Containerization | Latest | - | Standard |
| **GitHub Actions** | CI/CD | - | - | Standard |

---

## 12. Complexity & Effort Estimation

### 12.1 Work Breakdown Structure (WBS) Summary

```
Total Project Effort: ~2,484 hours (~62 weeks at 40 hrs/week)
Team Size: 5-6 people (Firmware, ROS/Autonomy, CV, Testing, Infra)

Phase 0 (Infrastructure):     92 hours  (~2-3 engineers)
Phase 1 (Firmware/Drivers):  544 hours  (~2-3 engineers)
Phase 2 (Autonomy/Nav2):     468 hours  (~2 engineers)
Phase 3 (Vision/Detection):  420 hours  (~2-3 engineers)
Phase 4 (Integration):       652 hours  (~3-4 engineers)
Phase 5 (LoRa) [Optional]:   308 hours  (~2 engineers, Phase 2)
```

### 12.2 Effort per Skill

```
Firmware/Embedded (STM32/FreeRTOS):  ~800 hours (32% of Phase 1-4)
ROS 2 / Autonomy:                    ~600 hours (24% of Phase 1-4)
Computer Vision / YOLOv8:             ~400 hours (16% of Phase 1-4)
Integration Testing / Deployment:     ~500 hours (20% of Phase 1-4)
DevOps / CI-CD / Documentation:       ~184 hours  (8% of Phase 1-4)
```

---

## 13. Quality Gates & Validation Checklist

### 13.1 Phase Gate Requirements

**Gate 0→1: Infrastructure Readiness**
- [ ] Main branch clean, PCB on hardware/rev-a
- [ ] STM32 toolchain builds firmware
- [ ] ROS2 workspace compiles
- [ ] CI/CD pipeline passes first build
- **Gate Owner**: Tech Lead | **Approval Criteria**: All checks pass

**Gate 1→2: Firmware Integration**
- [ ] Motor control loop at 100 Hz, jitter < 2ms
- [ ] All sensors reading without I2C/SPI deadlocks
- [ ] UART communication 100% reliable over 10-minute stress test
- [ ] Safety watchdog triggers correctly on heartbeat loss
- **Gate Owner**: Firmware Lead | **Approval Criteria**: Field bench tests pass

**Gate 2→3: Autonomy Readiness**
- [ ] Odometry error < 5% over 10m straight-line test
- [ ] nav2 controller follows 10m waypoint within ±0.5m
- [ ] Sensor fusion (IMU + odometry) stable over 30 min
- **Gate Owner**: Autonomy Lead | **Approval Criteria**: Track test success

**Gate 3→4: Vision Capability**
- [ ] YOLOv8 runs at ≥15 FPS on RPi CM4
- [ ] Detects 5/5 planted test objects in clear lighting
- [ ] Red ball tracker achieves ±5cm centroid accuracy at 2m distance
- [ ] Smoke sensor triggers alert at calibrated threshold
- **Gate Owner**: CV Lead | **Approval Criteria**: Bench + lab tests pass

**Gate 4→5: Integration & Field Ready**
- [ ] System boots reliably from cold start
- [ ] Autonomous mission completes 1000m patrol without intervention
- [ ] Data logger captures all telemetry; post-analysis tools work
- [ ] Graceful degradation verified (camera failure, low battery, comms loss)
- [ ] Thermal, vibration, and environmental stress tests pass
- **Gate Owner**: Test Lead | **Approval Criteria**: Field trial success + no major bugs

---

## 14. Success Metrics & Acceptance Criteria

### 14.1 Functional Success Metrics

| Metric | Target | Verification |
|--------|--------|--------------|
| **Autonomous Navigation Accuracy** | ±0.5m over 500m patrol | GPS ground truth comparison |
| **Detection Sensitivity** | ≥80% true positive rate @ 0.5m confidence | 100+ labeled test images |
| **Motor Latency** | <5ms end-to-end cmd→response | Scope capture of UART + motor current |
| **Vision FPS** | ≥15 FPS YOLOv8 on RPi CM4 | Benchmark script (% throttling acceptable) |
| **Autonomy Duration** | ≥2 hours on single charge | 3+ consecutive mission tests |
| **System Uptime** | 99%+ no unplanned reboots over 24h | Extended soak test |

### 14.2 Non-Functional Success Metrics

| Metric | Target | Verification |
|--------|--------|--------------|
| **Build Time** | <2 min incremental ROS rebuild | `colcon build --symlink-install` timing |
| **Memory Footprint** | <400 MB total ROS nodes | `ps aux` snapshot during nominal operation |
| **Code Coverage** | ≥80% unit test coverage (firmware) | gcov report |
| **Documentation Completeness** | All APIs & node interfaces documented | Doxygen + ROS wiki index |
| **Deployment Reproducibility** | Fresh RPi CM4 ready in <30 min | Runbook test on virgin hardware |

---

## 15. Deployment & Operations Plan

### 15.1 RPi CM4 Deployment Checklist

```bash
# Prerequisites
[ ] Ubuntu 22.04 LTS flashed to microSD card
[ ] Network (WiFi or Ethernet) accessible
[ ] Docker installed (for development environment)

# Installation Steps
1. [ ] Clone repo: git clone https://github.com/team/forest-rover.git
2. [ ] Run setup: bash ros2_ws/workspace_setup.sh
3. [ ] Build firmware: cd firmware && cmake --build . --target clean all
4. [ ] Build ROS: cd ros2_ws && colcon build
5. [ ] Configure environment: source install/setup.bash
6. [ ] Calibrate motors: python3 scripts/calibrate_motors.py
7. [ ] Calibrate camera: python3 scripts/calibrate_camera.py
8. [ ] Test UART bridge: ros2 launch forest_rover_description rover.launch.xml
9. [ ] Verify all nodes publishing: ros2 node list && ros2 topic list

# Field Operations
[ ] Pre-Flight Checklist
  - [ ] Battery fully charged
  - [ ] Motors free-spinning
  - [ ] Camera lens clean
  - [ ] LoRa antenna secured
  - [ ] Logging storage available (≥10 GB)
  
[ ] Execute Mission
  - [ ] ros2 launch forest_rover_description complete.launch.xml
  - [ ] Set waypoints via /set_mission_waypoints service
  - [ ] Confirm rover starts autonomous patrol
  - [ ] Monitor real-time dashboard (rviz2 or web UI)
  
[ ] Post-Mission Analysis
  - [ ] Stop rover: ros2 service call /emergency_stop
  - [ ] Download rosbag: cp /tmp/mission_*.db3 /external/drive/
  - [ ] Run analysis: python3 scripts/analyze_mission_data.py mission_*.db3
  - [ ] Review detection logs: grep "class_name.*deer" mission_events.log
```

### 15.2 Continuous Improvement Loop

- **Weekly**: Review detection accuracy metrics; flag misclassifications for retraining
- **Monthly**: Analysis of odometry drift; adjust EKF parameters if needed; motor maintenance
- **Quarterly**: Firmware security updates; ROS dependency patches; YOLOv8 model refresh with new field data

---

## 16. Communication & Reporting Structure

### 16.1 Stakeholder Updates

| Stakeholder | Frequency | Format | Owner |
|-------------|-----------|--------|-------|
| **Project Manager** | Weekly | 1-page status (blockers, milestones hit, risks) | Tech Lead |
| **Hardware Team** | Bi-weekly | Tech sync (integration issues, sensor calibration) | Firmware Lead |
| **Field Testing** | After each trial | Debrief + data summary (detections, odometry, duration) | Test Lead |
| **Leadership** | Monthly | Executive summary (progress, budget, risks) | Project Lead |

### 16.2 Technical Documentation Updates

- **After Phase Gates**: Architecture diagrams, lessons learned
- **After Field Trials**: Postmortem analysis, performance metrics
- **Quarterly**: Comprehensive project retrospective; knowledge base update

---

## ADDITIONAL RESOURCES

### Recommended External References

| Resource | Type | Use Case |
|----------|------|----------|
| **ROS 2 Official Docs** (docs.ros.org) | Documentation | Nav2 stack reference, message definitions |
| **Ultralytics YOLOv8 Docs** (docs.ultralytics.com) | Guide | Model training, export, inference optimization |
| **STM32F4 Reference Manual** (STMicroelectronics) | Hardware Ref | Peripheral timing, register definitions |
| **Nav2 Humble Tuning Guide** (github.com/ros-planning/navigation2) | Guide | Costmap config, planner parameters |
| **FreeRTOS Handbook** (freertos.org) | Reference | Task scheduling, synchronization primitives |

---

## 📋 DISPLAY INSTRUCTIONS FOR OUTER AGENT

**Outer Agent: You MUST present this development plan using the following format:**

1. **Present the COMPLETE development roadmap** - Do not summarize or abbreviate sections
2. **Preserve ALL task breakdown structures** with checkboxes and formatting intact
3. **Show the full risk assessment matrix** with all columns and rows
4. **Display ALL planning templates exactly as generated** - Do not merge sections
5. **Maintain all markdown formatting** including tables, checklists, and code blocks
6. **Present the complete technical specification** without condensing
7. **Show ALL quality gates and validation checklists** in full detail
8. **Display the complete library research section** with all recommendations and evaluations

**Do NOT create an executive summary or overview - present the complete development plan exactly as generated with all detail intact.**

