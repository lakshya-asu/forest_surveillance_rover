# Forest Surveillance Rover - Implementation Plan Summary

**Status**: ✅ Comprehensive plan complete - ready to execute  
**Total Effort**: ~520 person-hours  
**Timeline**: 16 weeks (single dev) | 8 weeks (team of 2-3)  
**GitHub**: Feature branches from `develop` into `main`

---

## 🎯 Quick Overview

### Project Vision
Autonomous forest rover with **ROS 2 (RPi CM4)** high-level autonomy + **STM32F407** real-time motor control, featuring:
- 🦌 **YOLOv8 animal detection** (transfer learned)
- 🔥 **MQ-2 smoke/fire detection** with alerts
- 🔴 **Red ball color tracking** & pursuit
- 🗺️ **nav2 autonomous navigation** + SLAM
- 📡 **Full telemetry logging** (rosbag2)

---

## 📋 Implementation Phases

### Phase 0: Infrastructure (Weeks 1-2) - 50 hours
```
✓ Move PCB to hardware/rev-a branch
✓ Initialize ROS2 workspace
✓ STM32CubeIDE + OpenOCD setup
✓ Docker ARM64 cross-compilation
✓ GitHub Actions CI/CD
✓ Architecture documentation
```

### Phase 1: STM32 Firmware (Weeks 3-6) - 120 hours
```
Week 3 - Core              Week 4 - Motors         Week 5 - Sensors       Week 6 - ROS Bridge
├─ Clock/GPIO setup        ├─ DRV8871 PWM driver  ├─ BME280 (I2C)         ├─ UART protocol
├─ FreeRTOS scheduler      ├─ PID controller      ├─ BNO055 (IMU)         ├─ stm32_driver node
├─ UART + COBS framing     ├─ Encoder feedback    ├─ MQ-2 (ADC)           ├─ Sensor topics
└─ Encoder ISRs            └─ Watchdog safety     └─ Non-blocking acq.    └─ Motor commands
```
**Deliverable**: ✅ Motor control @ 100Hz + all sensors → ROS topics

### Phase 2: Autonomy Stack (Weeks 7-10) - 140 hours
```
Week 7 - Odometry          Week 8 - Navigation    Week 9 - SLAM?         Week 10 - Safety
├─ Wheel odometry          ├─ nav2 behavior tree  ├─ SLAM framework       ├─ Diagnostics
├─ EKF sensor fusion       ├─ cmd_vel bridge      ├─ 20m route test       ├─ Data logger
├─ TF frames               ├─ Remote mode         └─ Error bounds         ├─ Telemetry
└─ URDF model              └─ 10m waypoint test   (odometry-only OK)      └─ E-stop verification
```
**Deliverable**: ✅ Autonomous waypoint navigation + data logging

### Phase 3: Computer Vision (Weeks 11-14) - 130 hours
```
Weeks 11-12 - YOLOv8       Week 13 - Color Tracking Week 14 - State Machine
├─ Download/benchmark      ├─ OpenCV HSV           ├─ Patrol mode
├─ Optional: Fine-tune      ├─ Centroid detection   ├─ Animal investigation
├─ ONNX export             └─ Ball tracking node   ├─ Ball pursuit
└─ yolo_detector node       (≥15 FPS on CM4)      ├─ Fire alert
   (/perception/detections)                        └─ Behavior transitions
```
**Deliverable**: ✅ Multi-task autonomy with detection logging

### Phase 4: Integration & Field Testing (Weeks 15-16) - 80 hours
```
Week 15 - Full System              Week 16 - Deployment
├─ End-to-end mission testing      ├─ Final field trials
├─ Performance profiling           ├─ Deployment checklist
├─ Battery/CPU optimization        └─ Troubleshooting guide
└─ Ground-truth log collection
```
**Deliverable**: ✅ Production-ready system + documentation

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│            Raspberry Pi CM4 (High-Level Autonomy)           │
├─────────────────────────────────────────────────────────────┤
│  ROS 2 Humble                                               │
│  ├─ nav2 (costmap + planner)                                │
│  ├─ YOLOv8 detector (≥15 FPS)                               │
│  ├─ Color tracker (red ball)                                │
│  ├─ State machine (patrol/investigate/track/alert)          │
│  └─ Data logger (rosbag2)                                   │
└──────────────┬────────────────────────────────────────────┬─┘
               │ UART (115200 baud)                         │
               ↓                                            ↓
    ┌──────────────────────────┐        ┌──────────────────┐
    │   STM32F407              │        │  USB Camera      │
    │   (Real-Time Control)    │        │  (30 FPS)        │
    ├──────────────────────────┤        └──────────────────┘
    │  FreeRTOS Tasks:         │
    │  ├─ Motor control (100Hz) │
    │  ├─ Sensor acquisition    │
    │  ├─ UART ROS bridge       │
    │  └─ Safety watchdog       │
    │                          │
    │  Hardware:              │
    │  ├─ Motors (PWM)        │
    │  ├─ Encoders            │
    │  ├─ BME280 (I2C)        │
    │  ├─ BNO055 (I2C)        │
    │  ├─ MQ-2 (ADC)          │
    │  └─ [Future: LoRa SPI]  │
    └──────────────────────────┘
            ↓
        [3S LiPo Battery]
```

---

## 📊 Technology Stack

| Layer | Technology | Version |
|-------|-----------|---------|
| **OS** | Ubuntu 22.04 LTS ARM64 | - |
| **ROS** | ROS 2 Humble (LTS) | latest |
| **Navigation** | nav2 | Humble |
| **Vision** | YOLOv8 (Ultralytics) | latest |
| **Inference** | ONNX Runtime (ARM) | latest |
| **Camera** | ros2_v4l2_camera | - |
| **Sensor Fusion** | robot_localization EKF | - |
| **Logging** | rosbag2 (SQLite) | - |
| **STM32** | STM32CubeIDE + HAL | latest |
| **RTOS** | FreeRTOS | latest |
| **Compiler** | ARM-GCC (arm-none-eabi) | 11.3+ |
| **Build** | CMake + colcon | - |

---

## 🔄 Git Branching Strategy

```
main (ROS code - new)
  ↑
  ├─ release/v1.0
  │  └─ hotfixes/
  │
develop (integration)
  ↑
  ├─ feature/phase-0-setup
  ├─ feature/phase-1-firmware
  │  ├─ feature/motors
  │  ├─ feature/sensors
  │  └─ feature/ros-bridge
  ├─ feature/phase-2-autonomy
  │  ├─ feature/odometry
  │  └─ feature/nav2-integration
  ├─ feature/phase-3-vision
  │  ├─ feature/yolov8-detector
  │  └─ feature/color-tracker
  └─ feature/phase-4-integration

hardware/rev-a (PCB - existing)
  └─ All KiCAD, gerbers, docs, simulation
```

---

## 📁 Directory Structure

```
forest_surveillance_rover/
├── firmware/                              # STM32F407 code
│   ├── STM32_FirmwareProject/
│   │   ├── Core/Inc + Src/                # HAL + FreeRTOS tasks
│   │   ├── Drivers/                       # STM32 HAL drivers
│   │   └── Middlewares/FreeRTOS/
│   ├── Tools/
│   │   ├── openocd/
│   │   └── uart_bootloader/
│   └── Tests/
│
├── ros2_workspace/src/
│   ├── forest_rover_core/ (meta-package)
│   ├── forest_rover_hardware/
│   │   ├── stm32_firmware_driver/        # UART bridge
│   │   ├── camera_driver/
│   │   └── sensor_fusion_node/
│   ├── forest_rover_perception/
│   │   ├── yolo_detector_node/
│   │   └── color_tracker_node/
│   ├── forest_rover_autonomy/
│   │   └── autonomy_manager/            # State machine
│   ├── forest_rover_description/        # URDF + TF
│   ├── forest_rover_msgs/               # Custom messages
│   └── forest_rover_utils/              # Tools, diagnostics
│
├── docs/
│   ├── ROS_ARCHITECTURE.md
│   ├── FIRMWARE_ARCHITECTURE.md
│   ├── MOTOR_TUNING.md
│   ├── YOLO_FINETUNING.md
│   ├── DEPLOYMENT.md
│   └── SAFETY.md
│
├── docker/
│   └── Dockerfile.arm64
│
├── hardware/                            # (moved to hardware/rev-a)
├── simulation/                          # (moved to hardware/rev-a)
└── IMPLEMENTATION_PLAN.md               # Full detailed plan
```

---

## 🚀 Immediate Next Steps

### Week 1 - Priority Actions
```
[ ] 1. Create hardware/rev-a branch (preserve all PCB work)
[ ] 2. Reset main branch for ROS development
[ ] 3. Initialize ROS2 workspace:
       mkdir -p ros2_workspace/src
       cd ros2_workspace
       colcon build
[ ] 4. Create feature/phase-0-setup branch
[ ] 5. Set up STM32CubeIDE project + toolchain
[ ] 6. Create .github/workflows/ci-ros-firmware.yml
[ ] 7. Write Phase 0 completion checklist
```

### Week 1 - Parallel Work Streams
```
Stream A (Infrastructure)    Stream B (Firmware Init)
├─ ROS workspace setup       ├─ STM32CubeIDE project
├─ Docker ARM64 builder      ├─ Clock/GPIO configuration
├─ GitHub Actions CI/CD      ├─ FreeRTOS kernel integration
└─ Architecture docs         └─ UART + COBS framing
```

---

## ✅ Success Criteria (MVP)

- [x] Autonomous patrol with nav2 (no manual steering)
- [x] YOLOv8 detects animals @ ≥15 FPS, ≥60% mAP
- [x] Smoke sensor alerts on fire
- [x] Red ball tracking in 3-5m range
- [x] 2+ hour battery operation
- [x] Full telemetry logging (rosbag2)
- [x] Safe E-stop + watchdog functional
- [x] Production-ready GitHub repo + CI/CD

---

## 📈 Risk Mitigation

| Risk | Severity | Mitigation |
|------|----------|-----------|
| RPi CM4 shortage | HIGH | Pre-order; test on available hardware |
| YOLOv8 FPS | HIGH | Profile early Phase 3.1; quantization option |
| Motor PID tuning | MEDIUM | Automated tuning script; reference params |
| SLAM accuracy | MEDIUM | Odometry-only OK; LiDAR upgrade path |
| UART reliability | MEDIUM | Watchdog + retransmission in protocol |
| Field durability | MEDIUM | Weatherproofing; thermal testing |

---

## 🔗 Quick Links

- **Full Plan**: [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md)
- **ROS 2 Humble Docs**: https://docs.ros.org/en/humble/
- **nav2 Docs**: https://docs.nav2.org/
- **YOLOv8 Repo**: https://github.com/ultralytics/ultralytics
- **STM32 HAL**: https://www.st.com/
- **FreeRTOS**: https://www.freertos.org/

---

**Plan Version**: 1.0  
**Status**: Approved for Phase 0 Kickoff  
**Date**: March 31, 2026
