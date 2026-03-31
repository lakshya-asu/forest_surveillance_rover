# STM32F407 Firmware Architecture - Forest Surveillance Rover

**Status**: Phase 0 Skeleton Complete  
**Target**: STM32F407VG (168 MHz, 1 MB Flash, 196 KB RAM)  
**RTOS**: FreeRTOS (optional for Phase 1+)

---

## System Overview

The STM32F407 firmware provides real-time control for motors, sensor acquisition, and UART communication with Raspberry Pi CM4 ROS 2 stack.

```
┌────────────────────────────────────────────┐
│  STM32F407 Real-Time Firmware             │
│  Clock: 168 MHz | RTOS: FreeRTOS          │
├────────────────────────────────────────────┤
│                                            │
│  MotorControlTask (100 Hz, Priority 3)    │
│  ├─ Read encoders (ISR-captured)          │
│  ├─ PID velocity controller               │
│  └─ PWM drive motors (DRV8871)            │
│                                            │
│  SensorAcqTask (50 Hz, Priority 2)        │
│  ├─ Poll BME280 (I2C non-blocking)        │
│  ├─ Poll BNO055 (I2C non-blocking)        │
│  ├─ Read MQ-2 ADC                         │
│  └─ Queue data to UART                    │
│                                            │
│  UARTRoSBridgeTask (Variable, Priority 1) │
│  ├─ RX: Motor commands from RPi           │
│  ├─ TX: Sensor data (COBS framed)         │
│  ├─ COBS encode/decode                    │
│  └─ CRC-16 checksum validation            │
│                                            │
│  SafetyWatchdogTask (10 Hz, Priority 4)   │
│  ├─ Detect ROS heartbeat loss             │
│  ├─ Monitor battery voltage               │
│  ├─ Check motor stall                     │
│  └─ Emergency stop handling               │
│                                            │
└────────────────────────────────────────────┘
       │ UART1 (115200)
       ├─ DRV8871 (motor drivers)
       ├─ I2C1 (sensors)
       ├─ ADC1 (analog)
       └─ GPIO (encoders, LEDs, buttons)
```

---

## 1. Task Scheduler (FreeRTOS)

### Task Priorities (Higher = More Important)

| Priority | Task | Frequency | Deadline | Notes |
|----------|------|-----------|----------|-------|
| 4 | SafetyWatchdog | 10 Hz | 100 ms | E-stop, watchdog timeout |
| 3 | MotorControl | 100 Hz | 10 ms | Hard real-time, PID loop |
| 2 | SensorAcq | 50 Hz | 20 ms | Non-blocking I2C state machine |
| 1 | UARTRoS | Variable | - | Low priority, event-driven |

### Context Switching

- Total CPU time < 75% at full load
- Idle task handles low-power sleep (optional)
- No busy-waiting; all tasks use `vTaskDelay()`

---

## 2. Motor Control Subsystem

### Hardware Interface

| Component | Interface | MCU Pin | Notes |
|-----------|-----------|---------|-------|
| Motor A PWM | TIM3_CH1 | PB4 (AF2) | 20 kHz PWM, 0-100% duty |
| Motor A DIR | GPIO | PB9 | Direction control |
| Motor B PWM | TIM3_CH2 | PB5 (AF2) | 20 kHz PWM, 0-100% duty |
| Motor B DIR | GPIO | PB7 | Direction control |
| Encoder A | TIM2_CH1 | PA0 (AF1) | Quadrature decoder |
| Encoder B | TIM2_CH2 | PA1 (AF1) | Quadrature decoder |

### Control Flow

```
Motor Command (ROS service)
    ↓
UARTRoSBridgeTask decodes command
    ↓
Store in motor_target_velocity[2]
    ↓
MotorControlTask (10 ms loop)
    ├─ Read encoder counts from ISR
    ├─ Calculate actual velocity (rpm → m/s)
    ├─ Compute error: target - actual
    ├─ PID update: output = Kp*e + Ki*∫e + Kd*de/dt
    ├─ Clamp PWM to ±100% + margin
    └─ Write PWM to TIM3
       
Safety watchdog:
    If no command received > 500 ms → set speed = 0
```

### PID Tuning Parameters

```c
typedef struct {
  float Kp;              // Proportional gain
  float Ki;              // Integral gain
  float Kd;              // Derivative gain
  float setpoint;        // Target velocity (m/s)
  float accumulated_error;
  float last_error;
  uint32_t last_time;
} PIDController_t;

// Initial defaults (will be tuned in Phase 1.2)
#define DEFAULT_KP 1.0f
#define DEFAULT_KI 0.1f
#define DEFAULT_KD 0.01f
```

### Encoder Feedback

```c
// Encoder resolution
#define ENCODER_CPR 464         // Counts per revolution
#define WHEEL_RADIUS 0.05f      // 5 cm

// Calculate velocity from encoder counts
float encoder_to_velocity(uint32_t counts, uint32_t dt_ms) {
  float rpm = (counts * 1000) / (ENCODER_CPR * dt_ms);
  float velocity_mps = rpm * WHEEL_RADIUS * (3.14159 / 30);  // m/s
  return velocity_mps;
}
```

---

## 3. Sensor Acquisition Subsystem

### Hardware Interfaces

| Sensor | Interface | Address | Rate | Data |
|--------|-----------|---------|------|------|
| **BME280** | I2C1 | 0x77 | 50 Hz | Temp, pressure, humidity |
| **BNO055** | I2C1 | 0x29 | 100 Hz | Accel, gyro, mag, quaternion |
| **MQ-2** | ADC1_CH0 | PA0 | 10 Hz | Raw ADC value → ppm |
| **Battery** | ADC1_CH1 | PA1 | 10 Hz | Voltage divider monitoring |

### Non-Blocking I2C State Machine

```c
typedef enum {
  I2C_IDLE,
  I2C_BME280_REQ,
  I2C_BME280_WAIT,
  I2C_BME280_PARSE,
  I2C_BNO055_REQ,
  I2C_BNO055_WAIT,
  I2C_BNO055_PARSE,
  I2C_DONE
} I2CState_t;

// SensorAcquisitionTask polls each state
void vSensorAcquisitionTask(void *pvParameters) {
  while (1) {
    switch (i2c_state) {
      case I2C_IDLE:
        i2c_state = I2C_BME280_REQ;
        // Non-blocking I2C read initiated
        HAL_I2C_Mem_Read_IT(&hi2c1, BME280_ADDR, REG, SIZE, buf, LEN);
        break;
      case I2C_BME280_WAIT:
        // ISR sets done flag asynchronously
        if (i2c_done_flag) {
          ParseBME280(buf, &bme280_data);
          i2c_state = I2C_BNO055_REQ;
        }
        break;
      // Similar for BNO055...
      case I2C_DONE:
        PublishSensorData();
        i2c_state = I2C_IDLE;
        break;
    }
    vTaskDelay(pdMS_TO_TICKS(20));  // 50 Hz polling
  }
}
```

### Sensor Data Structure

```c
typedef struct {
  // BME280
  float temperature_c;
  float pressure_pa;
  float humidity_rh;
  
  // BNO055
  float accel_x, accel_y, accel_z;        // m/s²
  float gyro_x, gyro_y, gyro_z;           // rad/s
  float mag_x, mag_y, mag_z;              // Gauss
  float quaternion_w, qx, qy, qz;         // Quatern
  float roll_rad, pitch_rad, yaw_rad;     // Euler (computed from quat)
  
  // MQ-2 Gas Sensor
  uint16_t mq2_adc_raw;
  float ppm_co;
  float ppm_smoke;
  bool smoke_alert;
  
  // Timestamp
  uint32_t timestamp_us;
} SensorData_t;
```

---

## 4. UART Communication Protocol

### Serial Frame Format (COBS)

COBS (Consistent Overhead Byte Stuffing) ensures no special bytes in payload.

```
[COBS_ENCODED_DATA][DELIMITER]
     variable        0x00 (COBS delimiter)
```

### Message Structure

```c
typedef struct {
  uint8_t message_type;     // 0x10=sensor, 0x20=motor cmd, etc.
  uint16_t length;          // Payload size (little-endian)
  uint8_t payload[256];     // Variable length
  uint16_t crc16_ccitt;     // CRC-16 CCITT (calculated on type+length+payload)
} SerialMessage_t;
```

### Heartbeat & Watchdog

```
RPi (every 100ms)
    │
    ├─ MOTOR_CMD with non-zero speed
    │
STM32 (motor watchdog timer)
    ├─ Reset timer on valid command
    ├─ If timeout (500ms) → set speed = 0, motors stop
    └─ Log safety event
```

---

## 5. Safety & Fault Handling

### Watchdog Timer

- **Hardware Watchdog**: Independent watchdog (IWDG) with 4s timeout
- **Software Watchdog**: ROS heartbeat timeout (500 ms)
- **Stall Detection**: Motor current limit via DRV8871 FB pin

### Fault Response

```c
void vSafetyWatchdogTask(void *pvParameters) {
  uint32_t last_heartbeat = xTaskGetTickCount();
  
  while (1) {
    uint32_t now = xTaskGetTickCount();
    uint32_t elapsed = now - last_heartbeat;
    
    // Check for ROS heartbeat loss
    if (elapsed > pdMS_TO_TICKS(ROS_HEARTBEAT_TIMEOUT_MS)) {
      SetMotorSpeed(0, 0);  // Emergency stop
      SetLED(RED);          // Alert LED
      LogSafetyEvent("ROS heartbeat lost");
      // Stay stopped until POR or explicit reset
    }
    
    // Check battery voltage
    if (battery_voltage < BATTERY_LOW_THRESHOLD_V) {
      LogSafetyEvent("Battery low");
      // Continue operation but log warning
    }
    
    // Reset hardware watchdog
    HAL_IWDG_Refresh(&hiwdg);
    
    vTaskDelay(pdMS_TO_TICKS(100));  // 10 Hz check
  }
}
```

---

## 6. Memory Layout

### Flash (1 MB = 1024 KB)

```
0x00000000 - 0x0003FFFF  (256 KB) - Bootloader (if DFU enabled)
0x00040000 - 0x00FBFFFF  (768 KB) - Application code
0x00FC0000 - 0x00FFFFFF  (256 KB) - (Flash config, reserved)
```

### RAM (196 KB)

```
0x20000000 - 0x2002FFFF  (192 KB) - Data, FreeRTOS heap, stacks
0x20030000 - 0x20030FFF  (4 KB)   - DMA buffers
```

### FreeRTOS Heap Configuration

```c
#define configTOTAL_HEAP_SIZE (40 * 1024)  // 40 KB for tasks + queues
#define configMINIMAL_STACK_SIZE 128        // MCU-specific minimum
```

---

## 7. Build Instructions

### Prerequisites

```bash
# ARM GCC toolchain
apt install gcc-arm-none-eabi g++-arm-none-eabi arm-none-eabi-newlib cmake

# OpenOCD (debugging)
apt install openocd

# STM32CubeMX (optional, for visual configuration)
# Download from https://www.st.com/stm32cubeide
```

### Build Process

```bash
cd firmware/STM32_FirmwareProject

mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# Generate binaries
ls -lh forest-rover.{elf,hex,bin}
```

### Flash Firmware

#### Option 1: OpenOCD + GDB

```bash
# Terminal 1: Start OpenOCD
openocd -f Tools/openocd/forest-rover-stm32f407.cfg

# Terminal 2: GDB debug session
arm-none-eabi-gdb build/forest-rover.elf
(gdb) target remote localhost:3333
(gdb) load forest-rover.elf
(gdb) monitor reset halt
(gdb) continue
```

#### Option 2: STLink CLI (if using ST-Link)

```bash
st-flash write build/forest-rover.bin 0x08000000
st-flash --reset
```

#### Option 3: UART DFU Bootloader (Phase 1)

```bash
# Via ROS service or Python script
dfu-util -i 0 -a 0 -D build/forest-rover.bin
```

---

## 8. Phase 0 Completion Status

- [x] CMakeLists.txt for ARM-GCC cross-compilation
- [x] main.c with FreeRTOS initialization
- [x] Config header with pin definitions
- [x] FreeRTOS task skeletons
- [x] Empty HAL init functions
- [ ] Complete HAL peripheral setup (Phase 1.1)
- [ ] Motor control PID loop (Phase 1.2)
- [ ] Sensor driver implementations (Phase 1.3)
- [ ] UART COBS protocol (Phase 1.4)

---

## 9. Performance Budget

### CPU Time

```
MotorControlTask:   ~2 ms (executing PID once per 10ms period)
SensorAcqTask:      ~5 ms (I2C wait + parsing)
UARTRoSTask:        ~1 ms (UART ISR + COBS decode)
SafetyTask:         ~0.5 ms (checks + watchdog)
──────────────────────────────
Total execution:    ~8.5 ms per 10ms frame
Utilization:        ~85% (headroom for ISRs)
```

### Memory

```
Code:               ~150 KB (STM32 HAL + FreeRTOS + app)
RAM Heap:           ~40 KB (tasks, queues, buffers)
Stack:              ~16 KB (4 tasks × 4 KB each)
Data/BSS:           ~5 KB (global variables)
──────────────────────────
Total usage:        ~210 KB (comfortable headroom)
```

---

## 10. Next Steps (Phase 1.1)

1. Complete STM32CubeMX initialization for clock, GPIO, UART1, I2C1
2. Integrate FreeRTOS kernel
3. Implement UART interrupt handler + COBS framing
4. Add encoder ISR quadrature decoder
5. Boot test on hardware + serial console verification

---

**Next Phase**: Phase 1.1 begins firmware development with core HAL + RTOS.
