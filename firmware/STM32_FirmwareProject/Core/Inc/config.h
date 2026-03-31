#ifndef CONFIG_H
#define CONFIG_H

/* ===== MCU Configuration ===== */
#define STM32F407_CLOCK_FREQ_HZ    168000000U  // 168 MHz
#define SYSTICK_FREQ_HZ            1000U       // 1 kHz system tick

/* ===== UART Configuration ===== */
#define UART_ROS_BAUD_RATE         115200U
#define UART_ROS_BUFFER_SIZE        256U

/* ===== Motor Configuration ===== */
#define MOTOR_CONTROL_FREQ_HZ       100U        // 100 Hz control loop
#define MOTOR_PWM_FREQ_HZ           20000U      // 20 kHz PWM
#define MOTOR_A_PORT                GPIOB
#define MOTOR_A_PIN_PWM             GPIO_PIN_8
#define MOTOR_A_PIN_DIR             GPIO_PIN_9
#define MOTOR_B_PORT                GPIOB
#define MOTOR_B_PIN_PWM             GPIO_PIN_6
#define MOTOR_B_PIN_DIR             GPIO_PIN_7

/* ===== Encoder Configuration ===== */
#define ENCODER_CPR                 464U        // Counts per revolution
#define WHEEL_RADIUS_M              0.05f       // 5 cm wheels

/* ===== Sensor Configuration ===== */
#define BME280_I2C_ADDR             0x77U       // I2C address
#define BNO055_I2C_ADDR             0x29U
#define MQ2_ADC_CHANNEL             ADC_CHANNEL_0  // PA0

/* ===== Safety Configuration ===== */
#define ROS_HEARTBEAT_TIMEOUT_MS    500U        // Motor watchdog timeout
#define BATTERY_LOW_THRESHOLD_V     9.0f        // 3S LiPo low battery

/* ===== FreeRTOS Configuration ===== */
#define RTOS_TICK_RATE_HZ           1000U
#define MOTOR_TASK_PRIORITY         3
#define SENSOR_TASK_PRIORITY        2
#define UART_TASK_PRIORITY          1
#define SAFETY_TASK_PRIORITY        4

#endif /* CONFIG_H */
