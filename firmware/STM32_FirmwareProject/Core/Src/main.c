/**
 * @file main.c
 * @brief STM32F407 Real-Time Firmware Entry Point
 *
 * Initializes STM32 HAL, FreeRTOS kernel, and spawns all real-time tasks
 * for Forest Surveillance Rover autonomous robot.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "config.h"
#include "encoder.h"
#include "motor_driver.h"
#include "sensor_drivers.h"
#include "stm32f4xx_hal.h"
#include "uart_bridge.h"
#include "FreeRTOS.h"
#include "task.h"

/* ===== Global Peripheral Handles ===== */
UART_HandleTypeDef huart1;
I2C_HandleTypeDef hi2c1;
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim3;
static uint8_t g_uart_rx_byte = 0;
static volatile TickType_t g_last_cmd_tick = 0;
static volatile bool g_estop_latched = false;

/* ===== Forward Declarations ===== */
static void SystemClockConfig(void);
static void MX_GPIO_Init(void);
static void MX_UART1_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);  // Encoder
static void MX_TIM5_Init(void);  // Encoder
static void MX_TIM3_Init(void);  // Motor PWM

/* FreeRTOS Task Prototypes */
void vMotorControlTask(void *pvParameters);
void vSensorAcquisitionTask(void *pvParameters);
void vUARTRoSBridgeTask(void *pvParameters);
void vSafetyWatchdogTask(void *pvParameters);

/**
 * @brief Main entry point
 */
int main(void) {
    /* Initialize HAL Library */
    HAL_Init();

    /* Configure System Clock */
    SystemClockConfig();

    /* Initialize Peripherals */
    MX_GPIO_Init();
    MX_UART1_Init();        // ROS UART bridge
    MX_I2C1_Init();         // Sensors (BME280, BNO055)
    MX_ADC1_Init();         // Gas sensor, voltage dividers
    MX_TIM2_Init();         // Encoder input capture
    MX_TIM5_Init();         // Encoder input capture
    MX_TIM3_Init();         // Motor A/B PWM output
    g_last_cmd_tick = xTaskGetTickCount();

    /* Print boot message */
    printf("========== Forest Rover STM32F407 Firmware ==========\r\n");
    printf("System Clock: %lu Hz\r\n", SystemCoreClock);
    printf("FreeRTOS Heap: %lu bytes\r\n", configTOTAL_HEAP_SIZE);

    /* Create FreeRTOS Tasks */
    xTaskCreate(
        vMotorControlTask,
        "MotorCtrlTask",
        configMINIMAL_STACK_SIZE + 256,
        NULL,
        MOTOR_TASK_PRIORITY,
        NULL
    );

    xTaskCreate(
        vSensorAcquisitionTask,
        "SensorTask",
        configMINIMAL_STACK_SIZE + 128,
        NULL,
        SENSOR_TASK_PRIORITY,
        NULL
    );

    xTaskCreate(
        vUARTRoSBridgeTask,
        "UARTTask",
        configMINIMAL_STACK_SIZE + 256,
        NULL,
        UART_TASK_PRIORITY,
        NULL
    );

    xTaskCreate(
        vSafetyWatchdogTask,
        "SafetyTask",
        configMINIMAL_STACK_SIZE + 128,
        NULL,
        SAFETY_TASK_PRIORITY,
        NULL
    );

    /* Start Scheduler */
    printf("Starting FreeRTOS Scheduler...\r\n");
    vTaskStartScheduler();

    /* Should not reach here */
    while (1);
    return 0;
}

/**
 * @brief System Clock Configuration (168 MHz from internal HSI or external HSE)
 */
static void SystemClockConfig(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Configure the main internal regulator output voltage */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Initializes the RCC Oscillators */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        while (1);
    }

    /* Initializes the CPU, AHB and APB busses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        while (1);
    }

    /* Configure SysTick for 1 kHz */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / SYSTICK_FREQ_HZ);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}

/**
 * @brief GPIO Initialization (motors, encoders, safety pins)
 */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configure GPIO pins for motor drivers (direction pins) */
    GPIO_InitStruct.Pin = MOTOR_A_PIN_DIR | MOTOR_B_PIN_DIR;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MOTOR_A_PORT, &GPIO_InitStruct);

    /* TODO: Configure encoder input pins, LED pins, button pins */
}

/**
 * @brief UART1 Initialization (115200 baud for ROS bridge)
 */
static void MX_UART1_Init(void) {
    huart1.Instance = USART1;
    huart1.Init.BaudRate = UART_ROS_BAUD_RATE;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart1) != HAL_OK) {
        while (1);
    }

    uart_bridge_init(&huart1);
    HAL_UART_Receive_IT(&huart1, &g_uart_rx_byte, 1);
}

/**
 * @brief I2C1 Initialization (100 kHz for BME280, BNO055)
 */
static void MX_I2C1_Init(void) {
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        while (1);
    }
}

/**
 * @brief ADC1 Initialization (gas sensor, battery voltage)
 */
static void MX_ADC1_Init(void) {
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        while (1);
    }
}

/**
 * @brief Timer2 Initialization (encoder input capture)
 */
static void MX_TIM2_Init(void) {
    TIM_Encoder_InitTypeDef sEncoderConfig = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFFFFFF;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sEncoderConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sEncoderConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sEncoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sEncoderConfig.IC1Filter = 0;
    sEncoderConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sEncoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sEncoderConfig.IC2Filter = 0;

    if (HAL_TIM_Encoder_Init(&htim2, &sEncoderConfig) != HAL_OK) {
        while (1);
    }
}

/**
 * @brief Timer5 Initialization (encoder input capture)
 */
static void MX_TIM5_Init(void) {
    TIM_Encoder_InitTypeDef sEncoderConfig = {0};

    htim5.Instance = TIM5;
    htim5.Init.Prescaler = 0;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 0xFFFFFFFF;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sEncoderConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sEncoderConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sEncoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sEncoderConfig.IC1Filter = 0;
    sEncoderConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sEncoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sEncoderConfig.IC2Filter = 0;

    if (HAL_TIM_Encoder_Init(&htim5, &sEncoderConfig) != HAL_OK) {
        while (1);
    }
}

/**
 * @brief Timer3 Initialization (motor PWM output)
 */
static void MX_TIM3_Init(void) {
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = (SystemCoreClock / MOTOR_PWM_FREQ_HZ) - 1;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
        while (1);
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
}

/* ===== FreeRTOS Task Implementations ===== */

/**
 * @brief Motor Control Task (100 Hz)
 */
void vMotorControlTask(void *pvParameters) {
    (void)pvParameters;
    encoder_init(&htim2, &htim5);
    motor_driver_init(&htim3);

    while (1) {
        encoder_update_from_isr();
        encoder_ticks_t ticks = encoder_get_ticks();
        float left_rpm = (float)ticks.left_ticks * 0.1f;
        float right_rpm = (float)ticks.right_ticks * 0.1f;
        motor_driver_update(left_rpm, right_rpm);
        vTaskDelay(pdMS_TO_TICKS(10));  // 100 Hz (10ms)
    }
}

/**
 * @brief Sensor Acquisition Task (50 Hz)
 */
void vSensorAcquisitionTask(void *pvParameters) {
    (void)pvParameters;
    sensor_drivers_init(&hi2c1, &hadc1);

    while (1) {
        sensor_drivers_poll_nonblocking();
        vTaskDelay(pdMS_TO_TICKS(20));  // 50 Hz (20ms)
    }
}

/**
 * @brief UART ROS Bridge Task
 */
void vUARTRoSBridgeTask(void *pvParameters) {
    (void)pvParameters;

    while (1) {
        uart_frame_t frame;
        if (uart_bridge_pop_frame(&frame)) {
            if (frame.msg_id == 0x01U && frame.payload_len >= 8U) {
                if (!g_estop_latched) {
                    float left_rpm = 0.0f;
                    float right_rpm = 0.0f;
                    memcpy(&left_rpm, &frame.payload[0], sizeof(float));
                    memcpy(&right_rpm, &frame.payload[4], sizeof(float));
                    motor_driver_set_target_rpm(left_rpm, right_rpm);
                    g_last_cmd_tick = xTaskGetTickCount();
                }
            } else if (frame.msg_id == 0x02U && frame.payload_len >= 1U) {
                if (frame.payload[0] != 0U) {
                    g_estop_latched = true;
                    motor_driver_apply_watchdog_stop();
                } else {
                    g_estop_latched = false;
                    motor_driver_set_target_rpm(0.0f, 0.0f);
                    g_last_cmd_tick = xTaskGetTickCount();
                }
            }
        }

        sensor_snapshot_t snapshot = sensor_drivers_get_latest();
        uint8_t payload[20] = {0};
        memcpy(&payload[0], &snapshot.temperature_c, sizeof(float));
        memcpy(&payload[4], &snapshot.humidity_percent, sizeof(float));
        memcpy(&payload[8], &snapshot.pressure_hpa, sizeof(float));
        memcpy(&payload[12], &snapshot.smoke_ppm, sizeof(float));
        payload[16] = snapshot.motion_detected ? 1U : 0U;
        uart_bridge_send_frame(0x20U, payload, 17U);

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/**
 * @brief Safety Watchdog Task (10 Hz)
 */
void vSafetyWatchdogTask(void *pvParameters) {
    (void)pvParameters;

    while (1) {
        TickType_t now = xTaskGetTickCount();
        TickType_t timeout_ticks = pdMS_TO_TICKS(ROS_HEARTBEAT_TIMEOUT_MS);
        if ((now - g_last_cmd_tick) > timeout_ticks) {
            motor_driver_apply_watchdog_stop();
        }

        motor_state_t state = motor_driver_get_state();
        if (state.watchdog_triggered) {
            motor_driver_apply_watchdog_stop();
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // 10 Hz (100ms)
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        uart_bridge_on_rx_byte(g_uart_rx_byte);
        HAL_UART_Receive_IT(&huart1, &g_uart_rx_byte, 1);
    }
}

/**
 * @brief Override HAL_Delay for compatibility with FreeRTOS
 */
void HAL_Delay(__IO uint32_t Delay) {
    vTaskDelay(pdMS_TO_TICKS(Delay));
}

/**
 * @brief FreeRTOS Idle Hook
 */
void vApplicationIdleHook(void) {
    /* Optional: Put MCU in low-power mode */
}

/**
 * @brief FreeRTOS Stack Overflow Hook
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void)xTask;
    (void)pcTaskName;
    /* Trap: Stack overflow detected */
    while (1);
}

/* Printf redirect to UART */
int _write(int file, char *ptr, int len) {
    (void)file;
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}
