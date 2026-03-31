#ifndef SENSOR_DRIVERS_H
#define SENSOR_DRIVERS_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

typedef struct {
    float temperature_c;
    float humidity_percent;
    float pressure_hpa;
    float smoke_ppm;
    bool motion_detected;
} sensor_snapshot_t;

void sensor_drivers_init(I2C_HandleTypeDef *i2c, ADC_HandleTypeDef *adc);
void sensor_drivers_poll_nonblocking(void);
sensor_snapshot_t sensor_drivers_get_latest(void);

#endif
