#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

typedef struct {
    float left_rpm_target;
    float right_rpm_target;
    bool watchdog_triggered;
} motor_state_t;

void motor_driver_init(TIM_HandleTypeDef *pwm_timer);
void motor_driver_set_target_rpm(float left_rpm, float right_rpm);
void motor_driver_apply_watchdog_stop(void);
void motor_driver_update(float left_rpm_measured, float right_rpm_measured);
motor_state_t motor_driver_get_state(void);

#endif
