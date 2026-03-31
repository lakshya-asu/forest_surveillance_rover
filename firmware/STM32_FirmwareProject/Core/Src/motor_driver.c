#include "motor_driver.h"

#include "config.h"

static TIM_HandleTypeDef *s_pwm_timer = NULL;
static motor_state_t s_state = {0};

static float s_kp = 0.10f;

void motor_driver_init(TIM_HandleTypeDef *pwm_timer) {
    s_pwm_timer = pwm_timer;
    s_state.left_rpm_target = 0.0f;
    s_state.right_rpm_target = 0.0f;
    s_state.watchdog_triggered = false;

    if (s_pwm_timer != NULL) {
        HAL_TIM_PWM_Start(s_pwm_timer, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(s_pwm_timer, TIM_CHANNEL_2);
    }
}

void motor_driver_set_target_rpm(float left_rpm, float right_rpm) {
    s_state.left_rpm_target = left_rpm;
    s_state.right_rpm_target = right_rpm;
    s_state.watchdog_triggered = false;
}

void motor_driver_apply_watchdog_stop(void) {
    s_state.left_rpm_target = 0.0f;
    s_state.right_rpm_target = 0.0f;
    s_state.watchdog_triggered = true;
    if (s_pwm_timer != NULL) {
        __HAL_TIM_SET_COMPARE(s_pwm_timer, TIM_CHANNEL_1, 0U);
        __HAL_TIM_SET_COMPARE(s_pwm_timer, TIM_CHANNEL_2, 0U);
    }
}

void motor_driver_update(float left_rpm_measured, float right_rpm_measured) {
    if (s_pwm_timer == NULL || s_state.watchdog_triggered) {
        return;
    }

    float left_error = s_state.left_rpm_target - left_rpm_measured;
    float right_error = s_state.right_rpm_target - right_rpm_measured;

    uint32_t period = __HAL_TIM_GET_AUTORELOAD(s_pwm_timer);
    float left_duty = 0.5f + (s_kp * left_error * 0.01f);
    float right_duty = 0.5f + (s_kp * right_error * 0.01f);

    if (left_duty < 0.0f) {
        left_duty = 0.0f;
    }
    if (left_duty > 1.0f) {
        left_duty = 1.0f;
    }
    if (right_duty < 0.0f) {
        right_duty = 0.0f;
    }
    if (right_duty > 1.0f) {
        right_duty = 1.0f;
    }

    __HAL_TIM_SET_COMPARE(s_pwm_timer, TIM_CHANNEL_1, (uint32_t)(left_duty * period));
    __HAL_TIM_SET_COMPARE(s_pwm_timer, TIM_CHANNEL_2, (uint32_t)(right_duty * period));
}

motor_state_t motor_driver_get_state(void) {
    return s_state;
}
