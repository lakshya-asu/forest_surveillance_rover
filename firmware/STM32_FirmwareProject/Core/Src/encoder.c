#include "encoder.h"

static TIM_HandleTypeDef *s_left_timer = NULL;
static TIM_HandleTypeDef *s_right_timer = NULL;
static int32_t s_left_ticks = 0;
static int32_t s_right_ticks = 0;

void encoder_init(TIM_HandleTypeDef *left_timer, TIM_HandleTypeDef *right_timer) {
    s_left_timer = left_timer;
    s_right_timer = right_timer;
    s_left_ticks = 0;
    s_right_ticks = 0;

    if (s_left_timer != NULL) {
        HAL_TIM_Encoder_Start_IT(s_left_timer, TIM_CHANNEL_ALL);
    }
    if (s_right_timer != NULL) {
        HAL_TIM_Encoder_Start_IT(s_right_timer, TIM_CHANNEL_ALL);
    }
}

void encoder_update_from_isr(void) {
    if (s_left_timer != NULL) {
        s_left_ticks = (int32_t)__HAL_TIM_GET_COUNTER(s_left_timer);
    }
    if (s_right_timer != NULL) {
        s_right_ticks = (int32_t)__HAL_TIM_GET_COUNTER(s_right_timer);
    }
}

encoder_ticks_t encoder_get_ticks(void) {
    encoder_ticks_t ticks;
    ticks.left_ticks = s_left_ticks;
    ticks.right_ticks = s_right_ticks;
    return ticks;
}
