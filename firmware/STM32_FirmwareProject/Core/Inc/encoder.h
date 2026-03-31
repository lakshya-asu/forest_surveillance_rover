#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#include "stm32f4xx_hal.h"

typedef struct {
    int32_t left_ticks;
    int32_t right_ticks;
} encoder_ticks_t;

void encoder_init(TIM_HandleTypeDef *left_timer, TIM_HandleTypeDef *right_timer);
void encoder_update_from_isr(void);
encoder_ticks_t encoder_get_ticks(void);

#endif
