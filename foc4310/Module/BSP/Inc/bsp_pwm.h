#ifndef _BSP_PWM_H
#define _BSP_PWM_H
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_tim.h"
#include <stdint.h>

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint16_t arr;
    
    uint32_t freq;
} bsp_pwm_t;

void bsp_pwm_register(bsp_pwm_t *pwm, TIM_HandleTypeDef *htim, uint32_t channel);
void bsp_pwm_start(bsp_pwm_t *pwm);
void bsp_pwm_stop(bsp_pwm_t *pwm);
void bsp_pwm_setccr(bsp_pwm_t *pwm, uint16_t ccr);
#endif
