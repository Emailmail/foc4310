#include "bsp_pwm.h"
#include "stm32g4xx_hal_rcc.h"
#include "stm32g4xx_hal_tim.h"

// ---------------- pwm 最底层操作 ----------------
/**
 * @brief 获取定时器时钟频率
 */
static uint32_t bsp_pwm_gettimclk(TIM_TypeDef *instance) {
    uint32_t pclk;
    uint32_t ppre;

    if ((uint32_t)instance < APB2PERIPH_BASE) {
        pclk = HAL_RCC_GetPCLK1Freq();
        ppre = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
    } else {
        pclk = HAL_RCC_GetPCLK2Freq();
        ppre = (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;
    }

    // APB 预分频器 > 1 时, 定时器时钟 = 2 * PCLK
    if (ppre >= 4) {
        return pclk * 2;
    }
    return pclk;
}

// ---------------- pwm 用户函数 ----------------
/**
 * @brief 注册 PWM
 */
void bsp_pwm_register(bsp_pwm_t *pwm, TIM_HandleTypeDef *htim, uint32_t channel) {
    pwm->htim = htim;
    pwm->channel = channel;
    pwm->arr = htim->Instance->ARR;
    pwm->freq = bsp_pwm_gettimclk(htim->Instance) / (htim->Instance->PSC + 1) / (pwm->arr + 1);
}

/**
 * @brief 启动 PWM 输出
 */
void bsp_pwm_start(bsp_pwm_t *pwm) {
    HAL_TIM_PWM_Start(pwm->htim, pwm->channel);
    HAL_TIMEx_PWMN_Start(pwm->htim, pwm->channel); // 同时使能 CCxE/CCxNE/MOE/CEN
}

/**
 * @brief 停止 PWM 输出
 */
void bsp_pwm_stop(bsp_pwm_t *pwm) {
    HAL_TIM_PWM_Stop(pwm->htim, pwm->channel);
}

/**
 * @brief 设置 PWM 占空比
 * @param ccr 比较值 (0 ~ ARR)
 */
void bsp_pwm_setccr(bsp_pwm_t *pwm, uint16_t ccr) {
    __HAL_TIM_SET_COMPARE(pwm->htim, pwm->channel, ccr);
}
