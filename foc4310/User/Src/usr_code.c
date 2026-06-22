// ---------------- loop 必要头文件 ----------------
#include "usr_code.h"
#include "arm_math.h"
#include "bsp_spi.h"
#include "bsp_uart.h"
#include "spi.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"
#include "tim.h"

// ---------------- 根据需要添加其他模块的头文件 ----------------
#include "foc.h"
#include "as5047p.h"
#include "get_vol.h"
#include "usart.h"
#include "vofa.h"

// ---------------- 全局变量 ----------------
// bsp
bsp_pwm_t pwma, pwmb, pwmc;
bsp_spi_t as5047p_spi;
bsp_uart_t vofa_uart;
// hardware & algorithm
foc_t foc;
as5047p_t as5047p;
vofa_t vofa;
// else
float theta_virtual = 0.0f; // 电角度模拟变量
float mech_angle = 0.0f; // 机械角度变量
float elec_angle = 0.0f;
float vofa_tx_data[4] = {0}; // VOFA 接收数据数组

uint32_t cali_delay = 20000;
uint32_t cali_count = 0;
float cali_sum = 0.0f;
float cali_avg = 0.0f;

/**
 * @brief main() 初始化函数
 */
void setup(void) {
    // bsp 注册
    bsp_pwm_register(&pwma, &htim1, TIM_CHANNEL_1);
    bsp_pwm_register(&pwmb, &htim1, TIM_CHANNEL_2);
    bsp_pwm_register(&pwmc, &htim1, TIM_CHANNEL_3);
    bsp_spi_register(&as5047p_spi, &hspi1, GPIOA, GPIO_PIN_15, &as5047p, NULL);
    bsp_uart_register(&vofa_uart, &huart1, &vofa, NULL);
    vofa_register(&vofa, &vofa_uart);
    bsp_uart_rx_start(&vofa_uart); // 启动 DMA 接收

    // hardware 注册
    as5047p_register(&as5047p, &as5047p_spi, 0.01f);
    get_vbus_start_dma();  // 启动 ADC2 连续 DMA

    // algorithm 注册
    foc_register(&foc, &pwma, &pwmb, &pwmc, vbus_vol);
    foc_init(&foc); // 启动 FOC 硬件

    // 启动硬件
    HAL_TIM_Base_Start_IT(&htim6); // 启动定时器中断
}

/**
 * @brief main() 的 while(1) 循环函数
 */
void loop(void) {
    vofa_tx_data[0] = mech_angle; // 机械角度
    vofa_tx_data[1] = elec_angle; // 电角度
    vofa_tx_data[2] = theta_virtual; // 电角度模拟
    vofa_tx_data[3] = elec_angle - theta_virtual;
    vofa_send(&vofa, vofa_tx_data, 4);
    HAL_Delay(1);
}

/**
 * @brief TIM 定时中断 (0.1ms) 回调函数
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if(htim->Instance == TIM6) {
        mech_angle = as5047p_read_angle(&as5047p); // 读取机械角度
        elec_angle = (_2PI - mech_angle) * 14.0f - 4.21829319f;
        // elec_angle = (_2PI - mech_angle) * 14.0f;

        // theta_virtual += 0.0005f; // 电角度模拟
        // theta_virtual = (theta_virtual >  (28 * PI) ? theta_virtual - (28 * PI) : theta_virtual);

        foc_openloop_virtual(&foc, 0.0f, 1.0f, elec_angle);

        // if(cali_delay == 0) {
        //     if(cali_count < 50000) {
        //         if(elec_angle - theta_virtual > 0.0f  && elec_angle - theta_virtual < 6.29f) {
        //             cali_sum += elec_angle - theta_virtual;
        //             cali_count ++;
        //         }
        //     }
        //     else {
        //         cali_avg = cali_sum / 50000.0f;
        //     }
        // }
        // else {
        //     cali_delay --;
        // }
    }   
}
