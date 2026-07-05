// ---------------- loop 必要头文件 ----------------
#include "usr_code.h"
#include "bsp_spi.h"
#include "bsp_uart.h"
#include "spi.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"
#include "tim.h"

// ---------------- 根据需要添加其他模块的头文件 ----------------
#include "foc.h"
#include "as5047p.h"
#include "usart.h"
#include "vofa.h"
#include "get_vol.h"
#include "as5047p_cali.h"
#include "vofa_cdc.h"
// ---------------- 全局变量 ----------------
// bsp
bsp_pwm_t pwmu, pwmv, pwmw;
bsp_spi_t as5047p_spi;
bsp_uart_t vofa_uart;
// hardware & algorithm
foc_t foc;
as5047p_t as5047p;
vofa_t vofa;
// else
float speed = 0.0f; // 机械角速度变量
float mech_angle = 0.0f; // 机械角度变量
float vofa_tx_data[8] = {0}; // VOFA 接收数据数组

float Id = 0.0f;
float Iq = 0.0f;
float radpersec = 0.0f;

/**
 * @brief main() 初始化函数
 */
void setup(void) {
    get_vbus_start_dma();  // 获取母线电压

    // bsp 注册
    bsp_pwm_register(&pwmu, &htim1, TIM_CHANNEL_3); // FOC PWM U
    bsp_pwm_register(&pwmv, &htim1, TIM_CHANNEL_2); // FOC PWM V
    bsp_pwm_register(&pwmw, &htim1, TIM_CHANNEL_1); // FOC PWM W
    bsp_spi_register(&as5047p_spi, &hspi1, GPIOA, GPIO_PIN_15, &as5047p, NULL);    // AS5047P SPI
    bsp_uart_register(&vofa_uart, &huart1, &vofa, NULL);    // VOFA UART

    // hardware 注册
    as5047p_register(&as5047p, &as5047p_spi, 0.3f);    // AS5047P 注册
    // vofa_register(&vofa, &vofa_uart);   // VOFA 注册

    // vofa cdc
    vofa_cdc_rx_bind(0, &Id);
    vofa_cdc_rx_bind(1, &foc.speed_pid.Ki);
    vofa_cdc_rx_bind(2, &radpersec);
    vofa_cdc_rx_bind(3, &foc.speed_pid.Kp);

    // algorithm 注册
    foc_register(&foc, &pwmu, &pwmv, &pwmw, vbus_vol, 14, 4.09710753);  // FOC 注册
    foc_setpid_param(&foc, 30.0f, 1.0f, 30.0f, 1.0f);
    foc_setpid_outlimit(&foc, 7.0f, 7.0f);
    foc_setpid_intelimit(&foc, 10.0f, 10.0f);
    foc_setpid_speed(&foc, 0.03950f, 0.000800f, 3.0f, 1000.0f);  // 速度环 PID
    foc_settorquefeedforward(&foc, 0.073f, 0.1f, 0.015f, 0.016f);
    foc_init(&foc); // FOC 初始化

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);   // ADC1 注入通道触发信号
    HAL_Delay(50);
    get_current_offset(); // 获取电机三相电流偏置
    get_current_start(); // 获取电流

    // 启动硬件
    HAL_TIM_Base_Start_IT(&htim6); // 启动定时器中断
}

/**
 * @brief main() 的 while(1) 循环函数
 */
void loop(void) {
    
    HAL_Delay(1);
}

/**
 * @brief TIM 定时中断 (0.1ms) 回调函数
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if(htim->Instance == TIM6) {
        vofa_tx_data[0] = foc.speed_pid.pout;
        vofa_tx_data[1] = foc.speed_pid.iout;
        vofa_tx_data[2] = foc.torque_feedforward;
        vofa_tx_data[3] = speed;
        vofa_tx_data[4] = foc.Iab.b;
        vofa_tx_data[5] = foc.Idq.d;
        vofa_tx_data[6] = foc.Idq.q;
        vofa_tx_data[7] = mech_angle;

        vofa_cdc_send(vofa_tx_data, 8);
    }
}

/**
 * @brief ADC 注入通道转换完成回调函数
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) {
        get_current_update();
        mech_angle = as5047p_read_angle(&as5047p); // 读取机械角度
        as5047p_getspeed();          // 获取机械角速度
        foc_update(&foc, mech_angle, Iu, Iv, Iw, speed); // 更新 FOC 传感器数据
        // foc_openloop_svpwm(&foc, 0.0f, 1.0f); // FOC 开环控制 (SVPWM)
        // foc_setcurrent(&foc, Id, Iq);
        foc_setspeed(&foc, radpersec, 0.0f); // FOC 速度控制
    }
}
