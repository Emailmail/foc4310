// ---------------- loop 必要头文件 ----------------
#include "usr_code.h"
#include "bsp_spi.h"
#include "bsp_uart.h"
#include "spi.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"
#include "tim.h"
#include "fdcan.h"

// ---------------- 根据需要添加其他模块的头文件 ----------------
#include "foc.h"
#include "as5047p.h"
#include "usart.h"
#include "vofa.h"
#include "get_vol.h"
#include "as5047p_cali.h"
#include "vofa_cdc.h"
#include "communicate.h"
#include "bsp_fdcan.h"
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
float raw_angle = 0.0f;      // 原始编码器角度 [rad]
float vofa_tx_data[8] = {0}; // VOFA 发送数据数组

float Id = 0.0f;
float Iq = 0.0f;
float radpersec = 0.0f;
float pos = 0.0f;

// CAN 通信
foc_mode_t foc_mode = FOC_MODE_POSITION;
uint8_t foc_enabled = 0;
bsp_fdcan_t fdcan_dev = { .hfdcan = &hfdcan1 };
        communicate_t comm;

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

    // asp5047p 注册
    as5047p_register(&as5047p, &as5047p_spi);

    // vofa 注册
    vofa_register(&vofa, &vofa_uart);
    vofa_cdc_rx_bind(0, &radpersec);
    vofa_cdc_rx_bind(1, &pos);
    vofa_cdc_rx_bind(2, &foc.speed_pid.Kp);
    vofa_cdc_rx_bind(3, &foc.speed_pid.Ki);
    vofa_cdc_rx_bind(4, &foc.speed_pid.inte_lim);
    vofa_cdc_rx_bind(5, &foc.speed);

    // foc 注册
    foc_register(&foc, &pwmu, &pwmv, &pwmw, vbus_vol, 14, 4.09710753, 0.00005f);  // FOC 注册, dt=50us
    foc_lpf_init(&foc, 1500.0f, 100.0f);  // 电流 Fc=1500Hz, 速度 Fc=100Hz
    foc_setpid_currentparam(&foc, 9.6f, 0.35f, 11.2f, 0.35f);
    foc_setpid_currentoutlimit(&foc, 7.0f, 7.0f);
    foc_setpid_currentintelimit(&foc, 100.0f, 100.0f);
    foc_setpid_speed(&foc, 0.045f, 0.00015f, 3.0f, 1000.0f);  // 速度环 PID
    foc_setpid_position(&foc, 60.0f, 0.0001f, 30.0f, 100.0f);
    foc_init(&foc); // FOC 初始化

    // CAN 通信注册
    bsp_fdcan_init(&fdcan_dev);
    bsp_fdcan_register(&fdcan_dev, 0x400, &comm, communicate_rx_callback);
    fdcan_dev.tx_id = 0x500;
    communicate_init(&comm, &foc_enabled, &foc_mode,
                     &Iq, &radpersec, &pos,           // RX: CAN→控制目标
                     &foc.Idq.q, &foc.speed, &foc.mech_theta); // TX: 反馈→CAN
    foc_enabled = 1;

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);   // ADC1 注入通道触发信号
    HAL_Delay(50);
    get_current_offset(); // 获取电机三相电流偏置
    get_current_start(); // 获取电流
    foc_update(&foc, as5047p_read_angle(&as5047p), 0.0f, 0.0f, 0.0f);

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
 * @brief TIM 定时中断 (1ms) 回调函数
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if(htim->Instance == TIM6) {
        // VOFA 反馈
        vofa_tx_data[0] = foc.Idq.d;
        vofa_tx_data[1] = foc.Idq.q;
        vofa_tx_data[2] = foc.speed;
        vofa_tx_data[3] = foc.mech_theta;

        vofa_cdc_send(vofa_tx_data, 4);
        // CAN 反馈
        fdcan_dev.tx_len = communicate_build_feedback(&comm, fdcan_dev.fdcan_tx_buff);
        bsp_fdcan_tx(&fdcan_dev);
    }
}

/**
 * @brief ADC 注入通道转换完成回调函数
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) {
        get_current_update();
        raw_angle = as5047p_read_angle(&as5047p); // 读取原始角度

        foc_update(&foc, raw_angle, Iu, Iv, Iw); // 更新 FOC 数据

        if (foc_enabled) {
            switch (foc_mode) {
                case FOC_MODE_CURRENT:
                    foc_setcurrent(&foc, Id, Iq);
                    break;
                case FOC_MODE_SPEED:
                    foc_setspeed(&foc, radpersec, 0.0f);
                    break;
                case FOC_MODE_POSITION:
                    foc_setposition(&foc, pos, 0.0f);
                    break;
                default:
                    break;
            }
        }
    }
}
