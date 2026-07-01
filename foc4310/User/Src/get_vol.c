#include "get_vol.h"
#include "adc.h"
#include "bsp_pwm.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_adc_ex.h"
#include "stm32g4xx_hal_tim.h"
#include "tim.h"
#include <stdint.h>
// ---------------- 获取 vbus ----------------
#define VREFINT_CAL       (*((uint16_t*)0x1FFF75AA)) // VREFINT 校准值 (3.0V)

// 对 vbus 做线性校准
#define VBUS_CALIB_K  0.9727626459           // 校准系数
#define VBUS_CALIB_B  0.9046692607           // 校准偏置

// vbus 采集相关变量
uint16_t vbus_dma_buffer[2] = {0}; // DMA 接收缓冲区 [0]=NTC, [1]=Vbus
float vbus_vol = 0.0f; // Vbus 电压
float ntc_vol = 0.0f;  // NTC 电压
float vdda = 3.3f;     // 实测 VDDA

// vbus 采集函数
void get_vbus_start_dma(void) {
    HAL_Delay(300); // 等待 ADC 稳定

    // 读 ADC1 Rank1 VREFINT 获取真实 VDDA
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
        vdda = 3.0f * VREFINT_CAL / HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    // 读取 vbus
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)vbus_dma_buffer, 2);

    // 首次读取
    HAL_Delay(200); // 等待 DMA 稳定
    ntc_vol  = (vbus_dma_buffer[0] / 4095.0f) * vdda;
    vbus_vol = (vbus_dma_buffer[1] / 4095.0f) * vdda * 19.0f * VBUS_CALIB_K + VBUS_CALIB_B;
}

// vbus 更新函数 (在定时器中断回调中调用)
void update_vbus(void) {
    ntc_vol  = (vbus_dma_buffer[0] / 4095.0f) * vdda;
    vbus_vol = (vbus_dma_buffer[1] / 4095.0f) * vdda * 19.0f * VBUS_CALIB_K + VBUS_CALIB_B;   // 分压比 1:19
}

// ---------------- 获取 Iu & Iv & Iw ----------------
#define Rsense 0.005f   // 采样电阻值 (mΩ)
#define Gina240a2 50    // INA240A2 的电压增益值(V/V)

// 变量
float Iu = 0.0f; // Iu 电流
float Iv = 0.0f; // Iv 电流
float Iw = 0.0f; // Iw 电流
float Iu_offset = 0.0f; // Iu 偏置
float Iw_offset = 0.0f; // Iw 偏置

// 校准电流
void get_current_offset(void) {
    uint32_t adc_u_sum = 0;
    uint32_t adc_w_sum = 0;
    const uint16_t sample_count = 100;
    for (uint16_t i = 0; i < sample_count; i++) {
        HAL_ADCEx_InjectedStart(&hadc1);
        HAL_ADCEx_InjectedPollForConversion(&hadc1, 10);

        adc_u_sum += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        adc_w_sum += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);

        HAL_Delay(1);
    }
    float adc_u_offset = adc_u_sum / (float)sample_count;
    float adc_w_offset = adc_w_sum / (float)sample_count;
    float vu_offset = (adc_u_offset / 4095.0f) * vdda;
    float vw_offset = (adc_w_offset / 4095.0f) * vdda;
    Iu_offset = (vu_offset - vdda / 2.0f) / (Gina240a2 * Rsense);
    Iw_offset = (vw_offset - vdda / 2.0f) / (Gina240a2 * Rsense);
    HAL_ADCEx_InjectedStop(&hadc1);
}

// 读取电流初始化
void get_current_start(void) {
    HAL_ADCEx_InjectedStart_IT(&hadc1); // 启动 ADC 注入通道转换并开启中断
    __HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_JEOC); // 改用 JEOS：两路全完成才进 ISR
    __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOS);
}

void get_current_update(void) {
    // 直接读 JDR 寄存器，与 Nebula 一致，避免 HAL_ADCEx_InjectedGetValue 的时序抖动
    uint16_t adc_u_raw = hadc1.Instance->JDR1;
    uint16_t adc_w_raw = hadc1.Instance->JDR2;

    // 转换为电压
    float vu = (adc_u_raw / 4095.0f) * vdda;
    float vw = (adc_w_raw / 4095.0f) * vdda;

    // 转换为电流 (考虑 INA240A2 的增益和采样电阻)
    Iu = (vu - vdda / 2.0f) / (Gina240a2 * Rsense) - Iu_offset; // Iu = (Vu - Vref) / (G * Rsense)
    Iw = (vw - vdda / 2.0f) / (Gina240a2 * Rsense) - Iw_offset; // Iw = (Vw - Vref) / (G * Rsense)
    Iv = -Iu - Iw; // Iu + Iv + Iw = 0
}

