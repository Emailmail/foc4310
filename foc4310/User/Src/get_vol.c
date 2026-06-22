#include "get_vol.h"
#include "adc.h"
#include "stm32g4xx_hal.h"

// ---------------- vbus ----------------
#define VREFINT_CAL       (*((uint16_t*)0x1FFF75AA)) // VREFINT 校准值 (3.0V)

// 对 vbus 做线性校准
#define VBUS_CALIB_K  0.9727626459           // 校准系数
#define VBUS_CALIB_B  0.9046692607           // 校准偏置

uint16_t vbus_dma_buffer[2] = {0}; // DMA 接收缓冲区 [0]=NTC, [1]=Vbus
float vbus_vol = 0.0f; // Vbus 电压
float ntc_vol = 0.0f;  // NTC 电压
float vdda = 3.3f;     // 实测 VDDA

void get_vbus_start_dma(void) {
    HAL_Delay(300); // 等待 ADC 稳定

    // 读 ADC1 Rank3 VREFINT 获取真实 VDDA
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10); // rank1
    HAL_ADC_PollForConversion(&hadc1, 10); // rank2
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

void update_vbus(void) {
    ntc_vol  = (vbus_dma_buffer[0] / 4095.0f) * vdda;
    vbus_vol = (vbus_dma_buffer[1] / 4095.0f) * vdda * 19.0f * VBUS_CALIB_K + VBUS_CALIB_B;   // 分压比 1:19
}
