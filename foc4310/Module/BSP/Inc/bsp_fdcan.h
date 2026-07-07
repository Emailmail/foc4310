/**
 * @file bsp_fdcan.h
 * @brief FDCAN 驱动层 — 单总线多设备抽象
 * @note  一个 FDCAN 外设上可挂载多个设备 (靠 rx_id 区分)
 *        框架参考 bsp_uart
 */

#ifndef _BSP_FDCAN_H
#define _BSP_FDCAN_H
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"
#include <stdint.h>

#define DEVICE_FDCAN_CNT 8     // 单条 FDCAN 总线最大设备数

typedef void (*fdcan_rx_callback)(void *device_instance, uint8_t *data, uint8_t len);

typedef struct {
    FDCAN_HandleTypeDef *hfdcan;     // HAL FDCAN 句柄

    // 接收
    uint32_t rx_id;                  // 接收 ID
    void *device_instance;           // 模块实例指针 (回调时传回)
    fdcan_rx_callback device_callback;

    // 发送
    uint32_t tx_id;                  // 发送 ID
    uint8_t  tx_len;                 // 发送数据长度 (0~8)
    uint8_t  fdcan_tx_buff[8];       // 发送缓冲区
} bsp_fdcan_t;

void bsp_fdcan_init(bsp_fdcan_t *fdcan);
void bsp_fdcan_filter_init(bsp_fdcan_t *fdcan);
void bsp_fdcan_register(bsp_fdcan_t *fdcan, uint32_t rx_id,
                        void *device_instance, fdcan_rx_callback callback);
void bsp_fdcan_tx(bsp_fdcan_t *fdcan);

#ifdef __cplusplus
}
#endif
#endif
