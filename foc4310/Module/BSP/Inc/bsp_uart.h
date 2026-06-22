#ifndef _BSP_UART_H
#define _BSP_UART_H
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_uart.h"
#include "stm32g4xx_hal_uart_ex.h"
#include <stdint.h>

#define DEVICE_UART_CNT 5     // 最大串口数量
#define UART_RXBUFF_LIMIT 32 // 串口接收缓冲区大小

typedef void (*uart_device_callback)(void *device_instance, uint16_t size); // 模块回调函数,用于解析协议

typedef enum {
    BSP_UART_TX_BLOCK = 0,
    BSP_UART_TX_IT,
    BSP_UART_TX_DMA,
} bsp_uart_tx_mode_t;

typedef  struct {
    UART_HandleTypeDef *huart;
    uint8_t rxbuff[UART_RXBUFF_LIMIT];

    void *device_instance; // 模块实例指针,用于回调函数解析协议时使用
    uart_device_callback device_callback;
} bsp_uart_t;

void bsp_uart_register(bsp_uart_t *uart, UART_HandleTypeDef *huart, void *device_instance, uart_device_callback device_callback);
void bsp_uart_rx_start(bsp_uart_t *uart);
uint8_t bsp_uart_isidle(bsp_uart_t *uart);
void bsp_uart_tx(bsp_uart_t *uart, uint8_t *data, uint16_t size, bsp_uart_tx_mode_t mode);

#ifdef __cplusplus
}
#endif
#endif
