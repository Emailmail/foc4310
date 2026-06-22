#include "bsp_uart.h"

/* uart 数组, 所有注册了的 uart 实例都会保存在这里 */
static uint8_t idx = 0;
static bsp_uart_t *uart_instances[DEVICE_UART_CNT] = {NULL};

/**
 * @brief 注册 uart 设备实例
 */
void bsp_uart_register(bsp_uart_t *uart, UART_HandleTypeDef *huart, void *device_instance, uart_device_callback device_callback) {
    if (idx >= DEVICE_UART_CNT) {
        return;
    }
    
    uart->huart = huart;
    uart->device_instance = device_instance;
    uart->device_callback = device_callback;

    uart_instances[idx++] = uart;
}

/**
 * @brief 启动 uart 接收
 */
void bsp_uart_rx_start(bsp_uart_t *uart) {
    HAL_UARTEx_ReceiveToIdle_DMA(uart->huart, uart->rxbuff, UART_RXBUFF_LIMIT);
    __HAL_DMA_DISABLE_IT(uart->huart->hdmarx, DMA_IT_HT);   // 关闭 DMA 半传输中断(开启 DMA 接收中断会默认开启), 确保只在传输完成时触发回调
}

/** 
 * @brief uart 查询是否空闲 (可防止短时间内重复调用发送导致丢包)
 */
uint8_t bsp_uart_isidle(bsp_uart_t *uart) {
    return (uart->huart->gState == HAL_UART_STATE_READY);
}

/**
 * @brief uart 发送
 */
void bsp_uart_tx(bsp_uart_t *uart, uint8_t *data, uint16_t size, bsp_uart_tx_mode_t mode) {
    switch (mode) {
        case BSP_UART_TX_BLOCK:
            HAL_UART_Transmit(uart->huart, data, size, HAL_MAX_DELAY);
            break;
        case BSP_UART_TX_IT:
            HAL_UART_Transmit_IT(uart->huart, data, size);
            break;
        case BSP_UART_TX_DMA:
            HAL_UART_Transmit_DMA(uart->huart, data, size);
            break;
    }
}

/** 
 * @brief uart 接收完成回调函数, 此处统一管理
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    for (uint8_t i = 0; i < idx; ++i) {
        if (huart == uart_instances[i]->huart) {
            if (uart_instances[i]->device_callback != NULL) {  // 如果定义了模块回调函数, 就调用
                uart_instances[i]->device_callback(uart_instances[i]->device_instance, Size);
            }

            bsp_uart_rx_start(uart_instances[i]);   // 重启 DMA 接收

            return;
        }
    }
}

/** 
 * @brief uart 错误回调函数, 此处统一管理
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < idx; ++i) {
        if (huart == uart_instances[i]->huart) {
            bsp_uart_rx_start(uart_instances[i]);
            return;
        }
    }
}
