#include "vofa.h"
#include "bsp_uart.h"
#include <string.h>

static const uint8_t vofa_frame_head[4] = {0x00, 0x00, 0x80, 0x7F}; // float NaN 帧头
#define VOFA_TX_MAX_FLOATS 16
static uint8_t vofa_tx_buf[4 + VOFA_TX_MAX_FLOATS * 4]; // 帧头 + 数据拼接缓冲

/**
 * @brief VOFA 接收回调 — 由 bsp_uart 在收到数据后调用
 */
static void vofa_rx_callback(void *instance, uint16_t size) {
    vofa_t *vofa = (vofa_t *)instance;
    uint8_t *buf = vofa->uart->rxbuff;

    // 帧: | 0x0A | id | data[4] | 0x0B | = 7 bytes
    if (size != 7 || buf[0] != 0x0A || buf[6] != 0x0B)
        return;

    uint8_t id = buf[1];
    if (id >= VOFA_RX_NUM)
        return;

    for (int i = 0; i < 4; i++)
        ((uint8_t *)&vofa->rx_var[id])[i] = buf[i + 2];
}

/**
 * @brief 注册 VOFA
 */
void vofa_register(vofa_t *vofa, bsp_uart_t *uart) {
    vofa->uart = uart;
    uart->device_instance = vofa;
    uart->device_callback = vofa_rx_callback;
    bsp_uart_rx_start(uart); // 启动 DMA 接收
}

/**
 * @brief 发送 float 数组到 VOFA+ (JustFloat 模式)
 */
void vofa_send(vofa_t *vofa, float *data, uint16_t len) {
    if (len > VOFA_TX_MAX_FLOATS) return;

    uint16_t total = 4 + len * 4;
    memcpy(vofa_tx_buf, vofa_frame_head, 4);
    memcpy(vofa_tx_buf + 4, data, len * 4);
    bsp_uart_tx(vofa->uart, vofa_tx_buf, total, BSP_UART_TX_DMA);
}
