#include "vofa.h"

static const uint8_t vofa_frame_head[4] = {0x00, 0x00, 0x80, 0x7F}; // float NaN 帧头

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
}

/**
 * @brief 发送 float 数组到 VOFA+ (JustFloat 模式)
 */
void vofa_send(vofa_t *vofa, float *data, uint16_t len) {
    bsp_uart_tx(vofa->uart, (uint8_t *)vofa_frame_head, 4, BSP_UART_TX_BLOCK);
    bsp_uart_tx(vofa->uart, (uint8_t *)data, len * 4, BSP_UART_TX_BLOCK);
}
