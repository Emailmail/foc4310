#include "vofa_cdc.h"
#include "usbd_cdc_if.h"
#include <string.h>

#define VOFA_CDC_MAX_FLOATS 16
static const uint8_t vofa_frame_head[4] = {0x00, 0x00, 0x80, 0x7F};
static uint8_t vofa_cdc_tx_buf[4 + VOFA_CDC_MAX_FLOATS * 4];

static float *vofa_cdc_rx_map[VOFA_CDC_RX_NUM];

void vofa_cdc_send(float *data, uint16_t len) {
    if (len > VOFA_CDC_MAX_FLOATS) return;
    uint16_t total = 4 + len * 4;
    memcpy(vofa_cdc_tx_buf, vofa_frame_head, 4);
    memcpy(vofa_cdc_tx_buf + 4, data, len * 4);
    CDC_Transmit_FS(vofa_cdc_tx_buf, total);
}

void vofa_cdc_rx_bind(uint8_t id, float *var) {
    if (id < VOFA_CDC_RX_NUM)
        vofa_cdc_rx_map[id] = var;
}

void vofa_cdc_receive(uint8_t *buf, uint16_t len) {
    // 帧: | 0x0A | id | data[4] | 0x0B | = 7 bytes
    if (len != 7 || buf[0] != 0x0A || buf[6] != 0x0B)
        return;

    uint8_t id = buf[1];
    float value;
    memcpy(&value, &buf[2], 4);

    if (id < VOFA_CDC_RX_NUM && vofa_cdc_rx_map[id] != NULL)
        *vofa_cdc_rx_map[id] = value;
}
