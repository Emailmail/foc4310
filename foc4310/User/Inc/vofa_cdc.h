#ifndef VOFA_CDC_H
#define VOFA_CDC_H
#include <stdint.h>

#define VOFA_CDC_RX_NUM 16

void vofa_cdc_send(float *data, uint16_t len);
void vofa_cdc_receive(uint8_t *buf, uint16_t len);
void vofa_cdc_rx_bind(uint8_t id, float *var);

#endif
