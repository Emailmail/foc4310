#ifndef VOFA_H
#define VOFA_H
#ifdef __cplusplus
extern "C" {
#endif
#include "bsp_uart.h"

#define VOFA_RX_NUM 8

typedef struct {
    bsp_uart_t *uart;
    float rx_var[VOFA_RX_NUM];
} vofa_t;

void vofa_register(vofa_t *vofa, bsp_uart_t *uart);
void vofa_send(vofa_t *vofa, float *data, uint16_t len);
#ifdef __cplusplus
}
#endif
#endif
