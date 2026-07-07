#include "communicate.h"
#include <stddef.h>

#define CAN_I_MAX     10.0f
#define CAN_SPEED_MAX 1000.0f
#define CAN_ANGLE_MAX 6.283185307f

void communicate_init(communicate_t *comm,
                      uint8_t *foc_enabled, foc_mode_t *foc_mode,
                      float *rx_iq_ref, float *rx_speed_ref, float *rx_pos_ref,
                      float *tx_iq, float *tx_speed, float *tx_angle) {
    comm->foc_enabled  = foc_enabled;
    comm->foc_mode     = foc_mode;
    comm->rx_iq_ref    = rx_iq_ref;
    comm->rx_speed_ref = rx_speed_ref;
    comm->rx_pos_ref   = rx_pos_ref;
    comm->tx_iq        = tx_iq;
    comm->tx_speed     = tx_speed;
    comm->tx_angle     = tx_angle;
}

void communicate_rx_callback(void *instance, uint8_t *data, uint8_t len) {
    (void)len;
    communicate_t *comm = (communicate_t *)instance;

    uint8_t cmd = data[0];
    int16_t val = (int16_t)(data[1] | ((uint16_t)data[2] << 8));

    switch (cmd) {
        case 0x00: break;

        case 0x01: *comm->foc_enabled = 1; break;

        case 0x02: *comm->foc_enabled = 0; break;

        case 0x03:
            *comm->foc_mode  = FOC_MODE_CURRENT;
            *comm->rx_iq_ref = (float)val / INT16_MAX * CAN_I_MAX;
            break;

        case 0x04:
            *comm->foc_mode     = FOC_MODE_SPEED;
            *comm->rx_speed_ref = (float)val / INT16_MAX * CAN_SPEED_MAX
                                  * 0.104719755f;
            break;

        case 0x05:
            *comm->foc_mode   = FOC_MODE_POSITION;
            *comm->rx_pos_ref = (float)((uint16_t)val) / UINT16_MAX * CAN_ANGLE_MAX;
            break;

        default: break;
    }
}

uint8_t communicate_build_feedback(communicate_t *comm, uint8_t *buf) {
    buf[0] = *comm->foc_enabled ? 0x01 : 0x00;
    buf[1] = 0x00;

    int16_t iq = (int16_t)(*comm->tx_iq / CAN_I_MAX * INT16_MAX);
    buf[2] = (uint8_t)(iq & 0xFF);
    buf[3] = (uint8_t)((iq >> 8) & 0xFF);

    float speed_rpm = *comm->tx_speed * 9.549296586f;
    int16_t speed = (int16_t)(speed_rpm / CAN_SPEED_MAX * INT16_MAX);
    buf[4] = (uint8_t)(speed & 0xFF);
    buf[5] = (uint8_t)((speed >> 8) & 0xFF);

    uint16_t angle = (uint16_t)(*comm->tx_angle / CAN_ANGLE_MAX * UINT16_MAX);
    buf[6] = (uint8_t)(angle & 0xFF);
    buf[7] = (uint8_t)((angle >> 8) & 0xFF);

    return 8;
}
