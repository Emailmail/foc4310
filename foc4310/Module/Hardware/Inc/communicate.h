#ifndef _COMMUNICATE_H
#define _COMMUNICATE_H
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// FOC 控制模式 (与 CAN 协议指令类型对齐)
typedef enum {
    FOC_MODE_DISABLE  = 0x00,
    FOC_MODE_CURRENT  = 0x03,
    FOC_MODE_SPEED    = 0x04,
    FOC_MODE_POSITION = 0x05,
} foc_mode_t;

// 通信实例 (集成所有数据指针)
typedef struct {
    // RX: CAN → 控制目标
    uint8_t    *foc_enabled;
    foc_mode_t *foc_mode;
    float      *rx_iq_ref;      // 电流控制目标 (A)
    float      *rx_speed_ref;   // 速度控制目标 (rad/s)
    float      *rx_pos_ref;     // 位置控制目标 (rad)

    // TX: 反馈 → CAN
    float *tx_iq;               // q轴电流 (A)
    float *tx_speed;            // 速度 (rad/s)
    float *tx_angle;            // 角度 (rad)
} communicate_t;

// 初始化通信实例
void communicate_init(communicate_t *comm,
                      uint8_t *foc_enabled, foc_mode_t *foc_mode,
                      float *rx_iq_ref, float *rx_speed_ref, float *rx_pos_ref,
                      float *tx_iq, float *tx_speed, float *tx_angle);

// CAN 接收回调 (符合 fdcan_rx_callback 签名)
void communicate_rx_callback(void *instance, uint8_t *data, uint8_t len);

// 打包 8-byte 反馈帧, 返回 8
uint8_t communicate_build_feedback(communicate_t *comm, uint8_t *buf);

#ifdef __cplusplus
}
#endif
#endif
