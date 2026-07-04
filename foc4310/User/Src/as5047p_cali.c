#include "as5047p_cali.h"
#include "arm_math.h"

uint32_t before_calicount = 10000;

uint32_t cali_count = 10000;
float cali_sum = 0.0f;
float cali_result = 0.0f;

void cali_update(void) {
    if(before_calicount > 0) {
        before_calicount--;
    } else {
        if(cali_count > 0) {
            cali_sum += mech_angle * 14.0f;
            cali_count--;
        } else {
            cali_result = cali_sum / 10000.0f;
        }
    }
}

// ---------------- 速度计算 ----------------
#define SPEED_REFRESH_CNT 10      // 降采样比: 每 10 个 FOC 周期算一次速度
#define SPEED_BUF_SIZE    20      // 滑动平均窗口大小
#define SPEED_DT          0.0005f  // 10 * 50us = 0.5ms

/**
 * @brief 获取电机机械角速度 (降采样 Δθ/Δt + 滑动平均滤波)
 * @note  结果写入全局变量 speed [rad/s]
 * @note  在 ADC JEOS ISR 中调用 (20kHz), 内部降采样到 2kHz 计算,
 *         1 LSB → 0.77 rad/s, 20 点滑动平均, 群延迟约 4.75ms
 */
void as5047p_getspeed(void) {
    static float last_angle = 0.0f;
    static float speed_buf[SPEED_BUF_SIZE] = {0};
    static float speed_sum = 0.0f;
    static uint8_t buf_idx = 0;
    static uint8_t buf_fill = 0;     // 已填充的样本数
    static uint8_t refresh_cnt = 0;

    refresh_cnt++;
    if (refresh_cnt >= SPEED_REFRESH_CNT) {
        refresh_cnt = 0;

        // 角度差分 + 翻转处理
        float delta = mech_angle - last_angle;
        if (delta > PI)        delta -= 2.0f * PI;
        else if (delta < -PI)  delta += 2.0f * PI;
        last_angle = mech_angle;

        // 转为 rad/s: speed = delta / (5 * 50us)
        float raw = delta / SPEED_DT;

        // 10 点滑动平均
        speed_sum -= speed_buf[buf_idx];
        speed_buf[buf_idx] = raw;
        speed_sum += raw;
        buf_idx = (buf_idx + 1) % SPEED_BUF_SIZE;

        // 缓冲区未满时用实际样本数做平均
        if (buf_fill < SPEED_BUF_SIZE) buf_fill++;
        speed = speed_sum / (float)buf_fill;
    }
}