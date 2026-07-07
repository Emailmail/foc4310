#include "lpf_2nd.h"
#include <math.h>

/**
 * @brief 初始化二阶 IIR 低通滤波器 (双线性变换)
 * @param Ts      采样周期 [s]
 * @param Fc      截止频率 [Hz]
 * @param damping 阻尼系数 (0.707 = 巴特沃斯, 无过冲)
 */
void lpf_2nd_init(lpf_2nd_t *f, float Ts, float Fc, float damping) {
    float Wc = 2.0f / Ts * tanf(3.14159265f * Fc * Ts);
    float Ts2 = Ts * Ts;
    float b0 = Wc * Wc * Ts2;
    float b1 = 2.0f * b0;
    float b2 = b0;
    float a0_inv = 1.0f / (4.0f + 4.0f * damping * Wc * Ts + b0);

    f->b0 = b0 * a0_inv;
    f->b1 = b1 * a0_inv;
    f->b2 = b2 * a0_inv;
    f->a1 = (8.0f - 2.0f * b0) * a0_inv;                 // 已取反，update 中直接 +
    f->a2 = (4.0f * damping * Wc * Ts - 4.0f - b0) * a0_inv;
    f->x1 = 0.0f; f->x2 = 0.0f;
    f->y1 = 0.0f; f->y2 = 0.0f;
}

/**
 * @brief 二阶 IIR 低通滤波器更新
 * @param x 当前输入
 * @return  滤波后输出
 */
float lpf_2nd_update(lpf_2nd_t *f, float x) {
    float y = f->b0 * x + f->b1 * f->x1 + f->b2 * f->x2
                         + f->a1 * f->y1 + f->a2 * f->y2;
    f->x2 = f->x1; f->x1 = x;
    f->y2 = f->y1; f->y1 = y;
    return y;
}

/**
 * @brief 复位滤波器状态到指定值 (消除启动瞬态)
 * @param value 初始值
 */
void lpf_2nd_reset(lpf_2nd_t *f, float value) {
    f->x1 = f->x2 = value;
    f->y1 = f->y2 = value;
}
