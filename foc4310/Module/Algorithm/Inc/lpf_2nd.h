#ifndef _LPF_2ND_H
#define _LPF_2ND_H

// 二阶 IIR 低通滤波器 (双线性变换)
// 参考 QD4310 LowPassFilter_2_Order
typedef struct {
    float b0, b1, b2;   // 前馈系数
    float a1, a2;       // 反馈系数 (已取反，update 中直接加)
    float x1, x2;       // x[n-1], x[n-2]
    float y1, y2;       // y[n-1], y[n-2]
} lpf_2nd_t;

void lpf_2nd_init(lpf_2nd_t *f, float Ts, float Fc, float damping);
float lpf_2nd_update(lpf_2nd_t *f, float x);
void lpf_2nd_reset(lpf_2nd_t *f, float value);

#endif
