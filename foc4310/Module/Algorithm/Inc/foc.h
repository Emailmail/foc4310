#ifndef _FOC_H
#define _FOC_H
#include "bsp_pwm.h"
#include <stdint.h>

#define USE_SVPWM (1)

#define _2PI 6.283185307179586f         // 2 * π

// foc 坐标系
typedef struct {
    float u;
    float v;
    float w;
} uvw_t;
typedef struct {
    float a;
    float b;
} ab_t;
typedef struct {
    float q;
    float d;
} dq_t;

// foc 实例
typedef struct {
    // 三条 pwm 通道 (SPWM / SVPWM)
    bsp_pwm_t *pwmu;
    bsp_pwm_t *pwmv;
    bsp_pwm_t *pwmw;

    float vdc; // 直流母线电压
    uint32_t arr; // 定时器 ARR 寄存器值

    // 坐标系
    uvw_t Uuvw;
    ab_t Uab;
    dq_t Udq;
    float elec_theta; // 电角度
    float mech_theta; // 机械角度
    
} foc_t;

void foc_register(foc_t *foc, bsp_pwm_t *pwmu, bsp_pwm_t *pwmv, bsp_pwm_t *pwmw, float vdc);
void foc_init(foc_t *foc);
void foc_openloop_virtual(foc_t *foc, float Ud, float Uq, float elec_theta);

#endif
