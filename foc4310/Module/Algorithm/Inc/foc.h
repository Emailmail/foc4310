#ifndef _FOC_H
#define _FOC_H
#include "bsp_pwm.h"
#include <stdint.h>

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

// foc pid 结构体
typedef struct {
    float Kp;
    float Ki;

    float err;
    float inte;

    float pout;
    float iout;
    float out;

    float inte_lim; // 积分限幅
    float out_lim; // 输出限幅
} foc_pid_t;

// foc 实例
typedef struct {
    // 三条 pwm 通道 (SPWM / SVPWM)
    bsp_pwm_t *pwmu;
    bsp_pwm_t *pwmv;
    bsp_pwm_t *pwmw;

    float vdc; // 直流母线电压
    uint32_t arr; // 定时器 ARR 寄存器值
    uint8_t pole_pairs; // 电机极对数

    // 三种坐标系下的电压
    uvw_t Uuvw;
    ab_t Uab;
    dq_t Udq;

    // 电流
    uvw_t Iuvw;
    ab_t Iab;
    dq_t Idq;
    float lowpass_alpha_u;  // 电流低通滤波系数
    float lowpass_alpha_v;
    float lowpass_alpha_w;
    float lowpass_alpha_d;
    float lowpass_alpha_q;

    float elec_theta; // 电角度
    float mech_theta; // 机械角度
    float elec_theta_offset; // 电角度零点偏移

    foc_pid_t id_pid;    // d 轴电流 PI
    foc_pid_t iq_pid;    // q 轴电流 PI
    foc_pid_t speed_pid; // 速度环 PI
    float speed;          // 当前机械角速度 [rad/s]
    uint32_t cnt;         // 速度环降采样计数器

    // 扭矩前馈补偿参数
    float static_compensation; // 静摩擦力补偿值
    float static_threshold; // 静摩擦力补偿阈值
    float fric_slope; // 摩擦力斜率
    float dynamic_compensation; // 动态摩擦力补偿值
    float torque_feedforward; // 扭矩前馈补偿值
} foc_t;

void foc_register(foc_t *foc, bsp_pwm_t *pwmu, bsp_pwm_t *pwmv, bsp_pwm_t *pwmw, float vdc, uint8_t pole_pairs, float elec_theta_offset);
void foc_init(foc_t *foc);
void foc_openloop_virtual(foc_t *foc, float Ud, float Uq, float elec_theta);
void foc_update(foc_t *foc, float mech_theta, float Iu, float Iv, float Iw, float speed);
void foc_openloop(foc_t *foc, float Ud, float Uq);
void foc_openloop_svpwm(foc_t *foc, float Ud, float Uq);
void foc_setpid_param(foc_t *foc, float id_kp, float id_ki, float iq_kp, float iq_ki);
void foc_setpid_outlimit(foc_t *foc, float id_out_lim, float iq_out_lim);
void foc_setpid_intelimit(foc_t *foc, float id_inte_lim, float iq_inte_lim);
void foc_setpid_speed(foc_t *foc, float kp, float ki, float out_lim, float inte_lim);
void foc_settorquefeedforward(foc_t *foc, float static_compensation, float static_threshold, float fric_slope, float dynamic_compensation);
void foc_setcurrent(foc_t *foc, float Id_ref, float Iq_ref);
void foc_setspeed(foc_t *foc, float speed_ref, float Id_ref);
#endif
