#ifndef _FOC_H
#define _FOC_H
#include "bsp_pwm.h"
#include "lpf_2nd.h"
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

    float vdc;         // 直流母线电压
    uint32_t arr;      // 定时器 ARR 寄存器值
    uint8_t pole_pairs;// 电机极对数
    float dt;          // FOC 采样周期 [s]

    // 三种坐标系下的电压
    uvw_t Uuvw;
    ab_t Uab;
    dq_t Udq;

    // 电流
    uvw_t Iuvw;
    ab_t Iab;
    dq_t Idq;

    lpf_2nd_t speed_lpf; // 速度二阶滤波器
    lpf_2nd_t id_lpf;   // d 轴电流二阶滤波器
    lpf_2nd_t iq_lpf;   // q 轴电流二阶滤波器

    float elec_theta; // 电角度
    float mech_theta; // 机械角度
    float elec_theta_offset; // 电角度零点偏移

    foc_pid_t id_pid;    // d 轴电流 PI
    foc_pid_t iq_pid;    // q 轴电流 PI
    foc_pid_t speed_pid; // 速度环 PI
    foc_pid_t pos_pid;   // 位置环 PI

    float speed;         // 当前机械角速度 [rad/s]
    float last_angle;    // 上一次角度 (速度差分)
    uint32_t cnt;        // 速度/位置环降采样计数器

} foc_t;

void foc_register(foc_t *foc, bsp_pwm_t *pwmu, bsp_pwm_t *pwmv, bsp_pwm_t *pwmw, float vdc, uint8_t pole_pairs, float elec_theta_offset, float dt);
void foc_lpf_init(foc_t *foc, float current_fc, float speed_fc);
void foc_init(foc_t *foc);
void foc_openloop_virtual(foc_t *foc, float Ud, float Uq, float elec_theta);
void foc_update(foc_t *foc, float raw_angle, float Iu, float Iv, float Iw);
void foc_openloop(foc_t *foc, float Ud, float Uq);
void foc_openloop_svpwm(foc_t *foc, float Ud, float Uq);
void foc_setpid_currentparam(foc_t *foc, float id_kp, float id_ki, float iq_kp, float iq_ki);
void foc_setpid_currentoutlimit(foc_t *foc, float id_out_lim, float iq_out_lim);
void foc_setpid_currentintelimit(foc_t *foc, float id_inte_lim, float iq_inte_lim);
void foc_setpid_speed(foc_t *foc, float kp, float ki, float out_lim, float inte_lim);
void foc_setpid_position(foc_t *foc, float kp, float ki, float out_lim, float inte_lim);
void foc_setcurrent(foc_t *foc, float Id_ref, float Iq_ref);
void foc_setspeed(foc_t *foc, float speed_ref, float Id_ref);
void foc_setposition(foc_t *foc, float pos_ref, float Id_ref);
#endif
