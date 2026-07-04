#include "foc.h"
#include "arm_math.h"
#include "bsp_pwm.h"

#define SQRT3 1.732050807568877f      // √3
#define SQRT3_DIV2 0.866025403784438f // √3 / 2

// ---------------- FOC 数学运算 ----------------
/**
 * @brief park 逆变换
 */
static void inpark(dq_t *dq, ab_t *ab, float theta) {
  float cos_theta = arm_cos_f32(theta);
  float sin_theta = arm_sin_f32(theta);

  ab->a =
      dq->d * cos_theta - dq->q * sin_theta; // α = d  *cosθ - q * sinθ
  ab->b =
      dq->d * sin_theta + dq->q * cos_theta; // β = d  *sinθ + q * cosθ
}

/**
 * @brief clarke 逆变换
 */
static void inclarke(ab_t *ab, uvw_t *uvw) {
  uvw->u = ab->a; // u = α;
  uvw->v = -0.5f * ab->a +
           SQRT3_DIV2 * ab->b; // v = (-α + √3 * β) / 2
  uvw->w = -0.5f * ab->a -
           SQRT3_DIV2 * ab->b; // w = (-α - √3 * β) / 2
}

/**
 * @brief park 正变换
 */
static void park(ab_t *ab, dq_t *dq, float theta) {
  float cos_theta = arm_cos_f32(theta);
  float sin_theta = arm_sin_f32(theta);

  dq->d =
      ab->a * cos_theta + ab->b * sin_theta; // d = α * cosθ + β * sinθ
  dq->q =
      -ab->a * sin_theta + ab->b * cos_theta; // q = -α * sinθ + β * cosθ
}

/**
 * @brief clarke 正变换
 */
static void clarke(uvw_t *uvw, ab_t *ab) {
  ab->a = uvw->u; // α = u
  ab->b = (uvw->u + 2 * uvw->v) / SQRT3;  // β = (u + 2 * v) / √3
}

/**
 * @brief FOC PID 控制
 */
static float foc_pid_update(foc_pid_t *pid, float ref, float fb) {
  // 计算误差
  float err = ref - fb;

  // 计算参数
  pid->err = err;
  pid->inte += err;
  pid->inte = (pid->inte > pid->inte_lim) ? pid->inte_lim : (pid->inte < -pid->inte_lim) ? -pid->inte_lim : pid->inte;  // 积分限幅

  // 计算输出
  pid->pout = pid->Kp * err;
  pid->iout = pid->Ki * pid->inte;
  pid->out = pid->pout + pid->iout;
  pid->out = (pid->out > pid->out_lim) ? pid->out_lim : (pid->out < -pid->out_lim) ? -pid->out_lim : pid->out;  // 输出限幅

  return pid->out;
}
// ---------------- FOC 硬件操作 ----------------
static void foc_setspwm(bsp_pwm_t *spwmu, bsp_pwm_t *spwmv, bsp_pwm_t *spwmw,
                        uvw_t *Uuvw, float vdc, uint32_t arr) {
  // 计算 CCR
  uint32_t uCCR = (uint32_t)((Uuvw->u + vdc / 2.0f) / vdc * arr);
  uint32_t vCCR = (uint32_t)((Uuvw->v + vdc / 2.0f) / vdc * arr);
  uint32_t wCCR = (uint32_t)((Uuvw->w + vdc / 2.0f) / vdc * arr);

  // 设置 CCR
  bsp_pwm_setccr(spwmu, uCCR);
  bsp_pwm_setccr(spwmv, vCCR);
  bsp_pwm_setccr(spwmw, wCCR);
}

/**
 * @brief SVPWM 扇区判断 (αβ → 1~6)
 */
static uint8_t svpwm_sector(ab_t *Uab) {
  float A = Uab->b;
  float B = SQRT3 * Uab->a - Uab->b;
  float C = -SQRT3 * Uab->a - Uab->b;

  uint8_t sum = 0;
  if (A > 0) sum += 1;
  if (B > 0) sum += 2;
  if (C > 0) sum += 4;

  switch (sum) {
    case 3: return 1;
    case 1: return 2;
    case 5: return 3;
    case 4: return 4;
    case 6: return 5;
    case 2: return 6;
    default: return 1;
  }
}

/**
 * @brief SVPWM: αβ → CCR1/2/3 直接写寄存器
 */
static void foc_setsvpwm(bsp_pwm_t *spwmu, bsp_pwm_t *spwmv, bsp_pwm_t *spwmw,
                         ab_t *Uab, float vdc, uint32_t arr) {
  (void)spwmv; // V 句柄，保留接口一致性
  (void)spwmw; // W 句柄，保留接口一致性
  uint8_t sector = svpwm_sector(Uab);
  TIM_TypeDef *tim = spwmu->htim->Instance;

  float tmp = (float)arr * SQRT3 / vdc;
  float X = tmp * (Uab->b);
  float Y = tmp * (Uab->a * SQRT3_DIV2 + Uab->b * 0.5f);
  float Z = tmp * (-Uab->a * SQRT3_DIV2 + Uab->b * 0.5f);

  float T1, T2;
  switch (sector) {
    case 1: T1 = -Z; T2 =  X; break;
    case 2: T1 =  Z; T2 =  Y; break;
    case 3: T1 =  X; T2 = -Y; break;
    case 4: T1 = -X; T2 =  Z; break;
    case 5: T1 = -Y; T2 = -Z; break;
    case 6: T1 =  Y; T2 = -X; break;
    default: T1 = 0; T2 = 0; break;
  }

  // 过调制钳位
  if (T1 + T2 > arr) {
    float sum = T1 + T2;
    T1 = arr * T1 / sum;
    T2 = arr * T2 / sum;
  }
  float T0 = (float)arr - T1 - T2;

  float Ta = T0 / 2.0f;
  float Tb = Ta + T1;
  float Tc = Tb + T2;

  uint32_t aCCR, bCCR, cCCR;
  switch (sector) {
    case 1: aCCR = Ta; bCCR = Tb; cCCR = Tc; break;
    case 2: aCCR = Tb; bCCR = Ta; cCCR = Tc; break;
    case 3: aCCR = Tc; bCCR = Ta; cCCR = Tb; break;
    case 4: aCCR = Tc; bCCR = Tb; cCCR = Ta; break;
    case 5: aCCR = Tb; bCCR = Tc; cCCR = Ta; break;
    case 6: aCCR = Ta; bCCR = Tc; cCCR = Tb; break;
    default: aCCR = 0; bCCR = 0; cCCR = 0; break;
  }

  // U→CH3, V→CH2, W→CH1 (与 PWM 注册顺序一致)
  tim->CCR3 = aCCR; // U
  tim->CCR2 = bCCR; // V
  tim->CCR1 = cCCR; // W
}
// ---------------- foc 用户函数 ----------------
/**
 * @brief 注册 FOC
 */
void foc_register(foc_t *foc, bsp_pwm_t *pwmu, bsp_pwm_t *pwmv, bsp_pwm_t *pwmw, float vdc, uint8_t pole_pairs, float elec_theta_offset) {
  // 参数传递
  foc->pwmu = pwmu;
  foc->pwmv = pwmv;
  foc->pwmw = pwmw;
  foc->vdc = vdc;
  foc->pole_pairs = pole_pairs;
  foc->elec_theta_offset = elec_theta_offset;

  // 低通滤波默认系数
  foc->lowpass_alpha_u = 0.3f;
  foc->lowpass_alpha_v = 0.3f;
  foc->lowpass_alpha_w = 0.3f;
  foc->lowpass_alpha_d = 0.02f;
  foc->lowpass_alpha_q = 0.02f;

  // 计算 ARR
  foc->arr = pwmu->arr; // 假设三路 PWM 定时器 ARR 相同

  // 速度环初始化
  foc->speed = 0.0f;
  foc->cnt = 0;
  foc->speed_pid.Kp = 0.0f;
  foc->speed_pid.Ki = 0.0f;
  foc->speed_pid.err = 0.0f;
  foc->speed_pid.inte = 0.0f;
  foc->speed_pid.out = 0.0f;
  foc->speed_pid.out_lim = 3.0f;   // Iq 参考值输出限幅 ±3A
  foc->speed_pid.inte_lim = 5.0f;  // 积分限幅 ±5A
}

/**
 * @brief FOC 初始化
 */
void foc_init(foc_t *foc) {
  bsp_pwm_setccr(foc->pwmu, 0);
  bsp_pwm_setccr(foc->pwmv, 0);
  bsp_pwm_setccr(foc->pwmw, 0);

  bsp_pwm_start(foc->pwmu);
  bsp_pwm_start(foc->pwmv);
  bsp_pwm_start(foc->pwmw);
}

/**
 * @brief FOC 开环 (电角度模拟, 调试时使用)
 */
void foc_openloop_virtual(foc_t *foc, float Ud, float Uq, float elec_theta) {
  foc->Udq.d = Ud;
  foc->Udq.q = Uq;
  foc->elec_theta = elec_theta;

  inpark(&foc->Udq, &foc->Uab, foc->elec_theta);
  inclarke(&foc->Uab, &foc->Uuvw);

  foc_setspwm(foc->pwmu, foc->pwmv, foc->pwmw, &foc->Uuvw, foc->vdc, foc->arr);
}

/**
 * @brief FOC 传感器数据更新 (用于后续做运算)
 */
void foc_update(foc_t *foc, float mech_theta, float Iu, float Iv, float Iw, float speed) {
  // 更新角度
  foc->mech_theta = mech_theta;
  foc->elec_theta = mech_theta * foc->pole_pairs - foc->elec_theta_offset;

  // 存储速度反馈
  foc->speed = speed;

  // 对 uvw 电流进行低通滤波
  foc->Iuvw.u = foc->lowpass_alpha_u * Iu + (1.0f - foc->lowpass_alpha_u) * foc->Iuvw.u;
  foc->Iuvw.v = foc->lowpass_alpha_v * Iv + (1.0f - foc->lowpass_alpha_v) * foc->Iuvw.v;
  foc->Iuvw.w = foc->lowpass_alpha_w * Iw + (1.0f - foc->lowpass_alpha_w) * foc->Iuvw.w;

  // 更新电流
  clarke(&foc->Iuvw, &foc->Iab);
  park(&foc->Iab, &foc->Idq, foc->elec_theta);

  // 对 dq 电流进行低通滤波
  foc->Idq.d = foc->lowpass_alpha_d * foc->Idq.d + (1.0f - foc->lowpass_alpha_d) * foc->Idq.d;
  foc->Idq.q = foc->lowpass_alpha_q * foc->Idq.q + (1.0f - foc->lowpass_alpha_q) * foc->Idq.q;
}

/**
 * @brief FOC 开环 (SPWM)
 */
void foc_openloop(foc_t *foc, float Ud, float Uq) {
  foc->Udq.d = Ud;
  foc->Udq.q = Uq;

  inpark(&foc->Udq, &foc->Uab, foc->elec_theta);
  inclarke(&foc->Uab, &foc->Uuvw);

  foc_setspwm(foc->pwmu, foc->pwmv, foc->pwmw, &foc->Uuvw, foc->vdc, foc->arr);
}

/**
 * @brief FOC 开环 (SVPWM)
 */
void foc_openloop_svpwm(foc_t *foc, float Ud, float Uq) {
  // SVPWM 推导时的方向与 SPWM 相反，所以这里取负号
  foc->Udq.d = - Ud;
  foc->Udq.q = - Uq;

  inpark(&foc->Udq, &foc->Uab, foc->elec_theta);
  foc_setsvpwm(foc->pwmu, foc->pwmv, foc->pwmw, &foc->Uab, foc->vdc, foc->arr);
}

/**
 * @brief FOC PID 参数设置
 */
void foc_setpid_param(foc_t *foc, float id_kp, float id_ki, float iq_kp, float iq_ki) {
  foc->id_pid.Kp = id_kp;
  foc->id_pid.Ki = id_ki;
  foc->iq_pid.Kp = iq_kp;
  foc->iq_pid.Ki = iq_ki;
}

/**
 * @brief FOC PID 输出限幅
 */
void foc_setpid_outlimit(foc_t *foc, float id_out_lim, float iq_out_lim) {
  foc->id_pid.out_lim = id_out_lim;
  foc->iq_pid.out_lim = iq_out_lim;
}

/**
 * @brief FOC PID 积分限幅
 */
void foc_setpid_intelimit(foc_t *foc, float id_inte_lim, float iq_inte_lim) {
  foc->id_pid.inte_lim = id_inte_lim;
  foc->iq_pid.inte_lim = iq_inte_lim;
}

/**
 * @brief FOC 速度环 PID 参数设置
 */
void foc_setpid_speed(foc_t *foc, float kp, float ki, float out_lim, float inte_lim) {
  foc->speed_pid.Kp = kp;
  foc->speed_pid.Ki = ki;
  foc->speed_pid.out_lim = out_lim;
  foc->speed_pid.inte_lim = inte_lim;
}

/**
 * @brief FOC 设置扭矩前馈补偿的系数，用于低速运行时克服静摩擦力
 * @brief static_compensation 静摩擦力补偿值 (静止或者微动时, 需要给电机一个最小的电流来克服静摩擦力)
 * @brief static_threshold 静摩擦力补偿阈值 (大于这个值后开始降低静摩擦力补偿值)
 * @brief fric_slope 摩擦力斜率 (当速度大于静摩擦力补偿阈值后, 补偿值随着速度升高而降低, fric_slope 为降低的斜率)
 * @brief dynamic_compensation 动态摩擦力补偿值 (电机起转到一定速度后, 可以降低静摩擦力补偿值, 这个是最小值)
 */
void foc_settorquefeedforward(foc_t *foc, float static_compensation, float static_threshold, float fric_slope, float dynamic_compensation) {
  foc->static_compensation = static_compensation;
  foc->static_threshold = static_threshold;
  foc->fric_slope = fric_slope;
  foc->dynamic_compensation = dynamic_compensation;
}

/**
 * @brief FOC 电流控制 (Id/Iq PI + SVPWM)
 */
void foc_setcurrent(foc_t *foc, float Id_ref, float Iq_ref) {
  // Id/Iq PI 控制器
  float Ud = foc_pid_update(&foc->id_pid, Id_ref, foc->Idq.d);
  float Uq = foc_pid_update(&foc->iq_pid, Iq_ref, foc->Idq.q);

  // 电压钳位
  float limit = foc->vdc * 0.577f; // Vdc / √3
  Ud = (Ud > limit) ? limit : (Ud < -limit) ? -limit : Ud;
  Uq = (Uq > limit) ? limit : (Uq < -limit) ? -limit : Uq;

  foc->Udq.d = Ud;
  foc->Udq.q = Uq;

  foc_openloop_svpwm(foc, Ud, Uq);
}

/**
 * @brief FOC 速度控制 (速度环 + 电流环级联, 速度环降频执行)
 * @param speed_ref 目标机械角速度 [rad/s]
 * @param Id_ref    d 轴电流参考值
 * @note  速度环每 20 个 FOC 周期执行一次
 */
void foc_setspeed(foc_t *foc, float speed_ref, float Id_ref) {
    foc->cnt++;

    // 速度环: 每 10 次更新一次 Iq_ref (2kHz)
    if (foc->cnt % 10 == 0) {
      foc_pid_update(&foc->speed_pid, speed_ref, foc->speed);
      foc->cnt = 0;
    }

    // 计算扭矩前馈补偿值
    if(speed_ref > foc->static_threshold / 100.0f) {
        if(foc->speed <= foc->static_threshold) {
            foc->torque_feedforward = foc->static_compensation; // 静摩擦力补偿
        } else {
            foc->torque_feedforward = foc->static_compensation - (foc->speed - foc->static_threshold) * foc->fric_slope;
            if(foc->torque_feedforward < foc->dynamic_compensation) {
                foc->torque_feedforward = foc->dynamic_compensation; // 动态摩擦力补偿
            }
        }
    } else if(speed_ref < -foc->static_threshold / 100.0f) {
        if(foc->speed >= -foc->static_threshold) {
            foc->torque_feedforward = -foc->static_compensation; // 静摩擦力补偿
        } else {
            foc->torque_feedforward = -foc->static_compensation - (foc->speed + foc->static_threshold) * foc->fric_slope;
            if(foc->torque_feedforward > -foc->dynamic_compensation) {
                foc->torque_feedforward = -foc->dynamic_compensation; // 动态摩擦力补偿
            }
        }
    }

    // 电流环: 每周期执行 (20kHz), Iq_ref 来自 speed_pid.out
    float Ud = foc_pid_update(&foc->id_pid, Id_ref, foc->Idq.d);
    float Uq = foc_pid_update(&foc->iq_pid, foc->speed_pid.out + foc->torque_feedforward, foc->Idq.q);

    foc->Udq.d = Ud;
    foc->Udq.q = Uq;
    foc_openloop_svpwm(foc, Ud, Uq);
}

/**
 * @brief FOC 位置控制 (todo)
 */

