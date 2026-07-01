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

// ---------------- SVPWM ----------------
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
static void foc_setsvpwm(foc_t *foc) {
  uint8_t sector = svpwm_sector(&foc->Uab);
  TIM_TypeDef *tim = foc->pwmu->htim->Instance;

  float tmp = (float)foc->arr * SQRT3 / foc->vdc;
  float X = tmp * (foc->Uab.b);
  float Y = tmp * (foc->Uab.a * SQRT3_DIV2 + foc->Uab.b * 0.5f);
  float Z = tmp * (-foc->Uab.a * SQRT3_DIV2 + foc->Uab.b * 0.5f);

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
  if (T1 + T2 > foc->arr) {
    float sum = T1 + T2;
    T1 = foc->arr * T1 / sum;
    T2 = foc->arr * T2 / sum;
  }
  float T0 = (float)foc->arr - T1 - T2;

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
  foc->lowpass_alpha_u = 0.2f;
  foc->lowpass_alpha_v = 0.2f;
  foc->lowpass_alpha_w = 0.2f;

  // 计算 ARR
  foc->arr = pwmu->arr; // 假设三路 PWM 定时器 ARR 相同
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
void foc_update(foc_t *foc, float mech_theta, float Iu, float Iv, float Iw) {
  // 更新角度
  foc->mech_theta = mech_theta;
  foc->elec_theta = mech_theta * foc->pole_pairs - foc->elec_theta_offset;

  foc->Iuvw.u = foc->lowpass_alpha_u * Iu + (1.0f - foc->lowpass_alpha_u) * foc->Iuvw.u;
  foc->Iuvw.v = foc->lowpass_alpha_v * Iv + (1.0f - foc->lowpass_alpha_v) * foc->Iuvw.v;
  foc->Iuvw.w = foc->lowpass_alpha_w * Iw + (1.0f - foc->lowpass_alpha_w) * foc->Iuvw.w;

  // 更新电流
  clarke(&foc->Iuvw, &foc->Iab);
  park(&foc->Iab, &foc->Idq, foc->elec_theta);
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
  foc->Udq.d = Ud;
  foc->Udq.q = Uq;

  inpark(&foc->Udq, &foc->Uab, foc->elec_theta);
  foc_setsvpwm(foc);
}

/**
 * @brief FOC 电流控制 (todo)
 */

/**
 * @brief FOC 速度控制 (todo)
 */

/**
 * @brief FOC 位置控制 (todo)
 */

