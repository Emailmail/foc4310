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

// ---------------- FOC 硬件操作 ----------------
#if USE_SVPWM // SVPWM 实现

#else // SPWM 实现
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
#endif
// ---------------- foc 用户函数 ----------------
/**
 * @brief 注册 FOC
 */
void foc_register(foc_t *foc, bsp_pwm_t *pwmu, bsp_pwm_t *pwmv, bsp_pwm_t *pwmw, float vdc) {
  // 参数传递
  foc->pwmu = pwmu;
  foc->pwmv = pwmv;
  foc->pwmw = pwmw;
  foc->vdc = vdc;

  // 计算 ARR
  foc->arr = pwmu->arr; // 假设三路 PWM 定时器 ARR 相同
}

/**
 * @brief FOC 初始化
 */
void foc_init(foc_t *foc) {
  bsp_pwm_start(foc->pwmu);
  bsp_pwm_start(foc->pwmv);
  bsp_pwm_start(foc->pwmw);
}

#if (USE_SVPWM == 1) // SVPWM 实现

#else (USE_SVPWM == 0) // SPWM 实现
/**
 * @brief FOC 开环 (电角度模拟)
 */
void foc_openloop_virtual(foc_t *foc, float Ud, float Uq, float elec_theta) {
  foc->Udq.d = Ud;
  foc->Udq.q = Uq;
  foc->elec_theta = elec_theta;

  inpark(&foc->Udq, &foc->Uab, foc->elec_theta);
  inclarke(&foc->Uab, &foc->Uuvw);

  foc_setspwm(foc->pwmu, foc->pwmv, foc->pwmw, &foc->Uuvw, foc->vdc, foc->arr);
}
#endif
