/*
 * foc_modulation.c
 *
 * [5] Modulation — Simulink: middle_clamp_modu
 * [7] Output — duty → TIM1->CCR1/2/3
 *
 * Formerly setPhaseVoltage() / setPwm() in motor_control.c.
 * The legacy sector-based SVPWM (#else branch of MIDDLE_CLAMP) and
 * setSixStepPhaseVoltage() were unused and have been removed.
 */

#include "foc_modulation.h"
#include <math.h>
#include "config.h"          // MIDDLE_CLAMP, SVPWM
#include "foc_math.h"
#include "foc_transform.h"   // inv_park, inv_clarke, angle_normalize

#ifndef MIDDLE_CLAMP
#error "foc_modulation.c only implements MIDDLE_CLAMP modulation"
#endif

duty_t modulation_step(dq_t v_dq, float sin_e, float cos_e, float vdc, abc_t v_sub)
{
  /* inverse Park / inverse Clarke */
  abc_t u = inv_clarke(inv_park(v_dq, sin_e, cos_e));

  // For DC offset corretion
  u.a -= v_sub.a;
  u.b -= v_sub.b;
  u.c -= v_sub.c;

  //Check for Duty scaling
  duty_t d = {
    .Da = _constrain((u.a / vdc+1)/2,0.0f,1.0f),
    .Db = _constrain((u.b / vdc+1)/2,0.0f,1.0f),
    .Dc = _constrain((u.c / vdc+1)/2,0.0f,1.0f),
  };

  #ifdef SVPWM
  float center = 0.5f;
  // discussed here: https://community.simplefoc.com/t/embedded-world-2023-stm32-cordic-co-processor/3107/165?u=candas1
  // a bit more info here: https://microchipdeveloper.com/mct5001:which-zsm-is-best
  // Midpoint Clamp
  float Dmin = fminf(d.Da, fminf(d.Db, d.Dc));
  float Dmax = fmaxf(d.Da, fmaxf(d.Db, d.Dc));
  center -= (Dmax+Dmin) / 2;
  d.Da += center;
  d.Db += center;
  d.Dc += center;
  #endif

  return d;
}

duty_t modulation_foc_step(float sin_e, float cos_e)
{
  dq_t  v_dq  = { .d = foc.Id_controller_output, .q = foc.Iq_controller_output };
  abc_t v_sub = { -foc.Iabc_controller_output[0], -foc.Iabc_controller_output[1], -foc.Iabc_controller_output[2] };
  return modulation_step(v_dq, sin_e, cos_e, motor.voltage_power_supply, v_sub);
}

void pwm_write(TIM_TypeDef *TIM_BASE, const duty_t *duty, int period)
{
  // 限制占空比从0到1
  float dc_a = _constrain(duty->Da , 0.0f , 1.0f );
  float dc_b = _constrain(duty->Db , 0.0f , 1.0f );
  float dc_c = _constrain(duty->Dc , 0.0f , 1.0f );

  //写入PWM到PWM 0 1 2 通道
  TIM_BASE->CCR1 = (uint32_t) roundf(dc_a*period);
  TIM_BASE->CCR2 = (uint32_t) roundf(dc_b*period);
  TIM_BASE->CCR3 = (uint32_t) roundf(dc_c*period);
}

void modulation_apply_voltage(dq_t v_dq, float angle_el, float vdc, TIM_TypeDef *TIM_BASE, int period)
{
  float s, c;
  _sincos(angle_normalize(angle_el), &s, &c);
  abc_t zero = {0.0f, 0.0f, 0.0f};
  duty_t d = modulation_step(v_dq, s, c, vdc, zero);
  pwm_write(TIM_BASE, &d, period);
}
