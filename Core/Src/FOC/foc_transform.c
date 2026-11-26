/*
 * foc_transform.c
 *
 * [2] Coordinate transforms — Simulink: idq_cal + IDQ_Filter
 *     and the inverse transforms used by middle_clamp_modu.
 *
 * Moved from motor_control.c (_normalizeAngle, _electricalAngle, cal_Idq,
 * inverse Park / Clarke part of setPhaseVoltage).
 */

#include "foc_transform.h"
#include "foc_math.h"
#include "lowpass_filter.h"

/* IDQ_Filter state (formerly filter_current_Id / filter_current_Iq in main.c) */
static lpf_t filter_current_Id = {.Tf = DTF, .Ts = (float)1/FREQ, .y_prev = 0.0f};
static lpf_t filter_current_Iq = {.Tf = QTF, .Ts = (float)1/FREQ, .y_prev = 0.0f};

/* ================================================================
 *  Angle helpers
 * ================================================================ */

float angle_normalize(float angle)
{
  float a = fmod(angle, 2*M_PI);   //取余运算可以用于归一化，列出特殊值例子算便知
  return a >= 0 ? a : (a + 2*M_PI);
}

float angle_electrical(float shaft_angle, int pole_pairs, int dir, float zero_offset)
{
  return angle_normalize(((float)(dir * pole_pairs)*shaft_angle)-zero_offset);
}

/* ================================================================
 *  Forward transforms — idq_cal
 * ================================================================ */

ab_t clarke(const float Iabc[3])
{
  float mid_current = (Iabc[0] + Iabc[1] + Iabc[2]) / 3.0f;
  // 将三相电流转换为两相电流
  float a = Iabc[0] - mid_current;
  float b = Iabc[1] - mid_current;
  ab_t i_ab = {
    .alpha = a,
    .beta  = _1_SQRT3*(2*b+a),
  };
  return i_ab;
}

dq_t park(ab_t i_ab, float theta_e)
{
  float angle_el = angle_normalize(theta_e);
  dq_t i_dq = {
    .d = cos(angle_el)*i_ab.alpha+sin(angle_el)*i_ab.beta,
    .q = -sin(angle_el)*i_ab.alpha+cos(angle_el)*i_ab.beta,
  };
  return i_dq;
}

dq_t transform_abc_to_dq(const float Iabc[3], float theta_e)
{
  return park(clarke(Iabc), theta_e);
}

/* ================================================================
 *  IDQ_Filter
 * ================================================================ */

void transform_step(float theta_e)
{
  dq_t i_dq = transform_abc_to_dq(foc.current_phase, theta_e);
  foc.filtered_Id = LowPassFilter_operator(i_dq.d, &filter_current_Id);
  foc.filtered_Iq = LowPassFilter_operator(i_dq.q, &filter_current_Iq);
}

void transform_idq_filter_reset(void)
{
  LowPassFilter_reset(&filter_current_Iq);
  LowPassFilter_reset(&filter_current_Id);
}

/* ================================================================
 *  Inverse transforms — middle_clamp_modu
 * ================================================================ */

ab_t inv_park(dq_t v_dq, float sin_e, float cos_e)
{
  ab_t v_ab = {
    .alpha = cos_e * v_dq.d - sin_e * v_dq.q,
    .beta  = sin_e * v_dq.d + cos_e * v_dq.q,
  };
  return v_ab;
}

abc_t inv_clarke(ab_t v_ab)
{
  abc_t v_abc = {
    .a = v_ab.alpha,
    .b = -0.5f * v_ab.alpha + _SQRT3_2 * v_ab.beta,
    .c = -0.5f * v_ab.alpha - _SQRT3_2 * v_ab.beta,
  };
  return v_abc;
}
