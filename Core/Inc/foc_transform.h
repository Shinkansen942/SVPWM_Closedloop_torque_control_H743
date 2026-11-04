/*
 * foc_transform.h
 *
 * [2] Coordinate transforms — Simulink: idq_cal + IDQ_Filter
 *     and the inverse transforms used by middle_clamp_modu.
 *
 * All functions are pure (no globals, no HAL) except transform_step(),
 * which reads / writes foc.* and owns the IDQ LPF state.
 */

#ifndef INC_FOC_TRANSFORM_H_
#define INC_FOC_TRANSFORM_H_

#include "foc_loop.h"   // abc_t, ab_t, dq_t

/* ---- Angle helpers ---- */
float angle_normalize(float angle);                                   // → [0, 2π)
float angle_electrical(float shaft_angle, int pole_pairs, int dir, float zero_offset);

/* ---- Forward transforms (idq_cal) ---- */
ab_t clarke(const float Iabc[3]);                  // Iabc → Iα, Iβ (zero-sequence removed)
dq_t park(ab_t i_ab, float theta_e);               // Iα, Iβ → Id, Iq
dq_t transform_abc_to_dq(const float Iabc[3], float theta_e);

/* ---- idq_cal + IDQ_Filter: foc.current_phase[] → foc.filtered_Id / filtered_Iq ---- */
void transform_step(float theta_e);
void transform_idq_filter_reset(void);

/* ---- Inverse transforms (middle_clamp_modu) ---- */
ab_t  inv_park(dq_t v_dq, float sin_e, float cos_e); // Vd, Vq → Vα, Vβ
abc_t inv_clarke(ab_t v_ab);                         // Vα, Vβ → Va, Vb, Vc

#endif /* INC_FOC_TRANSFORM_H_ */
