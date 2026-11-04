/*
 * foc_current_ctrl.h
 *
 * [4] Current controllers + feedforward — Simulink: current controllers + feedforward
 *     Id_ref / Iq_ref, filtered Id / Iq → Vd / Vq
 *
 * Also hosts the per-phase DC-offset loop (not in the Simulink model).
 *
 * Formerly part of foc_control_step() and measure_currents_and_check_oc() (foc_loop.c);
 * PID instances formerly defined in main.c.
 */

#ifndef INC_FOC_CURRENT_CTRL_H_
#define INC_FOC_CURRENT_CTRL_H_

#include <stddef.h>
#include <stdint.h>
#include "foc_loop.h"   // dq_t, abc_t, foc
#include "pid.h"

/* Update the output limit of every current PI (called each cycle and at start-up) */
void current_ctrl_set_voltage_limit(float voltage_limit);

/* foc.Id/Iq_controller_output = PI(foc.target_* - foc.filtered_*) + decoupling,
 * constrained to ±motor.voltage_limit (also refreshes the PI limits) */
void current_ctrl_step(void);

/* Per-phase DC-offset loop, enabled above 1000 rpm (hysteresis to 750 rpm)
 * foc.current_phase[] → foc.phase_dc[], foc.Iabc_controller_output[], foc.enable_dc_control */
void dc_offset_ctrl_step(void);

/* Resets (used by foc_state_update) */
void current_ctrl_reset(void);            // full reset: Id, Iq, Iabc[3]
void current_ctrl_integral_reset(void);   // integral only: Iq, Id, Iabc[3]

/* Gain access for CAN parameter report / tuning (can_app.c) */
pidc_t *current_ctrl_pid_iq(void);
pidc_t *current_ctrl_pid_id(void);
pidc_t *current_ctrl_pid_dc(size_t phase);

#endif /* INC_FOC_CURRENT_CTRL_H_ */
