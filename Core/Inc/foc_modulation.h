/*
 * foc_modulation.h
 *
 * [5] Modulation — Simulink: middle_clamp_modu
 *     Vd / Vq → inverse Park / Clarke → duty → middle clamp (SVPWM)
 * [7] Output — duty → TIM1->CCR1/2/3 (the only place that writes the PWM registers)
 *
 * Formerly setPhaseVoltage() / setPwm() in motor_control.c.
 */

#ifndef INC_FOC_MODULATION_H_
#define INC_FOC_MODULATION_H_

#include "stm32h7xx_hal.h"   // TIM_TypeDef
#include "foc_loop.h"        // dq_t, abc_t, duty_t

/* Pure: Vd/Vq + sin/cos(θe) + Vdc − per-phase correction → duty (0..1) */
duty_t modulation_step(dq_t v_dq, float sin_e, float cos_e, float vdc, abc_t v_sub);

/* ISR wrapper: modulation_step() fed from foc.Id/Iq_controller_output,
 * foc.Iabc_controller_output[] and motor.voltage_power_supply */
duty_t modulation_foc_step(float sin_e, float cos_e);

/* Hardware: duty → CCR1..3 of TIM_BASE */
void pwm_write(TIM_TypeDef *TIM_BASE, const duty_t *duty, int period);

/* Start-up helper (CAL_ZERO_ANGLE in main.c): apply a fixed Vd/Vq at angle_el */
void modulation_apply_voltage(dq_t v_dq, float angle_el, float vdc, TIM_TypeDef *TIM_BASE, int period);

#endif /* INC_FOC_MODULATION_H_ */
