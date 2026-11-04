/*
 * foc_protection.h
 *
 * [6] Protection — sliding-window fault detection
 *
 * Call positions in the ISR are kept identical to the original foc_loop.c:
 *   protection_check_encoder() / protection_check_soft_oc()  : right after sensing
 *   protection_post_step()                                   : after telemetry (HW OC, RMS OC, CAN timeout)
 *
 * Formerly part of update_encoder_and_voltage(), measure_currents_and_check_oc(),
 * check_hw_overcurrent(), check_rms_overcurrent(), check_can_timeout() (foc_loop.c).
 */

#ifndef INC_FOC_PROTECTION_H_
#define INC_FOC_PROTECTION_H_

#include <stdint.h>
#include "foc_loop.h"   // protection_t

void protection_check_encoder(const uint16_t adc2[4], float angle_pll);
void protection_check_soft_oc(const float Iabc[3]);
void protection_post_step(void);

/* Clear HW-OC / soft-OC / encoder windows (called by can_app.c when the motor is enabled) */
void protection_reset(void);

/* Read-only access for logging (soft_oc_sum, RMS_sum) */
const protection_t *protection_state(void);

#endif /* INC_FOC_PROTECTION_H_ */
