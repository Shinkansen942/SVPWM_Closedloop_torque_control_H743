/*
 * foc_telemetry.h
 *
 * [8] Telemetry — LED blink, periodic CAN report, SD log entry
 *
 * Formerly update_leds(), periodic_can_report(), fill_log_entry() (foc_loop.c).
 */

#ifndef INC_FOC_TELEMETRY_H_
#define INC_FOC_TELEMETRY_H_

#include "foc_loop.h"            // adc_raw_t, foc, motor, telem

/* Increment the LED / heartbeat / status counters (once per ISR, before sensing) */
void telemetry_tick(void);

/* LED, CAN report (writes telem.report_*), SD log entry (writes telem.IU_100..IW_100) */
void telemetry_step(adc_raw_t *raw);

#endif /* INC_FOC_TELEMETRY_H_ */
