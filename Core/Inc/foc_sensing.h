/*
 * foc_sensing.h
 *
 * [1] Sensing — Simulink: ADC block + Calculate Position and Speed
 *
 *   [1a] sensing_read_raw()  : DMA_ADCx_arr  → adc_raw_t   (must be the first action of the ISR)
 *   [1b] sensing_process()   : adc_raw_t     → foc.current_phase[], foc.angle_now, foc.filtered_RPM,
 *                                              motor.voltage_power_supply / voltage_limit,
 *                                              foc_meas_t (theta_e, sin/cos, omega_e)
 *
 * Formerly: read_adc_buffers / update_encoder_and_voltage / current scaling (foc_loop.c),
 *           sincos_encoder.c, cal_angular_vel (motor_control.c), calibrateOffsets (current_sense.c)
 */

#ifndef INC_FOC_SENSING_H_
#define INC_FOC_SENSING_H_

#include <stdint.h>
#include <math.h>        // float_t
#include "foc_loop.h"    // adc_raw_t, foc_meas_t

void sensing_read_raw(adc_raw_t *raw);
void sensing_process(adc_raw_t *raw, const uint16_t current_offset[4], foc_meas_t *meas);

/* Start-up helpers (main.c) */
void sensing_calibrate_offsets(uint16_t *offset, uint16_t *dma_adc);
void sensing_get_encoder_angle(uint16_t *val_arr, float *angle_el, float_t *speed_rpm, float_t *rad_pll);

#endif /* INC_FOC_SENSING_H_ */
