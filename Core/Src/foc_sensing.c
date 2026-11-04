/*
 * foc_sensing.c
 *
 * [1] Sensing — Simulink: ADC block + Calculate Position and Speed
 *
 * Formerly: read_adc_buffers / update_encoder_and_voltage / current scaling (foc_loop.c),
 *           sincos_encoder.c, cal_angular_vel (motor_control.c), calibrateOffsets (current_sense.c)
 */

#include "foc_sensing.h"
#include "main.h"            // HAL, SCB_InvalidateDCache_by_Addr
#include "config.h"          // ACAPLSB, DCVPLSB, RPMTF, FREQ
#include "foc_math.h"
#include "foc_transform.h"   // angle_normalize, angle_electrical
#include "lowpass_filter.h"
#include "PLL/PLL.h"

/* Speed filter (formerly filter_RPM in main.c) */
static lpf_t filter_RPM = {.Tf = RPMTF, .Ts = (float)1/FREQ, .y_prev = 0.0f};

/* ================================================================
 *  Private helpers
 * ================================================================ */

/* Encoder-angle differentiation (formerly cal_angular_vel in motor_control.c) */
static float cal_angular_vel(float angle_now, float *speed_rad)
{
  float return_value = 0.0f;
  static float angle_prev = -1.0f;
  if (angle_prev < 0){
    	angle_prev=angle_now;
    	return 0;
    }
    float delta_angle=angle_now - angle_prev;
    if (delta_angle >= 1.6*M_PI){
    	delta_angle-=2*M_PI;
      return_value = -1.0f;
    }
    if (delta_angle <= -1.6*M_PI){
      delta_angle+=2*M_PI;
      return_value = 1.0f;
    }
    angle_prev=angle_now;
    *speed_rad = delta_angle / motor.Ts;
    return return_value;
}

/* ================================================================
 *  [1a] ADC snapshot
 * ================================================================ */

void sensing_read_raw(adc_raw_t *raw)
{
  SCB_InvalidateDCache_by_Addr(DMA_ADC1_arr, sizeof(DMA_ADC1_arr));
  for (size_t i = 0; i < 4; i++)
  {
    raw->adc1[i] = DMA_ADC1_arr[i];
  }
  SCB_InvalidateDCache_by_Addr(DMA_ADC2_arr, sizeof(DMA_ADC2_arr));
  for (size_t i = 0; i < 4; i++)
  {
    raw->adc2[i] = DMA_ADC2_arr[i];
  }
  SCB_InvalidateDCache_by_Addr(DMA_ADC3_arr, sizeof(DMA_ADC3_arr));
  for (size_t i = 0; i < 6; i++)
  {
    raw->adc3[i] = DMA_ADC3_arr[i];
  }
}

/* ================================================================
 *  [1b] ADC → physical quantities
 * ================================================================ */

void sensing_process(adc_raw_t *raw, const uint16_t current_offset[4], foc_meas_t *meas)
{
  /* Position — sin/cos encoder + PLL */
  float speed_rad, angle_pll;
  float raw_angle;  // throwaway buffer for sensing_get_encoder_angle; only angle_pll is used
  sensing_get_encoder_angle(raw->adc2, &raw_angle, &speed_rad, &angle_pll);
  foc.angle_now = angle_normalize(angle_pll);
  meas->theta_e = angle_electrical(foc.angle_now, motor.pole_pairs, motor.dir, motor.zero_electric_angle);
  _sincos(angle_normalize(meas->theta_e), &meas->sin_e, &meas->cos_e);

  /* DC bus voltage */
  #ifdef MAX_600V
  motor.voltage_power_supply = (float)raw->adc3[0] * DCVPLSB * 11/8;  //FR
  #else
  motor.voltage_power_supply = (float)raw->adc3[0] * DCVPLSB;
  #endif
  motor.voltage_limit = motor.voltage_power_supply;

  /* Speed — angle differentiation + LPF */
  float angular_vel = 0.0f;
  cal_angular_vel(foc.angle_now, &angular_vel);
  foc.filtered_RPM = LowPassFilter_operator((float)motor.dir*angular_vel/4/2/M_PI*60, &filter_RPM);
  meas->omega_e    = foc.filtered_RPM * 4.0f * 2.0f * (float)M_PI / 60.0f;

  /* Phase currents */
  for (int i = 0; i < 3; i++)
  {
    foc.current_phase[i] = (float)(raw->adc1[i] - current_offset[i]) * ACAPLSB;
  }
}

/* ================================================================
 *  Start-up helpers
 * ================================================================ */

/* Formerly calibrateOffsets (current_sense.c) */
void sensing_calibrate_offsets(uint16_t *offset, uint16_t *dma_adc)
{
	const int calibration_rounds = 1000;
	uint32_t temp_offset[4] = {0};

	// 查找0电流时候的电压
	for (int i = 0; i < calibration_rounds; i++) {
		SCB_InvalidateDCache_by_Addr(dma_adc,32);
		for (int j=0;j<4;j++){
			temp_offset[j]+=dma_adc[j];
		}
		HAL_Delay(5);
	}
	for (int i = 0; i < 4; i++)
	{
		offset[i] = temp_offset[i]/calibration_rounds;
	}
}

/* Formerly Get_Encoder_Angle (sincos_encoder.c) */
void sensing_get_encoder_angle(uint16_t *val_arr, float *angle_el, float_t *speed_rpm, float_t *rad_pll)
{
    int16_t _sv = val_arr[0]-val_arr[1];
    int16_t _cv = val_arr[2]-val_arr[3];

    *angle_el = angle_normalize(_atan2(_sv,_cv));

    PLL_step((real_T)_sv,(real_T)_cv, false, rad_pll, speed_rpm);
}
