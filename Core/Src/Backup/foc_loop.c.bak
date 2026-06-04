/*
 * foc_loop.c
 *
 * FOC control-loop ISR and its helper functions.
  */

/* ---- System & HAL ---- */
#include "main.h"              // GPIO defines, HAL types; also pulls in sincos_encoder.h
#include <math.h>
#include <stdlib.h>            // abs()
#include <limits.h>            // INT16_MAX (for TIMING)

/* ---- Project modules ---- */
#include "foc_loop.h"          // motor_params_t, foc_state_t, protection_t, telemetry_t
#include "motor_control.h"     // _normalizeAngle, _electricalAngle, setPhaseVoltage,
                               // cal_angular_vel, cal_Idq, _constrain, _sign
#include "FOC.h"               // field_weaking_control, MTPA_control, field_weaking_angle_control
#include "pid.h"               // pidc_t, PID_operator, PID_reset, PID_integral_reset
#include "lowpass_filter.h"    // lpf_t, LowPassFilter_operator
#include "inverter_state.h"     // inverter_state, error_state, Enter_ERROR_State, Enter_READY_State
#include "can_app.h"           // CAN_Send_*, CAN_Timer
#include "logger.h"            // logger_t
#include "tim.h"               // htim1, htim5
#include "rtc.h"               // hrtc


/* ================================================================
 *  Forward declarations for static functions
 * ================================================================ */
static void read_adc_buffers(uint16_t *adc1, uint16_t *adc2, uint16_t *adc3);
static void handle_state_and_ramp(void);
static void update_encoder_and_voltage(uint16_t *adc2, uint16_t *adc3);
static void measure_currents_and_check_oc(uint16_t *adc1, float *phase_dc);
static void foc_control_step(float *phase_dc, float *Iabc_controller_output);
static void update_leds(void);
static void periodic_can_report(uint16_t *adc1, uint16_t *adc3);
static void fill_log_entry(uint16_t *adc2, float *current_phase_dc, float *Iabc_controller_output);
static void check_hw_overcurrent(void);
#ifdef RMSOCP
static void check_rms_overcurrent(void);
#endif
#ifdef CAN_OT_FAULT
static void check_can_timeout(void);
#endif
#ifdef TIMING
static void update_timing_stats(uint32_t tick_start);
#endif

/* ================================================================
 *  File-scope variables — only used within the ISR
 * ================================================================ */
static int indexLED = 0;
static int indexHeartbeat = 0;
static int indexStatus = 0;

/* ================================================================
 *  HAL Timer Callback - Main FOC Control Loop ISR
 *  This is the most critical function - called at FOC frequency
 * ================================================================ */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim != &htim1) return;

  if (!foc.run)
  {
    foc.run = 1;
    return;
  }
  foc.run = 0;

  HAL_GPIO_TogglePin(LED_TIM_GPIO_Port, LED_TIM_Pin);

  // ADC
  uint16_t ADC1_arr[4] = {0};
  uint16_t ADC2_arr[4] = {0};
  uint16_t ADC3_arr[6] = {0};
  read_adc_buffers(ADC1_arr, ADC2_arr, ADC3_arr);

  handle_state_and_ramp();

  #ifdef TIMING
  uint32_t tick_start = __HAL_TIM_GET_COUNTER(&htim5);
  #endif

  indexLED++;
  indexHeartbeat++;
  indexStatus++;
  CAN_Timer++;

  update_encoder_and_voltage(ADC2_arr, ADC3_arr);

  float current_phase_dc[3] = {0.0f};
  measure_currents_and_check_oc(ADC1_arr, current_phase_dc);

  float Iabc_controller_output[3] = {0.0f};
  foc_control_step(current_phase_dc, Iabc_controller_output);

  update_leds();

  periodic_can_report(ADC1_arr, ADC3_arr);

  // Logging
  fill_log_entry(ADC2_arr, current_phase_dc, Iabc_controller_output);

  check_hw_overcurrent();

  #ifdef RMSOCP
  check_rms_overcurrent();
  #endif

  #ifdef CAN_OT_FAULT
  check_can_timeout();
  #endif

  #ifdef TIMING
  update_timing_stats(tick_start);
  #endif

  HAL_GPIO_TogglePin(LED_TIM_GPIO_Port,LED_TIM_Pin);
}

/* ================================================================
 *  Helper Functions
 * ================================================================ */

// Read ADC values from DMA buffers
static void read_adc_buffers(uint16_t *adc1, uint16_t *adc2, uint16_t *adc3)
{
  SCB_InvalidateDCache_by_Addr(DMA_ADC1_arr, sizeof(DMA_ADC1_arr));
  for (size_t i = 0; i < 4; i++)
  {
    adc1[i] = DMA_ADC1_arr[i];
  }
  SCB_InvalidateDCache_by_Addr(DMA_ADC2_arr, sizeof(DMA_ADC2_arr));
  for (size_t i = 0; i < 4; i++)
  {
    adc2[i] = DMA_ADC2_arr[i];
  }
  SCB_InvalidateDCache_by_Addr(DMA_ADC3_arr, sizeof(DMA_ADC3_arr));
  for (size_t i = 0; i < 6; i++)
  {
    adc3[i] = DMA_ADC3_arr[i];
  }
}

static void handle_state_and_ramp()
{
  if (inverter_state == STATE_RUNNING)
  {
    float delta = foc.percent_torque_requested - foc.last_percent;
    delta = _constrain(delta, -foc.max_ramp, foc.max_ramp);
    if (foc.last_percent != 0.0f && foc.percent_torque_requested == 0.0f)
    {
      if (foc.fast_stop_enable)
      {
        PID_integral_reset(&pid_controller_current_Iq);
        PID_integral_reset(&pid_controller_current_Id);
        for (size_t i = 0; i < 3; i++)
        {
          PID_integral_reset(&pid_controller_current_Iabc[i]);
        }
      }
      foc.last_percent = 0.0f;
    }
    foc.last_percent = foc.last_percent + delta;
    foc.last_percent = foc.percent_torque_requested;
    HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_SET);
    if(motor.voltage_power_supply < 50.0f)
    {
      Enter_READY_State();
    }
  }
  else
  {
    foc.last_percent = 0.0f;
    HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_RESET);
  }
  if(inverter_state == STATE_READY|| inverter_state == STATE_ERROR)
  {
    PID_reset(&pid_controller_current_Id);
    PID_reset(&pid_controller_current_Iq);
    PID_reset(&pid_controller_current_OCP);
    PID_reset(&pid_controller_current_Ia);
    for (size_t i = 0; i < 3; i++)
    {
      PID_reset(&pid_controller_current_Iabc[i]);
    }

    // pid_controller_current_Iq.output_prev = motor.dir*foc.filtered_RPM/motor.electrical_constant;
  }
}

static void update_encoder_and_voltage(uint16_t *adc2, uint16_t *adc3)
{
  float speed_rad, angle_pll;
  float raw_angle;  // throwaway buffer for Get_Encoder_Angle; only angle_pll is used
  Get_Encoder_Angle(adc2, &raw_angle, &speed_rad, &angle_pll);
  int8_t enc_err = 0;
  foc.angle_now = angle_pll;
  if (foc.angle_now != foc.angle_now)
  {
    enc_err = 1;
  }
  for (size_t i = 0; i < 4; i++)
  {
    if(adc2[i] < ENC_UV)
    {
      #ifndef OPEN_LOOP_SPEED
      Enter_ERROR_State(ERROR_ENC);
      #endif
    }
    if (adc2[i] > 4096-ENC_UV)
    {
      #ifndef OPEN_LOOP_SPEED
      Enter_ERROR_State(ERROR_ENC);
      #endif
    }
  }
  prot.enc_sum -= prot.enc_buf[prot.enc_index];
  prot.enc_buf[prot.enc_index] = enc_err;
  prot.enc_sum += prot.enc_buf[prot.enc_index];
  if (prot.enc_sum > ENC_TIME/2)
  {
    if (inverter_state == STATE_RUNNING)
    {
      #ifndef OPEN_LOOP_SPEED
      Enter_ERROR_State(ERROR_ENC);
      #endif
    }
  }

  foc.angle_now = _normalizeAngle(foc.angle_now);

  motor.voltage_power_supply = (float)adc3[0] * DCVPLSB;
  motor.voltage_limit = motor.voltage_power_supply;

  float angular_vel = 0.0f;
  cal_angular_vel(foc.angle_now, &angular_vel);
  foc.filtered_RPM = LowPassFilter_operator((float)motor.dir*angular_vel/4/2/M_PI*60, &filter_RPM);
}

static void measure_currents_and_check_oc(uint16_t *adc1, float *phase_dc)
{
  int8_t soft_oc_detected = 0;
  prot.soft_oc_sum -= prot.soft_oc_buf[prot.soft_oc_index];

  for (int i = 0; i < 3; i++) {
      foc.current_phase[i] = (float)(adc1[i] - foc.current_offset[i]) * ACAPLSB;
      // OCP
      // filtered_Iabc[i] = LowPassFilter_operator(foc.current_phase[i],&filter_current_Iabc[i]);
      phase_dc[i] = LowPassFilter_operator(foc.current_phase[i], &filter_current_DC_Iabc[i]);
      if (foc.current_phase[i] > ACAOCP||foc.current_phase[i] < -ACAOCP)
      {
        soft_oc_detected = 1;
      }
      // foc.current_phase[i] = filtered_Iabc[i];
  }

  prot.soft_oc_buf[prot.soft_oc_index] = soft_oc_detected;
  prot.soft_oc_sum += prot.soft_oc_buf[prot.soft_oc_index];
  if (prot.soft_oc_sum > SOFT_OC_TIME/2 && inverter_state == STATE_RUNNING)
  {
    Enter_ERROR_State(ERROR_INSTANT_OC);
  }
  prot.soft_oc_index++;
  if (prot.soft_oc_index == SOFT_OC_TIME)
  {
    prot.soft_oc_index = 0;
  }

  pid_controller_current_Ia.limit = motor.voltage_limit;
  pid_controller_current_Id.limit = motor.voltage_limit;
  pid_controller_current_Iq.limit = motor.voltage_limit;
  pid_controller_current_OCP.limit = motor.voltage_limit;
  pid_controller_current_Iabc[0].limit = motor.voltage_limit;
  pid_controller_current_Iabc[1].limit = motor.voltage_limit;
  pid_controller_current_Iabc[2].limit = motor.voltage_limit;
}

static void foc_control_step(float *phase_dc, float *Iabc_controller_output)
{
  float temp_derate = 1.0f;
  #ifdef DISALBE_MOT_OT
  temp_derate = _constrain(((float)abs(T_Mot)-(float)T_DERATE_END)/(T_DERATE_START-T_DERATE_END),0.0f,1.0f);
  #endif

  foc.last_percent = _constrain(foc.last_percent, -temp_derate, temp_derate);
  float target_Is = motor.max_current * foc.last_percent;
  foc.target_Iq = target_Is;
  foc.target_Id = 0.0f;

  float Id, Iq;
  cal_Idq(foc.current_phase, _electricalAngle(foc.angle_now, motor.pole_pairs), &Id, &Iq);
  foc.filtered_Iq = LowPassFilter_operator(Iq, &filter_current_Iq);
  foc.filtered_Id = LowPassFilter_operator(Id, &filter_current_Id);

  float Vd_decoupling = (-1.0f)*(4*foc.filtered_RPM*2*M_PI/60)*motor.Lq*foc.filtered_Iq;
  float Vq_decoupling = (4*foc.filtered_RPM*2*M_PI/60)*(motor.Ld*foc.filtered_Id+motor.flux_linkage_m);
  Vq_decoupling = _constrain(Vq_decoupling, -motor.voltage_limit, motor.voltage_limit);
  Vd_decoupling = _constrain(Vd_decoupling, -motor.voltage_limit, motor.voltage_limit);

  foc.Id_fw = 0.0f;
  foc.Iq_fw = foc.target_Iq;
  foc.Id_MTPA = 0.0f;

  #ifdef FIELD_WEAKENING
  foc.Id_fw = field_weaking_control(fabsf(foc.filtered_RPM), fabsf(foc.filtered_Iq), fabsf(foc.Iq_controller_output), motor.voltage_limit);
  #endif

  #ifdef MTPA
  foc.Id_MTPA = MTPA_control(fabsf(filtered_Iq));
  #endif

  #ifdef FIELD_WEAKENING_ANGLE
  foc.Id_fw = foc.Id_MTPA;
  float fw_angle = field_weaking_angle_control(&foc.Iq_fw, &foc.Id_fw, foc.Iq_controller_output, foc.Id_controller_output, motor.voltage_limit);
  #endif

  #ifdef PERMANENT_FLUX
  foc.Id_fw = -MAX_FLUX_ID;
  #endif

  float Id_flux_control = foc.Id_fw < foc.Id_MTPA ? foc.Id_fw : foc.Id_MTPA;
  // foc.target_Id = _constrain(Id_flux_control,(-MAX_FLUX_ID)*fabsf(foc.last_percent)*4.0f,0.0f);
  foc.target_Id = Id_flux_control;
  // if(filtered_RPM > motor.voltage_power_supply/(motor.electrical_constant+0.02f) && foc.last_percent == 0.0f)
  // {
  //   foc.target_Iq = -1.0f * _sign(filtered_RPM);
  // }
  // foc.target_Id = Id_flux_control;
  float max_Iq = sqrtf(motor.max_current*motor.max_current - foc.target_Id*foc.target_Id);
  foc.target_Iq = _constrain(foc.target_Iq, -max_Iq,max_Iq);
  #ifdef OVERSPEED_PROT
  foc.target_Iq = foc.Id_fw<-MAX_TORQUE_FW_ID?0.0f:foc.target_Iq;
  #endif

  foc.Iq_controller_output = PID_operator(foc.target_Iq - foc.filtered_Iq, &pid_controller_current_Iq);
  foc.Id_controller_output = PID_operator(foc.target_Id - foc.filtered_Id, &pid_controller_current_Id);

  #ifdef Decouopling
  // Decoupling
  foc.Id_controller_output += Vd_decoupling;
  foc.Iq_controller_output += Vq_decoupling;
  #endif

  foc.Id_controller_output = _constrain(foc.Id_controller_output, -motor.voltage_limit, motor.voltage_limit);
  foc.Iq_controller_output = _constrain(foc.Iq_controller_output, -motor.voltage_limit, motor.voltage_limit);
  // float max_Id = sqrtf(motor.voltage_limit*motor.voltage_limit - foc.Iq_controller_output*foc.Iq_controller_output);
  // foc.Id_controller_output = _constrain(foc.Id_controller_output,-max_Id,max_Id);

  for (size_t i = 0; i < 3; i++)
  {
    Iabc_controller_output[i] = PID_operator(-phase_dc[i], &pid_controller_current_Iabc[i]);
  }
  if (abs(foc.filtered_RPM) > 1000 && foc.enable_dc_control == 0)
  {
    foc.enable_dc_control = 1;
  }
  else if (abs(foc.filtered_RPM) < 750 && foc.enable_dc_control == 1)
  {
    foc.enable_dc_control = 0;
  }
  if (!foc.enable_dc_control)
  {
    for (size_t i = 0; i < 3; i++)
    {
      Iabc_controller_output[i] = 0.0f;
      PID_reset(&pid_controller_current_Iabc[i]);
    }
  }

  setPhaseVoltage(foc.Iq_controller_output, foc.Id_controller_output, _electricalAngle(foc.angle_now, motor.pole_pairs),TIM1,-Iabc_controller_output[0],-Iabc_controller_output[1],-Iabc_controller_output[2]);
}

static void update_leds()
{
  if (indexLED == foc.freq/2)
  {
    if (inverter_state == STATE_READY)
    {
      HAL_GPIO_TogglePin(LED_RUN_GPIO_Port, LED_RUN_Pin);
    }
    else if (inverter_state == STATE_ERROR)
    {
      HAL_GPIO_TogglePin(LED_ERR_GPIO_Port, LED_ERR_Pin);
    }

    indexLED = 0;
  }
}

static void periodic_can_report(uint16_t *adc1, uint16_t *adc3)
{
  telem.report_status = 0;
  telem.report_DCV = (uint16_t) roundf(motor.voltage_power_supply*100);
  telem.report_DCA = (int16_t) roundf((float)(adc1[3] - foc.current_offset[3])*DCAPLSB*100);
  if (indexHeartbeat == foc.freq/10)
  {
    CAN_Send_Temp(adc3);
    CAN_Send_State(telem.report_DCV, telem.report_DCA);
    CAN_Send_Heartbeat();
    CAN_Send_Perameter();
    // CAN_Send_Heartbeat();
    indexHeartbeat = 0;
  }

  if (inverter_state == STATE_READY)
  {
    telem.report_status |= REPORT_STATUS_READY;
  } else if (inverter_state == STATE_RUNNING)
  {
    telem.report_status |= REPORT_STATUS_ENABLED;
  } else if (inverter_state == STATE_ERROR)
  {
    telem.report_status |= REPORT_STATUS_FAULT;
  }
  if (adc3[0]*DCVPLSB > 55)
  {
    telem.report_status |= REPORT_STATUS_HV;
  }

  if (indexStatus == foc.freq/100)
  {
    // int_RPM = (int) lroundf(filtered_RPM);
    int16_t report_RPM = (int16_t) roundf(foc.filtered_RPM);
    // int16_t report_RPM = (int16_t) roundf(zero_cross/4*100*60);
    foc.zero_cross = 0.0f;
    int16_t report_torque = (int16_t) roundf(foc.filtered_Iq/motor.max_current*1000);
    // int16_t report_torque = (int16_t) roundf(foc.Ia/motor.max_current*1000);
    // report_torque = LowPassFilter_operator(report_torque,&filter_report_torque);
    CAN_Send_Status(telem.report_status, report_torque, report_RPM);
    indexStatus = 0;
  }
}

static void fill_log_entry(uint16_t *adc2, float *current_phase_dc, float *Iabc_controller_output)
{
  telem.IU_100 = (int16_t)roundf(foc.current_phase[0]*100);
  telem.IV_100 = (int16_t)roundf(foc.current_phase[1]*100);
  telem.IW_100 = (int16_t)roundf(foc.current_phase[2]*100);
  HAL_RTC_GetDate(&hrtc, &log_date, RTC_FORMAT_BIN);
  HAL_RTC_GetTime(&hrtc, &log_time, RTC_FORMAT_BIN);
  if (log_time.Seconds != last_sec)
  {
    log_subsec = 0;
    last_sec = log_time.Seconds;
  }
  logger_t *entry = &log_buf[wr_log_buf_num][wr_log_index%3600];
  entry->LGHR = log_time.Hours;
  entry->LGMIN = log_time.Minutes;
  entry->LGSEC = log_time.Seconds;
  entry->LGERR = error_state;
  entry->LGSUBSEC = log_subsec;
  entry->LGDCV = telem.report_DCV;
  entry->LGDCA = telem.report_DCA;
  entry->LGIU = telem.IU_100;
  entry->LGIV = telem.IV_100;
  entry->LGIW = telem.IW_100;
  entry->LGVQ = (int16_t) roundf(foc.Iq_controller_output*10);
  entry->LGVD = (int16_t) roundf(foc.Id_controller_output*10);
  entry->LGSINE = adc2[0] - adc2[1];
  entry->LGCOS = adc2[2] - adc2[3];
  entry->LGANG = (uint16_t) roundf(foc.angle_now*100*180/M_PI);
  entry->LGTCMD = (int16_t) roundf(foc.last_percent*1000);
  entry->LGSTATE = telem.report_status;
  entry->LGVU = TIM1->CCR1;
  entry->LGVV = TIM1->CCR2;
  entry->LGVW = TIM1->CCR3;
  entry->LGRPM = (int16_t) roundf(foc.filtered_RPM);
  entry->LGID = (int16_t) roundf(foc.filtered_Id*100);
  entry->LGIQ = (int16_t) roundf(foc.filtered_Iq*10);
  entry->LGZERO = (uint16_t) prot.soft_oc_sum;
  entry->LGDCIU = (int16_t) roundf(foc.target_Id*100);
  entry->LGDCIV = (int16_t) roundf(foc.target_Iq*100);
  entry->LGDCIW = (int16_t) roundf(current_phase_dc[2]*100);
  entry->LGVA = (int16_t) roundf(foc.Id_fw*10);
  entry->LGVB = (int16_t) roundf(foc.Id_MTPA*10);
  entry->LGVC = (int16_t) roundf(Iabc_controller_output[2]*10);
  entry->LGRMSIU = (uint16_t) __HAL_TIM_GET_COUNTER(&htim5)>>16;
  entry->LGRMSIV = (uint16_t) __HAL_TIM_GET_COUNTER(&htim5);
  entry->LGRMSIW = (uint16_t) roundf(prot.RMS_sum[2]/100);
  entry->LGLOGBUF = wr_log_index;

  log_subsec++;
  wr_log_index++;
}

static void check_hw_overcurrent()
{
  if(inverter_state == STATE_RUNNING)
  {
    prot.oc_sum -= prot.oc_buf[prot.oc_index];
    prot.oc_buf[prot.oc_index] = (HAL_GPIO_ReadPin(OC_Fault_GPIO_Port,OC_Fault_Pin) == GPIO_PIN_RESET) ? 1 : 0;
    prot.oc_sum += prot.oc_buf[prot.oc_index];
    if (prot.oc_sum > HW_OC_TIME/2)
    {
      Enter_ERROR_State(ERROR_HW_OC);
    }
  }
  prot.oc_index++;
  if (prot.oc_index == HW_OC_TIME)
  {
    prot.oc_index = 0;
  }
}

#ifdef RMSOCP
static void check_rms_overcurrent()
{
  // Moving RMS for phase currents
  for (size_t i = 0; i < 3; i++)
  {
    prot.RMS_sum[i] -= (prot.RMS_buf[i][prot.indexRMS] * prot.RMS_buf[i][prot.indexRMS]);
  }
  prot.RMS_buf[0][prot.indexRMS] = telem.IU_100;
  prot.RMS_buf[1][prot.indexRMS] = telem.IV_100;
  prot.RMS_buf[2][prot.indexRMS] = telem.IW_100;
  for (size_t i = 0; i < 3; i++)
  {
    prot.RMS_sum[i] += (prot.RMS_buf[i][prot.indexRMS] * prot.RMS_buf[i][prot.indexRMS]);
  }
  prot.indexRMS++;
  if (prot.indexRMS == 10000)
  {
    prot.indexRMS = 0;
  }
  for (size_t i = 0; i < 3; i++)
  {
    if (prot.RMS_sum[i]/10000 > MOVRMSOCP)
    {
      Enter_ERROR_State(ERROR_RMS_OC);
    }
  }
}
#endif

#ifdef CAN_OT_FAULT
static void check_can_timeout()
{
  // CAN fault detect
  if (CAN_Timer == foc.freq && inverter_state == STATE_RUNNING)
  {
    foc.enable_hw_oc = 0;
    HAL_GPIO_WritePin(Motor_Enable_GPIO_Port, Motor_Enable_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_RESET);
    inverter_state = STATE_READY;
  }
}
#endif

#ifdef TIMING
static void update_timing_stats(uint32_t tick_start)
{
  loop_time = __HAL_TIM_GET_COUNTER(&htim5) - tick_start;
  if (loop_time > max_time)
  {
    max_time = loop_time;
  }
  if (loop_time < min_time)
  {
    min_time = loop_time;
  }
  int btw_time = tick_start - prev_time;
  if (btw_time > max_btw)
  {
    max_btw = btw_time;
  }
  prev_time = tick_start;
  if (indexTimer == foc.freq*10)
  {
    max_time = 0;
    min_time = INT16_MAX;
    max_btw = 0;
    indexTimer = 0;
    max_sdwrite = 0;
  }
  indexTimer++;
}
#endif