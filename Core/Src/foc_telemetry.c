/*
 * foc_telemetry.c
 *
 * [8] Telemetry — LED blink, periodic CAN report, SD log entry
 *
 * Formerly update_leds(), periodic_can_report(), fill_log_entry() (foc_loop.c).
 */

#include "foc_telemetry.h"
#include <math.h>
#include "main.h"            // GPIO defines
#include "config.h"
#include "inverter_state.h"  // inverter_state, error_state, REPORT_STATUS_*
#include "can_app.h"         // CAN_Send_*
#include "logger.h"          // logger_t, log_buf, log_*
#include "foc_protection.h"  // protection_state()
#include "tim.h"             // htim5
#include "rtc.h"             // hrtc

static int indexLED = 0;
static int indexHeartbeat = 0;
static int indexStatus = 0;

/* ================================================================
 *  Private helpers
 * ================================================================ */

static void update_leds(void)
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

static void fill_log_entry(uint16_t *adc2)
{
  const protection_t *prot = protection_state();

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
  entry->LGZERO = (uint16_t) prot->soft_oc_sum;
  entry->LGDCIU = (int16_t) roundf(foc.target_Id*100);
  entry->LGDCIV = (int16_t) roundf(foc.target_Iq*100);
  entry->LGDCIW = (int16_t) roundf(foc.phase_dc[2]*100);
  entry->LGVA = (int16_t) roundf(foc.Id_fw*10);
  entry->LGVB = (int16_t) roundf(foc.Id_MTPA*10);
  entry->LGVC = (int16_t) roundf(foc.Iabc_controller_output[2]*10);
  entry->LGRMSIU = (uint16_t) __HAL_TIM_GET_COUNTER(&htim5)>>16;
  entry->LGRMSIV = (uint16_t) __HAL_TIM_GET_COUNTER(&htim5);
  entry->LGRMSIW = (uint16_t) roundf(prot->RMS_sum[2]/100);
  entry->LGLOGBUF = wr_log_index;

  log_subsec++;
  wr_log_index++;
}

/* ================================================================
 *  Public API
 * ================================================================ */

void telemetry_tick(void)
{
  indexLED++;
  indexHeartbeat++;
  indexStatus++;
}

void telemetry_step(adc_raw_t *raw)
{
  update_leds();
  periodic_can_report(raw->adc1, raw->adc3);
  fill_log_entry(raw->adc2);
}
