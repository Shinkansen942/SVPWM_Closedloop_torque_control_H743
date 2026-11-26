/*
 * foc_protection.c
 *
 * [6] Protection — sliding-window fault detection
 *
 * Formerly part of update_encoder_and_voltage(), measure_currents_and_check_oc(),
 * check_hw_overcurrent(), check_rms_overcurrent(), check_can_timeout() (foc_loop.c);
 * `prot` formerly defined in main.c.
 */

#include "foc_protection.h"
#include "main.h"            // GPIO defines
#include "config.h"
#include "inverter_state.h"  // inverter_state, Enter_ERROR_State
#include "can_app.h"         // CAN_Timer

static protection_t prot = {0};

/* ================================================================
 *  Right after sensing
 * ================================================================ */

void protection_check_encoder(const uint16_t adc2[4], float angle_pll)
{
  int8_t enc_err = 0;
  if (angle_pll != angle_pll)
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
}

void protection_check_soft_oc(const float Iabc[3])
{
  int8_t soft_oc_detected = 0;
  prot.soft_oc_sum -= prot.soft_oc_buf[prot.soft_oc_index];

  for (int i = 0; i < 3; i++)
  {
    if (Iabc[i] > ACAOCP||Iabc[i] < -ACAOCP)
    {
      soft_oc_detected = 1;
    }
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
}

/* ================================================================
 *  After telemetry
 * ================================================================ */

static void check_hw_overcurrent(void)
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
static void check_rms_overcurrent(void)
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
static void check_can_timeout(void)
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

void protection_post_step(void)
{
  check_hw_overcurrent();

  #ifdef RMSOCP
  check_rms_overcurrent();
  #endif

  #ifdef CAN_OT_FAULT
  check_can_timeout();
  #endif
}

/* ================================================================
 *  Reset / access
 * ================================================================ */

void protection_reset(void)
{
  // Initialize hardware overcurrent protection buffer
  for (size_t i = 0; i < HW_OC_TIME; i++)
  {
    prot.oc_buf[i] = 0;
  }
  prot.oc_sum = 0;
  prot.oc_index = 0;

  // Initialize software overcurrent protection buffer
  prot.soft_oc_sum = 0;
  prot.soft_oc_index = 0;
  for (size_t i = 0; i < SOFT_OC_TIME; i++)
  {
    prot.soft_oc_buf[i] = 0;
  }

  // Initialize encoder fault detection buffer
  prot.enc_sum = 0;
  prot.enc_index = 0;
  for (size_t i = 0; i < ENC_TIME; i++)
  {
    prot.enc_buf[i] = 0;
  }
}

const protection_t *protection_state(void)
{
  return &prot;
}
