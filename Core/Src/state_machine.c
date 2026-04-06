/*
 * state_machine.c
 *
 * Inverter State Machine & Fault Management
 */

#include "state_machine.h"
#include "main.h"             // GPIO defines (Motor_Enable_Pin, LED_RUN_Pin, ...)
#include "lowpass_filter.h"   // LowPassFilter_reset()

/* ================================================================
 *  Extern declarations – variables defined in other modules
 * ================================================================ */

// --- Motor control (from main.c) ---
extern int    enable_hw_oc;
extern lpf_t  filter_current_Iabc[3];
extern lpf_t  filter_current_Iq;
extern lpf_t  filter_current_Id;

/* ================================================================
 *  State variables (owned by this module)
 * ================================================================ */

INV_Statustypedef inverter_state = STATE_INIT;
INV_Errortypedef  error_state    = ERROR_NONE;

/* ================================================================
 *  State transition functions
 * ================================================================ */

void Enter_ERROR_State(INV_Errortypedef error)
{
  inverter_state = STATE_ERROR;
  error_state = error;
  enable_hw_oc = 0;
  HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_RUN_GPIO_Port,LED_RUN_Pin,GPIO_PIN_RESET);
  LowPassFilter_reset(&filter_current_Iabc[0]);
  LowPassFilter_reset(&filter_current_Iabc[1]);
  LowPassFilter_reset(&filter_current_Iabc[2]);
  LowPassFilter_reset(&filter_current_Iq);
  LowPassFilter_reset(&filter_current_Id);
  // LowPassFilter_reset(&filter_RPM);
}

void Enter_READY_State(void)
{
  inverter_state = STATE_READY;
  error_state = ERROR_NONE;
  enable_hw_oc = 0;
  HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_RUN_GPIO_Port,LED_RUN_Pin,GPIO_PIN_SET);
  LowPassFilter_reset(&filter_current_Iabc[0]);
  LowPassFilter_reset(&filter_current_Iabc[1]);
  LowPassFilter_reset(&filter_current_Iabc[2]);
  LowPassFilter_reset(&filter_current_Iq);
  LowPassFilter_reset(&filter_current_Id);
  // LowPassFilter_reset(&filter_RPM);
}