/*
 * inverter_state.c
 *
 * Inverter State Machine & Fault Management
 * Owns the system state (Ready / Running / Error) and
 * provides the safety-critical state transition functions.
 *
 *  Created on: Feb 25, 2025
 *      Author: J.U
 */

#include "inverter_state.h"
#include "main.h"               // GPIO defines (Motor_Enable_Pin, LED_RUN_Pin, ...), _constrain
#include "foc_loop.h"           // foc, motor
#include "foc_transform.h"      // transform_idq_filter_reset()
#include "foc_current_ctrl.h"   // current_ctrl_reset(), current_ctrl_integral_reset()

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
  foc.enable_hw_oc = 0;
  HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_RUN_GPIO_Port,LED_RUN_Pin,GPIO_PIN_RESET);
  transform_idq_filter_reset();
  // LowPassFilter_reset(&filter_RPM);
}

void Enter_READY_State(void)
{
  inverter_state = STATE_READY;
  error_state = ERROR_NONE;
  foc.enable_hw_oc = 0;
  HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_RUN_GPIO_Port,LED_RUN_Pin,GPIO_PIN_SET);
  transform_idq_filter_reset();
  // LowPassFilter_reset(&filter_RPM);
}

/* ================================================================
 *  [0] Per-cycle state handling (called from the FOC ISR)
 * ================================================================ */

void foc_state_update(void)
{
  if (inverter_state == STATE_RUNNING)
  {
    float delta = foc.percent_torque_requested - foc.last_percent;
    delta = _constrain(delta, -foc.max_ramp, foc.max_ramp);
    if (foc.last_percent != 0.0f && foc.percent_torque_requested == 0.0f)
    {
      if (foc.fast_stop_enable)
      {
        current_ctrl_integral_reset();
      }
      foc.last_percent = 0.0f;
    }
    foc.last_percent = foc.last_percent + delta;
    //disable ramp limiting
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
    current_ctrl_reset();

    // pid_controller_current_Iq.output_prev = motor.dir*foc.filtered_RPM/motor.electrical_constant;
  }
}
