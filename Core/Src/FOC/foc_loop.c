/*
 * foc_loop.c
 *
 * FOC control-loop ISR.
 *
 * Each step corresponds to one Simulink block and one module.
 * Modules communicate through the signal structs in foc_loop.h;
 * every debug / Live Watch global (foc.* / motor.*) is written
 * in each step that produces it.
 */

/* ---- System & HAL ---- */
#include "main.h"
#include <math.h>
#include <limits.h>            // INT16_MAX (for TIMING)

/* ---- Project modules ---- */
#include "foc_loop.h"          // signal structs, motor / foc / telem
#include "inverter_state.h"    // foc_state_update
#include "can_app.h"           // CAN_Timer
#include "foc_sensing.h"       // [1]
#include "foc_transform.h"     // [2]
#include "foc_torque_cmd.h"    // [3]
#include "foc_current_ctrl.h"  // [4]
#include "foc_modulation.h"    // [5] [7]
#include "foc_protection.h"    // [6]
#include "foc_telemetry.h"     // [8]
#include "tim.h"               // htim1, htim5

/* ================================================================
 *  Forward declarations for static functions
 * ================================================================ */
static bool foc_isr_should_run(void);
#ifdef TIMING
static void update_timing_stats(uint32_t tick_start);
#endif

/* ================================================================
 *  HAL Timer Callback - Main FOC Control Loop ISR
 * ================================================================ */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim != &htim1) return;
  if (!foc_isr_should_run()) return;
  HAL_GPIO_TogglePin(LED_TIM_GPIO_Port, LED_TIM_Pin);

  adc_raw_t  raw;    // ADC snapshot (DMA buffers are overwritten asynchronously)
  foc_meas_t meas;   // theta_e, sin_e / cos_e, omega_e
  duty_t     duty;   // PWM duty 0..1

  /* [1a] ADC snapshot */
  sensing_read_raw(&raw);

  /* [0] State & torque request (ramp / reset PI) */
  foc_state_update();

  #ifdef TIMING
  uint32_t tick_start = __HAL_TIM_GET_COUNTER(&htim5);
  #endif

  /* Cycle counters */
  telemetry_tick();                     // indexLED / indexHeartbeat / indexStatus
  CAN_Timer++;                          // CAN timeout (reset on RX in can_app.c)

  /* [1b] Sensing — Simulink: ADC block + Calculate Position and Speed */
  sensing_process(&raw, foc.current_offset, &meas);   // writes foc.angle_now, foc.current_phase[],
                                                      // foc.filtered_RPM, motor.voltage_power_supply / limit
  protection_check_encoder(raw.adc2, foc.angle_now);
  protection_check_soft_oc(foc.current_phase);

  /* [2] Transform — Simulink: idq_cal + IDQ_Filter */
  transform_step(meas.theta_e);         // → foc.filtered_Id / filtered_Iq

  /* [3] Torque_cmd — T_cmd → Id_ref / Iq_ref */
  torque_cmd_step();                    // → foc.target_Id / target_Iq, Id_fw, Iq_fw, Id_MTPA

  /* [4] Current controllers + feedforward → Vd / Vq */
  current_ctrl_step();                  // → foc.Id_controller_output / Iq_controller_output
  dc_offset_ctrl_step();                // → foc.phase_dc[], foc.Iabc_controller_output[]

  /* [5] Modulation — Simulink: middle_clamp_modu */
  duty = modulation_foc_step(meas.sin_e, meas.cos_e);

  /* [7] Output */
  pwm_write(TIM1, &duty, foc.period);

  /* [8] Telemetry — logging */
  telemetry_step(&raw);

  /* [6] Protection (HW OC, RMS OC, CAN timeout) */
  protection_post_step();

  #ifdef TIMING
  update_timing_stats(tick_start);
  #endif

  HAL_GPIO_TogglePin(LED_TIM_GPIO_Port,LED_TIM_Pin);
}

/* ================================================================
 *  Helper Functions
 * ================================================================ */

/* TIM1 update fires twice per PWM period (center-aligned) — run the loop on every other one */
static bool foc_isr_should_run(void)
{
  if (!foc.run)
  {
    foc.run = 1;
    return false;
  }
  foc.run = 0;
  return true;
}

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
