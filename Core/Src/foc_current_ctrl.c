/*
 * foc_current_ctrl.c
 *
 * [4] Current controllers + feedforward — Simulink: current controllers + feedforward
 *
 * Formerly part of foc_control_step() and measure_currents_and_check_oc() (foc_loop.c);
 * PID / LPF instances formerly defined in main.c.
 */

#include "foc_current_ctrl.h"
#include <math.h>
#include <stdlib.h>          // abs()
#include "config.h"          // DECOUPLING, gains, DCTF, FREQ
#include "foc_math.h"
#include "lowpass_filter.h"

/* Current PI controllers (formerly pid_controller_current_Iq / _Id / _Iabc in main.c) */
static pidc_t pid_controller_current_Iq = {.P=QKP,.I=QKI,.D=QKD,.output_ramp=PID_RAMP,.limit=PID_LIMIT,.Ts=(float)1/FREQ,.error_prev=0,.output_prev=0,.integral_prev=0};
static pidc_t pid_controller_current_Id = {.P=DKP,.I=DKI,.D=DKD,.output_ramp=PID_RAMP,.limit=PID_LIMIT,.Ts=(float)1/FREQ,.error_prev=0,.output_prev=0,.integral_prev=0};
static pidc_t pid_controller_current_Iabc[3] = {
    {.P=DCKP,.I=DCKI,.D=PID_D,.output_ramp=PID_RAMP,.limit=PID_LIMIT,.Ts=(float)1/FREQ,.error_prev=0,.output_prev=0,.integral_prev=0},
    {.P=DCKP,.I=DCKI,.D=PID_D,.output_ramp=PID_RAMP,.limit=PID_LIMIT,.Ts=(float)1/FREQ,.error_prev=0,.output_prev=0,.integral_prev=0},
    {.P=DCKP,.I=DCKI,.D=PID_D,.output_ramp=PID_RAMP,.limit=PID_LIMIT,.Ts=(float)1/FREQ,.error_prev=0,.output_prev=0,.integral_prev=0}
};

/* DC-offset loop input filters (formerly filter_current_DC_Iabc in main.c) */
static lpf_t filter_current_DC_Iabc[3] = {
    {.Tf = DCTF, .Ts = (float)1/FREQ, .y_prev = 0.0f},
    {.Tf = DCTF, .Ts = (float)1/FREQ, .y_prev = 0.0f},
    {.Tf = DCTF, .Ts = (float)1/FREQ, .y_prev = 0.0f}
};

/* ================================================================
 *  Current controllers + feedforward
 * ================================================================ */

void current_ctrl_set_voltage_limit(float voltage_limit)
{
  pid_controller_current_Id.limit = voltage_limit;
  pid_controller_current_Iq.limit = voltage_limit;
  pid_controller_current_Iabc[0].limit = voltage_limit;
  pid_controller_current_Iabc[1].limit = voltage_limit;
  pid_controller_current_Iabc[2].limit = voltage_limit;
}

void current_ctrl_step(void)
{
  float voltage_limit = motor.voltage_limit;
  float rpm = foc.filtered_RPM;

  current_ctrl_set_voltage_limit(voltage_limit);

  /* feedforward (decoupling) */
  float Vd_decoupling = (-1.0f)*(4*rpm*2*M_PI/60)*motor.Lq*foc.filtered_Iq;
  float Vq_decoupling = (4*rpm*2*M_PI/60)*(motor.Ld*foc.filtered_Id+motor.flux_linkage_m);
  Vq_decoupling = _constrain(Vq_decoupling, -voltage_limit, voltage_limit);
  Vd_decoupling = _constrain(Vd_decoupling, -voltage_limit, voltage_limit);

  /* current controllers (Vq_PI / Vd_PI) */
  foc.Iq_controller_output = PID_operator(foc.target_Iq - foc.filtered_Iq, &pid_controller_current_Iq);
  foc.Id_controller_output = PID_operator(foc.target_Id - foc.filtered_Id, &pid_controller_current_Id);

  #ifdef DECOUPLING
  // Decoupling
  foc.Id_controller_output += Vd_decoupling;
  foc.Iq_controller_output += Vq_decoupling;
  #endif

  foc.Id_controller_output = _constrain(foc.Id_controller_output, -voltage_limit, voltage_limit);
  foc.Iq_controller_output = _constrain(foc.Iq_controller_output, -voltage_limit, voltage_limit);
  // float max_Id = sqrtf(voltage_limit*voltage_limit - foc.Iq_controller_output*foc.Iq_controller_output);
  // foc.Id_controller_output = _constrain(foc.Id_controller_output,-max_Id,max_Id);
}

/* ================================================================
 *  Per-phase DC-offset loop
 * ================================================================ */

void dc_offset_ctrl_step(void)
{
  for (size_t i = 0; i < 3; i++)
  {
    foc.phase_dc[i] = LowPassFilter_operator(foc.current_phase[i], &filter_current_DC_Iabc[i]);
  }

  for (size_t i = 0; i < 3; i++)
  {
    foc.Iabc_controller_output[i] = PID_operator(-foc.phase_dc[i], &pid_controller_current_Iabc[i]);
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
      foc.Iabc_controller_output[i] = 0.0f;
      PID_reset(&pid_controller_current_Iabc[i]);
    }
  }
}

/* ================================================================
 *  Resets
 * ================================================================ */

void current_ctrl_reset(void)
{
  PID_reset(&pid_controller_current_Id);
  PID_reset(&pid_controller_current_Iq);
  for (size_t i = 0; i < 3; i++)
  {
    PID_reset(&pid_controller_current_Iabc[i]);
  }
}

void current_ctrl_integral_reset(void)
{
  PID_integral_reset(&pid_controller_current_Iq);
  PID_integral_reset(&pid_controller_current_Id);
  for (size_t i = 0; i < 3; i++)
  {
    PID_integral_reset(&pid_controller_current_Iabc[i]);
  }
}

/* ================================================================
 *  Gain access (can_app.c)
 * ================================================================ */

pidc_t *current_ctrl_pid_iq(void)            { return &pid_controller_current_Iq; }
pidc_t *current_ctrl_pid_id(void)            { return &pid_controller_current_Id; }
pidc_t *current_ctrl_pid_dc(size_t phase)    { return &pid_controller_current_Iabc[phase]; }
