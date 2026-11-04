/*
 * foc_torque_cmd.c
 *
 * [3] Torque_cmd — Simulink: Torque_cmd block
 *     T_cmd (last_percent) → Id_ref / Iq_ref
 *
 * Formerly FOC.c (Torque_convertion, Regen_control, field_weaking_control,
 * MTPA_control, field_weaking_angle_control) + target calculation in
 * foc_control_step() (foc_loop.c).
 *
 * every motor constant and derived value should be line to neutral value
 */

#include "foc_torque_cmd.h"
#include <math.h>
#include <stdlib.h>          // abs()
#include "config.h"
#include "foc_math.h"
#include "pid.h"
#include "lowpass_filter.h"

#define FIELD_WEAKENING_DC_VOLTAGE_LIMIT 0.9f // 90% of DC bus voltage

static const int   pole_multipler = 4; // for 4 pole motor
static const float modulation_ref = 0.9f;

static pidc_t pid_controller_fw = {.P = FWKP, .I = FWKI, .D = PID_D, .output_ramp = PID_RAMP, .limit = 1, .Ts = (float)1/FREQ, .error_prev = 0, .output_prev = 0, .integral_prev = 0};
static lpf_t  filter_Idfw = {.Tf = FWTF, .Ts = (float)1/FREQ, .y_prev = 0.0f};

/* ================================================================
 *  Private helpers (formerly FOC.c)
 * ================================================================ */

/* Formerly Torque_convertion */
static float torque_to_iq(float last_percent, float Id)
{
    float T_cmd = motor.max_Torque * last_percent;
    float Iq = T_cmd / (1.5f * motor.pole_pairs * (motor.flux_linkage_m + (motor.Ld - motor.Lq) * Id));
    return Iq;
}

/* Formerly Regen_control */
static float regen_limit(float last_percent, float RPM, float Vdc, float max_ramp)
{
    float percent = last_percent;
    //T-N Quadrant regen logic
    int T_sign = percent >= 0.0f ? 1 : -1;
    int Speed_sign = RPM >= 0.0f ? 1 : -1;
    if (T_sign != Speed_sign)
    {
      return 0.0f;
    }

    //LCSP curve


    //Over Charge Protection
    float Vdc_upper_limit = Vdc * Vdc_upper_limit_scale; //4.2*0.98 = 4.116V
    float Vdc_lower_limit = Vdc * Vdc_lower_limit_scale; //4.2*0.90 = 3.78V
    float Vdc_derate = _constrain(((float)Vdc-(float)Vdc_upper_limit)/(Vdc_lower_limit-Vdc_upper_limit),0.0f,1.0f);
    // percent = _constrain(percent, -Vdc_derate, Vdc_derate);

    //speed constraint
    //start derate at low speed, end derate at MIN_REGEN_RPM
    float speed_derate = _constrain(((float)abs(RPM)-(float)RPM_DERATE_END)/(RPM_DERATE_START-RPM_DERATE_END),0.0f,1.0f);
    // percent = _constrain(percent, -speed_derate, speed_derate);
    //For safty
    if(abs(RPM) < abs(MIN_REGEN_RPM))
    {
      return 0.0f;
    }

     //slew rate limit
    float delta = percent - last_percent;
    delta = _constrain(delta, -max_ramp, max_ramp);
    percent = last_percent + delta;

    float derate = speed_derate < Vdc_derate ? speed_derate : Vdc_derate;
    percent = _constrain(percent, -derate, derate);

    return percent;
}

/* Formerly field_weaking_control */
static float field_weakening_id(float rpm, float Iq, float Vd, float Vdc)
{
    float omega_e = fabsf(rpm * pole_multipler * 2.0f * M_PI / 60.0f);
    Vdc = Vdc*FIELD_WEAKENING_DC_VOLTAGE_LIMIT; // convert DC bus voltage to line-line RMS voltage and limit to 90%
    omega_e = _constrain(omega_e, 1.0f, infinity()); // prevent division by zero
    float Vq = sqrtf(_constrain(Vdc * Vdc - Vd * Vd,0,infinity()))*0.707f;
    float Emag = rpm*motor.electrical_constant;
    float idfw_numerator = Vq - motor.Rs * fabsf(Iq) * 0.707 - Emag;
    float Idfw = 0.0f;
    if ( idfw_numerator < 0.0f)
    {
        Idfw = 1.414*idfw_numerator / (omega_e * motor.Ld);
        Idfw = _constrain(Idfw, -MAX_FLUX_ID, -MINIMUM_FW_ID);

    }
    Idfw = LowPassFilter_operator(Idfw,&filter_Idfw);
    Idfw = _constrain(Idfw, -MAX_FLUX_ID, 0.0f);
    return Idfw;
}

/* Formerly MTPA_control */
#ifdef MTPA
static float mtpa_id(float Iq)
{
    Iq = Iq * 0.707f; // convert to line-neutral value
    float L1 = 0.5*(motor.Ld - motor.Lq);
    float Id_optimal = 1.414f*(-motor.flux_linkage_m+sqrtf(motor.flux_linkage_m*motor.flux_linkage_m+16*L1*L1*Iq*Iq))/(4*L1);
    Id_optimal = _constrain(Id_optimal,-MAX_FLUX_ID,0.0f);
    return Id_optimal;
}
#endif

/* Formerly field_weaking_angle_control */
#ifdef FIELD_WEAKENING_ANGLE
static float field_weakening_angle(float *Iq, float *Id, float Vq, float Vd, float Vdc)
{
    Vdc = Vdc * FIELD_WEAKENING_DC_VOLTAGE_LIMIT; // convert DC bus voltage to line-line RMS voltage and limit to 90%
    Vq = Vq * 0.707f;
    Vd = Vd * 0.707f;
    float Vs = sqrtf(Vq * Vq + Vd * Vd);
    float modulation_index = Vs / Vdc;
    float angle_coefficient = 0.0f;
    angle_coefficient = PID_operator(modulation_index - modulation_ref, &pid_controller_fw);
    angle_coefficient = _constrain(angle_coefficient, -0.0f, 1.0f);

    float Is = sqrtf((*Iq) * (*Iq) + (*Id) * (*Id));
    float angle = M_PI - (M_PI - atan2(*Iq, *Id)) * angle_coefficient;
    *Id = Is * cosf(angle);
    *Iq = Is * sinf(angle);
    return angle;
}
#endif

/* ================================================================
 *  Public API
 * ================================================================ */

void torque_cmd_step(void)
{
  (void)pid_controller_fw;
  (void)modulation_ref;

  float temp_derate = 1.0f;
  #ifdef DISALBE_MOT_OT
  temp_derate = _constrain(((float)abs(T_Mot)-(float)T_DERATE_END)/(T_DERATE_START-T_DERATE_END),0.0f,1.0f);
  #endif

  // TO-DO! add regen logic here
  if (foc.enable_regen)
  {
    foc.last_percent = regen_limit(foc.last_percent, foc.filtered_RPM, motor.voltage_power_supply, foc.max_ramp);
  }
  //Power limitimg


  // calculate target Id and Iq based on last_percent and field weakening / MTPA control

  foc.last_percent = _constrain(foc.last_percent, -temp_derate, temp_derate);

  foc.target_Id = 0.0f;
  foc.target_Iq = motor.max_current * foc.last_percent;

  #ifdef TORQUE_CONTROL
  foc.target_Iq = torque_to_iq(foc.last_percent, foc.filtered_Id);
  #endif

  foc.Id_fw   = 0.0f;
  foc.Iq_fw   = foc.target_Iq;
  foc.Id_MTPA = 0.0f;

  /* foc.Id/Iq_controller_output still hold the previous cycle's Vd/Vq here (Simulink Feedback) */
  #ifdef FIELD_WEAKENING
  foc.Id_fw = field_weakening_id(fabsf(foc.filtered_RPM), fabsf(foc.filtered_Iq), fabsf(foc.Id_controller_output), motor.voltage_power_supply);
  #endif

  #ifdef MTPA
  foc.Id_MTPA = mtpa_id(fabsf(foc.filtered_Iq));
  #endif

  #ifdef FIELD_WEAKENING_ANGLE
  foc.Id_fw = foc.Id_MTPA;
  float fw_angle = field_weakening_angle(&foc.Iq_fw, &foc.Id_fw, foc.Iq_controller_output, foc.Id_controller_output, motor.voltage_power_supply);
  (void)fw_angle;
  #endif

  #ifdef PERMANENT_FLUX
  foc.Id_fw = -MAX_FLUX_ID;
  #endif

  foc.target_Id = foc.Id_fw < foc.Id_MTPA ? foc.Id_fw : foc.Id_MTPA;

  float max_Iq = sqrtf(motor.max_current*motor.max_current - foc.target_Id*foc.target_Id);
  foc.target_Iq = _constrain(foc.target_Iq, -max_Iq,max_Iq);
  #ifdef OVERSPEED_PROT
  foc.target_Iq = foc.Id_fw<-MAX_TORQUE_FW_ID?0.0f:foc.target_Iq;
  #endif
}

void torque_cmd_preset_fw_filter(float id_fw)
{
  filter_Idfw.y_prev = id_fw;
}
