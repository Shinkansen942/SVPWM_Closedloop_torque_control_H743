#include "FOC.h"
#include <algorithm>
#include <math.h>
#include "config.h"
#include "motor_control.h"
#include "pid.h"
#include  "lowpass_filter.h"
#include "foc_loop.h"

#define FIELD_WEAKENING_DC_VOLTAGE_LIMIT 0.9f // 90% of DC bus voltage

// every motor constant and derived value should be line to neutral value

extern motor_params_t motor;
const int pole_multipler = 4; // for 4 pole motor
const float modulation_ref = 0.9f;

pidc_t pid_controller_fw = {.P = FWKP, .I = FWKI, .D = PID_D, .output_ramp = PID_RAMP, .limit = 1, .error_prev = 0, .output_prev = 0, .integral_prev = 0};
extern lpf_t filter_Idfw;

float Torque_convertion(float last_percent, float Id)
{
    float T_cmd = motor.max_Torque * last_percent;
    float Iq = T_cmd / (1.5f * motor.pole_pairs * (motor.flux_linkage_m + (motor.Ld - motor.Lq) * Id));
    return Iq;
}

float Regen_control(float last_percent, float RPM, float Vdc)
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
    if(abs(foc.filtered_RPM) < abs(MIN_REGEN_RPM))
    {
      return 0.0f;
    }

     //slew rate limit
    float delta = percent - last_percent;
    delta = _constrain(delta, -foc.max_ramp, foc.max_ramp);
    percent = last_percent + delta;

    float derate = speed_derate < Vdc_derate ? speed_derate : Vdc_derate;
    percent = _constrain(percent, -derate, derate);

    return percent;
}

float field_weaking_control(float rpm, float Iq, float Vd, float Vdc)
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

float MTPA_control(float Iq)
{
    Iq = Iq * 0.707f; // convert to line-neutral value
    float L1 = 0.5*(motor.Ld - motor.Lq);
    float Id_optimal = 1.414f*(-motor.flux_linkage_m+sqrtf(motor.flux_linkage_m*motor.flux_linkage_m+16*L1*L1*Iq*Iq))/(4*L1);
    Id_optimal = _constrain(Id_optimal,-MAX_FLUX_ID,0.0f);
    return Id_optimal;
}

float field_weaking_angle_control(float *Iq, float *Id, float Vq, float Vd, float Vdc)
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