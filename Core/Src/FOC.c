#include "FOC.h"
#include <math.h>
#include "config.h"
#include "motor_control.h"
#include "pid.h"

#define FIELD_WEAKENING_DC_VOLTAGE_LIMIT 0.9f // 90% of DC bus voltage

// every motor constant and derived value should be line to neutral value

extern const float Ld;
extern const float Lq;
extern const float flux_linkage_m;
extern const float electrical_constant;
extern const float Rs;
const int pole_multipler = 4; // for 4 pole motor
const float modulation_ref = 0.9f;

pidc_t pid_controller_fw = {.P = FWKP, .I = FWKI, .D = PID_D, .output_ramp = PID_RAMP, .limit = 1, .error_prev = 0, .output_prev = 0, .integral_prev = 0};

float field_weaking_control(float rpm, float Iq, float Vd, float Vdc)
{
    float omega_e = fabsf(rpm * pole_multipler * 2.0f * M_PI / 60.0f);
    Vdc = Vdc*FIELD_WEAKENING_DC_VOLTAGE_LIMIT; // convert DC bus voltage to line-line RMS voltage and limit to 90%
    omega_e = _constrain(omega_e, 1.0f, infinity()); // prevent division by zero
    float Vq = sqrtf(Vdc * Vdc - Vd * Vd)*0.707f;
    float Emag = rpm*electrical_constant;
    float idfw_numerator = Vq - Rs * fabsf(Iq) * 0.707 - Emag;
    float Idfw = 0.0f;
    if ( idfw_numerator < 0.0f)
    {
        Idfw = 1.414*idfw_numerator / (omega_e * Ld);
        _constrain(Idfw, -MAX_FLUX_ID, -MINIMUM_FW_ID);

    }
    return _constrain(Idfw, -MAX_FLUX_ID, 0.0f);
}

float MTPA_control(float Iq)
{
    Iq = Iq * 0.707f; // convert to line-neutral value
    float L1 = 0.5*(Ld - Lq);
    float Id_optimal = 1.414f*(-flux_linkage_m+sqrtf(flux_linkage_m*flux_linkage_m+16*L1*L1*Iq*Iq))/(4*L1);
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