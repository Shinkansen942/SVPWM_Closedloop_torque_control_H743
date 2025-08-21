/*
 * motor_control.h
 *
 *  Created on: Jun 17, 2023
 *      Author: hht
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_
#include "stm32h7xx_hal.h"
#include <math.h>
#include "foc_utils.h"


#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define _SQRT3 1.73205080757f
#define _SQRT3_2 0.86602540378f
#define _1_SQRT3 0.57735026919f
#define _2_SQRT3 1.15470053838f

float _normalizeAngle(float angle);
float _electricalAngle(float shaft_angle, int pole_pairs);
void setPwm(float Ua, float Ub, float Uc, TIM_TypeDef * TIM_BASE);
void setPhaseVoltage(float Uq,float Ud, float angle_el, TIM_TypeDef * TIM_BASE,float Va,float Vb,float Vc);
void setSixStepPhaseVoltage(float Uq, float angle_el, TIM_TypeDef* TIM_BASE);
float cal_angular_vel(float angle_now,float* speed_RPM);
void cal_Idq(float* current_phase, float angle_el, float* Id, float* Iq);


#endif /* INC_MOTOR_CONTROL_H_ */
