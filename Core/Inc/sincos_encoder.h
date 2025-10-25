/*
* sincos_encoder.h
*
*  Created on: Feb 25, 2025
*      Author: J.U
*/

#ifndef __SINCOS_ENCODER_H__
#define __SINCOS_ENCODER_H__

#include "stm32h7xx_hal.h"
#include "foc_utils.h"
#include "motor_control.h"
#include "math.h"

void Get_Encoder_Angle(uint16_t* val_arr,float* angle_el,float_t *speed_rpm, float_t *rad_pll);




#endif /* __SINCOS_ENCODER_H_ */