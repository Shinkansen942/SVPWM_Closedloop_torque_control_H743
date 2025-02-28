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

void Get_Encoder_Angle(uint16_t* val_arr,float* angle_el);




#endif /* __SINCOS_ENCODER_H_ */