/*
 * lowpass_filter.c
 *
 *  Created on: Jun 18, 2023
 *      Author: hht
 */
#include "lowpass_filter.h"
extern float Ts;
float LowPassFilter_operator(float x, struct LowPassFilter* filter){
	float dt=Ts;
	float alpha = filter->Tf/(filter->Tf + dt);
	float y = alpha*filter->y_prev + (1.0f - alpha)*x;
	filter->y_prev = y;
	return y;
}
