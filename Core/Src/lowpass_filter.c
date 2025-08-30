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
	if (y == y)
	{
		filter->y_prev = y; // Update the previous value only if y is a valid number
	}
	
	return filter->y_prev;
}

void LowPassFilter_reset(lpf_t* filter)
{
	filter->y_prev = 0.0f; // Reset the previous value to zero
	// filter->timestamp_prev = 0; // Reset the timestamp if needed
}
