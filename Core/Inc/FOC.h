#ifndef __FOC_H__
#define __FOC_H__

#include "stm32h7xx_hal.h"

float field_weaking_control(float rpm, float Iq, float Vd, float Vdc);

float MTPA_control(float Iq);

void open_loop_test(TIM_TypeDef * TIM_BASE);

#endif /* __FOC_H__ */