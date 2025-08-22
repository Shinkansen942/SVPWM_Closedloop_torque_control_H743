#ifndef INV_FUNC_H
#define INV_FUNC_H

#include "main.h"
#include "inv_main.h"

float MCU_MapValue(uint16_t in_value, float in_min, float in_max, float out_min, float out_max);
float MCU_TemperatureCalculate(uint16_t ts_data);
void Enter_ERROR_State(INV_Errortypedef error);
void set_time (uint8_t hr, uint8_t min, uint8_t sec);
void set_date (uint8_t year, uint8_t month, uint8_t date, uint8_t day);

#endif