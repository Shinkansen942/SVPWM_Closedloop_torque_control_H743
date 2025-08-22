#include "inv_func.h"

void Enter_ERROR_State(INV_Errortypedef error)
{
  inverter_state = STATE_ERROR;
  error_state = error;
  enable_hw_oc = 0;
  HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_RUN_GPIO_Port,LED_RUN_Pin,GPIO_PIN_RESET);
  LowPassFilter_reset(&filter_current_Iabc[0]);
  LowPassFilter_reset(&filter_current_Iabc[1]);
  LowPassFilter_reset(&filter_current_Iabc[2]);
  LowPassFilter_reset(&filter_current_Iq);
  LowPassFilter_reset(&filter_current_Id);
  LowPassFilter_reset(&filter_RPM);
}

float MCU_MapValue(uint16_t in_value, float in_min, float in_max, float out_min, float out_max)
{
  return (float)(in_value - in_min) / (in_max - in_min) * (out_max - out_min) + out_min;
}
 
float MCU_TemperatureCalculate(uint16_t ts_data)
{
 
//  return (80 * (ts_data - ts_cal1)) / (ts_cal2 - ts_cal1) + 30;

  return MCU_MapValue(ts_data, ts_cal1, ts_cal2, TEMPSENSOR_CAL1_TEMP, TEMPSENSOR_CAL2_TEMP);
  return 1;
}


void set_time (uint8_t hr, uint8_t min, uint8_t sec)
{
	RTC_TimeTypeDef sTime = {0};
	sTime.Hours = hr;
	sTime.Minutes = min;
	sTime.Seconds = sec;
	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
	{
		Error_Handler();
	}
}

void set_date (uint8_t year, uint8_t month, uint8_t date, uint8_t day)  // monday = 1
{
	RTC_DateTypeDef sDate = {0};
	sDate.WeekDay = day;
	sDate.Month = month;
	sDate.Date = date;
	sDate.Year = year;
	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x2345);  // backup register
}