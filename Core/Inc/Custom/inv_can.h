#ifndef INV_CAN_H
#define INV_CAN_H

#include "main.h"
#include "inv_main.h"
#include "inv_func.h"

void Config_Fdcan1(void);
void CAN_Send_State(uint16_t DCV, int16_t DCA);
void CAN_Send_Status(uint16_t report_status,int16_t FB_Torque,int16_t Speed);
void CAN_Send_Temp(uint16_t ADC_arr[6]);
void CAN_Send_Heartbeat(void);
void CAN_Send_Perameter(void);

#endif // INV_CAN_H