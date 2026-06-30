/*
 * can_app.h
 *
 * CAN Application Layer - Handles all CAN bus communication,
 * including TX telemetry, RX command parsing, filter configuration,
 * and CAN error recovery.
 */

#ifndef INC_CAN_APP_H_
#define INC_CAN_APP_H_

#include "main.h"
#include "fdcan.h"
#include "inverter_state.h"

/* --------------- Public Variables --------------- */
extern int CAN_Timer;          // CAN timeout counter (reset on RX, checked in main loop)

/* --------------- Public Functions --------------- */
// FDCAN1 filter & notification configuration (call once after MX_FDCAN1_Init)
void Config_Fdcan1(void);

// Periodic TX functions (called from timer ISR in main.c)
void CAN_Send_State(uint16_t DCV, int16_t DCA);
void CAN_Send_Status(uint16_t report_status, int16_t FB_Torque, int16_t Speed);
void CAN_Send_Temp(uint16_t ADC_arr[6]);
void CAN_Send_Heartbeat(void);
void CAN_Send_Perameter(void);

// Note: HAL_FDCAN_RxFifo0Callback, HAL_FDCAN_RxFifo1Callback,
//       HAL_FDCAN_ErrorStatusCallback are weak-override HAL callbacks.
//       They don't need declaration here; the linker resolves them automatically.

#endif /* INC_CAN_APP_H_ */