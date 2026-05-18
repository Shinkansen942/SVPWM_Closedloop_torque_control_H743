/*
* inverter_state.h
*
* Inverter State Machine & Fault Management
* Owns the system state (Ready / Running / Error) and
* provides the safety-critical state transition functions.
*
*  Copied on: Feb 25, 2025
*      Author: Iansun
*/

#ifndef INC_INVERTER_STATE_H_
#define INC_INVERTER_STATE_H_

#define BIT(x) (1 << (x))

typedef enum
{
    STATE_INIT,
    STATE_READY,
    STATE_RUNNING,
    STATE_ERROR,
} INV_Statustypedef;

typedef enum
{
    ERROR_NONE,
    ERROR_INSTANT_OC,
    ERROR_RMS_OC,
    ERROR_INV_OT,
    ERROR_MOT_OT,
    ERROR_ENC,
    ERROR_CAN_OT,
    ERROR_GATE,
    ERROR_HW_OC,
} INV_Errortypedef;

enum control_bits
{
    CTRL_ENABLE = BIT(3),
    CTRL_FAULT_RESET = BIT(5),
};

enum report_status_bits
{
    REPORT_STATUS_READY = BIT(1),
    REPORT_STATUS_ENABLED = BIT(2),
    REPORT_STATUS_FAULT = BIT(3),
    REPORT_STATUS_HV = BIT(4),
};

/* --------------- State Variables --------------- */
extern INV_Statustypedef inverter_state;
extern INV_Errortypedef  error_state;

/* --------------- Public Functions --------------- */
void Enter_ERROR_State(INV_Errortypedef error);
void Enter_READY_State(void);

#endif /* INC_INVERTER_STATE_H_ */