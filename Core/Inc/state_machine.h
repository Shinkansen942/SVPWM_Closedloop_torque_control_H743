/*
 * state_machine.h
 *
 * Inverter State Machine & Fault Management
 * Owns the system state (Ready / Running / Error) and
 * provides the safety-critical state transition functions.
 */

#ifndef INC_STATE_MACHINE_H_
#define INC_STATE_MACHINE_H_

#include "inverter_state.h"   // INV_Statustypedef, INV_Errortypedef

/* --------------- State Variables --------------- */
extern INV_Statustypedef inverter_state;
extern INV_Errortypedef  error_state;

/* --------------- Public Functions --------------- */
void Enter_ERROR_State(INV_Errortypedef error);
void Enter_READY_State(void);

#endif /* INC_STATE_MACHINE_H_ */