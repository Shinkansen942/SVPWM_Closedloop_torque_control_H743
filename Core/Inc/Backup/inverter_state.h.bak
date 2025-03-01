/*
* inerter_state.h
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
}INV_Statustypedef;

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
 
#endif /* INC_INVERTER_STATE_H_ */