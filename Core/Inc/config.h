#ifndef __CONFIG_H__
#define __CONFIG_H__

// Code Config Options
#define SDDEBUG           //undefine to disable debug
#define CAL_ZERO_ANGLE  //undefine to disable zero electrical angle calibration
#define TIMING          //undefine to disable loop timing
// #define CAN_OT_FAULT    //undefine to disable CAN OVERTIME Falut
#define RMSOCP          //undefine to disable RMS overcurrent protection
// #define WAIT_GATE_READY //undefine to not wait for gate ready signal
#define OVERRIDE_OCP    //define to enable hardware ocp override
// #define CAN_CONFIG      //define to enable use CAN to change variables
// #define SIXSTEP         //define to enable six step comutation

// Motor number
#define MOT_RR

// Protections
#define SOFTOCP 10
#define ACAOCP 150
#define MOVRMSOCP 25000000 //should be 10000*OCP^2
#define MOS_OTP 1000 //should be 10 times otp temp in deg C
#define MOT_OTP 1200 //should be 10 times otp temp in deg C
#define ENC_UV 50    //should be 3~5% Encoder adc full range
#define PID_P 1.0f
#define PID_I 0.02f
#define PID_D 0.0f
#define PID_RAMP 1000.0f
#define PID_LIMIT 100000.0f

#endif