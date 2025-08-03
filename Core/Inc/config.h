#ifndef __CONFIG_H__
#define __CONFIG_H__

// Code Config Options
// #define SDDEBUG           //undefine to disable debug
// #define CAL_ZERO_ANGLE  //undefine to disable zero electrical angle calibration
// #define TIMING          //undefine to disable loop timing
#define CAN_OT_FAULT    //undefine to disable CAN OVERTIME Falut
#define RMSOCP          //undefine to disable RMS overcurrent protection
// #define WAIT_GATE_READY //undefine to not wait for gate ready signal
// #define OVERRIDE_OCP    //define to enable hardware ocp override
#define CAN_CONFIG      //define to enable use CAN to change variables
// #define SIXSTEP         //define to enable six step comutation
#define VQ_LEQ_0        //define to use Vq less than 0
#define MIDDLE_CLAMP    //define to use middle clamp

// #define OPEN_LOOP_SPEED
#ifdef OPEN_LOOP_SPEED
#define OPEN_LOOP_RPM 60 // Motor RPM
#endif

// Motor number
#define MOT_RR

// Protections
#define SOFTOCP 70
#define ACAOCP 20   
#define MOVRMSOCP 100000000 //should be 10000*OCP^2
#define MOS_OTP 1000 //should be 10 times otp temp in deg C
#define MOT_OTP 1200 //should be 10 times otp temp in deg C
#define ENC_UV 50    //should be 3~5% Encoder adc full range
#define PID_P 1.0f
#define PID_I 0.02f
#define PID_D 0.0f
#define PID_RAMP 1000.0f
#define PID_LIMIT 20.0f
#define RAMP_TIME 1.0f
#define HW_OC_TIME 2300 //should be in pwm cycles, 4600 is 100ms
#define SOFT_OC_TIME 50 //should be in pwm cycles, 100 is 2ms
#define ENC_TIME 50 //should be in pwm cycles, 100 is 2ms

#ifdef CAL_ZERO_ANGLE
#define ZERO_ELECTRIC_ANGLE 0.0f //should be in radians
#else
#ifdef MOT_RR
#define ZERO_ELECTRIC_ANGLE 1.28f //should be in radians
#define MOT_CURR 0.907f
#define FILENAME "MOT_RR_%04d%02d%02d_%02d%02d%02d.bin"
#endif
#ifdef MOT_RL
#define ZERO_ELECTRIC_ANGLE 4.74f //should be in radians
#define MOT_CURR 0.991f
#define FILENAME "MOT_RL_%04d%02d%02d_%02d%02d%02d.bin"
#endif
#endif

#endif