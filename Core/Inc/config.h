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
#define SVPWM           //define to use SVPWM

// #define OPEN_LOOP_SPEED
#ifdef OPEN_LOOP_SPEED
#define OPEN_LOOP_RPM 12000 // Motor RPM
#endif

// Motor number
// #define MOT_RR
#define MOT_RL 
// #define MOT_CAL

// Protections
#define SOFTOCP 70
#define ACAOCP 72   
#define MOVRMSOCP (uint32_t)36000000 //should be 10000*OCP^2
#define MOS_OTP 1000        //should be 10 times otp temp in deg C
#define MOT_OTP 700         //should be 10 times otp temp in deg C
#define MOT_UTP 50        //should be 10 times utp temp in deg C
#define ENC_UV 50           //should be 3~5% Encoder adc full range
#define PID_P 1.0f
#define PID_I 0.02f
#define PID_D 0.0f
#define PID_RAMP 10000.0f
#define PID_LIMIT 20.0f
// #define RAMP_TIME 1.0f
#define HW_OC_TIME      2300    //should be in pwm cycles, 2300 is 100ms
#define SOFT_OC_TIME    50      //should be in pwm cycles, 50 is 2ms
#define ENC_TIME        50      //should be in pwm cycles, 50 is 2ms
#define QKP             2.4f
#define DKP             1.92f
#define QKI             1.5f
#define DKI             1.5f
#define QKD             0.0f
#define DKD             0.0f
// #define QKP             1.5f
// #define DKP             0.9f
// #define QKI             0.625f
// #define DKI             0.625f
// #define QKD             0.0f
// #define DKD             0.0f
#define DCKP            1.0f
#define DCKI            1.0f
#define DERATE_START        9000    //should be in RPM, 1000 is 1000RPM
#define DERATE_END          13000   //should be in RPM, 12000 is 12000RPM
#define RAMP_TIME_DERATE    5       //time from 0 to 100 percent

#define FREQ_23KHZ

#ifdef FREQ_11KHZ
#define FREQ 11000
#define CCR 10908 // 11000Hz PWM frequency
#define QTF 0.0008f
#define DTF 0.0008f
#define ABCTF 0.00008f
#define RPMTF 0.1f
#define DCTF 0.1f
#endif

#ifdef FREQ_13KHZ
#define FREQ 13000
#define CCR 9230 // 13000Hz PWM frequency
#define QTF 0.000677f
#define DTF 0.000677f
#define ABCTF 0.0000677f
#define RPMTF 0.846f
#define DCTF 0.846f
#endif

#ifdef FREQ_23KHZ
#define FREQ 23000
#define CCR 5127 // 23000Hz PWM frequency
#define QTF 0.0003826f
#define DTF 0.0003826f
#define ABCTF 0.00003826f
#define RPMTF 0.04783f
#define DCTF 0.04783f
#endif

#ifdef CAL_ZERO_ANGLE
#define ZERO_ELECTRIC_ANGLE 0.0f //should be in radians
#define MOT_CURR 1.0f
#define FILENAME "MOT_CAL_%04d%02d%02d_%02d%02d%02d_NEW_V2_4.bin"
#else
#ifdef MOT_CAL
#define ZERO_ELECTRIC_ANGLE 2.5f //should be in radians
#define MOT_CURR 1.084f
#define FILENAME "MOT_CAL_%04d%02d%02d_%02d%02d%02d_NEW_V2_4.bin"
#endif
#ifdef MOT_RR
#define ZERO_ELECTRIC_ANGLE 1.28f //should be in radians
#define MOT_CURR 0.907f
#define FILENAME "MOT_RR_%04d%02d%02d_%02d%02d%02d_NEW_V2_4.bin"
#endif
#ifdef MOT_RL
#define ZERO_ELECTRIC_ANGLE 4.74f //should be in radians
#define MOT_CURR 0.991f
#define FILENAME "MOT_RL_%04d%02d%02d_%02d%02d%02d_NEW_V2_4.bin"
#endif
#endif

#endif