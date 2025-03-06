#ifndef __CONFIG_H__
#define __CONFIG_H__

#define DEBUG           //undefine if don't need debug
#define CAL_ZERO_ANGLE  //undefine if don't need zero electrical angle calibration
#define TIMING          //undefine if don't need loop timing
#define MOT_RR
#define ACAOCP 50
#define MOS_OTP 1000 //should be 10 times otp temp in deg C
#define MOT_OTP 1200 //should be 10 times otp temp in deg C
#define ENC_UV 50    //should be 3~5% Encoder adc full range

#endif