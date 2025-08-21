#ifndef __LOGGER_H__
#define __LOGGER_H__

#include "stdint.h"

typedef struct __attribute__((packed, aligned(32))) LOGGER
{
    uint8_t     LGHR;       //Hour                          1Byte
    uint8_t     LGMIN;      //Minute                        1Byte
    uint8_t     LGSEC;      //Second                        1Byte
    uint8_t     LGERR;      //Error if present              1Byte
    uint16_t    LGSUBSEC;   //Subsecond                     2Byte
    uint16_t    LGDCV;      //10*DC Voltage                 2Byte
    int16_t     LGDCA;      //100*DC Current                2Byte
    int16_t     LGIU;       //100*U Phase Current           2Byte
    int16_t     LGIV;       //100*V Phase Current           2Byte
    int16_t     LGIW;       //100*W Phase Current           2Byte
    int16_t     LGVQ;       //10*VQ                         2Byte
    int16_t     LGVD;       //10*VD                         2Byte
    int16_t     LGSINE;     //S+-S-                         2Byte
    int16_t     LGCOS;      //C+-C-                         2Byte
    uint16_t    LGANG;      //100*electrical angle          2Byte
    int16_t     LGTCMD;     //1000*Percent torque requested 2Byte
    uint16_t    LGSTATE;    //State as per feedback         2Byte
    uint16_t    LGVU;       //CCR1                          2Byte
    uint16_t    LGVV;       //CCR2                          2Byte
    uint16_t    LGVW;       //CCR3                          2Byte
    int16_t     LGRPM;      //RPM                           2Byte
    int16_t     LGIQ;       //100*Q current                 2Byte
    int16_t     LGID;       //100*D current                 2Byte
    uint16_t    LGZERO;     //100*Zero electrical angle     2Byte
    int16_t     LGDCIU;     //100*DC U Phase Current        2Byte
    int16_t     LGDCIV;     //100*DC V Phase Current        2Byte
    int16_t     LGDCIW;     //100*DC W Phase Current        2Byte
    int16_t     LGVA;       //10*VA correction voltage      2Byte
    int16_t     LGVB;       //10*VB correction voltage      2Byte
    int16_t     LGVC;       //10*VC correction voltage      2Byte
    uint16_t    LGRMSIU;    //100*IU RMS current            2Byte
    uint16_t    LGRMSIV;    //100*IV RMS current            2Byte
    uint16_t    LGRMSIW;    //100*IW RMS current            2Byte
    uint16_t    LGLOGBUF;   //LOGBUF                        2Byte
} logger_t;

void set_one(logger_t log_struct[2][2]);

#endif
