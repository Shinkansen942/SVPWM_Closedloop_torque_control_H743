#ifndef __LOGGER_H__
#define __LOGGER_H__

#include "stdint.h"

typedef struct LOGGER
{
    uint8_t     LGHR;       //Hour                          1Byte
    uint8_t     LGMIN;      //Minute                        1Byte
    uint8_t     LGSEC;      //Second                        1Byte
    // uint8_t     PAD;        //padding                       1Byte
    uint8_t     LGERR;      //Error if present              1Byte
    uint16_t    LGSUBSEC;   //Subsecond                     2Byte
    uint16_t    LGDCV;      //10*DC Voltage                 2Byte
    int16_t     LGDCA;      //100*DC Current                2Byte
    int16_t     LGIU;       //100*U Phase Current           2Byte
    int16_t     LGIV;       //100*V Phase Current           2Byte
    int16_t     LGIW;       //100*W Phase Current           2Byte
    uint16_t    LGTMOS;     //10*T Report                   2Byte
    uint16_t    LGTMOT;     //10*T Motor                    2Byte
    int16_t     LGSINE;     //S+-S-                         2Byte
    int16_t     LGCOS;      //C+-C-                         2Byte
    uint16_t    LGANG;      //100*electrical angle          2Byte
    int16_t     LGTCMD;     //10*Percent torque requested   2Byte
    uint16_t    LGSTATE;    //State as per feedback         2Byte
    uint16_t    LGCRC;      //CRC                           2Byte
} logger_t;

void set_one(logger_t log_struct[2][2]);

#endif
