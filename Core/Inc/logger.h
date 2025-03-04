#ifndef __LOGGER_H__
#define __LOGGER_H__

#include "stdint.h"

struct logger
{
    uint32_t    LGTIM;  //Timestamp Format TBD
    uint16_t    LGDCV;  //10*DC Voltage
    int16_t     LGDCA;  //100*DC Current
    int16_t     LGIU;   //100*U Phase Current
    int16_t     LGIV;   //100*V Phase Current
    int16_t     LGIW;   //100*W Phase Current
    int16_t     LGTMOS; //10*T Report
    int16_t     LGTMOT; //10*T Motor
    int16_t     LGSINE; //S+-S-
    int16_t     LGCOS;  //C+-C-
    float       LGANG;  //electrical angle from encoder
    int16_t     LGTCMD; //10*Percent torque requested
    uint16_t    LGSTATE;//State as per feedback
};



#endif
