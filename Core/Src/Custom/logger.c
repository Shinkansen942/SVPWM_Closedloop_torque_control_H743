#include "logger.h"

void set_one(logger_t log_struct[2][2])
{
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            log_struct[i][j].LGHR = 0x1;
            log_struct[i][j].LGMIN = 0x1;
            log_struct[i][j].LGSEC = 0x1;
            log_struct[i][j].LGSUBSEC = 0x1;
            log_struct[i][j].LGDCV = 0x1;
            log_struct[i][j].LGDCA = 0x1;
            log_struct[i][j].LGIU = 0x1;
            log_struct[i][j].LGIV = 0x1;
            log_struct[i][j].LGIW = 0x1;
            log_struct[i][j].LGVQ = 0x1;
            log_struct[i][j].LGVD = 0x1;
            log_struct[i][j].LGSINE = 0x1;
            log_struct[i][j].LGCOS = 0x1;
            log_struct[i][j].LGANG = 0x1;
            log_struct[i][j].LGTCMD = 0x1;
            log_struct[i][j].LGSTATE = 0x1;
            log_struct[i][j].LGVV = 0x1;
            log_struct[i][j].LGVU = 0x1;
            log_struct[i][j].LGVW = 0x1;
            log_struct[i][j].LGRPM = 0x1;
        }
        
    }
    
}