#include "sincos_encoder.h"
#include "PLL/PLL.h"


void Get_Encoder_Angle(uint16_t *val_arr, float *angle_el,float_t *speed_rpm, float_t *rad_pll)
{    
    int16_t _sv = val_arr[0]-val_arr[1];
    int16_t _cv = val_arr[2]-val_arr[3];
    
    *angle_el = _normalizeAngle(_atan2(_sv,_cv));  

    PLL_step((real_T)_sv,(real_T)_cv, false, rad_pll, speed_rpm);
}