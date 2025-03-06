#include "sincos_encoder.h"

void Get_Encoder_Angle(uint16_t *val_arr, float *angle_el)
{
    uint16_t _sv = val_arr[0]-val_arr[1];
    uint16_t _cv = val_arr[2]-val_arr[3];
    *angle_el = _normalizeAngle(_atan2(_sv,_cv));
}