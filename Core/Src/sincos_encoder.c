#include "sincos_encoder.h"


void Get_Encoder_Angle(uint16_t *val_arr, float *angle_el)
{
    static int16_t sv_mov[5] = {0};
    static int16_t cv_mov[5] = {0};
    static uint8_t index;
    int16_t _sv = val_arr[0]-val_arr[1];
    int16_t _cv = val_arr[2]-val_arr[3];
    sv_mov[index] = _sv;
    cv_mov[index] = _cv;
    int32_t _sv_sum = 0;
    int32_t _cv_sum = 0;
    for (size_t i = 0; i < 5; i++)
    {
        _sv_sum += sv_mov[i];
        _cv_sum += cv_mov[i];
    }
    // _sv = _sv_sum/5;
    // _cv = _cv_sum/5;
    
    static float angle_prev;
    *angle_el = _normalizeAngle(_atan2(_sv,_cv));  

    index++;
    if(index >= 5)
    {
        index = 0;
    }

}