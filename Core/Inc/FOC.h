#ifndef __FOC_H__
#define __FOC_H__

float Torque_convertion(float T_max, float last_percent, float Id);

float field_weaking_control(float rpm, float Iq, float Vd, float Vdc);

float MTPA_control(float Iq);

float field_weaking_angle_control(float *Iq, float *Id, float Vq, float Vd, float Vdc);
#endif /* __FOC_H__ */