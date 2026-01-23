#ifndef __FOC_H__
#define __FOC_H__

float field_weaking_control(float rpm, float Iq, float Vd, float Vdc);

float MTPA_control(float Iq);

float field_weaking_angle_control(float* Iq, float* Id, float Vq, float Vd, float Vdc);
#endif /* __FOC_H__ */