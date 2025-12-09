/*
 * mortor_control.c
 *
 *  Created on: Jun 17, 2023
 *      Author: hht
 */

#include "motor_control.h"
#include "config.h"
#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif
#define _HIGH_IMPEDANCE 0

extern float zero_electric_angle;
extern int pole_pairs;
extern float shaft_angle;
extern int dir;
extern float voltage_limit;
extern float voltage_power_supply;
extern int period;
// extern float angle_prev;
extern float Ts;
extern float max_current;

int trap_120_map[6][3] = {
  {_HIGH_IMPEDANCE,1,-1},
  {-1,1,_HIGH_IMPEDANCE},
  {-1,_HIGH_IMPEDANCE,1},
  {_HIGH_IMPEDANCE,-1,1},
  {1,-1,_HIGH_IMPEDANCE},
  {1,_HIGH_IMPEDANCE,-1} 
};


float _normalizeAngle(float angle){
  float a = fmod(angle, 2*M_PI);   //取余运算可以用于归一化，列出特殊值例子算便知
  return a >= 0 ? a : (a + 2*M_PI);
  //三目运算符。格式：condition ? expr1 : expr2
  //其中，condition 是要求值的条件表达式，如果条件成立，则返回 expr1 的值，否则返回 expr2 的值。
  //可以将三目运算符视为 if-else 语句的简化形式。
  //fmod 函数的余数的符号与除数相同。因此，当 angle 的值为负数时，余数的符号将与 _2M_PI 的符号相反。
  //也就是说，如果 angle 的值小于 0 且 _2M_PI 的值为正数，则 fmod(angle, _2M_PI) 的余数将为负数。
  //例如，当 angle 的值为 -M_PI/2，_2M_PI 的值为 2M_PI 时，fmod(angle, _2M_PI) 将返回一个负数。
  //在这种情况下，可以通过将负数的余数加上 _2M_PI 来将角度归一化到 [0, 2M_PI] 的范围内，以确保角度的值始终为正数。
}

float _electricalAngle(float shaft_angle, int pole_pairs) {
  return _normalizeAngle(((float)(dir * pole_pairs)*shaft_angle)-zero_electric_angle);
}

void setPwm(float Ua, float Ub, float Uc, TIM_TypeDef * TIM_BASE) {
//	// 限制上限
	// Ua = _constrain(Ua, 0.0f, voltage_limit);
	// Ub = _constrain(Ub, 0.0f, voltage_limit);
	// Uc = _constrain(Uc, 0.0f, voltage_limit);
	// 计算占空比
	// 限制占空比从0到1
	// float dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
	// float dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
	// float dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f );

  float dc_a = _constrain(Ua , 0.0f , 1.0f );
	float dc_b = _constrain(Ub , 0.0f , 1.0f );
	float dc_c = _constrain(Uc , 0.0f , 1.0f );

	//写入PWM到PWM 0 1 2 通道
	TIM_BASE->CCR1 = (uint32_t) roundf(dc_a*period);
	TIM_BASE->CCR2 = (uint32_t) roundf(dc_b*period);
	TIM_BASE->CCR3 = (uint32_t) roundf(dc_c*period);

}

void setPhaseVoltage(float Uq,float Ud, float angle_el, TIM_TypeDef * TIM_BASE,float Va,float Vb,float Vc) {
  angle_el = _normalizeAngle(angle_el);
  
  // #ifdef VQ_LEQ_0
  // if (Uq <0 ){
	//   angle_el+=M_PI;
	//   Uq=fabsf(Uq);
  // }
  // angle_el =  _normalizeAngle (angle_el);
  // #endif

  // int sector = floor(angle_el / M_PI*3) + 1;
  // calculate the duty cycles
  #ifdef MIDDLE_CLAMP
  float sa;
  float ca;
  _sincos(angle_el,&sa,&ca);
  float Ualpha =  ca * Ud - sa * Uq;  
  float Ubeta =  sa * Ud + ca * Uq;
  float Ua = Ualpha;
  float Ub = -0.5f * Ualpha + _SQRT3_2 * Ubeta;
  float Uc = -0.5f * Ualpha - _SQRT3_2 * Ubeta;
  Ua -= Va;
  Ub -= Vb;
  Uc -= Vc;
  float Da = _constrain((Ua / voltage_power_supply+1)/2,0.0f,1.0f);
  float Db = _constrain((Ub / voltage_power_supply+1)/2,0.0f,1.0f);
  float Dc = _constrain((Uc / voltage_power_supply+1)/2,0.0f,1.0f);
  #ifdef SVPWM
  float center = 0.5f;
  // discussed here: https://community.simplefoc.com/t/embedded-world-2023-stm32-cordic-co-processor/3107/165?u=candas1
  // a bit more info here: https://microchipdeveloper.com/mct5001:which-zsm-is-best
  // Midpoint Clamp
  float Dmin = fminf(Da, fminf(Db, Dc));
  float Dmax = fmaxf(Da, fmaxf(Db, Dc));
  center -= (Dmax+Dmin) / 2;
  Da += center;
  Db += center;
  Dc += center;
  #endif
  #else
  angle_el =  _normalizeAngle (angle_el+M_PI/2);
  int sector = floor(angle_el / M_PI*3) + 1;
  // calculate the duty cycles
  float T1 = _SQRT3 * sin(sector * M_PI/3 - angle_el) * Uq / voltage_power_supply;
  float T2 = _SQRT3 * sin(angle_el - (sector - 1.0) * M_PI/3) * Uq / voltage_power_supply;
  float T0 = 1 - T1 - T2;


  float Ta, Tb, Tc;
  switch (sector)
  {
    case 1:
      Ta = T1 + T2 + T0 / 2;
      Tb = T2 + T0 / 2;
      Tc = T0 / 2;
      break;
    case 2:
      Ta = T1 + T0 / 2;
      Tb = T1 + T2 + T0 / 2;
      Tc = T0 / 2;
      break;
    case 3:
      Ta = T0 / 2;
      Tb = T1 + T2 + T0 / 2;
      Tc = T2 + T0 / 2;
      break;
    case 4:
      Ta = T0 / 2;
      Tb = T1 + T0 / 2;
      Tc = T1 + T2 + T0 / 2;
      break;
    case 5:
      Ta = T2 + T0 / 2;
      Tb = T0 / 2;
      Tc = T1 + T2 + T0 / 2;
      break;
    case 6:
      Ta = T1 + T2 + T0 / 2;
      Tb = T0 / 2;
      Tc = T1 + T0 / 2;
      break;
    default:
      Ta = 0;
      Tb = 0;
      Tc = 0;
  }
  // 克拉克逆变换
  float Ua = Ta * voltage_power_supply;
  float Ub = Tb * voltage_power_supply;
  float Uc = Tc * voltage_power_supply;
  #endif
  // Ua = Da * voltage_power_supply;
  // Ub = Db * voltage_power_supply;
  // Uc = Dc * voltage_power_supply;
  setPwm(Da,Db,Dc,TIM_BASE);
}

void setSixStepPhaseVoltage(float Uq, float angle_el, TIM_TypeDef* TIM_BASE)
{
  float center;
  int sector;
  float Ua,Ub,Uc;
  sector = 6 * (_normalizeAngle(angle_el + (M_PI/6) ) / (2*M_PI)); // adding PI/6 to align with other modes
  // centering the voltages around either
  // modulation_centered == true > driver.voltage_limit/2
  // modulation_centered == false > or Adaptable centering, all phases drawn to 0 when Uq=0
  center = voltage_power_supply/2;
  if(trap_120_map[sector][0]  == _HIGH_IMPEDANCE){
    Ua = center;
    Ub = trap_120_map[sector][1] * Uq + center;
    Uc = trap_120_map[sector][2] * Uq + center;
  }else if(trap_120_map[sector][1]  == _HIGH_IMPEDANCE){
    Ua = trap_120_map[sector][0] * Uq + center;
    Ub = center;
    Uc = trap_120_map[sector][2] * Uq + center;
  }else{
    Ua = trap_120_map[sector][0] * Uq + center;
    Ub = trap_120_map[sector][1] * Uq + center;
    Uc = center;
  }

  setPwm(Ua,Ub,Uc,TIM_BASE);
}

float cal_angular_vel(float angle_now,float* speed_rad)
{
  float return_value = 0.0f;
  static float angle_prev = -1.0f;
  if (angle_prev < 0){
    	angle_prev=angle_now;
    	return 0;
    }
    float delta_angle=angle_now - angle_prev;
    if (delta_angle >= 1.6*M_PI){
    	delta_angle-=2*M_PI;
      return_value = -1.0f;
    }
    if (delta_angle <= -1.6*M_PI){
      delta_angle+=2*M_PI;
      return_value = 1.0f;
    }
    angle_prev=angle_now;
    *speed_rad = delta_angle / Ts;
    return return_value;


}
void cal_Idq(float* current_phase, float angle_el, float* Id, float* Iq)
{
	angle_el = _normalizeAngle(angle_el);
  float mid_current = (current_phase[0] + current_phase[1] + current_phase[2]) / 3.0f;
  // 将三相电流转换为两相电流
  float a = current_phase[0] - mid_current;
  float b = current_phase[1] - mid_current;
	float I_alpha=a;
  float I_beta=_1_SQRT3*(2*b+a);
//	 float Iq=-sin(angle_el)*I_alpha+cos(angle_el)*I_beta;
//	 float Id=cos(angle_el)*I_alpha+sin(angle_el)*I_beta;
	*Iq = -sin(angle_el)*I_alpha+cos(angle_el)*I_beta;
  *Id = cos(angle_el)*I_alpha+sin(angle_el)*I_beta;
}

void get_target_Idq(float Is, float speed_RPM, float* Id, float* Iq)
{
  float omega_e = (float)speed_RPM * 4 * 2.0f * M_PI / 60.0f;
  float discriminant = 8.08e-10f - (0.000000023333333333333339336063836389849*voltage_power_supply*voltage_power_supply)/omega_e/omega_e;
  float Id_optimal = 171.5 - sqrt((29412.25f + 0.5*Is*Is));
  discriminant = -1;

  if(discriminant > 0)
  {
    float sqrt_discriminant = sqrtf(discriminant);
    Id_optimal = 293.99999999999986840738140870695 - 14285714.2857142820391445899654*sqrt_discriminant;
    Id_optimal = _constrain(Id_optimal, -max_current, 0.0f);
  }

  float Iq_optimal = _sign(Is)*sqrt(Is*Is - Id_optimal*Id_optimal);
  *Id = _constrain(Id_optimal,-max_current,0.0f);
  *Iq = Iq_optimal;
}