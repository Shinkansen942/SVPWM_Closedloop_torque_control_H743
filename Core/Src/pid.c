/*
 * pid.c
 *
 *  Created on: Jun 18, 2023
 *      Author: hht
 */

#include "pid.h"
#include "motor_control.h"
extern float Ts;
float PID_operator(float error, struct PIDController* pid){
    // P环

    float proportional = pid->P * error;
    // Tustin 散点积分（I环）
    float integral = pid->integral_prev + pid->I*Ts*0.5f*(error + pid->error_prev);
    // integral = _constrain(integral, -pid->limit, pid->limit);
    // D环（微分环节）
    float derivative = pid->D*(error - pid->error_prev)/Ts;

	float output;
    float anit_windup;

    #ifndef ANTI_WINDUP
    integral = _constrain(integral, -pid->limit/1, pid->limit/1);
    output = proportional + integral + derivative;
    #endif
    #ifdef ANTI_WINDUP
    //anti_windup & back_calculation
    output = proportional + integral + derivative;
    anit_windup =  _constrain(output, -pid->limit, pid->limit) - output;

    integral += (1.0f/pid->P)*anit_windup;
    integral = _constrain(integral, -pid->limit/1, pid->limit/1);

    // 将P,I,D三环的计算值加起来
    output = proportional + integral + derivative;
    // anit_windup = output_constrained - output;

    #endif
    float output_constrained = _constrain(output, -pid->limit, pid->limit);

    output = output_constrained;

    if(pid->output_ramp > 0){
        // 对PID的变化速率进行限制
        float output_rate = (output - pid->output_prev)/Ts;
        if (output_rate > pid->output_ramp)
            output = pid->output_prev + pid->output_ramp*Ts;
        else if (output_rate < -pid->output_ramp)
            output = pid->output_prev - pid->output_ramp*Ts;
    }
    // 保存值（为了下一次循环）
    if(output == output)
    {
        if (output == pid->limit || output == -pid->limit)
        {
            pid->integral_prev = pid->integral_prev;;
        }
        else
        {
            pid->integral_prev = integral;
        }    
        pid->output_prev = output;
    }
    pid->error_prev = error;
//    pid->timestamp_prev = timestamp_now;
    return output;
}

void PID_reset(pidc_t* pid)
{
    pid->error_prev = 0;
    pid->output_prev = 0;
    pid->integral_prev = 0;
}

void PID_integral_reset(pidc_t* pid)
{
    pid->integral_prev = 0;
}