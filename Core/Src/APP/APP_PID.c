#include "APP_PID.h"
#include "main.h"
#include "APP_Encoder.h"

//
// @简介：初始化PID控制器
// @参数： kp 比例系数
// @参数： ki 积分系数
// @参数： kd 微分系数
void PID_Init(PID_Typedef *pid,float kp,float ki,float kd){
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->sp = 0.0f;

    pid->t_k_l = 0;
    pid->err_k_l = 0.0f;
    pid->err_int_k_l = 0.0f;
}

//
// @简介：改变PID值的设定值
// @参数： sp 设定值
void PID_ChangeSp(PID_Typedef *pid,float sp){
    pid->sp = sp;
}

//
// @简介：执行一次PID计算
// @参数： fb 反馈值
// @返回： PID值
float PID_Compute(PID_Typedef *pid,float fb){
    float err = pid->sp - fb;

    uint64_t t_k = micros();
    float deltaT = (float)(t_k - pid->t_k_l) * 1.0e-6f;
    float err_dev = (err - pid->err_k_l) / deltaT;
    float err_int = pid->err_int_k_l + (err + pid->err_k_l) * deltaT * 0.5f;
    
    float CO_p = pid->kp * err;
    float CO_i = pid->ki * err_int;
    float CO_d = pid->kd * err_dev;
    float CO = CO_p + CO_i + CO_d;  

    //更新
    pid->t_k_l = t_k;
    pid->err_k_l = err;
    pid->err_int_k_l = err_int;

    return CO;
}