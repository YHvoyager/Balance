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

    pid->UpperLimit = +8.4f;
    pid->LowerLimit = -8.4f;
}

//
// @简介：设置PID控制器的上限和下限
// @参数： UpperLimit 上限
// @参数： LowerLimit 下限
void PID_LimitConfig(PID_Typedef *pid,float UpperLimit,float LowerLimit){
    pid->UpperLimit = UpperLimit;
    pid->LowerLimit = LowerLimit;
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

    float err_dev = 0.0f;
    float err_int = 0.0f;

    if(pid->t_k_l != 0){
    err_dev = (err - pid->err_k_l) / deltaT;
    err_int = pid->err_int_k_l + (err + pid->err_k_l) * deltaT * 0.5f;}

    //积分限幅
    if(err_int > pid->UpperLimit) err_int = pid->UpperLimit;
    if(err_int < pid->LowerLimit) err_int = pid->LowerLimit;

    
    float CO_p = pid->kp * err;
    float CO_i = pid->ki * err_int;
    float CO_d = pid->kd * err_dev;
    float CO = CO_p + CO_i + CO_d;  

    //更新
    pid->t_k_l = t_k;
    pid->err_k_l = err;
    pid->err_int_k_l = err_int;


    //限制PID值在上限和下限之间
    if(CO >= pid->UpperLimit){
        CO = pid->UpperLimit;
    }
    if(CO <= pid->LowerLimit){
        CO = pid->LowerLimit;
    }


    return CO;
}

//
// @简介：对PID控制器进行复位，防止再次开机时积分项累加
// @参数： pid PID控制器指针
void PID_Reset(PID_Typedef *pid){
    pid->err_int_k_l = 0.0f;
    pid->err_k_l = 0.0f;
    pid->t_k_l = 0;
}