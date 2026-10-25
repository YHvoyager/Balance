#ifndef __APP_PID_H__
#define __APP_PID_H__

#include <stdint.h>

typedef struct {
    float kp;  //比例系数
    float ki;  //积分系数
    float kd;  //微分系数
    float sp;  //设定值

    uint64_t t_k_l;  //t[k - 1]上一次时间戳
    float err_k_l;  //err[k - 1]上一次PID运行时误差值
    float err_int_k_l;  //err_int[k - 1]上一次PID运行时的误差积分值

    float UpperLimit; //上限
    float LowerLimit; //下限
} PID_Typedef;


void PID_Init(PID_Typedef *pid,float kp,float ki,float kd);
void PID_ChangeSp(PID_Typedef *pid,float sp);
float PID_Compute(PID_Typedef *pid,float FB);
void PID_LimitConfig(PID_Typedef *pid,float UpperLimit,float LowerLimit);
void PID_Reset(PID_Typedef *pid);

#endif