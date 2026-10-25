#include "APP_Control.h"
#include "APP_PID.h"
#include "task.h"
#include "APP_MPU6050.h"
#include "qmath.h"
#include "APP_Encoder.h"
#include "APP_Motor_Control.h"

static PID_Typedef pid_theta; //Θ环的PID控制器
static PID_Typedef pid_theta_dot; //Θ'环的PID控制器

static const float lp = 0.094f; //平衡车轮胎中心到MPU6050中心的距离,单位：m
static const float g = 9.8f; //重力加速度,单位：m/s^2
static const float rw = 0.024f; //平衡车轮胎半径,单位：m

//
// @简介：初始化平衡车控制系统
// 
void APP_Control_Init(void){
    PID_Init(&pid_theta, 1.0f, 0.0f, 0.0f);
    PID_LimitConfig(&pid_theta, 12.57f, -12.57f); // Θ环的PID控制器输出范围为[-12.57, 12.57]

    PID_Init(&pid_theta_dot, 2.0f, 10.0f, 0.0f);
    PID_LimitConfig(&pid_theta_dot, 125.7f, -125.7f); // Θ'环的PID控制器输出范围为[-125.7, 125.7]
}

static float omega_ref = 0.0f; //平衡车轮胎转速,单位：rad/s
static uint64_t last_time = 0; //上一次执行时间,单位：微秒  

//
// @简介：平衡车控制系统进程函数，用于运算，需要在while中调用
//
void APP_Control_Proc(void){
    PERIODIC(5);  //控制程序5ms执行一次

    uint64_t now = micros();
    float deltaT = (now - last_time) * 1e-6f; //单位：秒

    // #1.将外环的设定值SP设置为0
    PID_ChangeSp(&pid_theta, 0);

    // #2.读取传感器数据
    float theta = APP_MPU6050_GetPitch() * 0.0174533f; //单位：弧度
    float theta_dot = (APP_MPU6050_GetGy() - APP_MPU6050_GetGyBias()) * 0.0174533f; //单位：弧度/秒

    // #3.计算外环PID输出
    float theta_dot_ref = PID_Compute(&pid_theta, theta);

    // #4.改变内环的设定值SP
    PID_ChangeSp(&pid_theta_dot, theta_dot_ref);

    // #5.计算内环PID输出
    float theta_dot_dot_ref = PID_Compute(&pid_theta_dot, theta_dot);

    // #6.倒立摆的逆解算
    float x_dot_dot_ref = (g * qsin(theta) - theta_dot_dot_ref * lp) / qcos(theta); //单位：m/s^2

    // #7.计算轮胎转速
    omega_ref += 1.0f / rw * x_dot_dot_ref * deltaT; //单位：rad/s

    // #8.设置电机转速
    APP_Motor_SetOmega_L(omega_ref);
    APP_Motor_SetOmega_R(omega_ref);

    last_time = now;
}