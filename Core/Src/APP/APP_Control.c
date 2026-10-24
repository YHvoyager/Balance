#include "main.h"
#include "stm32f1xx_hal_tim.h"
#include "tim.h"
#include <stdint.h>
#include "APP_Control.h"
#include "KEY.h"
#include "APP_PID.h"
#include "APP_Encoder.h"
#include "task.h"
#include "APP_Bat.h"

// TIM2：向上计数、PWM mode 1、高电平有效
// 默认不重映射：
// 左轮 IN1 = PA0 / TIM2_CH1
// 左轮 IN2 = PA1 / TIM2_CH2
// 右轮 IN1 = PA2 / TIM2_CH3
// 右轮 IN2 = PA3 / TIM2_CH4
//
// TIM2 时钟为 72MHz 时：
// PSC = 0，ARR = 3599，PWM = 20kHz

//
// @简介:初始化电机
//
static PID_Typedef PID_L;  //左电机调速系统PID控制器
static PID_Typedef PID_R;  //右电机调速系统PID控制器
void APP_Motor_Init(void)
{
    
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

//
// @简介:限幅
//
static uint32_t DutyToCompare(float duty)
{
    // 同时将负数和 NaN 处理为 0
    if (!(duty > 0.0f)) {
        return 0;
    }

    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim2) + 1U;

    if (duty >= 100.0f) {
        return period;
    }

    return (uint32_t)(period * duty / 100.0f + 0.5f);
}


//
// @简介:设置左轮电机PWM占空比 0~100
// @参数:duty 占空比 0~100
// PWM频率20kHZ，自动重装载值为3600
void ForwardDuty_L(float duty){
    if(duty > 100){
        duty = 100;
    }
    if(duty < 0){
        duty = 0;
    }   
    //设置PWM占空比,开启PA1 PWM输出，拉高PA0，占空比取100-duty
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, DutyToCompare(100.0f - duty));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, __HAL_TIM_GET_AUTORELOAD(&htim2) + 1U);
}

void ForwardDuty_R(float duty){
    if(duty > 100){
        duty = 100;
    }
    if(duty < 0){
        duty = 0;
    }
    //设置PWM占空比,开启PA3 PWM输出，拉高PA2，占空比取100-duty
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, DutyToCompare(100.0f - duty));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, __HAL_TIM_GET_AUTORELOAD(&htim2) + 1U);
}

void BackDuty_L(float duty){
    if(duty > 100){
        duty = 100;
    }
    if(duty < 0){
        duty = 0;
    }
    //设置PWM占空比，开启PA1 PWM输出，拉高PA0，占空比取100-duty
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, DutyToCompare(100.0f - duty));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, __HAL_TIM_GET_AUTORELOAD(&htim2) + 1U);
}

void BackDuty_R(float duty){
    if(duty > 100){
        duty = 100;
    }
    if(duty < 0){
        duty = 0;
    }
    //设置PWM占空比,开启PA3 PWM输出，拉高PA2，占空比取100-duty
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, DutyToCompare(100.0f - duty));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, __HAL_TIM_GET_AUTORELOAD(&htim2) + 1U);
}

//
// @简介:刹车
//
void Brake(void)
{
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim2) + 1U;

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, period);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, period);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, period);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, period);
}
// @简介:滑行
//
void Slide(void)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
}
// @简介:正转
//
void Forward(uint16_t L_duty, uint16_t R_duty){
    ForwardDuty_L(L_duty);
    ForwardDuty_R(R_duty);
}
// @简介:反转
//
void Back(uint16_t L_duty, uint16_t R_duty){
    BackDuty_L(L_duty);
    BackDuty_R(R_duty);
}

float Get_Duty(char Wheel)
{
    uint32_t compare;

    if (Wheel == 'L')
    {
        compare = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1);
    }
    else if (Wheel == 'R')
    {
        compare = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_4);
    }
    else
    {
        return 0.0f;
    }

    float pwm_duty =
        compare / (float)(__HAL_TIM_GET_AUTORELOAD(&htim2) + 1U);

    // 由于硬件使用 100-duty，显示时反算
    return (1.0f - pwm_duty) * 100.0f;
}

//
// @简介:电机测试
// 参数:KEY_HandleTypeDef *key2, KEY_HandleTypeDef *key3
void Motor_Test(KEY_HandleTypeDef *key2, KEY_HandleTypeDef *key3)
{
    KEY_Update(key2);
    KEY_Update(key3);
    if(KEY_WasPressed(key2)){
      //按一次右轮电机占空比加10%
      ForwardDuty_R(Get_Duty('R') - 10.0f);
      //清零last_pressed,清零pressed
      key2->last_pressed = 0;
      key2->pressed = 0;
    }
    if(KEY_WasPressed(key3)){
      //按一次左轮电机占空比加10%
      ForwardDuty_L(Get_Duty('L') - 10.0f);
      //清零last_pressed,清零pressed
      key3->last_pressed = 0;
      key3->pressed = 0;
    }
}

//
// @简介:电机调速系统进程函数
//
void APP_Motor_Proc(void){
    PERIODIC(1); //1ms执行一次

    // #1
    //通过编码器获取左轮胎角速度
    float omega_l = EncoderGetSpeed_L();
    //通过编码器获取右轮胎角速度
    float omega_r = EncoderGetSpeed_R();

    // #2 计算PID控制器的输出
    float Ua_l = PID_Compute(&PID_L, omega_l);
    float Ua_r = PID_Compute(&PID_R, omega_r);

    // #3 将电压Ua设置到电机两端
    //获取电池电压
    float vbat = APP_Bat_Get();

    //计算左右电机PWM信号的占空比
    float duty_l = Ua_l / vbat * 100.0f;
    float duty_r = Ua_r / vbat * 100.0f;

    //设置左右占空比
    ForwardDuty_L(duty_l);
    ForwardDuty_R(duty_r);
}


//
// @简介:设置左轮胎角速度
// @参数: omega_l 左轮胎角速度,单位:rad/s
void APP_Motor_SetOmega_L(float omega_l){
    PID_ChangeSp(&PID_L, omega_l);
} 

//
// @简介:设置右轮胎角速度
// @参数: omega_r 右轮胎角速度,单位:rad/s

void APP_Motor_SetOmega_R(float omega_r){
    PID_ChangeSp(&PID_R, omega_r);
}

void APP_PWM_Init(void)
{
    //初始化PID控制器
    PID_Init(&PID_L, 0.5f, 7.0f, 0.0f);
    PID_Init(&PID_R, 0.5f, 7.0f, 0.0f);
}
