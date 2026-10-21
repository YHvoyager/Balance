#include "main.h"
#include "stm32f1xx_hal_tim.h"
#include "tim.h"
#include <stdint.h>
#include "APP_Control.h"

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
void Motor_Init(void)
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
