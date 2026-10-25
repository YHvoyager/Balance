#include "APP_PWM.h"
#include "APP_PID.h"
#include "main.h"
#include "tim.h"

//
// @简介:初始化PWM
//
void APP_PWM_Init(void)
{
}

void APP_PWM_Cmd(uint8_t on){
    if(on){
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    }
    else{
        //关闭PWM输出
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
    }
}

float APP_PWM_Get_Duty(char Wheel)
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