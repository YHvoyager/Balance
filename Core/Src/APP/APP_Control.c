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
#include "APP_PWM.h"

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
static uint8_t motor_enabled = 0;
static volatile float motor_duty_l = 0.0f;
static volatile float motor_duty_r = 0.0f;

void APP_PID_SetLeft(float kp, float ki, float kd){
    PID_L.kp = kp;
    PID_L.ki = ki;
    PID_L.kd = kd;
    PID_Reset(&PID_L);
}
void APP_PID_SetRight(float kp, float ki, float kd){
    PID_R.kp = kp;
    PID_R.ki = ki;
    PID_R.kd = kd;
    PID_Reset(&PID_R);
}


//
// @简介:初始化电机
//
void APP_Motor_Init(void)
{
    //初始化PID控制器
    PID_Init(&PID_L, 0.5f, 0.0f, 0.0f);
    PID_Init(&PID_R, 0.5f, 0.0f, 0.0f);
    PID_LimitConfig(&PID_L, +8.4f, -8.4f);
    PID_LimitConfig(&PID_R, +8.4f, -8.4f);

    motor_enabled = 0;
    motor_duty_l = 0.0f;
    motor_duty_r = 0.0f;
    APP_PWM_Cmd(0);
    Brake();
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
void ForwardDuty_R(float duty){
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

void ForwardDuty_L(float duty){
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

void BackDuty_R(float duty){
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

void BackDuty_L(float duty){
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

//
// @简介:电机测试
// 参数:KEY_HandleTypeDef *key2, KEY_HandleTypeDef *key3
// void Motor_Test(KEY_HandleTypeDef *key1, KEY_HandleTypeDef *key2, KEY_HandleTypeDef *key3)
// {
//     KEY_Update(key1);
//     KEY_Update(key2);
//     KEY_Update(key3);
    
//     if(KEY_WasPressed(key1)){
//         //按一次就开或者关电机
//         //取反电机状态
//         Motor_status = !Motor_status;
//         APP_Motor_Cmd(Motor_status);
//     }
//     if(KEY_WasPressed(key2)){
//       //按一次右轮电机占空比加10%
//       ForwardDuty_R(Get_Duty('R') - 10.0f);
//       //清零last_pressed,清零pressed
//       key2->last_pressed = 0;
//       key2->pressed = 0;
//     }
//     if(KEY_WasPressed(key3)){
//       //按一次左轮电机占空比加10%
//       ForwardDuty_L(Get_Duty('L') - 10.0f);
//       //清零last_pressed,清零pressed
//       key3->last_pressed = 0;
//       key3->pressed = 0;
//     }
// }

//
// @简介:电机调速系统进程函数
//
void APP_Motor_Proc(void){
    PERIODIC(1); //1ms执行一次

    if(!motor_enabled){
        motor_duty_l = 0.0f;
        motor_duty_r = 0.0f;
        return;
    }
    // #1

    // #2 计算PID控制器的输出
    float Ua_l = PID_Compute(&PID_L, EncoderGetSpeed_L());
    float Ua_r = PID_Compute(&PID_R, EncoderGetSpeed_R());

    // #3 将电压Ua设置到电机两端
    //获取电池电压
    float vbat = APP_Bat_Get();

    //计算左右电机PWM信号的占空比
    float duty_l = Ua_l / vbat * 100.0f;
    float duty_r = Ua_r / vbat * 100.0f;

    // 保存带方向符号的计算占空比，供 VOFA 读取；负值表示反转。
    motor_duty_l = duty_l;
    motor_duty_r = duty_r;

    //设置左右占空比
    if(duty_l > 0.0f){
        ForwardDuty_L(duty_l);
    }
    else{
        BackDuty_L(-duty_l);
    }
    if(duty_r > 0.0f){
        ForwardDuty_R(duty_r);
    }
    else{
        BackDuty_R(-duty_r);
    }
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

float APP_Motor_GetDuty_L(void){
    return motor_duty_l;
}

float APP_Motor_GetDuty_R(void){
    return motor_duty_r;
}

//
// @简介:开关电机
// @参数: on 非零:开启,0:关闭
uint8_t APP_Motor_Cmd(uint8_t on){
    motor_enabled = (on != 0U);
    if (!motor_enabled) {
        motor_duty_l = 0.0f;
        motor_duty_r = 0.0f;
        Brake();
    }
    APP_PWM_Cmd(motor_enabled);
    PID_Reset(&PID_L);
    PID_Reset(&PID_R);
    return motor_enabled;
}
