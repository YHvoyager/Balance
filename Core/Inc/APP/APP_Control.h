#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 电机初始化，在 MX_TIM2_Init() 后调用
void Motor_Init(void);

// 单轮正转，占空比 0～100%
void ForwardDuty_L(float duty);
void ForwardDuty_R(float duty);

// 单轮反转，占空比 0～100%
void BackDuty_L(float duty);
void BackDuty_R(float duty);

// 两轮制动
void Brake(void);

// 两轮滑行
void Slide(void);

// 两轮正转，占空比 0～100%
void Forward(uint16_t L_duty, uint16_t R_duty);

// 两轮反转，占空比 0～100%
void Back(uint16_t L_duty, uint16_t R_duty);

// 获取轮子占空比，0～100%
float Get_Duty(char Wheel);
#ifdef __cplusplus
}
#endif

#endif /* APP_CONTROL_H */