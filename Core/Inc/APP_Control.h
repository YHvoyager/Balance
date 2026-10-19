#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

/* 电机编号，配合 forward / backward / stop / SetDuty / SetCompare 的 ID 参数使用 */
#define MOTOR_A 1
#define MOTOR_B 2

void motor_Init(void);
/* Compare 用 uint16_t：F1 的 TIM2 是 16 位定时器，ARR 最大 65535，
   uint8_t 只能到 255，ARR 一大就永远给不满占空比 */
void SetCompare(int ID,uint16_t Compare);
void SetDuty(int ID,float Duty);
void forward(int ID,float Duty);
void backward(int ID,float Duty);
void stop(int ID);
int16_t GetEncoderCount(int ID);
/* 轮速（差分式，不清零累计值）：dt 为采样周期(秒)，返回 转/秒，正转正反转负 */
float GetWheelSpeed(int ID, float dt);

#endif // CONTROL_H