#include "APP_Encoder.h"

//减速比为1:20.409,轮子直径48mm，线数为13
static const float Wheel_Diameter = 48.0f;
static const float Encoder_Line = 13.0f;
static const float Encoder_Ratio = 20.409f;

void Encoder_Init(Encoder_t *encoder, GPIO_TypeDef *Port, uint16_t Pin){
    encoder->last_capture = 0;
    encoder->period_ticks = 0;
    encoder->Port = Port;
    encoder->Pin = Pin;
}

//
// @简介: 编码器中断处理函数，用于更新电机应用方向和速度
//
void Encoder_EXTI_IRQHandler(Motor_t *motor){
    if(motor -> ID == 'L'){
        if(motor->encoder->last_capture == 0){
            motor->encoder->last_capture = HAL_GetTick();
        }
        //读取B相电平，PB7
        if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET){
            motor->encoder->period_ticks--;
            motor->Application = 1;
            motor->encoder->last_capture = HAL_GetTick();
        }
        else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET){
            motor->encoder->period_ticks++;
            motor->Application = -1;
            motor->encoder->last_capture = HAL_GetTick();
        }
    }
    if(motor -> ID == 'R'){
        if(motor->encoder->last_capture == 0){
            motor->encoder->last_capture = HAL_GetTick();
        }
        //读取A相电平，PA6
        if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET){
            motor->encoder->period_ticks--;
            motor->Application = 1;
            motor->encoder->last_capture = HAL_GetTick();
        }
        else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET){
            motor->encoder->period_ticks++;
            motor->Application = -1;
            motor->encoder->last_capture = HAL_GetTick();
        }
    }
}