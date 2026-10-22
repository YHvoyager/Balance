#ifndef __APP_ENCODER_H__
#define __APP_ENCODER_H__

#include <stdint.h>
#include "stm32f1xx_hal.h"

//
// @简介: 编码器
//
typedef struct {
    volatile uint32_t last_capture;
    volatile int32_t period_ticks;
    //port和pin
    GPIO_TypeDef *Port;
    uint16_t Pin;
} Encoder_t;

//
// @简介: 电机，包含电机ID、编码器、应用方向、速度等信息
//
typedef struct {
    volatile char ID;
    Encoder_t *encoder;
    int8_t Application;
    float Speed;
} Motor_t;

void Encoder_Init(Encoder_t *encoder, GPIO_TypeDef *port, uint16_t pin);
void Encoder_EXTI_IRQHandler(Motor_t *motor);

#endif /* __APP_ENCODER_H__ */