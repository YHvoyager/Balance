#ifndef __KEY_H
#define __KEY_H

#include "main.h"

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    GPIO_PinState active_state;

    volatile uint8_t irq_flag;
    uint32_t irq_tick;

    uint8_t pressed;
    uint8_t last_pressed;
} KEY_HandleTypeDef;

void KEY_Init(KEY_HandleTypeDef *key,
              GPIO_TypeDef *port,
              uint16_t pin,
              GPIO_PinState active_state);

void KEY_EXTI_Callback(KEY_HandleTypeDef *key);
void KEY_Update(KEY_HandleTypeDef *key);
uint8_t KEY_WasPressed(KEY_HandleTypeDef *key);
uint8_t KEY_IsPressed(KEY_HandleTypeDef *key);

#endif