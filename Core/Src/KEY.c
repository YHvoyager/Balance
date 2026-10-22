#include "KEY.h"

KEY_HandleTypeDef key1;
KEY_HandleTypeDef key2;
KEY_HandleTypeDef key3;

void KEY_Init(KEY_HandleTypeDef* key,
    GPIO_TypeDef* GPIO_Port,
    uint16_t pin,
    GPIO_PinState active_state)
    {
        key->port = GPIO_Port;
        key->pin = pin;
        key->active_state = active_state;
        key->irq_flag = 0;
        key->irq_tick = 0;
        key->pressed = 0;
        key->last_pressed = 0;
    }

void KEY_EXTI_Callback(KEY_HandleTypeDef* key)
    {
        key->irq_tick = HAL_GetTick();
        key->irq_flag = 1;
    }

void KEY_Update(KEY_HandleTypeDef* key)
    {
        GPIO_PinState state;

        if(key->irq_flag == 0){
            return;
        }

        //消抖，等待20ms
        if(HAL_GetTick() - key->irq_tick < 20){
            return;
        }

        state = HAL_GPIO_ReadPin(key->port, key->pin);
        key->pressed = (state == key->active_state);

        key->irq_flag = 0;
    }

uint8_t KEY_WasPressed(KEY_HandleTypeDef* key)
    {
        uint8_t result;

        result = key->pressed && !key->last_pressed;
        key->last_pressed = key->pressed;

        return result;
    }

uint8_t KEY_IsPressed(KEY_HandleTypeDef* key)
    {
        return key->pressed;
    }

    void KEY_AllInit(void){
    KEY_Init(&key1, KEY1_GPIO_Port, KEY1_Pin, GPIO_PIN_RESET);
    KEY_Init(&key2, KEY2_GPIO_Port, KEY2_Pin, GPIO_PIN_RESET);
    KEY_Init(&key3, KEY3_GPIO_Port, KEY3_Pin, GPIO_PIN_RESET);
    }