#include "KEY.h"
#include "task.h"
#include "APP_Motor_Control.h"

KEY_HandleTypeDef key1;
KEY_HandleTypeDef key2;
KEY_HandleTypeDef key3;

static uint8_t Motor_status = 0;  //电机状态 0:关闭,1:开启

void KEY_Init(KEY_HandleTypeDef* key,
    GPIO_TypeDef* GPIO_Port,
    uint16_t pin,
    GPIO_PinState active_state)
    {
        key->port = GPIO_Port;
        key->pin = pin;
        key->active_state = 0;  //按键按下时的电平状态
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

void KEY_Proc(KEY_HandleTypeDef* key1, KEY_HandleTypeDef* key2, KEY_HandleTypeDef* key3){
        PERIODIC(10)

        for(int i = 0; i < 3; i++){
        //判断标志位
        if(key1[i].irq_flag == 0){
            return;
        }

        //消抖，等待20ms
        if(HAL_GetTick() - key1[i].irq_tick < 20){
            return;
        }

        if(HAL_GPIO_ReadPin(key1[i].port, key1[i].pin) == key1[i].active_state){
            key1[i].pressed = 1;
        }
        else{
            key1[i].pressed = 0;
        }

        if(key1[i].pressed == 1){
            //按键按下
            if(i == 0){
            //按键1按下
            Motor_status = !Motor_status;
            APP_Motor_Cmd(Motor_status);
            }
            key1[i].pressed = 0;
        }

        //重置标志位
        key1[i].irq_flag = 0;
        }
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