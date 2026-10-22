#include "APP_Encoder.h"

//减速比为1:20.409,轮子直径48mm，线数为13
static const float Wheel_Diameter = 48.0f;
static const float Encoder_Line = 13.0f;
static const float Encoder_Ratio = 20.409f;

//Application，从正转变成反转时 =-2，从反转转变成正时 =2

//注册编码器
Encoder_t Encoder_L;
Encoder_t Encoder_R;

//注册电机
Motor_t Motor_L;
Motor_t Motor_R;

void Encoder_Init(Encoder_t *encoder, GPIO_TypeDef *Port, uint16_t Pin){
    encoder->t0 = 0;
    encoder->t1 = 0;
    encoder->period_ticks = 0;
    encoder->Port = Port;
    encoder->Pin = Pin;
}

//
// @简介: 编码器中断处理函数，用于更新电机应用方向和速度
//
void Encoder_EXTI_IRQHandler(Motor_t *motor){
    if(motor -> ID == 'L'){
        if(motor->encoder->t1 == 0){
            motor->encoder->t1 = micros();
        }
        //读取B相电平，PB7
        if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET){  //当前轮胎为反转
            motor->encoder->period_ticks--;
            if(motor->Application == 1){  //之前为正转，现在为反转，Application=-2
                motor->Application = -2;
            }else{
            motor->Application = -1;
            }
            motor->encoder->t1 = motor->encoder->t0;
            motor->encoder->t0 = micros();
        }
        else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET){  //当前轮胎为正转
            motor->encoder->period_ticks++;
            if(motor->Application == -1){  //之前为反转，现在为正转，Application=2
                motor->Application = 2;
            }else{
            motor->Application = 1;
            }
            motor->encoder->t1 = motor->encoder->t0;
            motor->encoder->t0 = micros();
        }
    }
    if(motor -> ID == 'R'){
        if(motor->encoder->t1 == 0){
            motor->encoder->t1 = micros();
        }
        //读取A相电平，PA6
        if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET){  //当前轮胎为反转
            motor->encoder->period_ticks--;
            if(motor->Application == 1){  //之前为正转，现在为反转，Application=-2
                motor->Application = -2;
            }else{
            motor->Application = -1;
            }
            motor->encoder->t1 = motor->encoder->t0;
            motor->encoder->t0 = micros();
        }
        else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET){  //当前轮胎为正转
            motor->encoder->period_ticks++;
            if(motor->Application == -1){  //之前为反转，现在为正转，Application=2
                motor->Application = 2;
            }else{
            motor->Application = 1;
            }
            motor->encoder->t1 = motor->encoder->t0;
            motor->encoder->t0 = micros();
        }
    }
}



void Encoder_Count_Init(void){

    
    static uint8_t initialized = 0;
    if (!initialized) {
        // 当前板级配置中，PA7 和 PB6 是已启用的 EXTI 输入。
        Encoder_Init(&Encoder_L, GPIOA, GPIO_PIN_7);
        Encoder_Init(&Encoder_R, GPIOB, GPIO_PIN_6);

        Motor_R = (Motor_t){'R', &Encoder_R, 0, 0.0f};
        Motor_L = (Motor_t){'L', &Encoder_L, 0, 0.0f};
        initialized = 1;
    }

}

float EncoderGetSpeed_L(void){
    float T;
    uint64_t now = micros();
    if(Motor_L.Application == -2 || Motor_L.Application == 2){
        return 0.0f;
    }
    if((Encoder_L.t0 - Encoder_L.t1) < (now - Encoder_L.t0)){
        T = (now - Encoder_L.t0) * 1.0e-6f;
    }
    if((Encoder_L.t0 - Encoder_L.t1) > (now - Encoder_L.t0)){
        T = (Encoder_L.t0 - Encoder_L.t1) * 1.0e-6f;
    }

    return Motor_L.Application / T / Encoder_Line / Encoder_Ratio * 360.0f;
}

float EncoderGetSpeed_R(void){
    float T;
    uint64_t now = micros();
    if(Motor_R.Application == -2 || Motor_R.Application == 2){
        return 0.0f;
    }
    if((Encoder_R.t0 - Encoder_R.t1) < (now - Encoder_R.t0)){
        T = (now - Encoder_R.t0) * 1.0e-6f;
    }
    if((Encoder_R.t0 - Encoder_R.t1) > (now - Encoder_R.t0)){
        T = (Encoder_R.t0 - Encoder_R.t1) * 1.0e-6f;
    }

    return Motor_R.Application / T / Encoder_Line / Encoder_Ratio * 360.0f;
}

void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t micros(void)
{
    return DWT->CYCCNT /
           (SystemCoreClock / 1000000U);
}