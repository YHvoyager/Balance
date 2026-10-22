#include "Encoder_test.h"

//注册编码器
Encoder_t Encoder_L;
Encoder_t Encoder_R;

Motor_t Motor_L;
Motor_t Motor_R;

void Encoder_Test(void){

    
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