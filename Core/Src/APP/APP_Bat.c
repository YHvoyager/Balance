#include "APP_Bat.h"
#include "adc.h"
#include "task.h"

#define ADC_VREF        3.3f
#define ADC_MAX         4095.0f
#define DIVIDER_RATIO   2.733f
// 一点校准：当前显示 7.68 V、万用表实测 7.57 V，修正系数为
// 1.0237 * (7.57 / 7.68) = 1.009038。
#define ADC_CALIBRATION 1.00904f

static float battery_voltage = 7.4f;

void APP_ADC_Init(void)
{
    // ADC 初始化完成后调用一次
    HAL_ADCEx_Calibration_Start(&hadc1);
}

void APP_Bat_Get_Proc(void)
{
    // 每 10 ms 更新一次
    PERIODIC(10);

    uint32_t sum = 0;

    for (uint8_t i = 0; i < 8; i++)
    {
        if (HAL_ADC_Start(&hadc1) != HAL_OK)
            return;

        if (HAL_ADC_PollForConversion(&hadc1, 2) != HAL_OK)
        {
            HAL_ADC_Stop(&hadc1);
            return;
        }

        sum += HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
    }

    float adc_voltage =
        (sum / 8.0f) * ADC_VREF / ADC_MAX;

    battery_voltage =
        adc_voltage * DIVIDER_RATIO * ADC_CALIBRATION;
}

float APP_Bat_Get(void)
{
    return battery_voltage;
}
