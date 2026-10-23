/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "APP_Control.h"
#include "APP_MPU6050.h"
#include "oled.h"
#include <string.h>
#include <stdio.h>
#include <KEY.h>
#include "APP_Encoder.h"
#include "printf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
void FloatToStr(char *out, float v)
{
  if (v < 0) { *out++ = '-'; v = -v; }   // 处理负号
  int whole = (int)v;                     // 整数部分
  int frac = (int)((v - whole) * 100.0f + 0.5f); // 四舍五入保留2位小数
  if (frac >= 100) { whole++; frac -= 100; }     // 进位
  sprintf(out, "%d.%02d", whole, frac);   // %d 不依赖浮点printf
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  Timer1_StartUs();
  /* USER CODE BEGIN 2 */
  Motor_Init();
  MPU6050_Init();
  
  HAL_Delay(100); //等待MPU6050稳定
  OLED_Init();

  char display_str[20];

  KEY_AllInit();

  Encoder_Count_Init();


  // Forward(10, 10);
  // HAL_Delay(50);
  // Forward(50, 50);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Motor_Test(&key2, &key3);
    MPU6050_Proc();
    // OLED_NewFrame();
  // char display_str[128];
  // // 显示占空比
  // sprintf(display_str,
  //       "L:%d%% R:%d%%",
  //       (int)(Get_Duty('L') + 0.5f),
  //       (int)(Get_Duty('R') + 0.5f));
  // OLED_PrintString(0, 0, display_str, &font16x16, OLED_COLOR_NORMAL);

    // OLED_NewFrame();
    char speed_str[20];

    // FloatToStr(speed_str, EncoderGetSpeed_L());
    // sprintf(speed_str, "%s", speed_str);
    // OLED_PrintString(0, 0, speed_str, &font16x16, OLED_COLOR_NORMAL);

    // FloatToStr(speed_str, EncoderGetSpeed_R());
    // sprintf(speed_str, "%s", speed_str);
    // OLED_PrintString(0, 16, speed_str, &font16x16, OLED_COLOR_NORMAL);

    // 发送速度到 VOFA+（FireWater：一行 = 一帧，逗号分通道，\n 结尾）
    // 用 FloatToStr 自己转字符串，不依赖 printf 的浮点支持（newlib-nano 默认没有 %f）
    char vofa_l[12], vofa_r[12];
    FloatToStr(vofa_l, EncoderGetSpeed_L());
    FloatToStr(vofa_r, EncoderGetSpeed_R());
    printf("%s,%s\n", vofa_l, vofa_r);

    // //显示编码器计数
    // sprintf(display_str, "L: %ld ", (long)Encoder_L.period_ticks);
    // OLED_PrintString(0,32,display_str,&font16x16, OLED_COLOR_NORMAL);
    // sprintf(display_str, "R: %ld", (long)Encoder_R.period_ticks);
    // OLED_PrintString(0,48,display_str,&font16x16, OLED_COLOR_NORMAL);

    // OLED_ShowFrame();

  // OLED_ShowFrame();
    

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == KEY1_Pin)
    {
        KEY_EXTI_Callback(&key1);
    }
    else if (GPIO_Pin == KEY2_Pin)
    {
        KEY_EXTI_Callback(&key2);
    }
    else if (GPIO_Pin == KEY3_Pin)
    {
        KEY_EXTI_Callback(&key3);
    }
    else if (GPIO_Pin ==  A_B_Pin)
    {
        Encoder_EXTI_IRQHandler(&Motor_R);
    }
    else if (GPIO_Pin == B_A_Pin)
    {
        Encoder_EXTI_IRQHandler(&Motor_L);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
