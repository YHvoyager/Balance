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
#include "Vofa.h"
#include "APP_Bat.h"
#include "task.h"
#include "ESP01S.h"
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

static volatile float targetOmega = 0.0f;

static void VOFA_SendPIDTelemetry(void);

static void VOFA_PID_CommandProc(void)
{
  char line[80];
  float kp, ki, kd;

  while (Vofa_GetLine(line, sizeof(line)) > 0U)
  {
    if (strcmp(line, "get") == 0 || strcmp(line, "?") == 0)
    {
      /* 配置会在低频 PID 遥测中回传；get 立即触发一次回传。 */
      VOFA_SendPIDTelemetry();
      continue;
    }

    if (Vofa_GetFloat(line, "target", &kp))
    {
      if (kp >= -30.0f && kp <= 30.0f)
      {
        targetOmega = kp;
        APP_Motor_SetOmega_L(kp);
        APP_Motor_SetOmega_R(kp);
      }
      continue;
    }

    /* 命令格式：L kp=0.5 ki=0.01 kd=0 或 R kp=... ki=... kd=... */
    if ((line[0] != 'L' && line[0] != 'R') ||
        !Vofa_GetFloat(line, "kp", &kp) ||
        !Vofa_GetFloat(line, "ki", &ki) ||
        !Vofa_GetFloat(line, "kd", &kd))
    {
      continue;
    }

    /* 限制输入范围，错误命令不会写入 PID。 */
    if (kp < 0.0f || kp > 1000.0f || ki < 0.0f || ki > 1000.0f ||
        kd < 0.0f || kd > 1000.0f)
    {
      continue;
    }

    if (line[0] == 'L') APP_PID_SetLeft(kp, ki, kd);
    else                APP_PID_SetRight(kp, ki, kd);
  }
}

static void SendData_Proc(void)
{
  PERIODIC(10); // 100 Hz

  float data[6] = {
    EncoderGetSpeed_L(),
    EncoderGetSpeed_R(),
    targetOmega,
    APP_Bat_Get(),
    APP_Motor_GetDuty_L(),
    APP_Motor_GetDuty_R()
  };
  Vofa_Send("motor", data, 6, 2);
}

static void SendPIDData_Proc(void)
{
  PERIODIC(500); // 2 Hz，减少对控制主循环的阻塞
  VOFA_SendPIDTelemetry();
}

static void VOFA_SendPIDTelemetry(void)
{
  float data[6];
  APP_PID_GetLeft(&data[0], &data[1], &data[2]);
  APP_PID_GetRight(&data[3], &data[4], &data[5]);
  Vofa_Send("pid", data, 6, 4);
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
  /* USER CODE BEGIN 2 */
  Timer1_StartUs();
  APP_Motor_Init();
  APP_MPU6050_Init();
  HAL_Delay(100); //等待MPU6050稳定
  OLED_Init();
  char display_str[20];
  KEY_AllInit();
  APP_Encoder_Init();
  APP_ADC_Init();
  APP_PWM_Init();
  Vofa_Init();
  /* ★ 串口接收只有一条路：ESP01S_Init() 开 HAL_UARTEx_ReceiveToIdle_DMA，
     由 HAL_UARTEx_RxEventCallback 把字节喂进 Vofa_FeedByte。
     ★ 不要再额外调 HAL_UART_Receive_IT(&huart1, ...)：
       huart1 的接收已经被 DMA 占着，Receive_IT 只会返回 HAL_BUSY，
       而下面那个 HAL_UART_RxCpltCallback 永远不会触发 ⇒ 一条命令也收不到。 */
  ESP01S_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    VOFA_PID_CommandProc();

    
    KEY_Proc(&key1, &key2, &key3);
    APP_Bat_Get_Proc();
    APP_MPU6050_Proc();
    APP_Motor_Proc();

      

    // OLED_NewFrame();

    // //显示角速度
    // FloatToStr(display_str, EncoderGetSpeed_L());
    // OLED_PrintString(0, 0, display_str, &font16x16, 0);
    // FloatToStr(display_str, EncoderGetSpeed_R());
    // OLED_PrintString(0, 16, display_str, &font16x16, 0);

    // OLED_ShowFrame();

    SendData_Proc();
    SendPIDData_Proc();
    

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
