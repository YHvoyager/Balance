#include "APP_RemotePID.h"

#include "APP_Control.h"
#include "Vofa.h"
#include "usart.h"

/* Keep the UART callback declaration available without exposing HAL details in the public header. */
#include "stm32f1xx_hal.h"

static uint8_t s_rxByte;

void APP_RemotePID_Init(void)
{
  Vofa_Init();
  (void)HAL_UART_Receive_IT(&huart1, &s_rxByte, 1U);
}

void APP_RemotePID_RxByte(uint8_t byte)
{
  Vofa_FeedByte(byte);
}

void APP_RemotePID_Proc(void)
{
  char line[80];
  float kp, ki, kd;

  while (Vofa_GetLine(line, sizeof(line)) > 0U)
  {
    if ((line[0] != 'L' && line[0] != 'R') ||
        !Vofa_GetFloat(line, "kp", &kp) ||
        !Vofa_GetFloat(line, "ki", &ki) ||
        !Vofa_GetFloat(line, "kd", &kd))
    {
      continue;
    }

    if (kp < 0.0f || kp > 1000.0f || ki < 0.0f || ki > 1000.0f ||
        kd < 0.0f || kd > 1000.0f)
    {
      continue;
    }

    if (line[0] == 'L') APP_PID_SetLeft(kp, ki, kd);
    else                APP_PID_SetRight(kp, ki, kd);
  }
}

/* Add this callback to the application when the module is enabled. */
void APP_RemotePID_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    APP_RemotePID_RxByte(s_rxByte);
    (void)HAL_UART_Receive_IT(&huart1, &s_rxByte, 1U);
  }
}
