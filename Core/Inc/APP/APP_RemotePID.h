#ifndef APP_REMOTE_PID_H
#define APP_REMOTE_PID_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Optional module, disabled by default. To enable:
 * 1. Configure CMake with -DENABLE_REMOTE_PID_TUNING=ON.
 * 2. Call APP_RemotePID_Init() once after MX_USART1_UART_Init().
 * 3. Forward USART1 receive-complete events to APP_RemotePID_RxCpltCallback().
 * 4. Call APP_RemotePID_Proc() from the main loop.
 */
void APP_RemotePID_Init(void);

/* Call from the main loop to apply received commands. */
void APP_RemotePID_Proc(void);

/* Optional explicit byte feed, for projects with an existing USART callback. */
void APP_RemotePID_RxByte(uint8_t byte);

/* Call this from HAL_UART_RxCpltCallback() when enabling the module. */
void APP_RemotePID_RxCpltCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif
