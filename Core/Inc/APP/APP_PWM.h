#ifndef __APP_PWM_H__
#define __APP_PWM_H__

#include <stdint.h>

void APP_PWM_Cmd(uint8_t on);
float APP_PWM_Get_Duty(char Wheel);

#endif