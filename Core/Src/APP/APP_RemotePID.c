#include "APP_RemotePID.h"

#include "APP_Control.h"
#include "APP_Motor_Control.h"
#include "ESP01S.h"
#include "Vofa.h"

void APP_RemotePID_Init(void)
{
  Vofa_Init();
  ESP01S_Init();
}

void APP_RemotePID_Proc(void)
{
  char line[80];
  float kp, ki, kd;
  char command;

  while (Vofa_GetLine(line, sizeof(line)) > 0U)
  {
    command = line[0];
    if ((command != 'A' && command != 'G' && command != 'L' && command != 'R') ||
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

    if (command == 'A')      APP_Control_SetAnglePID(kp, ki, kd);
    else if (command == 'G') APP_Control_SetRatePID(kp, ki, kd);
    else if (command == 'L') APP_PID_SetLeft(kp, ki, kd);
    else                     APP_PID_SetRight(kp, ki, kd);
  }
}
