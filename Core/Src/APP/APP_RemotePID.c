#include "APP_RemotePID.h"

#include <stdio.h>      /* sscanf */
#include <strings.h>    /* strncasecmp */

#include "APP_Control.h"
#include "APP_Motor_Control.h"
#include "ESP01S.h"
#include "Vofa.h"

/* 遥控量的合法范围 */
#define CMD_MIN   (-100)
#define CMD_MAX   ( 100)

/* 调试观测用：最近一条解析成功的 move 指令 */
volatile int g_remote_turn = 0;
volatile int g_remote_move = 0;

void APP_RemotePID_Init(void)
{
  Vofa_Init();
  ESP01S_Init();
}

/*
 * 解析运动指令：move <turnSpeed> <moveSpeed>
 *   turnSpeed  转向速度 [-100, +100]，正数右转
 *   moveSpeed  移动速度 [-100, +100]，正数前进
 *
 * 两种失败都直接丢弃整条指令、保持上一次的值：
 *   1. 参数个数不足（只发了 "move 20"）
 *   2. 参数越界（发了 "move 500 0"）
 * 越界不截断，因为截断会让车悄悄跑到边界值，操作者会误以为是没收到。
 */
static void handle_move(const char *line)
{
  int turnSpeed = 0;
  int moveSpeed = 0;

  /* ★ 用 line + 4 跳过 "move"，不要把它写进格式串。
     sscanf 的格式串区分大小写，而上面的 strncasecmp 不区分 ——
     两者不一致会让 "MOVE 10 10" 被判成格式错误。 */
  if (sscanf(line + 4, "%d %d", &turnSpeed, &moveSpeed) != 2)
  {
    return;
  }

  if (turnSpeed < CMD_MIN || turnSpeed > CMD_MAX ||
      moveSpeed < CMD_MIN || moveSpeed > CMD_MAX)
  {
    return;
  }

  /* 记下来供调试观察 */
  g_remote_turn = turnSpeed;
  g_remote_move = moveSpeed;
  APP_Control_SetCmd(turnSpeed, moveSpeed);

  /* move 是指令百分数 [-100, +100]，换算成速度环的线速度目标：
   *   move = +100  ->  +0.7 m/s
   *   move = -100  ->  -0.7 m/s
   *   倍率 = 0.01 * 0.7 = 0.007 (m/s 每单位)
   */
  APP_Control_SetMoveSpeed((float)moveSpeed * 0.01f * 0.7f);

  /* turn 同样换成转向环的目标偏航角速度：
   *   turn = +100  ->  +2.0 rad/s（约 115°/s）
   */
  APP_Control_SetTurnRate((float)turnSpeed * 0.01f * APP_MAX_YAW_RATE);
}

void APP_RemotePID_Proc(void)
{
  char line[80];
  float kp, ki, kd;
  char command;

  while (Vofa_GetLine(line, sizeof(line)) > 0U)
  {
    /* ---- 运动指令：move <turn> <move> ---- */
    if (strncasecmp(line, "move ", 5) == 0)
    {
      handle_move(line);
      continue;                 /* 不是调参指令，别往下走 */
    }

    /* ---- 调参指令：A/G/L/R kp=.. ki=.. kd=.. ---- */
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
