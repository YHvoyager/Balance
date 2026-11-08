#ifndef APP_REMOTE_PID_H
#define APP_REMOTE_PID_H

#ifdef __cplusplus
extern "C" {
#endif

/* Commands: A/G/L/R kp=... ki=... kd=... for angle, gyro rate, left/right speed. */
/*           move <turnSpeed> <moveSpeed>       for remote motion command.    */
void APP_RemotePID_Init(void);
void APP_RemotePID_Proc(void);

/* 调试观测：最近一条解析成功的 move 指令（还没收到时为 0）。
   在调试器的 Watch 窗口里加这两个变量，就能确认解析是否正确。
   目前只做记录，不驱动电机。 */
extern volatile int g_remote_turn;   /* -100 ~ +100 */
extern volatile int g_remote_move;   /* -100 ~ +100 */

#ifdef __cplusplus
}
#endif

#endif
