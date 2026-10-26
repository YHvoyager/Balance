#ifndef APP_REMOTE_PID_H
#define APP_REMOTE_PID_H

#ifdef __cplusplus
extern "C" {
#endif

/* Commands: A/G/L/R kp=... ki=... kd=... for angle, gyro rate, left/right speed. */
void APP_RemotePID_Init(void);
void APP_RemotePID_Proc(void);

#ifdef __cplusplus
}
#endif

#endif
