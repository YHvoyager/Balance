#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#include <stdint.h>

typedef struct {
    float pitch_deg;          // 互补滤波后的 pitch，单位°
    float gyro_dps;           // 去掉零偏后的角速度，单位°/s
    float theta_dot_ref;      // 内环设定值
    float theta_dot_dot_ref;  // 内环输出
    float omega_ref;          // 轮速设定值，rad/s
    float pitch_acc_deg;      // 加速度计单独解算的 pitch，单位°
    float gyro_bias_dps;      // 当前使用的陀螺零偏，单位°/s
    uint8_t motor_en;         // 0 = 电机未使能
} APP_ControlTelemetry;

void APP_Control_Init(void);
void APP_Control_Proc(void);
void APP_Control_SetAnglePID(float kp, float ki, float kd);
void APP_Control_SetRatePID(float kp, float ki, float kd);
void APP_Control_GetTelemetry(APP_ControlTelemetry *telemetry);

#endif /* APP_CONTROL_H */
