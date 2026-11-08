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

/* ---------------------------------------------------------------------------
 * 遥控指令缓存（move 指令解析后存在这里）
 *
 *   turn  ∈ [-100, +100]   转向量：正数右转
 *   move  ∈ [-100, +100]   移动量：正数前进
 *
 * ★ 当前阶段【只存不用】。控制环还没读这两个量，存进来是为验证链路和解析。
 *   以后要真正控制，在 APP_Control_Proc() 里把百分数换算成物理量即可。
 * ------------------------------------------------------------------------ */
void APP_Control_SetCmd(int turn, int move);
void APP_Control_GetCmd(int *turn, int *move);

/**
 * 设置速度环的目标线速度，单位 m/s。正数前进，0 = 原地平衡。
 *
 * ★ 注意：不要直接在指令解析里调 PID_ChangeSp()。
 *   APP_Control_Proc() 每帧都会重设速度环 SP，直接改会被下一帧覆盖。
 *   这个函数是把值存起来，Proc 每帧读它。
 */
void APP_Control_SetMoveSpeed(float MoveSpeed);

/**
 * 设置转向环的目标偏航角速度，单位 rad/s。
 * 正数 → ω_diff > 0 → 左轮快右轮慢。
 */
void APP_Control_SetTurnRate(float rate);

/* turn = 100 对应的目标偏航角速度（rad/s）。2.0 ≈ 115°/s，想更灵活就调大 */
#define APP_MAX_YAW_RATE   2.0f

/** 调试用：读转向环的实测偏航角速度(rad/s) 和输出轮速差(rad/s) */
void APP_Control_GetTurnDebug(float *gz_rad, float *omega_diff);

#endif /* APP_CONTROL_H */
