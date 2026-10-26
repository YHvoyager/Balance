#ifndef APP_CONTROL_H
#define APP_CONTROL_H

typedef struct {
    float pitch_deg;
    float gyro_dps;
    float theta_dot_ref;
    float theta_dot_dot_ref;
    float omega_ref;
} APP_ControlTelemetry;

void APP_Control_Init(void);
void APP_Control_Proc(void);
void APP_Control_SetAnglePID(float kp, float ki, float kd);
void APP_Control_SetRatePID(float kp, float ki, float kd);
void APP_Control_GetTelemetry(APP_ControlTelemetry *telemetry);

#endif /* APP_CONTROL_H */
