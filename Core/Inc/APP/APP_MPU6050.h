#ifndef __MPU6050_H__
#define __MPU6050_H__

#define MPU_6050_ADDRESS (0x68 << 1) // MPU6050 I2C地址

#define MPU6050_WHO_AM_I_REG     0x75
#define MPU6050_PWR_MGMT_1_REG   0x6B
#define MPU6050_ACCEL_XOUT_H_REG 0x3B

// 欧拉角（单位°，供外部读取）
extern float yaw, pitch, roll;

void APP_MPU6050_Init(void);
void MPU6050_GetData(void);
void APP_MPU6050_Proc(void);
float MPU6050_GetAx(void);
float MPU6050_GetAy(void);
float MPU6050_GetAz(void);
float MPU6050_GetTemperature(void);
float MPU6050_GetGx(void);
float MPU6050_GetGy(void);
float MPU6050_GetGz(void);
/** 去零偏、过死区后的 Z 轴角速度，单位 度/秒。转向环用它作反馈。 */
float APP_MPU6050_GetGzRate(void);
float APP_MPU6050_GetYaw(void);
float APP_MPU6050_GetPitch(void);
float APP_MPU6050_GetRoll(void);
float APP_MPU6050_GetGy(void);
float APP_MPU6050_GetGyBias(void);
float APP_MPU6050_GetPitchAcc(void);   // 加速度计单独解算的 pitch（未融合），单位°

#endif