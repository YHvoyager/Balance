#include "APP_MPU6050.h"
#include "i2c.h"
#include "stdio.h"
#include "task.h"
#include "math.h"

uint8_t MPU_buffer[14];

float Ax, Ay, Az;
float Gx, Gy, Gz;
float temperature;
float yaw, pitch, roll;
float yaw_a, pitch_a, roll_a;
float yaw_g, pitch_g, roll_g;
static float Gx_bias, Gy_bias, Gz_bias;
#define YAW_GYRO_ZERO_DEADBAND_DPS 0.2f
//yaw绕z轴旋转，pitch绕y轴旋转，roll绕x轴旋转
//单位°

HAL_StatusTypeDef MPU6050_WriteByte(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(
        &hi2c1,
        MPU_6050_ADDRESS,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1,
        100
    );
}

void MPU6050_Init(){
    MPU6050_WriteByte(MPU6050_PWR_MGMT_1_REG, 0x00); //解除休眠模式
    //设置加速度计和陀螺仪的量程
    MPU6050_WriteByte(0x1C, 0x00); //加速度计量程2g
    MPU6050_WriteByte(0x1B, 0x00); //陀螺仪量程±250°/s
    HAL_Delay(100);
    Gx_bias = Gy_bias = Gz_bias = 0.0f;
    for (uint16_t i = 0; i < 200; i++) {
        MPU6050_GetData();
        Gx_bias += MPU6050_GetGx();
        Gy_bias += MPU6050_GetGy();
        Gz_bias += MPU6050_GetGz();
        HAL_Delay(2);
    }
    Gx_bias /= 200.0f; Gy_bias /= 200.0f; Gz_bias /= 200.0f;
}

void MPU6050_GetData(){
    HAL_I2C_Mem_Read(
        &hi2c1,
        MPU_6050_ADDRESS,
        MPU6050_ACCEL_XOUT_H_REG,
        I2C_MEMADD_SIZE_8BIT,
        MPU_buffer,
        14,
        100
    );
    // Ax = (int16_t)((MPU_buffer[0] << 8) | MPU_buffer[1]) / 16384.0f;
    // Ay = (int16_t)((MPU_buffer[2] << 8) | MPU_buffer[3]) / 16384.0f;
    // Az = (int16_t)((MPU_buffer[4] << 8) | MPU_buffer[5]) / 16384.0f;
    
    // temperature = (int16_t)((MPU_buffer[6] << 8) | MPU_ buffer[7]) / 340.0f + 36.53f;

    // Gx = (int16_t)((MPU_buffer[8] << 8) | MPU_buffer[9]) / 131.0f;
    // Gy = (int16_t)((MPU_buffer[10] << 8) | MPU_buffer[11]) / 131.0f;
    // Gz = (int16_t)((MPU_buffer[12] << 8) | MPU_buffer[13]) / 131.0f;
}

float MPU6050_GetAx (void){
    return Ax = (int16_t)((MPU_buffer[0] << 8) | MPU_buffer[1]) / 16384.0f;
}

float MPU6050_GetAy (void){
    return Ay = (int16_t)((MPU_buffer[2] << 8) | MPU_buffer[3]) / 16384.0f;
}

float MPU6050_GetAz (void){
    return Az = (int16_t)((MPU_buffer[4] << 8) | MPU_buffer[5]) / 16384.0f;
}

float MPU6050_GetTemperature (void){
    return temperature = (int16_t)((MPU_buffer[6] << 8) | MPU_buffer[7]) / 340.0f + 36.53f;
}

float MPU6050_GetGx (void){
    return Gx = (int16_t)((MPU_buffer[8] << 8) | MPU_buffer[9]) / 131.0f;
}

float MPU6050_GetGy (void){
    return Gy = (int16_t)((MPU_buffer[10] << 8) | MPU_buffer[11]) / 131.0f;
}

float MPU6050_GetGz (void){
    return Gz = (int16_t)((MPU_buffer[12] << 8) | MPU_buffer[13]) / 131.0f;
}
//
// @brief MPU6050进程函数
//
void MPU6050_Proc(void){

    PERIODIC(5); //每5ms执行一次
    MPU6050_GetData();
    MPU6050_GetGx();
    MPU6050_GetGy();
    MPU6050_GetGz();
    //通过陀螺仪的测量结果计算欧拉角
    float gz_rate = Gz - Gz_bias;
    if (fabsf(gz_rate) < YAW_GYRO_ZERO_DEADBAND_DPS) gz_rate = 0.0f;
    yaw_g  = yaw + gz_rate * 0.005f;
    pitch_g = pitch + (Gy - Gy_bias) * 0.005f;
    roll_g = roll + (Gx - Gx_bias) * 0.005f;

    //通过加速度计解算欧拉角
    MPU6050_GetAx();
    MPU6050_GetAy();
    MPU6050_GetAz();
    //计算欧拉角
    pitch_a = -atan2(Ax, Az) * 180.0f / 3.1415927f;
    roll_a = atan2(Ay, sqrtf(Ax * Ax + Az * Az)) * 180.0f / 3.1415927f;

    //使用互补滤波器融合陀螺仪和加速度计的测量结果
    yaw = yaw_g;
    if (yaw >= 360.0f) yaw -= 360.0f;
    else if (yaw < 0.0f) yaw += 360.0f;
    pitch = 0.95238 * pitch + 0.04762 * pitch_a;
    roll = 0.95238 * roll + 0.04762 * roll_a;
   }
