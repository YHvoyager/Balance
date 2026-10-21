#include "APP_MPU6050.h"
#include "i2c.h"
#include "stdio.h"
#include "task.h"

uint8_t MPU_buffer[14];

float Ax, Ay, Az;
float Gx, Gy, Gz;
float temperature;
float yaw, pitch, roll;
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
    MPU6050_WriteByte(0x1B, 0x18); //陀螺仪量程+-2000°/s    
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
    yaw  = yaw + Gz * 0.005f; //每次调用间隔为5ms秒
    pitch = pitch + Gy * 0.005f;
    roll = roll + Gx * 0.005f;
   }