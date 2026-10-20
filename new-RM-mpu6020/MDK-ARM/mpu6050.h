#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"

#define MPU6050_ADDR        0x68

// 寄存器地址
#define MPU6050_SMPLRT_DIV      0x19
#define MPU6050_CONFIG          0x1A
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_WHO_AM_I        0x75

// 数据结构体
typedef struct {
    int16_t ax, ay, az;      // 加速度原始值
    int16_t gx, gy, gz;      // 角速度原始值
    float ax_f, ay_f, az_f;  // 加速度物理值 (g)
    float gx_f, gy_f, gz_f;  // 角速度物理值 (deg/s)
} MPU6050_Data_t;

// 零偏校准值
extern float gyro_offset_x, gyro_offset_y, gyro_offset_z;
extern float accel_offset_x, accel_offset_y, accel_offset_z;

uint8_t MPU6050_Init(void);
uint8_t MPU6050_ReadData(MPU6050_Data_t *data);
void MPU6050_Calibrate(uint16_t samples);

#endif