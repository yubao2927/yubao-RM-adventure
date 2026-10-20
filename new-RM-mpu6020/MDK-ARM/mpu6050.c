#include "mpu6050.h"
#include "soft_i2c.h"
#include <math.h>

float gyro_offset_x = 0, gyro_offset_y = 0, gyro_offset_z = 0;
float accel_offset_x = 0, accel_offset_y = 0, accel_offset_z = 0;

// 量程灵敏度：默认±2g 和 ±250°/s
#define ACCEL_SENSITIVITY   16384.0f  // LSB/g
#define GYRO_SENSITIVITY    131.0f    // LSB/(°/s)

uint8_t MPU6050_Init(void)
{
    uint8_t who = I2C_ReadReg(MPU6050_ADDR, MPU6050_WHO_AM_I);
    if (who != 0x68) return 1;  // 通信失败

    // 复位设备
    I2C_WriteReg(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x80);
    HAL_Delay(100);
    // 唤醒，选择X轴陀螺仪作为时钟源
    I2C_WriteReg(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x01);
    HAL_Delay(10);
    // 采样率分频：1kHz / (1+0) = 1kHz（陀螺仪输出频率）
    I2C_WriteReg(MPU6050_ADDR, MPU6050_SMPLRT_DIV, 0x00);
    // 配置DLPF：带宽44Hz，延迟4.9ms
    I2C_WriteReg(MPU6050_ADDR, MPU6050_CONFIG, 0x03);
    // 陀螺仪量程：±250°/s
    I2C_WriteReg(MPU6050_ADDR, MPU6050_GYRO_CONFIG, 0x00);
    // 加速度量程：±2g
    I2C_WriteReg(MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 0x00);

    return 0;
}

uint8_t MPU6050_ReadData(MPU6050_Data_t *data)
{
    uint8_t buf[14];
    if (I2C_ReadRegs(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, buf, 14) != 0)
        return 1;

    data->ax = (int16_t)((buf[0] << 8) | buf[1]);
    data->ay = (int16_t)((buf[2] << 8) | buf[3]);
    data->az = (int16_t)((buf[4] << 8) | buf[5]);
    // buf[6], buf[7] 是温度
    data->gx = (int16_t)((buf[8] << 8) | buf[9]);
    data->gy = (int16_t)((buf[10] << 8) | buf[11]);
    data->gz = (int16_t)((buf[12] << 8) | buf[13]);

    // 转换为物理值（减去零偏）
    data->ax_f = data->ax / ACCEL_SENSITIVITY - accel_offset_x;
    data->ay_f = data->ay / ACCEL_SENSITIVITY - accel_offset_y;
    data->az_f = data->az / ACCEL_SENSITIVITY - accel_offset_z;
    data->gx_f = data->gx / GYRO_SENSITIVITY - gyro_offset_x;
    data->gy_f = data->gy / GYRO_SENSITIVITY - gyro_offset_y;
    data->gz_f = data->gz / GYRO_SENSITIVITY - gyro_offset_z;

    return 0;
}

// 上电自校准：静止采样N次取平均作为零偏
// 注意：调用此函数时传感器必须水平静止放置
void MPU6050_Calibrate(uint16_t samples)
{
    MPU6050_Data_t d;
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    float sum_ax = 0, sum_ay = 0, sum_az = 0;

    // 先把零偏清零，否则采集到的数据不准确
    gyro_offset_x = gyro_offset_y = gyro_offset_z = 0;
    accel_offset_x = accel_offset_y = accel_offset_z = 0;

    for (uint16_t i = 0; i < samples; i++) {
        MPU6050_ReadData(&d);
        sum_gx += d.gx_f;
        sum_gy += d.gy_f;
        sum_gz += d.gz_f;
        sum_ax += d.ax_f;
        sum_ay += d.ay_f;
        sum_az += d.az_f;
        HAL_Delay(5);
    }

    gyro_offset_x = sum_gx / samples;
    gyro_offset_y = sum_gy / samples;
    gyro_offset_z = sum_gz / samples;

    // 加速度计零偏：水平静止时X=0, Y=0, Z=1g
    accel_offset_x = sum_ax / samples;
    accel_offset_y = sum_ay / samples;
    accel_offset_z = sum_az / samples - 1.0f;  // 减去重力1g
}