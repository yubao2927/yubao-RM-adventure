#ifndef __MAHONY_H
#define __MAHONY_H

// Mahony滤波器可调参数
typedef struct {
    float Kp;          // 比例增益（加速度计权重）
    float Ki;          // 积分增益（陀螺仪零偏纠正）
    float halfT;       // 半采样周期
} MahonyParams_t;

typedef struct {
    float q0, q1, q2, q3;  // 四元数
    float roll, pitch, yaw; // 欧拉角（度）
} Attitude_t;

extern MahonyParams_t mahony_params;
extern Attitude_t attitude;

void Mahony_Init(float kp, float ki);
void Mahony_Update(float gx, float gy, float gz,
                   float ax, float ay, float az,
                   float dt);
void Mahony_GetEulerAngle(void);

#endif