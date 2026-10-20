#include "mahony.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define RAD_TO_DEG  (180.0f / M_PI)

MahonyParams_t mahony_params;
Attitude_t attitude;

void Mahony_Init(float kp, float ki)
{
    mahony_params.Kp = kp;
    mahony_params.Ki = ki;
    mahony_params.halfT = 0.0f;

    attitude.q0 = 1.0f;
    attitude.q1 = 0.0f;
    attitude.q2 = 0.0f;
    attitude.q3 = 0.0f;
    attitude.roll = 0.0f;
    attitude.pitch = 0.0f;
    attitude.yaw = 0.0f;
}

void Mahony_Update(float gx, float gy, float gz,
                   float ax, float ay, float az,
                   float dt)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    mahony_params.halfT = dt * 0.5f;

    // 将角速度从度/秒转换为弧度/秒
    gx *= (M_PI / 180.0f);
    gy *= (M_PI / 180.0f);
    gz *= (M_PI / 180.0f);

    // 加速度归一化
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 估计重力方向（从当前四元数推算）
        halfvx = attitude.q1 * attitude.q3 - attitude.q0 * attitude.q2;
        halfvy = attitude.q0 * attitude.q1 + attitude.q2 * attitude.q3;
        halfvz = attitude.q0 * attitude.q0 - 0.5f + attitude.q3 * attitude.q3;

        // 误差：测量重力方向与估计方向的叉积
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // 积分项：消除陀螺仪零偏
        if (mahony_params.Ki > 0.0f) {
            // 这里用简单的比例积分，实际项目中可加积分限幅
            gx += mahony_params.Ki * halfex * dt * 2.0f;
            gy += mahony_params.Ki * halfey * dt * 2.0f;
            gz += mahony_params.Ki * halfez * dt * 2.0f;
        }

        // 比例项反馈到角速度
        gx += mahony_params.Kp * halfex * 2.0f;
        gy += mahony_params.Kp * halfey * 2.0f;
        gz += mahony_params.Kp * halfez * 2.0f;
    }

    // 四元数微分方程积分
    gx *= mahony_params.halfT;
    gy *= mahony_params.halfT;
    gz *= mahony_params.halfT;
    qa = attitude.q0;
    qb = attitude.q1;
    qc = attitude.q2;

    attitude.q0 += (-qb * gx - qc * gy - attitude.q3 * gz);
    attitude.q1 += (qa * gx + qc * gz - attitude.q3 * gy);
    attitude.q2 += (qa * gy - qb * gz + attitude.q3 * gx);
    attitude.q3 += (qa * gz + qb * gy - qc * gx);

    // 四元数归一化
    recipNorm = 1.0f / sqrtf(attitude.q0 * attitude.q0 +
                             attitude.q1 * attitude.q1 +
                             attitude.q2 * attitude.q2 +
                             attitude.q3 * attitude.q3);
    attitude.q0 *= recipNorm;
    attitude.q1 *= recipNorm;
    attitude.q2 *= recipNorm;
    attitude.q3 *= recipNorm;
}

// 四元数 → 欧拉角
void Mahony_GetEulerAngle(void)
{
    // Roll (X轴旋转)
    attitude.roll = atan2f(2.0f * (attitude.q0 * attitude.q1 +
                                   attitude.q2 * attitude.q3),
                           1.0f - 2.0f * (attitude.q1 * attitude.q1 +
                                          attitude.q2 * attitude.q2)) * RAD_TO_DEG;

    // Pitch (Y轴旋转)
    float sinp = 2.0f * (attitude.q0 * attitude.q2 -
                         attitude.q3 * attitude.q1);
    if (fabsf(sinp) >= 1.0f)
        attitude.pitch = copysignf(M_PI / 2.0f, sinp) * RAD_TO_DEG;
    else
        attitude.pitch = asinf(sinp) * RAD_TO_DEG;

    // Yaw (Z轴旋转)
    attitude.yaw = atan2f(2.0f * (attitude.q0 * attitude.q3 +
                                  attitude.q1 * attitude.q2),
                          1.0f - 2.0f * (attitude.q2 * attitude.q2 +
                                         attitude.q3 * attitude.q3)) * RAD_TO_DEG;
}