#include "slave_comm.h"
#include "mahony.h"
#include <string.h>

extern UART_HandleTypeDef huart1;
extern Attitude_t attitude;

static uint8_t tx_buf[27];

// CRC8校验（多项式 0x07）
uint8_t CRC8_Calc(uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    return crc;
}

// 从机每1ms调用此函数发送一帧
void Slave_SendFrame(void)
{
    uint8_t idx = 0;

    tx_buf[idx++] = 0xAA;  // 帧头1
    tx_buf[idx++] = 0x55;  // 帧头2
    tx_buf[idx++] = 0x01;  // 帧类型：姿态数据

    // 四元数（float小端）
    memcpy(&tx_buf[idx], &attitude.q0, 4); idx += 4;
    memcpy(&tx_buf[idx], &attitude.q1, 4); idx += 4;
    memcpy(&tx_buf[idx], &attitude.q2, 4); idx += 4;
    memcpy(&tx_buf[idx], &attitude.q3, 4); idx += 4;

    // 欧拉角（int16，放大100倍保留两位小数）
    int16_t r = (int16_t)(attitude.roll * 100);
    int16_t p = (int16_t)(attitude.pitch * 100);
    int16_t y = (int16_t)(attitude.yaw * 100);
    memcpy(&tx_buf[idx], &r, 2); idx += 2;
    memcpy(&tx_buf[idx], &p, 2); idx += 2;
    memcpy(&tx_buf[idx], &y, 2); idx += 2;

    // CRC8（覆盖帧类型到欧拉角）
    tx_buf[idx] = CRC8_Calc(&tx_buf[2], idx - 2); idx++;

    tx_buf[idx++] = 0x0D;  // 帧尾

    HAL_UART_Transmit(&huart1, tx_buf, idx, 10); // 10ms超时
}