#ifndef __MASTER_COMM_H
#define __MASTER_COMM_H

#include "main.h"
extern uint8_t rx_byte;      // 加上这一行
extern uint8_t pc_rx_byte;   // 加上这一行
// 函数声明
void Master_Init(void);
void Master_ProcessByte(uint8_t byte);
void Master_Poll(void);              // 主循环调用：统一执行转发/重传，不在中断里阻塞发送
void Master_RequestRetransmit(void);
void PC_ProcessCommand(uint8_t byte);
uint8_t CRC8_Calc(uint8_t *data, uint16_t len);

#endif
