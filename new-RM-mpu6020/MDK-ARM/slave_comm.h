#ifndef __SLAVE_COMM_H
#define __SLAVE_COMM_H

#include "main.h"

// º¯ÊıÉùÃ÷
void Slave_SendFrame(void);
uint8_t CRC8_Calc(uint8_t *data, uint16_t len);
void Slave_ProcessCommand(uint8_t *cmd, uint16_t len);

#endif