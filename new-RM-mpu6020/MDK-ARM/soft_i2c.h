#ifndef __SOFT_I2C_H
#define __SOFT_I2C_H

#include "main.h"

// Òý½Åºê¶¨Òå
#define I2C_SCL_H()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET)
#define I2C_SCL_L()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET)
#define I2C_SDA_H()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET)
#define I2C_SDA_L()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET)
#define I2C_SDA_READ() HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11)

void I2C_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_SendByte(uint8_t byte);
uint8_t I2C_ReceiveByte(void);
void I2C_SendAck(uint8_t ack);
uint8_t I2C_WaitAck(void);
uint8_t I2C_WriteReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
uint8_t I2C_ReadReg(uint8_t dev_addr, uint8_t reg_addr);
uint8_t I2C_ReadRegs(uint8_t dev_addr, uint8_t reg_addr, uint8_t *buf, uint8_t len);
uint8_t I2C_WriteRegs(uint8_t dev_addr, uint8_t reg_addr, uint8_t *buf, uint8_t len);

#endif