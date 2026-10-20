#include "soft_i2c.h"

// 微秒级延时（用SysTick实现，72MHz主频下约72个周期为1μs）
static void i2c_delay(void)
{
    uint32_t i = 50;  // 根据实际主频调整，约5μs
    while(i--);
}

void I2C_Init(void)
{
    // GPIO已在MX_GPIO_Init()中配置为开漏输出
    I2C_SCL_H();
    I2C_SDA_H();
}

void I2C_Start(void)
{
    I2C_SDA_H();
    I2C_SCL_H();
    i2c_delay();
    I2C_SDA_L();   // SCL高电平期间，SDA从高变低 → 起始信号
    i2c_delay();
    I2C_SCL_L();   // 拉低SCL，占用总线
    i2c_delay();
}

void I2C_Stop(void)
{
    I2C_SDA_L();
    I2C_SCL_H();
    i2c_delay();
    I2C_SDA_H();   // SCL高电平期间，SDA从低变高 → 停止信号
    i2c_delay();
}

void I2C_SendByte(uint8_t byte)
{
    for (uint8_t i = 0; i < 8; i++) {
        if (byte & 0x80)
            I2C_SDA_H();
        else
            I2C_SDA_L();
        byte <<= 1;
        i2c_delay();
        I2C_SCL_H();
        i2c_delay();
        I2C_SCL_L();
        i2c_delay();
    }
}

uint8_t I2C_ReceiveByte(void)
{
    uint8_t byte = 0;
    I2C_SDA_H();  // 释放SDA，设为输入
    for (uint8_t i = 0; i < 8; i++) {
        byte <<= 1;
        I2C_SCL_H();
        i2c_delay();
        if (I2C_SDA_READ())
            byte |= 0x01;
        I2C_SCL_L();
        i2c_delay();
    }
    return byte;
}

uint8_t I2C_WaitAck(void)
{
    uint8_t ack;
    uint16_t timeout = 0;
    
    I2C_SDA_H();
    i2c_delay();
    I2C_SCL_H();
    i2c_delay();
    
    // 等待从机拉低 SDA（给 ACK）
    while (I2C_SDA_READ()) 
    {
        timeout++;
        if (timeout > 1000) // 如果等了1000次还没拉低，说明硬件没接好或坏了
        {
            I2C_SCL_L();
            i2c_delay();
            return 1; // 强行返回 NACK（错误），防止程序死在这里
        }
    }
    
    ack = I2C_SDA_READ(); // 此时SDA应该是低电平，ack=0
    I2C_SCL_L();
    i2c_delay();
    return ack;
}

void I2C_SendAck(uint8_t ack)
{
    if (ack == 0)
        I2C_SDA_L();  // 发送ACK
    else
        I2C_SDA_H();  // 发送NACK
    i2c_delay();
    I2C_SCL_H();
    i2c_delay();
    I2C_SCL_L();
    i2c_delay();
    I2C_SDA_H();
}

// 写一个字节到指定寄存器
uint8_t I2C_WriteReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
    I2C_Start();
    I2C_SendByte(dev_addr << 1);  // 写地址
    if (I2C_WaitAck()) { I2C_Stop(); return 1; }
    I2C_SendByte(reg_addr);
    if (I2C_WaitAck()) { I2C_Stop(); return 1; }
    I2C_SendByte(data);
    if (I2C_WaitAck()) { I2C_Stop(); return 1; }
    I2C_Stop();
    return 0;
}

// 读一个字节
uint8_t I2C_ReadReg(uint8_t dev_addr, uint8_t reg_addr)
{
    uint8_t data;
    I2C_Start();
    I2C_SendByte(dev_addr << 1);
    if (I2C_WaitAck()) { I2C_Stop(); return 0xFF; } // 读失败返回 0xFF
    I2C_SendByte(reg_addr);
    if (I2C_WaitAck()) { I2C_Stop(); return 0xFF; }
    I2C_Start();
    I2C_SendByte((dev_addr << 1) | 0x01);
    if (I2C_WaitAck()) { I2C_Stop(); return 0xFF; }
    data = I2C_ReceiveByte();
    I2C_SendAck(1); 
    I2C_Stop();
    return data;
}

// 连续读多个字节
uint8_t I2C_ReadRegs(uint8_t dev_addr, uint8_t reg_addr, uint8_t *buf, uint8_t len)
{
    I2C_Start();
    I2C_SendByte(dev_addr << 1);
    if (I2C_WaitAck()) { I2C_Stop(); return 1; }
    I2C_SendByte(reg_addr);
    if (I2C_WaitAck()) { I2C_Stop(); return 1; }
    I2C_Start();
    I2C_SendByte((dev_addr << 1) | 0x01);
    if (I2C_WaitAck()) { I2C_Stop(); return 1; }
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = I2C_ReceiveByte();
        if (i < len - 1)
            I2C_SendAck(0);  // ACK
        else
            I2C_SendAck(1);  // NACK（最后一个字节）
    }
    I2C_Stop();
    return 0;
}