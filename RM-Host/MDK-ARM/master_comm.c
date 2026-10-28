#include "master_comm.h"
#include <string.h>
// 放在文件开头的全局变量区域
uint8_t rx_byte;       // 存放 USART1 接收到的字节
uint8_t pc_rx_byte;    // 存放 USART2 接收到的字节
// 声明外部串口句柄（在main.c中定义）
extern UART_HandleTypeDef huart1; // 负责与从机通信
extern UART_HandleTypeDef huart2; // 负责与PC通信

#define FRAME_SIZE 27

// 定义状态机的各个状态
typedef enum {
    STATE_HEADER1, // 等待第一个帧头 0xAA
    STATE_HEADER2, // 等待第二个帧头 0x55
    STATE_TYPE,    // 等待帧类型 0x01
    STATE_DATA,    // 接收核心数据
    STATE_CRC,     // 接收CRC校验位
    STATE_TAIL     // 接收帧尾 0x0D
} RxState_t;

static RxState_t rx_state = STATE_HEADER1;
static uint8_t rx_buffer[FRAME_SIZE];
static uint8_t rx_index = 0;

// 中断里只解析、置标志；实际发送由主循环 Master_Poll() 完成，
// 避免在中断里阻塞发送 27 字节（约0.59ms）导致下一帧字节丢失
static uint8_t tx_to_pc[FRAME_SIZE];
static volatile uint8_t tx_flags = 0;
#define TX_FLAG_TO_PC     0x01u
#define TX_FLAG_RETRANS   0x02u


// CRC8校验实现（如果主机没有单独建crc8.c，就放这里）
uint8_t CRC8_Calc(uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0x07;
            else crc <<= 1;
        }
    }
    return crc;
}

// 主机初始化：开启串口接收中断
void Master_Init(void)
{
    // 开启USART1接收从机的数据
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    // 开启USART2接收PC的指令
    HAL_UART_Receive_IT(&huart2, &pc_rx_byte, 1);
}

// 处理从机发来的一个字节
void Master_ProcessByte(uint8_t byte)
{
    switch (rx_state) {
    case STATE_HEADER1:
        if (byte == 0xAA) { // 匹配到第一个帧头
            rx_buffer[0] = byte;
            rx_index = 1;   // 准备存放下一个字节
            rx_state = STATE_HEADER2;
        }
        break;

        case STATE_HEADER2:
        if (byte == 0x55) { // 匹配到第二个帧头
            rx_buffer[rx_index++] = byte;
            rx_state = STATE_TYPE;
        } else if (byte == 0xAA) { // 【容错升级】如果收到的又是 0xAA，说明可能是新的一帧的开头，重新等 0x55
            rx_buffer[0] = byte;
            rx_index = 1; 
            // 状态保持 STATE_HEADER2 不变
        } else {
            rx_state = STATE_HEADER1; // 匹配彻底失败，重新等待帧头
            rx_index = 0;             // 【重要】索引清零，防止数据错乱
        }
        break;

    case STATE_TYPE:
        rx_buffer[rx_index++] = byte;
        rx_state = STATE_DATA; // 开始接收数据段
        break;

    case STATE_DATA:
        rx_buffer[rx_index++] = byte;
        // 数据总共21字节 (4*4 四元数 + 2*3 欧拉角)。加上前3个字节，一共24字节
        if (rx_index == 25) { 
            rx_state = STATE_CRC;
        }
        break;

    case STATE_CRC:
        rx_buffer[rx_index++] = byte; // 存下CRC值
        rx_state = STATE_TAIL;
        break;

    case STATE_TAIL:
        if (byte == 0x0D) { // 收到帧尾
            rx_buffer[rx_index] = byte; // 存入0x0D
            
            // 开始校验：对索引2到索引24的数据（共23字节）计算CRC
            uint8_t calc_crc = CRC8_Calc(&rx_buffer[2], 23); 
            
            if (calc_crc == rx_buffer[25]) {
                // 校验成功！拷贝到发送缓冲，由主循环经 USART2 转发给 PC
                memcpy(tx_to_pc, rx_buffer, FRAME_SIZE);
                tx_flags |= TX_FLAG_TO_PC;
            } else {
                // 校验失败！置标志，由主循环发送重传请求
                tx_flags |= TX_FLAG_RETRANS;
            }
        }
        // 无论成功失败，状态机重置，准备接收下一帧
        rx_state = STATE_HEADER1;
        rx_index = 0;
        break;
    }
}

// 主循环调用：把中断里攒下的发送动作统一执行
void Master_Poll(void)
{
    if (tx_flags & TX_FLAG_TO_PC) {
        tx_flags &= ~TX_FLAG_TO_PC;
        HAL_UART_Transmit(&huart2, tx_to_pc, FRAME_SIZE, 10);
    }
    if (tx_flags & TX_FLAG_RETRANS) {
        tx_flags &= ~TX_FLAG_RETRANS;
        Master_RequestRetransmit();
    }
}

// 发送重传请求指令给从机
void Master_RequestRetransmit(void)
{
    uint8_t req[5];
    req[0] = 0xAA;
    req[1] = 0x55;
    req[2] = 0x02;  // 0x02表示重传请求
    req[3] = CRC8_Calc(&req[2], 1); // 校验帧类型
    req[4] = 0x0D;
    HAL_UART_Transmit(&huart1, req, 5, 10);
}
// 处理PC发来的指令（透传给从机）
// PC指令格式：0xAA 0x55 0x03 Kp(4字节) Ki(4字节) CRC8 0x0D  (共12字节)
void PC_ProcessCommand(uint8_t byte)
{
    static uint8_t pc_buf[12];
    static uint8_t pc_idx = 0;

    // 寻找帧头
    if (pc_idx == 0 && byte != 0xAA) return;
    pc_buf[pc_idx++] = byte;

    // 收满12个字节
    if (pc_idx >= 12) {
        // 校验通过后透传给从机（也可以不做校验直接传，因为PC端一般不会错）
        if (pc_buf[11] == 0x0D) {
            HAL_UART_Transmit(&huart1, pc_buf, 12, 10); 
        }
        pc_idx = 0; // 重置，准备接收下一条指令
    }
}
