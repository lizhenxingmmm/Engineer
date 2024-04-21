/**
 * @file UARTComm.h
 * @author wexhi (wexhi@qq.com)
 * @brief 用于两块开发板间的串口通信
 * @version 0.1
 * @date 2024-04-21
 *
 * @copyright Copyright (c) 2024 CQU QianLi EC 2024 all rights reserved
 *
 */
#ifndef UARTCOMM_H
#define UARTCOMM_H

#include "bsp_usart.h"
#include "daemon.h"

#define UARTCOMM_MAX_BUFF_SIZE 128 // 最大接收/发送缓冲区大小，如果不够可以增加，但不要超过255
#define UARTCOMM_HEADER        's' // 串口通信的帧头
#define UARTCOMM_TAIL          'e' // 串口通信的帧尾
#define UARTCOMM_OFFSET_BYTES  4   // 's' + data length + crc8 + 'e'

#pragma pack(1)
/* UART comm 结构体, 拥有UART comm的app应该包含一个UART comm指针 */
typedef struct
{
    /* 串口实例 */
    USART_Instance *uart_instance;
    /* 发送部分 */
    uint8_t send_data_len;                                               // 发送数据长度
    uint8_t send_buf_len;                                                // 发送缓冲区长度,为发送数据长度+帧头单包数据长度帧尾以及校验和(4)
    uint8_t raw_sendbuf[UARTCOMM_MAX_BUFF_SIZE + UARTCOMM_OFFSET_BYTES]; // 额外4个bytes保存帧头帧尾和校验和
    /* 接收部分 */
    uint8_t recv_data_len;                                               // 接收数据长度
    uint8_t recv_buf_len;                                                // 接收缓冲区长度,为接收数据长度+帧头单包数据长度帧尾以及校验和(4)
    uint8_t raw_recvbuf[UARTCOMM_MAX_BUFF_SIZE + UARTCOMM_OFFSET_BYTES]; // 额外4个bytes保存帧头帧尾和校验和
    uint8_t unpacked_recv_data[UARTCOMM_MAX_BUFF_SIZE];                  // 解包后的数据,调用UARTCommGet()后cast成对应的类型通过指针读取即可
    /* 接收和更新标志位*/
    uint8_t recv_state;   // 接收状态,
    uint8_t cur_recv_len; // 当前已经接收到的数据长度(包括帧头帧尾datalen和校验和)
    uint8_t update_flag;  // 数据更新标志位,当接收到新数据时,会将此标志位置1,调用CANCommGet()后会将此标志位置0

    Daemon_Instance *ucomm_daemon; // 守护进程
} UARTComm_Instance;
#pragma pack()

/* UART comm 初始化结构体 */
typedef struct
{
    UART_HandleTypeDef *uart_handle; // 串口句柄
    uint8_t send_data_len;           // 发送数据长度
    uint8_t recv_data_len;           // 接收数据长度

    uint16_t daemon_counter; // 守护进程计数器
} UARTComm_Init_Config_s;

#endif // UARTCOMM_H