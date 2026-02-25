#ifndef __USART_H
#define __USART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdio.h>

/* UART DMA 接收缓冲区大小 */
#define UART_DMA_BUF_SIZE  256

/* 全局句柄 */
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef  hdma_usart1_rx;

/* 双缓冲区（ping-pong）：ISR 写一个，任务处理另一个，互不干扰 */
extern uint8_t            g_uart_rx_buf[2][UART_DMA_BUF_SIZE];
extern volatile uint8_t   g_uart_rx_buf_idx;   /* 当前 DMA 正在写入的 buffer 索引 */
extern volatile uint16_t  g_uart_rx_len;        /* 刚完成帧的长度（ISR 中计算） */

/* 函数声明 */
void USART1_UART_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __USART_H */
