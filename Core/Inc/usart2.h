#ifndef __USART2_H
#define __USART2_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/* ============================================================================
 * USART2 — ESP32-S3 通信通道
 *
 * 硬件连接：
 *   PA2 (TX) → ESP32-S3 串口0 RX
 *   PA3 (RX) ← ESP32-S3 串口0 TX
 *
 * 时钟：APB1 = 32MHz（HCLK/2）
 * 波特率：115200
 * DMA：DMA1_Channel6 (USART2_RX)
 * 中断：USART2 IDLE + DMA1_Channel6
 * 接收模式：DMA 双缓冲 + IDLE 帧检测（与 USART1 同一架构）
 * ============================================================================ */

/* DMA 接收缓冲区大小：257 字节（比最大帧 256B 多 1 字节）
 * 目的：OTA DATA 帧恰好 256 字节时，DMA TC 不会触发（缓冲区未填满），
 * 只走 IDLE 中断路径，避免 DMA TC 与 IDLE 竞争导致帧丢失。
 * 额外消耗仅 2 字节（双缓冲 × 1），不影响 RAM 上限。 */
#define UART2_DMA_BUF_SIZE  257

/* 全局句柄 */
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef  hdma_usart2_rx;

/* 双缓冲区（ping-pong） */
extern uint8_t            g_uart2_rx_buf[2][UART2_DMA_BUF_SIZE];
extern volatile uint8_t   g_uart2_rx_buf_idx;
extern volatile uint16_t  g_uart2_rx_len;

/* 初始化 */
void USART2_UART_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __USART2_H */
