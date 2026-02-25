#include "usart2.h"

/* ================= 全局变量 ================= */
UART_HandleTypeDef huart2;
DMA_HandleTypeDef  hdma_usart2_rx;

/* 双缓冲区 */
uint8_t            g_uart2_rx_buf[2][UART2_DMA_BUF_SIZE];
volatile uint8_t   g_uart2_rx_buf_idx = 0;
volatile uint16_t  g_uart2_rx_len     = 0;

/* ================= 串口初始化 ================= */
void USART2_UART_Init(void)
{
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        while(1);
    }
}
