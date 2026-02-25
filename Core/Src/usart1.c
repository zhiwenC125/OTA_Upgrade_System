#include "usart1.h"
#include "usart2.h"

/* ================= 全局变量 ================= */
UART_HandleTypeDef huart1;
DMA_HandleTypeDef  hdma_usart1_rx;  /* 全局唯一 DMA 句柄，ISR 也用它 */

/* 双缓冲区 */
uint8_t            g_uart_rx_buf[2][UART_DMA_BUF_SIZE];
volatile uint8_t   g_uart_rx_buf_idx = 0;
volatile uint16_t  g_uart_rx_len     = 0;

/*
 * printf 重定向统一由 syscalls.c 中的 _write() 处理，
 * 这里不再重复实现 __io_putchar，避免维护混乱。
 */

/* ================= 串口初始化 ================= */
void USART1_UART_Init(void)
{
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        while(1);
    }
}

/* ================= MSP 初始化 (底层引脚 + DMA + 中断) ================= */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if(huart->Instance == USART1)
    {
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_DMA1_CLK_ENABLE();

        /* PA9 -> TX (复用推挽) */
        GPIO_InitStruct.Pin   = GPIO_PIN_9;
        GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* PA10 -> RX (浮空输入) */
        GPIO_InitStruct.Pin  = GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* ====== 修复 #1：使用全局 hdma_usart1_rx，不再用局部 static ====== */
        hdma_usart1_rx.Instance                 = DMA1_Channel5;
        hdma_usart1_rx.Init.Direction            = DMA_PERIPH_TO_MEMORY;
        hdma_usart1_rx.Init.PeriphInc            = DMA_PINC_DISABLE;
        hdma_usart1_rx.Init.MemInc               = DMA_MINC_ENABLE;
        hdma_usart1_rx.Init.PeriphDataAlignment  = DMA_PDATAALIGN_BYTE;
        hdma_usart1_rx.Init.MemDataAlignment     = DMA_MDATAALIGN_BYTE;
        hdma_usart1_rx.Init.Mode                 = DMA_NORMAL;
        hdma_usart1_rx.Init.Priority             = DMA_PRIORITY_MEDIUM;

        if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
        {
            while(1);
        }

        /* 将 DMA 句柄关联到 UART（HAL 内部会通过 huart->hdmarx 找到它） */
        __HAL_LINKDMA(huart, hdmarx, hdma_usart1_rx);

        /* DMA 通道中断 */
        HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

        /* ====== 修复 #2：开启 USART1 NVIC（之前缺失，IDLE 中断无法触发） ====== */
        HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);

        /* 开启 IDLE 空闲中断 */
        __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    }
    else if(huart->Instance == USART2)
    {
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_DMA1_CLK_ENABLE();

        /* PA2 -> TX (复用推挽) */
        GPIO_InitStruct.Pin   = GPIO_PIN_2;
        GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* PA3 -> RX (上拉输入：未接 ESP32 时保持高电平，防止浮空噪声误触发) */
        GPIO_InitStruct.Pin  = GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* DMA1_Channel6: USART2_RX */
        hdma_usart2_rx.Instance                 = DMA1_Channel6;
        hdma_usart2_rx.Init.Direction            = DMA_PERIPH_TO_MEMORY;
        hdma_usart2_rx.Init.PeriphInc            = DMA_PINC_DISABLE;
        hdma_usart2_rx.Init.MemInc               = DMA_MINC_ENABLE;
        hdma_usart2_rx.Init.PeriphDataAlignment  = DMA_PDATAALIGN_BYTE;
        hdma_usart2_rx.Init.MemDataAlignment     = DMA_MDATAALIGN_BYTE;
        hdma_usart2_rx.Init.Mode                 = DMA_NORMAL;
        hdma_usart2_rx.Init.Priority             = DMA_PRIORITY_MEDIUM;

        if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
        {
            while(1);
        }

        __HAL_LINKDMA(huart, hdmarx, hdma_usart2_rx);

        /* DMA 通道中断 */
        HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

        /* USART2 NVIC */
        HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);

        /* 开启 IDLE 空闲中断 */
        __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    }
}
