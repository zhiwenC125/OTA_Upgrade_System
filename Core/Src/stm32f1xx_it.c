/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
#include "usart1.h"
#include "usart2.h"
#include "FreeRTOS.h"
#include "semphr.h"

/* OTA 信号量（定义在 main.c） */
extern SemaphoreHandle_t xOtaSemaphore;

/* ESP32 通信信号量（定义在 main.c） */
extern SemaphoreHandle_t xEsp32Semaphore;

/* 占位句柄（stm32f1xx_it.c 中引用，暂未使用） */
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern TIM_HandleTypeDef htim2;

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/

void NMI_Handler(void)
{
    while (1) {}
}

void HardFault_Handler(void)
{
    while (1) {}
}

void MemManage_Handler(void)
{
    while (1) {}
}

void BusFault_Handler(void)
{
    while (1) {}
}

void UsageFault_Handler(void)
{
    while (1) {}
}

void DebugMon_Handler(void)
{
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/******************************************************************************/

void DMA1_Channel2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_spi1_rx);
}

void DMA1_Channel3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_spi1_tx);
}

void DMA1_Channel5_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart1_rx);
}

void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim2);
}

/**
  * @brief USART1 中断 —— IDLE 空闲帧检测
  *
  * 修复 #3：在 ISR 中完成所有时序敏感操作：
  *   1) 停止 DMA
  *   2) 读取 DMA 计数器算出本帧长度
  *   3) 切换双缓冲索引
  *   4) 在新 buffer 上重启 DMA（不会覆盖刚收完的数据）
  *   5) 释放信号量通知任务
  *
  * 任务拿到信号量后，处理 "另一个" buffer（刚收完的那个），无竞争。
  */
void USART1_IRQHandler(void)
{
    /* 先清除所有 UART 错误标志（PE/FE/NE/ORE），防止 HAL_UART_IRQHandler
     * 检测到 ORE 后调用 UART_EndRxTransfer 永久禁用 DMA。
     * STM32F1 SR 寄存器：读 SR 再读 DR 即可清除错误标志。 */
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE) ||
        __HAL_UART_GET_FLAG(&huart1, UART_FLAG_FE)  ||
        __HAL_UART_GET_FLAG(&huart1, UART_FLAG_NE)  ||
        __HAL_UART_GET_FLAG(&huart1, UART_FLAG_PE))
    {
        volatile uint32_t tmp = huart1.Instance->SR;
        tmp = huart1.Instance->DR;   /* 读 DR 清除错误标志 */
        (void)tmp;
    }

    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)
    {
        /* 清除 IDLE 标志 */
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);

        /* 1. 停止 DMA，锁定计数器 */
        HAL_UART_DMAStop(&huart1);

        /* 2. 计算接收长度（在 ISR 中算，此时计数器还是准确的） */
        g_uart_rx_len = UART_DMA_BUF_SIZE
                        - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);

        /* 3. 切换双缓冲：当前完成帧在 buf[idx]，切换到另一个 */
        g_uart_rx_buf_idx = 1 - g_uart_rx_buf_idx;

        /* 4. 在新 buffer 上重启 DMA（旧 buffer 留给任务安全处理） */
        HAL_UART_Receive_DMA(&huart1,
                             g_uart_rx_buf[g_uart_rx_buf_idx],
                             UART_DMA_BUF_SIZE);

        /* 5. 安全释放信号量（检查非空，防止上电噪声时 semaphore 还未创建） */
        if (xOtaSemaphore != NULL)
        {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(xOtaSemaphore, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
    /* 注意：不调用 HAL_UART_IRQHandler()，因为我们已手动处理所有标志。
     * 调用它会导致 HAL 在检测到 ORE 时再次中止 DMA，破坏双缓冲状态。 */
}

/* ====== USART2 — ESP32-S3 通信通道 ====== */

void DMA1_Channel6_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart2_rx);
}

/**
  * @brief USART2 中断 —— IDLE 空闲帧检测（与 USART1 同一架构）
  *
  * UART2_DMA_BUF_SIZE = 257，比最大帧（256B）多 1，
  * 所以 DMA TC 永远不会触发，所有帧统一由 IDLE 处理，逻辑简洁无竞争。
  */
void USART2_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE) ||
        __HAL_UART_GET_FLAG(&huart2, UART_FLAG_FE)  ||
        __HAL_UART_GET_FLAG(&huart2, UART_FLAG_NE)  ||
        __HAL_UART_GET_FLAG(&huart2, UART_FLAG_PE))
    {
        volatile uint32_t tmp = huart2.Instance->SR;
        tmp = huart2.Instance->DR;
        (void)tmp;
    }

    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);

        HAL_UART_DMAStop(&huart2);

        g_uart2_rx_len = UART2_DMA_BUF_SIZE
                         - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);

        g_uart2_rx_buf_idx = 1 - g_uart2_rx_buf_idx;

        HAL_UART_Receive_DMA(&huart2,
                             g_uart2_rx_buf[g_uart2_rx_buf_idx],
                             UART2_DMA_BUF_SIZE);

        if (xEsp32Semaphore != NULL)
        {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(xEsp32Semaphore, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}
