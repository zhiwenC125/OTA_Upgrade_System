#include "esp32_comm.h"
#include "usart2.h"
#include "ota_task.h"
#include "sensor_task.h"
#include <stdio.h>
#include <string.h>

void vEsp32CommTask(void *pvParameters)
{
    (void)pvParameters;

    /* 启动 USART2 DMA 接收 */
    HAL_UART_Receive_DMA(&huart2,
                         g_uart2_rx_buf[g_uart2_rx_buf_idx],
                         UART2_DMA_BUF_SIZE);

    printf("[ESP32] Comm task started (OTA bridge mode)\r\n");

    /* 诊断：启动时在 USART2 上发送测试消息，ESP32 应收到此消息 */
    const char *hello = "STM32_READY\r\n";
    if (xSemaphoreTake(xUart2TxMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        HAL_UART_Transmit(&huart2, (uint8_t *)hello, (uint16_t)strlen(hello), 100);
        xSemaphoreGive(xUart2TxMutex);
    }
    printf("[ESP32] Sent startup probe on USART2 (%d bytes)\r\n",
           (int)strlen(hello));

    /* 诊断：打印 USART2 寄存器状态 */
    printf("[ESP32] USART2 SR=0x%04lX, CR1=0x%04lX, BRR=0x%04lX\r\n",
           huart2.Instance->SR,
           huart2.Instance->CR1,
           huart2.Instance->BRR);

    uint32_t rx_count = 0;
    uint32_t idle_count = 0;

    for (;;)
    {
        /* 使用超时等待（5秒），定期报告状态 */
        if (xSemaphoreTake(xEsp32Semaphore, pdMS_TO_TICKS(5000)) == pdPASS)
        {
            /* ISR 已切换缓冲，刚收完的帧在另一个 buffer */
            uint8_t  proc_idx = 1 - g_uart2_rx_buf_idx;
            uint16_t len      = g_uart2_rx_len;

            if (len == 0) continue;

            rx_count++;

            /* 帧重同步：扫描缓冲区找到第一个 SOF(0xAA) 字节。
             * 正常情况 sof_offset==0；若 ESP32 上一帧发送途中被心跳字节
             * "污染"（极端竞争条件），前几字节可能是残留的 ASCII 文本，
             * 跳过这些垃圾字节可以避免 OTA 状态机因非法 CMD 而进入错误分支。 */
            uint16_t sof_offset = 0;
            while (sof_offset < len && g_uart2_rx_buf[proc_idx][sof_offset] != OTA_SOF)
            {
                sof_offset++;
            }

            /* 检测 OTA 帧（以 SOF 0xAA 开头） */
            if (sof_offset < len && g_uart2_rx_buf[proc_idx][sof_offset] == OTA_SOF) {
                uint8_t  *frame = g_uart2_rx_buf[proc_idx] + sof_offset;
                uint16_t  flen  = len - sof_offset;

                if (sof_offset > 0) {
                    printf("[ESP32] WARN: skipped %d garbage byte(s) before SOF\r\n",
                           (int)sof_offset);
                }

                /* 诊断：在调用处理函数前，直接通过 USART2 发送原始确认，
                 * Python 可见，确认 STM32 已收到帧且 TX 路径正常 */
                char diag[48];
                int dn = snprintf(diag, sizeof(diag),
                                  "[DBG]RX:len=%d,cmd=0x%02X\r\n",
                                  flen, frame[1]);
                if (xSemaphoreTake(xUart2TxMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    HAL_UART_Transmit(&huart2, (uint8_t *)diag, (uint16_t)dn, 100);
                    xSemaphoreGive(xUart2TxMutex);
                }

                printf("[ESP32] OTA frame via MQTT: %d bytes\r\n", flen);
                ota_process_frame(frame, flen, &huart2);
            } else {
                /* 非 OTA 数据：日志 + 回复 OK（保持心跳兼容） */
                printf("[ESP32] %d bytes: ", len);
                uint16_t print_len = (len > 64) ? 64 : len;
                for (uint16_t i = 0; i < print_len; i++) {
                    uint8_t ch = g_uart2_rx_buf[proc_idx][i];
                    if (ch >= 0x20 || ch == '\r' || ch == '\n' || ch == '\t')
                        printf("%c", ch);
                    else
                        printf(".");
                }
                if (len > 64) printf("...");
                printf("\r\n");

                if (xSemaphoreTake(xUart2TxMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    HAL_UART_Transmit(&huart2, (uint8_t *)"OK\r\n", 4, 100);
                    xSemaphoreGive(xUart2TxMutex);
                }
            }
        }
        else
        {
            /* 超时：5秒内没收到任何数据，打印诊断信息 */
            idle_count++;
            printf("[ESP32] DIAG: no USART2 RX in 5s (idle=%lu, total_rx=%lu, "
                   "SR=0x%04lX, DMA_CNT=%lu)\r\n",
                   idle_count, rx_count,
                   huart2.Instance->SR,
                   (uint32_t)__HAL_DMA_GET_COUNTER(&hdma_usart2_rx));
        }
    }
}
