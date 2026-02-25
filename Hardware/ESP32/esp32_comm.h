#ifndef __ESP32_COMM_H
#define __ESP32_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* ============================================================================
 * ESP32-S3 通信模块
 *
 * 通过 USART2 (PA2/PA3) 与 ESP32-S3 串口0 通信。
 * 接收到的数据通过 USART1 printf 转发到 PC 终端，方便调试。
 * 同时回显到 USART2，让 ESP32-S3 端也能看到确认。
 *
 * 测试方法：
 *   ESP32-S3 上电后会通过串口0输出启动日志（如 "ESP-ROM:..."）
 *   STM32 收到后会在 PC 串口终端显示：
 *     [ESP32] 42 bytes: ESP-ROM:esp32s3-2021...
 * ============================================================================ */

/* ESP32 通信信号量（定义在 main.c） */
extern SemaphoreHandle_t xEsp32Semaphore;

/**
 * @brief  ESP32 通信任务
 *
 * 启动 USART2 DMA 接收，等待 ESP32-S3 发来的数据，
 * 收到后通过 printf (USART1) 打印到 PC 终端。
 */
void vEsp32CommTask(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif /* __ESP32_COMM_H */
