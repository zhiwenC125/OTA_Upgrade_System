#ifndef __COMM_TEST_H
#define __COMM_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "task.h"

/**
 * @brief  通信测试任务入口
 *
 * 依次运行以下测试：
 *   1. USART1 串口发送/DMA 接收回环测试
 *   2. SPI1 W25Qxx 外部 Flash 读写校验
 *   3. STM32 内部 Flash 擦写校验
 *
 * 测试结果通过 printf (USART1) 输出到串口终端。
 */
void vCommTestTask(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif /* __COMM_TEST_H */
