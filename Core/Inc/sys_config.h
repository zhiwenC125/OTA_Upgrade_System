#ifndef __SYS_CONFIG_H
#define __SYS_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/* LED 定义 */
#define LED_PIN GPIO_PIN_13
#define LED_PORT GPIOC

/* DHT11 引脚定义 */
#define DHT11_GPIO_PORT GPIOB
#define DHT11_GPIO_PIN  GPIO_PIN_1

/* 函数声明 */
void SystemClock_Config(void);
void GPIO_Init(void);

/* FreeRTOS 运行时统计（DWT 周期计数器） */
void vConfigureTimerForRunTimeStats(void);
uint32_t ulGetRunTimeCounterValue(void);

#ifdef __cplusplus
}
#endif

#endif /* __SYS_CONFIG_H */