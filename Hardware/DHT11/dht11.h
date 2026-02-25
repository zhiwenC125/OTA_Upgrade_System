#ifndef __DHT11_H
#define __DHT11_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* DHT11 引脚定义 */
#define DHT11_PORT  GPIOB
#define DHT11_PIN   GPIO_PIN_1

/* 返回码 */
#define DHT11_OK            0
#define DHT11_ERR_TIMEOUT   1
#define DHT11_ERR_CHECKSUM  2

/* 读取结果 */
typedef struct {
    uint8_t humidity_int;
    uint8_t humidity_dec;   /* DHT11 始终为 0，保留以兼容 DHT22 */
    uint8_t temp_int;
    uint8_t temp_dec;       /* DHT11 始终为 0 */
} DHT11_Data_t;

/**
 * @brief  使能 DWT 周期计数器（64MHz → 15.625ns 分辨率）
 *         必须在 SystemClock_Config() 之后、调度器启动之前调用
 */
void DHT11_DWT_Init(void);

/**
 * @brief  读取 DHT11 温湿度
 *
 * 内部使用 taskENTER_CRITICAL() 保护 ~4ms 的时序操作。
 * 调用间隔不应小于 1 秒（DHT11 硬件限制）。
 *
 * @param  data  输出结构体指针
 * @return DHT11_OK / DHT11_ERR_TIMEOUT / DHT11_ERR_CHECKSUM
 */
uint8_t DHT11_Read(DHT11_Data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __DHT11_H */
