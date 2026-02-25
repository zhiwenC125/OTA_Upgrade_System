#include "dht11.h"
#include "FreeRTOS.h"
#include "task.h"

/* ============================================================================
 * DWT 微秒延时（基于 Cortex-M3 Data Watchpoint and Trace 单元）
 * ============================================================================ */

void DHT11_DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U); /* 64MHz → 64 ticks/us */
    while ((DWT->CYCCNT - start) < ticks)
        ;
}

/* ============================================================================
 * GPIO 操作（PB1 开漏模式：写 1 = 释放总线，写 0 = 拉低）
 * ============================================================================ */

static inline void dht11_pin_low(void)
{
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);
}

static inline void dht11_pin_release(void)
{
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
}

static inline uint8_t dht11_pin_read(void)
{
    return (uint8_t)HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN);
}

/**
 * @brief 等待引脚变为指定电平，超时返回非零
 * @param level 目标电平 0 或 1
 * @param timeout_us 最大等待微秒
 */
static uint8_t wait_for_level(uint8_t level, uint32_t timeout_us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t limit = timeout_us * (SystemCoreClock / 1000000U);
    while (dht11_pin_read() != level) {
        if ((DWT->CYCCNT - start) > limit)
            return 1; /* 超时 */
    }
    return 0;
}

/* ============================================================================
 * DHT11 单总线读取
 *
 * 协议时序：
 *   1. 主机拉低 ≥18ms（起始信号）
 *   2. 主机释放，等待 DHT11 响应：低 80us + 高 80us
 *   3. 40 bit 数据，每 bit：低 50us + 高 26~28us(0) 或 70us(1)
 *   4. 校验和 = byte[0]+byte[1]+byte[2]+byte[3] 的低 8 位
 *
 * 整个过程在 taskENTER_CRITICAL() 中执行（~4ms），
 * 防止 FreeRTOS 上下文切换破坏微秒级时序。
 * ============================================================================ */

uint8_t DHT11_Read(DHT11_Data_t *data)
{
    uint8_t raw[5] = {0};

    /* ---- 起始信号（临界区外：18ms 拉低可被打断，不影响时序） ---- */
    dht11_pin_low();
    delay_us(18000); /* 18ms */
    dht11_pin_release();
    delay_us(30);    /* 等待 DHT11 响应 */

    /* ---- 进入临界区：40-bit 读取 ~4ms ---- */
    taskENTER_CRITICAL();

    /* 等待 DHT11 拉低（响应信号开始） */
    if (wait_for_level(0, 100) != 0) goto timeout;
    /* 等待 DHT11 拉高（响应信号低电平结束） */
    if (wait_for_level(1, 100) != 0) goto timeout;
    /* 等待 DHT11 拉低（响应信号高电平结束，数据传输开始） */
    if (wait_for_level(0, 100) != 0) goto timeout;

    /* 读取 40 bit */
    for (int i = 0; i < 40; i++) {
        /* 每 bit 以 ~50us 低电平开始 */
        if (wait_for_level(1, 80) != 0) goto timeout;

        /* 测量高电平持续时间：>40us 为 1，<40us 为 0 */
        uint32_t start = DWT->CYCCNT;
        if (wait_for_level(0, 100) != 0) goto timeout;
        uint32_t elapsed_us = (DWT->CYCCNT - start) / (SystemCoreClock / 1000000U);

        if (elapsed_us > 40) {
            raw[i / 8] |= (1 << (7 - (i % 8)));
        }
    }

    taskEXIT_CRITICAL();

    /* ---- 校验和验证 ---- */
    uint8_t cksum = raw[0] + raw[1] + raw[2] + raw[3];
    if (cksum != raw[4]) {
        return DHT11_ERR_CHECKSUM;
    }

    data->humidity_int = raw[0];
    data->humidity_dec = raw[1];
    data->temp_int     = raw[2];
    data->temp_dec     = raw[3];
    return DHT11_OK;

timeout:
    taskEXIT_CRITICAL();
    return DHT11_ERR_TIMEOUT;
}
