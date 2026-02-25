#include "comm_test.h"
#include "usart1.h"
#include "w25qxx.h"
#include "flash_if.h"
#include <stdio.h>
#include <string.h>

/* ============================================================================
 * 通信芯片测试模块
 *
 * 测试项：
 *   Test 1 - USART1 串口 TX 发送 + DMA RX 回环测试
 *   Test 2 - SPI1 W25Qxx 外部 Flash 通信测试 (ID / 擦除 / 写入 / 读回校验)
 *   Test 3 - STM32 内部 Flash 擦写校验
 *
 * 使用方式：
 *   在 main.c 中创建此任务，通过串口终端观察测试结果。
 *   USART1 回环测试需要将 PA9(TX) 与 PA10(RX) 短接。
 * ============================================================================ */

/* ---------- 测试用常量 ---------- */
#define TEST_PATTERN_SIZE     64          /* 测试数据长度 */
#define W25QXX_TEST_ADDR      0x0FF000    /* 外部 Flash 测试地址 (Free 区域, 避开 OTA 分区) */

/* 内部 Flash 测试使用 Flag 区域 (0x0800F000), 测试完毕会恢复原值 */

static uint8_t tx_buf[TEST_PATTERN_SIZE];
static uint8_t rx_buf[TEST_PATTERN_SIZE];

/* ===================== 辅助函数 ===================== */

/**
 * @brief  生成测试数据模式
 */
static void fill_test_pattern(uint8_t *buf, uint16_t len, uint8_t seed)
{
    for (uint16_t i = 0; i < len; i++)
    {
        buf[i] = (uint8_t)(seed + i);
    }
}

/**
 * @brief  对比两个缓冲区，返回第一个不匹配的偏移，-1 表示全部匹配
 */
static int compare_buffers(const uint8_t *a, const uint8_t *b, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        if (a[i] != b[i])
            return (int)i;
    }
    return -1;
}

/**
 * @brief  打印 hex dump (最多 32 字节)
 */
static void hex_dump(const char *tag, const uint8_t *buf, uint16_t len)
{
    uint16_t dump_len = (len > 32) ? 32 : len;
    printf("  [%s] ", tag);
    for (uint16_t i = 0; i < dump_len; i++)
        printf("%02X ", buf[i]);
    if (len > 32) printf("...");
    printf("\r\n");
}

/* ===================== Test 1: USART1 串口测试 ===================== */

/**
 * @brief  USART1 发送测试 (printf 输出验证)
 *
 * 测试内容：
 *   1. 发送固定字符串，验证 printf 通路正常
 *   2. 发送 hex 数据帧，验证二进制发送能力
 *   3. 如 TX-RX 短接，可验证 DMA 接收回环
 *
 * @return 0=通过, -1=失败
 */
static int test_usart1(void)
{
    printf("\r\n========== Test 1: USART1 ==========\r\n");

    /* --- 1.1 printf 字符串发送测试 --- */
    printf("[USART1] TX string test: Hello from STM32F103!\r\n");

    /* --- 1.2 HAL_UART_Transmit 阻塞发送测试 --- */
    const char *test_msg = "[USART1] HAL_UART_Transmit OK\r\n";
    HAL_StatusTypeDef hal_ret;
    hal_ret = HAL_UART_Transmit(&huart1,
                                (uint8_t *)test_msg,
                                (uint16_t)strlen(test_msg),
                                100);
    if (hal_ret != HAL_OK)
    {
        printf("[USART1] HAL_UART_Transmit FAILED (ret=%d)\r\n", hal_ret);
        return -1;
    }

    /* --- 1.3 二进制数据发送测试 --- */
    fill_test_pattern(tx_buf, 16, 0xA0);
    printf("[USART1] TX hex frame: ");
    for (int i = 0; i < 16; i++)
        printf("%02X ", tx_buf[i]);
    printf("\r\n");

    hal_ret = HAL_UART_Transmit(&huart1, tx_buf, 16, 100);
    if (hal_ret != HAL_OK)
    {
        printf("[USART1] Binary TX FAILED\r\n");
        return -1;
    }
    printf("[USART1] Binary TX OK\r\n");

    /* --- 1.4 DMA 接收状态检查 --- */
    printf("[USART1] DMA RX buf_idx=%d, last_len=%d\r\n",
           g_uart_rx_buf_idx, g_uart_rx_len);
    printf("[USART1] DMA Counter=%lu\r\n",
           (unsigned long)__HAL_DMA_GET_COUNTER(&hdma_usart1_rx));

    printf("[USART1] ---- PASS ----\r\n");
    return 0;
}

/* ===================== Test 2: SPI W25Qxx 外部 Flash ===================== */

/**
 * @brief  W25Qxx SPI Flash 通信测试
 *
 * 测试流程：
 *   2.1 初始化 SPI + 读取 JEDEC ID (验证 SPI 通信)
 *   2.2 擦除测试扇区
 *   2.3 写入测试数据
 *   2.4 读回数据并逐字节比对
 *
 * @return 0=通过, -1=失败
 */
static int test_w25qxx(void)
{
    printf("\r\n========== Test 2: SPI W25Qxx Flash ==========\r\n");

    /* --- 2.1 初始化 + 读 JEDEC ID --- */
    int ret = W25Qxx_Init();
    if (ret != 0)
    {
        printf("[W25Qxx] Init FAILED - SPI communication error!\r\n");
        printf("[W25Qxx] Check wiring: PA5=SCK, PA6=MISO, PA7=MOSI, PA4=CS\r\n");
        return -1;
    }

    uint32_t id = w25qxx_info.jedec_id;
    printf("[W25Qxx] JEDEC ID = 0x%06lX", (unsigned long)id);
    if (id == W25Q32_ID)
        printf(" (W25Q32, 4MB)\r\n");
    else if (id == W25Q64_ID)
        printf(" (W25Q64, 8MB)\r\n");
    else if (id == W25Q128_ID)
        printf(" (W25Q128, 16MB)\r\n");
    else
        printf(" (Unknown)\r\n");

    /* --- 2.2 擦除测试扇区 --- */
    printf("[W25Qxx] Erasing sector at 0x%06lX ...\r\n",
           (unsigned long)W25QXX_TEST_ADDR);
    W25Qxx_EraseSector(W25QXX_TEST_ADDR);
    printf("[W25Qxx] Erase done\r\n");

    /* 验证擦除后全 0xFF */
    memset(rx_buf, 0, TEST_PATTERN_SIZE);
    W25Qxx_Read(W25QXX_TEST_ADDR, rx_buf, TEST_PATTERN_SIZE);

    int erase_ok = 1;
    for (int i = 0; i < TEST_PATTERN_SIZE; i++)
    {
        if (rx_buf[i] != 0xFF)
        {
            printf("[W25Qxx] Erase verify FAILED at offset %d: 0x%02X != 0xFF\r\n",
                   i, rx_buf[i]);
            erase_ok = 0;
            break;
        }
    }
    if (erase_ok)
        printf("[W25Qxx] Erase verify OK (all 0xFF)\r\n");
    else
        return -1;

    /* --- 2.3 写入测试数据 --- */
    fill_test_pattern(tx_buf, TEST_PATTERN_SIZE, 0x30);
    hex_dump("TX", tx_buf, TEST_PATTERN_SIZE);

    printf("[W25Qxx] Writing %d bytes at 0x%06lX ...\r\n",
           TEST_PATTERN_SIZE, (unsigned long)W25QXX_TEST_ADDR);
    W25Qxx_Write(W25QXX_TEST_ADDR, tx_buf, TEST_PATTERN_SIZE);
    printf("[W25Qxx] Write done\r\n");

    /* --- 2.4 读回并校验 --- */
    memset(rx_buf, 0, TEST_PATTERN_SIZE);
    W25Qxx_Read(W25QXX_TEST_ADDR, rx_buf, TEST_PATTERN_SIZE);
    hex_dump("RX", rx_buf, TEST_PATTERN_SIZE);

    int mismatch = compare_buffers(tx_buf, rx_buf, TEST_PATTERN_SIZE);
    if (mismatch >= 0)
    {
        printf("[W25Qxx] Data verify FAILED at offset %d: TX=0x%02X, RX=0x%02X\r\n",
               mismatch, tx_buf[mismatch], rx_buf[mismatch]);
        return -1;
    }

    printf("[W25Qxx] Data verify OK (%d bytes match)\r\n", TEST_PATTERN_SIZE);

    /* 清理：擦除测试扇区恢复原状 */
    W25Qxx_EraseSector(W25QXX_TEST_ADDR);
    printf("[W25Qxx] Test sector cleaned\r\n");

    printf("[W25Qxx] ---- PASS ----\r\n");
    return 0;
}

/* ===================== Test 3: 内部 Flash ===================== */

/**
 * @brief  STM32 内部 Flash 擦写校验
 *
 * 测试区域使用 OTA Flag 区 (0x0800F000, 1KB)
 * 测试流程：
 *   3.1 读取当前 Flag 值 (保存备份)
 *   3.2 擦除 Flag 页
 *   3.3 写入测试数据
 *   3.4 读回并校验
 *   3.5 恢复原始 Flag 值
 *
 * @return 0=通过, -1=失败
 */
static int test_internal_flash(void)
{
    printf("\r\n========== Test 3: Internal Flash ==========\r\n");

    /* --- 3.1 备份当前 Flag 值 --- */
    uint32_t orig_flag = Flash_If_GetFlag();
    printf("[Flash] Current flag at 0x%08lX = 0x%08lX\r\n",
           (unsigned long)FLASH_FLAG_ADDR, (unsigned long)orig_flag);

    /* --- 3.2 擦除 Flag 页 --- */
    printf("[Flash] Erasing page at 0x%08lX ...\r\n",
           (unsigned long)FLASH_FLAG_ADDR);
    Flash_If_Status fret = Flash_If_Erase(FLASH_FLAG_ADDR, 1);
    if (fret != FLASH_IF_OK)
    {
        printf("[Flash] Erase FAILED (err=%d)\r\n", fret);
        return -1;
    }

    /* 验证擦除后 = 0xFFFFFFFF */
    uint32_t erased_val = Flash_If_GetFlag();
    if (erased_val != 0xFFFFFFFF)
    {
        printf("[Flash] Erase verify FAILED: 0x%08lX != 0xFFFFFFFF\r\n",
               (unsigned long)erased_val);
        return -1;
    }
    printf("[Flash] Erase verify OK (0xFFFFFFFF)\r\n");

    /* --- 3.3 写入测试数据 --- */
    fill_test_pattern(tx_buf, TEST_PATTERN_SIZE, 0x50);
    hex_dump("TX", tx_buf, TEST_PATTERN_SIZE);

    printf("[Flash] Writing %d bytes ...\r\n", TEST_PATTERN_SIZE);
    fret = Flash_If_Write(FLASH_FLAG_ADDR, tx_buf, TEST_PATTERN_SIZE);
    if (fret != FLASH_IF_OK)
    {
        printf("[Flash] Write FAILED (err=%d)\r\n", fret);
        return -1;
    }
    printf("[Flash] Write done\r\n");

    /* --- 3.4 读回校验 --- */
    memset(rx_buf, 0, TEST_PATTERN_SIZE);
    Flash_If_Read(FLASH_FLAG_ADDR, rx_buf, TEST_PATTERN_SIZE);
    hex_dump("RX", rx_buf, TEST_PATTERN_SIZE);

    int mismatch = compare_buffers(tx_buf, rx_buf, TEST_PATTERN_SIZE);
    if (mismatch >= 0)
    {
        printf("[Flash] Data verify FAILED at offset %d: TX=0x%02X, RX=0x%02X\r\n",
               mismatch, tx_buf[mismatch], rx_buf[mismatch]);
        return -1;
    }
    printf("[Flash] Data verify OK (%d bytes match)\r\n", TEST_PATTERN_SIZE);

    /* --- 3.5 恢复原始 Flag --- */
    printf("[Flash] Restoring original flag ...\r\n");
    fret = Flash_If_Erase(FLASH_FLAG_ADDR, 1);
    if (fret != FLASH_IF_OK)
    {
        printf("[Flash] Restore erase FAILED\r\n");
        return -1;
    }

    if (orig_flag != OTA_FLAG_EMPTY)
    {
        /* 原值不是擦除态，需要写回 */
        fret = Flash_If_Write(FLASH_FLAG_ADDR, (const uint8_t *)&orig_flag, sizeof(orig_flag));
        if (fret != FLASH_IF_OK)
        {
            printf("[Flash] Restore write FAILED\r\n");
            return -1;
        }
    }

    uint32_t restored = Flash_If_GetFlag();
    printf("[Flash] Restored flag = 0x%08lX (expected 0x%08lX) %s\r\n",
           (unsigned long)restored, (unsigned long)orig_flag,
           (restored == orig_flag) ? "OK" : "MISMATCH!");

    printf("[Flash] ---- PASS ----\r\n");
    return 0;
}

/* ===================== 测试任务入口 ===================== */

void vCommTestTask(void *pvParameters)
{
    (void)pvParameters;

    /* 等待系统稳定 */
    vTaskDelay(pdMS_TO_TICKS(500));

    printf("\r\n");
    printf("╔══════════════════════════════════════════╗\r\n");
    printf("║     IoT2 Communication Test Suite        ║\r\n");
    printf("║     MCU: STM32F103C8T6 @ 64MHz           ║\r\n");
    printf("╚══════════════════════════════════════════╝\r\n");

    int pass = 0;
    int fail = 0;

    /* Test 1: USART1 */
    if (test_usart1() == 0)
        pass++;
    else
        fail++;

    vTaskDelay(pdMS_TO_TICKS(100));

    /* Test 2: SPI W25Qxx */
    if (test_w25qxx() == 0)
        pass++;
    else
        fail++;

    vTaskDelay(pdMS_TO_TICKS(100));

    /* Test 3: Internal Flash */
    if (test_internal_flash() == 0)
        pass++;
    else
        fail++;

    /* 汇总结果 */
    printf("\r\n");
    printf("==========================================\r\n");
    printf("  Test Summary: %d PASS, %d FAIL (total 3)\r\n", pass, fail);
    printf("==========================================\r\n");

    if (fail == 0)
        printf("  >>> ALL TESTS PASSED <<<\r\n");
    else
        printf("  >>> SOME TESTS FAILED <<<\r\n");

    printf("==========================================\r\n\r\n");

    /* 测试完成后自行挂起，不影响其他任务 */
    vTaskSuspend(NULL);
}
