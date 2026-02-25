#include "ota_task.h"
#include "ota_meta.h"
#include "sha256.h"
#include "usart1.h"
#include "flash_if.h"
#include "w25qxx.h"
#include "task.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* ============================================================================
 * OTA 状态机
 * ============================================================================ */

typedef enum {
    OTA_STATE_IDLE = 0,     /* 等待 START 命令 */
    OTA_STATE_STARTED,      /* START 已收到，Download 区已擦除，等待 DATA */
    OTA_STATE_RECEIVING,    /* 正在接收 DATA 包 */
    OTA_STATE_ERROR,        /* 协议错误，等待 ABORT 或新 START */
} OTA_State_t;

/* OTA 上下文（仅 .c 内可见） */
static OTA_State_t  s_state         = OTA_STATE_IDLE;
static uint32_t     s_fw_total_size = 0;        /* START 中声明的固件总大小 */
static uint32_t     s_fw_crc32      = 0;        /* START 中声明的期望 CRC32 */
static uint32_t     s_bytes_written = 0;        /* 已写入 W25Q32 的字节数 */
static uint16_t     s_next_seq      = 0;        /* 期望的下一个包序号 */
static uint32_t     s_running_crc   = 0xFFFFFFFFU;  /* 滚动 CRC32 累积值 */

/* HMAC-SHA256 固件签名验证 */
static const uint8_t s_ota_hmac_key[OTA_HMAC_KEY_SIZE] = {
    0x49, 0x6F, 0x54, 0x32, 0x2D, 0x4F, 0x54, 0x41,  /* "IoT2-OTA" */
    0x2D, 0x48, 0x4D, 0x41, 0x43, 0x2D, 0x4B, 0x65,  /* "-HMAC-Ke" */
    0x79, 0x2D, 0x32, 0x30, 0x32, 0x36, 0x00, 0x00,  /* "y-2026\0\0" */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
static HMAC_SHA256_CTX s_hmac_ctx;                     /* 增量 HMAC 上下文 */
static uint8_t         s_expected_hmac[SHA256_DIGEST_SIZE]; /* START 中携带的期望 HMAC */

/* 固件元数据（Phase 7: 防回滚 + 版本追踪） */
static uint8_t  s_new_ver[3];          /* START 中携带的新固件版本 [major, minor, patch] */
static uint32_t s_build_ts;            /* START 中携带的构建时间戳 */

/* ============================================================================
 * 工具函数：CRC16-CCITT（多项式 0x1021，初值 0xFFFF）
 *   覆盖范围：帧中 [CMD..PAYLOAD]，不含 SOF，不含 CRC16 自身
 * ============================================================================ */
static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x8000U) ?
                  (uint16_t)((crc << 1) ^ 0x1021U) :
                  (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* ============================================================================
 * 工具函数：CRC32（IEEE 802.3，反射多项式 0xEDB88320）
 *   用法：先用 0xFFFFFFFF 初始化，逐块调用，最后 XOR 0xFFFFFFFF 取反。
 *   即：final = crc32_update(0xFFFFFFFF, data, len) ^ 0xFFFFFFFF
 * ============================================================================ */
static uint32_t crc32_update(uint32_t crc, const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        crc ^= (uint32_t)data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc & 1U) ? (crc >> 1) ^ 0xEDB88320U : (crc >> 1);
        }
    }
    return crc;
}

/* ============================================================================
 * OTA 状态重置（错误或 ABORT 时调用）
 * ============================================================================ */
static void ota_reset(void)
{
    s_state         = OTA_STATE_IDLE;
    s_fw_total_size = 0;
    s_fw_crc32      = 0;
    s_bytes_written = 0;
    s_next_seq      = 0;
    s_running_crc   = 0xFFFFFFFFU;
}

/* ============================================================================
 * 响应输出辅助：printf → USART1(PC) + 可选回写 response_uart（ESP32→MQTT）
 * ============================================================================ */
static UART_HandleTypeDef *s_response_uart = NULL;

static void ota_printf(const char *fmt, ...)
{
    char tmp[128];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(tmp, sizeof(tmp), fmt, args);
    va_end(args);

    /* 始终输出到 USART1（PC 终端） */
    printf("%s", tmp);

    /* 若设置了 response_uart，也回传给 ESP32（转发到 MQTT status） */
    if (s_response_uart != NULL && n > 0) {
        HAL_UART_Transmit(s_response_uart, (uint8_t *)tmp,
                          (uint16_t)n, 100);
    }
}

/* ============================================================================
 * 帧解析与业务处理（公开函数，可从 USART1/USART2 任务调用）
 *   buf            : 指向刚收完的 DMA 缓冲区
 *   len            : 本帧字节数（由 ISR 通过 DMA 计数器计算）
 *   response_uart  : 若非 NULL，响应文本也发送到该 UART
 * ============================================================================ */
void ota_process_frame(const uint8_t *buf, uint16_t len,
                       UART_HandleTypeDef *response_uart)
{
    /* 互斥保护：防止 USART1/USART2 两路同时操作状态机 */
    if (xSemaphoreTake(xOtaMutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        printf("[OTA] ERR: mutex timeout, frame dropped\r\n");
        return;
    }
    s_response_uart = response_uart;

    /* ---- 1. 最小帧长检查 ---- */
    if (len < OTA_FRAME_OVERHEAD) {
        ota_printf("[OTA] ERR: frame too short (%d bytes)\r\n", len);
        goto cleanup;
    }

    /* ---- 2. SOF 检查 ---- */
    if (buf[0] != OTA_SOF) {
        ota_printf("[OTA] ERR: bad SOF 0x%02X\r\n", buf[0]);
        goto cleanup;
    }

    /* ---- 3. 解包头部字段 ---- */
    uint8_t  cmd     = buf[1];
    uint16_t seq     = ((uint16_t)buf[2] << 8) | (uint16_t)buf[3];
    uint16_t pay_len = ((uint16_t)buf[4] << 8) | (uint16_t)buf[5];

    /* ---- 4. 帧长一致性检查 ---- */
    if ((uint16_t)(OTA_FRAME_OVERHEAD + pay_len) != len) {
        ota_printf("[OTA] ERR: len mismatch (hdr says %d, got %d)\r\n",
                   OTA_FRAME_OVERHEAD + pay_len, len);
        goto cleanup;
    }

    /* ---- 5. CRC16 校验（覆盖 [CMD..PAYLOAD]，即 buf[1..len-3]） ---- */
    uint16_t rx_crc   = ((uint16_t)buf[len - 2] << 8) | (uint16_t)buf[len - 1];
    uint16_t calc_crc = crc16_ccitt(&buf[1], (uint16_t)(len - 3));
    if (rx_crc != calc_crc) {
        ota_printf("[OTA] NACK seq=%u CRC16 err (rx=%04X calc=%04X)\r\n",
                   seq, rx_crc, calc_crc);
        goto cleanup;
    }

    const uint8_t *payload = &buf[OTA_FRAME_HDR];

    /* ---- 6. 命令分发 ---- */
    switch (cmd)
    {
    /* ------------------------------------------------------------------ */
    case CMD_OTA_START:
    {
        if (pay_len != OTA_START_PAYLOAD_SIZE) {
            ota_printf("[OTA] ERR: START needs %d-byte payload, got %d\r\n",
                       OTA_START_PAYLOAD_SIZE, pay_len);
            goto cleanup;
        }
        uint32_t fw_size = ((uint32_t)payload[0] << 24) |
                           ((uint32_t)payload[1] << 16) |
                           ((uint32_t)payload[2] <<  8) |
                            (uint32_t)payload[3];
        uint32_t fw_crc  = ((uint32_t)payload[4] << 24) |
                           ((uint32_t)payload[5] << 16) |
                           ((uint32_t)payload[6] <<  8) |
                            (uint32_t)payload[7];

        /* 提取 HMAC-SHA256 签名（32 字节，payload[8..39]） */
        memcpy(s_expected_hmac, &payload[8], SHA256_DIGEST_SIZE);

        /* 提取固件版本和构建时间戳（payload[40..51]，Phase 7 新增） */
        s_new_ver[0] = payload[40];  /* major */
        s_new_ver[1] = payload[41];  /* minor */
        s_new_ver[2] = payload[42];  /* patch */
        /* payload[43] reserved */
        s_build_ts   = ((uint32_t)payload[44] << 24) |
                       ((uint32_t)payload[45] << 16) |
                       ((uint32_t)payload[46] <<  8) |
                        (uint32_t)payload[47];

        /* 防回滚检查：新版本必须 >= 当前运行版本 */
        {
            uint32_t new_v = OTA_VERSION_TO_U32(s_new_ver[0], s_new_ver[1], s_new_ver[2]);
            uint32_t cur_v = OTA_VERSION_TO_U32(OTA_CURRENT_VERSION_MAJOR,
                                                OTA_CURRENT_VERSION_MINOR,
                                                OTA_CURRENT_VERSION_PATCH);
            if (new_v < cur_v) {
                ota_printf("[OTA] ERR: anti-rollback! new=%u.%u.%u < current=%u.%u.%u\r\n",
                           s_new_ver[0], s_new_ver[1], s_new_ver[2],
                           OTA_CURRENT_VERSION_MAJOR, OTA_CURRENT_VERSION_MINOR,
                           OTA_CURRENT_VERSION_PATCH);
                goto cleanup;
            }
        }

        if (fw_size == 0U || fw_size > W25QXX_OTA_DOWNLOAD_SIZE) {
            ota_printf("[OTA] ERR: invalid fw_size=%lu (max=%lu)\r\n",
                       fw_size, (uint32_t)W25QXX_OTA_DOWNLOAD_SIZE);
            goto cleanup;
        }

        /* 通知传感器任务暂停 USART2 TX */
        {
            extern volatile uint8_t g_ota_session_active;
            g_ota_session_active = 1;
        }

        ota_printf("[OTA] START: fw_size=%lu, CRC32=0x%08lX, ver=%u.%u.%u\r\n",
                   fw_size, fw_crc, s_new_ver[0], s_new_ver[1], s_new_ver[2]);

        /* ---- 备份当前 App 到 W25Q32 Backup 区（供回滚用，约 2s） ---- */
        ota_printf("[OTA] Backing up current App (55KB) to W25Q32 Backup...\r\n");
        W25Qxx_EraseBlock64(W25QXX_OTA_BACKUP_ADDR);
        {
            uint8_t  bak_buf[256];
            uint32_t bak_offset = 0U;
            while (bak_offset < FLASH_APP_SIZE) {
                uint32_t chunk = ((FLASH_APP_SIZE - bak_offset) > 256U)
                                  ? 256U : (FLASH_APP_SIZE - bak_offset);
                Flash_If_Read(FLASH_APP_ADDR + bak_offset, bak_buf, chunk);
                W25Qxx_Write(W25QXX_OTA_BACKUP_ADDR + bak_offset, bak_buf, chunk);
                bak_offset += chunk;
            }
        }
        ota_printf("[OTA] Backup OK\r\n");

        /* ---- 擦除 W25Q32 Download 区（64KB block，约 1.5s） ---- */
        W25Qxx_EraseBlock64(W25QXX_OTA_DOWNLOAD_ADDR);

        /* 初始化上下文 */
        s_fw_total_size = fw_size;
        s_fw_crc32      = fw_crc;
        s_bytes_written = 0;
        s_next_seq      = 0;
        s_running_crc   = 0xFFFFFFFFU;
        s_state         = OTA_STATE_STARTED;

        /* 初始化增量 HMAC-SHA256 上下文（DATA 阶段逐帧 update） */
        hmac_sha256_init(&s_hmac_ctx, s_ota_hmac_key, OTA_HMAC_KEY_SIZE);

        ota_printf("[OTA] Erase done\r\n");
        ota_printf("[OTA] READY\r\n");
        break;
    }

    /* ------------------------------------------------------------------ */
    case CMD_OTA_DATA:
    {
        if (s_state != OTA_STATE_STARTED && s_state != OTA_STATE_RECEIVING) {
            ota_printf("[OTA] ERR: DATA before START\r\n");
            goto cleanup;
        }
        if (pay_len == 0U) {
            ota_printf("[OTA] ERR: DATA with empty payload\r\n");
            goto cleanup;
        }
        /* 序号检查（防丢包/乱序） */
        if (seq != s_next_seq) {
            ota_printf("[OTA] NACK seq=%u (expected %u)\r\n", seq, s_next_seq);
            s_state = OTA_STATE_ERROR;
            goto cleanup;
        }
        /* 溢出检查 */
        if (s_bytes_written + (uint32_t)pay_len > s_fw_total_size) {
            ota_printf("[OTA] ERR: overflow (written=%lu + len=%u > total=%lu)\r\n",
                       s_bytes_written, pay_len, s_fw_total_size);
            s_state = OTA_STATE_ERROR;
            goto cleanup;
        }

        /* 写入 W25Q32 Download 区 */
        W25Qxx_Write(W25QXX_OTA_DOWNLOAD_ADDR + s_bytes_written,
                     payload, (uint32_t)pay_len);

        /* 更新滚动 CRC32 */
        s_running_crc = crc32_update(s_running_crc, payload, (uint32_t)pay_len);

        /* 更新滚动 HMAC-SHA256 */
        hmac_sha256_update(&s_hmac_ctx, payload, (size_t)pay_len);

        s_bytes_written += (uint32_t)pay_len;
        s_next_seq++;
        s_state = OTA_STATE_RECEIVING;

        ota_printf("[OTA] ACK seq=%u (%lu/%lu bytes)\r\n",
                   seq, s_bytes_written, s_fw_total_size);
        break;
    }

    /* ------------------------------------------------------------------ */
    case CMD_OTA_END:
    {
        if (s_state != OTA_STATE_RECEIVING) {
            ota_printf("[OTA] ERR: END without receiving data\r\n");
            goto cleanup;
        }
        /* 大小检查 */
        if (s_bytes_written != s_fw_total_size) {
            ota_printf("[OTA] ERR: incomplete (written=%lu, expect=%lu)\r\n",
                       s_bytes_written, s_fw_total_size);
            ota_reset();
            goto cleanup;
        }
        /* CRC32 最终校验 */
        uint32_t final_crc = s_running_crc ^ 0xFFFFFFFFU;
        if (final_crc != s_fw_crc32) {
            ota_printf("[OTA] ERR: CRC32 mismatch (calc=0x%08lX, expect=0x%08lX)\r\n",
                       final_crc, s_fw_crc32);
            ota_reset();
            goto cleanup;
        }

        /* HMAC-SHA256 签名校验 */
        {
            uint8_t computed_hmac[SHA256_DIGEST_SIZE];
            hmac_sha256_final(&s_hmac_ctx, computed_hmac);
            if (memcmp(computed_hmac, s_expected_hmac, SHA256_DIGEST_SIZE) != 0) {
                ota_printf("[OTA] ERR: HMAC-SHA256 mismatch! Firmware rejected.\r\n");
                ota_printf("[OTA] Expected: %02X%02X%02X%02X...  Computed: %02X%02X%02X%02X...\r\n",
                           s_expected_hmac[0], s_expected_hmac[1],
                           s_expected_hmac[2], s_expected_hmac[3],
                           computed_hmac[0], computed_hmac[1],
                           computed_hmac[2], computed_hmac[3]);
                ota_reset();
                goto cleanup;
            }
            ota_printf("[OTA] HMAC-SHA256 OK\r\n");
        }

        /* 恢复传感器上报（虽然即将 SystemReset，但保持一致性） */
        {
            extern volatile uint8_t g_ota_session_active;
            g_ota_session_active = 0;
        }

        ota_printf("[OTA] CRC32 + HMAC OK (0x%08lX). Writing upgrade flag...\r\n", final_crc);

        /* 写固件元数据到 W25Q32 Meta 区（Phase 7: 版本追踪） */
        {
            OTA_Meta_t meta;
            meta.magic          = OTA_META_MAGIC;
            meta.version_major  = s_new_ver[0];
            meta.version_minor  = s_new_ver[1];
            meta.version_patch  = s_new_ver[2];
            meta.meta_version   = OTA_META_VERSION;
            meta.build_timestamp = s_build_ts;
            meta.fw_size        = s_fw_total_size;
            meta.fw_crc32       = s_fw_crc32;
            memcpy(meta.hmac, s_expected_hmac, SHA256_DIGEST_SIZE);
            OTA_Meta_Write(&meta);
            ota_printf("[OTA] Meta written (ver=%u.%u.%u)\r\n",
                       s_new_ver[0], s_new_ver[1], s_new_ver[2]);
        }

        /* 写升级标志：UPGRADE_PENDING + boot_count=0（Bootloader 将改为 BOOT_COUNTING(1)） */
        if (Flash_If_SetFlagEx(OTA_FLAG_UPGRADE_PENDING, 0U) != FLASH_IF_OK) {
            ota_printf("[OTA] ERR: failed to set OTA flag\r\n");
            ota_reset();
            goto cleanup;
        }

        ota_printf("[OTA] DONE. Resetting in 1s...\r\n");

        /* 释放 mutex 后再延时复位，避免长时间持锁 */
        s_response_uart = NULL;
        xSemaphoreGive(xOtaMutex);
        vTaskDelay(pdMS_TO_TICKS(1000));

        /* 软复位 → Bootloader 检查标志 → 拷贝固件 → 跳转 App */
        HAL_NVIC_SystemReset();
        return;  /* 不会执行到此处 */
    }

    /* ------------------------------------------------------------------ */
    case CMD_OTA_ABORT:
    {
        extern volatile uint8_t g_ota_session_active;
        g_ota_session_active = 0;
        ota_printf("[OTA] ABORT received. Resetting OTA state.\r\n");
        ota_reset();
        break;
    }

    /* ------------------------------------------------------------------ */
    default:
        ota_printf("[OTA] ERR: unknown CMD 0x%02X\r\n", cmd);
        break;
    }

cleanup:
    s_response_uart = NULL;
    xSemaphoreGive(xOtaMutex);
}

/* ============================================================================
 * OTA 任务主体
 *   - 启动 USART1 DMA 接收
 *   - 等待 ISR 释放信号量（每帧一次）
 *   - 处理完整帧
 * ============================================================================ */
void vOtaProcessTask(void *pvParameters)
{
    (void)pvParameters;

    /*
     * 信号量已在 main.c 中创建（调度器启动前），
     * 在这里启动 DMA 接收，使用双缓冲的当前索引。
     */
    HAL_UART_Receive_DMA(&huart1,
                         g_uart_rx_buf[g_uart_rx_buf_idx],
                         UART_DMA_BUF_SIZE);

    /* 若 Bootloader 刚完成 OTA 拷贝（BOOT_COUNTING），App 首次成功启动则确认 */
    if (Flash_If_GetFlag() == OTA_FLAG_BOOT_COUNTING) {
        Flash_If_SetFlagEx(OTA_FLAG_CONFIRMED, 0U);
        printf("[OTA] New firmware confirmed OK. Rollback watchdog cleared.\r\n");
    }

    printf("[OTA] Task started v4.1 (FW %u.%u.%u). Waiting for firmware...\r\n",
           OTA_CURRENT_VERSION_MAJOR, OTA_CURRENT_VERSION_MINOR, OTA_CURRENT_VERSION_PATCH);

    for (;;)
    {
        if (xSemaphoreTake(xOtaSemaphore, portMAX_DELAY) == pdPASS)
        {
            /*
             * ISR 已完成：停 DMA → 算长度 → 切换缓冲索引 → 在新 buf 上重启 DMA
             * 所以刚收完的帧在 buf[1 - g_uart_rx_buf_idx]，可以安全读取。
             */
            uint8_t  proc_idx = 1U - g_uart_rx_buf_idx;
            uint16_t len      = g_uart_rx_len;

            if (len > 0U) {
                ota_process_frame(g_uart_rx_buf[proc_idx], len, NULL);
            }
        }
    }
}
