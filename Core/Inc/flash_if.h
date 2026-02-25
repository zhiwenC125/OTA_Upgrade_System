#ifndef __FLASH_IF_H
#define __FLASH_IF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/* ============================================================================
 * STM32F103C8T6 内部 Flash 分区规划 (64KB, 页大小 1KB)
 *
 * 【方案 A — 内部 Bootloader + 外部 Download】
 * STM32 内部实现 Bootloader，Download 区放在外部 W25Q32 SPI Flash。
 * App 获得 55KB 空间，不受 Download 区挤占。
 *
 * 内部 Flash:
 *  ┌────────────────┐ 0x08000000
 *  │  Bootloader    │  8KB  ( 8 pages)   ← 上电首先运行（含 SPI 驱动）
 *  ├────────────────┤ 0x08002000
 *  │  App (运行区)   │ 55KB  (55 pages)   ← 主业务代码（FreeRTOS）
 *  ├────────────────┤ 0x0800FC00
 *  │  OTA Flag      │  1KB  ( 1 page)    ← 升级标志
 *  └────────────────┘ 0x08010000
 *
 * 外部 W25Q32 (4MB):
 *  ┌────────────────┐ 0x000000
 *  │  Download      │ 64KB  ← 新固件暂存（App 接收后写入）
 *  ├────────────────┤ 0x010000
 *  │  Backup        │ 64KB  ← 旧固件备份（回滚用）
 *  ├────────────────┤ 0x020000
 *  │  Meta          │  4KB  ← 版本号/CRC32/固件大小
 *  ├────────────────┤ 0x021000
 *  │  Free          │ ~3.8MB
 *  └────────────────┘
 *
 * OTA 流程：
 *   1. 树莓派5 → (WiFi/MQTT) → ESP32：推送新固件
 *   2. ESP32 → UART → STM32 App：逐包传输固件数据
 *   3. STM32 App：CRC32 校验 + 写入 W25Q32 Download 区
 *   4. 写入 OTA_FLAG_UPGRADE_PENDING → 软复位
 *   5. Bootloader：检查标志 → 初始化 SPI → 从 W25Q32 读取 → 写入 App 区
 *      → 清标志 → 跳转 App
 * ============================================================================ */

/* Flash 基础参数（FLASH_PAGE_SIZE 已在 HAL 中定义为 0x400U，不重复定义） */
#define FLASH_IF_TOTAL_SIZE     ((uint32_t)0x10000)  /* 64KB */
#define FLASH_IF_BASE_ADDR      ((uint32_t)0x08000000)
#define FLASH_IF_END_ADDR       ((uint32_t)0x0800FFFF)

/* 内部 Flash 分区地址 */
#define FLASH_BOOT_ADDR         ((uint32_t)0x08000000)
#define FLASH_BOOT_SIZE         ((uint32_t)0x2000)   /* 8KB */

#define FLASH_APP_ADDR          ((uint32_t)0x08002000)
#define FLASH_APP_SIZE          ((uint32_t)0xDC00)   /* 55KB */

#define FLASH_FLAG_ADDR         ((uint32_t)0x0800FC00)
#define FLASH_FLAG_SIZE         FLASH_PAGE_SIZE      /* 1KB */

/* OTA 标志位魔数（写入 Flag 区第一个 word） */
#define OTA_FLAG_UPGRADE_PENDING  ((uint32_t)0xAA55AA55)  /* 有新固件待升级 */
#define OTA_FLAG_UPGRADE_DONE     ((uint32_t)0x55AA55AA)  /* 兼容旧版：升级已完成 */
#define OTA_FLAG_BOOT_COUNTING    ((uint32_t)0xC3A5C3A5)  /* 新固件已拷贝，等待 App 确认 */
#define OTA_FLAG_CONFIRMED        ((uint32_t)0x5A5AA5A5)  /* App 已确认正常，回滚计数清零 */
#define OTA_FLAG_EMPTY            ((uint32_t)0xFFFFFFFF)  /* 擦除后默认值，无升级 */

/* Flag 区第二个 word（offset +4）存储启动计数 */
#define OTA_BOOT_COUNT_MAX        3U                       /* 超过此次数触发回滚 */

/* 返回值 */
typedef enum {
    FLASH_IF_OK      = 0,
    FLASH_IF_ERASE_ERR,
    FLASH_IF_WRITE_ERR,
    FLASH_IF_ADDR_ERR,
    FLASH_IF_LOCK_ERR,
} Flash_If_Status;

/* ============================================================================
 * API
 * ============================================================================ */

/**
 * @brief  擦除指定地址开始的若干页
 * @param  start_addr  起始地址（必须页对齐）
 * @param  num_pages   要擦除的页数
 * @return Flash_If_Status
 */
Flash_If_Status Flash_If_Erase(uint32_t start_addr, uint32_t num_pages);

/**
 * @brief  向 Flash 写入数据（半字对齐，内部按 16-bit 写入）
 * @param  dest_addr   目标地址（必须半字对齐）
 * @param  src         数据源指针
 * @param  len         数据长度（字节，内部会向上对齐到偶数）
 * @return Flash_If_Status
 */
Flash_If_Status Flash_If_Write(uint32_t dest_addr, const uint8_t *src, uint32_t len);

/**
 * @brief  从 Flash 读取数据（直接内存映射读取）
 * @param  src_addr    源地址
 * @param  dest        目标缓冲区
 * @param  len         读取长度（字节）
 */
void Flash_If_Read(uint32_t src_addr, uint8_t *dest, uint32_t len);

/**
 * @brief  设置 OTA 升级标志（仅写 state word）
 * @param  flag  OTA_FLAG_UPGRADE_PENDING / OTA_FLAG_UPGRADE_DONE 等
 * @return Flash_If_Status
 */
Flash_If_Status Flash_If_SetFlag(uint32_t flag);

/**
 * @brief  设置 OTA 升级标志及启动计数（同时写 state + boot_count 两个 word）
 * @param  flag        OTA_FLAG_BOOT_COUNTING / OTA_FLAG_CONFIRMED 等
 * @param  boot_count  启动尝试次数（OTA_FLAG_BOOT_COUNTING 时有效）
 * @return Flash_If_Status
 */
Flash_If_Status Flash_If_SetFlagEx(uint32_t flag, uint32_t boot_count);

/**
 * @brief  读取 OTA 升级标志（state word）
 * @return 当前标志值
 */
uint32_t Flash_If_GetFlag(void);

/**
 * @brief  读取启动计数（Flag 区 offset+4 的 word）
 * @return 启动尝试次数
 */
uint32_t Flash_If_GetBootCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __FLASH_IF_H */
