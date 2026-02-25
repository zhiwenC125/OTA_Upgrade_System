#ifndef __W25QXX_H
#define __W25QXX_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/* ============================================================================
 * W25Qxx SPI NOR Flash 驱动
 *
 * 支持型号：W25Q64 (8MB) / W25Q128 (16MB)
 *
 * 硬件连接（SPI1）：
 *   PA5 -> CLK     PA6 -> DO (MISO)
 *   PA7 -> DI (MOSI)   PA4 -> CS
 *
 * OTA 外部存储分区规划（方案 A — Download 放外部）：
 *  ┌──────────────────┐ 0x000000
 *  │  Download        │ 64KB (1 block)  新固件暂存区（App 接收后写入）
 *  ├──────────────────┤ 0x010000
 *  │  Backup          │ 64KB (1 block)  旧固件备份（回滚用，≥ App 55KB）
 *  ├──────────────────┤ 0x020000
 *  │  Meta            │  4KB (1 sector) 版本号/CRC32/固件大小
 *  ├──────────────────┤ 0x021000
 *  │  Free            │ ~3.8MB 剩余空间（日志/配置存储）
 *  └──────────────────┘
 *
 * 注：方案 A 下 App 占内部 Flash 55KB，Download 在外部 W25Q32。
 *     Bootloader 上电时从 W25Q32 Download 区读取固件，写入内部 App 区。
 * ============================================================================ */

/* ---------- CS 引脚配置 ---------- */
#define W25QXX_CS_PORT    GPIOA
#define W25QXX_CS_PIN     GPIO_PIN_4

#define W25QXX_CS_LOW()   HAL_GPIO_WritePin(W25QXX_CS_PORT, W25QXX_CS_PIN, GPIO_PIN_RESET)
#define W25QXX_CS_HIGH()  HAL_GPIO_WritePin(W25QXX_CS_PORT, W25QXX_CS_PIN, GPIO_PIN_SET)

/* ---------- W25Qxx SPI 指令集 ---------- */
#define W25X_WriteEnable        0x06
#define W25X_WriteDisable       0x04
#define W25X_ReadStatusReg1     0x05
#define W25X_ReadStatusReg2     0x35
#define W25X_WriteStatusReg     0x01
#define W25X_ReadData           0x03
#define W25X_FastReadData       0x0B
#define W25X_PageProgram        0x02
#define W25X_SectorErase        0x20    /* 4KB */
#define W25X_BlockErase32       0x52    /* 32KB */
#define W25X_BlockErase64       0xD8    /* 64KB */
#define W25X_ChipErase          0xC7
#define W25X_PowerDown          0xB9
#define W25X_ReleasePowerDown   0xAB
#define W25X_ManufactDeviceID   0x90
#define W25X_JedecDeviceID      0x9F

/* ---------- 已知芯片 ID ---------- */
#define W25Q32_ID               0xEF4016    /* W25Q32  4MB */
#define W25Q64_ID               0xEF4017    /* W25Q64  8MB */
#define W25Q128_ID              0xEF4018    /* W25Q128 16MB */

/* ---------- 芯片参数 ---------- */
#define W25QXX_PAGE_SIZE        256         /* 页大小（字节），编程单位 */
#define W25QXX_SECTOR_SIZE      4096        /* 扇区大小（字节），最小擦除单位 */
#define W25QXX_BLOCK_SIZE       65536       /* 块大小（字节） */

/* ---------- OTA 分区地址（外部 Flash 上的偏移，方案 A） ---------- */
#define W25QXX_OTA_DOWNLOAD_ADDR    0x000000    /* 新固件暂存区 */
#define W25QXX_OTA_DOWNLOAD_SIZE    0x010000    /* 64KB (≥ App 55KB) */
#define W25QXX_OTA_BACKUP_ADDR      0x010000    /* 旧固件备份区 */
#define W25QXX_OTA_BACKUP_SIZE      0x010000    /* 64KB */
#define W25QXX_OTA_META_ADDR        0x020000    /* 元信息区 */
#define W25QXX_OTA_META_SIZE        0x001000    /* 4KB (1 sector) */

/* ---------- 芯片信息结构 ---------- */
typedef struct {
    uint32_t jedec_id;          /* JEDEC ID */
    uint32_t capacity_bytes;    /* 容量（字节） */
    uint16_t page_count;        /* 总页数 (未使用，仅供参考) */
    uint16_t sector_count;      /* 总扇区数 */
} W25Qxx_Info_t;

extern W25Qxx_Info_t w25qxx_info;
extern SPI_HandleTypeDef hspi1;

/* ---------- API ---------- */

/**
 * @brief  初始化 SPI1 + CS 引脚 + 读取芯片 ID
 * @return 0=成功，-1=未检测到芯片
 */
int W25Qxx_Init(void);

/**
 * @brief  读取 JEDEC ID（Manufacturer + Type + Capacity）
 * @return 24-bit ID，如 0xEF4017 = W25Q64
 */
uint32_t W25Qxx_ReadJedecID(void);

/**
 * @brief  读取数据
 * @param  addr   24-bit 起始地址
 * @param  buf    目标缓冲区
 * @param  len    读取长度（无限制，可跨页/扇区/块）
 */
void W25Qxx_Read(uint32_t addr, uint8_t *buf, uint32_t len);

/**
 * @brief  页编程（写入最多 256 字节，不能跨页）
 * @param  addr   24-bit 起始地址（页内偏移）
 * @param  data   数据指针
 * @param  len    写入长度（≤256，且不能跨越页边界）
 */
void W25Qxx_WritePage(uint32_t addr, const uint8_t *data, uint16_t len);

/**
 * @brief  任意长度写入（自动分页，写入前需确保区域已擦除）
 * @param  addr   起始地址
 * @param  data   数据指针
 * @param  len    数据长度
 */
void W25Qxx_Write(uint32_t addr, const uint8_t *data, uint32_t len);

/**
 * @brief  擦除一个扇区（4KB）
 * @param  sector_addr  扇区内任意地址（自动对齐到 4KB 边界）
 */
void W25Qxx_EraseSector(uint32_t sector_addr);

/**
 * @brief  擦除一个块（64KB）
 * @param  block_addr  块内任意地址
 */
void W25Qxx_EraseBlock64(uint32_t block_addr);

/**
 * @brief  全片擦除（耗时较长，W25Q64 约 20-100s）
 */
void W25Qxx_EraseChip(void);

/**
 * @brief  等待芯片空闲（写入/擦除完成）
 */
void W25Qxx_WaitBusy(void);

#ifdef __cplusplus
}
#endif

#endif /* __W25QXX_H */
