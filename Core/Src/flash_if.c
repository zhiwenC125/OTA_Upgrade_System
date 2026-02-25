#include "flash_if.h"
#include <string.h>

/* ============================================================================
 * STM32F103 内部 Flash 驱动
 *
 * 关键特性（中密度系列）：
 *   - 页大小：1KB（0x400）
 *   - 编程粒度：半字（16-bit / 2 字节）
 *   - 擦除粒度：页（1KB）
 *   - 擦除后值：0xFF（即 word = 0xFFFFFFFF）
 *   - 写入前必须先擦除（只能从 1→0，不能从 0→1）
 * ============================================================================ */

/**
 * @brief  地址合法性检查
 */
static inline int Flash_If_AddrValid(uint32_t addr, uint32_t len)
{
    return (addr >= FLASH_IF_BASE_ADDR)
        && ((addr + len) <= (FLASH_IF_END_ADDR + 1));
}

/**
 * @brief  擦除指定地址开始的若干页
 */
Flash_If_Status Flash_If_Erase(uint32_t start_addr, uint32_t num_pages)
{
    /* 地址必须页对齐 */
    if ((start_addr % FLASH_PAGE_SIZE) != 0)
        return FLASH_IF_ADDR_ERR;

    if (!Flash_If_AddrValid(start_addr, num_pages * FLASH_PAGE_SIZE))
        return FLASH_IF_ADDR_ERR;

    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error = 0;

    /* 解锁 Flash */
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK)
        return FLASH_IF_LOCK_ERR;

    /* 配置擦除参数 */
    erase_init.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase_init.PageAddress = start_addr;
    erase_init.NbPages     = num_pages;

    /* 执行擦除 */
    status = HAL_FLASHEx_Erase(&erase_init, &page_error);

    /* 锁定 Flash */
    HAL_FLASH_Lock();

    if (status != HAL_OK)
        return FLASH_IF_ERASE_ERR;

    return FLASH_IF_OK;
}

/**
 * @brief  向 Flash 写入数据（按半字写入）
 *
 * STM32F103 内部 Flash 只支持半字（16-bit）编程。
 * 如果 len 为奇数，最后一个字节会补 0xFF 凑成半字写入。
 */
Flash_If_Status Flash_If_Write(uint32_t dest_addr, const uint8_t *src, uint32_t len)
{
    /* 地址必须半字对齐（2 字节） */
    if ((dest_addr % 2) != 0)
        return FLASH_IF_ADDR_ERR;

    if (!Flash_If_AddrValid(dest_addr, len))
        return FLASH_IF_ADDR_ERR;

    if (len == 0)
        return FLASH_IF_OK;

    HAL_StatusTypeDef status;

    status = HAL_FLASH_Unlock();
    if (status != HAL_OK)
        return FLASH_IF_LOCK_ERR;

    Flash_If_Status ret = FLASH_IF_OK;
    uint32_t addr = dest_addr;
    uint32_t i = 0;

    /* 每次写入一个半字（16-bit） */
    while (i < len)
    {
        uint16_t halfword;

        if (i + 1 < len)
        {
            /* 正常：两个字节组成半字（小端序） */
            halfword = (uint16_t)src[i] | ((uint16_t)src[i + 1] << 8);
            i += 2;
        }
        else
        {
            /* 最后一个奇数字节：高字节补 0xFF */
            halfword = (uint16_t)src[i] | 0xFF00;
            i += 1;
        }

        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, halfword);
        if (status != HAL_OK)
        {
            ret = FLASH_IF_WRITE_ERR;
            break;
        }

        addr += 2;
    }

    HAL_FLASH_Lock();
    return ret;
}

/**
 * @brief  从 Flash 读取数据
 *
 * STM32 内部 Flash 是内存映射的，可以直接读取。
 */
void Flash_If_Read(uint32_t src_addr, uint8_t *dest, uint32_t len)
{
    memcpy(dest, (const void *)src_addr, len);
}

/**
 * @brief  设置 OTA 升级标志
 *
 * 先擦除 Flag 页，再写入标志值到第一个 word（4 字节）。
 */
Flash_If_Status Flash_If_SetFlag(uint32_t flag)
{
    Flash_If_Status ret;

    /* 擦除 Flag 区（1 页） */
    ret = Flash_If_Erase(FLASH_FLAG_ADDR, 1);
    if (ret != FLASH_IF_OK)
        return ret;

    /* 写入 4 字节标志 */
    ret = Flash_If_Write(FLASH_FLAG_ADDR, (const uint8_t *)&flag, sizeof(flag));
    return ret;
}

/**
 * @brief  读取 OTA 升级标志
 */
uint32_t Flash_If_GetFlag(void)
{
    return *(__IO uint32_t *)FLASH_FLAG_ADDR;
}

/**
 * @brief  设置 OTA 升级标志及启动计数
 *
 * 先擦除 Flag 页，再写入 state（word0）和 boot_count（word1）。
 */
Flash_If_Status Flash_If_SetFlagEx(uint32_t flag, uint32_t boot_count)
{
    Flash_If_Status ret;

    ret = Flash_If_Erase(FLASH_FLAG_ADDR, 1);
    if (ret != FLASH_IF_OK)
        return ret;

    ret = Flash_If_Write(FLASH_FLAG_ADDR,
                         (const uint8_t *)&flag, sizeof(flag));
    if (ret != FLASH_IF_OK)
        return ret;

    ret = Flash_If_Write(FLASH_FLAG_ADDR + 4U,
                         (const uint8_t *)&boot_count, sizeof(boot_count));
    return ret;
}

/**
 * @brief  读取启动计数（Flag 区 offset+4 的 word）
 */
uint32_t Flash_If_GetBootCount(void)
{
    return *(__IO uint32_t *)(FLASH_FLAG_ADDR + 4U);
}
