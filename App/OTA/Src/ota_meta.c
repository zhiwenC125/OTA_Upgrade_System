#include "ota_meta.h"
#include "w25qxx.h"
#include <string.h>

/* ============================================================================
 * 当前固件版本号 — 每次发布新固件时手动递增
 * Python sender 端 --fw-version 必须 >= 此值，否则 STM32 拒绝升级
 * ============================================================================ */
const uint8_t OTA_CURRENT_VERSION_MAJOR = 1;
const uint8_t OTA_CURRENT_VERSION_MINOR = 0;
const uint8_t OTA_CURRENT_VERSION_PATCH = 0;

int OTA_Meta_Read(OTA_Meta_t *meta)
{
    W25Qxx_Read(W25QXX_OTA_META_ADDR, (uint8_t *)meta, sizeof(OTA_Meta_t));
    if (meta->magic != OTA_META_MAGIC)
        return -1;
    return 0;
}

void OTA_Meta_Write(const OTA_Meta_t *meta)
{
    W25Qxx_EraseSector(W25QXX_OTA_META_ADDR);
    W25Qxx_Write(W25QXX_OTA_META_ADDR, (const uint8_t *)meta, sizeof(OTA_Meta_t));
}
