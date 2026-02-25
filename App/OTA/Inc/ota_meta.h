#ifndef __OTA_META_H
#define __OTA_META_H

#include <stdint.h>

/* ============================================================================
 * OTA 固件元数据（存储在 W25Q32 Meta 扇区: 0x020000）
 *
 * 布局 (52 bytes):
 *   [magic:4][ver_major:1][ver_minor:1][ver_patch:1][meta_ver:1]
 *   [build_timestamp:4][fw_size:4][fw_crc32:4]
 *   [hmac_sha256:32]
 *
 * OTA END 校验通过后写入 Meta 区，Bootloader 拷贝前可读取日志。
 * App 启动时用编译期版本号做防回滚检查（START 阶段）。
 * ============================================================================ */

#define OTA_META_MAGIC          0x4D455441U  /* "META" little-endian */
#define OTA_META_VERSION        1U           /* 元数据格式版本 */

typedef struct __attribute__((packed)) {
    uint32_t magic;             /* 必须为 OTA_META_MAGIC */
    uint8_t  version_major;
    uint8_t  version_minor;
    uint8_t  version_patch;
    uint8_t  meta_version;      /* = OTA_META_VERSION */
    uint32_t build_timestamp;   /* Unix epoch (截断为 32-bit) */
    uint32_t fw_size;           /* 固件二进制大小（字节） */
    uint32_t fw_crc32;          /* 固件 CRC32 */
    uint8_t  hmac[32];          /* HMAC-SHA256 */
} OTA_Meta_t;                   /* 52 bytes total */

/* 版本号打包为单个 uint32 供比较: major*10000 + minor*100 + patch */
#define OTA_VERSION_TO_U32(maj, min, pat) \
    ((uint32_t)(maj) * 10000U + (uint32_t)(min) * 100U + (uint32_t)(pat))

/* 当前固件版本（定义在 ota_meta.c） */
extern const uint8_t OTA_CURRENT_VERSION_MAJOR;
extern const uint8_t OTA_CURRENT_VERSION_MINOR;
extern const uint8_t OTA_CURRENT_VERSION_PATCH;

/**
 * @brief  从 W25Q32 Meta 区读取元数据
 * @param  meta  输出结构体
 * @return 0=成功（magic 有效），-1=无效或为空
 */
int  OTA_Meta_Read(OTA_Meta_t *meta);

/**
 * @brief  将元数据写入 W25Q32 Meta 区（先擦后写）
 * @param  meta  待写入的元数据
 */
void OTA_Meta_Write(const OTA_Meta_t *meta);

#endif /* __OTA_META_H */
