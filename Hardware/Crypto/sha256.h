#ifndef __SHA256_H
#define __SHA256_H

#include <stdint.h>
#include <stddef.h>

/* ============================================================================
 * SHA-256 (FIPS 180-4) + HMAC-SHA256 (RFC 2104)
 *
 * 纯软件实现，适用于 STM32F103（Cortex-M3，无硬件加密加速器）。
 * ============================================================================ */

#define SHA256_BLOCK_SIZE   64U
#define SHA256_DIGEST_SIZE  32U

typedef struct {
    uint32_t state[8];              /* 中间 hash 值 (H0..H7) */
    uint8_t  data[SHA256_BLOCK_SIZE]; /* 当前未处理的数据块 */
    uint32_t datalen;               /* data[] 中已填充的字节数 */
    uint64_t bitlen;                /* 已处理的总比特数 */
} SHA256_CTX;

/* --- 基础 SHA-256 接口 --- */
void sha256_init(SHA256_CTX *ctx);
void sha256_update(SHA256_CTX *ctx, const uint8_t *data, size_t len);
void sha256_final(SHA256_CTX *ctx, uint8_t hash[SHA256_DIGEST_SIZE]);

/* --- HMAC-SHA256 一次性接口 --- */
void hmac_sha256(const uint8_t *key, size_t key_len,
                 const uint8_t *data, size_t data_len,
                 uint8_t out[SHA256_DIGEST_SIZE]);

/* --- HMAC-SHA256 增量接口（用于 OTA 逐帧计算） --- */
typedef struct {
    SHA256_CTX inner;               /* 内层 SHA256 上下文 */
    uint8_t    o_key_pad[SHA256_BLOCK_SIZE]; /* 外层 key XOR opad */
} HMAC_SHA256_CTX;

void hmac_sha256_init(HMAC_SHA256_CTX *ctx,
                      const uint8_t *key, size_t key_len);
void hmac_sha256_update(HMAC_SHA256_CTX *ctx,
                        const uint8_t *data, size_t len);
void hmac_sha256_final(HMAC_SHA256_CTX *ctx,
                       uint8_t out[SHA256_DIGEST_SIZE]);

#endif /* __SHA256_H */
