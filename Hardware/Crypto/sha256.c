#include "sha256.h"
#include <string.h>

/* ============================================================================
 * SHA-256 常量 K[64]（FIPS 180-4 Section 4.2.2）
 * ============================================================================ */
static const uint32_t K[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5,
    0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
    0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
    0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
    0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
    0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
    0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5,
    0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
    0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
};

/* ---- 位运算宏 ---- */
#define ROTR(x, n)   (((x) >> (n)) | ((x) << (32 - (n))))
#define CH(x, y, z)  (((x) & (y)) ^ (~(x) & (z)))
#define MAJ(x, y, z) (((x) & (y)) ^ ((x) & (z)) ^ ((y) & (z)))
#define EP0(x)        (ROTR(x, 2)  ^ ROTR(x, 13) ^ ROTR(x, 22))
#define EP1(x)        (ROTR(x, 6)  ^ ROTR(x, 11) ^ ROTR(x, 25))
#define SIG0(x)       (ROTR(x, 7)  ^ ROTR(x, 18) ^ ((x) >> 3))
#define SIG1(x)       (ROTR(x, 17) ^ ROTR(x, 19) ^ ((x) >> 10))

/* ============================================================================
 * 处理一个 64 字节 block
 * ============================================================================ */
static void sha256_transform(SHA256_CTX *ctx, const uint8_t block[64])
{
    uint32_t W[64];
    uint32_t a, b, c, d, e, f, g, h;
    uint32_t t1, t2;

    /* 准备消息调度 W[0..63] */
    for (int i = 0; i < 16; i++) {
        W[i] = ((uint32_t)block[i * 4]     << 24) |
               ((uint32_t)block[i * 4 + 1] << 16) |
               ((uint32_t)block[i * 4 + 2] <<  8) |
               ((uint32_t)block[i * 4 + 3]);
    }
    for (int i = 16; i < 64; i++) {
        W[i] = SIG1(W[i - 2]) + W[i - 7] + SIG0(W[i - 15]) + W[i - 16];
    }

    /* 初始化工作变量 */
    a = ctx->state[0]; b = ctx->state[1];
    c = ctx->state[2]; d = ctx->state[3];
    e = ctx->state[4]; f = ctx->state[5];
    g = ctx->state[6]; h = ctx->state[7];

    /* 64 轮压缩 */
    for (int i = 0; i < 64; i++) {
        t1 = h + EP1(e) + CH(e, f, g) + K[i] + W[i];
        t2 = EP0(a) + MAJ(a, b, c);
        h = g; g = f; f = e;
        e = d + t1;
        d = c; c = b; b = a;
        a = t1 + t2;
    }

    /* 累加 */
    ctx->state[0] += a; ctx->state[1] += b;
    ctx->state[2] += c; ctx->state[3] += d;
    ctx->state[4] += e; ctx->state[5] += f;
    ctx->state[6] += g; ctx->state[7] += h;
}

/* ============================================================================
 * SHA-256 公开接口
 * ============================================================================ */

void sha256_init(SHA256_CTX *ctx)
{
    ctx->datalen = 0;
    ctx->bitlen  = 0;
    ctx->state[0] = 0x6a09e667;
    ctx->state[1] = 0xbb67ae85;
    ctx->state[2] = 0x3c6ef372;
    ctx->state[3] = 0xa54ff53a;
    ctx->state[4] = 0x510e527f;
    ctx->state[5] = 0x9b05688c;
    ctx->state[6] = 0x1f83d9ab;
    ctx->state[7] = 0x5be0cd19;
}

void sha256_update(SHA256_CTX *ctx, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        ctx->data[ctx->datalen] = data[i];
        ctx->datalen++;
        if (ctx->datalen == 64) {
            sha256_transform(ctx, ctx->data);
            ctx->bitlen += 512;
            ctx->datalen = 0;
        }
    }
}

void sha256_final(SHA256_CTX *ctx, uint8_t hash[SHA256_DIGEST_SIZE])
{
    uint32_t i = ctx->datalen;

    /* 填充 0x80 + 若干 0x00 */
    ctx->data[i++] = 0x80;

    if (i > 56) {
        /* 当前 block 空间不够放 8 字节长度，需要多一个 block */
        while (i < 64)
            ctx->data[i++] = 0x00;
        sha256_transform(ctx, ctx->data);
        i = 0;
    }
    while (i < 56)
        ctx->data[i++] = 0x00;

    /* 追加消息总长度（bit，大端序 64 位） */
    ctx->bitlen += (uint64_t)ctx->datalen * 8;
    ctx->data[63] = (uint8_t)(ctx->bitlen);
    ctx->data[62] = (uint8_t)(ctx->bitlen >> 8);
    ctx->data[61] = (uint8_t)(ctx->bitlen >> 16);
    ctx->data[60] = (uint8_t)(ctx->bitlen >> 24);
    ctx->data[59] = (uint8_t)(ctx->bitlen >> 32);
    ctx->data[58] = (uint8_t)(ctx->bitlen >> 40);
    ctx->data[57] = (uint8_t)(ctx->bitlen >> 48);
    ctx->data[56] = (uint8_t)(ctx->bitlen >> 56);
    sha256_transform(ctx, ctx->data);

    /* 输出 hash（大端序） */
    for (i = 0; i < 8; i++) {
        hash[i * 4]     = (uint8_t)(ctx->state[i] >> 24);
        hash[i * 4 + 1] = (uint8_t)(ctx->state[i] >> 16);
        hash[i * 4 + 2] = (uint8_t)(ctx->state[i] >>  8);
        hash[i * 4 + 3] = (uint8_t)(ctx->state[i]);
    }
}

/* ============================================================================
 * HMAC-SHA256 一次性接口
 * ============================================================================ */

void hmac_sha256(const uint8_t *key, size_t key_len,
                 const uint8_t *data, size_t data_len,
                 uint8_t out[SHA256_DIGEST_SIZE])
{
    HMAC_SHA256_CTX ctx;
    hmac_sha256_init(&ctx, key, key_len);
    hmac_sha256_update(&ctx, data, data_len);
    hmac_sha256_final(&ctx, out);
}

/* ============================================================================
 * HMAC-SHA256 增量接口
 *
 * HMAC(K, m) = H((K' XOR opad) || H((K' XOR ipad) || m))
 *   K' = K if len<=64, else K' = H(K)
 *   ipad = 0x36 repeated, opad = 0x5c repeated
 * ============================================================================ */

void hmac_sha256_init(HMAC_SHA256_CTX *ctx,
                      const uint8_t *key, size_t key_len)
{
    uint8_t k_pad[SHA256_BLOCK_SIZE];

    /* 如果 key 超过 block size，先 hash 它 */
    if (key_len > SHA256_BLOCK_SIZE) {
        SHA256_CTX tmp;
        sha256_init(&tmp);
        sha256_update(&tmp, key, key_len);
        sha256_final(&tmp, k_pad);
        memset(k_pad + SHA256_DIGEST_SIZE, 0,
               SHA256_BLOCK_SIZE - SHA256_DIGEST_SIZE);
    } else {
        memcpy(k_pad, key, key_len);
        if (key_len < SHA256_BLOCK_SIZE)
            memset(k_pad + key_len, 0, SHA256_BLOCK_SIZE - key_len);
    }

    /* 计算 o_key_pad = k_pad XOR opad（保存供 final 使用） */
    for (size_t i = 0; i < SHA256_BLOCK_SIZE; i++) {
        ctx->o_key_pad[i] = k_pad[i] ^ 0x5C;
    }

    /* 内层：sha256_init + update(k_pad XOR ipad) */
    uint8_t i_key_pad[SHA256_BLOCK_SIZE];
    for (size_t i = 0; i < SHA256_BLOCK_SIZE; i++) {
        i_key_pad[i] = k_pad[i] ^ 0x36;
    }

    sha256_init(&ctx->inner);
    sha256_update(&ctx->inner, i_key_pad, SHA256_BLOCK_SIZE);
}

void hmac_sha256_update(HMAC_SHA256_CTX *ctx,
                        const uint8_t *data, size_t len)
{
    sha256_update(&ctx->inner, data, len);
}

void hmac_sha256_final(HMAC_SHA256_CTX *ctx,
                       uint8_t out[SHA256_DIGEST_SIZE])
{
    uint8_t inner_hash[SHA256_DIGEST_SIZE];

    /* 完成内层 hash */
    sha256_final(&ctx->inner, inner_hash);

    /* 外层：H(o_key_pad || inner_hash) */
    SHA256_CTX outer;
    sha256_init(&outer);
    sha256_update(&outer, ctx->o_key_pad, SHA256_BLOCK_SIZE);
    sha256_update(&outer, inner_hash, SHA256_DIGEST_SIZE);
    sha256_final(&outer, out);
}
