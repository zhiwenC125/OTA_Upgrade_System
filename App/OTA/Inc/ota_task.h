#ifndef __OTA_TASK_H
#define __OTA_TASK_H

#include "FreeRTOS.h"
#include "semphr.h"
#include "stm32f1xx_hal.h"

/* OTA 信号量（定义在 main.c，ISR 和任务共用） */
extern SemaphoreHandle_t xOtaSemaphore;

/* OTA 互斥锁（保护状态机，允许 USART1/USART2 双通道安全访问） */
extern SemaphoreHandle_t xOtaMutex;

/* ============================================================================
 * OTA UART 协议帧格式（IDLE 检测边界，单帧最大 256 字节）
 *
 *  [SOF:1][CMD:1][SEQ_H:1][SEQ_L:1][LEN_H:1][LEN_L:1][PAYLOAD:LEN][CRC16_H:1][CRC16_L:1]
 *
 *  SOF   = 0xAA
 *  CMD   : 见下方定义
 *  SEQ   : 包序号，大端序，从 0 开始递增
 *  LEN   : PAYLOAD 字节数，大端序，最大 248
 *  CRC16 : CRC16-CCITT，覆盖 [CMD..PAYLOAD]（不含 SOF，不含 CRC16 自身）
 *
 *  START 载荷 (52 bytes): [fw_size:4 大端][fw_crc32:4 大端][hmac_sha256:32][ver:3][rsv:1][timestamp:4 大端]
 *  DATA  载荷 (1~248 bytes) : 固件原始数据
 *  END   载荷 : 空（LEN=0）
 *  ABORT 载荷 : 空（LEN=0）
 *
 * OTA 流程：
 *   发送方: START → DATA×N → END
 *   STM32 : 擦 Download 区 → 逐包写 W25Q32 → 滚动 CRC32 + 滚动 HMAC-SHA256 →
 *           END 时校验 CRC32 → 校验 HMAC → 设 OTA_FLAG_UPGRADE_PENDING → SystemReset()
 * ============================================================================ */

#define OTA_SOF              0xAAU
#define OTA_FRAME_HDR        6U     /* SOF+CMD+SEQ(2)+LEN(2) */
#define OTA_FRAME_CRC        2U
#define OTA_FRAME_OVERHEAD   (OTA_FRAME_HDR + OTA_FRAME_CRC)   /* 8 bytes */
#define OTA_MAX_PAYLOAD      248U   /* 256 - 8 */

#define CMD_OTA_START        0x01U
#define CMD_OTA_DATA         0x02U
#define CMD_OTA_END          0x03U
#define CMD_OTA_ABORT        0x04U

/* HMAC-SHA256 固件签名相关 */
#define OTA_HMAC_KEY_SIZE        32U
#define OTA_START_PAYLOAD_SIZE   48U   /* fw_size(4) + fw_crc32(4) + hmac(32) + ver(3) + rsv(1) + timestamp(4) */

void vOtaProcessTask(void *pvParameters);

/**
 * @brief  处理一帧完整的 OTA 协议数据（可从任意任务调用）
 *
 * @param buf            完整帧缓冲区（以 SOF 开头）
 * @param len            帧长度（字节）
 * @param response_uart  若非 NULL，响应文本也发送到该 UART（用于 ESP32→MQTT 回传）
 *
 * 线程安全：内部使用 xOtaMutex 互斥。
 */
void ota_process_frame(const uint8_t *buf, uint16_t len,
                       UART_HandleTypeDef *response_uart);

#endif /* __OTA_TASK_H */
