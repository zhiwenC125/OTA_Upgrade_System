/**
 * ESP32-S3 Gateway — UART 通信模块
 *
 * 通过 UART 与 STM32F103 通信：
 *   ESP32 TX → STM32 PA3 (USART2 RX)
 *   ESP32 RX ← STM32 PA2 (USART2 TX)
 *
 * 功能：
 *   1. 每 2 秒发送心跳消息给 STM32
 *   2. 接收 STM32 的回复并打印到 ESP32 控制台
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "wifi_station.h"
#include "mqtt_client_task.h"

/* ======================== 配置 ======================== */

/* UART 端口号（UART1，因为 UART0 是 ESP32 调试口） */
#define STM32_UART_NUM    UART_NUM_1

/* 波特率，与 STM32 USART2 一致 */
#define STM32_UART_BAUD   115200

/* GPIO 引脚（根据你的 ESP32-S3 接线修改） */
#define STM32_UART_TX_PIN 17   /* ESP32 TX → STM32 PA3 (RX) */
#define STM32_UART_RX_PIN 18   /* ESP32 RX ← STM32 PA2 (TX) */

/* 接收缓冲区大小 */
#define UART_RX_BUF_SIZE  256

static const char *TAG = "STM32_COMM";

/* ======================== UART 初始化 ======================== */

static void stm32_uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate  = STM32_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    /* 安装 UART 驱动，分配缓冲区
     * TX buffer=512：防止 256字节 OTA 帧因 FIFO(128B) 限制被拆成两批发送
     * 有了 TX ring buffer，uart_write_bytes(256B) 一次性写入缓冲区，
     * DMA/FIFO 连续搬运，STM32 不会误检测到 IDLE 中断把帧截断。 */
    ESP_ERROR_CHECK(uart_driver_install(STM32_UART_NUM,
                                        UART_RX_BUF_SIZE * 2,  /* RX buffer */
                                        512,                    /* TX buffer (512B ring buf, 防止256B帧被FIFO截断) */
                                        0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(STM32_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(STM32_UART_NUM,
                                  STM32_UART_TX_PIN,   /* TX */
                                  STM32_UART_RX_PIN,   /* RX */
                                  UART_PIN_NO_CHANGE,   /* RTS */
                                  UART_PIN_NO_CHANGE)); /* CTS */

    ESP_LOGI(TAG, "UART%d initialized: %d baud, TX=GPIO%d, RX=GPIO%d",
             STM32_UART_NUM, STM32_UART_BAUD,
             STM32_UART_TX_PIN, STM32_UART_RX_PIN);
}

/* ======================== 发送任务 ======================== */

static void uart_tx_task(void *param)
{
    uint32_t seq = 0;
    char tx_buf[64];

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(2000));

        /* OTA session 期间（START 已发出、END/ABORT 尚未发出）跳过心跳。
         * 使用 session 级标志而非单帧 busy 标志：DATA 帧之间有 ~100ms 间隙，
         * busy 为 false 时心跳仍可能插入 UART，导致 STM32 IDLE 中断误判帧边界。 */
        if (mqtt_ota_session_active()) {
            ESP_LOGD(TAG, "TX: heartbeat skipped (OTA session active)");
            continue;
        }

        int len = snprintf(tx_buf, sizeof(tx_buf),
                           "ESP32 heartbeat #%lu\r\n", (unsigned long)seq++);

        uart_write_bytes(STM32_UART_NUM, tx_buf, len);
        ESP_LOGI(TAG, "TX: %.*s", len - 2, tx_buf);
    }
}

/* ======================== 接收任务 ======================== */

static void uart_rx_task(void *param)
{
    uint8_t rx_buf[UART_RX_BUF_SIZE];
    uint32_t rx_count = 0;
    uint32_t poll_count = 0;

    for (;;) {
        /* 阻塞等待数据，超时 1000ms */
        int len = uart_read_bytes(STM32_UART_NUM, rx_buf,
                                   sizeof(rx_buf) - 1,
                                   pdMS_TO_TICKS(1000));
        if (len > 0) {
            rx_count++;
            rx_buf[len] = '\0';
            ESP_LOGI(TAG, "RX #%lu (%d bytes): %s",
                     (unsigned long)rx_count, len, (char *)rx_buf);

            /* 根据内容类型路由到不同 MQTT 主题：
             * - JSON（以 '{' 开头）：传感器数据/告警/统计
             * - 其他文本：OTA 响应（[OTA] ACK/NACK/...） */
            if (mqtt_is_connected()) {
                if (len >= 2 && rx_buf[0] == '{') {
                    if (strstr((char *)rx_buf, "\"type\":\"alert\"")) {
                        mqtt_publish_alert((const char *)rx_buf, len);
                    } else {
                        mqtt_publish_sensor((const char *)rx_buf, len);
                    }
                } else {
                    mqtt_publish_status((const char *)rx_buf, len);
                }
            }
        } else {
            /* 每 10 秒打印一次诊断（10 次超时） */
            poll_count++;
            if (poll_count % 10 == 0) {
                ESP_LOGW(TAG, "DIAG: no UART RX from STM32 in %lus (total_rx=%lu)",
                         (unsigned long)poll_count, (unsigned long)rx_count);
            }
        }
    }
}

/* ======================== 入口 ======================== */

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-S3 Gateway starting...");

    /* 1. NVS 初始化（WiFi 驱动必需） */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* 2. 连接 WiFi（阻塞直到成功或失败） */
    if (wifi_station_init() != ESP_OK) {
        ESP_LOGE(TAG, "WiFi connection failed! Continuing without network...");
    }

    /* 3. 启动 MQTT 客户端（WiFi 连接后） */
    if (wifi_is_connected()) {
        if (mqtt_client_start() != ESP_OK) {
            ESP_LOGE(TAG, "MQTT client start failed!");
        }
    }

    /* 4. 初始化与 STM32 通信的 UART */
    stm32_uart_init();

    /* 5. 创建收发任务 */
    xTaskCreate(uart_tx_task, "uart_tx", 2048, NULL, 5, NULL);
    xTaskCreate(uart_rx_task, "uart_rx", 3072, NULL, 5, NULL);

    ESP_LOGI(TAG, "Tasks created, communication started.");
}
