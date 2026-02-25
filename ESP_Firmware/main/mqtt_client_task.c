#include "mqtt_client_task.h"
#include "mqtt_config.h"
#include "wifi_station.h"

#include "mqtt_client.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include <string.h>
#include <stdio.h>

static const char *TAG = "MQTT";

/* 与 hello_world_main.c 共用的 UART 端口号 */
#define STM32_UART_NUM  UART_NUM_1

/* OTA 帧最大长度：SOF(1)+CMD(1)+SEQ(2)+LEN(2)+PAYLOAD(248)+CRC16(2) = 256 */
#define OTA_FRAME_MAX   256

/* TX 队列深度：缓存最多 4 帧（START + 3 个 DATA） */
#define OTA_TX_QUEUE_DEPTH  4

/* OTA 帧标识字节 */
#define OTA_SOF_BYTE    0xAA
#define CMD_OTA_START   0x01
#define CMD_OTA_END     0x03
#define CMD_OTA_ABORT   0x04

/* 队列消息体：复制一份完整帧数据，彻底脱离 MQTT 内部 buffer */
typedef struct {
    uint8_t  data[OTA_FRAME_MAX];
    uint16_t len;
} ota_tx_item_t;

static esp_mqtt_client_handle_t s_mqtt_client    = NULL;
static bool                     s_mqtt_connected = false;

/* per-frame busy flag（保持 mqtt_ota_uart_busy() 向后兼容） */
static volatile bool s_ota_uart_busy = false;

/* session-level flag：START 收到后置位，END/ABORT 后清除；
 * uart_tx_task 检查此标志来整个 OTA session 期间压制心跳 */
static volatile bool s_ota_session_active = false;

/* FreeRTOS 队列：MQTT 事件回调 → UART OTA TX 专用任务 */
static QueueHandle_t s_ota_tx_queue = NULL;

/* ============================================================================
 * UART OTA TX 专用任务
 *
 * 从队列取帧后负责等待 UART 空闲、发送、等待发完，完全在独立任务上下文中
 * 执行，MQTT 事件回调永远不会在此阻塞，event->data 不再有被覆写的窗口。
 * ============================================================================ */

static void uart_ota_tx_task(void *param)
{
    ota_tx_item_t item;

    for (;;) {
        /* 阻塞等待队列有帧 */
        if (xQueueReceive(s_ota_tx_queue, &item, portMAX_DELAY) != pdPASS) {
            continue;
        }

        s_ota_uart_busy = true;

        ESP_LOGI(TAG, "OTA-TX: sending %d bytes to STM32 (SOF=0x%02X cmd=0x%02X)",
                 item.len,
                 item.len >= 1 ? item.data[0] : 0xFF,
                 item.len >= 2 ? item.data[1] : 0xFF);

        /* 等待前一帧彻底发完（256B@115200 ≈ 22ms，给 50ms 裕量） */
        uart_wait_tx_done(STM32_UART_NUM, pdMS_TO_TICKS(50));

        /* 发送本帧（数据已在 item.data，与 MQTT buffer 完全无关） */
        int written = uart_write_bytes(STM32_UART_NUM, item.data, item.len);

        /* 等待本帧发完 */
        uart_wait_tx_done(STM32_UART_NUM, pdMS_TO_TICKS(50));

        ESP_LOGI(TAG, "OTA-TX: done (%d/%d bytes written)", written, item.len);

        s_ota_uart_busy = false;
    }
}

/* ============================================================================
 * MQTT 事件回调
 * ============================================================================ */

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch (event_id) {
    case MQTT_EVENT_CONNECTED:
        s_mqtt_connected = true;
        ESP_LOGI(TAG, "MQTT connected to %s", MQTT_BROKER_URI);

        /* 订阅 OTA 命令主题 */
        esp_mqtt_client_subscribe(s_mqtt_client, MQTT_TOPIC_OTA_CMD, MQTT_OTA_QOS);
        ESP_LOGI(TAG, "Subscribed to %s (QoS %d)", MQTT_TOPIC_OTA_CMD, MQTT_OTA_QOS);

        /* 发布上线消息 */
        esp_mqtt_client_publish(s_mqtt_client, MQTT_TOPIC_HEARTBEAT,
                                "online", 6, MQTT_HB_QOS, 0);
        break;

    case MQTT_EVENT_DISCONNECTED:
        s_mqtt_connected     = false;
        s_ota_session_active = false;   /* 断线时清除 session 状态 */
        ESP_LOGW(TAG, "MQTT disconnected. Auto-reconnect will retry...");
        break;

    case MQTT_EVENT_DATA:
    {
        /* 检查是否为 OTA 命令主题 */
        if (event->topic_len > 0 &&
            strncmp(event->topic, MQTT_TOPIC_OTA_CMD, event->topic_len) == 0)
        {
            if (event->data_len <= 0 || event->data_len > OTA_FRAME_MAX) {
                ESP_LOGW(TAG, "OTA frame size %d out of range (1~%d), ignored",
                         event->data_len, OTA_FRAME_MAX);
                break;
            }

            /* ★ 关键修复：在回调返回前（MQTT task 让出 CPU 之前）立即把
             *   event->data 复制到栈上的 item.data。
             *   event->data 只是 MQTT 库内部 ring buffer 的指针，只要调用了
             *   vTaskDelay 或任何可以切换任务的函数，MQTT 库就可能用下一个
             *   TCP 包覆盖该 buffer，导致 uart_write_bytes 发出垃圾数据。
             *   拷贝后的 item 通过队列传给专用 TX 任务，与 MQTT buffer 完全解耦。 */
            ota_tx_item_t item;
            memcpy(item.data, event->data, event->data_len);
            item.len = (uint16_t)event->data_len;

            /* 根据帧 CMD 字节更新 session 状态（在入队前，使 uart_tx_task 能立即看到） */
            if (item.len >= 2 && item.data[0] == OTA_SOF_BYTE) {
                uint8_t cmd = item.data[1];
                if (cmd == CMD_OTA_START) {
                    s_ota_session_active = true;
                    ESP_LOGI(TAG, "OTA session started");
                } else if (cmd == CMD_OTA_END || cmd == CMD_OTA_ABORT) {
                    /* 注意：在帧实际发完并得到响应前不急于清除；
                     * uart_ota_tx_task 发完后 busy 变 false 足以区分；
                     * 这里在帧入队后由 TX task 发完再清除会更安全，
                     * 但简单起见在入队时清除（心跳不会在 50ms TX 期间插入）。 */
                    s_ota_session_active = false;
                    ESP_LOGI(TAG, "OTA session ending (cmd=0x%02X)", cmd);
                }
            }

            ESP_LOGI(TAG, "OTA frame queued: %d bytes (cmd=0x%02X)",
                     item.len, item.len >= 2 ? item.data[1] : 0xFF);

            /* 入队（不阻塞：队列满说明 TX 任务跟不上，丢弃并报错） */
            if (xQueueSend(s_ota_tx_queue, &item, 0) != pdPASS) {
                ESP_LOGE(TAG, "OTA TX queue full, frame dropped! (len=%d)", item.len);
                s_ota_session_active = false;
            }
        } else {
            ESP_LOGI(TAG, "MQTT msg on topic [%.*s]: %.*s",
                     event->topic_len, event->topic,
                     event->data_len, event->data);
        }
        break;
    }

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT error type: 0x%x", event->error_handle->error_type);
        break;

    default:
        break;
    }
}

/* ============================================================================
 * 心跳任务
 * ============================================================================ */

static void mqtt_heartbeat_task(void *param)
{
    uint32_t uptime = 0;
    char msg[64];

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(MQTT_HEARTBEAT_INTERVAL_MS));
        uptime += MQTT_HEARTBEAT_INTERVAL_MS / 1000;

        if (s_mqtt_connected) {
            int len = snprintf(msg, sizeof(msg),
                               "{\"status\":\"alive\",\"uptime\":%lu,\"wifi\":%s}",
                               (unsigned long)uptime,
                               wifi_is_connected() ? "true" : "false");
            esp_mqtt_client_publish(s_mqtt_client, MQTT_TOPIC_HEARTBEAT,
                                    msg, len, MQTT_HB_QOS, 0);
        }
    }
}

/* ============================================================================
 * 公开 API
 * ============================================================================ */

esp_err_t mqtt_client_start(void)
{
    /* 创建 OTA TX 队列（每个 item = ota_tx_item_t ≈ 258 字节，4 个 ≈ 1KB） */
    s_ota_tx_queue = xQueueCreate(OTA_TX_QUEUE_DEPTH, sizeof(ota_tx_item_t));
    if (s_ota_tx_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create OTA TX queue");
        return ESP_ERR_NO_MEM;
    }

    /* 创建 UART OTA TX 专用任务
     * 优先级 6：高于 uart_tx_task(5)，确保 OTA 帧优先于心跳发出 */
    xTaskCreate(uart_ota_tx_task, "ota_uart_tx", 3072, NULL, 6, NULL);

    /* TLS CA 证书（定义在 ca_cert.c） */
    extern const char ca_cert_pem[];

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
        .broker.verification.certificate = ca_cert_pem,
        .credentials.username = MQTT_USERNAME,
        .credentials.authentication.password = MQTT_PASSWORD,
        .network.timeout_ms = 10000,
        .buffer.size = 512,     /* 确保 OTA 帧（最大 256B）不分片 */
    };

    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (s_mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to init MQTT client");
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(esp_mqtt_client_register_event(
        s_mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));

    esp_err_t err = esp_mqtt_client_start(s_mqtt_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(err));
        return err;
    }

    /* 创建 MQTT 心跳任务 */
    xTaskCreate(mqtt_heartbeat_task, "mqtt_hb", 2048, NULL, 3, NULL);

    ESP_LOGI(TAG, "MQTT client started, connecting to %s ...", MQTT_BROKER_URI);
    return ESP_OK;
}

esp_err_t mqtt_publish_status(const char *data, int len)
{
    if (!s_mqtt_connected || s_mqtt_client == NULL) {
        return ESP_FAIL;
    }

    int msg_id = esp_mqtt_client_publish(s_mqtt_client, MQTT_TOPIC_OTA_STATUS,
                                         data, len, MQTT_OTA_QOS, 0);
    return (msg_id >= 0) ? ESP_OK : ESP_FAIL;
}

bool mqtt_is_connected(void)
{
    return s_mqtt_connected;
}

bool mqtt_ota_uart_busy(void)
{
    return s_ota_uart_busy;
}

bool mqtt_ota_session_active(void)
{
    return s_ota_session_active;
}

esp_err_t mqtt_publish_sensor(const char *data, int len)
{
    if (!s_mqtt_connected || s_mqtt_client == NULL) return ESP_FAIL;
    int msg_id = esp_mqtt_client_publish(s_mqtt_client, MQTT_TOPIC_SENSOR,
                                         data, len, 0, 0);
    return (msg_id >= 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t mqtt_publish_alert(const char *data, int len)
{
    if (!s_mqtt_connected || s_mqtt_client == NULL) return ESP_FAIL;
    int msg_id = esp_mqtt_client_publish(s_mqtt_client, MQTT_TOPIC_ALERT,
                                         data, len, 1, 0);
    return (msg_id >= 0) ? ESP_OK : ESP_FAIL;
}
