#ifndef MQTT_CLIENT_TASK_H
#define MQTT_CLIENT_TASK_H

#include "esp_err.h"
#include <stdbool.h>

/**
 * @brief  启动 MQTT 客户端并连接到 Broker
 *
 * 订阅 OTA 命令主题，创建心跳任务。
 * 必须在 wifi_station_init() 成功后调用。
 *
 * @return ESP_OK 成功启动，ESP_FAIL 失败
 */
esp_err_t mqtt_client_start(void);

/**
 * @brief  发布消息到 OTA 状态主题
 *
 * 用于将 STM32 响应转发到 MQTT（ESP32 UART RX → MQTT status）。
 *
 * @param data  数据指针
 * @param len   数据长度
 * @return ESP_OK 成功，ESP_FAIL 失败或未连接
 */
esp_err_t mqtt_publish_status(const char *data, int len);

/**
 * @brief  查询 MQTT 是否已连接
 */
bool mqtt_is_connected(void);

/**
 * @brief  查询当前是否有 OTA 帧正在通过 UART 发送（单帧粒度）
 *
 * uart_tx_task 在发送心跳前应优先检查 mqtt_ota_session_active()，
 * 此接口保留供需要单帧粒度判断的场景使用。
 */
bool mqtt_ota_uart_busy(void);

/**
 * @brief  查询当前是否处于 OTA session（START 已发出、END/ABORT 尚未发出）
 *
 * 比 mqtt_ota_uart_busy() 粒度更粗（整个 session 期间为 true），
 * uart_tx_task 应使用此接口压制心跳，防止心跳帧在两个 OTA DATA 帧之间
 * 插入 UART，导致 STM32 接收错乱。
 *
 * 置位条件：收到 CMD_OTA_START 帧时
 * 清除条件：收到 CMD_OTA_END / CMD_OTA_ABORT 帧，或 MQTT 断线时
 */
bool mqtt_ota_session_active(void);

/**
 * @brief  发布传感器数据到 sensor/data 主题（QoS 0）
 */
esp_err_t mqtt_publish_sensor(const char *data, int len);

/**
 * @brief  发布告警到 sensor/alert 主题（QoS 1）
 */
esp_err_t mqtt_publish_alert(const char *data, int len);

#endif /* MQTT_CLIENT_TASK_H */
