#ifndef WIFI_STATION_H
#define WIFI_STATION_H

#include "esp_err.h"
#include <stdbool.h>

/**
 * @brief  初始化 WiFi 站点模式并连接到 AP
 *
 * 阻塞直到连接成功或达到最大重试次数。
 * 调用前需先初始化 NVS (nvs_flash_init)。
 *
 * @return ESP_OK 连接成功，ESP_FAIL 连接失败
 */
esp_err_t wifi_station_init(void);

/**
 * @brief  查询当前 WiFi 是否已连接
 */
bool wifi_is_connected(void);

#endif /* WIFI_STATION_H */
