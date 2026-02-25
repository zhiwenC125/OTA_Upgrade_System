#include "sensor_task.h"
#include "dht11.h"
#include "usart2.h"
#include "sys_config.h"
#include <stdio.h>
#include <string.h>

/* ============================================================================
 * vSensorTask — DHT11 周期采集
 *
 * 优先级 1（最低应用任务），栈 128 words。
 * 每 2 秒读取 DHT11，将结果压入 xSensorQueue。
 * ============================================================================ */

void vSensorTask(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();
    DHT11_Data_t raw;
    SensorReading_t reading;

    printf("[Sensor] Task started. Reading DHT11 every 2s on PB1.\r\n");

    for (;;) {
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(2000));

        uint8_t rc = DHT11_Read(&raw);

        reading.temperature = (int8_t)raw.temp_int;
        reading.humidity    = raw.humidity_int;
        reading.valid       = (rc == DHT11_OK) ? 1U : 0U;
        reading.tick        = (uint32_t)xTaskGetTickCount();

        if (rc == DHT11_OK) {
            printf("[Sensor] T=%d C, H=%u %%\r\n",
                   (int)reading.temperature, (unsigned)reading.humidity);
        } else {
            printf("[Sensor] Read error (code=%u)\r\n", (unsigned)rc);
        }

        /* 非阻塞入队；队列满时丢弃最旧数据 */
        if (xQueueSend(xSensorQueue, &reading, 0) != pdPASS) {
            SensorReading_t discard;
            xQueueReceive(xSensorQueue, &discard, 0);
            xQueueSend(xSensorQueue, &reading, 0);
        }
    }
}

/* ============================================================================
 * vDataProcessTask — 数据融合 + 告警 + 上报
 *
 * 优先级 1，栈 192 words。
 * 从 Queue 接收数据，维护滑动窗口平均值，
 * 阈值检查触发 EventGroup + LED，
 * 定期通过 USART2 发送 JSON 到 ESP32 → MQTT。
 * ============================================================================ */

void vDataProcessTask(void *pvParameters)
{
    (void)pvParameters;
    SensorReading_t reading;
    SensorReading_t window[SENSOR_WINDOW_SIZE];
    uint8_t win_count   = 0;
    uint8_t win_idx     = 0;
    uint8_t report_cnt  = 0;
    uint8_t stats_cnt   = 0;

    printf("[DataProc] Task started. Window=%d, Thresholds: T>%d, H>%u\r\n",
           SENSOR_WINDOW_SIZE, SENSOR_TEMP_THRESHOLD, SENSOR_HUMI_THRESHOLD);

    for (;;) {
        /* 阻塞等待传感器数据（超时 5s，防止无传感器时死等） */
        if (xQueueReceive(xSensorQueue, &reading, pdMS_TO_TICKS(5000)) != pdPASS) {
            continue;
        }

        /* 跳过无效读数 */
        if (!reading.valid) continue;

        /* ---- 滑动窗口 ---- */
        window[win_idx] = reading;
        win_idx = (win_idx + 1) % SENSOR_WINDOW_SIZE;
        if (win_count < SENSOR_WINDOW_SIZE) win_count++;

        int32_t temp_sum = 0, humi_sum = 0;
        for (uint8_t i = 0; i < win_count; i++) {
            temp_sum += window[i].temperature;
            humi_sum += window[i].humidity;
        }
        int16_t  avg_temp = (int16_t)(temp_sum / (int32_t)win_count);
        uint16_t avg_humi = (uint16_t)(humi_sum / (int32_t)win_count);

        /* ---- 阈值检查 → EventGroup ---- */
        if (avg_temp > SENSOR_TEMP_THRESHOLD)
            xEventGroupSetBits(xSensorEventGroup, SENSOR_EVT_TEMP_HIGH);
        else
            xEventGroupClearBits(xSensorEventGroup, SENSOR_EVT_TEMP_HIGH);

        if ((uint16_t)avg_humi > SENSOR_HUMI_THRESHOLD)
            xEventGroupSetBits(xSensorEventGroup, SENSOR_EVT_HUMI_HIGH);
        else
            xEventGroupClearBits(xSensorEventGroup, SENSOR_EVT_HUMI_HIGH);

        /* ---- LED 告警（PC13 低电平点亮） ---- */
        EventBits_t bits = xEventGroupGetBits(xSensorEventGroup);
        if (bits & (SENSOR_EVT_TEMP_HIGH | SENSOR_EVT_HUMI_HIGH)) {
            HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        } else {
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET); /* LED 灭 */
        }

        /* ---- 定期上报（每 5 次有效读数 ≈ 10s） ---- */
        report_cnt++;
        if (report_cnt >= 5 && !g_ota_session_active) {
            report_cnt = 0;

            char json[128];
            int n = snprintf(json, sizeof(json),
                "{\"type\":\"sensor\",\"temp\":%d,\"humi\":%u,"
                "\"avg_t\":%d,\"avg_h\":%u,\"alert\":%u}\r\n",
                (int)reading.temperature, (unsigned)reading.humidity,
                (int)avg_temp, (unsigned)avg_humi,
                (unsigned)(bits & 0x03));

            if (xSemaphoreTake(xUart2TxMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                HAL_UART_Transmit(&huart2, (uint8_t *)json, (uint16_t)n, 200);
                xSemaphoreGive(xUart2TxMutex);
            }

            /* 超阈值时额外发送告警 */
            if (bits & (SENSOR_EVT_TEMP_HIGH | SENSOR_EVT_HUMI_HIGH)) {
                char alert[96];
                int an = snprintf(alert, sizeof(alert),
                    "{\"type\":\"alert\",\"temp\":%d,\"humi\":%u,"
                    "\"flags\":%u}\r\n",
                    (int)avg_temp, (unsigned)avg_humi,
                    (unsigned)(bits & 0x03));

                if (xSemaphoreTake(xUart2TxMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    HAL_UART_Transmit(&huart2, (uint8_t *)alert, (uint16_t)an, 200);
                    xSemaphoreGive(xUart2TxMutex);
                }
            }
        }

        /* ---- 运行时统计（每 15 次有效读数 ≈ 30s） ---- */
        stats_cnt++;
        if (stats_cnt >= 15 && !g_ota_session_active) {
            stats_cnt = 0;

            char stats[128];
            int sn = snprintf(stats, sizeof(stats),
                "{\"type\":\"stats\",\"heap\":%u,"
                "\"s_hwm\":%u,\"d_hwm\":%u,"
                "\"o_hwm\":%u,\"e_hwm\":%u}\r\n",
                (unsigned)xPortGetFreeHeapSize(),
                (unsigned)uxTaskGetStackHighWaterMark(xSensorTaskHandle),
                (unsigned)uxTaskGetStackHighWaterMark(xDataProcessTaskHandle),
                (unsigned)uxTaskGetStackHighWaterMark(xOtaTaskHandle),
                (unsigned)uxTaskGetStackHighWaterMark(xEspCommTaskHandle));

            if (xSemaphoreTake(xUart2TxMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                HAL_UART_Transmit(&huart2, (uint8_t *)stats, (uint16_t)sn, 200);
                xSemaphoreGive(xUart2TxMutex);
            }
        }
    }
}
