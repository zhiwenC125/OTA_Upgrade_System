#ifndef __SENSOR_TASK_H
#define __SENSOR_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "semphr.h"

/* ============================================================================
 * 传感器数据结构（Queue 传输单元）
 * ============================================================================ */
typedef struct {
    int8_t   temperature;   /* DHT11 温度整数部分（℃） */
    uint8_t  humidity;      /* DHT11 湿度整数部分（%RH） */
    uint8_t  valid;         /* 1 = 读取成功, 0 = 超时/校验错误 */
    uint8_t  _pad;          /* 对齐填充 */
    uint32_t tick;          /* 采集时刻 xTaskGetTickCount() */
} SensorReading_t;          /* 8 bytes */

/* ============================================================================
 * EventGroup 位定义
 * ============================================================================ */
#define SENSOR_EVT_TEMP_HIGH    (1 << 0)    /* 温度超阈值 */
#define SENSOR_EVT_HUMI_HIGH   (1 << 1)    /* 湿度超阈值 */

/* ============================================================================
 * 阈值与窗口参数
 * ============================================================================ */
#define SENSOR_TEMP_THRESHOLD   35          /* ℃ */
#define SENSOR_HUMI_THRESHOLD   80          /* %RH */
#define SENSOR_WINDOW_SIZE      5           /* 滑动窗口深度 */

/* ============================================================================
 * 共享 FreeRTOS 对象（在 main.c 中创建）
 * ============================================================================ */
extern QueueHandle_t        xSensorQueue;
extern EventGroupHandle_t   xSensorEventGroup;
extern SemaphoreHandle_t    xUart2TxMutex;

/* 任务句柄（用于 uxTaskGetStackHighWaterMark） */
extern TaskHandle_t xOtaTaskHandle;
extern TaskHandle_t xEspCommTaskHandle;
extern TaskHandle_t xSensorTaskHandle;
extern TaskHandle_t xDataProcessTaskHandle;

/* OTA session 标志（ota_task.c 中设置） */
extern volatile uint8_t g_ota_session_active;

/* ============================================================================
 * 任务入口
 * ============================================================================ */
void vSensorTask(void *pvParameters);
void vDataProcessTask(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_TASK_H */
