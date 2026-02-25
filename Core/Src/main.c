#include "stm32f1xx_hal.h"
#include "sys_config.h"
#include "usart1.h"
#include "usart2.h"
#include "ota_task.h"
#include "esp32_comm.h"
#include "comm_test.h"
#include "w25qxx.h"
#include "dht11.h"
#include "sensor_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include <stdio.h>

/* 编译开关：定义此宏时运行通信测试，注释掉则运行正常 OTA 流程 */
// #define ENABLE_COMM_TEST   /* 关闭：运行正常 OTA 模式 */

/* 占位变量，防止 stm32f1xx_it.c 中 extern 报错（SPI1 使用 Polling 模式，DMA 未启用） */
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
TIM_HandleTypeDef htim2;

/* OTA 信号量（修复 #7：在调度器启动前创建，ISR 使用前确保有效） */
SemaphoreHandle_t xOtaSemaphore = NULL;

/* ESP32-S3 通信信号量 */
SemaphoreHandle_t xEsp32Semaphore = NULL;

/* OTA 互斥锁（保护状态机，允许 USART1/USART2 双通道安全访问） */
SemaphoreHandle_t xOtaMutex = NULL;

/* USART2 TX 互斥锁（保护 vDataProcessTask 与 vEsp32CommTask 的 TX 竞争） */
SemaphoreHandle_t xUart2TxMutex = NULL;

/* 传感器 Queue 和 EventGroup */
QueueHandle_t        xSensorQueue      = NULL;
EventGroupHandle_t   xSensorEventGroup = NULL;

/* 任务句柄（用于运行时统计 uxTaskGetStackHighWaterMark） */
TaskHandle_t xOtaTaskHandle         = NULL;
TaskHandle_t xEspCommTaskHandle     = NULL;
TaskHandle_t xSensorTaskHandle      = NULL;
TaskHandle_t xDataProcessTaskHandle = NULL;

/* OTA session 标志（ota_task.c 中 SET/CLR，sensor_task.c 中读取） */
volatile uint8_t g_ota_session_active = 0;

int main(void)
{
    /* 1. 硬件初始化 */
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    DHT11_DWT_Init();       /* 使能 DWT 周期计数器（DHT11 时序 + 运行时统计） */
    USART1_UART_Init();
    USART2_UART_Init();

    /* 2. 初始化外部 Flash（SPI1 + W25Q32，OTA Task 写入固件前必须先初始化） */
    if (W25Qxx_Init() != 0)
    {
        printf("[System] ERROR: W25Q32 init failed! Check SPI wiring.\r\n");
        while (1);
    }

    printf("[System] FreeRTOS Starting...\r\n");

#ifdef ENABLE_COMM_TEST
    /* 通信测试模式：创建测试任务 (栈 512 字 = 2048 字节，printf 需要较大栈) */
    xTaskCreate(vCommTestTask, "CommTest", 512, NULL, 2, NULL);
#else
    /* 正常 OTA + 传感器模式 */
    /* 2. 创建信号量、互斥锁、Queue、EventGroup */
    xOtaSemaphore    = xSemaphoreCreateBinary();
    xEsp32Semaphore  = xSemaphoreCreateBinary();
    xOtaMutex        = xSemaphoreCreateMutex();
    xUart2TxMutex    = xSemaphoreCreateMutex();
    xSensorQueue     = xQueueCreate(SENSOR_WINDOW_SIZE, sizeof(SensorReading_t));
    xSensorEventGroup = xEventGroupCreate();

    /* 3. 创建任务（保存句柄用于运行时统计） */
    xTaskCreate(vOtaProcessTask,  "OTA",      512, NULL, 3, &xOtaTaskHandle);
    xTaskCreate(vEsp32CommTask,   "ESP32",    512, NULL, 2, &xEspCommTaskHandle);
    xTaskCreate(vSensorTask,      "Sensor",   128, NULL, 1, &xSensorTaskHandle);
    xTaskCreate(vDataProcessTask, "DataProc", 192, NULL, 1, &xDataProcessTaskHandle);
#endif

    /* 4. 启动调度器 */
    vTaskStartScheduler();

    /* 如果运行到这里，说明内存不足导致调度器启动失败 */
    while (1)
    {
        printf("[Error] RTOS Start Failed!\r\n");
        HAL_Delay(1000);
    }
}
