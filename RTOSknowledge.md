# RTOS 核心知识点总结

本文档记录在 IoT2 OTA 项目中实际运用和调试中发现的 RTOS 与嵌入式系统知识点。

---

## 目录

1. [FreeRTOS 三大核心机制](#一freertos-三大核心机制)
2. [DMA 双缓冲（Ping-Pong）架构](#二dma-双缓冲ping-pong架构)
3. [互斥锁（Mutex）与资源保护](#三互斥锁mutex与资源保护)
4. [FreeRTOS 队列（Queue）在 ESP32 中的应用](#四freertos-队列queue在-esp32-中的应用)
5. [多任务协作模式](#五多任务协作模式)
6. [外设初始化时序](#六外设初始化时序)
7. [ISR 与任务上下文的 API 差异](#七isr-与任务上下文的-api-差异)
8. [volatile 与多上下文共享变量](#八volatile-与多上下文共享变量)
9. [Bootloader 跳转与硬件状态恢复](#九bootloader-跳转与硬件状态恢复)
10. [源码阅读指南（FreeRTOS 内核）](#十源码阅读指南freertos-内核)
11. [OTA 项目调试 Checklist](#十一ota-项目调试-checklist)
12. [面试常见问题](#十二面试常见问题)

---

## 一、FreeRTOS 三大核心机制

### 1.1 延迟中断处理（Deferred Interrupt Processing）

**裸机做法：** 在 `USART1_IRQHandler` 里做所有事（收数据、解析协议、甚至写入 Flash）。
这会导致中断占用 CPU 时间过长，把其他重要任务卡死。

**RTOS 做法（本项目实际代码）：**

```c
// stm32f1xx_it.c — USART1 IDLE 中断
void USART1_IRQHandler(void)
{
    // 清除错误标志（读 SR + 读 DR）
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE) || /* ... */) {
        volatile uint32_t tmp = huart1.Instance->SR;
        tmp = huart1.Instance->DR;
        (void)tmp;
    }

    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);

        // ISR 中只做最快的操作：
        HAL_UART_DMAStop(&huart1);                          // 1. 停 DMA
        g_uart_rx_len = UART_DMA_BUF_SIZE                   // 2. 算长度
                        - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
        g_uart_rx_buf_idx = 1 - g_uart_rx_buf_idx;          // 3. 切换缓冲
        HAL_UART_Receive_DMA(&huart1,                        // 4. 重启 DMA
                             g_uart_rx_buf[g_uart_rx_buf_idx],
                             UART_DMA_BUF_SIZE);

        // 5. 把脏活累活扔给任务去做
        if (xOtaSemaphore != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(xOtaSemaphore, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
    // ★ 不调用 HAL_UART_IRQHandler()，避免 HAL 在 ORE 时禁用 DMA
}
```

- 中断里只做最快的事：清标志、停 DMA、切缓冲、重启 DMA、释放信号量
- 把解析协议、CRC 校验、写 Flash 等耗时操作扔给 `vOtaProcessTask`
- 收益：中断永远"快进快出"，系统响应速度极高

### 1.2 阻塞与 CPU 使用率（Blocking vs Polling）

**裸机做法：** `while(RX_Flag == 0);` 死等数据。CPU 在空转，费电且干不了别的事。

**RTOS 做法：** `xSemaphoreTake(..., portMAX_DELAY)`

```c
// ota_task.c — vOtaProcessTask
void vOtaProcessTask(void *pvParameters)
{
    HAL_UART_Receive_DMA(&huart1, g_uart_rx_buf[g_uart_rx_buf_idx], UART_DMA_BUF_SIZE);

    // 启动确认（回滚看门狗）
    if (Flash_If_GetFlag() == OTA_FLAG_BOOT_COUNTING) {
        Flash_If_SetFlagEx(OTA_FLAG_CONFIRMED, 0U);
        printf("[OTA] New firmware confirmed OK.\r\n");
    }

    printf("[OTA] Task started v4.0. Waiting for firmware...\r\n");

    for (;;) {
        // 阻塞等待：没有数据时 ota_task 被移出"运行队列"，完全不占 CPU
        // CPU 自动去跑空闲任务（Idle Task）或其他就绪任务
        if (xSemaphoreTake(xOtaSemaphore, portMAX_DELAY) == pdPASS) {
            uint8_t  proc_idx = 1U - g_uart_rx_buf_idx;
            uint16_t len      = g_uart_rx_len;
            if (len > 0U) {
                ota_process_frame(g_uart_rx_buf[proc_idx], len, NULL);
            }
        }
    }
}
```

### 1.3 上下文切换（Context Switch）

**现象：** 为什么在中断里调用 `portYIELD_FROM_ISR`，中断结束后就能立马跳到 `ota_task`，而不是回到 `main`？

**原理：** RTOS 修改了 CPU 的堆栈指针（PSP），"欺骗" CPU 以为 `ota_task` 才是刚才被打断的地方。

```c
// ISR 中的关键代码
BaseType_t xHigherPriorityTaskWoken = pdFALSE;
xSemaphoreGiveFromISR(xOtaSemaphore, &xHigherPriorityTaskWoken);
portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
// 如果 xHigherPriorityTaskWoken == pdTRUE:
//   → 触发 PendSV 中断
//   → ISR 退出后立即切换到 vOtaProcessTask（不回到之前被中断的低优先级任务）
```

> **面试金句：** FreeRTOS 在 Cortex-M3 上通过触发 PendSV（可悬起系统调用）中断来完成上下文切换，以此保证不会打断其他紧急硬件中断。

---

## 二、DMA 双缓冲（Ping-Pong）架构

### 2.1 设计原理

```text
ISR（IDLE 检测）                    任务（Deferred Processing）
1. 停 DMA                           取信号量（阻塞等待）
2. 计算帧长度                        读 buf[1 - g_uart_rx_buf_idx]
3. 切换 buf 索引  --信号量-->         处理数据（OTA 协议解析）
4. 在新 buf 上重启 DMA
5. xSemaphoreGiveFromISR
```

ISR 永远只写"当前 buf"，任务永远只读"另一个 buf"，**无需加锁**。

### 2.2 USART1 完整 ISR 实现

```c
// stm32f1xx_it.c — USART1 IDLE 中断（OTA 通道）
void USART1_IRQHandler(void)
{
    /* 先清除所有 UART 错误标志（PE/FE/NE/ORE）
     * STM32F1 SR 寄存器：读 SR 再读 DR 即可清除错误标志 */
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE) ||
        __HAL_UART_GET_FLAG(&huart1, UART_FLAG_FE)  ||
        __HAL_UART_GET_FLAG(&huart1, UART_FLAG_NE)  ||
        __HAL_UART_GET_FLAG(&huart1, UART_FLAG_PE))
    {
        volatile uint32_t tmp = huart1.Instance->SR;
        tmp = huart1.Instance->DR;   /* 读 DR 清除错误标志 */
        (void)tmp;
    }

    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);

        /* 1. 停止 DMA，锁定计数器 */
        HAL_UART_DMAStop(&huart1);

        /* 2. 计算接收长度（在 ISR 中算，此时计数器是准确的） */
        g_uart_rx_len = UART_DMA_BUF_SIZE
                        - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);

        /* 3. 切换双缓冲：当前完成帧在 buf[idx]，切换到另一个 */
        g_uart_rx_buf_idx = 1 - g_uart_rx_buf_idx;

        /* 4. 在新 buffer 上重启 DMA（旧 buffer 留给任务安全处理） */
        HAL_UART_Receive_DMA(&huart1,
                             g_uart_rx_buf[g_uart_rx_buf_idx],
                             UART_DMA_BUF_SIZE);

        /* 5. 安全释放信号量 */
        if (xOtaSemaphore != NULL)
        {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(xOtaSemaphore, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}
```

### 2.3 USART2 的 DMA 缓冲区为什么是 257？

```c
// usart2.h
#define UART2_DMA_BUF_SIZE  257  // 比最大帧 256B 多 1
```

**原因：** OTA 帧最大 256 字节。若 DMA 缓冲区也是 256，恰好收满 256 字节时
**DMA TC（传输完成）中断** 和 **UART IDLE 中断** 会同时触发，两者竞争导致帧被拆分。

设为 257 后，DMA TC 永远不会触发（帧最大 256B < 257），所有帧统一由 IDLE 处理。

> 不可改大到 512 — STM32F103 只有 20KB RAM，双缓冲 × 512 = 1KB 已占 5% 内存，
> 实测 512 会 HardFault（栈溢出）。

### 2.4 双缓冲变量定义

```c
// usart1.h — USART1 双缓冲（OTA 通道）
#define UART_DMA_BUF_SIZE   256
extern uint8_t  g_uart_rx_buf[2][UART_DMA_BUF_SIZE];  // 双缓冲
extern uint8_t  g_uart_rx_buf_idx;                      // 当前 DMA 写入的缓冲索引
extern uint16_t g_uart_rx_len;                          // 上一帧长度

// usart2.h — USART2 双缓冲（ESP32 通道）
#define UART2_DMA_BUF_SIZE  257
extern uint8_t  g_uart2_rx_buf[2][UART2_DMA_BUF_SIZE];
extern uint8_t  g_uart2_rx_buf_idx;
extern uint16_t g_uart2_rx_len;
```

### 2.5 为什么不调用 HAL_UART_IRQHandler？

```c
// ★ 不调用 HAL_UART_IRQHandler()
// 原因：HAL 检测到 ORE (Overrun Error) 时会调用 UART_EndRxTransfer()
//       永久禁用 DMA 接收，导致之后所有帧丢失。
// 我们手动清除错误标志（读 SR + 读 DR），完全掌控 DMA 生命周期。
```

这是一个重要的工程决策：HAL 库的"安全"行为（遇到错误停止 DMA）在持续通信场景下
反而成为问题。手动管理中断标志虽然更复杂，但能保证 DMA 永不被意外禁用。

---

## 三、互斥锁（Mutex）与资源保护

### 3.1 为什么需要互斥锁？

本项目中，OTA 状态机可以从**两个通道**被访问：

- **USART1**（PC 直连 OTA）→ `vOtaProcessTask` 调用 `ota_process_frame(..., NULL)`
- **USART2**（ESP32 MQTT OTA）→ `vEsp32CommTask` 调用 `ota_process_frame(..., &huart2)`

两个任务可能在不同时刻调用同一个函数修改同一组静态变量（`s_state`, `s_bytes_written` 等）。
没有互斥锁就是**竞态条件（Race Condition）**。

### 3.2 互斥锁创建与使用

```c
// Core/Src/main.c — 在调度器启动前创建
SemaphoreHandle_t xOtaMutex = NULL;

int main(void)
{
    // ... 硬件初始化 ...
    xOtaMutex = xSemaphoreCreateMutex();  // 创建互斥锁
    xTaskCreate(vOtaProcessTask, "OTA_Task", 512, NULL, 3, NULL);
    xTaskCreate(vEsp32CommTask, "ESP32_Comm", 512, NULL, 2, NULL);
    vTaskStartScheduler();
}
```

### 3.3 goto cleanup 模式（资源保护的 C 语言惯用法）

```c
// ota_task.c — ota_process_frame
void ota_process_frame(const uint8_t *buf, uint16_t len,
                       UART_HandleTypeDef *response_uart)
{
    // ★ 入口：获取互斥锁
    if (xSemaphoreTake(xOtaMutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        printf("[OTA] ERR: mutex timeout, frame dropped\r\n");
        return;
    }
    s_response_uart = response_uart;

    // 所有错误分支都 goto cleanup，不直接 return
    if (len < OTA_FRAME_OVERHEAD) {
        ota_printf("[OTA] ERR: frame too short\r\n");
        goto cleanup;  // ← 跳到统一出口
    }

    if (buf[0] != OTA_SOF) {
        ota_printf("[OTA] ERR: bad SOF\r\n");
        goto cleanup;  // ← 跳到统一出口
    }

    // ... 正常处理逻辑 ...

    switch (cmd) {
    case CMD_OTA_START: /* ... */ break;
    case CMD_OTA_DATA:
        if (seq != s_next_seq) {
            s_state = OTA_STATE_ERROR;
            goto cleanup;  // ← 序号错误也走统一出口
        }
        // ... 写入 W25Q32 ...
        break;
    case CMD_OTA_END:
        // END 帧特殊处理：手动释放锁后复位（不走 cleanup）
        s_response_uart = NULL;
        xSemaphoreGive(xOtaMutex);
        vTaskDelay(pdMS_TO_TICKS(1000));
        HAL_NVIC_SystemReset();
        return;
    }

cleanup:
    // ★ 出口：无论走哪个分支，锁一定会被释放
    s_response_uart = NULL;
    xSemaphoreGive(xOtaMutex);
}
```

**为什么用 goto 而不是嵌套 if？**

1. 避免遗忘释放锁导致死锁（所有错误路径汇聚到一个出口）
2. 减少代码缩进层级（错误分支不嵌套）
3. 这是 Linux 内核的标准模式（`goto err_unlock`, `goto out_free`）

### 3.4 Mutex vs Binary Semaphore

| 特性 | Mutex（互斥锁） | Binary Semaphore（二值信号量） |
| --- | --- | --- |
| 所有权 | 有（谁 Take 谁 Give） | 无（任何人都能 Give） |
| 优先级继承 | 支持（防止优先级反转） | 不支持 |
| 用途 | 保护共享资源 | ISR → Task 事件通知 |
| 本项目使用 | `xOtaMutex` 保护 OTA 状态机 | `xOtaSemaphore` / `xEsp32Semaphore` 通知帧到达 |

---

## 四、FreeRTOS 队列（Queue）在 ESP32 中的应用

### 4.1 问题背景

ESP32 的 MQTT 事件回调运行在 **MQTT 任务上下文**中。回调中的 `event->data` 指针
指向 MQTT 库内部的 ring buffer。如果在回调中直接调用 `uart_write_bytes()`：

```text
MQTT 回调 → uart_write_bytes(event->data, len)
                   ↓
              此时 MQTT task 让出 CPU
                   ↓
              MQTT 库收到新 TCP 包，覆盖 ring buffer
                   ↓
              uart_write_bytes 实际发出的是垃圾数据！
```

### 4.2 解决方案：队列 + 专用发送任务

```c
// mqtt_client_task.c

// 队列消息体：复制一份完整帧数据，彻底脱离 MQTT buffer
typedef struct {
    uint8_t  data[256];   // OTA_FRAME_MAX
    uint16_t len;
} ota_tx_item_t;

static QueueHandle_t s_ota_tx_queue = NULL;  // 深度 4

// 初始化
esp_err_t mqtt_client_start(void)
{
    // 每个 item ≈ 258 字节，4 个 ≈ 1KB
    s_ota_tx_queue = xQueueCreate(OTA_TX_QUEUE_DEPTH, sizeof(ota_tx_item_t));

    // 创建专用发送任务（优先级 6，高于心跳）
    xTaskCreate(uart_ota_tx_task, "ota_uart_tx", 2048, NULL, 6, NULL);

    // ... MQTT 客户端初始化 ...
}
```

### 4.3 MQTT 回调中的零拷贝修复

```c
// mqtt_client_task.c — MQTT_EVENT_DATA 处理
case MQTT_EVENT_DATA:
{
    if (strncmp(event->topic, MQTT_TOPIC_OTA_CMD, event->topic_len) == 0)
    {
        // ★ 关键：立即复制 event->data 到栈上的 item
        // 必须在回调返回前（MQTT task 让出 CPU 之前）完成复制
        ota_tx_item_t item;
        memcpy(item.data, event->data, event->data_len);
        item.len = (uint16_t)event->data_len;

        // 根据帧 CMD 字节更新 session 状态
        if (item.len >= 2 && item.data[0] == 0xAA) {
            uint8_t cmd = item.data[1];
            if (cmd == 0x01)       s_ota_session_active = true;   // START
            else if (cmd == 0x03 || cmd == 0x04)
                                    s_ota_session_active = false;  // END/ABORT
        }

        // 入队（不阻塞：队列满 → 丢帧报错）
        if (xQueueSend(s_ota_tx_queue, &item, 0) != pdPASS) {
            ESP_LOGE(TAG, "OTA TX queue full, frame dropped!");
        }
    }
    break;
}
```

### 4.4 专用 UART 发送任务

```c
// mqtt_client_task.c
static void uart_ota_tx_task(void *param)
{
    ota_tx_item_t item;

    for (;;) {
        // 阻塞等待队列有帧
        if (xQueueReceive(s_ota_tx_queue, &item, portMAX_DELAY) != pdPASS)
            continue;

        s_ota_uart_busy = true;

        // 等待前一帧彻底发完（256B@115200 ≈ 22ms，给 50ms 裕量）
        uart_wait_tx_done(STM32_UART_NUM, pdMS_TO_TICKS(50));

        // 发送本帧（数据在 item.data 中，与 MQTT buffer 完全无关）
        uart_write_bytes(STM32_UART_NUM, item.data, item.len);

        // 等待本帧发完
        uart_wait_tx_done(STM32_UART_NUM, pdMS_TO_TICKS(50));

        s_ota_uart_busy = false;
    }
}
```

### 4.5 设计要点总结

| 设计决策 | 原因 |
| --- | --- |
| `memcpy` 到栈上再入队 | `event->data` 是 MQTT ring buffer 指针，随时可能被覆盖 |
| 队列深度 = 4 | 缓冲 START + 前几个 DATA（Python 发帧间隔 ~100ms，处理 ~25ms） |
| 入队不阻塞（timeout=0） | MQTT 回调不能阻塞（会影响 MQTT 连接维护） |
| 专用 TX 任务优先级 = 6 | 高于心跳任务（5），确保 OTA 帧优先发送 |
| `uart_wait_tx_done` | 确保帧间无重叠，STM32 IDLE 检测能正确分帧 |

---

## 五、多任务协作模式

### 5.1 STM32 任务架构

```text
┌──────────────────────────────────────────────────────┐
│                    main.c                             │
│  HAL_Init → SystemClock → GPIO → USART → W25Q       │
│  xOtaSemaphore = xSemaphoreCreateBinary()            │
│  xEsp32Semaphore = xSemaphoreCreateBinary()          │
│  xOtaMutex = xSemaphoreCreateMutex()                 │
│  xTaskCreate(vOtaProcessTask, prio=3)                │
│  xTaskCreate(vEsp32CommTask,  prio=2)                │
│  vTaskStartScheduler()                               │
└──────────────────────────────────────────────────────┘

┌─────────────────┐         ┌─────────────────────────┐
│ USART1 ISR      │ ──sem──►│ vOtaProcessTask (prio 3)│
│ IDLE 检测       │         │  读 buf[1 - idx]        │
│ DMA 双缓冲切换  │         │  ota_process_frame(NULL)│
│ g_uart_rx_len   │         │  CRC + W25Q32 写入      │
└─────────────────┘         └─────────────────────────┘

┌─────────────────┐         ┌─────────────────────────┐
│ USART2 ISR      │ ──sem──►│ vEsp32CommTask (prio 2) │
│ IDLE 检测       │         │  SOF 帧重同步           │
│ DMA 双缓冲切换  │         │  ota_process_frame(uart2)│
│ g_uart2_rx_len  │         │  非OTA → 回复 OK        │
└─────────────────┘         └─────────────────────────┘
```

### 5.2 ESP32 任务架构

```text
┌────────────────────────────────────────────┐
│              app_main()                     │
│  NVS → WiFi → MQTT → UART → 创建任务       │
└────────────────────────────────────────────┘

┌──────────────────────────┐
│ MQTT Event Handler       │   (MQTT task 上下文)
│ OTA cmd → memcpy → 入队  │──queue──►┌──────────────────────┐
│ session flag 管理        │          │ uart_ota_tx_task (6) │
└──────────────────────────┘          │ 出队 → UART 发送     │
                                      └──────────────────────┘

┌──────────────────────────┐
│ uart_tx_task (5)         │   心跳任务
│ 2s 间隔发送心跳          │   OTA session 期间自动跳过
└──────────────────────────┘

┌──────────────────────────┐
│ uart_rx_task (5)         │   STM32 回复接收
│ uart_read_bytes 阻塞等待 │   → mqtt_publish_status
└──────────────────────────┘

┌──────────────────────────┐
│ mqtt_heartbeat_task (3)  │   MQTT JSON 心跳
│ 10s 间隔发布到 Broker    │
└──────────────────────────┘
```

### 5.3 双通道 OTA 共存设计

`ota_process_frame()` 是公共帧处理函数，两个通道共享同一个状态机：

```c
// USART1 通道（PC 直连）— response_uart = NULL，回复只走 printf(USART1)
void vOtaProcessTask(void *pvParameters)
{
    for (;;) {
        xSemaphoreTake(xOtaSemaphore, portMAX_DELAY);
        ota_process_frame(g_uart_rx_buf[proc_idx], len, NULL);
    }
}

// USART2 通道（ESP32 MQTT）— response_uart = &huart2，回复同时发给 ESP32
void vEsp32CommTask(void *pvParameters)
{
    for (;;) {
        xSemaphoreTake(xEsp32Semaphore, pdMS_TO_TICKS(5000));
        // SOF 帧重同步
        while (sof_offset < len && buf[sof_offset] != 0xAA) sof_offset++;
        ota_process_frame(frame, flen, &huart2);
    }
}
```

**`ota_printf` 双通道输出：**

```c
// ota_task.c — 同时输出到 USART1(调试) 和 response_uart(ESP32)
static void ota_printf(const char *fmt, ...)
{
    char tmp[128];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(tmp, sizeof(tmp), fmt, args);
    va_end(args);

    printf("%s", tmp);  // USART1 调试输出

    // 如果是 ESP32 通道，还要回传给 ESP32 → MQTT → PC
    if (s_response_uart != NULL && n > 0) {
        HAL_UART_Transmit(s_response_uart, (uint8_t *)tmp, (uint16_t)n, 100);
    }
}
```

### 5.4 ESP32 帧重同步机制

```c
// esp32_comm.c — vEsp32CommTask 中的 SOF 扫描
// 正常情况 sof_offset==0；若有残留心跳字节"污染"则跳过
uint16_t sof_offset = 0;
while (sof_offset < len && g_uart2_rx_buf[proc_idx][sof_offset] != OTA_SOF)
    sof_offset++;

if (sof_offset > 0) {
    printf("[ESP32] WARN: skipped %d garbage byte(s) before SOF\r\n",
           (int)sof_offset);
}
```

这是对抗心跳字节与 OTA 帧合并的**防御性编程**：即使心跳抑制偶尔失效，
帧重同步也能在混合数据中找到有效 OTA 帧。

---

## 六、外设初始化时序

### 6.1 正确的初始化顺序

```c
// Core/Src/main.c
int main(void)
{
    // 1. HAL 初始化（SysTick、Flash 预取）
    HAL_Init();

    // 2. 时钟配置（HSI/2 × PLL16 = 64MHz）
    SystemClock_Config();

    // 3. GPIO 初始化（PC13 LED）
    GPIO_Init();

    // 4. UART 初始化（USART1 + USART2，含 DMA 配置）
    USART1_UART_Init();
    USART2_UART_Init();

    // 5. ★ SPI Flash 初始化（必须在调度器前，必须检查返回值）
    if (W25Qxx_Init() != 0) {
        printf("[System] ERROR: W25Q32 init failed!\r\n");
        while (1);  // 初始化失败立即停止
    }

    // 6. 创建同步原语（必须在 ISR 可能触发前）
    xOtaSemaphore   = xSemaphoreCreateBinary();
    xEsp32Semaphore = xSemaphoreCreateBinary();
    xOtaMutex       = xSemaphoreCreateMutex();

    // 7. 创建任务
    xTaskCreate(vOtaProcessTask, "OTA_Task",   512, NULL, 3, NULL);
    xTaskCreate(vEsp32CommTask, "ESP32_Comm", 512, NULL, 2, NULL);

    // 8. 启动调度器
    vTaskStartScheduler();

    // 不应到达此处
    while (1) { printf("[Error] RTOS Start Failed!\r\n"); HAL_Delay(1000); }
}
```

### 6.2 为什么 W25Qxx_Init 必须在调度器前？

```text
如果在任务内部调用 W25Qxx_Init()：
  - 若任务还未运行就有 IDLE 中断触发 → xSemaphoreGiveFromISR 对空指针操作 → HardFault
  - W25Qxx_Init 内部使用 HAL_SPI_Transmit（阻塞式），在调度器启动前调用是安全的
  - 在任务中调用则需要考虑 SPI 总线互斥（其他任务可能也在用 SPI）

如果忘记调用 W25Qxx_Init()：
  - hspi1.Instance = NULL（全零）
  - 所有 SPI 操作静默返回 HAL_ERROR
  - W25Q32 读回全 0xFF，但 CRC 校验的是 UART 缓冲区 → 假通过
```

### 6.3 信号量 NULL 检查

ISR 中释放信号量前检查 `!= NULL`，防止上电噪声在信号量创建前触发中断：

```c
if (xOtaSemaphore != NULL)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xOtaSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
```

---

## 七、ISR 与任务上下文的 API 差异

FreeRTOS 的 API 分为两套，**不可混用**：

| 场景 | 信号量释放 | 信号量获取 | 队列发送 |
| --- | --- | --- | --- |
| 普通任务中 | `xSemaphoreGive(sem)` | `xSemaphoreTake(sem, ticks)` | `xQueueSend(q, &item, ticks)` |
| ISR 中 | `xSemaphoreGiveFromISR(sem, &woken)` | **不允许**（ISR 不可阻塞） | `xQueueSendFromISR(q, &item, &woken)` |

**错误使用的后果：**

- 在 ISR 中调用 `xSemaphoreTake` → 阻塞 → CPU 永远停在 ISR → 系统挂死
- 在 ISR 中调用 `xSemaphoreGive`（而非 FromISR 版本）→ 不会触发上下文切换 → 延迟响应

**`portYIELD_FROM_ISR` 的作用：**

```c
BaseType_t xHigherPriorityTaskWoken = pdFALSE;
xSemaphoreGiveFromISR(xOtaSemaphore, &xHigherPriorityTaskWoken);
portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
// 如果信号量唤醒了更高优先级的任务：
//   xHigherPriorityTaskWoken = pdTRUE
//   portYIELD_FROM_ISR 触发 PendSV 中断
//   ISR 退出后立即切换到高优先级任务
```

**ISR 优先级配置（`FreeRTOSConfig.h`）：**

```c
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY  5
// 使用 FreeRTOS API 的 ISR 优先级必须 ≥ 5（数值越大优先级越低）
// USART1/2 的 NVIC 优先级设为 5，符合要求
// 优先级 < 5 的 ISR 不能调用任何 FreeRTOS API
```

---

## 八、volatile 与多上下文共享变量

### 8.1 ESP32 中的 volatile 标志

```c
// mqtt_client_task.c
static volatile bool s_ota_uart_busy = false;       // ISR/任务间共享
static volatile bool s_ota_session_active = false;   // 多任务间共享
```

**为什么用 `volatile`？**

- `s_ota_session_active` 在 MQTT 事件回调中设置，在 `uart_tx_task` 中读取
- 没有 `volatile`，编译器可能将其缓存在寄存器中，读不到最新值
- `volatile` 告诉编译器"这个变量可能在任何时候被外部修改，每次都要从内存读取"

### 8.2 STM32 中的共享变量

```c
// usart1.h
extern uint8_t  g_uart_rx_buf_idx;  // ISR 写，任务读
extern uint16_t g_uart_rx_len;      // ISR 写，任务读
```

这些变量在 ISR 和任务间共享，但由于双缓冲设计的天然隔离（ISR 写完后才释放信号量，
任务收到信号量后才读），不需要额外的 `volatile` 或锁保护。

---

## 九、Bootloader 跳转与硬件状态恢复

裸机跳转到另一段程序需要手动恢复硬件状态：

```c
// Bootloader/main.c
static void Boot_JumpToApp(void)
{
    uint32_t app_msp   = *(__IO uint32_t *)FLASH_APP_ADDR;
    uint32_t app_reset = *(__IO uint32_t *)(FLASH_APP_ADDR + 4);

    // 1. 校验 MSP（必须在合理的 SRAM 范围）
    if ((app_msp & 0x2FFE0000) != 0x20000000) return;

    // 2. 关中断
    __disable_irq();

    // 3. 停 SysTick（否则进 App 后触发 Bootloader 的 Handler）
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    // 4. 复位 SPI1（让 App 从干净状态重新初始化）
    __HAL_RCC_SPI1_FORCE_RESET();
    __HAL_RCC_SPI1_RELEASE_RESET();
    __HAL_RCC_SPI1_CLK_DISABLE();

    // 5. 设置向量表（告诉 CPU App 的中断向量在哪里）
    SCB->VTOR = FLASH_APP_ADDR;  // 0x08002000

    // 6. 设置主栈指针 + 跳转
    __set_MSP(app_msp);
    ((void (*)(void))app_reset)();
}
```

**常见错误：**

| 遗漏步骤 | 后果 |
| --- | --- |
| 忘记停 SysTick | App 进入后触发 Bootloader 的 SysTick Handler（向量表未切换时） |
| 忘记设置 VTOR | App 的中断向量找不到，第一个中断触发 HardFault |
| 未复位外设 | App 初始化同一外设时状态异常 |
| 未关中断 | 跳转过程中可能触发异常 |

**App 侧配合（`system_stm32f1xx.c`）：**

```c
#define USER_VECT_TAB_ADDRESS
#define VECT_TAB_OFFSET  0x00002000U
// SystemInit() 中执行: SCB->VTOR = 0x08000000 + 0x2000 = 0x08002000
```

---

## 十、源码阅读指南（FreeRTOS 内核）

带着问题去读源码，不要全读，只读关键函数：

### 10.1 任务是怎么"出生"的？

- **目标文件：** `tasks.c`
- **搜索函数：** `prvInitialiseNewTask`（被 `xTaskCreate` 调用）
- **看什么：**
  - 如何初始化 TCB（任务控制块）
  - 如何把任务的堆栈填充成像是被中断打断的样子（伪造现场）—— 这是理解任务切换的基石

### 10.2 信号量其实是队列？

- **目标文件：** `queue.c`（`semphr.h` 里全是宏，通过宏定义指向 queue 函数）
- **搜索函数：** `xQueueGenericReceive`（对应 `xSemaphoreTake`）
- **看什么：** 重点看 `vTaskPlaceOnEventList`，理解任务如何把自己从就绪列表挪到等待列表（阻塞的本质）

### 10.3 谁在执行切换？（最硬核部分）

- **目标文件：** `port.c`（`Middlewares/FreeRTOS/Port/` 下）
- **搜索函数：** `xPortPendSVHandler`
- **看什么：** 这是汇编代码，看它如何保存 r4-r11 寄存器，如何切换 `pxCurrentTCB`，然后恢复下一个任务的寄存器

---

## 十一、OTA 项目调试 Checklist

遇到问题时按以下顺序排查：

```text
□ 1. 串口有无输出？
      无输出 → 检查 SysTick_Handler 是否被弱符号覆盖

□ 2. W25Q32 读回全 0xFF？
      → 检查 W25Qxx_Init() 是否在 vTaskStartScheduler 前调用
      → 用 Bootloader W25Q32 探测打印验证 SPI 通路

□ 3. Bootloader 无法跳转 App？
      → App MSP 是否为有效 SRAM 地址（0x20000000~0x20004FFF）
      → App 区是否被擦除（全 0xFF → MSP=0xFFFFFFFF）

□ 4. App 跳转后立即 HardFault？
      → 检查 SCB->VTOR 是否设置为 0x08002000
      → 检查 system_stm32f1xx.c 中 VECT_TAB_OFFSET 是否为 0x2000

□ 5. CRC32 校验通过但数据错误？
      → 确认校验的是 W25Q32 回读数据，而非 UART 接收缓冲区

□ 6. OTA 过程中 DMA 突然停止接收？
      → 检查 ISR 中是否误调用 HAL_UART_IRQHandler()
      → 该函数检测到 ORE 会永久禁用 DMA

□ 7. ESP32 OTA 帧偶发 NACK？
      → 检查心跳是否在 OTA session 期间被正确抑制
      → 检查 MQTT buffer 是否在发送前被覆盖

□ 8. USART2 收到截断帧？
      → 检查 UART2_DMA_BUF_SIZE 是否为 257（不是 256）
      → 256 会导致 DMA TC 与 IDLE 竞争
```

---

## 十二、面试常见问题

### Q1：FreeRTOS 的任务切换是如何实现的？

**答：** 在 Cortex-M3 上，FreeRTOS 通过 **PendSV 中断** 实现上下文切换。PendSV 被设为最低优先级，确保不会打断其他硬件中断。切换过程：

1. 硬件自动保存 r0-r3, r12, LR, PC, xPSR（8 个寄存器）到当前任务栈
2. `xPortPendSVHandler` 手动保存 r4-r11 到当前任务栈
3. 更新 `pxCurrentTCB` 指向下一个任务的 TCB
4. 从新任务的栈恢复 r4-r11
5. 中断返回时硬件自动恢复 r0-r3 等，CPU 开始执行新任务

### Q2：为什么本项目 ISR 中不调用 HAL_UART_IRQHandler？

**答：** HAL 的 IRQ Handler 在检测到 ORE（Overrun Error）时会调用 `UART_EndRxTransfer()` 永久禁用 DMA。在高速持续通信场景（OTA 帧间隔 ~100ms）中，ORE 偶尔发生是正常的，不应导致 DMA 永久停止。我们手动清除错误标志（读 SR + 读 DR），保持 DMA 持续运行。

### Q3：互斥锁和信号量有什么区别？什么时候用哪个？

**答：**

- **互斥锁（Mutex）**：有所有权概念，谁获取谁释放，支持优先级继承。用于**保护共享资源**。本项目用 `xOtaMutex` 保护 OTA 状态机，因为两个任务（USART1 和 USART2）可能同时访问。
- **二值信号量**：无所有权，任何人都能释放。用于**事件通知**。本项目用 `xOtaSemaphore` / `xEsp32Semaphore` 实现 ISR → Task 的帧到达通知。

### Q4：DMA 双缓冲有什么好处？为什么不用单缓冲？

**答：** 单缓冲下，ISR 停止 DMA 后任务处理数据期间，如果有新数据到来就会丢失。双缓冲（Ping-Pong）让 ISR 切换到另一个缓冲重启 DMA，新数据写入新缓冲，任务安全处理旧缓冲，两者完全无竞争，不需要加锁。

### Q5：ESP32 的 MQTT 回调为什么要 memcpy 而不是直接发送？

**答：** `event->data` 指向 MQTT 库内部 ring buffer。如果在回调中调用 `uart_write_bytes`（可能涉及任务切换），MQTT 库可能在此期间用新的 TCP 包覆盖该 buffer，导致发出垃圾数据。立即 `memcpy` 到栈变量，然后通过 FreeRTOS 队列传给专用发送任务，彻底解耦数据生命周期。
