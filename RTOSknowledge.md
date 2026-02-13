# RTOS 核心知识点总结

本文档记录在 IoT2 OTA 项目中实际运用和调试中发现的 RTOS 与嵌入式系统知识点。

---

## 一、FreeRTOS 三大核心机制（面试必问）

在第一阶段中，你实际上运用了以下三个核心机制，请务必理解它们与裸机的区别：

### 1. 延迟中断处理（Deferred Interrupt Processing）

**裸机做法：** 在 `USART1_IRQHandler` 里做所有事（收数据、解析协议、甚至写入 Flash）。
这会导致中断占用 CPU 时间过长，把其他重要任务卡死。

**RTOS 做法（本项目代码）：**

- 中断里只做最快的事：清除标志位、释放信号量（`xSemaphoreGiveFromISR`）。
- 把脏活累活扔给任务（`ota_task`）去做。
- 收益：系统响应速度极大提高，中断永远是"快进快出"的。

### 2. 阻塞与 CPU 使用率（Blocking vs Polling）

**裸机做法：** `while(RX_Flag == 0);` 死等数据。CPU 在空转，费电且干不了别的事。

**RTOS 做法：** `xSemaphoreTake(..., portMAX_DELAY)`

- 当没有数据时，`ota_task` 会被移出"运行队列"，完全不占用 CPU。
- CPU 会自动去跑空闲任务（Idle Task）或其他就绪任务。

### 3. 上下文切换（Context Switch）

**现象：** 为什么在中断里调用 `portYIELD_FROM_ISR`，中断结束后就能立马跳到 `ota_task`，而不是回到 `main`？

**原理：** 这是 RTOS 的魔法。它修改了 CPU 的堆栈指针（PSP），欺骗 CPU 以为 `ota_task` 才是刚才被打断的地方。

> **面试金句：** FreeRTOS 在 Cortex-M3 上是通过触发 PendSV（可悬起系统调用）中断来完成上下文切换的，以此保证不会打断其他紧急硬件中断。

---

## 二、从本项目调试中发现的关键知识点

### 2.1 弱符号（Weak Symbol）陷阱：SysTick_Handler

**问题现象：** Bootloader 复位后只打印 `===== Bootloader =====`，之后无任何输出。

**根本原因链：**

```text
startup_stm32f103xb.s 中：
  .thumb_set SysTick_Handler, Default_Handler
  Default_Handler = while(1) 死循环

HAL_Init() → HAL_InitTick() → 开启 SysTick 1ms 中断
→ 1ms 后触发 SysTick_Handler
→ 跳入 Default_Handler（死循环）
→ CPU 完全停止，看门狗未启用故无复位
```

**修复：** 在 Bootloader/main.c 中显式定义：

```c
void SysTick_Handler(void)
{
    HAL_IncTick();
}
```

**原理：** C 语言的弱符号（`__attribute__((weak))`）机制允许强定义覆盖弱定义。
启动文件中的 `Default_Handler` 是弱定义，只要在任意 `.c` 文件中提供同名强定义，链接器就会选择强定义版本。

**教训：** 独立 Bootloader 工程（不含完整 HAL 驱动）使用 `HAL_Init()` 时，必须确保所有被 HAL 开启的中断都有对应的 Handler 实现，否则第一次中断即死机。

---

### 2.2 外设初始化时序：W25Qxx_Init() 必须在调度器前调用

**问题现象：** T2 测试中 W25Q32 写入后读回全为 `0xFF`，但测试报告 PASS（假通过）。

**根本原因：** `main.c` 从未调用 `W25Qxx_Init()`，导致：

```text
hspi1.Instance = 0（未初始化）
→ HAL_SPI_Transmit/Receive 检查 State != HAL_SPI_STATE_READY
→ 直接返回 HAL_ERROR，但调用方未检查返回值
→ 所有 SPI 操作静默失败，W25Q32 从未被实际写入
→ ota_task 的 CRC32 校验的是 UART 接收缓冲区的数据，而非 W25Q32 回读
→ T2/T3 假通过
```

**修复：** 在 `xTaskCreate` 之前初始化所有外设：

```c
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    USART1_UART_Init();
    USART2_UART_Init();

    // 必须在任务创建和调度器启动前初始化 SPI Flash
    if (W25Qxx_Init() != 0) {
        printf("[System] ERROR: W25Q32 init failed!\r\n");
        while (1);
    }

    xTaskCreate(...);   // 之后才创建任务
    vTaskStartScheduler();
}
```

**原则：** 所有硬件外设（SPI、I2C、ADC 等）的初始化必须在 `vTaskStartScheduler()` 之前完成。任务中使用外设时应假定外设已初始化，而非在任务内部懒初始化（除非有明确的同步机制）。

---

### 2.3 ISR 与任务上下文的 API 差异

FreeRTOS 的 API 分为两套，**不可混用**：

| 场景 | 信号量释放 | 信号量获取 |
| --- | --- | --- |
| 普通任务中 | `xSemaphoreGive(sem)` | `xSemaphoreTake(sem, ticks)` |
| 中断服务程序（ISR）中 | `xSemaphoreGiveFromISR(sem, &woken)` | 不允许在 ISR 中阻塞等待 |

**本项目用法（UART IDLE 中断）：**

```c
void USART1_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // ISR 中只能用 FromISR 版本
    xSemaphoreGiveFromISR(xOtaSemaphore, &xHigherPriorityTaskWoken);

    // 通知调度器检查是否需要切换任务
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
```

**`portYIELD_FROM_ISR` 的作用：** 如果释放信号量唤醒了更高优先级的任务，`xHigherPriorityTaskWoken` 被置为 `pdTRUE`，`portYIELD_FROM_ISR` 会触发 PendSV 中断，ISR 退出后立即切换到高优先级任务，而非继续运行被中断的低优先级任务。

---

### 2.4 二值信号量（Binary Semaphore）在 ISR→Task 通信中的用法

**创建：** 必须在调度器启动前、ISR 可能触发前创建：

```c
xOtaSemaphore = xSemaphoreCreateBinary();
// 初始状态：空（任务第一次 Take 会阻塞，等待 ISR Give）
```

**任务侧：**

```c
void vOtaProcessTask(void *pvParameters)
{
    while (1) {
        // 阻塞等待，不占 CPU
        xSemaphoreTake(xOtaSemaphore, portMAX_DELAY);
        // 处理数据...
    }
}
```

**二值信号量 vs 计数信号量：**

- 二值信号量：只有 0/1 两个状态，适合"有事件发生"通知，不关心事件累积次数
- 计数信号量：可以累积多次 Give，适合生产者速度 > 消费者速度的场景

---

### 2.5 Bootloader 跳转到 App 的完整步骤

裸机跳转到另一段程序需要手动恢复硬件状态：

```c
static void Boot_JumpToApp(void)
{
    uint32_t app_msp   = *(__IO uint32_t *)FLASH_APP_ADDR;      // 取 App 的栈顶
    uint32_t app_reset = *(__IO uint32_t *)(FLASH_APP_ADDR + 4); // 取 App 的复位向量

    // 1. 校验 MSP 是否在合理的 SRAM 范围
    if ((app_msp & 0x2FFE0000) != 0x20000000) {
        // App 区为空或损坏，拒绝跳转
        return;
    }

    // 2. 关中断，防止跳转过程中触发异常
    __disable_irq();

    // 3. 停止 SysTick（否则进入 App 后可能触发 Bootloader 的 SysTick Handler）
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    // 4. 复位 SPI1（释放外设，让 App 重新初始化）
    __HAL_RCC_SPI1_FORCE_RESET();
    __HAL_RCC_SPI1_RELEASE_RESET();
    __HAL_RCC_SPI1_CLK_DISABLE();

    // 5. 设置向量表偏移（告诉 CPU App 的中断向量表在哪里）
    SCB->VTOR = FLASH_APP_ADDR;  // 0x08002000

    // 6. 设置主栈指针 + 跳转
    __set_MSP(app_msp);
    ((void (*)(void))app_reset)();
}
```

**常见错误：**

- 忘记停 SysTick → App 进入后触发 Bootloader 的 SysTick Handler（若向量表未切换）
- 忘记设置 VTOR → App 的中断向量表找不到，第一个中断触发就 HardFault
- 未复位外设 → App 初始化同一外设时状态异常

**App 侧配合：** `system_stm32f1xx.c` 中必须设置 `VECT_TAB_OFFSET = 0x2000`，使 VTOR 指向 `0x08002000`。

---

## 三、源码阅读指南（FreeRTOS 内核）

带着问题去读源码，不要全读，只读关键函数：

### 3.1 任务是怎么"出生"的？

- **目标文件：** `tasks.c`
- **搜索函数：** `prvInitialiseNewTask`（被 `xTaskCreate` 调用）
- **看什么：**
  - 如何初始化 TCB（任务控制块）
  - 如何把任务的堆栈填充成像是被中断打断的样子（伪造现场）—— 这是理解任务切换的基石

### 3.2 信号量其实是队列？

- **目标文件：** `queue.c`（`semphr.h` 里全是宏，通过宏定义指向了 queue 的函数）
- **搜索函数：** `xQueueGenericReceive`（对应 `xSemaphoreTake`）
- **看什么：** 重点看 `vTaskPlaceOnEventList`，理解任务如何把自己从就绪列表挪到等待列表（阻塞的本质）

### 3.3 谁在执行切换？（最硬核部分）

- **目标文件：** `port.c`（`Middlewares/FreeRTOS/Port/` 下）
- **搜索函数：** `xPortPendSVHandler`
- **看什么：** 这是汇编代码，看它如何保存 r4-r11 寄存器，如何切换 `pxCurrentTCB`，然后恢复下一个任务的寄存器

---

## 四、OTA 项目调试 Checklist

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
```
