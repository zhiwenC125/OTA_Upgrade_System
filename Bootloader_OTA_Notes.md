# Bootloader 与 OTA 空中升级技术笔记

> 基于 STM32F103C8T6 + W25Q32 + FreeRTOS 实战总结
> 项目：IoT2 OTA 空中升级（方案 A — 外部 Download）

---

## 目录

1. [什么是 Bootloader](#一什么是-bootloader)
2. [Flash 分区规划](#二flash-分区规划)
3. [Bootloader 实现细节](#三bootloader-实现细节)
4. [OTA 协议设计](#四ota-协议设计)
5. [OTA 状态机](#五ota-状态机)
6. [CRC 校验机制](#六crc-校验机制)
7. [调试中踩过的坑](#七调试中踩过的坑)
8. [完整升级流程时序图](#八完整升级流程时序图phase-7含回滚)
9. [面试常见问题](#九面试常见问题)

---

## 一、什么是 Bootloader

### 1.1 概念

Bootloader 是上电后第一段运行的程序，负责：

1. 硬件最小化初始化（时钟、串口、SPI）
2. 检测是否有待安装的新固件
3. 若有，将新固件从暂存区复制到 App 区
4. 跳转到 App 执行

它相当于嵌入式设备的"BIOS + 安装程序"。

### 1.2 为什么需要独立的 Bootloader

| 方式 | 优点 | 缺点 |
| --- | --- | --- |
| 无 Bootloader，直接烧录 | 简单 | 必须用调试器（JTAG/SWD），无法远程更新 |
| 有 Bootloader | 可通过串口/网络远程更新固件 | 占用一部分 Flash，跳转时序要处理 |

### 1.3 本项目方案选择（方案 A — 外部 Download）

```text
方案 A：新固件先写入外部 W25Q32，Bootloader 从外部 Flash 拷贝到内部 Flash
方案 B：新固件直接写入内部 Flash 的 Download 槽，Bootloader 内部拷贝

选择方案 A 的原因：
  - STM32F103C8T6 只有 64KB 内部 Flash，放不下两份 App
  - 外部 W25Q32 有 4MB，空间充裕，可同时存 Download + Backup + 日志
  - Bootloader 不依赖 FreeRTOS，代码简洁，体积小（< 8KB）
```

---

## 二、Flash 分区规划

### 2.1 内部 Flash（64KB）

```text
地址空间（STM32F103C8T6，1KB/页）：

0x08000000 ┌─────────────────────────┐
           │   Bootloader (8KB)      │  8 页，独立 CMake 工程
           │   包含：SPI+UART驱动    │
0x08002000 ├─────────────────────────┤
           │   App (55KB)            │  55 页，FreeRTOS + OTA Task
           │   向量表偏移 0x2000      │
0x0800FC00 ├─────────────────────────┤
           │   OTA Flag (1KB)        │  1 页，升级标志位
0x08010000 └─────────────────────────┘
```

> **为什么 Flag 放最后一页（0x0800FC00）？**
> STM32F103 内部 Flash 擦除单位是 1KB/页。把 Flag 放在独立页，
> 擦写标志位时不会影响 App 代码。

### 2.2 外部 W25Q32（4MB）

```text
0x000000 ┌─────────────────────────┐
         │   Download (64KB)       │  新固件暂存区，OTA Task 写入
         │   App 写入，Bootloader读 │
0x010000 ├─────────────────────────┤
         │   Backup (64KB)         │  旧固件备份（回滚预留）
0x020000 ├─────────────────────────┤
         │   Meta (4KB)            │  版本号、CRC32、固件大小
0x021000 ├─────────────────────────┤
         │   Free (~3.8MB)         │  日志、配置文件预留
0x400000 └─────────────────────────┘
```

### 2.3 OTA Flag 标志位定义

Flag 区（`0x0800FC00`，1KB）存储两个 word：

| 偏移 | 字段 | 说明 |
| --- | --- | --- |
| +0 | state (4B) | 状态魔数（见下表） |
| +4 | boot_count (4B) | BOOT_COUNTING 时的启动尝试次数 |

| state 值 | 名称 | 写入方 | 含义 |
| --- | --- | --- | --- |
| `0xFFFFFFFF` | EMPTY | — | 擦除后默认值，无升级 |
| `0xAA55AA55` | UPGRADE_PENDING | OTA Task (App) | 新固件已校验通过，待 Bootloader 拷贝 |
| `0xC3A5C3A5` | BOOT_COUNTING | Bootloader | 新固件已拷贝，等待 App 确认（超过 3 次 → 回滚） |
| `0x5A5AA5A5` | CONFIRMED | App / Bootloader | App 正常运行确认 / 回滚完成标记 |
| `0x55AA55AA` | DONE | — | 兼容旧版，等同 CONFIRMED |

**写入机制（STM32F103 半字写入）：**

```c
// Flash_If_SetFlagEx(state, boot_count) 实现：
// 先擦除 Flag 页，再依次写入两个 4-byte word（各拆成两次 16-bit 写入）
Flash_If_Erase(FLASH_FLAG_ADDR, 1);
Flash_If_Write(FLASH_FLAG_ADDR,      &state,      4);  // word0: state
Flash_If_Write(FLASH_FLAG_ADDR + 4,  &boot_count, 4);  // word1: boot_count
```

> STM32F103 内部 Flash **不支持 32-bit 一次写入**，只能按 16-bit 半字写。

---

## 三、Bootloader 实现细节

### 3.1 独立工程要点

Bootloader 是独立的 CMake 工程（`Bootloader/CMakeLists.txt`），与 App 分开编译：

```text
Bootloader 依赖：HAL（Flash/SPI/GPIO/RCC） + CMSIS + 启动文件
Bootloader 不包含：FreeRTOS、USART DMA、任何 App 业务代码
```

**链接脚本差异：**

| | Bootloader | App |
| --- | --- | --- |
| 链接脚本 | `STM32F103C8Tx_BOOT.ld` | `STM32F103C8Tx_FLASH.ld` |
| Flash 起始 | `0x08000000` | `0x08002000` |
| Flash 大小 | `8KB` | `55KB` |
| VTOR | `0x08000000`（默认） | `0x08002000`（需设置） |

### 3.2 最小化 UART 调试输出（寄存器级）

Bootloader 不引入 HAL UART 驱动（减小体积），使用寄存器直接操作：

```c
static void Boot_UART_Init(void)
{
    // 开时钟（USART1 挂在 APB2）
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN;

    // PA9（TX）配置为复用推挽输出 50MHz
    // CRH[7:4] 对应 PA9，值 0xB = 1011b（AF_PP 50MHz）
    GPIOA->CRH &= ~(0xFU << 4);
    GPIOA->CRH |=  (0xBU << 4);

    // 波特率：BRR = fclk / baudrate = 8MHz / 115200 ≈ 69
    USART1->BRR = 69U;
    USART1->CR1 = USART_CR1_TE | USART_CR1_UE;
}
```

> **为什么 BRR=69？**
> Bootloader 运行在 HSI 8MHz（无 PLL），不是 App 的 64MHz。
> 同一个 BRR 值在不同时钟下波特率完全不同，两者不可混用。

### 3.3 跳转到 App 的完整步骤

```c
static void Boot_JumpToApp(void)
{
    // 1. 读取 App 的初始 MSP 和复位向量
    //    向量表布局：[0]=MSP, [4]=Reset_Handler
    uint32_t app_msp   = *(__IO uint32_t *)0x08002000;
    uint32_t app_reset = *(__IO uint32_t *)0x08002004;

    // 2. 校验 MSP 是否在合理的 SRAM 范围
    //    STM32F103 SRAM: 0x20000000 ~ 0x20004FFF (20KB)
    if ((app_msp & 0x2FFE0000) != 0x20000000) {
        // App 区空或损坏，拒绝跳转
        return;
    }

    // 3. 关中断，防止跳转过程中意外触发
    __disable_irq();

    // 4. 停 SysTick（否则 App 进入后可能触发 Bootloader 的 Handler）
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    // 5. 复位并禁用 SPI1（让 App 重新初始化）
    __HAL_RCC_SPI1_FORCE_RESET();
    __HAL_RCC_SPI1_RELEASE_RESET();
    __HAL_RCC_SPI1_CLK_DISABLE();

    // 6. 切换向量表到 App
    SCB->VTOR = 0x08002000;

    // 7. 设置主栈指针，跳转
    __set_MSP(app_msp);
    ((void (*)(void))app_reset)();
}
```

### 3.4 防"空 Flash 死循环"保护

W25Q32 全新或 SPI 故障时，读回全为 `0xFF`。此时若直接拷贝，
会把 App 区写满 `0xFF`，导致 MSP=0xFFFFFFFF，永远无法启动。

**防护代码：**

```c
// 探测 W25Q32 前 8 字节
uint8_t probe[8] = {0};
Boot_W25Q_Read(0, probe, 8);

// 全为 0xFF → SPI 故障或 Flash 为空，跳过拷贝
uint8_t all_ff = 1;
for (int i = 0; i < 8; i++) {
    if (probe[i] != 0xFF) { all_ff = 0; break; }
}
if (all_ff) {
    // 清除 PENDING 标志，防止下次复位再次擦除 App
    HAL_FLASHEx_Erase(&erase_flag_page, &err);
    goto do_jump;  // 直接跳转（若 App 有效则正常启动）
}
```

---

## 四、OTA 协议设计

### 4.1 帧格式

```text
┌─────┬─────┬───────┬───────┬───────────────┬────────┐
│ SOF │ CMD │ SEQ   │ LEN   │ PAYLOAD       │ CRC16  │
│ 1B  │ 1B  │ 2B    │ 2B    │ 0 ~ 248B      │ 2B     │
└─────┴─────┴───────┴───────┴───────────────┴────────┘
 0xAA        大端序  大端序                   大端序
```

| 字段 | 说明 |
| --- | --- |
| SOF | 帧起始符，固定 `0xAA`，用于 DMA IDLE 检测后的帧定位 |
| CMD | 命令类型（见下表） |
| SEQ | 包序号，大端序，从 0 递增，END 帧为最后 DATA 序号 +1 |
| LEN | PAYLOAD 字节数，大端序 |
| PAYLOAD | 数据内容（START/DATA 有，END/ABORT 无） |
| CRC16 | CRC16-CCITT，覆盖 `[CMD + SEQ + LEN + PAYLOAD]` |

**命令类型：**

| CMD | 值 | PAYLOAD | 说明 |
| --- | --- | --- | --- |
| START | `0x01` | `[fw_size:4][fw_crc32:4]`（大端序） | 开始升级，携带总大小和期望 CRC32 |
| DATA | `0x02` | 固件原始数据（1~248 字节） | 固件分包，顺序发送 |
| END | `0x03` | 空 | 发送完毕，触发 CRC32 最终校验 |
| ABORT | `0x04` | 空 | 中止升级，状态机回到 IDLE |

### 4.2 单帧最大尺寸限制（248 字节）

```text
DMA 缓冲区 = 256 字节
帧头开销   = SOF(1) + CMD(1) + SEQ(2) + LEN(2) + CRC16(2) = 8 字节
最大 PAYLOAD = 256 - 8 = 248 字节
```

DMA 设置为每次最多接收 256 字节，UART IDLE 中断触发时停止，
保证一次 DMA 事件对应恰好一帧数据，简化解析逻辑。

### 4.3 START 帧的 PAYLOAD 解析

```c
// START payload: 8 字节，大端序
uint32_t fw_size  = ((uint32_t)payload[0] << 24) |
                    ((uint32_t)payload[1] << 16) |
                    ((uint32_t)payload[2] << 8)  |
                     (uint32_t)payload[3];

uint32_t fw_crc32 = ((uint32_t)payload[4] << 24) |
                    ((uint32_t)payload[5] << 16) |
                    ((uint32_t)payload[6] << 8)  |
                     (uint32_t)payload[7];
```

---

## 五、OTA 状态机

### 5.1 状态转移图

```text
                    ┌──────────────────────────────┐
                    │             IDLE             │
                    └──────────────┬───────────────┘
                                   │ CMD_OTA_START
                                   │ 擦除 W25Q32 Download 区
                                   ▼
                    ┌──────────────────────────────┐
                    │           STARTED            │
                    └──────────────┬───────────────┘
                                   │ CMD_OTA_DATA (seq=0)
                                   │ 写入 W25Q32，滚动计算 CRC32
                                   ▼
                    ┌──────────────────────────────┐
              ┌────►│         RECEIVING            │◄────┐
              │     └──────────────┬───────────────┘     │
              │                    │ CMD_OTA_DATA         │
              └────────────────────┘ (seq 递增)           │
                                   │ CMD_OTA_END          │
                                   │ 校验 CRC32 + 大小    │
                                   ▼                      │
                    ┌──────────────────────────────┐      │
                    │ 写 OTA_FLAG_UPGRADE_PENDING  │      │
                    │ HAL_NVIC_SystemReset()        │      │
                    └──────────────────────────────┘      │
                                                          │
任意状态 ──── CMD_OTA_ABORT ──────────────────────► IDLE ─┘
任意状态 ──── CRC16 错误 / 序号不连续 ─────────► ERROR（等待 ABORT）
```

### 5.2 关键状态处理逻辑

**START 帧处理：**

```c
case OTA_STATE_IDLE:
    if (cmd == CMD_OTA_START && payload_len == 8) {
        ota_ctx.fw_size  = parse_u32_be(payload);
        ota_ctx.fw_crc32 = parse_u32_be(payload + 4);
        ota_ctx.bytes_received = 0;
        ota_ctx.crc32_calc     = 0xFFFFFFFF;  // CRC32 初值
        ota_ctx.next_seq       = 0;

        // 擦除 W25Q32 Download 区（64KB = 1个 Block64）
        W25Qxx_EraseBlock64(W25QXX_OTA_DOWNLOAD_ADDR);

        printf("[OTA] READY\r\n");
        state = OTA_STATE_RECEIVING;
    }
    break;
```

**DATA 帧处理：**

```c
case OTA_STATE_RECEIVING:
    if (cmd == CMD_OTA_DATA) {
        // 1. 序号校验
        if (seq != ota_ctx.next_seq) {
            printf("[OTA] NACK seq=%u (expected %u)\r\n", seq, ota_ctx.next_seq);
            state = OTA_STATE_ERROR;
            break;
        }
        // 2. 写入 W25Q32
        uint32_t write_addr = W25QXX_OTA_DOWNLOAD_ADDR + ota_ctx.bytes_received;
        W25Qxx_Write(write_addr, payload, payload_len);

        // 3. 滚动计算 CRC32
        ota_ctx.crc32_calc = crc32_update(ota_ctx.crc32_calc, payload, payload_len);
        ota_ctx.bytes_received += payload_len;

        printf("[OTA] ACK seq=%u (%lu/%lu bytes)\r\n",
               seq, ota_ctx.bytes_received, ota_ctx.fw_size);
        ota_ctx.next_seq++;
    }
    break;
```

**END 帧处理：**

```c
case OTA_STATE_RECEIVING:
    if (cmd == CMD_OTA_END) {
        // 1. 大小校验
        if (ota_ctx.bytes_received != ota_ctx.fw_size) {
            printf("[OTA] ERR: size mismatch\r\n");
            state = OTA_STATE_ERROR;
            break;
        }
        // 2. CRC32 最终校验
        uint32_t final_crc = ota_ctx.crc32_calc ^ 0xFFFFFFFF;
        if (final_crc != ota_ctx.fw_crc32) {
            printf("[OTA] ERR: CRC32 mismatch\r\n");
            state = OTA_STATE_ERROR;
            break;
        }
        // 3. 写升级标志 + 复位
        printf("[OTA] CRC32 OK (0x%08lX). Writing upgrade flag...\r\n", final_crc);
        Flash_If_SetFlag(OTA_FLAG_UPGRADE_PENDING);
        HAL_Delay(100);
        printf("[OTA] DONE. Resetting in 1s...\r\n");
        HAL_Delay(1000);
        HAL_NVIC_SystemReset();
    }
    break;
```

---

## 六、CRC 校验机制

### 6.1 两层校验

| 层次 | 算法 | 覆盖范围 | 目的 |
| --- | --- | --- | --- |
| 帧完整性 | CRC16-CCITT（初值 0xFFFF，多项式 0x1021） | 每帧 `[CMD..PAYLOAD]` | 检测单帧传输错误 |
| 固件完整性 | CRC32 IEEE 802.3（zlib 标准） | 所有 DATA payload 累积 | 检测固件完整性/篡改 |

### 6.2 CRC16-CCITT 实现

```c
uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
        }
    }
    return crc;
}
```

### 6.3 CRC32 滚动计算

CRC32 **不是等所有数据收完才计算**，而是随每个 DATA 帧滚动累积：

```c
// 每收到一个 DATA 帧就更新
static uint32_t crc32_update(uint32_t crc, const uint8_t *data, uint32_t len)
{
    crc = ~crc;  // 等价于 XOR 初值 0xFFFFFFFF
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc & 1) ? ((crc >> 1) ^ 0xEDB88320) : (crc >> 1);
        }
    }
    return ~crc;  // 最终取反
}

// END 帧时：final_crc == fw_crc32 则通过
```

> **与 Python 侧一致性验证：**
> Python `zlib.crc32(data) & 0xFFFFFFFF` 与上述 STM32 实现结果完全一致，
> 均为 CRC32 IEEE 802.3（小端位序）。

### 6.4 为什么不用 STM32 硬件 CRC？

STM32F103 内置 CRC32 硬件单元，但：

- 硬件 CRC32 使用大端位序，与 PC 侧（小端）结果不同
- 需要额外的字节序转换，反而增加复杂度
- 软件实现在此场景（< 64KB 固件）速度完全够用

---

## 七、调试中踩过的坑

### 坑 1：SysTick_Handler 是弱符号，1ms 后死机

**现象：** Bootloader 串口只输出第一行 `===== Bootloader =====`，之后无任何输出，也不执行任何逻辑。

**根本原因：**

```text
startup_stm32f103xb.s:
  .thumb_set SysTick_Handler, Default_Handler
  Default_Handler:
      b  Default_Handler   ← 死循环

HAL_Init() 内部开启 SysTick 1ms 中断
→ 1ms 后触发 SysTick_Handler
→ 跳入 Default_Handler 死循环
→ CPU 完全停止（无看门狗）
```

**修复：** 在 Bootloader/main.c 中定义强符号覆盖弱定义：

```c
void SysTick_Handler(void) { HAL_IncTick(); }
```

**教训：** 使用 HAL 的任何工程，必须确保 `SysTick_Handler` 有实现。
独立 Bootloader 工程不会自动包含 App 的中断处理文件（`stm32f1xx_it.c`）。

---

### 坑 2：W25Qxx_Init() 未调用，SPI 操作静默失败

**现象：** T2/T3 测试均显示 PASS，但 T4（Bootloader 拷贝）时 W25Q32 读回全 `0xFF`。

**根本原因：**

```text
main.c 未调用 W25Qxx_Init()
→ hspi1.Instance = NULL（全零）
→ HAL_SPI_Transmit/Receive: 检查 hspi1.State != HAL_SPI_STATE_READY
→ 直接返回 HAL_ERROR，未向上传递
→ W25Qxx_Write() 内部 WritePage 失败，但没有返回值供调用方检查
→ ota_task 的 CRC32 校验对象是 UART 接收缓冲区数据，而非 W25Q32 回读
→ T2/T3 "假通过"
```

**修复：**

```c
int main(void) {
    HAL_Init();
    SystemClock_Config();
    // ...其他初始化...

    if (W25Qxx_Init() != 0) {           // 加返回值检查
        printf("[System] ERROR: W25Q32 init failed!\r\n");
        while (1);                       // 初始化失败立即停止
    }

    xTaskCreate(...);
    vTaskStartScheduler();
}
```

**教训：**

- 外设初始化必须在调度器前完成并检查返回值
- CRC 校验的数据源必须是写入目标（W25Q32 回读），而非写入源（UART 缓冲区）
- HAL 函数失败时静默返回错误码，不会主动报告，必须手动检查

---

### 坑 3：T2/T3 假通过的识别方法

**判断依据：** Bootloader 打印 `W25Q32[0..7]` 的内容

| 内容 | 含义 |
| --- | --- |
| `FF FF FF FF FF FF FF FF` | SPI 失败或从未写入，T2/T3 是假通过 |
| `55 AA 55 AA 55 AA 55 AA` | T2（假固件）写入成功，T2 真通过 |
| `00 50 00 20 D5 2A 00 08` | T4（真实固件）写入成功，向量表头部（MSP+Reset Vector） |

---

### 坑 4：Bootloader 时钟与 App 时钟不同

| | Bootloader | App |
| --- | --- | --- |
| 时钟源 | HSI（8MHz，无 PLL） | HSI/2 × PLL16 = 64MHz |
| UART BRR | 69（115200 @ 8MHz） | 由 HAL 自动计算 |
| SPI 速率 | APB2/4 = 2MHz | APB2/4 = 16MHz |

Bootloader 使用 `system_boot.c`（不启用 PLL），App 使用 `system_stm32f1xx.c`（配置 PLL）。

---

### 坑 5：App 向量表必须设置 VTOR 偏移

App 烧录在 `0x08002000`，但 CPU 上电默认向量表在 `0x08000000`。
若不设置 VTOR，App 的中断向量找不到，第一个中断（如 SysTick）触发就 HardFault。

**配置位置：** `Core/Src/system_stm32f1xx.c`

```c
#define USER_VECT_TAB_ADDRESS
#define VECT_TAB_OFFSET  0x00002000U   // App 偏移量
// 等效于 SCB->VTOR = 0x08000000 + 0x2000 = 0x08002000
```

---

## 八、完整升级流程时序图（Phase 7，含回滚）

```text
PC (ota_mqtt_sender.py)   ESP32-S3      STM32 App (ota_task v4.0)   Bootloader
        │                    │                   │                       │
        │─ MQTT publish ─────►│                  │                       │
        │  cmd topic          │── UART ──────────►│                       │
        │  START 帧           │                  │备份当前App→W25Q32 Backup│
        │                     │                  │擦 W25Q32 Download     │
        │                     │◄── UART ─────────│  (~3.7s)              │
        │◄─ MQTT status ──────│                  │                       │
        │   [OTA] READY       │                  │                       │
        │                     │                  │                       │
        │─ MQTT DATA seq=0 ───►│── UART ─────────►│写 W25Q32,累积CRC32    │
        │◄─ MQTT ACK seq=0 ───│◄── UART ─────────│                       │
        │        ...          │        ...        │        ...            │
        │─ MQTT DATA seq=N ───►│── UART ─────────►│                       │
        │◄─ MQTT ACK seq=N ───│◄── UART ─────────│                       │
        │                     │                  │                       │
        │─ MQTT END ──────────►│── UART ─────────►│校验 CRC32             │
        │◄─ MQTT CRC32 OK ────│◄── UART ─────────│写 Flag=PENDING+count=0│
        │                     │                  │SystemReset()          │
        │                     │                  │         ──────────────►│
        │                     │                  │                  ┌─────────────────┐
        │                     │                  │                  │读 Flag=PENDING   │
        │                     │                  │                  │探测 W25Q32[0..7] │
        │                     │                  │                  │擦 App(55KB)      │
        │                     │                  │                  │拷贝 Download→Flash│
        │                     │                  │                  │写 Flag=           │
        │                     │                  │                  │  BOOT_COUNTING(1) │
        │                     │                  │                  └────────┬─────────┘
        │                     │                  │◄──────────── 跳转 App     │
        │◄─ MQTT status ──────│◄── UART ─────────│ confirmed OK             │
        │                     │                  │ 写 Flag=CONFIRMED        │
        │                     │                  │ Task started v4.0 ✅     │

--- 回滚路径（新固件 3 次启动失败）---

        │                     │                  │                  ┌─────────────────┐
        │                     │                  │                  │读Flag=BOOT_COUNT │
        │                     │                  │                  │count=03≥3        │
        │                     │                  │                  │擦 App(55KB)      │
        │                     │                  │                  │拷贝 Backup→Flash │
        │                     │                  │                  │写 Flag=CONFIRMED │
        │                     │                  │                  └────────┬─────────┘
        │                     │                  │◄──────────── 跳转旧App   │
        │                     │                  │ Task started v4.0 ✅     │
        │                     │                  │（旧固件已恢复）           │
```

---

## 九、面试常见问题

### Q1：Bootloader 和 App 如何共享同一个 UART 接口？

**答：** 它们是顺序执行的，不会同时运行。Bootloader 使用寄存器级 UART（无 DMA）做调试输出，跳转到 App 前会关闭 SysTick 并复位外设。App 重新初始化 UART（HAL + DMA），从 Bootloader 手里"接管"硬件。

### Q2：为什么 OTA 固件存在外部 Flash 而不是内部 Flash 的另一半？

**答：** STM32F103C8T6 只有 64KB 内部 Flash，放一份 App 大约需要 ~23KB，再加 Bootloader(8KB) 和 Flag(1KB)，剩余空间不足以存第二份。外部 W25Q32 有 4MB，可以轻松存放 Download 区、Backup 区和日志，且不占用宝贵的内部 Flash 空间。

### Q3：如果升级过程中断电怎么办（防砖保护）？

**答：** 本项目（Phase 7）已实现完整的防砖 + 回滚机制：

**传输阶段断电（OTA Task 正在接收）：**

| 断电时机 | 结果 | 恢复方式 |
| --- | --- | --- |
| 写 W25Q32 期间 | Flag 未设 PENDING，Bootloader 正常启动旧 App | 自动恢复 ✅ |
| W25Q32 写完，Flag 写 PENDING 前 | 同上 | 自动恢复 ✅ |

**Bootloader 拷贝阶段断电：**

| 断电时机 | 结果 | 恢复方式 |
| --- | --- | --- |
| 擦 App 期间 / 拷贝 Flash 期间 | Flag 仍 PENDING，W25Q32 Backup 有旧固件 | 下次复位继续拷贝（幂等）✅ |
| 拷贝完，写 BOOT_COUNTING 前 | App 区已是新固件，重启后 Bootloader 再次拷贝（可能重复但无害） | 自动恢复 ✅ |

**新固件运行崩溃（boot_count 机制）：**

| 情况 | 机制 | 结果 |
| --- | --- | --- |
| 新固件 MSP 无效（完全不能运行） | Bootloader JumpToApp 失败 → 不跳转 → 手动复位 → count++ | 3 次后自动回滚 ✅ |
| 新固件能启动但立刻崩溃 | IWDG 看门狗超时复位 → count++ | 3 次后自动回滚（需启用 IWDG，Phase 8 实现） |
| 新固件正常 | OTA Task 写 CONFIRMED，count 清零 | 正常运行 ✅ |

**回滚源**：OTA START 时 App 已将当前固件备份到 W25Q32 Backup 区（`0x010000`），Bootloader 回滚时从该区域恢复。

### Q4：CRC16 和 CRC32 为什么要两套？

**答：**

- CRC16 覆盖单帧，实时校验，**发现错误立即 NACK**，不等到最后。好处是及时反馈，避免继续写入损坏数据。
- CRC32 覆盖全部固件，**END 帧时最终确认**。好处是对整体完整性提供更强保证（CRC32 碰撞概率远低于 CRC16）。

两者配合：CRC16 保证每帧传输可靠，CRC32 保证最终固件不被篡改或截断。

### Q5：VTOR 是什么？为什么 App 需要设置它？

**答：** VTOR（Vector Table Offset Register，`SCB->VTOR`）是 Cortex-M3 的一个寄存器，告诉 CPU"中断向量表在哪个地址"。

- CPU 上电默认 VTOR = 0，向量表在 `0x08000000`（Bootloader 所在地址）
- App 的向量表在 `0x08002000`，如果不设置 VTOR，CPU 仍然去 `0x08000000` 查找中断，最终 HardFault
- 设置方法：在 `system_stm32f1xx.c` 中配置 `VECT_TAB_OFFSET = 0x2000`，系统初始化时自动执行 `SCB->VTOR = 0x08000000 + 0x2000`
