# Bootloader 与 OTA 空中升级技术笔记

> 基于 STM32F103C8T6 + W25Q32 + FreeRTOS + ESP32-S3 实战总结
> 项目：IoT2 OTA 空中升级（方案 A — 外部 Download）

---

## 目录

1. [什么是 Bootloader](#一什么是-bootloader)
2. [Flash 分区规划](#二flash-分区规划)
3. [Bootloader 实现细节](#三bootloader-实现细节)
4. [OTA 协议设计](#四ota-协议设计)
5. [OTA 状态机](#五ota-状态机)
6. [CRC 校验机制](#六crc-校验机制)
7. [ESP32-S3 网关（MQTT 透明桥接）](#七esp32-s3-网关mqtt-透明桥接)
8. [回滚机制（Phase 7）](#八回滚机制phase-7)
9. [调试中踩过的坑](#九调试中踩过的坑)
10. [完整升级流程时序图](#十完整升级流程时序图phase-7含回滚)
11. [面试常见问题](#十一面试常见问题)

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

**对应代码（`flash_if.h`）：**

```c
#define FLASH_BOOT_ADDR   ((uint32_t)0x08000000)
#define FLASH_BOOT_SIZE   ((uint32_t)0x2000)   /* 8KB */
#define FLASH_APP_ADDR    ((uint32_t)0x08002000)
#define FLASH_APP_SIZE    ((uint32_t)0xDC00)   /* 55KB */
#define FLASH_FLAG_ADDR   ((uint32_t)0x0800FC00)
#define FLASH_FLAG_SIZE   FLASH_PAGE_SIZE      /* 1KB */
```

**链接脚本差异：**

```text
# Bootloader (STM32F103C8Tx_BOOT.ld)
FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 8K

# App (STM32F103C8Tx_FLASH.ld)
FLASH (rx) : ORIGIN = 0x08002000, LENGTH = 55K
FLAG  (rw) : ORIGIN = 0x0800FC00, LENGTH = 1K   ← .ota_flag (NOLOAD) 防 App 占用
```

### 2.2 外部 W25Q32（4MB）

```text
0x000000 ┌─────────────────────────┐
         │   Download (64KB)       │  新固件暂存区，OTA Task 写入
         │   App 写入，Bootloader读 │
0x010000 ├─────────────────────────┤
         │   Backup (64KB)         │  旧固件备份（回滚用）
0x020000 ├─────────────────────────┤
         │   Meta (4KB)            │  版本号、CRC32、固件大小
0x021000 ├─────────────────────────┤
         │   Free (~3.8MB)         │  日志、配置文件预留
0x400000 └─────────────────────────┘
```

**对应代码（`w25qxx.h`）：**

```c
#define W25QXX_OTA_DOWNLOAD_ADDR  ((uint32_t)0x000000)
#define W25QXX_OTA_DOWNLOAD_SIZE  ((uint32_t)0x010000)  /* 64KB */
#define W25QXX_OTA_BACKUP_ADDR    ((uint32_t)0x010000)
#define W25QXX_OTA_BACKUP_SIZE    ((uint32_t)0x010000)  /* 64KB */
#define W25QXX_OTA_META_ADDR      ((uint32_t)0x020000)
#define W25QXX_OTA_META_SIZE      ((uint32_t)0x001000)  /* 4KB */
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

**写入实现（`flash_if.c`）— STM32F103 半字写入：**

```c
Flash_If_Status Flash_If_SetFlagEx(uint32_t flag, uint32_t boot_count)
{
    Flash_If_Status ret;

    // 先擦除 Flag 页（1KB），再依次写入两个 word
    ret = Flash_If_Erase(FLASH_FLAG_ADDR, 1);
    if (ret != FLASH_IF_OK) return ret;

    // word0: state（拆成两次 16-bit 半字写入）
    ret = Flash_If_Write(FLASH_FLAG_ADDR,
                         (const uint8_t *)&flag, sizeof(flag));
    if (ret != FLASH_IF_OK) return ret;

    // word1: boot_count
    ret = Flash_If_Write(FLASH_FLAG_ADDR + 4U,
                         (const uint8_t *)&boot_count, sizeof(boot_count));
    return ret;
}
```

> STM32F103 内部 Flash **不支持 32-bit 一次写入**，只能按 16-bit 半字写。
> `Flash_If_Write` 内部将每个 word 拆成两次 `HAL_FLASH_Program(HALFWORD)`。

---

## 三、Bootloader 实现细节

### 3.1 独立工程要点

Bootloader 是独立的 CMake 工程（`Bootloader/CMakeLists.txt`），与 App 分开编译：

```text
Bootloader 依赖：HAL（Flash/SPI/GPIO/RCC） + CMSIS + 启动文件
Bootloader 不包含：FreeRTOS、USART DMA、任何 App 业务代码
```

| | Bootloader | App |
| --- | --- | --- |
| 链接脚本 | `STM32F103C8Tx_BOOT.ld` | `STM32F103C8Tx_FLASH.ld` |
| Flash 起始 | `0x08000000` | `0x08002000` |
| Flash 大小 | `8KB` | `55KB` |
| VTOR | `0x08000000`（默认） | `0x08002000`（需设置） |
| 时钟 | HSI 8MHz（无 PLL） | HSI/2 × PLL16 = 64MHz |

### 3.2 最小化 UART 调试输出（寄存器级）

Bootloader 不引入 HAL UART 驱动（减小体积），使用寄存器直接操作：

```c
// Bootloader/main.c
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

static void Boot_PutChar(char c)
{
    while (!(USART1->SR & USART_SR_TXE));  // 等待 TX 寄存器空
    USART1->DR = (uint8_t)c;
}

static void Boot_Print(const char *s)
{
    while (*s) Boot_PutChar(*s++);
    while (!(USART1->SR & USART_SR_TC));    // 等待传输完成
}
```

> **为什么 BRR=69？**
> Bootloader 运行在 HSI 8MHz（无 PLL），不是 App 的 64MHz。
> 同一个 BRR 值在不同时钟下波特率完全不同，两者不可混用。

### 3.3 跳转到 App 的完整步骤

```c
// Bootloader/main.c
typedef void (*pFunction)(void);

static void Boot_JumpToApp(void)
{
    // 1. 读取 App 的初始 MSP 和复位向量
    //    向量表布局：[0]=MSP, [4]=Reset_Handler
    uint32_t app_msp   = *(__IO uint32_t *)FLASH_APP_ADDR;      // 0x08002000
    uint32_t app_reset = *(__IO uint32_t *)(FLASH_APP_ADDR + 4); // 0x08002004

    // 2. 校验 MSP 是否在合理的 SRAM 范围
    //    STM32F103 SRAM: 0x20000000 ~ 0x20004FFF (20KB)
    if ((app_msp & 0x2FFE0000) != 0x20000000) {
        Boot_Print("[BOOT] ERROR: Invalid MSP! App is empty/corrupt.\r\n");
        return;  // App 区空或损坏，拒绝跳转
    }

    Boot_Print("[BOOT] Jumping to App...\r\n");

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
    SCB->VTOR = FLASH_APP_ADDR;  // 0x08002000

    // 7. 设置主栈指针，跳转
    __set_MSP(app_msp);
    pFunction jump = (pFunction)app_reset;
    jump();

    while (1);  // 不应到达此处
}
```

### 3.4 Bootloader 主逻辑（含回滚）

```c
// Bootloader/main.c — main()
int main(void)
{
    HAL_Init();
    Boot_UART_Init();
    Boot_Print("\r\n===== Bootloader =====\r\n");

    uint32_t flag = Boot_GetFlag();

    if (flag == OTA_FLAG_UPGRADE_PENDING)
    {
        // ★ 正常升级路径
        Boot_SPI_Init();

        // 探测 W25Q32 前 8 字节（防空 Flash 死循环）
        uint8_t probe[8] = {0};
        Boot_W25Q_Read(0, probe, 8);
        uint8_t all_ff = 1;
        for (int i = 0; i < 8; i++) {
            if (probe[i] != 0xFF) { all_ff = 0; break; }
        }
        if (all_ff) {
            // SPI 故障或 Flash 为空，清 PENDING 标志，跳过拷贝
            /* ... 擦除 Flag 页 ... */
            goto do_jump;
        }

        HAL_FLASH_Unlock();
        Boot_EraseApp();                            // 擦除 App 区 55KB
        Boot_CopyW25QToAppFrom(W25Q_DOWNLOAD_ADDR); // 从 Download 拷贝
        Boot_SetFlagEx(OTA_FLAG_BOOT_COUNTING, 1U); // 写 BOOT_COUNTING(1)
        HAL_FLASH_Lock();
    }
    else if (flag == OTA_FLAG_BOOT_COUNTING)
    {
        uint32_t boot_count = Boot_GetBootCount();

        if (boot_count >= OTA_BOOT_COUNT_MAX)  // ≥3 次失败
        {
            // ★ 回滚路径：从 Backup 区恢复旧固件
            Boot_SPI_Init();
            HAL_FLASH_Unlock();
            Boot_EraseApp();
            Boot_CopyW25QToAppFrom(W25Q_BACKUP_ADDR);  // 从 Backup 恢复
            Boot_SetFlagEx(OTA_FLAG_CONFIRMED, 0U);
            HAL_FLASH_Lock();
        }
        else
        {
            // 还有机会，累加计数再试
            HAL_FLASH_Unlock();
            Boot_SetFlagEx(OTA_FLAG_BOOT_COUNTING, boot_count + 1U);
            HAL_FLASH_Lock();
        }
    }

do_jump:
    Boot_JumpToApp();
    while (1);
}
```

### 3.5 防"空 Flash 死循环"保护

W25Q32 全新或 SPI 故障时，读回全为 `0xFF`。此时若直接拷贝，
会把 App 区写满 `0xFF`，导致 MSP=0xFFFFFFFF，永远无法启动。

**防护逻辑：** 读前 8 字节探测，全 `0xFF` 则清除 PENDING 标志并直接跳转。

### 3.6 W25Q32 到内部 Flash 的拷贝函数

```c
// Bootloader/main.c — 支持从 Download 或 Backup 拷贝
static HAL_StatusTypeDef Boot_CopyW25QToAppFrom(uint32_t w25q_src_addr)
{
    uint8_t  page_buf[256];
    uint32_t remaining = FLASH_APP_SIZE;    // 55KB
    uint32_t src_addr  = w25q_src_addr;     // Download=0x000000 或 Backup=0x010000
    uint32_t dest_addr = FLASH_APP_ADDR;    // 0x08002000

    while (remaining > 0) {
        uint32_t chunk = (remaining > 256) ? 256 : remaining;
        Boot_W25Q_Read(src_addr, page_buf, chunk);

        // 按半字写入内部 Flash
        for (uint32_t i = 0; i < chunk; i += 2) {
            uint16_t halfword;
            if (i + 1 < chunk)
                halfword = (uint16_t)page_buf[i] | ((uint16_t)page_buf[i+1] << 8);
            else
                halfword = (uint16_t)page_buf[i] | 0xFF00;  // 奇数字节补 0xFF

            HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, dest_addr, halfword);
            dest_addr += 2;
        }
        src_addr  += chunk;
        remaining -= chunk;
    }
    return HAL_OK;
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

**对应代码（`ota_task.h`）：**

```c
#define OTA_SOF              0xAAU
#define OTA_FRAME_HDR        6U     /* SOF+CMD+SEQ(2)+LEN(2) */
#define OTA_FRAME_CRC        2U
#define OTA_FRAME_OVERHEAD   (OTA_FRAME_HDR + OTA_FRAME_CRC)   /* 8 bytes */
#define OTA_MAX_PAYLOAD      248U   /* 256 - 8 */

#define CMD_OTA_START        0x01U
#define CMD_OTA_DATA         0x02U
#define CMD_OTA_END          0x03U
#define CMD_OTA_ABORT        0x04U
```

### 4.2 单帧最大尺寸限制（248 字节）

```text
DMA 缓冲区 = 256 字节
帧头开销   = SOF(1) + CMD(1) + SEQ(2) + LEN(2) + CRC16(2) = 8 字节
最大 PAYLOAD = 256 - 8 = 248 字节
```

DMA 设置为每次最多接收 256 字节，UART IDLE 中断触发时停止，
保证一次 DMA 事件对应恰好一帧数据，简化解析逻辑。

### 4.3 帧解析入口（`ota_process_frame`）

```c
// ota_task.c — 公共帧处理函数（USART1/USART2 双通道共用）
void ota_process_frame(const uint8_t *buf, uint16_t len,
                       UART_HandleTypeDef *response_uart)
{
    // 互斥锁保护状态机（USART1 和 USART2 可能并发调用）
    if (xSemaphoreTake(xOtaMutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        printf("[OTA] ERR: mutex timeout, frame dropped\r\n");
        return;
    }
    s_response_uart = response_uart;  // 设置回复通道

    // 1. 基本校验：长度、SOF
    if (len < OTA_FRAME_OVERHEAD) { /* ... */ goto cleanup; }
    if (buf[0] != OTA_SOF)        { /* ... */ goto cleanup; }

    // 2. 解析帧头
    uint8_t  cmd     = buf[1];
    uint16_t seq     = ((uint16_t)buf[2] << 8) | (uint16_t)buf[3];
    uint16_t pay_len = ((uint16_t)buf[4] << 8) | (uint16_t)buf[5];

    // 3. CRC16 校验
    uint16_t rx_crc   = ((uint16_t)buf[len-2] << 8) | (uint16_t)buf[len-1];
    uint16_t calc_crc = crc16_ccitt(&buf[1], (uint16_t)(len - 3));
    if (rx_crc != calc_crc) { /* NACK */ goto cleanup; }

    // 4. 根据 CMD 分发处理
    switch (cmd) {
    case CMD_OTA_START: /* ... */ break;
    case CMD_OTA_DATA:  /* ... */ break;
    case CMD_OTA_END:   /* ... */ break;
    case CMD_OTA_ABORT: /* ... */ break;
    }

cleanup:
    s_response_uart = NULL;
    xSemaphoreGive(xOtaMutex);  // ★ goto cleanup 模式确保互斥锁必然释放
}
```

---

## 五、OTA 状态机

### 5.1 状态转移图

```text
                    ┌──────────────────────────────┐
                    │             IDLE             │
                    └──────────────┬───────────────┘
                                   │ CMD_OTA_START
                                   │ 备份 App → W25Q32 Backup
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

### 5.2 START 帧处理（含备份）

```c
// ota_task.c — CMD_OTA_START 处理
case CMD_OTA_START:
{
    if (pay_len != 8U) { /* ERR */ goto cleanup; }

    // 解析固件大小和 CRC32（大端序）
    uint32_t fw_size = ((uint32_t)payload[0] << 24) | ((uint32_t)payload[1] << 16) |
                       ((uint32_t)payload[2] <<  8) |  (uint32_t)payload[3];
    uint32_t fw_crc  = ((uint32_t)payload[4] << 24) | ((uint32_t)payload[5] << 16) |
                       ((uint32_t)payload[6] <<  8) |  (uint32_t)payload[7];

    if (fw_size == 0U || fw_size > W25QXX_OTA_DOWNLOAD_SIZE) { /* ERR */ goto cleanup; }

    // ★ Phase 7: 备份当前 App 到 W25Q32 Backup 区（回滚数据源）
    ota_printf("[OTA] Backing up current App (55KB) to W25Q32 Backup...\r\n");
    W25Qxx_EraseBlock64(W25QXX_OTA_BACKUP_ADDR);
    {
        uint8_t  bak_buf[256];
        uint32_t bak_offset = 0U;
        while (bak_offset < FLASH_APP_SIZE) {
            uint32_t chunk = ((FLASH_APP_SIZE - bak_offset) > 256U)
                             ? 256U : (FLASH_APP_SIZE - bak_offset);
            Flash_If_Read(FLASH_APP_ADDR + bak_offset, bak_buf, chunk);
            W25Qxx_Write(W25QXX_OTA_BACKUP_ADDR + bak_offset, bak_buf, chunk);
            bak_offset += chunk;
        }
    }

    // 擦除 Download 区
    W25Qxx_EraseBlock64(W25QXX_OTA_DOWNLOAD_ADDR);

    // 初始化状态
    s_fw_total_size = fw_size;  s_fw_crc32 = fw_crc;
    s_bytes_written = 0;  s_next_seq = 0;  s_running_crc = 0xFFFFFFFFU;
    s_state = OTA_STATE_STARTED;
    ota_printf("[OTA] READY\r\n");
    break;
}
```

### 5.3 DATA 帧处理

```c
// ota_task.c — CMD_OTA_DATA 处理
case CMD_OTA_DATA:
{
    if (s_state != OTA_STATE_STARTED && s_state != OTA_STATE_RECEIVING)
        { /* ERR */ goto cleanup; }
    if (pay_len == 0U) { /* ERR */ goto cleanup; }

    // 序号校验
    if (seq != s_next_seq) {
        ota_printf("[OTA] NACK seq=%u (expected %u)\r\n", seq, s_next_seq);
        s_state = OTA_STATE_ERROR;
        goto cleanup;
    }

    // 溢出检查
    if (s_bytes_written + (uint32_t)pay_len > s_fw_total_size) {
        s_state = OTA_STATE_ERROR;
        goto cleanup;
    }

    // 写入 W25Q32 Download 区
    W25Qxx_Write(W25QXX_OTA_DOWNLOAD_ADDR + s_bytes_written, payload, (uint32_t)pay_len);

    // 滚动计算 CRC32
    s_running_crc = crc32_update(s_running_crc, payload, (uint32_t)pay_len);
    s_bytes_written += (uint32_t)pay_len;
    s_next_seq++;
    s_state = OTA_STATE_RECEIVING;

    ota_printf("[OTA] ACK seq=%u (%lu/%lu bytes)\r\n",
               seq, s_bytes_written, s_fw_total_size);
    break;
}
```

### 5.4 END 帧处理

```c
// ota_task.c — CMD_OTA_END 处理
case CMD_OTA_END:
{
    if (s_state != OTA_STATE_RECEIVING) { /* ERR */ goto cleanup; }

    // 大小校验
    if (s_bytes_written != s_fw_total_size) {
        ota_printf("[OTA] ERR: incomplete\r\n");
        ota_reset(); goto cleanup;
    }

    // CRC32 最终校验
    uint32_t final_crc = s_running_crc ^ 0xFFFFFFFFU;
    if (final_crc != s_fw_crc32) {
        ota_printf("[OTA] ERR: CRC32 mismatch\r\n");
        ota_reset(); goto cleanup;
    }

    // 写升级标志 + 复位
    ota_printf("[OTA] CRC32 OK (0x%08lX). Writing upgrade flag...\r\n", final_crc);
    Flash_If_SetFlagEx(OTA_FLAG_UPGRADE_PENDING, 0U);
    ota_printf("[OTA] DONE. Resetting in 1s...\r\n");

    // 释放互斥锁后延迟复位
    s_response_uart = NULL;
    xSemaphoreGive(xOtaMutex);
    vTaskDelay(pdMS_TO_TICKS(1000));
    HAL_NVIC_SystemReset();
    return;  // 不走 cleanup（已手动释放锁）
}
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
// ota_task.c
static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x8000U) ?
                  (uint16_t)((crc << 1) ^ 0x1021U) :
                  (uint16_t)(crc << 1);
        }
    }
    return crc;
}
```

### 6.3 CRC32 滚动计算

CRC32 **不是等所有数据收完才计算**，而是随每个 DATA 帧滚动累积：

```c
// ota_task.c
static uint32_t crc32_update(uint32_t crc, const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        crc ^= (uint32_t)data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc & 1U) ? (crc >> 1) ^ 0xEDB88320U : (crc >> 1);
        }
    }
    return crc;
}

// 初始值 s_running_crc = 0xFFFFFFFF
// 每个 DATA 帧: s_running_crc = crc32_update(s_running_crc, payload, len)
// END 帧时:     final_crc = s_running_crc ^ 0xFFFFFFFF
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

## 七、ESP32-S3 网关（MQTT 透明桥接）

### 7.1 网关架构

ESP32-S3 作为 WiFi/MQTT 透明桥接网关，不解析 OTA 协议内容，只负责数据搬运：

```text
MQTT Broker ─── WiFi ───► ESP32-S3 ─── UART1 ───► STM32
  cmd 主题                   │                       │
  (二进制帧)                 │                       │
                             │◄── UART1 ──── STM32 ──┘
                             │  (ACK/状态文本)
MQTT Broker ◄── WiFi ───── ESP32-S3
  status 主题
```

### 7.2 ESP32 启动流程（`hello_world_main.c`）

```c
// ESP_Firmware/main/hello_world_main.c
void app_main(void)
{
    // 1. NVS 初始化（WiFi 驱动必需）
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    // 2. 连接 WiFi（阻塞直到成功或失败）
    wifi_station_init();

    // 3. WiFi 连接成功后启动 MQTT 客户端
    if (wifi_is_connected()) {
        mqtt_client_start();
    }

    // 4. 初始化 STM32 通信 UART
    stm32_uart_init();  // UART1, GPIO17(TX)/GPIO18(RX), 115200

    // 5. 创建收发任务
    xTaskCreate(uart_tx_task, "uart_tx", 2048, NULL, 5, NULL);  // 心跳
    xTaskCreate(uart_rx_task, "uart_rx", 2048, NULL, 5, NULL);  // STM32 回复
}
```

### 7.3 UART 初始化（TX buf=512B 关键设计）

```c
// ESP_Firmware/main/hello_world_main.c
static void stm32_uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };

    // ★ TX buffer=512：防止 256 字节 OTA 帧因 FIFO(128B) 限制被拆成两批发送
    // 有了 TX ring buffer，uart_write_bytes(256B) 一次性写入缓冲区，
    // DMA/FIFO 连续搬运，STM32 不会误检测到 IDLE 中断把帧截断。
    uart_driver_install(STM32_UART_NUM,
                        512,    /* RX buffer */
                        512,    /* TX buffer (防止帧截断) */
                        0, NULL, 0);
    uart_param_config(STM32_UART_NUM, &uart_config);
    uart_set_pin(STM32_UART_NUM, 17, 18, -1, -1);
}
```

### 7.4 MQTT 事件回调 — 数据零拷贝陷阱

```c
// ESP_Firmware/main/mqtt_client_task.c
case MQTT_EVENT_DATA:
{
    if (strncmp(event->topic, MQTT_TOPIC_OTA_CMD, event->topic_len) == 0)
    {
        // ★ 关键修复：立即复制 event->data 到栈上
        // event->data 是 MQTT 库内部 ring buffer 的指针，
        // 只要调用 vTaskDelay 或任何可切换任务的函数，
        // MQTT 库就可能用下一个 TCP 包覆盖该 buffer，
        // 导致 uart_write_bytes 发出垃圾数据。
        ota_tx_item_t item;
        memcpy(item.data, event->data, event->data_len);
        item.len = (uint16_t)event->data_len;

        // 根据 CMD 字节更新 session 状态
        if (item.len >= 2 && item.data[0] == OTA_SOF_BYTE) {
            uint8_t cmd = item.data[1];
            if (cmd == CMD_OTA_START)
                s_ota_session_active = true;   // 开始 OTA session
            else if (cmd == CMD_OTA_END || cmd == CMD_OTA_ABORT)
                s_ota_session_active = false;  // 结束 OTA session
        }

        // 入队到 TX 专用任务（不阻塞）
        if (xQueueSend(s_ota_tx_queue, &item, 0) != pdPASS) {
            ESP_LOGE(TAG, "OTA TX queue full, frame dropped!");
        }
    }
    break;
}
```

### 7.5 OTA 帧 UART 发送任务（解耦 MQTT 回调）

```c
// ESP_Firmware/main/mqtt_client_task.c
// 队列消息体：复制一份完整帧数据，彻底脱离 MQTT 内部 buffer
typedef struct {
    uint8_t  data[256];   // OTA_FRAME_MAX
    uint16_t len;
} ota_tx_item_t;

static QueueHandle_t s_ota_tx_queue = NULL;  // 深度 4

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

        // 发送本帧（数据已在 item.data，与 MQTT buffer 完全无关）
        uart_write_bytes(STM32_UART_NUM, item.data, item.len);

        // 等待本帧发完
        uart_wait_tx_done(STM32_UART_NUM, pdMS_TO_TICKS(50));

        s_ota_uart_busy = false;
    }
}
```

### 7.6 心跳抑制机制

```c
// ESP_Firmware/main/hello_world_main.c — uart_tx_task
static void uart_tx_task(void *param)
{
    uint32_t seq = 0;
    char tx_buf[64];

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(2000));

        // ★ OTA session 期间跳过心跳
        // 使用 session 级标志（START→END 整个过程），而非单帧 busy 标志
        // 原因：DATA 帧之间有 ~100ms 间隙，busy 为 false 时心跳仍可能
        //       插入 UART，导致 STM32 IDLE 中断误判帧边界
        if (mqtt_ota_session_active()) {
            continue;
        }

        int len = snprintf(tx_buf, sizeof(tx_buf),
                           "ESP32 heartbeat #%lu\r\n", (unsigned long)seq++);
        uart_write_bytes(STM32_UART_NUM, tx_buf, len);
    }
}
```

### 7.7 STM32 侧 ESP32 帧接收（帧重同步）

```c
// Hardware/ESP32/esp32_comm.c — vEsp32CommTask
void vEsp32CommTask(void *pvParameters)
{
    HAL_UART_Receive_DMA(&huart2, g_uart2_rx_buf[g_uart2_rx_buf_idx], UART2_DMA_BUF_SIZE);

    for (;;) {
        if (xSemaphoreTake(xEsp32Semaphore, pdMS_TO_TICKS(5000)) == pdPASS) {
            uint8_t  proc_idx = 1 - g_uart2_rx_buf_idx;
            uint16_t len      = g_uart2_rx_len;
            if (len == 0) continue;

            // ★ 帧重同步：扫描找到第一个 SOF(0xAA) 字节
            // 正常情况 sof_offset==0；若有残留心跳字节则跳过
            uint16_t sof_offset = 0;
            while (sof_offset < len &&
                   g_uart2_rx_buf[proc_idx][sof_offset] != OTA_SOF)
                sof_offset++;

            if (sof_offset < len) {
                uint8_t  *frame = g_uart2_rx_buf[proc_idx] + sof_offset;
                uint16_t  flen  = len - sof_offset;

                // 调用公共 OTA 帧处理函数，response_uart=&huart2
                ota_process_frame(frame, flen, &huart2);
            } else {
                // 非 OTA 数据（心跳等），回复 OK
                HAL_UART_Transmit(&huart2, (uint8_t *)"OK\r\n", 4, 100);
            }
        }
    }
}
```

---

## 八、回滚机制（Phase 7）

### 8.1 状态机

```text
OTA Task (App)                    Bootloader
─────────────────                 ─────────────────
收到 START 帧
  → 备份 App → W25Q32 Backup
  → 擦除 Download 区
收到 DATA 帧 ×N
  → 写入 Download 区
收到 END 帧
  → CRC32 校验通过
  → SetFlagEx(PENDING, 0)
  → SystemReset()
                                  读 Flag = PENDING
                                    → 探测 W25Q32
                                    → 擦除 App
                                    → 拷贝 Download → Flash
                                    → SetFlagEx(BOOT_COUNTING, 1)
                                    → 跳转 App

App 启动成功
  → 检测 BOOT_COUNTING
  → SetFlagEx(CONFIRMED, 0)
  → 正常运行 ✅

App 启动失败（崩溃/HardFault）     Bootloader 再次运行
                                    → 读 BOOT_COUNTING, count++
                                    → count < 3: 再试
                                    → count ≥ 3: 触发回滚
                                      → 擦除 App
                                      → 拷贝 Backup → Flash
                                      → SetFlagEx(CONFIRMED, 0)
                                      → 跳转旧 App ✅
```

### 8.2 App 启动确认代码

```c
// ota_task.c — vOtaProcessTask 入口
void vOtaProcessTask(void *pvParameters)
{
    // 启动 DMA 接收
    HAL_UART_Receive_DMA(&huart1, g_uart_rx_buf[g_uart_rx_buf_idx], UART_DMA_BUF_SIZE);

    // ★ 回滚看门狗确认：如果当前处于 BOOT_COUNTING 状态，说明是新固件首次成功启动
    if (Flash_If_GetFlag() == OTA_FLAG_BOOT_COUNTING) {
        Flash_If_SetFlagEx(OTA_FLAG_CONFIRMED, 0U);
        printf("[OTA] New firmware confirmed OK. Rollback watchdog cleared.\r\n");
    }

    printf("[OTA] Task started v4.0. Waiting for firmware...\r\n");

    for (;;) {
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

### 8.3 断电安全分析

**传输阶段断电（OTA Task 正在接收）：**

| 断电时机 | 结果 | 恢复方式 |
| --- | --- | --- |
| 写 W25Q32 期间 | Flag 未设 PENDING，Bootloader 正常启动旧 App | 自动恢复 ✅ |
| W25Q32 写完，Flag 写 PENDING 前 | 同上 | 自动恢复 ✅ |

**Bootloader 拷贝阶段断电：**

| 断电时机 | 结果 | 恢复方式 |
| --- | --- | --- |
| 擦 App / 拷贝 Flash 期间 | Flag 仍 PENDING，Backup 有旧固件 | 下次复位继续拷贝（幂等）✅ |
| 拷贝完，写 BOOT_COUNTING 前 | App 区已是新固件，重启再拷贝（无害） | 自动恢复 ✅ |

**新固件运行崩溃（boot_count 机制）：**

| 情况 | 机制 | 结果 |
| --- | --- | --- |
| 新固件 MSP 无效 | Bootloader JumpToApp 失败 → count++ | 3 次后自动回滚 ✅ |
| 新固件能启动但崩溃 | IWDG 看门狗超时复位 → count++ | 3 次后自动回滚（需 Phase 8） |
| 新固件正常 | OTA Task 写 CONFIRMED | 正常运行 ✅ |

---

## 九、调试中踩过的坑

### 坑 1：SysTick_Handler 是弱符号，1ms 后死机

**现象：** Bootloader 串口只输出第一行 `===== Bootloader =====`，之后无任何输出。

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

---

### 坑 2：W25Qxx_Init() 未调用，SPI 操作静默失败

**现象：** T2/T3 测试均显示 PASS，但 T4（Bootloader 拷贝）时 W25Q32 读回全 `0xFF`。

**根本原因：**

```text
main.c 未调用 W25Qxx_Init()
→ hspi1.Instance = NULL（全零）
→ HAL_SPI_Transmit/Receive: 检查 hspi1.State != HAL_SPI_STATE_READY
→ 直接返回 HAL_ERROR，未向上传递
→ ota_task 的 CRC32 校验对象是 UART 接收缓冲区数据，而非 W25Q32 回读
→ T2/T3 "假通过"
```

**修复（`Core/Src/main.c`）：**

```c
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    USART1_UART_Init();
    USART2_UART_Init();

    // ★ 必须在任务创建和调度器启动前初始化 SPI Flash
    if (W25Qxx_Init() != 0) {
        printf("[System] ERROR: W25Q32 init failed! Check SPI wiring.\r\n");
        while (1);
    }

    xOtaSemaphore   = xSemaphoreCreateBinary();
    xEsp32Semaphore = xSemaphoreCreateBinary();
    xOtaMutex       = xSemaphoreCreateMutex();

    xTaskCreate(vOtaProcessTask,  "OTA_Task",   512, NULL, 3, NULL);
    xTaskCreate(vEsp32CommTask,   "ESP32_Comm", 512, NULL, 2, NULL);

    vTaskStartScheduler();
}
```

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

---

### 坑 5：App 向量表必须设置 VTOR 偏移

App 烧录在 `0x08002000`，但 CPU 上电默认向量表在 `0x08000000`。
若不设置 VTOR，第一个中断（如 SysTick）触发就 HardFault。

**配置位置：** `Core/Src/system_stm32f1xx.c`

```c
#define USER_VECT_TAB_ADDRESS
#define VECT_TAB_OFFSET  0x00002000U   // App 偏移量
// 等效于 SCB->VTOR = 0x08000000 + 0x2000 = 0x08002000
```

---

### 坑 6：UART2 DMA 缓冲区必须是 257 而非 256

**现象：** ESP32 发送 256 字节 OTA 帧时，STM32 偶尔收到截断帧。

**根本原因：** DMA 缓冲区设为 256 时，恰好收满 256 字节会同时触发 DMA TC（传输完成）
中断和 UART IDLE 中断，两者竞争导致帧被拆分。

**修复：**

```c
// usart2.h
#define UART2_DMA_BUF_SIZE  257  // 比最大帧 256B 多 1
// DMA TC 永远不会触发（帧最大 256B < 257），
// 所有帧统一由 IDLE 处理，逻辑简洁无竞争
```

> 注意：不可改大到 512，实测会 HardFault（RAM 不足）。

---

### 坑 7：ESP32 心跳与 OTA 帧合并导致数据损坏

**现象：** MQTT OTA 传输过程中，部分 DATA 帧被 STM32 NACK。

**根本原因：** ESP32 心跳任务在 DATA 帧间隙（~100ms）插入心跳文本，
导致 STM32 DMA 在一次 IDLE 事件中收到"心跳+OTA帧"混合数据。

**修复：** 使用 session 级 flag（`s_ota_session_active`）而非单帧 busy flag：

```c
// mqtt_client_task.c — MQTT 回调中设置
if (cmd == CMD_OTA_START)                    s_ota_session_active = true;
else if (cmd == CMD_OTA_END || cmd == CMD_OTA_ABORT)  s_ota_session_active = false;

// hello_world_main.c — 心跳任务检查
if (mqtt_ota_session_active()) continue;  // 整个 OTA session 期间跳过心跳
```

---

### 坑 8：USART ISR 不可调用 HAL_UART_IRQHandler

**现象：** OTA 过程中偶发 DMA 停止接收，之后所有帧丢失。

**根本原因：** `HAL_UART_IRQHandler()` 检测到 ORE（Overrun Error）时会调用
`UART_EndRxTransfer()` 永久禁用 DMA 接收。

**修复（`stm32f1xx_it.c`）：** 手动清除错误标志，不调用 HAL 的 IRQ Handler：

```c
void USART1_IRQHandler(void)
{
    // 手动清除所有错误标志（读 SR 再读 DR）
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE) ||
        __HAL_UART_GET_FLAG(&huart1, UART_FLAG_FE)  ||
        __HAL_UART_GET_FLAG(&huart1, UART_FLAG_NE)  ||
        __HAL_UART_GET_FLAG(&huart1, UART_FLAG_PE))
    {
        volatile uint32_t tmp = huart1.Instance->SR;
        tmp = huart1.Instance->DR;
        (void)tmp;
    }

    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        // ... DMA 停止 + 切换缓冲 + 重启 DMA + 释放信号量 ...
    }

    // ★ 不调用 HAL_UART_IRQHandler()
    // 原因：HAL 检测到 ORE 会调用 UART_EndRxTransfer 永久禁用 DMA
}
```

---

## 十、完整升级流程时序图（Phase 7，含回滚）

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

## 十一、面试常见问题

### Q1：Bootloader 和 App 如何共享同一个 UART 接口？

**答：** 它们是顺序执行的，不会同时运行。Bootloader 使用寄存器级 UART（无 DMA）做调试输出，跳转到 App 前会关闭 SysTick 并复位外设。App 重新初始化 UART（HAL + DMA），从 Bootloader 手里"接管"硬件。

### Q2：为什么 OTA 固件存在外部 Flash 而不是内部 Flash 的另一半？

**答：** STM32F103C8T6 只有 64KB 内部 Flash，放一份 App 大约需要 ~23KB，再加 Bootloader(8KB) 和 Flag(1KB)，剩余空间不足以存第二份。外部 W25Q32 有 4MB，可以轻松存放 Download 区、Backup 区和日志，且不占用宝贵的内部 Flash 空间。

### Q3：如果升级过程中断电怎么办（防砖保护）？

**答：** 本项目已实现完整的防砖 + 回滚机制（见第八章断电安全分析表）。核心原理：

- 传输阶段断电：Flag 未设 PENDING，Bootloader 正常启动旧 App
- 拷贝阶段断电：Flag 仍 PENDING，下次复位继续拷贝（幂等操作）
- 新固件崩溃：boot_count 累加至 3 次后自动从 W25Q32 Backup 回滚

### Q4：CRC16 和 CRC32 为什么要两套？

**答：**

- CRC16 覆盖单帧，实时校验，**发现错误立即 NACK**，不等到最后。好处是及时反馈，避免继续写入损坏数据。
- CRC32 覆盖全部固件，**END 帧时最终确认**。好处是对整体完整性提供更强保证（CRC32 碰撞概率远低于 CRC16）。

两者配合：CRC16 保证每帧传输可靠，CRC32 保证最终固件不被篡改或截断。

### Q5：VTOR 是什么？为什么 App 需要设置它？

**答：** VTOR（Vector Table Offset Register，`SCB->VTOR`）告诉 CPU"中断向量表在哪个地址"。CPU 上电默认在 `0x08000000`（Bootloader 位置），App 的向量表在 `0x08002000`，不设置 VTOR 就会去错误的地址查找中断处理函数，触发 HardFault。

### Q6：ESP32 在 OTA 中的角色是什么？为什么不让 STM32 直接连 WiFi？

**答：** STM32F103 没有 WiFi 硬件，且 RAM 只有 20KB，无法运行 TCP/IP 栈。ESP32-S3 作为透明桥接网关：

- 上行：订阅 MQTT OTA 命令主题 → 通过 UART 二进制透传给 STM32
- 下行：接收 STM32 的 ACK/状态文本 → 发布到 MQTT 状态主题
- ESP32 不解析 OTA 协议内容，只做数据搬运，解耦网络层和固件升级逻辑

### Q7：为什么 MQTT 用 QoS 0 而不是 QoS 1/2？

**答：** OTA 协议自身已有 ACK/重传机制（通过 Python 侧超时重发）。QoS 1/2 会导致 Broker 重传旧帧，而 STM32 状态机已推进到新序号，重传帧会因序号不匹配被 NACK，反而干扰正常流程。

### Q8：`goto cleanup` 模式的作用是什么？

**答：** `ota_process_frame` 使用 `xOtaMutex` 互斥锁保护状态机，锁在函数入口获取。任何错误分支都通过 `goto cleanup` 跳转到统一出口释放锁，避免：

1. 遗忘释放锁导致死锁
2. 多个 return 分支重复写释放代码
3. 这是嵌入式 C 中常见的资源管理模式（类似 Linux 内核风格）
