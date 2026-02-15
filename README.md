# IoT2 — STM32 OTA 空中升级项目

**MCU:** STM32F103C8T6 (Cortex-M3, 72MHz, 64KB Flash, 20KB RAM)
**外部存储:** W25Q32 SPI NOR Flash (4MB)
**RTOS:** FreeRTOS v10.0.1
**方案:** 内部 Bootloader + 外部 Download（方案 A）

---

## 项目进度总览

| 阶段 | 内容 | 状态 |
| --- | --- | --- |
| Phase 1 | RTOS 框架 + UART/DMA 双缓冲通信 | ✅ 完成 |
| Phase 2 | Flash 驱动（内部 flash_if + 外部 w25qxx） | ✅ 完成 |
| Phase 3 | Bootloader（SPI 读取 + 固件拷贝 + 跳转） | ✅ 完成 |
| Phase 4 | OTA Task 完整实现（协议 + 状态机 + CRC32） | ✅ 完成 |
| Phase 5 | MQTT 集成（ESP32 WiFi/MQTT 透明桥） | ✅ 完成 |
| Phase 6 | 端到端测试（MQTT→ESP32→STM32→W25Q32→Bootloader） | ✅ 完成 |
| Phase 7 | 回滚机制（BOOT_COUNTING + W25Q32 Backup 区） | ✅ 完成 |

---

## 测试结果（全部通过）

| 测试 | 内容 | 验证点 | 结果 |
| --- | --- | --- | --- |
| T1 | 系统启动 + Bootloader 跳转 | RTOS 双任务打印 ready，Bootloader VTOR 正确 | ✅ PASS |
| T2 | W25Q32 SPI 读写 + OTA 协议状态机 | 假固件写入 W25Q32，CRC32 校验，Bootloader 拷贝 | ✅ PASS |
| T3 | 错误注入（CRC16 / 序号 / ABORT） | 3 个 Case 全部正确响应 NACK/ERR | ✅ PASS |
| T4 | 完整 OTA 流程（USART1 直连） | 真实固件传输，CRC32 OK，新固件启动 | ✅ PASS |
| T5 | MQTT 端到端 OTA（Python→MQTT→ESP32→STM32） | v3.0→v4.0 固件升级，BOOT_COUNTING→CONFIRMED | ✅ PASS |
| T6 | 回滚机制（假固件 → 3次失败 → 自动回滚） | count=03 触发回滚，从 Backup 恢复，旧固件运行 | ✅ PASS |

**T5/T6 关键输出节选：**

```text
[BOOT] Flag -> BOOT_COUNTING(1)            ← T5: 新固件首次启动
[OTA] New firmware confirmed OK.           ← T5: App 写入 CONFIRMED
[OTA] Task started v4.0.                  ← T5: 升级成功

[BOOT] BOOT_COUNTING: count=03            ← T6: 第3次失败
[BOOT] Too many failed boots! Rolling back ← T6: 触发回滚
[BOOT] Rollback complete. Flag -> CONFIRMED
[OTA] Task started v4.0.                  ← T6: 旧固件恢复运行
```

---

## Flash 分区规划

### 内部 Flash (64KB)

| 分区 | 起始地址 | 大小 | 作用 |
| --- | --- | --- | --- |
| Bootloader | `0x08000000` | 8KB | 上电首先运行，含 SPI 驱动 |
| App | `0x08002000` | 55KB | 主业务代码（FreeRTOS + 各任务） |
| OTA Flag | `0x0800FC00` | 1KB | 升级标志位 |

### 外部 W25Q32 (4MB)

| 分区 | 起始地址 | 大小 | 作用 |
| --- | --- | --- | --- |
| Download | `0x000000` | 64KB | 新固件暂存（App OTA 接收时写入） |
| Backup | `0x010000` | 64KB | 旧固件备份（OTA START 时自动备份，回滚用） |
| Meta | `0x020000` | 4KB | 版本号/CRC32/固件大小（预留） |
| Free | `0x021000` | ~3.8MB | 日志/配置预留 |

### OTA Flag 区（内部 Flash 0x0800FC00）

| 偏移 | 字段 | 说明 |
| --- | --- | --- |
| +0 | state (4B) | 见下表 |
| +4 | boot_count (4B) | BOOT_COUNTING 时的启动尝试次数 |

| state 值 | 含义 |
| --- | --- |
| `0xFFFFFFFF` | 空（擦除后默认值） |
| `0xAA55AA55` | UPGRADE_PENDING — 新固件待拷贝 |
| `0xC3A5C3A5` | BOOT_COUNTING — 新固件已拷贝，等待 App 确认（超过 3 次则回滚） |
| `0x5A5AA5A5` | CONFIRMED — App 已确认正常运行 |
| `0x55AA55AA` | DONE — 兼容旧版 |

---

## OTA 升级完整流程（Phase 7 含回滚）

```text
PC (Python) -- MQTT --> ESP32-S3 -- USART2 --> STM32 App (OTA Task v4.0)
  1. 收 START 帧
     a. 备份当前 App (55KB) 到 W25Q32 Backup 区 (0x010000)  ← 回滚数据源
     b. 擦除 W25Q32 Download 区 (0x000000)
  2. 收 DATA 帧xN -> 逐包写入 W25Q32 Download，滚动计算 CRC32
  3. 收 END 帧 -> 校验 CRC32
  4. Flash_If_SetFlagEx(UPGRADE_PENDING, 0)
  5. HAL_NVIC_SystemReset()

Bootloader（0x08000000）— 正常升级路径
  1. 读 Flag == UPGRADE_PENDING
  2. 初始化 SPI1，探测 W25Q32 前 8 字节（全 0xFF 则跳过）
  3. 擦除 App 区（55KB）
  4. 从 W25Q32 Download (0x000000) 拷贝 -> 内部 Flash App 区
  5. 写 Flag = BOOT_COUNTING(1) -> 跳转 App

App 首次成功启动（OTA Task 初始化完成）
  -> 检测 Flag == BOOT_COUNTING -> 写 Flag = CONFIRMED  ← 确认新固件健康

Bootloader — 回滚路径（新固件连续失败 3 次）
  1. 读 Flag == BOOT_COUNTING, count >= 3
  2. 从 W25Q32 Backup (0x010000) 恢复 -> 内部 Flash App 区
  3. 写 Flag = CONFIRMED -> 跳转旧 App
```

---

## OTA UART 帧协议

USART1 IDLE 检测触发，单帧最大 256 字节（等于 DMA 缓冲区大小）。

```text
[SOF:1][CMD:1][SEQ:2][LEN:2][PAYLOAD:0~248][CRC16:2]

SOF  = 0xAA
CMD  : 0x01=START  0x02=DATA  0x03=END  0x04=ABORT
SEQ  : 包序号（大端序，从 0 递增）
LEN  : PAYLOAD 字节数（大端序）
CRC16: CRC16-CCITT，覆盖 [CMD..PAYLOAD]

START payload (8 bytes): [fw_size:4][fw_crc32:4] 大端序
DATA  payload (1~248 bytes): 固件原始数据
END   payload: 空
```

---

## Phase 1 — RTOS 框架与 UART/DMA 通信

### 任务分配

| 任务 | 优先级 | 栈 | 功能 |
| --- | --- | --- | --- |
| `vOtaProcessTask` | 3（高） | 512 words | OTA 固件接收与升级 |
| `vEsp32CommTask` | 2（中） | 256 words | ESP32-S3 通信转发 |

### UART/DMA 双缓冲架构（Ping-Pong）

```text
ISR（IDLE 检测）                    任务（Deferred Processing）
1. 停 DMA                           取信号量（阻塞等待）
2. 计算帧长度                        读 buf[1 - g_uart_rx_buf_idx]
3. 切换 buf 索引  --信号量-->         处理数据
4. 在新 buf 上重启 DMA
5. xSemaphoreGiveFromISR
```

ISR 永远只写"当前 buf"，任务永远只读"另一个 buf"，无需加锁。

---

## Phase 2 — Flash 驱动

### 内部 Flash（flash_if.c）

- `Flash_If_Erase(addr, pages)` — 页擦除（1KB/页）
- `Flash_If_Write(addr, src, len)` — 半字写入（STM32F103 只支持 16-bit 编程）
- `Flash_If_SetFlag(flag)` / `Flash_If_GetFlag()` — OTA 标志位管理

### 外部 W25Q32（w25qxx.c）

- `W25Qxx_Init()` — JEDEC ID 检测（支持 W25Q32/64/128），**必须在 FreeRTOS 启动前调用**
- `W25Qxx_Write(addr, data, len)` — 任意长度写入（自动分页）
- `W25Qxx_EraseBlock64(addr)` — 64KB block 擦除
- `W25Qxx_Read(addr, buf, len)` — 任意长度读取

> **注意：** `W25Qxx_Init()` 必须在 `xTaskCreate` 之前调用，否则 `hspi1` 句柄
> 全零，后续所有 SPI 操作静默失败（HAL 检查 State != READY 直接返回错误）。

---

## Phase 3 — Bootloader

独立 CMake 工程，无 FreeRTOS，烧录在 `0x08000000`（8KB 以内）。

启动序列：

1. 读 `0x0800FC00` 标志位
2. 若为 `OTA_FLAG_UPGRADE_PENDING`（`0xAA55AA55`）：初始化 SPI → 探测前 8 字节（防空 Flash） → 擦 App 区 → 拷贝固件 → 写 `OTA_FLAG_UPGRADE_DONE`
3. 设置 MSP + 跳转 `0x08002000`

**关键修复（调试中发现）：**

| 问题 | 根因 | 修复 |
| --- | --- | --- |
| 复位后无任何串口输出 | `SysTick_Handler` 被 `startup.s` alias 到 `Default_Handler`（死循环），`HAL_Init()` 开启 SysTick 1ms 后立即挂死 | 在 Bootloader/main.c 添加 `void SysTick_Handler(void){ HAL_IncTick(); }` 覆盖弱定义 |
| W25Q32 读回全 0xFF | App 从未调用 `W25Qxx_Init()`，`hspi1.Instance=0`，所有 SPI 操作静默失败 | main.c 加 `W25Qxx_Init()` 调用 |
| Bootloader 擦 App 后无法跳转 | W25Q32 写失败 → 读回全 0xFF → 拷贝 0xFF → MSP=0xFFFFFFFF → 校验失败 | 上述两点修复后自动解决 |

---

## Phase 4 — OTA Task 完整实现

`App/OTA/Src/ota_task.c` — 状态机 + 协议解析 + CRC 校验

### 状态机

```text
IDLE
 | CMD_OTA_START（擦 W25Q32 Download 区）
 v
STARTED
 | CMD_OTA_DATA（seq=0）
 v
RECEIVING -- CMD_OTA_DATA（seq=1,2,...）--> RECEIVING
 | CMD_OTA_END（大小 + CRC32 双重校验）
 v
[设 OTA Flag -> SystemReset -> Bootloader]

任意状态 + CMD_OTA_ABORT --> IDLE
序号不连续 / 溢出 / CRC16 错误 --> ERROR（等待 ABORT）
```

### 校验机制

| 层次 | 算法 | 覆盖范围 |
| --- | --- | --- |
| 帧完整性 | CRC16-CCITT | 每帧 [CMD..PAYLOAD] |
| 固件完整性 | CRC32（IEEE 802.3） | 全部固件，END 时最终校验 |

---

## 硬件配置

| 接口 | 引脚 | 用途 |
| --- | --- | --- |
| USART1 | PA9(TX) / PA10(RX) | 调试 + OTA 接收（115200 baud） |
| USART2 | PA2(TX) / PA3(RX) | ESP32-S3 通信 |
| SPI1 | PA5(SCK) / PA6(MISO) / PA7(MOSI) / PA4(CS) | W25Q32（Mode0，APB2/4=16MHz） |

**时钟配置：**

- App：HSI/2 × PLL16 = 64MHz（`system_stm32f1xx.c`，`VECT_TAB_OFFSET=0x2000`）
- Bootloader：HSI 8MHz（无 PLL，`system_boot.c`，VTOR 在 `0x08000000`）

---

## 测试脚本（tests/ota_mqtt_sender.py）

依赖：`pip install paho-mqtt`，需要 MQTT Broker（如 Mosquitto）在局域网运行。

```bash
# 协议状态机测试（假固件 1KB）
python tests/ota_mqtt_sender.py --broker 192.168.0.3 --test-protocol

# 错误注入测试（CRC16 / 序号跳变 / ABORT 恢复）
python tests/ota_mqtt_sender.py --broker 192.168.0.3 --test-error-injection

# 完整 OTA（真实固件）
python tests/ota_mqtt_sender.py --broker 192.168.0.3 --firmware build/STM32_AIoT_Project.bin

# 调整帧间延迟（默认 100ms）
python tests/ota_mqtt_sender.py --broker 192.168.0.3 --firmware build/STM32_AIoT_Project.bin --inter-frame-ms 50
```

**MQTT 主题：**

| 主题 | 方向 | 内容 |
| --- | --- | --- |
| `device/stm32_iot2/ota/cmd` | PC → STM32 | OTA 二进制帧（QoS 0） |
| `device/stm32_iot2/ota/status` | STM32 → PC | ACK/NACK/状态文本 |
| `device/stm32_iot2/heartbeat` | STM32 → PC | ESP32 心跳（OTA 期间自动抑制） |

---

## 硬件架构

```text
PC (Python ota_mqtt_sender.py)
    │  WiFi / MQTT (paho-mqtt)
    ▼
[MQTT Broker (Mosquitto)]
    │  WiFi
    ▼
ESP32-S3 (ESP-IDF 5.5.2)
  └─ UART1 GPIO17(TX)/GPIO18(RX) ──────────────────────► STM32F103C8T6
                                                           ├─ USART2 PA2/PA3: ESP32 通信
                                                           ├─ USART1 PA9/PA10: 调试输出
                                                           └─ SPI1 PA4/PA5/PA6/PA7: W25Q32
```

---

## 下一步（可选）

- [ ] Phase 8：IWDG 硬件看门狗联动（检测运行中崩溃，触发 boot_count 累加）
- [ ] Phase 9：固件版本信息嵌入（Meta 区 + MQTT 版本上报）
- [ ] Phase 10：远程诊断（FreeRTOS 堆/任务状态定时上报 MQTT）
- [ ] Phase 11：固件签名验证（ECDSA，防止非法固件注入）
