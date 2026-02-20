# IoT2 — STM32 OTA 空中升级项目

**MCU:** STM32F103C8T6 (Cortex-M3, 72MHz, 64KB Flash, 20KB RAM)
**网关:** ESP32-S3 (ESP-IDF 5.5.2, WiFi + MQTT)
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
| Phase 5 | ESP32-S3 WiFi/MQTT 透明桥接网关 | ✅ 完成 |
| Phase 6 | 回滚机制（BOOT_COUNTING + W25Q32 Backup 区） | ✅ 完成 |
| Phase 7 | 安全加固（TLS + MQTT 认证 + HMAC-SHA256 签名 + 防回滚） | ✅ 完成 |

---

## 硬件架构

```text
PC (Python ota_mqtt_sender.py)
    │  WiFi / MQTT TLS (paho-mqtt, QoS 1)
    ▼
[MQTT Broker (Mosquitto TLS:8883)]  ← 局域网 192.168.0.3
    │  认证: username/password
    │  加密: TLSv1.2 + CA 证书验证
    │  WiFi
    ▼
ESP32-S3 (ESP-IDF 5.5.2)
  ├─ WiFi STA (wifi_station.c)
  ├─ MQTT Client (mqtt_client_task.c)
  │    ├─ 订阅 ota/cmd → 入队 s_ota_tx_queue
  │    ├─ uart_ota_tx_task (prio 6): 出队 → UART1 发送
  │    └─ mqtt_heartbeat_task (prio 3): 10s JSON 心跳
  └─ UART1 GPIO17(TX)/GPIO18(RX) ──────────────► STM32F103C8T6
       115200 8N1, TX buf=512B                     ├─ USART2 PA2/PA3: ESP32 通信
                                                   │   └─ vEsp32CommTask (prio 2)
                                                   │       SOF 帧重同步 → ota_process_frame()
                                                   ├─ USART1 PA9/PA10: PC 调试 + 直连 OTA
                                                   │   └─ vOtaProcessTask (prio 3)
                                                   │       DMA 双缓冲 + IDLE 检测
                                                   └─ SPI1 PA4/PA5/PA6/PA7: W25Q32
                                                       固件暂存 + 备份
```

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
| T7 | TLS + HMAC + 防回滚（Python→MQTT(TLS)→STM32） | 29KB 真实固件，247 帧全部 ACK，HMAC 校验通过，版本 1.0.0→1.0.1 | ✅ PASS |

**T5/T6/T7 关键输出节选：**

```text
[BOOT] Flag -> BOOT_COUNTING(1)            ← T5: 新固件首次启动
[OTA] New firmware confirmed OK.           ← T5: App 写入 CONFIRMED
[OTA] Task started v4.0.                  ← T5: 升级成功

[BOOT] BOOT_COUNTING: count=03            ← T6: 第3次失败
[BOOT] Too many failed boots! Rolling back ← T6: 触发回滚
[BOOT] Rollback complete. Flag -> CONFIRMED
[OTA] Task started v4.0.                  ← T6: 旧固件恢复运行

[OTA] START: fw_size=29604, CRC32=0xC1C65F82, ver=1.0.1  ← T7: Phase 7 协议
[OTA] ACK seq=0 (120/29604 bytes)         ← T7: QoS 1 全帧成功
[OTA] HMAC-SHA256 OK                      ← T7: 签名校验
[OTA] Meta written (ver=1.0.1)            ← T7: 版本记录
[BOOT] Meta: ver=1.0.1, ts=2026-02-20     ← T7: Bootloader 读取元数据
```

---

## Flash 分区规划

### 内部 Flash (64KB)

| 分区 | 起始地址 | 大小 | 作用 |
| --- | --- | --- | --- |
| Bootloader | `0x08000000` | 8KB | 上电首先运行，含 SPI 驱动 |
| App | `0x08002000` | 55KB | 主业务代码（FreeRTOS + 各任务） |
| OTA Flag | `0x0800FC00` | 1KB | 升级标志位（state + boot_count） |

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

START payload (48 bytes, Phase 7 v3 协议):
  [fw_size:4][fw_crc32:4][hmac_sha256:32][ver_major:1][ver_minor:1][ver_patch:1][rsv:1][timestamp:4]

DATA  payload (1~248 bytes): 固件原始数据
END   payload: 空
```

---

## 项目目录结构

```text
IoT2/
├── CMakeLists.txt                          App 构建系统（arm-none-eabi-gcc）
├── App/
│   ├── OTA/
│   │   ├── Inc/ota_task.h                  OTA 协议定义 + 帧格式
│   │   └── Src/ota_task.c                  OTA 状态机 + CRC 校验 + 回滚备份
│   └── Test/
│       └── Src/comm_test.c                 硬件通信测试套件
├── Bootloader/
│   ├── CMakeLists.txt                      独立构建（无 FreeRTOS）
│   ├── STM32F103C8Tx_BOOT.ld              链接脚本 (0x08000000, 8KB)
│   ├── main.c                              升级/回滚/跳转逻辑
│   └── system_boot.c                       HSI 8MHz 时钟（无 PLL）
├── Core/
│   ├── Inc/
│   │   ├── flash_if.h                      内部 Flash 分区 + OTA Flag 定义
│   │   ├── FreeRTOSConfig.h                6KB 堆, 1000Hz Tick, prio≥5 for ISR
│   │   ├── usart1.h                        DMA 双缓冲 (256B)
│   │   └── usart2.h                        DMA 双缓冲 (257B, 防 TC/IDLE 竞争)
│   ├── Src/
│   │   ├── flash_if.c                      内部 Flash 擦/写/读 + SetFlagEx
│   │   ├── main.c                          App 入口: 外设初始化→信号量→任务
│   │   ├── stm32f1xx_it.c                  USART1/2 IDLE ISR (双缓冲切换)
│   │   ├── sys_config.c                    PLL 64MHz + GPIO
│   │   └── syscalls.c                      printf → USART1 重定向
│   └── Startup/
│       ├── startup_stm32f103xb.s           启动文件 + 向量表
│       └── STM32F103C8Tx_FLASH.ld          App 链接脚本 (0x08002000, 55KB)
├── ESP_Firmware/
│   ├── CMakeLists.txt                      ESP-IDF 构建
│   └── main/
│       ├── hello_world_main.c              ESP32 入口: WiFi→MQTT→UART→任务
│       ├── wifi_station.c/h                WiFi STA 连接（事件组等待）
│       ├── mqtt_client_task.c/h            MQTT 订阅 + OTA 帧队列 + 心跳
│       ├── mqtt_config.h                   Broker URI + 主题定义
│       └── wifi_config.h                   SSID/密码
├── Hardware/
│   ├── ESP32/
│   │   └── esp32_comm.c/h                  STM32 侧 ESP32 通信任务
│   └── W25Qxx/
│       └── w25qxx.c/h                      SPI Flash 驱动（JEDEC 自动检测）
└── tests/
    ├── ota_mqtt_sender.py                  MQTT OTA 发送器（协议/错误/完整测试）
    ├── ota_sender.py                       串口直连 OTA 发送器
    └── diag_serial.py                      串口诊断工具
```

---

## 硬件配置

| 接口 | 引脚 | 用途 |
| --- | --- | --- |
| USART1 | PA9(TX) / PA10(RX) | 调试 + OTA 接收（115200 baud） |
| USART2 | PA2(TX) / PA3(RX) | ESP32-S3 通信 |
| SPI1 | PA5(SCK) / PA6(MISO) / PA7(MOSI) / PA4(CS) | W25Q32（Mode0，APB2/4=16MHz） |
| ESP32 UART1 | GPIO17(TX) / GPIO18(RX) | STM32 通信（115200 baud） |

**时钟配置：**

- App：HSI/2 × PLL16 = 64MHz（`system_stm32f1xx.c`，`VECT_TAB_OFFSET=0x2000`）
- Bootloader：HSI 8MHz（无 PLL，`system_boot.c`，VTOR 在 `0x08000000`）

---

## STM32 任务分配

| 任务 | 优先级 | 栈 | 功能 |
| --- | --- | --- | --- |
| `vOtaProcessTask` | 3（高） | 512 words | USART1 DMA 接收 + OTA 协议处理 |
| `vEsp32CommTask` | 2（中） | 512 words | USART2 DMA 接收 + ESP32 帧转发 |

## ESP32 任务分配

| 任务 | 优先级 | 栈 | 功能 |
| --- | --- | --- | --- |
| `uart_ota_tx_task` | 6（高） | 3072 | OTA 帧出队 → UART 发送（mqtt_client_task.c） |
| `uart_tx_task` | 5 | 3072 | 心跳发送（OTA session 期间自动抑制） |
| `uart_rx_task` | 5 | 3072 | STM32 回复接收 → MQTT status 转发 |
| `mqtt_heartbeat_task` | 3 | 3072 | MQTT JSON 心跳（10s 间隔） |

---

## MQTT 主题

| 主题 | 方向 | 内容 | QoS |
| --- | --- | --- | --- |
| `device/stm32_iot2/ota/cmd` | PC → STM32 | OTA 二进制帧（HMAC 签名） | QoS 1（Phase 7 修复） |
| `device/stm32_iot2/ota/status` | STM32 → PC | ACK/NACK/状态文本 | QoS 1 |
| `device/stm32_iot2/heartbeat` | STM32 → PC | ESP32 心跳（OTA 期间自动抑制） | QoS 0 |

---

## 测试脚本（tests/ota_mqtt_sender.py）

依赖：`pip install paho-mqtt`，需要 MQTT Broker（如 Mosquitto）在局域网运行。

```bash
# Phase 7: TLS + 认证（推荐，生产环境）
python tests/ota_mqtt_sender.py --broker 192.168.0.3 \
  --tls-ca certs/ca.crt --mqtt-user iot2_sender --mqtt-pass sender_secure_2026 \
  --firmware build/STM32_AIoT_Project.bin --fw-version 1.0.1

# 明文测试（仅限 localhost 调试）
python tests/ota_mqtt_sender.py --broker 127.0.0.1 --mqtt-port 1883 \
  --firmware build/STM32_AIoT_Project.bin

# 协议状态机测试（假固件 1KB）
python tests/ota_mqtt_sender.py --broker 192.168.0.3 --tls-ca certs/ca.crt \
  --mqtt-user iot2_sender --mqtt-pass sender_secure_2026 --test-protocol

# 错误注入测试（CRC16 / 序号跳变 / ABORT 恢复）
python tests/ota_mqtt_sender.py --broker 192.168.0.3 --tls-ca certs/ca.crt \
  --mqtt-user iot2_sender --mqtt-pass sender_secure_2026 --test-error-injection
```

---

## 构建与烧录

```bash
# App 构建
cd IoT2 && mkdir build && cd build
cmake -G "Ninja" -DCMAKE_TOOLCHAIN_FILE=../arm-gcc-toolchain.cmake ..
ninja

# Bootloader 构建
cd IoT2/Bootloader && mkdir build && cd build
cmake -G "Ninja" -DCMAKE_TOOLCHAIN_FILE=../../arm-gcc-toolchain.cmake ..
ninja

# 烧录（STM32CubeProgrammer CLI）
STM32_Programmer_CLI -c port=SWD -d Bootloader.bin 0x08000000
STM32_Programmer_CLI -c port=SWD -d App.bin 0x08002000

# ESP32 构建与烧录
cd IoT2/ESP_Firmware
idf.py build
idf.py -p COMx flash monitor
```

---

## Phase 7 安全特性详解

### 传输层安全（TLS）

```bash
# Mosquitto TLS 配置（deploy/mosquitto/mosquitto_tls.conf）
listener 8883
cafile   E:\IoT2\certs\ca.crt
certfile E:\IoT2\certs\server.crt
keyfile  E:\IoT2\certs\server.key
tls_version tlsv1.2
allow_anonymous false
password_file C:\Program Files\mosquitto\passwd
```

- ESP32 使用 mbedTLS 验证服务器证书
- Python 使用 OpenSSL 验证服务器证书
- **关键经验**: TLS 证书必须包含 IP SAN（Subject Alternative Name），仅 CN 字段不足

### 固件完整性（HMAC-SHA256）

```c
// 发送端（Python）
hmac_digest = hmac.new(OTA_HMAC_KEY, fw_data, hashlib.sha256).digest()  // 32 字节

// 接收端（STM32）
hmac_sha256_init(&s_hmac_ctx, s_ota_hmac_key, 32);
// DATA 帧逐帧更新
hmac_sha256_update(&s_hmac_ctx, payload, len);
// END 时验证
hmac_sha256_final(&s_hmac_ctx, computed_hmac);
if (memcmp(computed_hmac, expected_hmac, 32) != 0) → 拒绝
```

### 防回滚机制

```c
// OTA START 时检查版本号
uint32_t new_ver = (major << 16) | (minor << 8) | patch;
uint32_t cur_ver = OTA_VERSION_TO_U32(1, 0, 0);
if (new_ver < cur_ver) {
    ota_printf("[OTA] ERR: anti-rollback! new=%u.%u.%u < current\r\n");
    goto cleanup;  // 拒绝旧版本固件
}
```

### MQTT 认证

```bash
# Mosquitto 密码文件创建
mosquitto_passwd -c "C:\Program Files\mosquitto\passwd" iot2_esp32
mosquitto_passwd -b "C:\Program Files\mosquitto\passwd" iot2_sender sender_secure_2026
```

## 下一步（可选）

- [x] Phase 7：安全加固（TLS + MQTT 认证 + HMAC-SHA256 + 防回滚）✅
- [ ] Phase 8：IWDG 硬件看门狗联动（检测运行中崩溃，触发 boot_count 累加）
- [ ] Phase 9：远程诊断（FreeRTOS 堆/任务状态定时上报 MQTT）
- [ ] Phase 10：多设备管理（MQTT 通配符订阅 + 设备 ID 路由）
