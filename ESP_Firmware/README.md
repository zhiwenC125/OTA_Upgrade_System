# ESP32-S3 OTA 网关固件

**芯片：** ESP32-S3
**框架：** ESP-IDF 5.5.2
**作用：** WiFi/MQTT ↔ UART 透明桥，将 MQTT 上的 OTA 指令转发给 STM32F103

---

## 功能概述

```text
MQTT Broker (WiFi)
      │  topic: device/stm32_iot2/ota/cmd
      ▼
ESP32-S3 Gateway
      │  UART1 GPIO17(TX)/GPIO18(RX)  115200 baud
      ▼
STM32F103C8T6 (USART2 PA2/PA3)
```

1. **OTA 转发**：订阅 `device/stm32_iot2/ota/cmd`，收到二进制帧后直接通过 UART 发给 STM32
2. **状态回传**：从 UART 读取 STM32 的 ACK/NACK/状态文本，发布到 `device/stm32_iot2/ota/status`
3. **心跳**：每 2 秒发送心跳给 STM32；**OTA session 期间自动抑制**（防止心跳与 OTA 帧混淆）

---

## 源文件结构

```
ESP_Firmware/main/
├── hello_world_main.c    入口 + UART 初始化 + 收发任务
├── wifi_station.c/h      WiFi Station 连接（阻塞式，重试 10 次）
├── mqtt_client_task.c/h  MQTT 客户端 + OTA 帧转发 + 心跳抑制
└── CMakeLists.txt
```

---

## MQTT 主题

| 主题 | QoS | 方向 | 内容 |
| --- | --- | --- | --- |
| `device/stm32_iot2/ota/cmd` | 0 | 订阅（下行） | OTA 二进制帧（最大 256B） |
| `device/stm32_iot2/ota/status` | 0 | 发布（上行） | STM32 响应文本（ACK/NACK/状态） |
| `device/stm32_iot2/heartbeat` | 0 | 发布（上行） | `ESP32 heartbeat #N` |

> **注意：OTA CMD 必须使用 QoS 0**。QoS 1 的重传机制会导致 Broker 重发旧的 DATA 帧，
> STM32 检测到序号不连续后进入 ERROR 状态，OTA 失败。

---

## UART 配置

| 参数 | 值 |
| --- | --- |
| 端口 | UART_NUM_1 |
| TX | GPIO17 |
| RX | GPIO18 |
| 波特率 | 115200 |
| TX ring buffer | 512B（防止 256B OTA 帧被 FIFO 截断） |
| RX buffer | 512B |

---

## WiFi / MQTT 配置

编辑 `wifi_station.c` 中的宏：

```c
#define WIFI_SSID      "your_ssid"
#define WIFI_PASSWORD  "your_password"
```

编辑 `mqtt_client_task.c` 中的宏：

```c
#define MQTT_BROKER_URI   "mqtt://192.168.0.3:1883"
```

---

## 编译与烧录

```bash
cd ESP_Firmware
idf.py set-target esp32s3
idf.py build
idf.py -p COM_PORT flash monitor
```

---

## 心跳抑制机制

OTA session 期间（START 帧发出 → END/ABORT 收到之前），`uart_tx_task` 跳过心跳发送：

```c
if (mqtt_ota_session_active()) {
    continue;  // 不发心跳
}
```

**原因：** STM32 USART2 使用 IDLE 中断检测帧边界。若心跳（ASCII 文本）与 OTA 二进制帧间隔不足，
DMA 会将两者合并为一帧，导致 SOF/CRC 校验失败，OTA 中断。

---

## 启动日志示例

```text
I (xxx) STM32_COMM: ESP32-S3 Gateway starting...
I (xxx) wifi: connected to AP, IP: 192.168.0.x
I (xxx) MQTT: Connected to broker
I (xxx) MQTT: Subscribed to device/stm32_iot2/ota/cmd
I (xxx) STM32_COMM: UART1 initialized: 115200 baud, TX=GPIO17, RX=GPIO18
I (xxx) STM32_COMM: TX: ESP32 heartbeat #0
I (xxx) STM32_COMM: RX #1 (4 bytes): OK
```
