#!/usr/bin/env python3
"""
ota_mqtt_sender.py — IoT2 OTA via MQTT 发送脚本
================================================
通过 MQTT 向 ESP32-S3 → STM32 发送 OTA 协议帧，实现远程固件升级。

链路：本脚本 → MQTT Broker (TLS/明文) → ESP32-S3 → UART → STM32 → W25Q32

协议帧格式：
  [SOF:1][CMD:1][SEQ_H:1][SEQ_L:1][LEN_H:1][LEN_L:1][PAYLOAD:LEN][CRC16_H:1][CRC16_L:1]

START payload (v3, 52 bytes):
  [fw_size:4 BE][fw_crc32:4 BE][hmac_sha256:32][ver_major:1][ver_minor:1][ver_patch:1][rsv:1][timestamp:4 BE]

用法示例：
  # 1. TLS + 认证 + HMAC（推荐）
  python ota_mqtt_sender.py --broker 192.168.0.3 --tls-ca certs/ca.crt \
      --mqtt-user iot2_sender --mqtt-pass sender_secure_2026 --firmware build/App.bin

  # 2. 明文（调试用，仅限 localhost）
  python ota_mqtt_sender.py --broker 127.0.0.1 --mqtt-port 1883 --firmware build/App.bin

  # 3. 协议测试
  python ota_mqtt_sender.py --broker 192.168.0.3 --tls-ca certs/ca.crt \
      --mqtt-user iot2_sender --mqtt-pass sender_secure_2026 --test-protocol

依赖：pip install paho-mqtt
"""

import argparse
import struct
import time
import zlib
import sys
import os
import threading
import hmac
import hashlib

# Windows GBK 控制台兼容
if sys.platform == "win32":
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("[ERROR] 缺少 paho-mqtt，请运行: pip install paho-mqtt")
    sys.exit(1)

# ─────────────────────────── 协议常量 ───────────────────────────

OTA_SOF          = 0xAA
CMD_OTA_START    = 0x01
CMD_OTA_DATA     = 0x02
CMD_OTA_END      = 0x03
CMD_OTA_ABORT    = 0x04
OTA_MAX_PAYLOAD  = 248

# MQTT 主题（与 ESP32 mqtt_config.h 一致）
MQTT_TOPIC_OTA_CMD    = "device/stm32_iot2/ota/cmd"
MQTT_TOPIC_OTA_STATUS = "device/stm32_iot2/ota/status"

# HMAC-SHA256 共享密钥（32 bytes，与 STM32 ota_task.c 中的 OTA_HMAC_KEY 一致）
# 生产环境应从安全存储读取，此处硬编码用于开发测试
OTA_DEFAULT_HMAC_KEY = bytes([
    0x49, 0x6F, 0x54, 0x32, 0x2D, 0x4F, 0x54, 0x41,  # "IoT2-OTA"
    0x2D, 0x48, 0x4D, 0x41, 0x43, 0x2D, 0x4B, 0x65,  # "-HMAC-Ke"
    0x79, 0x2D, 0x32, 0x30, 0x32, 0x36, 0x00, 0x00,  # "y-2026\0\0"
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
])

# ─────────────────────────── CRC 计算 ───────────────────────────

def crc16_ccitt(data: bytes) -> int:
    """CRC16-CCITT，初值 0xFFFF，多项式 0x1021"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def crc32_ieee(data: bytes) -> int:
    """CRC32 IEEE 802.3"""
    return zlib.crc32(data) & 0xFFFFFFFF

# ─────────────────────────── 帧构建 ───────────────────────────

def build_frame(cmd: int, seq: int, payload: bytes) -> bytes:
    pay_len = len(payload)
    header = bytes([cmd, (seq >> 8) & 0xFF, seq & 0xFF,
                    (pay_len >> 8) & 0xFF, pay_len & 0xFF])
    crc_data = header + payload
    crc = crc16_ccitt(crc_data)
    return bytes([OTA_SOF]) + crc_data + bytes([(crc >> 8) & 0xFF, crc & 0xFF])


def build_start_frame(fw_size: int, fw_crc32: int,
                      fw_data: bytes, hmac_key: bytes,
                      version: tuple = (1, 0, 0),
                      build_timestamp: int = None) -> bytes:
    """构建 START 帧（v3 协议：52 字节 payload = 8B header + 32B HMAC + 4B ver + 4B timestamp）"""
    if build_timestamp is None:
        build_timestamp = int(time.time())
    hmac_digest = hmac.new(hmac_key, fw_data, hashlib.sha256).digest()
    payload = struct.pack(">II", fw_size, fw_crc32) + hmac_digest
    payload += struct.pack("BBBB", version[0], version[1], version[2], 0)  # ver + reserved
    payload += struct.pack(">I", build_timestamp)
    return build_frame(CMD_OTA_START, 0, payload)


def build_data_frame(seq: int, chunk: bytes) -> bytes:
    return build_frame(CMD_OTA_DATA, seq, chunk)


def build_end_frame(seq: int) -> bytes:
    return build_frame(CMD_OTA_END, seq, b"")


def build_abort_frame() -> bytes:
    return build_frame(CMD_OTA_ABORT, 0, b"")

# ─────────────────────────── MQTT OTA 发送器 ───────────────────────────

class MqttOtaSender:
    def __init__(self, broker_host: str, broker_port: int = 8883,
                 tls_ca: str = None, hmac_key: bytes = OTA_DEFAULT_HMAC_KEY,
                 username: str = None, password: str = None):
        self.broker_host = broker_host
        self.broker_port = broker_port
        self.hmac_key = hmac_key
        self.connected = False

        self.client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id="ota_sender_pc",
            protocol=mqtt.MQTTv311,
        )

        # TLS 配置
        if tls_ca:
            self.client.tls_set(ca_certs=tls_ca)
            print(f"[TLS] 已加载 CA 证书: {tls_ca}")

        # MQTT 认证
        if username and password:
            self.client.username_pw_set(username, password)
            print(f"[AUTH] 用户: {username}")

        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message

        # 用于同步等待 STM32 响应
        self.response_event = threading.Event()
        self.last_response = ""
        self._response_lines = []
        self._response_lock = threading.Lock()

    def _on_connect(self, client, userdata, flags, reason_code, properties=None):
        if reason_code == 0:
            self.connected = True
            client.subscribe(MQTT_TOPIC_OTA_STATUS, qos=1)
            tls_str = "TLS" if self.client._ssl else "TCP"
            print(f"[MQTT] 已连接到 {self.broker_host}:{self.broker_port} ({tls_str})，订阅 status 主题")
        else:
            print(f"[MQTT] 连接失败，rc={reason_code}")

    def _on_message(self, client, userdata, msg):
        text = msg.payload.decode("utf-8", errors="replace").rstrip()
        with self._response_lock:
            self._response_lines.append(text)
        print(f"  STM32> {text}")
        self.response_event.set()

    def connect(self) -> bool:
        try:
            self.client.connect(self.broker_host, self.broker_port, keepalive=60)
        except Exception as e:
            print(f"[MQTT] 连接异常: {e}")
            return False

        self.client.loop_start()
        for _ in range(50):
            if self.connected:
                return True
            time.sleep(0.1)
        print("[MQTT] 连接超时")
        return False

    def disconnect(self):
        self.client.loop_stop()
        self.client.disconnect()
        print("[MQTT] 已断开连接")

    def _clear_response(self):
        with self._response_lock:
            self._response_lines.clear()
            self.last_response = ""
        self.response_event.clear()

    def _wait_response(self, timeout: float = 5.0, done_keywords=None) -> str:
        """等待 STM32 响应，收集所有在超时期间到达的行。

        done_keywords: 当响应文本中出现这些关键字之一时立即返回，
                       避免因心跳噪音("OK")触发提前超时。
                       若为 None，则保留原有的 0.5s 滑动窗口行为。
        """
        deadline = time.time() + timeout
        last_count = 0
        while time.time() < deadline:
            remaining = deadline - time.time()
            if remaining <= 0:
                break
            self.response_event.wait(timeout=min(remaining, 0.5))
            self.response_event.clear()

            with self._response_lock:
                current_count = len(self._response_lines)
                current_text  = "\n".join(self._response_lines)

            if current_count > last_count:
                last_count = current_count
                if done_keywords and any(kw in current_text for kw in done_keywords):
                    # 找到终止关键字，短暂等待尾随行后立即返回
                    time.sleep(0.05)
                    break
                elif not done_keywords:
                    # 无 done_keywords 时保留原有 0.5s 滑动窗口
                    deadline = time.time() + 0.5

        with self._response_lock:
            result = "\n".join(self._response_lines)
            self.last_response = result
        return result

    def send_frame(self, frame: bytes, label: str, verbose: bool = True):
        if verbose:
            hex_preview = frame[:16].hex(" ")
            suffix = "..." if len(frame) > 16 else ""
            print(f"  TX [{label}] {len(frame):3d}B: {hex_preview}{suffix}")
        # QoS 1：确保 Broker 确认收到（诊断 DATA 帧丢失问题）
        info = self.client.publish(MQTT_TOPIC_OTA_CMD, payload=frame, qos=1)
        try:
            info.wait_for_publish(timeout=5)
        except ValueError:
            pass  # already published
        if not info.is_published():
            print(f"  [WARN] MQTT publish NOT confirmed for {label}!")

    def send_frame_and_wait(self, frame: bytes, label: str,
                            timeout: float = 5.0,
                            verbose: bool = True,
                            done_keywords=None) -> str:
        self._clear_response()
        self.send_frame(frame, label, verbose)
        return self._wait_response(timeout, done_keywords=done_keywords)

    def send_firmware(self, firmware: bytes,
                      chunk_size: int = OTA_MAX_PAYLOAD,
                      inter_frame_ms: int = 100,
                      verbose: bool = True,
                      fw_version: tuple = (1, 0, 0)) -> bool:
        fw_size  = len(firmware)
        fw_crc32 = crc32_ieee(firmware)
        fw_hmac  = hmac.new(self.hmac_key, firmware, hashlib.sha256).digest()

        print(f"\n[OTA] 固件大小: {fw_size} 字节  CRC32: 0x{fw_crc32:08X}")
        print(f"[OTA] HMAC-SHA256: {fw_hmac[:16].hex()}...")
        print(f"[OTA] 固件版本: {fw_version[0]}.{fw_version[1]}.{fw_version[2]}")
        total_frames = (fw_size + chunk_size - 1) // chunk_size
        print(f"[OTA] DATA 帧数: {total_frames}  每帧载荷: {chunk_size} 字节")
        print(f"[OTA] 传输方式: MQTT (帧间延迟 {inter_frame_ms}ms)\n")

        # ── START ──
        print("[OTA] >>> 发送 START 帧 (含 HMAC-SHA256 签名 + 版本信息)")
        resp = self.send_frame_and_wait(
            build_start_frame(fw_size, fw_crc32, firmware, self.hmac_key,
                              version=fw_version),
            "START", timeout=8.0,
            done_keywords=["READY", "[OTA] ERR"])
        if "READY" not in resp:
            print(f"[OTA] ERR: START 未收到 READY，响应: {resp!r}")
            return False

        # ── DATA ──
        # 等待 STM32 消化 START 处理后残留的 USART2 噪声帧
        time.sleep(1.5)
        print(f"\n[OTA] >>> 发送 {total_frames} 个 DATA 帧")
        offset = 0
        seq = 0
        while offset < fw_size:
            chunk = firmware[offset:offset + chunk_size]
            resp = self.send_frame_and_wait(
                build_data_frame(seq, chunk),
                f"DATA seq={seq}", timeout=10.0, verbose=verbose,
                done_keywords=[f"ACK seq={seq}", "NACK", "[OTA] ERR"])

            if f"ACK seq={seq}" not in resp:
                print(f"\n[OTA] ERR: DATA seq={seq} 未收到 ACK，响应: {resp!r}")
                return False

            offset += len(chunk)
            seq    += 1
            if inter_frame_ms > 0:
                time.sleep(inter_frame_ms / 1000.0)

            pct = offset * 100 // fw_size
            bar = "█" * (pct // 5) + "░" * (20 - pct // 5)
            print(f"  进度: [{bar}] {pct:3d}%  ({offset}/{fw_size} bytes)", end="\r")

        print()

        # ── END ──
        print("\n[OTA] >>> 发送 END 帧")
        resp = self.send_frame_and_wait(build_end_frame(seq), "END", timeout=8.0,
            done_keywords=["DONE", "CRC32 OK", "HMAC OK", "[OTA] ERR"])
        if "CRC32 OK" not in resp and "DONE" not in resp:
            print(f"[OTA] ERR: END 响应异常: {resp!r}")
            return False

        print("\n[OTA] [PASS] 固件发送完成，STM32 即将复位进入 Bootloader")
        return True

# ─────────────────────────── 测试模式 ───────────────────────────

def test_protocol(sender: MqttOtaSender) -> bool:
    print("\n" + "="*55)
    print("  TEST: OTA 协议状态机 via MQTT（假固件，1KB）")
    print("="*55)

    fw_size = 1024
    firmware = bytes([0x55 if i % 2 == 0 else 0xAA for i in range(fw_size)])
    fw_crc32 = crc32_ieee(firmware)
    print(f"  测试固件: {fw_size} 字节，CRC32=0x{fw_crc32:08X}")

    result = sender.send_firmware(firmware, chunk_size=248, inter_frame_ms=100)
    if result:
        print("\n[TEST] [PASS] OTA via MQTT 协议状态机正常")
    else:
        print("\n[TEST] [FAIL] OTA via MQTT 协议状态机异常")
    return result


def test_error_injection(sender: MqttOtaSender) -> bool:
    print("\n" + "="*55)
    print("  TEST: 错误注入 via MQTT")
    print("="*55)

    passed = 0
    failed = 0

    firmware = bytes(256)
    fw_crc32 = crc32_ieee(firmware)

    # ── Case 1: 错误 CRC16 ──
    print("\n[Case 1] CRC16 损坏的 DATA 帧")
    resp = sender.send_frame_and_wait(
        build_start_frame(256, fw_crc32, firmware, sender.hmac_key),
        "START", timeout=8.0,
        done_keywords=["READY", "[OTA] ERR"])
    if "READY" in resp:
        good = build_data_frame(0, bytes(64))
        bad  = good[:-2] + bytes([0x00, 0x00])
        resp = sender.send_frame_and_wait(bad, "DATA(bad CRC)", timeout=3.0)
        if "CRC16 err" in resp or "NACK" in resp:
            print("  [PASS] STM32 正确检测到 CRC16 错误")
            passed += 1
        else:
            print(f"  [FAIL] 未检测到 CRC16 错误，响应: {resp!r}")
            failed += 1
    else:
        print("  SKIP: START 失败")

    # ABORT 重置
    sender.send_frame_and_wait(build_abort_frame(), "ABORT", timeout=3.0)
    time.sleep(0.5)

    # ── Case 2: 错误序号 ──
    print("\n[Case 2] 错误序号的 DATA 帧")
    resp = sender.send_frame_and_wait(
        build_start_frame(256, fw_crc32, firmware, sender.hmac_key),
        "START", timeout=8.0,
        done_keywords=["READY", "[OTA] ERR"])
    if "READY" in resp:
        resp = sender.send_frame_and_wait(
            build_data_frame(5, bytes(64)), "DATA(seq=5, expect 0)", timeout=3.0)
        if "NACK" in resp or "expected" in resp:
            print("  [PASS] STM32 正确检测到序号错误")
            passed += 1
        else:
            print(f"  [FAIL] 未检测到序号错误，响应: {resp!r}")
            failed += 1
    else:
        print("  SKIP: START 失败")

    sender.send_frame_and_wait(build_abort_frame(), "ABORT", timeout=3.0)
    time.sleep(0.5)

    # ── Case 3: ABORT 重置验证 ──
    print("\n[Case 3] ABORT 后状态机应回到 IDLE")
    resp = sender.send_frame_and_wait(
        build_start_frame(256, fw_crc32, firmware, sender.hmac_key),
        "START", timeout=8.0,
        done_keywords=["READY", "[OTA] ERR"])
    if "READY" in resp:
        sender.send_frame_and_wait(
            build_data_frame(0, bytes(64)), "DATA seq=0", timeout=3.0)
        resp = sender.send_frame_and_wait(
            build_abort_frame(), "ABORT", timeout=3.0)
        if "ABORT" in resp:
            time.sleep(0.3)
            resp = sender.send_frame_and_wait(
                build_data_frame(1, bytes(32)), "DATA(after ABORT)", timeout=3.0)
            if "DATA before START" in resp or "ERR" in resp:
                print("  [PASS] ABORT 后状态机回到 IDLE")
                passed += 1
            else:
                print(f"  [FAIL] 状态机未重置，响应: {resp!r}")
                failed += 1
        else:
            print(f"  [FAIL] ABORT 未响应: {resp!r}")
            failed += 1

    print(f"\n[错误注入] 结果: {passed} PASS, {failed} FAIL")
    return failed == 0

# ─────────────────────────── main ───────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="IoT2 OTA via MQTT 发送脚本 (TLS + HMAC-SHA256)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # TLS 加密连接（推荐）
  python ota_mqtt_sender.py --broker 192.168.0.3 --tls-ca certs/ca.crt --firmware build/App.bin

  # 明文连接（调试用）
  python ota_mqtt_sender.py --broker 192.168.0.3 --mqtt-port 1883 --firmware build/App.bin

  # 协议测试
  python ota_mqtt_sender.py --broker 192.168.0.3 --tls-ca certs/ca.crt --test-protocol

  # 自定义 HMAC 密钥（hex 格式）
  python ota_mqtt_sender.py --broker 192.168.0.3 --hmac-key 0102030405... --firmware build/App.bin
        """
    )
    parser.add_argument("--broker", required=True, help="MQTT Broker IP 地址")
    parser.add_argument("--mqtt-port", type=int, default=None,
                        help="MQTT 端口 (默认: 有 TLS 时 8883，无 TLS 时 1883)")
    parser.add_argument("--tls-ca", default=None, help="CA 证书路径 (启用 TLS)")
    parser.add_argument("--mqtt-user", default=None, help="MQTT 用户名")
    parser.add_argument("--mqtt-pass", default=None, help="MQTT 密码")
    parser.add_argument("--hmac-key", default=None,
                        help="HMAC-SHA256 密钥 (64 位 hex 字符串，默认使用内置测试密钥)")
    parser.add_argument("--firmware", help="固件 .bin 文件路径")
    parser.add_argument("--test-protocol", action="store_true", help="协议状态机测试")
    parser.add_argument("--test-error-injection", action="store_true", help="错误注入测试")
    parser.add_argument("--fw-version", default="1.0.0",
                        help="固件版本号 (major.minor.patch，默认 1.0.0)")
    parser.add_argument("--chunk-size", type=int, default=248, help="DATA 帧载荷大小")
    parser.add_argument("--inter-frame-ms", type=int, default=100, help="帧间延迟 ms (默认 100)")
    parser.add_argument("--quiet", action="store_true", help="减少输出")
    args = parser.parse_args()

    if args.chunk_size < 1 or args.chunk_size > OTA_MAX_PAYLOAD:
        print(f"[ERROR] --chunk-size 必须在 1~{OTA_MAX_PAYLOAD} 之间")
        sys.exit(1)

    # 确定端口
    if args.mqtt_port is None:
        port = 8883 if args.tls_ca else 1883
    else:
        port = args.mqtt_port

    # 解析 HMAC 密钥
    hmac_key = OTA_DEFAULT_HMAC_KEY
    if args.hmac_key:
        try:
            hmac_key = bytes.fromhex(args.hmac_key)
            if len(hmac_key) != 32:
                print(f"[ERROR] HMAC 密钥必须是 32 字节 (64 hex 字符)，当前 {len(hmac_key)} 字节")
                sys.exit(1)
        except ValueError:
            print("[ERROR] HMAC 密钥必须是有效的 hex 字符串")
            sys.exit(1)

    # 解析固件版本号
    try:
        fw_ver_parts = tuple(int(x) for x in args.fw_version.split("."))
        if len(fw_ver_parts) != 3 or any(v < 0 or v > 255 for v in fw_ver_parts):
            raise ValueError
    except (ValueError, AttributeError):
        print(f"[ERROR] --fw-version 格式错误，需 major.minor.patch (如 1.2.3)")
        sys.exit(1)

    tls_str = f"TLS (CA: {args.tls_ca})" if args.tls_ca else "明文"
    print(f"[INFO] 连接 MQTT Broker: {args.broker}:{port} ({tls_str})")
    sender = MqttOtaSender(args.broker, port, tls_ca=args.tls_ca, hmac_key=hmac_key,
                           username=args.mqtt_user, password=args.mqtt_pass)
    if not sender.connect():
        print("[ERROR] 无法连接到 MQTT Broker")
        sys.exit(1)

    # 等待一下让 MQTT 连接稳定
    time.sleep(1.0)

    success = True
    try:
        if args.test_protocol:
            success = test_protocol(sender)

        elif args.test_error_injection:
            success = test_error_injection(sender)

        elif args.firmware:
            if not os.path.isfile(args.firmware):
                print(f"[ERROR] 文件不存在: {args.firmware}")
                sys.exit(1)
            with open(args.firmware, "rb") as f:
                firmware = f.read()
            print(f"[INFO] 加载固件: {args.firmware} ({len(firmware)} 字节)")
            success = sender.send_firmware(
                firmware,
                chunk_size=args.chunk_size,
                inter_frame_ms=args.inter_frame_ms,
                verbose=not args.quiet,
                fw_version=fw_ver_parts,
            )
        else:
            print("[ERROR] 请指定操作: --test-protocol / --test-error-injection / --firmware <file>")
            parser.print_help()
            sys.exit(1)
    finally:
        sender.disconnect()

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
