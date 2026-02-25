#!/usr/bin/env python3
"""
ota_sender.py — IoT2 OTA 协议测试脚本
======================================
用途：通过 PC 串口 (USART1) 向 STM32 发送 OTA 协议帧，验证 Phase 3/4 实现。

协议帧格式：
  [SOF:1][CMD:1][SEQ_H:1][SEQ_L:1][LEN_H:1][LEN_L:1][PAYLOAD:LEN][CRC16_H:1][CRC16_L:1]
  SOF = 0xAA，CRC16-CCITT 覆盖 [CMD..PAYLOAD]

用法示例：
  # 1. 用假数据测试 OTA 状态机（不会触发真正的 Bootloader 复位）
  python ota_sender.py --port COM3 --test-protocol

  # 2. 发送真实 .bin 文件（会写入 W25Q32 并触发 SystemReset）
  python ota_sender.py --port COM3 --firmware App.bin

  # 3. 错误注入测试（验证错误处理）
  python ota_sender.py --port COM3 --test-error-injection

依赖：pip install pyserial
"""

import argparse
import struct
import time
import zlib
import sys
import os
import hmac
import hashlib

# Windows GBK 控制台兼容：强制 stdout/stderr 使用 UTF-8
if sys.platform == "win32":
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("[ERROR] 缺少 pyserial，请运行: pip install pyserial")
    sys.exit(1)

# ─────────────────────────── 协议常量 ───────────────────────────

OTA_SOF          = 0xAA
CMD_OTA_START    = 0x01
CMD_OTA_DATA     = 0x02
CMD_OTA_END      = 0x03
CMD_OTA_ABORT    = 0x04
OTA_MAX_PAYLOAD  = 248   # 256 - 8 (frame overhead)

# HMAC-SHA256 共享密钥（32 bytes，与 STM32 ota_task.c 一致）
OTA_DEFAULT_HMAC_KEY = bytes([
    0x49, 0x6F, 0x54, 0x32, 0x2D, 0x4F, 0x54, 0x41,  # "IoT2-OTA"
    0x2D, 0x48, 0x4D, 0x41, 0x43, 0x2D, 0x4B, 0x65,  # "-HMAC-Ke"
    0x79, 0x2D, 0x32, 0x30, 0x32, 0x36, 0x00, 0x00,  # "y-2026\0\0"
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
])

# ─────────────────────────── CRC 计算 ───────────────────────────

def crc16_ccitt(data: bytes) -> int:
    """CRC16-CCITT，初值 0xFFFF，多项式 0x1021（与 STM32 侧一致）"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def crc32_ieee(data: bytes) -> int:
    """CRC32 IEEE 802.3（与 STM32 侧 crc32_update 一致）"""
    return zlib.crc32(data) & 0xFFFFFFFF

# ─────────────────────────── 帧构建 ───────────────────────────

def build_frame(cmd: int, seq: int, payload: bytes) -> bytes:
    """构建一帧 OTA 数据"""
    pay_len = len(payload)
    # 头部：CMD + SEQ(大端2字节) + LEN(大端2字节)
    header = bytes([cmd, (seq >> 8) & 0xFF, seq & 0xFF,
                    (pay_len >> 8) & 0xFF, pay_len & 0xFF])
    # CRC16 覆盖 [CMD..PAYLOAD]
    crc_data = header + payload
    crc = crc16_ccitt(crc_data)
    frame = bytes([OTA_SOF]) + crc_data + bytes([(crc >> 8) & 0xFF, crc & 0xFF])
    return frame


def build_start_frame(fw_size: int, fw_crc32: int,
                      fw_data: bytes = None, hmac_key: bytes = OTA_DEFAULT_HMAC_KEY,
                      version: tuple = (1, 0, 0),
                      build_timestamp: int = None) -> bytes:
    """构建 START 帧（v3 协议：52 字节 payload）"""
    if build_timestamp is None:
        build_timestamp = int(time.time())
    if fw_data is not None:
        hmac_digest = hmac.new(hmac_key, fw_data, hashlib.sha256).digest()
    else:
        hmac_digest = b'\x00' * 32  # 测试用：无 HMAC
    payload = struct.pack(">II", fw_size, fw_crc32) + hmac_digest
    payload += struct.pack("BBBB", version[0], version[1], version[2], 0)
    payload += struct.pack(">I", build_timestamp)
    return build_frame(CMD_OTA_START, 0, payload)


def build_data_frame(seq: int, chunk: bytes) -> bytes:
    return build_frame(CMD_OTA_DATA, seq, chunk)


def build_end_frame(seq: int) -> bytes:
    return build_frame(CMD_OTA_END, seq, b"")


def build_abort_frame() -> bytes:
    return build_frame(CMD_OTA_ABORT, 0, b"")

# ─────────────────────────── 串口工具 ───────────────────────────

def open_serial(port: str, baud: int = 115200) -> serial.Serial:
    ser = serial.Serial()
    ser.port     = port
    ser.baudrate = baud
    ser.bytesize = serial.EIGHTBITS
    ser.parity   = serial.PARITY_NONE
    ser.stopbits = serial.STOPBITS_ONE
    ser.timeout  = 3.0
    ser.dsrdtr   = False
    ser.rtscts   = False
    ser.xonxoff  = False
    ser.dtr      = False
    ser.rts      = False
    ser.open()

    # 主动复位 STM32：拉高 DTR 100ms（CH340 NRST），
    # 消除上次测试可能残留的 DMA/ORE 累积错误，保证干净启动。
    ser.dtr = True
    time.sleep(0.1)
    ser.dtr = False
    return ser


def read_response(ser: serial.Serial, timeout: float = 3.0) -> str:
    """读取串口响应直到超时，返回所有文本"""
    ser.timeout = timeout
    lines = []
    deadline = time.time() + timeout
    while time.time() < deadline:
        line = ser.readline()
        if line:
            text = line.decode("utf-8", errors="replace").rstrip()
            lines.append(text)
            print(f"  STM32> {text}")
            # ACK/NACK/ERR 出现即刷新截止时间（STM32 有响应，继续等）
            deadline = time.time() + 1.0
    return "\n".join(lines)


def send_frame(ser: serial.Serial, frame: bytes, label: str, verbose: bool = True):
    if verbose:
        hex_preview = frame[:16].hex(" ")
        suffix = "..." if len(frame) > 16 else ""
        print(f"  TX [{label}] {len(frame):3d}B: {hex_preview}{suffix}")
    ser.write(frame)
    ser.flush()

# ─────────────────────────── OTA 发送主逻辑 ───────────────────────────

def send_firmware(ser: serial.Serial, firmware: bytes,
                  chunk_size: int = OTA_MAX_PAYLOAD,
                  inter_frame_ms: int = 50,
                  verbose: bool = True,
                  fw_version: tuple = (1, 0, 0),
                  hmac_key: bytes = OTA_DEFAULT_HMAC_KEY) -> bool:
    """
    发送完整固件，返回 True 表示成功。
    注意：发完 END 帧后 STM32 会触发 SystemReset()，所以不等最终回复。
    """
    fw_size  = len(firmware)
    fw_crc32 = crc32_ieee(firmware)

    print(f"\n[OTA] 固件大小: {fw_size} 字节  CRC32: 0x{fw_crc32:08X}")
    print(f"[OTA] 固件版本: {fw_version[0]}.{fw_version[1]}.{fw_version[2]}")
    total_frames = (fw_size + chunk_size - 1) // chunk_size
    print(f"[OTA] DATA 帧数: {total_frames}  每帧载荷: {chunk_size} 字节\n")

    # ── START ──
    print("[OTA] >>> 发送 START 帧 (含 HMAC-SHA256 + 版本信息)")
    start_frame = build_start_frame(fw_size, fw_crc32, firmware, hmac_key,
                                    version=fw_version)
    send_frame(ser, start_frame, "START", verbose)
    resp = read_response(ser, timeout=5.0)
    if "READY" not in resp:
        print(f"[OTA] ERR: START 未收到 READY，STM32 响应: {resp!r}")
        return False

    # ── DATA ──
    print(f"\n[OTA] >>> 发送 {total_frames} 个 DATA 帧")
    offset = 0
    seq = 0
    while offset < fw_size:
        chunk = firmware[offset:offset + chunk_size]
        data_frame = build_data_frame(seq, chunk)
        send_frame(ser, data_frame, f"DATA seq={seq}", verbose)

        resp = read_response(ser, timeout=3.0)
        if f"ACK seq={seq}" not in resp:
            print(f"[OTA] ERR: DATA seq={seq} 未收到 ACK，响应: {resp!r}")
            return False

        offset += len(chunk)
        seq    += 1
        if inter_frame_ms > 0:
            time.sleep(inter_frame_ms / 1000.0)

        # 进度显示
        pct = offset * 100 // fw_size
        bar = "█" * (pct // 5) + "░" * (20 - pct // 5)
        print(f"  进度: [{bar}] {pct:3d}%  ({offset}/{fw_size} bytes)", end="\r")

    print()  # 换行

    # ── END ──
    print("\n[OTA] >>> 发送 END 帧")
    end_frame = build_end_frame(seq)
    send_frame(ser, end_frame, "END", verbose)
    # STM32 收到有效 END 后会输出 "DONE. Resetting in 1s..." 然后复位
    resp = read_response(ser, timeout=5.0)
    if "CRC32 OK" not in resp and "DONE" not in resp:
        print(f"[OTA] ERR: END 响应异常: {resp!r}")
        return False

    print("\n[OTA] [PASS] 固件发送完成，STM32 即将复位进入 Bootloader")
    return True

# ─────────────────────────── 测试模式 ───────────────────────────

def test_protocol(ser: serial.Serial) -> bool:
    """
    协议测试：使用小型假固件（1KB，填充 0x55 0xAA 交替），
    验证 OTA 状态机正常工作（不触发 SystemReset）。

    由于测试固件不是真实 App，只验证到 CRC32 OK 即停止；
    使用 --abort-after-end 选项在 END 前发 ABORT 改为中途测试。
    """
    print("\n" + "="*55)
    print("  TEST: OTA 协议状态机（假固件，1KB）")
    print("="*55)

    # 生成 1KB 测试固件
    fw_size = 1024
    firmware = bytes([0x55 if i % 2 == 0 else 0xAA for i in range(fw_size)])
    fw_crc32 = crc32_ieee(firmware)
    print(f"  测试固件: {fw_size} 字节，CRC32=0x{fw_crc32:08X}")

    result = send_firmware(ser, firmware, chunk_size=248, inter_frame_ms=20)
    if result:
        print("\n[TEST] [PASS] PASS: OTA 协议状态机正常")
    else:
        print("\n[TEST] [FAIL] FAIL: OTA 协议状态机异常")
    return result


def test_error_injection(ser: serial.Serial) -> bool:
    """
    错误注入测试：验证 STM32 对以下异常的处理：
      1. 错误的 CRC16
      2. 错误的序号（跳号）
      3. 非 START 状态下发 DATA
      4. ABORT 能重置状态机
    """
    print("\n" + "="*55)
    print("  TEST: 错误注入（验证异常处理）")
    print("="*55)

    passed = 0
    failed = 0

    # ── Case 1: 错误 CRC16 ──
    print("\n[Case 1] 发送 CRC16 损坏的 DATA 帧（应被忽略）")
    # 先发 START
    firmware = bytes(256)
    fw_crc32 = crc32_ieee(firmware)
    sf = build_start_frame(256, fw_crc32)
    send_frame(ser, sf, "START")
    resp = read_response(ser, timeout=3.0)
    if "READY" not in resp:
        print("  SKIP: START 失败，跳过 Case 1")
    else:
        # 构造坏帧：正常 DATA 但最后两字节 CRC 故意写错
        good = build_data_frame(0, bytes(64))
        bad  = good[:-2] + bytes([0x00, 0x00])
        send_frame(ser, bad, "DATA(bad CRC)")
        resp = read_response(ser, timeout=2.0)
        if "CRC16 err" in resp or "NACK" in resp:
            print("  [PASS] PASS: STM32 正确检测到 CRC16 错误")
            passed += 1
        else:
            print(f"  [FAIL] FAIL: 未检测到 CRC16 错误，响应: {resp!r}")
            failed += 1

    # ABORT 重置状态
    ser.write(build_abort_frame())
    ser.flush()
    read_response(ser, timeout=1.5)
    time.sleep(0.3)             # 等待 DMA 重启稳定
    ser.reset_input_buffer()

    # ── Case 2: 错误序号 ──
    print("\n[Case 2] 发送错误序号的 DATA 帧（应收到 NACK）")
    sf = build_start_frame(256, fw_crc32)
    send_frame(ser, sf, "START")
    resp = read_response(ser, timeout=5.0)  # erase 最多 300ms，留足时间
    if "READY" in resp:
        # 序号从 5 开始（应该从 0 开始）
        bad_seq_frame = build_data_frame(5, bytes(64))
        send_frame(ser, bad_seq_frame, "DATA(seq=5, expect 0)")
        resp = read_response(ser, timeout=3.0)
        if "NACK" in resp or "expected" in resp:
            print("  [PASS] PASS: STM32 正确检测到序号错误")
            passed += 1
        else:
            print(f"  [FAIL] FAIL: 未检测到序号错误，响应: {resp!r}")
            failed += 1
    else:
        print(f"  SKIP: Case 2 START 失败")

    # ABORT 重置状态
    ser.write(build_abort_frame())
    ser.flush()
    read_response(ser, timeout=1.5)
    time.sleep(0.3)
    ser.reset_input_buffer()

    # ── Case 3: ABORT 重置 ──
    print("\n[Case 3] ABORT 后状态机应回到 IDLE")
    # 先正常开始
    sf = build_start_frame(256, fw_crc32)
    send_frame(ser, sf, "START")
    resp = read_response(ser, timeout=5.0)
    if "READY" in resp:
        df = build_data_frame(0, bytes(64))
        send_frame(ser, df, "DATA seq=0")
        read_response(ser, timeout=2.0)
        # 发 ABORT
        ser.write(build_abort_frame())
        ser.flush()
        resp = read_response(ser, timeout=2.0)
        if "ABORT" in resp:
            print("  ABORT 收到，验证状态已重置 (发一个 DATA，应报 'DATA before START')")
            df2 = build_data_frame(1, bytes(32))
            send_frame(ser, df2, "DATA(after ABORT)")
            resp2 = read_response(ser, timeout=2.0)
            if "DATA before START" in resp2 or "ERR" in resp2:
                print("  [PASS] PASS: ABORT 后状态机回到 IDLE")
                passed += 1
            else:
                print(f"  [FAIL] FAIL: ABORT 后状态机未重置，响应: {resp2!r}")
                failed += 1
        else:
            print(f"  [FAIL] FAIL: ABORT 未响应: {resp!r}")
            failed += 1

    print(f"\n[错误注入] 结果: {passed} PASS, {failed} FAIL")
    return failed == 0


def list_ports():
    print("可用串口:")
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("  (未找到串口)")
    for p in ports:
        print(f"  {p.device:10s}  {p.description}")

# ─────────────────────────── main ───────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="IoT2 OTA 协议测试脚本",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python ota_sender.py --list-ports
  python ota_sender.py --port COM3 --test-protocol
  python ota_sender.py --port COM3 --test-error-injection
  python ota_sender.py --port COM3 --firmware build/App.bin
  python ota_sender.py --port COM3 --firmware build/App.bin --chunk-size 128
        """
    )
    parser.add_argument("--port",   help="串口号 (如 COM3, /dev/ttyUSB0)")
    parser.add_argument("--baud",   type=int, default=115200, help="波特率 (默认 115200)")
    parser.add_argument("--list-ports", action="store_true", help="列出可用串口后退出")
    parser.add_argument("--firmware",   help="固件 .bin 文件路径 (发送真实固件)")
    parser.add_argument("--test-protocol",       action="store_true", help="OTA 协议状态机测试（假固件）")
    parser.add_argument("--test-error-injection", action="store_true", help="错误注入测试")
    parser.add_argument("--fw-version", default="1.0.0",
                        help="固件版本号 (major.minor.patch，默认 1.0.0)")
    parser.add_argument("--hmac-key", default=None,
                        help="HMAC-SHA256 密钥 (64 位 hex 字符串，默认使用内置测试密钥)")
    parser.add_argument("--chunk-size", type=int, default=248, help="DATA 帧载荷大小 (默认 248)")
    parser.add_argument("--inter-frame-ms", type=int, default=50, help="帧间延迟 ms (默认 50)")
    parser.add_argument("--quiet", action="store_true", help="减少输出")
    args = parser.parse_args()

    if args.list_ports:
        list_ports()
        return

    if not args.port:
        parser.print_help()
        print("\n[ERROR] 请指定 --port")
        list_ports()
        sys.exit(1)

    # 解析固件版本号
    try:
        fw_ver_parts = tuple(int(x) for x in args.fw_version.split("."))
        if len(fw_ver_parts) != 3 or any(v < 0 or v > 255 for v in fw_ver_parts):
            raise ValueError
    except (ValueError, AttributeError):
        print(f"[ERROR] --fw-version 格式错误，需 major.minor.patch (如 1.2.3)")
        sys.exit(1)

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

    if args.chunk_size < 1 or args.chunk_size > OTA_MAX_PAYLOAD:
        print(f"[ERROR] --chunk-size 必须在 1~{OTA_MAX_PAYLOAD} 之间")
        sys.exit(1)

    print(f"[INFO] 连接 {args.port} @ {args.baud} baud...")
    try:
        ser = open_serial(args.port, args.baud)
    except serial.SerialException as e:
        print(f"[ERROR] 无法打开串口: {e}")
        sys.exit(1)

    print(f"[INFO] 串口已打开，等待 STM32 就绪 (4s)...")
    print("[INFO] 提示: 若 STM32 无响应，请在此期间按一次 RESET 键")
    time.sleep(4.0)
    # 清空缓冲区
    ser.reset_input_buffer()

    success = True
    try:
        if args.test_protocol:
            success = test_protocol(ser)

        elif args.test_error_injection:
            success = test_error_injection(ser)

        elif args.firmware:
            if not os.path.isfile(args.firmware):
                print(f"[ERROR] 文件不存在: {args.firmware}")
                sys.exit(1)
            with open(args.firmware, "rb") as f:
                firmware = f.read()
            print(f"[INFO] 加载固件: {args.firmware} ({len(firmware)} 字节)")
            success = send_firmware(
                ser, firmware,
                chunk_size=args.chunk_size,
                inter_frame_ms=args.inter_frame_ms,
                verbose=not args.quiet,
                fw_version=fw_ver_parts,
                hmac_key=hmac_key,
            )
        else:
            print("[ERROR] 请指定操作: --test-protocol / --test-error-injection / --firmware <file>")
            parser.print_help()
            sys.exit(1)

    finally:
        ser.close()

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
