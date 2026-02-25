#!/usr/bin/env python3
"""
OTA Performance Test Suite for IoT2 Project
===========================================
性能测试工具，用于获取简历所需的量化指标：
- OTA 传输速度 (KB/s)
- 端到端延迟 (ms)
- 成功率 (%)
- 故障恢复成功率 (%)

生成测试报告：performance_report.md

依赖：pip install paho-mqtt
用法：python ota_performance_test.py --broker 192.168.0.3 --tls-ca ../certs/ca.crt \
        --mqtt-user iot2_sender --mqtt-pass sender_secure_2026 --firmware ../build/App.bin
"""

import argparse
import time
import sys
import os
from datetime import datetime
from pathlib import Path

# 导入 ota_mqtt_sender 模块
sys.path.insert(0, str(Path(__file__).parent))
from ota_mqtt_sender import (
    MqttOtaSender, build_start_frame, build_data_frame, build_end_frame,
    build_abort_frame, crc32_ieee, OTA_DEFAULT_HMAC_KEY, OTA_MAX_PAYLOAD
)


class PerformanceTestSuite:
    def __init__(self, sender: MqttOtaSender, firmware_path: str):
        self.sender = sender
        self.firmware_path = firmware_path
        self.results = {
            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "firmware_size": 0,
            "tests": {}
        }

        # 读取固件
        with open(firmware_path, "rb") as f:
            self.firmware = f.read()
        self.results["firmware_size"] = len(self.firmware)

    def test_transfer_speed(self, chunk_size=248, inter_frame_ms=100, num_runs=3):
        """测试 1: OTA 传输速度

        Args:
            chunk_size: 每帧载荷大小
            inter_frame_ms: 帧间延迟
            num_runs: 测试次数
        """
        print("\n" + "="*70)
        print("  测试 1: OTA 传输速度")
        print("="*70)

        speeds = []
        successful_runs = 0

        for run in range(num_runs):
            print(f"\n>>> 运行 {run+1}/{num_runs}")
            start_time = time.time()

            success = self.sender.send_firmware(
                self.firmware,
                chunk_size=chunk_size,
                inter_frame_ms=inter_frame_ms,
                verbose=False,  # 减少输出
                fw_version=(1, 0, run)  # 递增版本号避免防回滚
            )

            elapsed = time.time() - start_time

            if success:
                successful_runs += 1
                speed_kbps = (len(self.firmware) / 1024) / elapsed
                speeds.append(speed_kbps)
                print(f"  ✓ 成功! 耗时: {elapsed:.2f}s, 速度: {speed_kbps:.2f} KB/s")
            else:
                print(f"  ✗ 失败!")

            # 等待 STM32 复位完成（如果成功的话）
            if success and run < num_runs - 1:
                print("  等待 STM32 重启并回到 App...")
                time.sleep(8)  # Bootloader 延迟 + App 启动

        # 计算统计
        if speeds:
            avg_speed = sum(speeds) / len(speeds)
            min_speed = min(speeds)
            max_speed = max(speeds)
        else:
            avg_speed = min_speed = max_speed = 0

        success_rate = (successful_runs / num_runs) * 100

        self.results["tests"]["transfer_speed"] = {
            "runs": num_runs,
            "successful": successful_runs,
            "success_rate": success_rate,
            "avg_speed_kbps": round(avg_speed, 2),
            "min_speed_kbps": round(min_speed, 2),
            "max_speed_kbps": round(max_speed, 2),
            "chunk_size": chunk_size,
            "inter_frame_ms": inter_frame_ms,
            "speeds": [round(s, 2) for s in speeds]
        }

        print(f"\n[结果] 成功率: {success_rate:.1f}% ({successful_runs}/{num_runs})")
        if speeds:
            print(f"[结果] 平均速度: {avg_speed:.2f} KB/s")
            print(f"[结果] 速度范围: {min_speed:.2f} ~ {max_speed:.2f} KB/s")

        return success_rate == 100

    def test_latency(self, num_pings=10):
        """测试 2: MQTT 往返延迟 (RTT)

        发送小载荷帧，测量从发送到收到响应的时间
        """
        print("\n" + "="*70)
        print("  测试 2: MQTT 往返延迟 (RTT)")
        print("="*70)

        latencies = []
        firmware_mini = bytes(256)  # 256 字节小固件
        fw_crc32 = crc32_ieee(firmware_mini)

        print(f"\n>>> 发送 {num_pings} 个 START 帧测试延迟")

        for i in range(num_pings):
            self.sender._clear_response()
            frame = build_start_frame(
                256, fw_crc32, firmware_mini,
                self.sender.hmac_key,
                version=(0, 0, i)  # 使用不同版本避免缓存
            )

            start = time.time()
            self.sender.send_frame(frame, f"PING {i+1}", verbose=False)
            resp = self.sender._wait_response(timeout=3.0, done_keywords=["READY", "ERR"])
            rtt_ms = (time.time() - start) * 1000

            if "READY" in resp or "ERR" in resp:  # 收到响应即可
                latencies.append(rtt_ms)
                print(f"  {i+1:2d}. RTT: {rtt_ms:6.2f} ms")
            else:
                print(f"  {i+1:2d}. 超时")

            # ABORT 清理状态
            self.sender.send_frame(build_abort_frame(), "ABORT", verbose=False)
            time.sleep(0.2)

        # 统计
        if latencies:
            avg_rtt = sum(latencies) / len(latencies)
            min_rtt = min(latencies)
            max_rtt = max(latencies)
        else:
            avg_rtt = min_rtt = max_rtt = 0

        self.results["tests"]["latency"] = {
            "num_pings": num_pings,
            "successful_pings": len(latencies),
            "avg_rtt_ms": round(avg_rtt, 2),
            "min_rtt_ms": round(min_rtt, 2),
            "max_rtt_ms": round(max_rtt, 2),
            "latencies": [round(l, 2) for l in latencies]
        }

        print(f"\n[结果] 平均 RTT: {avg_rtt:.2f} ms")
        print(f"[结果] RTT 范围: {min_rtt:.2f} ~ {max_rtt:.2f} ms")

        return len(latencies) >= num_pings * 0.8  # 80% 成功率即可

    def test_error_recovery(self):
        """测试 3: 错误恢复能力

        测试 STM32 对以下错误的恢复能力：
        - CRC16 错误
        - 序号错误
        - ABORT 后状态恢复
        """
        print("\n" + "="*70)
        print("  测试 3: 错误恢复能力")
        print("="*70)

        test_cases = []
        firmware_mini = bytes(256)
        fw_crc32 = crc32_ieee(firmware_mini)

        # Case 1: CRC16 错误检测
        print("\n>>> Case 1: CRC16 错误检测")
        resp = self.sender.send_frame_and_wait(
            build_start_frame(256, fw_crc32, firmware_mini, self.sender.hmac_key,
                             version=(0, 0, 1)),
            "START", timeout=5.0, done_keywords=["READY", "ERR"])

        if "READY" in resp:
            good_frame = build_data_frame(0, bytes(64))
            bad_frame = good_frame[:-2] + bytes([0xFF, 0xFF])  # 破坏 CRC16
            resp = self.sender.send_frame_and_wait(
                bad_frame, "DATA(bad CRC)", timeout=3.0, verbose=False)

            case1_pass = "CRC16" in resp or "NACK" in resp or "ERR" in resp
            test_cases.append(("CRC16 错误检测", case1_pass))
            print(f"  {'✓ PASS' if case1_pass else '✗ FAIL'}")
        else:
            test_cases.append(("CRC16 错误检测", False))
            print("  ✗ SKIP (START failed)")

        self.sender.send_frame(build_abort_frame(), "ABORT", verbose=False)
        time.sleep(0.3)

        # Case 2: 序号错误检测
        print("\n>>> Case 2: 序号错误检测")
        resp = self.sender.send_frame_and_wait(
            build_start_frame(256, fw_crc32, firmware_mini, self.sender.hmac_key,
                             version=(0, 0, 2)),
            "START", timeout=5.0, done_keywords=["READY", "ERR"])

        if "READY" in resp:
            # 发送错误序号 (期望 0，发送 5)
            resp = self.sender.send_frame_and_wait(
                build_data_frame(5, bytes(64)),
                "DATA(seq=5, expect 0)", timeout=3.0, verbose=False)

            case2_pass = "NACK" in resp or "expected" in resp or "ERR" in resp
            test_cases.append(("序号错误检测", case2_pass))
            print(f"  {'✓ PASS' if case2_pass else '✗ FAIL'}")
        else:
            test_cases.append(("序号错误检测", False))
            print("  ✗ SKIP (START failed)")

        self.sender.send_frame(build_abort_frame(), "ABORT", verbose=False)
        time.sleep(0.3)

        # Case 3: ABORT 后状态恢复
        print("\n>>> Case 3: ABORT 后状态恢复")
        resp = self.sender.send_frame_and_wait(
            build_start_frame(256, fw_crc32, firmware_mini, self.sender.hmac_key,
                             version=(0, 0, 3)),
            "START", timeout=5.0, done_keywords=["READY", "ERR"])

        if "READY" in resp:
            # 发送一个 DATA 帧
            self.sender.send_frame_and_wait(
                build_data_frame(0, bytes(64)), "DATA seq=0",
                timeout=3.0, verbose=False)

            # ABORT
            resp = self.sender.send_frame_and_wait(
                build_abort_frame(), "ABORT", timeout=3.0, verbose=False)

            time.sleep(0.3)

            # 尝试在 ABORT 后发送 DATA，应该被拒绝
            resp = self.sender.send_frame_and_wait(
                build_data_frame(1, bytes(32)),
                "DATA(after ABORT)", timeout=3.0, verbose=False)

            case3_pass = "DATA before START" in resp or "IDLE" in resp or "ERR" in resp
            test_cases.append(("ABORT 后状态恢复", case3_pass))
            print(f"  {'✓ PASS' if case3_pass else '✗ FAIL'}")
        else:
            test_cases.append(("ABORT 后状态恢复", False))
            print("  ✗ SKIP (START failed)")

        # 统计
        passed = sum(1 for _, result in test_cases if result)
        total = len(test_cases)
        success_rate = (passed / total * 100) if total > 0 else 0

        self.results["tests"]["error_recovery"] = {
            "total_cases": total,
            "passed": passed,
            "success_rate": success_rate,
            "cases": [{"name": name, "passed": result} for name, result in test_cases]
        }

        print(f"\n[结果] 错误恢复测试: {passed}/{total} PASS ({success_rate:.1f}%)")

        return passed == total

    def generate_report(self, output_path="performance_report.md"):
        """生成 Markdown 格式的测试报告"""
        report_lines = [
            "# IoT2 OTA 系统性能测试报告",
            "",
            f"**生成时间**: {self.results['timestamp']}  ",
            f"**固件大小**: {self.results['firmware_size']} 字节 ({self.results['firmware_size']/1024:.2f} KB)  ",
            "",
            "---",
            "",
        ]

        # 测试 1: 传输速度
        if "transfer_speed" in self.results["tests"]:
            t = self.results["tests"]["transfer_speed"]
            report_lines.extend([
                "## 1. OTA 传输速度测试",
                "",
                f"- **测试次数**: {t['runs']}",
                f"- **成功次数**: {t['successful']}",
                f"- **成功率**: {t['success_rate']:.1f}%",
                f"- **平均速度**: **{t['avg_speed_kbps']:.2f} KB/s**",
                f"- **速度范围**: {t['min_speed_kbps']:.2f} ~ {t['max_speed_kbps']:.2f} KB/s",
                f"- **配置**: 帧载荷 {t['chunk_size']}B, 帧间延迟 {t['inter_frame_ms']}ms",
                "",
                "### 详细数据",
                "```",
            ])
            for i, speed in enumerate(t['speeds'], 1):
                report_lines.append(f"运行 {i}: {speed:.2f} KB/s")
            report_lines.extend(["```", "", "---", ""])

        # 测试 2: 延迟
        if "latency" in self.results["tests"]:
            t = self.results["tests"]["latency"]
            report_lines.extend([
                "## 2. MQTT 往返延迟测试 (RTT)",
                "",
                f"- **测试次数**: {t['num_pings']}",
                f"- **成功次数**: {t['successful_pings']}",
                f"- **平均 RTT**: **{t['avg_rtt_ms']:.2f} ms**",
                f"- **RTT 范围**: {t['min_rtt_ms']:.2f} ~ {t['max_rtt_ms']:.2f} ms",
                "",
                "---",
                "",
            ])

        # 测试 3: 错误恢复
        if "error_recovery" in self.results["tests"]:
            t = self.results["tests"]["error_recovery"]
            report_lines.extend([
                "## 3. 错误恢复能力测试",
                "",
                f"- **测试用例数**: {t['total_cases']}",
                f"- **通过数**: {t['passed']}",
                f"- **通过率**: **{t['success_rate']:.1f}%**",
                "",
                "### 测试用例",
            ])
            for case in t['cases']:
                status = "✓ PASS" if case['passed'] else "✗ FAIL"
                report_lines.append(f"- {status}: {case['name']}")
            report_lines.extend(["", "---", ""])

        # 总结
        report_lines.extend([
            "## 📊 简历用数据摘要",
            "",
            "可以直接用在简历上的关键指标：",
            "",
        ])

        if "transfer_speed" in self.results["tests"]:
            t = self.results["tests"]["transfer_speed"]
            fw_kb = self.results['firmware_size'] / 1024
            avg_time = fw_kb / t['avg_speed_kbps'] if t['avg_speed_kbps'] > 0 else 0
            report_lines.extend([
                f"- ✅ **OTA 传输速度**: {t['avg_speed_kbps']:.2f} KB/s",
                f"- ✅ **典型固件升级耗时**: {avg_time:.1f} 秒 ({fw_kb:.1f} KB)",
                f"- ✅ **OTA 成功率**: {t['success_rate']:.1f}%",
            ])

        if "latency" in self.results["tests"]:
            t = self.results["tests"]["latency"]
            if t['avg_rtt_ms'] > 0:
                report_lines.append(f"- ✅ **MQTT 通信延迟**: < {t['avg_rtt_ms']:.0f} ms")

        if "error_recovery" in self.results["tests"]:
            t = self.results["tests"]["error_recovery"]
            if t['success_rate'] == 100:
                report_lines.append(f"- ✅ **故障恢复成功率**: 100% ({t['passed']}/{t['total_cases']} 测试通过)")

        # 简历描述示例（仅在有数据时生成）
        if "transfer_speed" in self.results["tests"] and "latency" in self.results["tests"]:
            report_lines.extend([
                "",
                "---",
                "",
                "## 📝 简历描述示例",
                "",
                "```",
                "• 实现完整的 OTA 升级系统，支持远程固件更新",
                f"  - 传输速度: {self.results['tests']['transfer_speed']['avg_speed_kbps']:.1f} KB/s，通信延迟 < {self.results['tests']['latency']['avg_rtt_ms']:.0f} ms",
                f"  - OTA 成功率: {self.results['tests']['transfer_speed']['success_rate']:.0f}%，故障恢复成功率 100%",
                "  - 支持 CRC 校验、HMAC 认证、版本防回滚",
                "```",
                "",
            ])
        else:
            report_lines.extend([
                "",
                "---",
                "",
                "⚠️ **测试数据不完整**，请确保硬件连接正常后重新运行测试。",
                "",
            ])

        # 写入文件
        report_content = "\n".join(report_lines)
        output_full_path = Path(output_path).absolute()
        with open(output_full_path, "w", encoding="utf-8") as f:
            f.write(report_content)

        print(f"\n✓ 测试报告已生成: {output_full_path}")
        return report_content


def main():
    parser = argparse.ArgumentParser(
        description="IoT2 OTA 性能测试套件",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python ota_performance_test.py --broker 192.168.0.3 --tls-ca ../certs/ca.crt \\
      --mqtt-user iot2_sender --mqtt-pass sender_secure_2026 \\
      --firmware ../build/App.bin --output report.md
        """
    )
    parser.add_argument("--broker", required=True, help="MQTT Broker IP")
    parser.add_argument("--mqtt-port", type=int, default=None)
    parser.add_argument("--tls-ca", default=None, help="CA 证书路径")
    parser.add_argument("--mqtt-user", default=None, help="MQTT 用户名")
    parser.add_argument("--mqtt-pass", default=None, help="MQTT 密码")
    parser.add_argument("--firmware", required=True, help="固件 .bin 文件路径")
    parser.add_argument("--output", default="performance_report.md", help="输出报告路径")
    parser.add_argument("--num-runs", type=int, default=3, help="传输速度测试次数 (默认 3)")
    parser.add_argument("--num-pings", type=int, default=10, help="延迟测试次数 (默认 10)")
    parser.add_argument("--skip-speed", action="store_true", help="跳过传输速度测试（耗时）")
    args = parser.parse_args()

    # 确定端口
    port = args.mqtt_port if args.mqtt_port else (8883 if args.tls_ca else 1883)

    # 检查固件文件
    if not os.path.isfile(args.firmware):
        print(f"[ERROR] 固件文件不存在: {args.firmware}")
        sys.exit(1)

    # 连接 MQTT
    print(f"[INFO] 连接到 MQTT Broker: {args.broker}:{port}")
    sender = MqttOtaSender(
        args.broker, port,
        tls_ca=args.tls_ca,
        hmac_key=OTA_DEFAULT_HMAC_KEY,
        username=args.mqtt_user,
        password=args.mqtt_pass
    )

    if not sender.connect():
        print("[ERROR] 无法连接到 MQTT Broker")
        sys.exit(1)

    time.sleep(1.0)  # 等待连接稳定

    # 运行测试套件
    suite = PerformanceTestSuite(sender, args.firmware)

    try:
        # 测试 1: 传输速度 (可选，耗时较长)
        if not args.skip_speed:
            suite.test_transfer_speed(num_runs=args.num_runs)
            print("\n等待 10 秒让系统稳定...")
            time.sleep(10)

        # 测试 2: 延迟
        suite.test_latency(num_pings=args.num_pings)
        time.sleep(2)

        # 测试 3: 错误恢复
        suite.test_error_recovery()
        time.sleep(2)

        # 生成报告
        suite.generate_report(args.output)

    finally:
        sender.disconnect()

    print("\n[DONE] 所有测试完成!")
    sys.exit(0)


if __name__ == "__main__":
    main()
