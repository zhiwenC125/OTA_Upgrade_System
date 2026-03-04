#!/usr/bin/env python3
"""
diag_monitor.py — IoT2 远程诊断监控
====================================
订阅 MQTT 主题，实时显示 STM32 设备的诊断、传感器、告警数据。

用法示例：
  # TLS + 认证（推荐）
  python diag_monitor.py --broker 192.168.0.3 --tls-ca certs/ca.crt \
      --mqtt-user iot2_sender --mqtt-pass sender_secure_2026

  # 明文（调试用）
  python diag_monitor.py --broker 127.0.0.1 --mqtt-port 1883

依赖：pip install paho-mqtt
"""

import argparse
import json
import sys
import ssl
import time
from datetime import datetime

# Windows GBK 控制台兼容
if sys.platform == "win32":
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("[ERROR] 缺少 paho-mqtt，请运行: pip install paho-mqtt")
    sys.exit(1)

# ─────────────────────────── MQTT 主题 ───────────────────────────

DEVICE_ID = "stm32_iot2"
TOPIC_DIAGNOSTICS = f"device/{DEVICE_ID}/diagnostics"
TOPIC_SENSOR      = f"device/{DEVICE_ID}/sensor/data"
TOPIC_ALERT       = f"device/{DEVICE_ID}/sensor/alert"
TOPIC_HEARTBEAT   = f"device/{DEVICE_ID}/heartbeat"
TOPIC_OTA_STATUS  = f"device/{DEVICE_ID}/ota/status"

# ─────────────────────────── 格式化输出 ───────────────────────────

def fmt_time():
    return datetime.now().strftime("%H:%M:%S")

def on_diag(payload):
    """格式化诊断数据"""
    d = json.loads(payload)
    up = d.get("up", 0)
    h, m, s = up // 3600, (up % 3600) // 60, up % 60
    print(f"[{fmt_time()}] DIAG  | uptime={h:02d}:{m:02d}:{s:02d} "
          f"heap={d.get('heap',0)} tasks={d.get('tasks',0)} "
          f"ota={'ACTIVE' if d.get('ota',0) else 'idle'} "
          f"wdog={'ON' if d.get('wdog',0) else 'OFF'}")
    print(f"         HWM  | sensor={d.get('s_hwm','?')} "
          f"dataproc={d.get('d_hwm','?')} "
          f"ota={d.get('o_hwm','?')} "
          f"esp_comm={d.get('e_hwm','?')}")

def on_sensor(payload):
    """格式化传感器数据"""
    d = json.loads(payload)
    alert_flags = d.get("alert", 0)
    alert_str = ""
    if alert_flags & 1:
        alert_str += " TEMP_HIGH"
    if alert_flags & 2:
        alert_str += " HUMI_HIGH"
    print(f"[{fmt_time()}] SENSOR| temp={d.get('temp','?')}C "
          f"humi={d.get('humi','?')}% "
          f"avg_t={d.get('avg_t','?')} avg_h={d.get('avg_h','?')}"
          f"{alert_str}")

def on_alert(payload):
    """格式化告警"""
    d = json.loads(payload)
    print(f"[{fmt_time()}] ALERT | temp={d.get('temp','?')}C "
          f"humi={d.get('humi','?')}% flags=0x{d.get('flags',0):02X}")

def on_heartbeat(payload):
    """格式化心跳"""
    try:
        d = json.loads(payload)
        print(f"[{fmt_time()}] HB    | status={d.get('status','?')} "
              f"uptime={d.get('uptime','?')}s wifi={d.get('wifi','?')}")
    except json.JSONDecodeError:
        print(f"[{fmt_time()}] HB    | {payload}")

def on_ota_status(payload):
    """格式化 OTA 状态"""
    print(f"[{fmt_time()}] OTA   | {payload}")

# ─────────────────────────── MQTT 回调 ───────────────────────────

TOPIC_HANDLERS = {
    TOPIC_DIAGNOSTICS: on_diag,
    TOPIC_SENSOR:      on_sensor,
    TOPIC_ALERT:       on_alert,
    TOPIC_HEARTBEAT:   on_heartbeat,
    TOPIC_OTA_STATUS:  on_ota_status,
}

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"[{fmt_time()}] Connected to MQTT broker")
        for topic in TOPIC_HANDLERS:
            client.subscribe(topic, qos=0)
            print(f"  Subscribed: {topic}")
    else:
        print(f"[{fmt_time()}] Connection failed (rc={rc})")

def on_message(client, userdata, msg):
    payload = msg.payload.decode("utf-8", errors="replace").strip()
    handler = TOPIC_HANDLERS.get(msg.topic)
    if handler:
        try:
            handler(payload)
        except (json.JSONDecodeError, KeyError) as e:
            print(f"[{fmt_time()}] PARSE | topic={msg.topic} error={e} raw={payload}")
    else:
        print(f"[{fmt_time()}] ???   | topic={msg.topic} payload={payload}")

def on_disconnect(client, userdata, rc):
    if rc != 0:
        print(f"[{fmt_time()}] Disconnected (rc={rc}), reconnecting...")

# ─────────────────────────── Main ───────────────────────────

def main():
    parser = argparse.ArgumentParser(description="IoT2 Remote Diagnostics Monitor")
    parser.add_argument("--broker", default="192.168.0.3", help="MQTT broker IP")
    parser.add_argument("--mqtt-port", type=int, default=None,
                        help="MQTT port (default: 8883 with TLS, 1883 without)")
    parser.add_argument("--tls-ca", default=None, help="CA certificate for TLS")
    parser.add_argument("--mqtt-user", default=None, help="MQTT username")
    parser.add_argument("--mqtt-pass", default=None, help="MQTT password")
    args = parser.parse_args()

    client = mqtt.Client(client_id="iot2_diag_monitor", protocol=mqtt.MQTTv311)
    client.on_connect    = on_connect
    client.on_message    = on_message
    client.on_disconnect = on_disconnect

    if args.mqtt_user:
        client.username_pw_set(args.mqtt_user, args.mqtt_pass)

    port = args.mqtt_port
    if args.tls_ca:
        client.tls_set(ca_certs=args.tls_ca, tls_version=ssl.PROTOCOL_TLSv1_2)
        if port is None:
            port = 8883
    else:
        if port is None:
            port = 1883

    print(f"IoT2 Diagnostics Monitor")
    print(f"Connecting to {args.broker}:{port} ...")
    print(f"{'='*60}")

    try:
        client.connect(args.broker, port, keepalive=60)
        client.loop_forever()
    except KeyboardInterrupt:
        print(f"\n[{fmt_time()}] Stopped by user")
        client.disconnect()
    except Exception as e:
        print(f"[{fmt_time()}] Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
