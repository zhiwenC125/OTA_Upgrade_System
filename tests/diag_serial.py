"""诊断脚本：检测 CH340 DTR 复位行为"""
import serial, time

PORT = "COM7"

ser = serial.Serial()
ser.port = PORT
ser.baudrate = 115200
ser.timeout = 0.1
ser.dsrdtr = False
ser.rtscts = False
ser.dtr = False
ser.rts = False
ser.open()
print("[DIAG] Port opened (DTR=False before open)")

# 主动触发 DTR 复位脉冲
print("[DIAG] Sending DTR reset pulse (100ms)...")
ser.dtr = True
time.sleep(0.1)
ser.dtr = False
print("[DIAG] DTR released. Reading 5s for startup lines...")

startup_lines = []
deadline = time.time() + 5.0
while time.time() < deadline:
    line = ser.readline()
    if line:
        text = line.decode("utf-8", "replace").rstrip()
        startup_lines.append(text)
        print(f"  RX> {text}")

if startup_lines:
    print(f"\n[DIAG] DTR->NRST confirmed! STM32 reset and printed {len(startup_lines)} startup lines")
    print("[DIAG] Wait more and send START to confirm DMA works after reset...")
    time.sleep(1.0)
    ser.reset_input_buffer()
else:
    print("[DIAG] *** No startup lines - DTR NOT connected to STM32 NRST!")
    print("[DIAG] Try RTS instead...")
    # Try RTS
    ser.rts = True
    time.sleep(0.1)
    ser.rts = False
    print("[DIAG] RTS pulse sent. Reading 3s...")
    deadline = time.time() + 3.0
    rts_lines = []
    while time.time() < deadline:
        line = ser.readline()
        if line:
            text = line.decode("utf-8", "replace").rstrip()
            rts_lines.append(text)
            print(f"  RX> {text}")
    if rts_lines:
        print("[DIAG] RTS->NRST confirmed!")
    else:
        print("[DIAG] Neither DTR nor RTS resets STM32. Need physical reset button.")
    ser.reset_input_buffer()

print("\n[DIAG] Sending START frame...")
frame = bytes([0xAA, 0x01, 0x00, 0x00, 0x00, 0x08,
               0x00, 0x00, 0x01, 0x00,
               0x0D, 0x96, 0x85, 0x58,
               0xCD, 0xBF])
ser.write(frame)
ser.flush()
print(f"[DIAG] Sent: {frame.hex(' ')}")

print("[DIAG] Reading 5s for response...")
got_any = False
deadline = time.time() + 5.0
while time.time() < deadline:
    line = ser.readline()
    if line:
        text = line.decode("utf-8", "replace").rstrip()
        print(f"  RX> {text}")
        got_any = True

if not got_any:
    print("[DIAG] No response - STM32 DMA not working")
else:
    print("[DIAG] Got response - DMA working!")

ser.close()
print("[DIAG] Done.")
