#!/usr/bin/env python3
"""CDC シェル経由で blink -> algo1 -> algo2 -> algo1 の連続ホットスワップ検証"""
import time, serial

PORT = "/dev/cu.usbmodem101"

def connect_serial():
    for _ in range(5):
        try:
            ser = serial.Serial(PORT, 115200, timeout=1.0, dsrdtr=True)
            ser.dtr = True
            ser.rts = True
            time.sleep(1.0)
            ser.reset_input_buffer()
            return ser
        except Exception as e:
            time.sleep(1.0)
    raise RuntimeError("シリアルポート接続失敗")

ser = connect_serial()

def send_cmd(cmd_str, wait_sec=2.5):
    ser.reset_input_buffer()
    ser.write(f"  {cmd_str.strip()}\r\n".encode("utf-8"))
    ser.flush()
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < wait_sec:
        n = ser.in_waiting
        if n > 0:
            chunk = ser.read(n)
            buf.extend(chunk)
            if buf.endswith(b"shizuku> ") or b"\nshizuku> " in buf:
                break
        time.sleep(0.05)
    return buf.decode("utf-8", errors="replace")

print("=== 1. 初期 ps ===")
print(send_cmd("ps"))

print("\n=== 2. swap blink /bin/algo1.bin 0 (algo1 起動) ===")
print(send_cmd("swap blink /bin/algo1.bin 0", 3.0))

print("\n=== 3. algo1 実行中 ps ===")
print(send_cmd("ps"))

print("\n=== 4. swap algo1 /bin/algo2.bin 0 (algo2 へスワップ) ===")
print(send_cmd("swap algo1 /bin/algo2.bin 0", 3.0))

print("\n=== 5. algo2 実行中 ps ===")
print(send_cmd("ps"))

print("\n=== 6. swap algo2 /bin/algo1.bin 0 (algo1 へ戻すスワップ) ===")
print(send_cmd("swap algo2 /bin/algo1.bin 0", 3.0))

print("\n=== 7. 再び algo1 実行中 ps ===")
print(send_cmd("ps"))

ser.close()
