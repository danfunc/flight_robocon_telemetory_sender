#!/usr/bin/env python3
"""Flash FS を完全フォーマットして状態を確認するスクリプト"""
import time, serial

PORT = "/dev/cu.usbmodem101"
ser = serial.Serial(PORT, 115200, timeout=1.0, dsrdtr=True)
ser.dtr = True
ser.rts = True
time.sleep(1.0)
ser.reset_input_buffer()

def send_cmd(cmd_str, wait_sec=3.0):
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

print("--- 1. フォーマット前の ls ---")
print(send_cmd("ls"))

print("\n--- 2. format 実行 ---")
print(send_cmd("format", 4.0))

print("\n--- 3. フォーマット後の ls ---")
print(send_cmd("ls"))

print("\n--- 4. メモリ・容量情報 (mem) ---")
print(send_cmd("mem"))

ser.close()
