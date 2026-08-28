#!/usr/bin/env python3
"""unload & load による動的アルゴリズム切り替えテスト"""
import time, serial

PORT = "/dev/cu.usbmodem101"
ser = serial.Serial(PORT, 115200, timeout=1.0, dsrdtr=True)
ser.dtr = True
ser.rts = True
time.sleep(1.0)

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

print("--- 1. unload algo1 ---")
print(send_cmd("unload algo1", 2.0))

print("\n--- 2. load /bin/algo2.bin 0 (algo2 起動) ---")
print(send_cmd("load /bin/algo2.bin 0", 2.5))

print("\n--- 3. ps ---")
print(send_cmd("ps"))

ser.close()
