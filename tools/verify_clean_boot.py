#!/usr/bin/env python3
"""picotool 完全消去＆再書き込み後の初期起動状態確認"""
import time, serial

PORT = "/dev/cu.usbmodem101"

def connect_serial():
    for _ in range(10):
        try:
            ser = serial.Serial(PORT, 115200, timeout=1.0, dsrdtr=True)
            ser.dtr = True
            ser.rts = True
            time.sleep(1.0)
            ser.reset_input_buffer()
            return ser
        except Exception:
            time.sleep(0.5)
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

print("=== 1. クリーン起動後の ls (Flash FS) ===")
print(send_cmd("ls"))

print("\n=== 2. クリーン起動後の ps (スレッド一覧) ===")
print(send_cmd("ps"))

print("\n=== 3. メモリ状態 (mem) ===")
print(send_cmd("mem"))

ser.close()
