#!/usr/bin/env python3
"""CDC シェル経由でフォーマット、アップロード、ホットスワップを一気通貫で確認するテスト"""
import time, serial

PORT = "/dev/cu.usbmodem101"
ser = serial.Serial(PORT, 115200, timeout=1.0, dsrdtr=True)
ser.dtr = True
ser.rts = True
time.sleep(1.0)

def send_cmd(cmd_str, wait_sec=2.0):
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

def upload_via_cdc(local_path, dest_path):
    with open(local_path, "rb") as f:
        data = f.read()
    size = len(data)
    print(f"📦 アップロード: {local_path} ({size}B) -> {dest_path}")
    ser.reset_input_buffer()
    ser.write(f"  upload {dest_path} {size}\r\n".encode("utf-8"))
    ser.flush()
    time.sleep(0.3)
    ser.write(data)
    ser.flush()
    time.sleep(0.8)
    out = ser.read(ser.in_waiting or 1024).decode("utf-8", errors="replace")
    print(f"  応答: {out.strip()[:100]}")

# 1. format
print("=== 1. format ===")
print(send_cmd("format", 3.0))

# 2. アップロード
print("\n=== 2. アップロード ===")
upload_via_cdc("bazel-bin/user_apps/algo1_fast.bin", "/bin/algo1.bin")
upload_via_cdc("bazel-bin/user_apps/algo2_slow.bin", "/bin/algo2.bin")

# 3. ls
print("\n=== 3. ls ===")
print(send_cmd("ls"))

# 4. swap blink /bin/algo1.bin 0
print("\n=== 4. swap blink /bin/algo1.bin 0 ===")
print(send_cmd("swap blink /bin/algo1.bin 0", 3.0))

print("\n=== 5. algo1 実行中 ps ===")
print(send_cmd("ps"))

# 5. swap algo1 /bin/algo2.bin 0
print("\n=== 6. swap algo1 /bin/algo2.bin 0 ===")
print(send_cmd("swap algo1 /bin/algo2.bin 0", 3.0))

print("\n=== 7. algo2 実行中 ps ===")
print(send_cmd("ps"))

ser.close()
