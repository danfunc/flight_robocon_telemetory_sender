#!/usr/bin/env python3
"""Flash FS へ algo1 / algo2 を確実にアップロードし、両方のロード・スワップを検証"""
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

def upload_file(local_path, dest_path):
    with open(local_path, "rb") as f:
        data = f.read()
    size = len(data)
    print(f"📦 アップロード: {local_path} ({size}B) -> {dest_path}")
    ser.reset_input_buffer()
    ser.write(f"  upload {dest_path} {size}\r\n".encode("utf-8"))
    ser.flush()
    time.sleep(0.5)
    ser.write(data)
    ser.flush()
    time.sleep(1.0)
    out = ser.read(ser.in_waiting or 1024).decode("utf-8", errors="replace")
    print(f"  応答: {out.strip()[:100]}")
    time.sleep(0.5)

# 1. 2つのファイルをアップロード
upload_file("bazel-bin/user_apps/algo1_fast.bin", "/bin/algo1.bin")
upload_file("bazel-bin/user_apps/algo2_slow.bin", "/bin/algo2.bin")

# 2. Flash FS ファイル一覧確認
print("\n=== Flash FS ファイル一覧 (ls) ===")
print(send_cmd("ls"))

# 3. algo1 をロードして実行
print("\n=== load /bin/algo1.bin 0 ===")
print(send_cmd("load /bin/algo1.bin 0", 3.0))

print("\n=== algo1 実行中 ps ===")
print(send_cmd("ps"))

# 4. algo1 をアンロード
print("\n=== unload algo1 ===")
print(send_cmd("unload algo1", 2.0))

# 5. algo2 をロードして実行
print("\n=== load /bin/algo2.bin 0 ===")
print(send_cmd("load /bin/algo2.bin 0", 3.0))

print("\n=== algo2 実行中 ps ===")
print(send_cmd("ps"))

ser.close()
