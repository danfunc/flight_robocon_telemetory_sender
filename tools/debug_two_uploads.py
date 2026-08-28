#!/usr/bin/env python3
"""2回連続アップロードの生デバッグログ採取"""
import time, serial

ser = serial.Serial("/dev/cu.usbmodem101", 115200, timeout=0.1, dsrdtr=True)
ser.dtr = True
ser.rts = True

print("⏳ Pico 起動待機...")
time.sleep(3.0)
if ser.in_waiting > 0:
    ser.read(ser.in_waiting)

def upload(local_path, dest_path):
    with open(local_path, "rb") as f:
        data = f.read()
    size = len(data)
    print(f"\n==========================================")
    print(f"📦 アップロード: {local_path} ({size}B) -> {dest_path}")
    print(f"==========================================")
    
    # upload コマンド送信
    ser.write(f"\r\nupload {dest_path} {size}\r\n".encode("utf-8"))
    ser.flush()
    
    # 応答ログ監視 (READY 待機)
    t0 = time.time()
    while time.time() - t0 < 3.0:
        if ser.in_waiting > 0:
            chunk = ser.read(ser.in_waiting)
            print("RX:", repr(chunk))
            if b"READY" in chunk:
                print("-> READY 検出！バイナリ送信...")
                break
        time.sleep(0.02)
        
    time.sleep(0.05)
    ser.write(data)
    ser.flush()
    
    # 応答ログ監視 (UPLOAD_OK 待機)
    t0 = time.time()
    while time.time() - t0 < 4.0:
        if ser.in_waiting > 0:
            chunk = ser.read(ser.in_waiting)
            print("RX(データ後):", repr(chunk))
            if b"UPLOAD_OK" in chunk:
                print(f"🎉 UPLOAD_OK 検出！")
                break
        time.sleep(0.02)

upload("bazel-bin/user_apps/algo1_fast.bin", "/bin/algo1.bin")
time.sleep(0.5)
upload("bazel-bin/user_apps/algo2_slow.bin", "/bin/algo2.bin")

ser.close()
