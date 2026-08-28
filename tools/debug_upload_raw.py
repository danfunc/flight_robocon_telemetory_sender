#!/usr/bin/env python3
import time, serial

ser = serial.Serial("/dev/cu.usbmodem101", 115200, timeout=0.1, dsrdtr=True)
ser.dtr = True
ser.rts = True

print("⏳ Pico のブート処理完了を待機中 (3.5秒)...")
time.sleep(3.5)

# 残留バッファを排出
if ser.in_waiting > 0:
    boot_log = ser.read(ser.in_waiting).decode("utf-8", errors="replace")
    print(f"ブートログ ({len(boot_log)} 文字受信): ...")

with open("bazel-bin/user_apps/algo1_fast.bin", "rb") as f:
    data = f.read()

# 1. upload コマンド送信
print("\n--> upload コマンド送信")
ser.write(f"upload /bin/algo1.bin {len(data)}\r\n".encode("utf-8"))
ser.flush()

# 2. READY 応答待機
t0 = time.time()
ready_received = False
while time.time() - t0 < 4.0:
    if ser.in_waiting > 0:
        chunk = ser.read(ser.in_waiting)
        print("受信:", repr(chunk))
        if b"READY" in chunk:
            print(f"✅ READY 受信成功！バイナリデータ ({len(data)}B) を送信開始...")
            ready_received = True
            break
    time.sleep(0.05)

if not ready_received:
    print("❌ READY 受信失敗")
    ser.close()
    exit(1)

time.sleep(0.05)
# 3. 生バイナリ送信
ser.write(data)
ser.flush()

# 4. 書き込み応答待機
t0 = time.time()
while time.time() - t0 < 4.0:
    if ser.in_waiting > 0:
        chunk = ser.read(ser.in_waiting)
        print("データ後受信:", repr(chunk))
        if b"UPLOAD_OK" in chunk:
            print("🎉 UPLOAD_OK 受信成功！Flash FS 保存完了！")
            break
    time.sleep(0.05)

ser.close()
