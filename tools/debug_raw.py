#!/usr/bin/env python3
import time, serial

ser = serial.Serial("/dev/cu.usbmodem101", 115200, timeout=0.1, dsrdtr=True)
ser.dtr = True
ser.rts = True
time.sleep(0.5)

# プロンプト確認
ser.write(b"\r\n")
ser.flush()
time.sleep(0.2)
out = ser.read(ser.in_waiting or 1024)
print("初期プロンプト:", repr(out))

# format 送信
print("--> format 送信")
ser.write(b"format\r\n")
ser.flush()

t0 = time.time()
while time.time() - t0 < 3.0:
    if ser.in_waiting > 0:
        chunk = ser.read(ser.in_waiting)
        print("受信:", repr(chunk))
        if b"shizuku> " in chunk:
            print("format 完了検出！")
            break
    time.sleep(0.05)

time.sleep(0.2)
print("--> upload コマンド送信")
ser.write(b"upload /bin/algo1.bin 179\r\n")
ser.flush()

t0 = time.time()
while time.time() - t0 < 3.0:
    if ser.in_waiting > 0:
        chunk = ser.read(ser.in_waiting)
        print("upload応答受信:", repr(chunk))
        if b"READY" in chunk:
            print("READY 検出！")
            break
    time.sleep(0.05)

ser.close()
