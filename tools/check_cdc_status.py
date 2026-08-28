#!/usr/bin/env python3
import serial, time

PORT = "/dev/cu.usbmodem103"
ser = serial.Serial(PORT, 115200, timeout=0.1, dsrdtr=True)
ser.dtr = True
time.sleep(1.5)

# 改行を送ってシェルのプロンプトを待つ
ser.write(b"\r\n")
time.sleep(0.5)
resp = ser.read(ser.in_waiting or 1024).decode(errors="replace")
print("初期プロンプト:", repr(resp))

def send_and_wait(c, wait=1.5):
    ser.write(("  " + c + "\r\n").encode())
    time.sleep(wait)
    out = ""
    while ser.in_waiting:
        out += ser.read(ser.in_waiting).decode(errors="replace")
        time.sleep(0.1)
    return out

print("--- ls ---")
print(send_and_wait("ls"))

print("--- load /bin/algo1.bin ---")
print(send_and_wait("load /bin/algo1.bin", 2.0))

print("--- ps ---")
print(send_and_wait("ps"))

ser.close()
