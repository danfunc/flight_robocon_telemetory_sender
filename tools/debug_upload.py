#!/usr/bin/env python3
import time, serial

ser = serial.Serial("/dev/cu.usbmodem101", 115200, timeout=1.0, dsrdtr=True)
ser.dtr = True
ser.rts = True
time.sleep(0.5)

with open("bazel-bin/user_apps/algo1_fast.bin", "rb") as f:
    data = f.read()

ser.reset_input_buffer()
ser.write(f"  upload /bin/algo1.bin {len(data)}\r\n".encode("utf-8"))
ser.flush()

time.sleep(0.5)
ready_resp = ser.read(ser.in_waiting or 1024).decode("utf-8", errors="replace")
print("READY応答:", repr(ready_resp))

ser.write(data)
ser.flush()

time.sleep(1.0)
upload_resp = ser.read(ser.in_waiting or 1024).decode("utf-8", errors="replace")
print("UPLOAD応答:", repr(upload_resp))

ser.close()
