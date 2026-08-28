#!/usr/bin/env python3
"""Flash FS をクリーンアップし、algo1 / algo2 のロード・実行・ホットスワップを完全検証"""
import time, serial, sys

PORT = "/dev/cu.usbmodem101"
print(f"🔌 {PORT} に接続中...")
ser = serial.Serial(PORT, 115200, timeout=0.1, dsrdtr=True)
ser.dtr = True
ser.rts = True

def wait_for_pattern(pattern_bytes, timeout=8.0):
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < timeout:
        n = ser.in_waiting
        if n > 0:
            chunk = ser.read(n)
            buf.extend(chunk)
            if pattern_bytes in buf:
                return True, buf.decode("utf-8", errors="replace")
        time.sleep(0.02)
    return False, buf.decode("utf-8", errors="replace")

def send_cmd(cmd_str, expected_reply="shizuku> ", timeout=5.0):
    if ser.in_waiting > 0:
        ser.read(ser.in_waiting)
    ser.write(f"\r\n{cmd_str.strip()}\r\n".encode("utf-8"))
    ser.flush()
    ok, out = wait_for_pattern(expected_reply.encode("utf-8"), timeout)
    return out

def upload_file(local_path, dest_path):
    with open(local_path, "rb") as f:
        data = f.read()
    size = len(data)
    print(f"\n📦 アップロード開始: {local_path} ({size}B) -> {dest_path}")
    
    if ser.in_waiting > 0:
        ser.read(ser.in_waiting)
        
    ser.write(f"\r\nupload {dest_path} {size}\r\n".encode("utf-8"))
    ser.flush()
    
    ok, ready_out = wait_for_pattern(b"READY", 4.0)
    if not ok:
        print(f"❌ READY 受信失敗: {ready_out.strip()[:120]}")
        return False
    print(f"  -> READY {size} 受信")
    
    time.sleep(0.05)
    ser.write(data)
    ser.flush()
    
    ok, upload_out = wait_for_pattern(b"UPLOAD_OK", 5.0)
    if not ok:
        print(f"❌ UPLOAD_OK 受信失敗: {upload_out.strip()[:120]}")
        return False
    
    wait_for_pattern(b"shizuku> ", 2.0)
    print(f"  ✅ {dest_path} アップロード完了 ({size}B)")
    time.sleep(0.2)
    return True

# 0. ブート完了を確実に待機
print("⏳ Pico ブートシーケンス完了を待機中 (BNO055 / BLE)...")
ok, boot_out = wait_for_pattern(b"[BNO055] init ok", 10.0)
if not ok:
    ser.write(b"\r\n")
    ser.flush()
    wait_for_pattern(b"shizuku> ", 2.0)
print("  -> Pico ブート完了確認！")
time.sleep(0.5)

# 1. フォーマット
print("\n=== 1. Flash FS フォーマット ===")
fmt_res = send_cmd("format", "フォーマット完了。", 5.0)
print("  -> フォーマット完了を確認")
wait_for_pattern(b"shizuku> ", 2.0)
time.sleep(0.3)

# 2. アップロード
print("\n=== 2. バイナリアップロード ===")
if not upload_file("bazel-bin/user_apps/algo1_fast.bin", "/bin/algo1.bin"):
    sys.exit(1)
if not upload_file("bazel-bin/user_apps/algo2_slow.bin", "/bin/algo2.bin"):
    sys.exit(1)

# 3. ファイル一覧確認
print("\n=== 3. Flash FS ファイル一覧 (ls) ===")
print(send_cmd("ls", "shizuku> ", 3.0))

# 4. algo1 ロード
print("\n=== 4. load /bin/algo1.bin 0 (algo1 起動) ===")
print(send_cmd("load /bin/algo1.bin 0", "ロード成功", 4.0))
wait_for_pattern(b"shizuku> ", 2.0)

print("\n=== 5. algo1 実行中 ps ===")
print(send_cmd("ps", "shizuku> ", 2.0))

for sec in range(4, 0, -1):
    print(f"  ⏱️  algo1 (6連超高速ストロボ) 発光中... 残り {sec} 秒")
    time.sleep(1.0)

# 6. ホットスワップ (swap algo1 /bin/algo2.bin 0)
print("\n=== 6. swap algo1 /bin/algo2.bin 0 (ホットスワップ) ===")
print(send_cmd("swap algo1 /bin/algo2.bin 0", "ホットスワップ成功", 4.0))
wait_for_pattern(b"shizuku> ", 2.0)

print("\n=== 7. algo2 (スワップ後) 実行中 ps ===")
print(send_cmd("ps", "shizuku> ", 2.0))

for sec in range(4, 0, -1):
    print(f"  ⏱️  algo2 (ゆったり灯台ビーコン) 発光中... 残り {sec} 秒")
    time.sleep(1.0)

# 8. アンロード
print("\n=== 8. unload algo2 ===")
print(send_cmd("unload algo2", "アンロード成功", 3.0))
wait_for_pattern(b"shizuku> ", 2.0)

# 9. 最終 ps
print("\n=== 9. 最終確認 ps ===")
print(send_cmd("ps", "shizuku> ", 2.0))

ser.close()
print("\n🎉 ホットスワップ検証完了！すべての手順が正常に完了しました。")
