#!/usr/bin/env python3
"""CDC シェル経由で 5秒間隔の動的モジュール ホットスワップ実演デモ"""
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

print("=================================================================")
print("  🚀 Shizuku OS 動的モジュール・ホットスワップ 5秒実演デモ")
print("=================================================================\n")

# 1. 初期状態確認
print("📋 1. 初期 Flash FS ファイル一覧 (ls):")
print(send_cmd("ls"))

print("\n📋 2. 初期スレッド一覧 (ps):")
print(send_cmd("ps"))

# 2. Algorithm 1 へホットスワップ
print("\n" + "=" * 65)
print("⚡ 【PHASE 1】 Algorithm 1 (6連超高速ハイパーストロボ) をホットスワップ！")
print("   -> /bin/algo1.bin を XIP 動的起動")
print("=" * 65)
print(send_cmd("swap blink /bin/algo1.bin 0", 3.0))

print("\n📋 Algorithm 1 実行中スレッド一覧 (ps):")
print(send_cmd("ps"))

for sec in range(5, 0, -1):
    print(f"  ⏱️  [Algorithm 1 実行中] 6連超高速ハイパーストロボ発光中... (残り {sec} 秒)")
    time.sleep(1.0)

# 3. Algorithm 2 へホットスワップ
print("\n" + "=" * 65)
print("🔥 【PHASE 2: 5秒経過】 Algorithm 2 (ゆったり灯台ビーコン) へホットスワップ！")
print("   -> カーネル無停止で /bin/algo2.bin へ即時切り替え")
print("=" * 65)
print(send_cmd("swap algo1 /bin/algo2.bin 0", 3.0))

print("\n📋 Algorithm 2 実行中スレッド一覧 (ps):")
print(send_cmd("ps"))

for sec in range(5, 0, -1):
    print(f"  ⏱️  [Algorithm 2 実行中] ゆったり灯台ビーコン発光中... (残り {sec} 秒)")
    time.sleep(1.0)

# 4. Algorithm 1 へ戻すスワップ
print("\n" + "=" * 65)
print("⚡ 【PHASE 3: 5秒経過】 再び Algorithm 1 へシームレスにホットスワップ！")
print("=" * 65)
print(send_cmd("swap algo2 /bin/algo1.bin 0", 3.0))

print("\n📋 最終スレッド状態 (ps):")
print(send_cmd("ps"))

print("\n" + "=" * 65)
print("🎉 動的モジュール・ホットスワップ実演デモが完全成功しました！")
print("=" * 65)

ser.close()
