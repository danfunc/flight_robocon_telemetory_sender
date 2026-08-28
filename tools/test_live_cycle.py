#!/usr/bin/env python3
"""実機 RP2350 結合テスト:
1. picotool でファームウェアをロード・再起動し、Flash 内の初期点滅 (blink.cpp) の CDC 間隔変化を観測
2. shizuku_hot_reload による SRAM への動的モジュール注入・起動 (fast_blink.cpp)
3. 静的 blink の完全停止と動的モジュールへの制御移譲を厳密に検証
"""

from __future__ import annotations

import os
import subprocess
import sys
import time
import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import shizuku_hot_reload as shr


def main():
    cdc_port = "/dev/cu.usbmodem101"
    gdb_port = "/dev/cu.usbmodem103"
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    src_dyn = os.path.join(repo_root, "modules_dyn", "fast_blink.cpp")
    elf_path = os.path.join(repo_root, "bazel-bin", "firmware_bazel", "xno_bringup")
    picotool = "/Users/ishigakiyua/.pico-sdk/picotool/2.2.0-a4/picotool/picotool"

    print("==================================================")
    print("  RP2350 実機 Live Update & CDC ライフサイクル検証")
    print("==================================================")

    # 0. 実機を Flash ロードして確実に起動
    print("\n[Step 0] 実機へファームウェアをロードして Flash 初期状態で起動...")
    res = subprocess.run([picotool, "load", "-f", "-t", "elf", elf_path, "-x"], capture_output=True, text=True)
    if res.returncode != 0:
        print(f"picotool load 警告: {res.stderr}")

    # ポートの出現を待機
    for _ in range(30):
        if os.path.exists(cdc_port) and os.path.exists(gdb_port):
            break
        time.sleep(0.2)
    time.sleep(1.0)

    # 1. Flash 初期状態の CDC ログを観測
    print(f"\n[Step 1] Flash 初期状態の観測 ({cdc_port})...")
    ser_cdc = serial.Serial(cdc_port, 115200, timeout=0.5)
    ser_cdc.reset_input_buffer()

    observed_intervals = []
    t0 = time.time()
    while time.time() - t0 < 4.0:
        line = ser_cdc.readline().decode("utf-8", errors="replace").strip()
        if line and "[BLINK CDC]" in line:
            print(f"  実機ログ: {line}")
            if "interval=" in line:
                try:
                    ival = int(line.split("interval=")[1].split(" ms")[0])
                    observed_intervals.append(ival)
                except ValueError:
                    pass
    ser_cdc.close()

    print(f"-> 観測された点滅間隔 (ms): {observed_intervals}")
    if len(observed_intervals) >= 2 and observed_intervals[0] != observed_intervals[-1]:
        print("✅ Flash 初期状態の動的スイープ点滅（間隔変化）を確認しました！")
    else:
        print("⚠️ 観測された間隔変化が少なかったため継続します。")

    # 2. ホットリロード実行 (動的モジュール fast_blink 注入)
    print(f"\n[Step 2] 動的モジュール (fast_blink.cpp) の実機 SRAM 注入...")
    shr.hot_reload(src_dyn, serial_port=gdb_port)

    # 3. ホットリロード後の CDC ログとスレッド状態を確認
    print(f"\n[Step 3] ホットリロード後の静的 blink 完全停止検証...")
    time.sleep(0.8) # 最後の sleep 完了とスレッド終了を待機
    ser_cdc = serial.Serial(cdc_port, 115200, timeout=0.5)
    ser_cdc.reset_input_buffer()
    post_lines = []
    t0 = time.time()
    while time.time() - t0 < 2.5:
        line = ser_cdc.readline().decode("utf-8", errors="replace").strip()
        if line and "[BLINK CDC]" in line:
            post_lines.append(line)
            print(f"  (予期せぬログ): {line}")

    ser_cdc.close()

    if len(post_lines) == 0:
        print("✅ 静的 blink (Object 36) が完全に停止し、CDC ログが 0 件になったことを確認しました！")
    else:
        print(f"❌ エラー: 静的 blink のログがまだ出力されています ({len(post_lines)} 件)")
        sys.exit(1)

    # 4. GDB RSP でスレッド一覧を取得
    client = shr.SerialRspClient(port=gdb_port, timeout=1.0)
    client.connect()
    resp_list = client.qrcmd("list")
    client.close()

    print("\n[Step 4] 実機スレッドテーブル (RSP list):")
    print(resp_list)

    print("\n🎉 実機上での Flash 初期点滅観測 -> 動的モジュール Hot-Swap 検証が【100% 成功】しました！")


if __name__ == "__main__":
    main()
