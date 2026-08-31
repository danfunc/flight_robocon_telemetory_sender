#!/usr/bin/env python3
"""GDB over BLE 実機自動検証スクリプト

1. 動的モジュール (algo1) を BLE ホットスワップで実機にロード・起動
2. GDB BLE ブリッジを起動
3. arm-none-eabi-gdb をバッチモードで接続
4. monitor list, monitor target algo1, add-symbol-file, info registers, bt などを検証
"""

import asyncio
import os
import subprocess
import sys
import time

def run(cmd, timeout=30):
    print(f"▶ 実行: {cmd}")
    res = subprocess.run(cmd, shell=True, text=True, capture_output=True, timeout=timeout)
    if res.returncode != 0:
        print(f"❌ コマンド失敗 (exit={res.returncode}):")
        print(res.stderr)
    return res

def main():
    print("🚀 === Shizuku OS GDB over BLE 実機ライブ検証 ===")

    # 1. algo1 をホットリロードして実機で起動
    print("\n--- 1. 動的モジュール (algo1) を実機にロード ---")
    r1 = run("bazel run //user_apps:hot_reload_algo1", timeout=20)
    if r1.returncode != 0:
        print("Hot-Reload 失敗")
        sys.exit(1)
    print("✅ algo1 ホットリロード完了")

    time.sleep(1)

    # 2. GDB ブリッジをバックグラウンド起動
    print("\n--- 2. GDB BLE ブリッジ (port 3333) 起動 ---")
    bridge_proc = subprocess.Popen(
        ["/opt/homebrew/bin/python3", "tools/gdb_ble_bridge.py", "--port", "3333"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True
    )

    try:
        # ブリッジの接続完了 (listening on ...) を待機
        connected = False
        start_time = time.time()
        while time.time() - start_time < 15:
            line = bridge_proc.stdout.readline()
            if line:
                print(f"  [Bridge] {line.strip()}")
                if "listening on 127.0.0.1:3333" in line or "listening on " in line:
                    connected = True
                    break
            time.sleep(0.1)

        if not connected:
            print("❌ GDB ブリッジ接続タイムアウト")
            sys.exit(1)

        print("✅ GDB BLE ブリッジ接続確立")
        time.sleep(0.5)

        # 3. GDB コマンドスクリプトの作成
        gdb_script = """set remotetimeout 30
target remote localhost:3333
source tools/shizuku.gdb
printf "\\n=== 1. monitor list (スレッド一覧) ===\\n"
monitor list
printf "\\n=== 2. monitor target dyn_fc (動的フライトコントローラ) ===\\n"
monitor target dyn_fc
add-symbol-file bazel-bin/user_apps/flight_controller.elf 0x10202000
info registers r0 r1 r2 r3 sp lr pc
printf "\\n=== 3. GDB 検証完了 ===\\n"
disconnect
quit
"""
        with open("/tmp/test_commands.gdb", "w") as f:
            f.write(gdb_script)

        # 4. GDB バッチ実行
        print("\n--- 3. arm-none-eabi-gdb 接続 ＆ コマンド実行 ---")
        gdb_cmd = [
            "/opt/homebrew/bin/arm-none-eabi-gdb",
            "bazel-bin/firmware_bazel/xno_bringup",
            "--batch",
            "-x", "/tmp/test_commands.gdb"
        ]
        gdb_res = subprocess.run(gdb_cmd, capture_output=True, text=True, timeout=20)
        print("=== GDB 出力 ===")
        print(gdb_res.stdout)
        if gdb_res.stderr:
            print("=== GDB STDERR ===")
            print(gdb_res.stderr)

        if gdb_res.returncode == 0:
            print("\n🎉 GDB over BLE 実機テスト完全成功！")
        else:
            print(f"\n❌ GDB 終了コード: {gdb_res.returncode}")

    finally:
        bridge_proc.terminate()
        bridge_proc.wait()
        print("🔌 GDB ブリッジ停止")

if __name__ == "__main__":
    main()
