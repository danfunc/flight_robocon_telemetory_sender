#!/usr/bin/env python3
"""5秒間隔の動的モジュール ホットスワップ実演デモ

Flash FS 上に2つの異なるアルゴリズムモジュールを配置し、
組み込み blink オブジェクトを RAM 上で停止（IPC メソッド呼び出し）させた上で、
5秒後に即座にアルゴリズムをホットスワップして点滅パターンをダイナミックに切り替えます。

- Algorithm 1: 6連超高速ハイパーストロボ (/bin/algo1.bin)
- Algorithm 2: ゆったり灯台ビーコン (/bin/algo2.bin)
"""

from __future__ import annotations

import asyncio
import binascii
import os
import sys
import time
from bleak import BleakClient, BleakScanner

NUS_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
DEVICE_NAME = "Shizuku UART"


async def upload_module(client: BleakClient, file_path: str, dest_path: str):
    if not os.path.exists(file_path):
        sys.exit(f"❌ ファイルが見つかりません: {file_path}")

    with open(file_path, "rb") as f:
        data = f.read()

    size = len(data)
    hex_str = binascii.hexlify(data).decode("ascii")

    print(f"📦 アップロード中: {file_path} ({size} bytes) -> {dest_path}")
    cmd_fb = f"FB{dest_path} {size}\n".encode("utf-8")
    await client.write_gatt_char(NUS_RX_UUID, cmd_fb, response=False)
    await asyncio.sleep(0.3)

    chunk_size = 64  # 32 bytes per chunk
    for i in range(0, len(hex_str), chunk_size):
        chunk = hex_str[i : i + chunk_size]
        cmd_fa = f"FA{chunk}\n".encode("utf-8")
        await client.write_gatt_char(NUS_RX_UUID, cmd_fa, response=False)
        await asyncio.sleep(0.04)

    await asyncio.sleep(0.3)
    await client.write_gatt_char(NUS_RX_UUID, b"FC\n", response=False)
    await asyncio.sleep(0.4)


async def run_demo():
    print("=" * 65)
    print("  🚀 Shizuku OS 動的モジュール・ホットスワップ 5秒デモ")
    print("=" * 65)

    algo1_bin = "bazel-bin/user_apps/algo1_fast.bin"
    algo2_bin = "bazel-bin/user_apps/algo2_slow.bin"

    print(f"🔍 BLE デバイス '{DEVICE_NAME}' を検索中...")
    device = await BleakScanner.find_device_by_filter(
        lambda d, ad: d.name == DEVICE_NAME, timeout=10.0
    )
    if not device:
        sys.exit(f"❌ '{DEVICE_NAME}' が見つかりませんでした。")

    responses = []

    def on_notify(sender, raw_data):
        text = raw_data.decode("utf-8", errors="replace")
        responses.append(text)
        for line in text.strip().split("\n"):
            if any(k in line for k in ["SWAP", "FB", "FC", "FL", "LED", "AUTOPILOT"]):
                print(f"  📡 [受信] {line}")

    async with BleakClient(device) as client:
        print(f"🔌 BLE 接続確立: {device.address}\n")
        await client.start_notify(NUS_TX_UUID, on_notify)
        await asyncio.sleep(0.3)

        # 0. 組み込み blink オブジェクトのメソッドを呼び出して RAM 上で停止
        print("🛑 組み込み blink オブジェクトを呼び出して一時停止中 (RAM上で停止)...")
        await client.write_gatt_char(NUS_RX_UUID, b"LE0\n", response=False)
        await asyncio.sleep(0.4)

        # 1. 2つのアルゴリズムを Flash FS へ転送
        print("【STEP 1】 Flash FS へ 2 種類のアルゴリズムモジュールを配置中...")
        await upload_module(client, algo1_bin, "/bin/algo1.bin")
        await upload_module(client, algo2_bin, "/bin/algo2.bin")

        print("📋 Flash FS 内のファイル確認 (FL)...")
        await client.write_gatt_char(NUS_RX_UUID, b"FL\n", response=False)
        await asyncio.sleep(0.5)

        # 2. Algorithm 1 へホットスワップ
        print("\n" + "=" * 65)
        print("⚡ 【PHASE 1】 Algorithm 1 (6連超高速ハイパーストロボ) をホットスワップ！")
        print("   -> /bin/algo1.bin を XIP 動的起動")
        print("=" * 65)
        await client.write_gatt_char(NUS_RX_UUID, b"SW/bin/algo1.bin\n", response=False)

        # 5秒カウントダウン
        for sec in range(5, 0, -1):
            print(f"  ⏱️  [Algorithm 1 実行中] 6連超高速ハイパーストロボ発光中... (残り {sec} 秒)")
            await asyncio.sleep(1.0)

        # 3. Algorithm 2 へホットスワップ
        print("\n" + "=" * 65)
        print("🔥 【PHASE 2: 5秒経過】 Algorithm 2 (ゆったり灯台ビーコン) へホットスワップ！")
        print("   -> カーネル無停止で /bin/algo2.bin へ即時切り替え")
        print("=" * 65)
        await client.write_gatt_char(NUS_RX_UUID, b"SW/bin/algo2.bin\n", response=False)

        # 5秒カウントダウン
        for sec in range(5, 0, -1):
            print(f"  ⏱️  [Algorithm 2 実行中] ゆったり灯台ビーコン発光中... (残り {sec} 秒)")
            await asyncio.sleep(1.0)

        print("\n" + "=" * 65)
        print("🎉 ホットスワップ実演デモが正常に完了しました！")
        print("   カーネル再起動なしで、2つのアルゴリズムがシームレスに切り替わりました。")
        print("=" * 65)


def main():
    asyncio.run(run_demo())


if __name__ == "__main__":
    main()
