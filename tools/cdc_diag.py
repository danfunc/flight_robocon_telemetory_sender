#!/usr/bin/env python3
"""BLE 経由アップロード + ロード + ホットスワップ検証（テレメトリ除外）"""
from __future__ import annotations
import asyncio
import binascii
import os
import sys
from bleak import BleakClient, BleakScanner

NUS_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
DEVICE_NAME = "Shizuku UART"

def on_notify(sender, raw_data):
    text = raw_data.decode("utf-8", errors="replace").strip()
    if not text.startswith("PICO,"):
        print(f"  📡 {text}")

async def ble_cmd(client, cmd, wait=1.0):
    print(f"\n>>> {cmd}")
    await client.write_gatt_char(NUS_RX_UUID, (cmd + "\n").encode(), response=False)
    await asyncio.sleep(wait)

async def upload(client, local, dest):
    with open(local, "rb") as f:
        data = f.read()
    hex_str = binascii.hexlify(data).decode()
    print(f"\n📦 upload {local} ({len(data)}B) -> {dest}")
    await ble_cmd(client, f"FB{dest} {len(data)}", 0.5)
    for i in range(0, len(hex_str), 64):
        await client.write_gatt_char(NUS_RX_UUID, f"FA{hex_str[i:i+64]}\n".encode(), response=False)
        await asyncio.sleep(0.05)
    await asyncio.sleep(0.3)
    await ble_cmd(client, "FC", 2.0)  # Flash 書き込み待ち

async def run():
    print("🔍 BLE 検索中...")
    dev = await BleakScanner.find_device_by_filter(
        lambda d, ad: d.name == DEVICE_NAME, timeout=10.0)
    if not dev:
        sys.exit("❌ デバイス見つからず")

    async with BleakClient(dev) as client:
        print(f"🔌 接続: {dev.address}")
        await client.start_notify(NUS_TX_UUID, on_notify)
        await asyncio.sleep(1.0)

        # 1. FL: ファイル一覧
        await ble_cmd(client, "FL", 1.0)

        # 2. アップロード
        await upload(client, "bazel-bin/user_apps/algo1_fast.bin", "/bin/algo1.bin")
        await upload(client, "bazel-bin/user_apps/algo2_slow.bin", "/bin/algo2.bin")

        # 3. FL: アップロード後の確認
        await ble_cmd(client, "FL", 1.0)

        # 4. LE0: blink 停止
        await ble_cmd(client, "LE0", 0.5)

        # 5. SW: algo1 ホットスワップ
        await ble_cmd(client, "SW/bin/algo1.bin", 3.0)

        # 6. 5秒待機
        print("\n⏱️  algo1 実行中 (5秒)...")
        await asyncio.sleep(5.0)

        # 7. SW: algo2 ホットスワップ
        await ble_cmd(client, "SW/bin/algo2.bin", 3.0)

        # 8. 5秒待機
        print("\n⏱️  algo2 実行中 (5秒)...")
        await asyncio.sleep(5.0)

        print("\n✅ 完了")

asyncio.run(run())
