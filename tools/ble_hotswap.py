#!/usr/bin/env python3
"""BLE 経由の Flash FS 動的モジュール（ELF/BIN）アップロード ＆ ホットスワップツール

BLE NUS 経由で Flash FS (/bin/...) へ動的モジュール（ELF/BIN）を送信し、
実行中のシステムを再起動することなく、即座に新しいアルゴリズムへホットスワップします。

使用例:
  python3 tools/ble_hotswap.py --file bazel-bin/user_apps/pulse_blink.bin --dest /bin/pulse_blink.bin
  python3 tools/ble_hotswap.py --file bazel-bin/user_apps/pulse_blink.elf --dest /bin/pulse_blink.elf
"""

from __future__ import annotations

import argparse
import asyncio
import binascii
import os
import sys
from bleak import BleakClient, BleakScanner

NUS_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
DEVICE_NAME = "Shizuku UART"


async def ble_hotswap(file_path: str, dest_path: str, chunk_size: int = 64):
    if not os.path.exists(file_path):
        sys.exit(f"エラー: ファイルが見つかりません: {file_path}")

    with open(file_path, "rb") as f:
        data = f.read()

    size = len(data)
    hex_str = binascii.hexlify(data).decode("ascii")

    print(f"📦 ホットスワップ対象: {file_path} ({size} bytes)")
    print(f"🎯 Flash FS 転送先: {dest_path}")

    print(f"🔍 BLE デバイス '{DEVICE_NAME}' を検索中...")
    device = await BleakScanner.find_device_by_filter(
        lambda d, ad: d.name == DEVICE_NAME, timeout=10.0
    )
    if not device:
        sys.exit(f"❌ '{DEVICE_NAME}' が見つかりませんでした")

    responses = []

    def on_notify(sender, raw_data):
        text = raw_data.decode("utf-8", errors="replace")
        responses.append(text)
        print(f"  [BLE RX] {text.strip()}")

    async with BleakClient(device) as client:
        print(f"🔌 BLE 接続完了: {device.address}")
        await client.start_notify(NUS_TX_UUID, on_notify)
        await asyncio.sleep(0.3)

        # 1. アップロード開始 (FB)
        print(f"\n1️⃣ Flash FS アップロード開始 (FB{dest_path} {size})...")
        cmd_fb = f"FB{dest_path} {size}\n".encode("utf-8")
        await client.write_gatt_char(NUS_RX_UUID, cmd_fb, response=False)
        await asyncio.sleep(0.3)

        # 2. データチャンク送信 (FA<hex>)
        print("2️⃣ データチャンクを送信中...")
        for i in range(0, len(hex_str), chunk_size):
            chunk = hex_str[i : i + chunk_size]
            cmd_fa = f"FA{chunk}\n".encode("utf-8")
            await client.write_gatt_char(NUS_RX_UUID, cmd_fa, response=False)
            await asyncio.sleep(0.04)

        await asyncio.sleep(0.3)

        # 3. Flash FS へ書き込み・コミット (FC)
        print("3️⃣ Flash FS へコミット (FC)...")
        await client.write_gatt_char(NUS_RX_UUID, b"FC\n", response=False)
        await asyncio.sleep(0.5)

        # 4. Flash FS ファイル一覧確認 (FL)
        print("\n4️⃣ Flash FS ファイル一覧を確認 (FL)...")
        await client.write_gatt_char(NUS_RX_UUID, b"FL\n", response=False)
        await asyncio.sleep(0.5)

        # 5. ホットスワップ実行 (SW)
        print(f"\n5️⃣ 🔥 アルゴリズムをホットスワップ実行中 (SW{dest_path})...")
        cmd_sw = f"SW{dest_path}\n".encode("utf-8")
        await client.write_gatt_char(NUS_RX_UUID, cmd_sw, response=False)
        await asyncio.sleep(0.8)

        print("\n🎉 ホットスワップ処理完了！")


def main():
    parser = argparse.ArgumentParser(
        description="BLE Flash FS Module Hot-Swap Tool"
    )
    parser.add_argument(
        "--file",
        default="bazel-bin/user_apps/pulse_blink.bin",
        help="Local ELF or BIN module file",
    )
    parser.add_argument(
        "--dest",
        default="/bin/pulse_blink.bin",
        help="Flash FS destination path",
    )
    args = parser.parse_args()

    asyncio.run(ble_hotswap(args.file, args.dest))


if __name__ == "__main__":
    main()
