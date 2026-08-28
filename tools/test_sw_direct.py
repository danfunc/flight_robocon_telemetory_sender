#!/usr/bin/env python3
"""BLE で SW コマンドを単独送信して応答とテレメトリの変化を確認"""
import asyncio
from bleak import BleakClient, BleakScanner

NUS_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
DEVICE_NAME = "Shizuku UART"

def on_notify(sender, raw_data):
    text = raw_data.decode("utf-8", errors="replace")
    for line in text.splitlines():
        line = line.strip()
        if not line: continue
        if line.startswith("PICO,"):
            # テレメトリは短縮表示
            pass
        else:
            print(f"  📡 [REPLY] {line}")

async def run():
    print("🔍 BLE 接続中...")
    dev = await BleakScanner.find_device_by_filter(
        lambda d, ad: d.name == DEVICE_NAME, timeout=10.0)
    if not dev:
        print("❌ 見つかりません")
        return

    async with BleakClient(dev) as client:
        print("🔌 接続完了")
        await client.start_notify(NUS_TX_UUID, on_notify)
        await asyncio.sleep(1.0)

        # ファイル一覧
        print(">>> 送信: FL")
        await client.write_gatt_char(NUS_RX_UUID, b"FL\n", response=False)
        await asyncio.sleep(1.5)

        # ホットスワップ
        print(">>> 送信: SW/bin/algo1.bin")
        await client.write_gatt_char(NUS_RX_UUID, b"SW/bin/algo1.bin\n", response=False)
        await asyncio.sleep(3.0)

        # もう一度ファイル一覧 & 統計
        print(">>> 送信: S")
        await client.write_gatt_char(NUS_RX_UUID, b"S\n", response=False)
        await asyncio.sleep(2.0)

asyncio.run(run())
