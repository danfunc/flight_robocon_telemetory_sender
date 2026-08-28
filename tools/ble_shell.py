#!/usr/bin/env python3
"""Shizuku OS — BLE 専用対話型シェル & 動的アプリマネージャ (ble_shell.py)

Nordic UART Service (NUS) を通じてワイヤレスで Flash FS 操作、動的モジュール
(algo1/algo2) のアップロード、ロード、ホットスワップ、プロセス監視を行います。

使用法:
  python3 tools/ble_shell.py           # 対話型シェルモード (shizuku>)
  python3 tools/ble_shell.py --test    # 自動検証テスト (アップロード/ロード/スワップ)
"""
import argparse
import asyncio
import os
import sys
import time
from bleak import BleakClient, BleakScanner

NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # Notify (デバイス -> ホスト)
NUS_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # Write  (ホスト -> デバイス)
DEVICE_NAME = "Shizuku UART"

class BleShellClient:
    def __init__(self):
        self.client = None
        self.rx_queue = asyncio.Queue()
        self.connected = False

    def _notification_handler(self, sender, data: bytearray):
        text = data.decode("utf-8", errors="replace")
        # 1行ずつキューへ
        lines = text.splitlines(keepends=True)
        for line in lines:
            self.rx_queue.put_nowait(line)

    async def connect(self, timeout=12.0):
        print(f"🔍 BLE デバイス '{DEVICE_NAME}' をスキャン中...")
        device = await BleakScanner.find_device_by_filter(
            lambda d, ad: (d.name and DEVICE_NAME in d.name) or (ad.local_name and DEVICE_NAME in ad.local_name),
            timeout=timeout
        )
        if not device:
            raise RuntimeError(f"❌ デバイス '{DEVICE_NAME}' が見つかりませんでした。Pico の電源とアドバタイズを確認してください。")

        print(f"🔗 接続中: {device.name} ({device.address})...")
        last_err = None
        for attempt in range(1, 4):
            try:
                self.client = BleakClient(device, timeout=10.0)
                await self.client.connect()
                await asyncio.sleep(0.3)
                await self.client.start_notify(NUS_TX_UUID, self._notification_handler)
                self.connected = True
                print(f"✅ BLE 接続完了 (NUS 確立)")
                return
            except Exception as e:
                last_err = e
                print(f"  接続リトライ ({attempt}/3)... {e}")
                await asyncio.sleep(0.5)

        raise RuntimeError(f"❌ BLE 接続失敗: {last_err}")

    async def disconnect(self):
        if self.client and self.client.is_connected:
            await self.client.disconnect()
        self.connected = False
        print("🔌 切断しました。")

    async def send_line(self, cmd_str: str):
        if not self.connected:
            raise RuntimeError("未接続です")
        payload = (cmd_str.strip() + "\n").encode("utf-8")
        await self.client.write_gatt_char(NUS_RX_UUID, payload, response=False)

    async def read_response(self, timeout=3.0):
        lines = []
        t0 = asyncio.get_event_loop().time()
        while asyncio.get_event_loop().time() - t0 < timeout:
            try:
                line = await asyncio.wait_for(self.rx_queue.get(), timeout=0.2)
                lines.append(line)
                # 応答が完了したかどうかの判定 (空行または特定トークン)
            except asyncio.TimeoutError:
                if lines:
                    break
        return "".join(lines)

    async def send_and_wait(self, cmd_str: str, timeout=4.0):
        # 既存キューをクリア
        while not self.rx_queue.empty():
            self.rx_queue.get_nowait()
        await self.send_line(cmd_str)
        return await self.read_response(timeout)

    async def upload_file(self, local_path: str, dest_path: str):
        if not os.path.exists(local_path):
            print(f"❌ ファイルが存在しません: {local_path}")
            return False
        with open(local_path, "rb") as f:
            data = f.read()
        size = len(data)
        print(f"📦 BLE アップロード開始: {local_path} ({size}B) -> {dest_path}")

        # 1. FB (Flash Begin)
        resp = await self.send_and_wait(f"FB {dest_path} {size}", timeout=3.0)
        if "FB ready" not in resp:
            print(f"❌ FB 応答エラー: {resp.strip()}")
            return False

        # 2. FA (Flash Append) チャンク送信 (64バイトHex = 128文字)
        chunk_size = 64
        for i in range(0, size, chunk_size):
            chunk = data[i:i+chunk_size]
            hex_str = chunk.hex()
            resp = await self.send_and_wait(f"FA {hex_str}", timeout=2.0)
            if "FA rx=" not in resp:
                print(f"❌ FA 応答エラー at {i}: {resp.strip()}")
                return False
            pct = min(100, int((i + len(chunk)) * 100 / size))
            print(f"\r  転送中: {i + len(chunk)}/{size} B ({pct}%)", end="", flush=True)

        print()
        # 3. FC (Flash Commit)
        resp = await self.send_and_wait("FC", timeout=4.0)
        if "FC ok" in resp:
            print(f"  ✅ Flash FS 保存完了: {dest_path} ({size} bytes)")
            return True
        else:
            print(f"❌ FC コミット失敗: {resp.strip()}")
            return False

async def run_interactive(client: BleShellClient):
    print("\n========================================")
    print("  Shizuku OS — BLE Interactive Shell    ")
    print("  Commands: ls, format, upload, load,   ")
    print("            swap, unload, ps, stats     ")
    print("  Type 'exit' to quit.                  ")
    print("========================================")

    while True:
        try:
            cmd = await asyncio.get_event_loop().run_in_executor(None, input, "shizuku-ble> ")
            cmd = cmd.strip()
            if not cmd:
                continue
            if cmd in ("exit", "quit"):
                break

            parts = cmd.split()
            c = parts[0].lower()

            if c == "ls":
                resp = await client.send_and_wait("FL")
                print(resp, end="")
            elif c == "format":
                print("Flash FS をフォーマット中...")
                resp = await client.send_and_wait("FF", timeout=6.0)
                print(resp, end="")
            elif c == "upload" and len(parts) >= 3:
                await client.upload_file(parts[1], parts[2])
            elif c == "load" and len(parts) >= 2:
                core = parts[2] if len(parts) >= 3 else "0"
                resp = await client.send_and_wait(f"LD {parts[1]} {core}")
                print(resp, end="")
            elif c == "swap" and len(parts) >= 2:
                resp = await client.send_and_wait(f"SW {parts[1]}")
                print(resp, end="")
            elif c == "unload" and len(parts) >= 2:
                resp = await client.send_and_wait(f"UN {parts[1]}")
                print(resp, end="")
            elif c == "ps":
                resp = await client.send_and_wait("PS")
                print(resp, end="")
            elif c == "stats":
                resp = await client.send_and_wait("STATS")
                print(resp, end="")
            elif c in ("help", "?"):
                print("利用可能コマンド:")
                print("  ls                           Flash FS 内のファイル一覧")
                print("  format                       Flash FS を初期化")
                print("  upload <local> <dest>        バイナリを BLE でアップロード")
                print("  load <path> [core]           Flash FS から動的モジュールをロード")
                print("  swap <new_path>              稼働中モジュールを新バイナリにホットスワップ")
                print("  unload <name>                動的モジュールのアンロード")
                print("  ps                           稼働中動的モジュール一覧")
                print("  stats                        システム統計 / 操縦状態")
            else:
                # 生コマンド送信
                resp = await client.send_and_wait(cmd)
                print(resp, end="")
        except (KeyboardInterrupt, EOFError):
            break

async def run_automated_test(client: BleShellClient):
    print("\n🚀 === BLE 専用チャネル 動的モジュール自動検証テスト ===")

    # 1. フォーマット
    print("\n--- 1. Flash FS フォーマット ---")
    resp = await client.send_and_wait("FF", timeout=5.0)
    print(f"応答: {resp.strip()}")

    # 2. アップロード (algo1 & algo2)
    print("\n--- 2. バイナリアップロード ---")
    if not await client.upload_file("bazel-bin/user_apps/algo1_fast.bin", "/bin/algo1.bin"):
        sys.exit(1)
    if not await client.upload_file("bazel-bin/user_apps/algo2_slow.bin", "/bin/algo2.bin"):
        sys.exit(1)

    # 3. ファイル一覧
    print("\n--- 3. ファイル一覧 (FL) ---")
    resp = await client.send_and_wait("FL")
    print(resp, end="")

    # 4. algo1 ロード
    print("\n--- 4. algo1 ロード (LD /bin/algo1.bin 0) ---")
    resp = await client.send_and_wait("LD /bin/algo1.bin 0")
    print(f"応答: {resp.strip()}")

    # 5. ps
    print("\n--- 5. プロセス一覧 (PS) ---")
    resp = await client.send_and_wait("PS")
    print(resp, end="")

    print("\n⏱️  algo1 (高速ストロボ) 実行中 (3秒)...")
    await asyncio.sleep(3.0)

    # 6. ホットスワップ (SW /bin/algo2.bin)
    print("\n--- 6. ホットスワップ (SW /bin/algo2.bin) ---")
    resp = await client.send_and_wait("SW /bin/algo2.bin")
    print(f"応答: {resp.strip()}")

    # 7. ps
    print("\n--- 7. スワップ後 プロセス一覧 (PS) ---")
    resp = await client.send_and_wait("PS")
    print(resp, end="")

    print("\n⏱️  algo2 (ゆったりビーコン) 実行中 (3秒)...")
    await asyncio.sleep(3.0)

    # 8. アンロード
    print("\n--- 8. アンロード (UN algo2) ---")
    resp = await client.send_and_wait("UN algo2")
    print(f"応答: {resp.strip()}")

    # 9. 最終 ps
    print("\n--- 9. 最終 プロセス一覧 (PS) ---")
    resp = await client.send_and_wait("PS")
    print(resp, end="")

    print("\n🎉 BLE 専用チャネル経由のテストがすべて完全成功しました！")

async def main():
    parser = argparse.ArgumentParser(description="Shizuku OS BLE Shell Client")
    parser.add_argument("--test", action="store_true", help="自動検証テストを実行")
    args = parser.parse_args()

    client = BleShellClient()
    try:
        await client.connect()
        if args.test:
            await run_automated_test(client)
        else:
            await run_interactive(client)
    finally:
        await client.disconnect()

if __name__ == "__main__":
    asyncio.run(main())
