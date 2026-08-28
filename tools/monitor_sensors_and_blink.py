#!/usr/bin/env python3
"""Shizuku BLE センサ監視 & LED パターン フィードバック ツール

BNO055 (IMU: 姿勢・オイラー角・加速度) および BME280 (気圧・高度・温度) の
リアルタイム テレメトリを BLE 経由で受信・表示し、手動での姿勢・センサ変化フィードバックを監視します。

使用例:
  python3 tools/monitor_sensors_and_blink.py
"""

from __future__ import annotations

import asyncio
import sys
import time
from bleak import BleakClient, BleakScanner

NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # notify (受信)
NUS_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # write  (送信)
DEVICE_NAME = "Shizuku UART"


class TelemetryMonitor:
    def __init__(self):
        self.rx_buffer = bytearray()
        self.packet_count = 0
        self.last_print = time.time()
        self.last_data = {}

    def handle_notify(self, sender, data: bytearray):
        self.rx_buffer.extend(data)
        while b"\n" in self.rx_buffer:
            line_bytes, self.rx_buffer = self.rx_buffer.split(b"\n", 1)
            line = line_bytes.decode("utf-8", errors="replace").strip()
            if line.startswith("PICO,"):
                self.parse_csv(line)

    def parse_csv(self, line: str):
        parts = line.split(",")
        # Format: PICO,seq,up_ms,temp,press,alt_baro,alt_fused,vel,speed,az,lax,lay,laz,gx,gy,gz,head,roll,pitch,...
        if len(parts) < 19:
            return
        try:
            self.packet_count += 1
            seq = int(parts[1])
            up_ms = int(parts[2])
            temp_c = int(parts[3]) / 100.0
            press_hpa = int(parts[4]) / 100.0
            alt_m = int(parts[5]) / 1000.0
            az = int(parts[9]) / 1000.0
            gx = int(parts[13]) / 1000.0
            gy = int(parts[14]) / 1000.0
            gz = int(parts[15]) / 1000.0
            head = int(parts[16]) / 100.0
            roll = int(parts[17]) / 100.0
            pitch = int(parts[18]) / 100.0

            self.last_data = {
                "seq": seq,
                "up_s": up_ms / 1000.0,
                "temp_c": temp_c,
                "press_hpa": press_hpa,
                "alt_m": alt_m,
                "head": head,
                "roll": roll,
                "pitch": pitch,
                "az": az,
                "gx": gx,
                "gy": gy,
                "gz": gz,
            }

            now = time.time()
            if now - self.last_print >= 0.2:  # 5Hz 表示更新
                self.last_print = now
                self.print_status()
        except Exception:
            pass

    def print_status(self):
        d = self.last_data
        if not d:
            return
        print(
            f"\r[BLE テレメトリ #{d['seq']:05d} | {d['up_s']:6.1f}s] "
            f"姿勢(Roll:{d['roll']:+6.1f}° Pitch:{d['pitch']:+6.1f}° Head:{d['head']:5.1f}°) | "
            f"環境(気温:{d['temp_c']:4.1f}°C 気圧:{d['press_hpa']:6.1f}hPa 高度:{d['alt_m']:+5.1f}m) | "
            f"重力G({d['gx']:+.2f}, {d['gy']:+.2f}, {d['gz']:+.2f})",
            end="",
            flush=True,
        )


async def main():
    print("==================================================")
    print("  Shizuku BLE センサ監視 & フィードバック")
    print("==================================================")
    print(f"デバイス '{DEVICE_NAME}' を探索中...")
    device = await BleakScanner.find_device_by_filter(
        lambda d, ad: d.name == DEVICE_NAME, timeout=10.0
    )
    if not device:
        print(f"エラー: '{DEVICE_NAME}' が見つかりませんでした。")
        return 1

    print(f"接続中: {device.name} ({device.address})...")
    monitor = TelemetryMonitor()

    async with BleakClient(device) as client:
        print("BLE 接続確立。テレメトリストリームを購読中...")
        print("--------------------------------------------------------------------------------")
        print("※ デバイスを傾けたり温めたりすると、リアルタイムで姿勢・気圧・気温が変化します。")
        print("--------------------------------------------------------------------------------")
        await client.start_notify(NUS_TX_UUID, monitor.handle_notify)

        t_end = time.time() + 20.0  # 20秒間監視
        while time.time() < t_end:
            await asyncio.sleep(0.1)

        await client.stop_notify(NUS_TX_UUID)

    print("\n\n監視完了。総受信パケット数:", monitor.packet_count)
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
