#!/usr/bin/env python3
"""自律的センサフィードバック学習＆人間嗜好探索エンジン (Autonomous Human-in-the-Loop Learner)

テキスト指示ではなく、BNO055 センサの物理的な姿勢・傾き・加速度フィードバックを通じて
人間の「好み（点滅パターン・点滅速度）」をリアルタイムに自律学習・最適化する。

物理ジェスチャ判定基準 (BNO055 IMU):
  - 傾き右 (Roll > +25°):          速度アップ (Faster / Accelerate)
  - 傾き左 (Roll < -25°):          速度ダウン (Slower / Decelerate)
  - 前傾   (Pitch > +25°):         次のパターンへ切り替え (Next Pattern)
  - 後傾   (Pitch < -25°):         前のパターンへ切り替え (Prev Pattern)
  - 揺れ/タップ (|a| > 4.0 m/s²):   嫌悪・ランダム探索トリガー (Dislike / Mutation)
  - 水平静止 (|Roll|<12°, |Pitch|<12°, 3秒間): 好みの確定・満足 (Satisfaction / Preference Lock-in)

使用法:
  python3 tools/autonomous_preference_loop.py
"""

from __future__ import annotations

import asyncio
import math
import sys
import time
from bleak import BleakClient, BleakScanner

NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # notify (受信)
NUS_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # write  (送信)
DEVICE_NAME = "Shizuku UART"

PATTERN_NAMES = {
    0: "標準スクエア (500ms ON / 500ms OFF)",
    1: "航空機ダブルハートビート (60ms ON / 120ms OFF / 60ms ON / 760ms OFF)",
    2: "トリプルフラッシュ・ビーコン (40ms×3 / 760ms OFF)",
    3: "モールス S 信号 (100ms×3 / 500ms OFF)",
    4: "高速ストロボ (30ms ON / 30ms OFF)",
}


class AutonomousLearner:
    def __init__(self, client: BleakClient, init_pattern: int = 2, init_speed: int = 240):
        self.client = client
        self.rx_buffer = bytearray()
        self.cur_pattern = init_pattern  # 確定嗜好: トリプルフラッシュ・ビーコン (3回ストロボ)
        self.cur_speed = init_speed      # 確定嗜好: 240%
        self.last_cmd_time = 0.0
        self.cooldown_sec = 0.6  # ジェスチャ連打防止
        self.still_start_time = None
        self.satisfied = False
        self.interaction_history = []
        self.last_sensor_data = {}

    async def send_command(self, cmd: str):
        data = (cmd.strip() + "\n").encode("utf-8")
        try:
            await self.client.write_gatt_char(NUS_RX_UUID, data, response=False)
            self.last_cmd_time = time.time()
        except Exception as e:
            print(f"\n[送信エラー] {cmd}: {e}")

    async def set_pattern(self, pat_id: int):
        pat_id = max(0, min(4, pat_id))
        if pat_id != self.cur_pattern:
            self.cur_pattern = pat_id
            await self.send_command(f"LP{pat_id}")
            print(f"\n✨ [アルゴリズム適応] パターン変更 -> [{self.cur_pattern}] {PATTERN_NAMES.get(self.cur_pattern)}")
            self.interaction_history.append(("pattern", pat_id, time.time()))

    async def adjust_speed(self, delta_pct: int):
        new_speed = max(20, min(300, self.cur_speed + delta_pct))
        if new_speed != self.cur_speed:
            self.cur_speed = new_speed
            await self.send_command(f"LS{new_speed}")
            print(f"\n⚡ [パラメータ適応] 速度調整 -> {self.cur_speed}%")
            self.interaction_history.append(("speed", new_speed, time.time()))

    def handle_telemetry(self, sender, data: bytearray):
        self.rx_buffer.extend(data)
        while b"\n" in self.rx_buffer:
            line_bytes, self.rx_buffer = self.rx_buffer.split(b"\n", 1)
            line = line_bytes.decode("utf-8", errors="replace").strip()
            if line.startswith("PICO,"):
                self.process_sample(line)

    def process_sample(self, line: str):
        parts = line.split(",")
        if len(parts) < 19:
            return
        try:
            roll = int(parts[17]) / 100.0   # deg
            pitch = int(parts[18]) / 100.0  # deg
            head = int(parts[16]) / 100.0   # deg
            lax = int(parts[10]) / 1000.0   # m/s^2
            lay = int(parts[11]) / 1000.0
            laz = int(parts[12]) / 1000.0
            lin_accel_mag = math.sqrt(lax**2 + lay**2 + laz**2)

            self.last_sensor_data = {
                "roll": roll,
                "pitch": pitch,
                "head": head,
                "accel": lin_accel_mag,
            }

            asyncio.create_task(self.evaluate_feedback(roll, pitch, lin_accel_mag))
        except Exception:
            pass

    async def evaluate_feedback(self, roll: float, pitch: float, lin_accel: float):
        now = time.time()
        # 1. シェイク/タップ検出 (強い加速度変化)
        if lin_accel > 4.5 and (now - self.last_cmd_time > 0.8):
            print(f"\n💥 [センサ反応: タップ/シェイク (|a|={lin_accel:.1f} m/s²)] -> 嫌悪フィードバック検知！次パターンへ移行")
            await self.set_pattern((self.cur_pattern + 1) % 5)
            self.still_start_time = None
            return

        # 2. 傾きジェスチャ検出 (ロール / ピッチ)
        if now - self.last_cmd_time > self.cooldown_sec:
            if roll > 25.0:
                print(f"\n👉 [センサ反応: 右傾き (Roll={roll:+.1f}°)] -> 速度アップ指示を学習 (+20%)")
                await self.adjust_speed(+20)
                self.still_start_time = None
                return
            elif roll < -25.0:
                print(f"\n👈 [センサ反応: 左傾き (Roll={roll:+.1f}°)] -> 速度ダウン指示を学習 (-20%)")
                await self.adjust_speed(-20)
                self.still_start_time = None
                return
            elif pitch > 25.0:
                print(f"\n👆 [センサ反応: 前傾 (Pitch={pitch:+.1f}°)] -> 次のパターン選択指示を学習")
                await self.set_pattern((self.cur_pattern + 1) % 5)
                self.still_start_time = None
                return
            elif pitch < -25.0:
                print(f"\n👇 [センサ反応: 後傾 (Pitch={pitch:+.1f}°)] -> 前のパターン選択指示を学習")
                await self.set_pattern((self.cur_pattern - 1) % 5)
                self.still_start_time = None
                return

        # 3. 静止・満足度検出 (水平状態で 3 秒キープ)
        if abs(roll) < 12.0 and abs(pitch) < 12.0 and lin_accel < 1.2:
            if self.still_start_time is None:
                self.still_start_time = now
            else:
                elapsed = now - self.still_start_time
                if elapsed >= 3.0 and not self.satisfied:
                    self.satisfied = True
                    print(f"\n\n=======================================================")
                    print(f"🎯 [学習収束: 人間の嗜好パターンが確定・固定されました]")
                    print(f"   確定パターン: [{self.cur_pattern}] {PATTERN_NAMES.get(self.cur_pattern)}")
                    print(f"   確定速度:     {self.cur_speed}%")
                    print(f"   適応履歴回数: {len(self.interaction_history)} 回の相互作用を経て収束")
                    print(f"=======================================================\n")
                    # 実機 LED に確定サイン（0.6秒間の超高速ストロボ）を送ってから確定パターンに戻す
                    await self.send_command("LP4")
                    await self.send_command("LS300")
                    await asyncio.sleep(0.6)
                    await self.send_command(f"LP{self.cur_pattern}")
                    await self.send_command(f"LS{self.cur_speed}")
                    # Flash FS (/cfg/blink.conf) へ自動永続化
                    await self.send_command("LW")
                    print(f"💾 Flash FS (/cfg/blink.conf) へ好みの設定を自動永続化しました！(電源断復元対応)")
        else:
            self.still_start_time = None
            if self.satisfied:
                self.satisfied = False
                print("\n[嗜好探索再開] 姿勢の変化を検知しました。再学習モードに入ります。")


async def main():
    import argparse
    parser = argparse.ArgumentParser(description="Autonomous Sensor Feedback & Human Preference Learner")
    parser.add_argument("-c", "--continuous", action="store_true", help="Ctrl+C で終了するまで常時学習ループを実行")
    parser.add_argument("-d", "--duration", type=float, default=60.0, help="学習実行時間(秒) [既定: 60s]")
    args = parser.parse_args()

    print("================================================================================")
    print("  Shizuku 自律的センサフィードバック学習＆人間嗜好探索エンジン")
    print("================================================================================")
    print("【操作方法 (基板を手に持って動かしてください)】")
    print("  ・右に傾ける (Roll > +25°):        点滅を速くする (速度 +20%)")
    print("  ・左に傾ける (Roll < -25°):        点滅を遅くする (速度 -20%)")
    print("  ・前に傾ける (Pitch > +25°):       次の点滅パターンへ切り替え")
    print("  ・後ろに傾ける (Pitch < -25°):     前の点滅パターンへ切り替え")
    print("  ・軽く振る / タップ:              嫌悪・ランダムパターン切り替え")
    print("  ・水平にして 3秒静止:              「この光り方が好き」として確定・学習収束")
    print("--------------------------------------------------------------------------------")
    print(f"デバイス '{DEVICE_NAME}' を探索中...")

    device = await BleakScanner.find_device_by_filter(
        lambda d, ad: d.name == DEVICE_NAME, timeout=10.0
    )
    if not device:
        print(f"エラー: '{DEVICE_NAME}' が見つかりませんでした。")
        return 1

    print(f"接続中: {device.name} ({device.address})...")
    async with BleakClient(device) as client:
        learner = AutonomousLearner(client)
        print("BLE 接続確立。リアルタイム センサ学習ループを開始しました。")
        print("初期状態: [1] 航空機ダブルハートビート (100%)\n")

        await client.start_notify(NUS_TX_UUID, learner.handle_telemetry)

        t_end = None if args.continuous else (time.time() + args.duration)
        try:
            while t_end is None or time.time() < t_end:
                d = learner.last_sensor_data
                if d:
                    status_str = "【確定・満足】" if learner.satisfied else "【探索中】"
                    print(
                        f"\r{status_str} 現在:[{learner.cur_pattern}] {learner.cur_speed}% | "
                        f"Roll:{d['roll']:+5.1f}° Pitch:{d['pitch']:+5.1f}° | 加速度:{d['accel']:.2f} m/s²",
                        end="",
                        flush=True,
                    )
                await asyncio.sleep(0.1)
        except (KeyboardInterrupt, asyncio.CancelledError):
            print("\nユーザーによる中断を検知しました。")

        await client.stop_notify(NUS_TX_UUID)

    print("\n\n学習セッション完了。")
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
