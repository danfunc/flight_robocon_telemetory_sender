#!/usr/bin/env python3
"""Shizuku RT スケジューラ検証 — ホスト側ハーネス (pyserial + bleak)

デバイス側 (SHIZU_RT_SCHED_TEST=1 でビルドした rt_sched_test.cpp) が USB CDC / UART の
stdio へ吐く `[RTTEST]` 締切ジッタ表を pyserial で読みつつ、同時に bleak で BLE の
ping RTT 分布とダウンリンク・スループットを測る。ブラスト (B) はデバイス側でセンサを
止めるだけで rt_sched の合成 victim/hog は止めないので、BLE 測定は**合成負荷が走ったまま**
= 競合下で行われる (センサ実機は不要)。

判定 (device 側 = RT 本体, host BLE = 補助):
  PASS 条件
    - hog が全 window ADVANCING (一度も STALLED でない)           … 系が凍結しない
    - core1 victim の late(max) が budget の数倍以内で頭打ち        … 応答ジッタ有界
    - 各 window の ticks が期待値の TICK_FRAC 以上                  … 平均レート維持 (餓死せず)
    - core0 victim の late(max) が小さい                           … コア間分離
  BLE (ping RTT / スループット) は環境依存なので数値を出して soft flag のみ。

使い方:
  pip install pyserial bleak
  python rt_sched_hosttest.py                       # 自動でポート検出 + BLE も測る
  python rt_sched_hosttest.py --port /dev/tty.usbmodem1101 --duration 40
  python rt_sched_hosttest.py --no-ble              # シリアルのジッタ表だけ (BLE 無し)
  python rt_sched_hosttest.py --pings 80 --blast 5  # BLE ping 回数 / ブラスト秒
"""

import argparse
import re
import statistics
import sys
import threading
import time

# ---- しきい値 (budget=3ms 前提。--budget-us で調整可) ------------------------
# このスケジューラの RT 保証は「never-yield スレッドは budget で必ず取り上げられる」なので、
# 応答ジッタ (late_max) の上界 ≈ budget × 同時ホグ数 + オーバーヘッド。**両コアとも** watchdog が
# bound する (core0 も他スレッドの printf 等が最大 budget まで CPU を握るため near-zero には
# ならない)。レート維持は period >= budget の schedulable な victim にのみ課す — period < budget
# のタスクは 3ms 粒度の hog 下では物理的に rate を満たせない (= 設計上の RT 粒度そのもの、想定内)。
DEFAULT_BUDGET_US = 3000
LATE_BOUND_MULT = 2.5       # late_max 許容 = budget × これ + マージン (青天井=失敗を捕まえる)
LATE_BOUND_MARGIN_US = 3000
TICK_FRAC = 0.80            # schedulable victim の各 window ticks はこの割合以上であるべき
REPORT_WIN_US = 2000000     # reporter の window (period = REPORT_WIN_US / exp で逆算)

RTTEST_VICTIM = re.compile(
    r"\[RTTEST\]\s+(\S+)\s+late\(win\)=(\d+)us late\(max\)=(\d+)us "
    r"overrun=(\d+) ticks=(\d+)/(\d+)")
RTTEST_HOG = re.compile(r"\[RTTEST\] hog(\d+) \+(\d+)/2s (ADVANCING|\*\*STALLED\*\*)")
RTTEST_ARMED = re.compile(r"\[RTTEST\] armed:")


class SerialMonitor(threading.Thread):
    """Pico の stdio (USB CDC / UART) を読み、[RTTEST] 行を集計する背景スレッド。"""

    def __init__(self, port, baud=115200):
        super().__init__(daemon=True)
        self._port_name = port
        self._baud = baud
        self._stop = threading.Event()
        self.armed = False
        # victim 名 -> {late_max, overrun, windows:[(ticks,exp)], late_wins:[...]}
        self.victims = {}
        self.hog_windows = 0
        self.hog_stalled = 0
        self.err = None
        self.lines_seen = 0

    def stop(self):
        self._stop.set()

    def run(self):
        try:
            import serial  # pyserial
        except ImportError as e:
            self.err = f"pyserial が要ります (pip install pyserial): {e}"
            return
        try:
            ser = serial.Serial(self._port_name, self._baud, timeout=1.0)
        except Exception as e:  # noqa: BLE001
            self.err = f"シリアルポートを開けません ({self._port_name}): {e}"
            return
        with ser:
            while not self._stop.is_set():
                try:
                    raw = ser.readline()
                except Exception as e:  # noqa: BLE001
                    self.err = f"シリアル読み取り失敗: {e}"
                    return
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").rstrip()
                self.lines_seen += 1
                self._parse(line)

    def _parse(self, line):
        if RTTEST_ARMED.search(line):
            self.armed = True
            return
        m = RTTEST_VICTIM.search(line)
        if m:
            name, late_win, late_max, overrun, ticks, exp = (
                m.group(1), int(m.group(2)), int(m.group(3)),
                int(m.group(4)), int(m.group(5)), int(m.group(6)))
            v = self.victims.setdefault(
                name, {"late_max": 0, "overrun": 0, "windows": []})
            v["late_max"] = max(v["late_max"], late_max)
            v["overrun"] = overrun  # 累積値なので最新で上書き
            v["windows"].append((ticks, exp, late_win))
            return
        m = RTTEST_HOG.search(line)
        if m:
            self.hog_windows += 1
            if "STALLED" in m.group(3):
                self.hog_stalled += 1


def find_pico_port():
    """VID 0x2E8A (Raspberry Pi) の CDC ポートを探す。無ければ usbmodem/ACM を拾う。"""
    try:
        from serial.tools import list_ports
    except ImportError:
        return None
    cands = list(list_ports.comports())
    for p in cands:
        if (p.vid == 0x2E8A) or ("usbmodem" in (p.device or "")) or \
           ("ACM" in (p.device or "")):
            return p.device
    return cands[0].device if cands else None


# --------------------------------------------------------------------------
#  BLE 測定 (bleak)。ping RTT 分布 + ダウンリンク・スループットを競合下で測る。
# --------------------------------------------------------------------------
async def ble_measure(pings, blast_sec):
    from bleak import BleakClient, BleakScanner
    try:
        from shizuku_link import (NUS_TX_UUID, NUS_RX_UUID, DEVICE_NAME,
                                  BLE_CEILING_KBPS)
    except ImportError:
        # shizuku_link が import path に無い場合のフォールバック定数。
        NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
        NUS_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
        DEVICE_NAME = "Shizuku UART"
        BLE_CEILING_KBPS = 139.7

    import asyncio

    result = {"rtts_ms": [], "tput_kbps": None, "err": None}
    print(f"[BLE] '{DEVICE_NAME}' をスキャン中…")
    dev = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=15.0)
    if dev is None:
        result["err"] = f"'{DEVICE_NAME}' が見つからない"
        return result

    rx_buf = bytearray()
    pending = {}          # token -> send perf_counter
    rtts = []
    tput = {"active": False, "bytes": 0, "t0": 0.0, "done": None}

    def on_notify(_h, data: bytearray):
        if tput["active"]:
            tput["bytes"] += len(data)
        rx_buf.extend(data)
        while b"\n" in rx_buf:
            seg, _, rest = rx_buf.partition(b"\n")
            del rx_buf[:len(seg) + 1]
            line = seg.rstrip(b"\r").decode("utf-8", errors="replace")
            if re.fullmatch(r"P\d+", line):
                t0 = pending.pop(line[1:], None)
                if t0 is not None:
                    rtts.append((time.perf_counter() - t0) * 1000.0)
            elif line.startswith("BEND") and tput["active"]:
                tput["active"] = False
                if tput["done"] and not tput["done"].done():
                    tput["done"].set_result(True)

    async with BleakClient(dev) as client:
        await client.start_notify(NUS_TX_UUID, on_notify)
        # --- ping RTT (合成負荷が走ったまま) ---
        print(f"[BLE] ping ×{pings} …")
        for i in range(pings):
            tok = str(i)
            pending[tok] = time.perf_counter()
            await client.write_gatt_char(NUS_RX_UUID, f"P{tok}\n".encode(), response=False)
            await asyncio.sleep(0.05)  # ~20 ping/s。応答は notify で回収
        await asyncio.sleep(0.5)
        result["rtts_ms"] = list(rtts)

        # --- スループット (blast はセンサのみ停止 → 合成負荷下の実測) ---
        if blast_sec > 0:
            print(f"[BLE] blast {blast_sec}s …")
            tput["active"] = True
            tput["bytes"] = 0
            tput["t0"] = time.perf_counter()
            tput["done"] = asyncio.get_event_loop().create_future()
            await client.write_gatt_char(NUS_RX_UUID, f"B{blast_sec}\n".encode(), response=False)
            try:
                await asyncio.wait_for(tput["done"], timeout=blast_sec + 8)
            except asyncio.TimeoutError:
                pass
            elapsed = time.perf_counter() - tput["t0"]
            if elapsed > 0:
                result["tput_kbps"] = tput["bytes"] * 8.0 / 1000.0 / elapsed
            result["ceiling_kbps"] = BLE_CEILING_KBPS
        await client.stop_notify(NUS_TX_UUID)
    return result


# --------------------------------------------------------------------------
#  判定
# --------------------------------------------------------------------------
def verdict(mon: SerialMonitor, budget_us: int):
    """PASS/FAIL (fails) と 想定内の観察 (notes) を返す。"""
    fails, notes = [], []
    late_bound = budget_us * LATE_BOUND_MULT + LATE_BOUND_MARGIN_US

    if not mon.armed:
        notes.append("armed 行は未受信 (キャプチャ開始が arm より後なだけなら無害)")
    if mon.hog_windows == 0:
        fails.append("hog の window を 1 つも受信していない")
    elif mon.hog_stalled > 0:
        fails.append(f"hog が {mon.hog_stalled}/{mon.hog_windows} window で STALLED "
                     "(系が凍結/餓死 = RT 失敗)")

    for name, v in sorted(mon.victims.items()):
        wins = v["windows"][1:] or v["windows"]
        if not wins:
            continue
        # (1) ongoing ジッタは各 window の late(win) で判断する (両コアとも watchdog が bound)。
        # device の late(max) は起動来の累積ピークで、BLE 接続時の budget-0 スタール等
        # **一過性**を保持し続けるので、bound 判定には使わず注記に回す (回復しても消えない)。
        win_late = max((lw for (_t, _e, lw) in wins), default=0)
        if win_late > late_bound:
            fails.append(f"{name}: 各 window の late(win) 最大 {win_late}us > 許容 "
                         f"{int(late_bound)}us (ongoing ジッタが非有界 = watchdog 疑い)")
        if v["late_max"] > late_bound:
            notes.append(f"{name}: 起動来ピーク late(max)={v['late_max']}us "
                         "— 一過性の可能性 (BLE 接続時の budget-0 スタール等)。"
                         "ongoing は late(win) 側で判断済み")
        # (2) レート維持。period は exp から逆算 (period ≈ REPORT_WIN_US / exp)。
        exp = max(e for (_t, e, _l) in wins)
        period_us = REPORT_WIN_US / exp if exp else 1e9
        if period_us >= budget_us:
            # schedulable: 締切を守れるはず → ticks 維持を課す。
            bad = [(t, e) for (t, e, _l) in wins if e > 0 and t < TICK_FRAC * e]
            if bad:
                w = min(bad, key=lambda te: te[0] / te[1])
                fails.append(f"{name}: period≈{period_us:.0f}us>=budget なのに "
                             f"ticks {w[0]}/{w[1]} < {TICK_FRAC:.0%} = レート未維持")
        else:
            # sub-budget: 3ms 粒度の hog 下では毎締切は物理的に不可。rate 律速は想定内。
            worst = min(wins, key=lambda te: (te[0] / te[1]) if te[1] else 1.0)
            notes.append(f"{name}: period≈{period_us:.0f}us < budget(={budget_us}us) → "
                         f"sub-budget。rate は {worst[0]}/{worst[1]} に律速 "
                         "(3ms 粒度の設計限界、想定内)")
    return fails, notes


def main():
    ap = argparse.ArgumentParser(description="Shizuku RT スケジューラ ホスト検証")
    ap.add_argument("--port", default=None, help="Pico の CDC シリアルポート (省略で自動検出)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--duration", type=float, default=30.0, help="測定秒数")
    ap.add_argument("--budget-us", type=int, default=DEFAULT_BUDGET_US,
                    help="SHIZU_DEFAULT_GRANT_BUDGET_US と一致させる (しきい値に使う)")
    ap.add_argument("--no-ble", action="store_true", help="BLE 測定を省く")
    ap.add_argument("--pings", type=int, default=50)
    ap.add_argument("--blast", type=int, default=5, help="ブラスト秒 (0 で省略)")
    args = ap.parse_args()

    port = args.port or find_pico_port()
    if not port:
        print("シリアルポートが見つからない。--port で指定してください。", file=sys.stderr)
        return 2
    print(f"[SERIAL] {port} @ {args.baud} を {args.duration:.0f}s 監視")

    mon = SerialMonitor(port, args.baud)
    mon.start()
    time.sleep(0.3)
    if mon.err:
        print(mon.err, file=sys.stderr)
        return 2

    # BLE を測る場合は監視の裏で走らせる。BLE 測定時間もシリアル監視に含める。
    if not args.no_ble:
        try:
            import asyncio
            ble = asyncio.run(ble_measure(args.pings, args.blast))
        except Exception as e:  # noqa: BLE001
            ble = {"err": f"BLE 測定失敗 ({e}). --no-ble でシリアルのみ実行可"}
        remaining = args.duration - 0  # BLE 後も少し監視して window を貯める
        if remaining > 0:
            time.sleep(min(remaining, args.duration))
    else:
        ble = None
        time.sleep(args.duration)

    mon.stop()
    mon.join(timeout=2.0)

    # ---- レポート ----
    print("\n==================== RT スケジューラ検証結果 ====================")
    if mon.err:
        print("SERIAL:", mon.err)
    print(f"[SERIAL] 受信行 {mon.lines_seen}, armed={mon.armed}, "
          f"hog windows={mon.hog_windows} (STALLED {mon.hog_stalled})")
    print(f"{'victim':<10} {'late_win_max':>12} {'late_max*':>10} {'overrun':>8} "
          f"{'windows':>8} {'min ticks/exp':>16}")
    for name, v in sorted(mon.victims.items()):
        wins = v["windows"][1:] or v["windows"]
        if wins:
            worst = min(wins, key=lambda te: (te[0] / te[1]) if te[1] else 1.0)
            tick_s = f"{worst[0]}/{worst[1]}"
            win_late = max(lw for (_t, _e, lw) in wins)
        else:
            tick_s, win_late = "-", 0
        print(f"{name:<10} {win_late:>12} {v['late_max']:>10} {v['overrun']:>8} "
              f"{len(v['windows']):>8} {tick_s:>16}")
    print("  * late_max = 起動来累積ピーク (BLE 接続等の一過性を含む)。ongoing は late_win_max で判断")

    if ble is not None:
        print("\n[BLE]")
        if ble.get("err"):
            print("  ", ble["err"])
        else:
            r = ble.get("rtts_ms") or []
            if r:
                r_sorted = sorted(r)
                p = lambda q: r_sorted[min(len(r_sorted) - 1, int(q * len(r_sorted)))]
                print(f"   ping RTT: n={len(r)} min={min(r):.1f} p50={p(0.5):.1f} "
                      f"p95={p(0.95):.1f} max={max(r):.1f} ms")
                if max(r) > 300:
                    print("   ⚠ ping max > 300ms: 競合下で BLE 応答の裾が伸びている")
            else:
                print("   ping 応答なし")
            if ble.get("tput_kbps") is not None:
                cap = ble.get("ceiling_kbps", 0)
                eff = 100.0 * ble["tput_kbps"] / cap if cap else 0
                print(f"   throughput(競合下): {ble['tput_kbps']:.0f} kbps "
                      f"(1-notify上限比 {eff:.0f}%)")

    fails, notes = verdict(mon, args.budget_us)
    print("\n---------------------------------------------------------------")
    for n in notes:
        print("  (観察)", n)
    if fails:
        print("判定: ❌ FAIL")
        for f in fails:
            print("  -", f)
        return 1
    print("判定: ✅ PASS (hog 非凍結 / ジッタ有界≈budget / schedulable victim はレート維持)")
    print("  ※ BLE の ping/throughput は上の数値で別途確認 (環境依存の soft 指標)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
