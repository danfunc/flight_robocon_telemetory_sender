#!/usr/bin/env python3
"""焼いて・起動を見届けて・要点だけ出す。

    tools/flash_watch.py <build_dir> [-t 秒] [-g 追加パターン] [--no-flash]

このリポジトリで実機検証をするたびに手書きしていた手順を 1 本にまとめたもの。
手書きだと以下を毎回踏むので、それらを全部ここで面倒を見る:

  1. **picotool は書けなくても終了コード 0 を返す**。固まったファームはベンダ
     リセットを処理できず「Waiting for device to reboot...」で終わるのに rc=0。
     → 完了行 "can be downloaded in absolute space" の有無で判定する
     (これを見落として切り分け結論を丸ごと間違えたことがある)
  2. **シリアルポートが他プロセスに掴まれる** (VSCode の serial monitor 拡張など)。
     → 開けるまでリトライする
  3. **再列挙でポート名が変わる** (usbmodem1101 → usbmodem101)。
     → 毎回 glob で取り直す
  4. 異常行 (panic / trapped / unknown / rejected / NO_STACK …) は黙って流れると
     気づかない。→ 常に拾って最後に件数を出す
"""
import glob
import os
import subprocess
import sys
import time

PICOTOOL = os.path.expanduser("~/.pico-sdk/picotool/2.2.0-a4/picotool/picotool")
SDK = os.path.expanduser("~/.pico-sdk/sdk/2.2.0")
TOOLCHAIN = os.path.expanduser("~/.pico-sdk/toolchain/14_2_Rel1")

# 常に拾う異常パターン。sink が詰まると panic すら出ないので、出たら必ず見る。
TROUBLE = ("panic", "PANIC", "trapped", "HardFault", "rejected", "parked",
           "NO_STACK", "unknown", "OBJAPI", "already initialized")
# 常に拾う健全性の指標。
HEALTH = ("[CPU]", "[BEACON]")


def sh(cmd):
    env = dict(os.environ,
               PICO_SDK_PATH=SDK, PICO_TOOLCHAIN_PATH=TOOLCHAIN,
               PATH=f"{TOOLCHAIN}/bin:" + os.environ.get("PATH", ""))
    return subprocess.run(cmd, shell=True, capture_output=True, text=True, env=env)


def flash(uf2):
    r = sh(f'"{PICOTOOL}" load -f "{uf2}"')
    out = r.stdout + r.stderr
    # ★rc ではなくこの行で判定する (rc=0 でも書けていないことがある)
    if "can be downloaded in absolute space" not in out:
        print("FLASH FAILED (書き込みに到達していない)")
        for l in out.splitlines():
            if "Loading into Flash" not in l and l.strip():
                print("   ", l)
        return False
    print(f"flashed: {uf2}")
    return True


def watch(seconds, extra):
    port = None
    deadline = time.time() + 40
    ser = None
    while time.time() < deadline and ser is None:
        ports = sorted(glob.glob("/dev/cu.usbmodem*"))  # 再列挙で名前が変わる
        if ports:
            try:
                import serial
                ser = serial.Serial(ports[0], 115200, timeout=0.3)
                port = ports[0]
            except Exception:
                ser = None  # 他プロセスが掴んでいる → 空くまで待つ
        time.sleep(0.3)
    if ser is None:
        # ★誰が掴んでいるかまで出す。ここを「出力が無い = ファームがハングした」と
        #   誤読すると切り分けを丸ごと間違える (実際に一度やった)。
        #   macOS では /dev/tty.* を開かれると対の /dev/cu.* が Resource busy になる
        #   ので、lsof は tty 側も含めて探すこと。
        print("!!! PORT BUSY / NOT FOUND !!!  (ファームのハングではない)")
        holder = subprocess.run("lsof 2>/dev/null | grep -i usbmodem",
                                shell=True, capture_output=True, text=True)
        if holder.stdout.strip():
            print("掴んでいるプロセス:")
            for l in holder.stdout.strip().splitlines():
                print("   ", l)
            print("→ VSCode の serial monitor 等を閉じて再実行")
        else:
            print("→ ポートが見つからない (デバイス未接続 / 再列挙待ち)")
        return 2
    print(f"watching {port} for {seconds}s ...")
    buf = b""
    t0 = time.time()
    while time.time() - t0 < seconds:
        try:
            d = ser.read(8192)
        except Exception as e:
            print("port lost:", e)
            break
        if d:
            buf += d
    lines = buf.decode("utf-8", "replace").splitlines()
    pats = tuple(extra) + TROUBLE
    hit = [l for l in lines if any(p in l for p in pats)]
    print(f"--- {len(buf)} bytes / {len(lines)} lines ---")
    for l in hit[-40:]:
        print(l)
    health = [l for l in lines if any(p in l for p in HEALTH)]
    for l in health[-4:]:
        print(l)
    bad = [l for l in lines if any(p in l for p in TROUBLE)]
    print(f"=== 異常行: {len(bad)} 件 ===")
    return 1 if bad else 0


def main():
    a = sys.argv[1:]
    if not a:
        print(__doc__)
        return 2
    build = a[0].rstrip("/")
    secs, extra, do_flash = 22, [], True
    i = 1
    while i < len(a):
        if a[i] == "-t":
            i += 1; secs = int(a[i])
        elif a[i] == "-g":
            i += 1; extra.append(a[i])
        elif a[i] == "--no-flash":
            do_flash = False
        i += 1
    if do_flash and not flash(f"{build}/main.uf2"):
        return 2
    return watch(secs, extra)


if __name__ == "__main__":
    sys.exit(main())
