#!/usr/bin/env python3
"""安全装置を経由した**有線**のファームウェア転送 (OTW: over the wire)。\n\n★"OTA" とは呼ばない。無線ではないので名前が嘘になる。経路は\n  母艦 --USB CDC--> Seeed XIAO (安全装置) --UART0--> Pico 2 W。

  Seeed XIAO (USB CDC, /dev/cu.usbmodem1101) --UART0--> Pico 2 W

XIAO の 'ubridge <n>' コマンドで n バイトぶんだけ CDC<->UART0 を生で
中継させ (XIAO 側 src/shell.rs の run_uart_bridge)、Pico 側は
shizuku_shell.cpp が UART0 を
読んで ota オブジェクトの入力ストリームへそのバイト列を流し込む
(通常は BLE の ble_uart 経由で来るのと同じ ota.hpp のプロトコル)。

  python3 tools/otw_via_safety.py <image>              # 送るだけ (ステージング)
  python3 tools/otw_via_safety.py <image> --commit      # 送って、本体へ移す
  python3 tools/otw_via_safety.py --commit              # bazel の既定出力を使う

★地上専用。飛行中に USB が母艦に繋がっていることはない前提で、BLE の
  OTA が使えなくなったとき (誤った像を焼いた直後など) の最後の書き込み口。
★速度は 115200 baud (8N1 = 10 bit/byte) が頭打ち ≈ 11.25 kB/s。BLE の
  28.5 kB/s よりずっと遅いが、「戻ってこられる」ことの方が優先される場面
  でしか使わない。

プロトコル (ota.hpp / tools/ota_send.py と同じ):
  [0] 'X','N','O','U' or 'Z' or 'C'  マジック
  [4] uint32 le  total  イメージ長
  [8] uint32 le  crc32
  [12..] 生データ (Z のときは 4KB 単位の独立 raw deflate)
"""
import glob
import os
import struct
import sys
import time
import zlib

try:
    import serial  # pyserial
except ImportError:
    sys.exit("pyserial が要ります: pip install pyserial")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ota_send import DEFAULT_IMAGE, deflate_chunks, load_image  # noqa: E402

DEFAULT_PORT = "/dev/cu.usbmodem1101"
BAUD = 115200
# ★中継中だけ上げる速度。常用は 115200 のまま (両側の実装参照: 片方だけ焼き
#   替えた瞬間に会話できなくなるのを避けるため、速いのは転送中だけ)。
#   115200 のままだと 11.3 kB/s で、BLE OTA の 26 kB/s より遅い ——
#   **有線が無線より遅いのはおかしい**というのが上げる動機。
#   両側とも 115200〜3000000 の範囲しか受け付けない。

# ★実測で決めた値 (2026-09-02)。1Mbaud/460800/375000/345600/320000 はいずれも
#   Pico 自身が返すテキストまで文字化けする物理的なビット化けで失敗し、
#   310000 と 300000 は繰り返し CRC 一致で成功した。300000/320000 の間に
#   はっきりした崖があるのは SNR がなだらかに劣化しているというより、
#   Pico (RP2350) と XIAO (RP2040) 双方のボーレート分周器が実クロックから
#   その値をどれだけ正確に作れるかの精度限界に見える。310000 でも通った
#   実績はあるが、崖のすぐそばなので余裕を持って 300000 を既定にする。
#   個体差・配線差があるので、崖の位置はボードごとに検証し直すこと。
BRIDGE_BAUD = 300000
WRITE_CHUNK = 256  # ホスト側の write() 粒度。UART の実速度で自然に律速される。


def find_port() -> str:
    if os.path.exists(DEFAULT_PORT):
        return DEFAULT_PORT
    candidates = sorted(glob.glob("/dev/cu.usbmodem*"))
    if not candidates:
        sys.exit("XIAO の USB CDC が見つかりません (/dev/cu.usbmodem*)")
    return candidates[0]


# 「無通信になるまで」だけを終了条件にすると、相手が定期送信をしている限り
# 永久に抜けられない。★実際に踏んだ: 安全装置 XIAO を Rust 化した際に
# ハートビートが「状態変化時のみ」から **10Hz 定期** に変わり、こちらの
# idle 判定が二度と成立しなくなって OTW が起動直後に固まった。
# 相手の送信間隔は相手の都合で変わるので、**絶対上限を必ず併せ持つこと**。
DRAIN_HARD_LIMIT_S = 15.0


def drain(ser: "serial.Serial", idle_seconds: float,
          hard_limit: float = DRAIN_HARD_LIMIT_S) -> bytes:
    """データが来なくなってから idle_seconds 経つまで読み続け、画面へ流す。

    ただし hard_limit を過ぎたら、まだ流れていても打ち切る。
    """
    buf = bytearray()
    start = time.time()
    deadline = start + idle_seconds
    while time.time() < deadline:
        if time.time() - start > hard_limit:
            sys.stdout.write(
                f"\n[drain] {hard_limit:.0f}s 上限で打ち切り "
                f"(相手が定期送信を続けている)\n")
            sys.stdout.flush()
            break
        chunk = ser.read(max(1, ser.in_waiting))
        if chunk:
            buf += chunk
            sys.stdout.write(chunk.decode(errors="replace"))
            sys.stdout.flush()
            deadline = time.time() + idle_seconds
    return bytes(buf)


def drain_until(ser: "serial.Serial", marker: bytes, idle_seconds: float,
                 hard_limit: float) -> bytes:
    """marker が出るか、無通信が idle_seconds 続くか、hard_limit を過ぎるまで読む。

    drain() と違い「相手がもう次を受け付けられる状態に戻った」ことを示す
    具体的な文言を待てる場合に使う — 固定の idle_seconds だけに頼ると、
    こちらの秒読みと相手の秒読みが別クロックでずれたときに、相手がまだ
    片付いていないうちに次のコマンドを送ってしまう (2026-09-02 実測)。
    """
    buf = bytearray()
    start = time.time()
    deadline = start + idle_seconds
    while time.time() < deadline:
        if marker in buf:
            break
        if time.time() - start > hard_limit:
            sys.stdout.write(
                f"\n[drain] '{marker.decode()}' が {hard_limit:.0f}s 待っても"
                f" 出ない。見切って進む\n")
            sys.stdout.flush()
            break
        chunk = ser.read(max(1, ser.in_waiting))
        if chunk:
            buf += chunk
            sys.stdout.write(chunk.decode(errors="replace"))
            sys.stdout.flush()
            deadline = time.time() + idle_seconds
    return bytes(buf)


def send_bridge(ser: "serial.Serial", payload: bytes, label: str) -> None:
    """XIAO を ubridge モードへ入れ、payload を生で流し込む。"""
    n = len(payload)
    ser.reset_input_buffer()
    ser.write(f"ubridge {n} {BRIDGE_BAUD}\n".encode())
    ser.flush()

    buf = bytearray()
    deadline = time.time() + 3.0
    ready = False
    while time.time() < deadline:
        chunk = ser.read(max(1, ser.in_waiting))
        if chunk:
            buf += chunk
            sys.stdout.write(chunk.decode(errors="replace"))
            sys.stdout.flush()
            # ★合図は 2 通りある。UBRIDGE_READY は Pico が返すが、**XIAO が
            #   それを待って自分で食べる**ので CDC 側へは出てこない (実測)。
            #   XIAO が中継を始めたことを告げる "forwarding" が、こちらから
            #   見える唯一の合図。片方だけを待つと永久に来ない。
            if b"UBRIDGE_READY" in buf or b"forwarding" in buf:
                ready = True
                break
    if not ready:
        sys.exit(f"{label}: 中継が始まりません "
                 f"(XIAO が UBRIDGE_READY を Pico から受け取れていない = "
                 f"UART0 配線か Pico 側ファームを確認)")

    print(f"\n[{label}] sending {n} bytes over UART0 (115200 baud)...")
    t0 = time.perf_counter()
    for off in range(0, n, WRITE_CHUNK):
        ser.write(payload[off:off + WRITE_CHUNK])
    ser.flush()
    elapsed = time.perf_counter() - t0
    rate = n / elapsed / 1024 if elapsed > 0 else 0
    print(f"[{label}] host-side write done in {elapsed:.1f}s ({rate:.1f} kB/s "
          f"— UART のハードウェア律速はこの後も続く)")
    # ★2026-09-02 実測: 固定 2 秒の drain() だけで次の send_bridge() へ進む
    #   と、こちらの 2 秒と XIAO 自身のアイドル待ち (これも 2 秒だが、別々の
    #   クロックで別々の起点から数えている) がわずかにずれて、XIAO がまだ
    #   run_uart_bridge() から戻り切る前に次の 'ubridge ...' を送ってしまう
    #   ことがあった。次のコマンドを受け付けられる状態に戻ったことは、
    #   XIAO 自身が最後に必ず出す "[BRIDGE] done" で分かるので、それを見て
    #   から進む (見えなければ hard_limit で打ち切って進む — 古い XIAO
    #   ファームでこの文言が変わっていた場合に永久に待たないため)。
    drain_until(ser, b"[BRIDGE] done", idle_seconds=2.0, hard_limit=5.0)
    # ★"[BRIDGE] done" が見えた後も、次の 'ubridge ...' を受け付けられる
    #   状態に戻るまでにもう少し時間がかかる (2026-09-02 実測: 0.2s では
    #   毎回失敗、5s なら毎回通った)。正確な内訳は未特定 — XIAO 側か Pico
    #   側か、あるいは両方の後始末が絡んでいるかは分かっていない。実用上は
    #   ここで長めに待つのが最短の対策。
    time.sleep(3.0)


def main() -> int:
    args = sys.argv[1:]
    global BRIDGE_BAUD
    # --baud=<n> は中継速度の実験用。配線が長い/ノイズが乗る環境で落とせるように。
    for a in list(args):
        if a.startswith("--baud="):
            BRIDGE_BAUD = int(a.split("=", 1)[1])
            args.remove(a)
    flags = {a for a in args if a.startswith("--")}
    positional = [a for a in args if not a.startswith("--")]
    if len(positional) > 1 or not flags <= {"--commit", "--raw"}:
        print(__doc__)
        return 2

    image_path = positional[0] if positional else DEFAULT_IMAGE
    if not os.path.exists(image_path):
        if os.path.exists(image_path + ".elf"):
            image_path += ".elf"
        elif os.path.exists(image_path + ".uf2"):
            image_path += ".uf2"
        else:
            sys.exit(f"イメージがありません: {image_path}\n  bazel build //...")

    do_commit = "--commit" in flags
    raw_upload = "--raw" in flags

    image = load_image(image_path)
    crc = zlib.crc32(image) & 0xFFFFFFFF
    if raw_upload:
        body = image
        header = b"XNOU" + struct.pack("<II", len(image), crc)
    else:
        body = deflate_chunks(image)
        header = b"XNOZ" + struct.pack("<II", len(image), crc)
    print(f"image: {image_path}  {len(image)} bytes  crc32={crc:08x}")
    if not raw_upload:
        print(f"  deflate: {len(body)} bytes on the wire "
              f"({len(body) / len(image) * 100:.1f}%)")

    port = find_port()
    print(f"opening {port} @ {BAUD} baud (中継中は {BRIDGE_BAUD} baud)")
    ser = serial.Serial(port, BAUD, timeout=0.1)
    time.sleep(0.3)
    # 開始前の掃除。相手が定期送信していると idle にならないので上限は短く。
    drain(ser, 0.3, hard_limit=1.0)

    send_bridge(ser, header + body, "upload")

    if do_commit:
        commit_cmd = b"XNOC" + struct.pack("<II", len(image), crc)
        print("\ncommitting — ここで USB / 電源を抜かないこと")
        send_bridge(ser, commit_cmd, "commit")

        print("waiting for Pico to reboot...")
        time.sleep(3.0)
        ser.reset_input_buffer()
        # ★小文字は XIAO の 1 キーショートカット即時実行に化ける ('s'=status,
        #   'e'=SW_INHIBIT トグル)。2026-09-01 の修正で複数文字コマンドは
        #   大文字必須になったのに、ここだけ直し忘れていた。実際に踏んだ:
        #   'send version' の 's' と 'e' が即時発火し、SW_INHIBIT が勝手に
        #   ON になった (安全フラグが検証コマンドの副作用で変わった)。
        ser.write(b"SEND version\n")
        text = drain(ser, 2.0).decode(errors="replace")
        if f"crc={crc:08x}" in text:
            print("RESULT: commit verified (running new image)")
        else:
            print("RESULT: could not verify from here — XIAO 経由の 'send "
                  "version' 応答に一致する crc が見えなかった。Pico の USB "
                  "CDC (もし繋げれば) か次回 BLE 接続で確認すること。")

    ser.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
