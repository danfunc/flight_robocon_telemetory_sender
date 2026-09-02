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
  python3 tools/otw_via_safety.py --legacy --commit     # 旧 XNOZ 経路 (回復用)

★既定は XNOR (チャンク単位の再送)。送り手は最後まで一括で送り切り、受け手が
  全チャンクを受け終えた時点で「失敗した seq の一覧」を返し、送り手はその分
  だけ再送する (HTTP の range 再取得と同じ発想)。一覧が空になるか上限回数まで
  繰り返す。--legacy は再送の効かない従来経路で、**新形式が壊れたときの
  回復用**に残してある。

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
from ota_send import (DEFAULT_IMAGE, MAX_ROUNDS, chunked_body,  # noqa: E402
                      deflate_chunk_list, deflate_chunks, load_image,
                      parse_need)

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
#
# ★★2026-09-02 追試 (チャンク再送を入れた後、像 445608B / 109 チャンク、
#   3 回ずつ交互に実行・1 回ごとに冷却)。実効速度 = 像 ÷ キャンペーン全体:
#
#     baud    1巡目の欠損   ラウンド   実効速度 (3 回)
#     300000     0/109         1       31.9 / 32.0 / (中継開始失敗 1)
#     310000     0/109         1       32.8 / 32.8 / 32.8
#     345600     4/109         2       24.5 / 24.5 / 24.5
#     375000     8/109         2       25.1 / (制御喪失) / 25.1
#     460800    24/109         3       14.3
#     921600      —            —       制御を保てず
#
#   ★**「多少化けても速い方が勝つ」帯は無かった。** 崖が急峻 (0 → 4 → 24 個)
#     で、再送 1 ラウンドの固定費が約 4.3 秒あるため、崖を越えた瞬間に必ず
#     負ける。チャンク再送は**速度を上げるためではなく、崖の向こうで失敗を
#     全損にしないため**に効いている、と読むのが正しい。
#   ★310000 が実測では最速 (+2.5%) だが、**既定は 300000 のままにする**。
#     320000 で崖なので 310000 の余裕は 3%、300000 なら 6%。OTW は「戻って
#     これる」ことが存在意義の経路で、2.5% の速度に余裕を半分渡すのは
#     割に合わない。崖の位置は配線・個体・温度で動く。
#     ★ただし再送が入った今、崖の向こうへ踏み込んでも全損ではなく再送に
#       なるので、次に触る人が 310000 を選ぶ判断はあり得る。
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


# ★2 回連続でブリッジを張ると「UBRIDGE_READY が来ません」で毎回失敗する
#   (2026-09-02 実測: 5 秒待てば毎回通り、0.2 秒では毎回失敗。XIAO 側か
#   Pico 側かは切り分けられていない)。実用対処として間隔を空ける。
#   ★★この待ちは**次のブリッジを張る直前**に払う。以前は各ブリッジの
#     終わりで払っていたが、チャンク再送では最後のブリッジのあとに来るのは
#     行コマンド (OTANEED) であって次のブリッジではないので、そこで 3 秒
#     待つのは丸損だった。必要なところでだけ払う。
BRIDGE_GAP_S = 3.0
_last_bridge_end = 0.0


def wait_after_bridge(label: str) -> None:
    """直前のブリッジの後始末が終わるまで空ける。

    ★★次のブリッジだけでなく**行コマンドの前にも要る** (2026-09-02 実測:
      待ちを外したら `[BRIDGE] done` の直後に送った OTANEED が消えた)。
      つまりこれは「ubridge を 2 回連続で張れない」問題ではなく、
      **ブリッジ直後の Pico は何も受け付けない**という問題だった。
      機序: end_uart_bridge() は ACK を送ってから UBRIDGE_LOST/DONE を出し、
      最後に baud を戻して RX を捨てる。XIAO は ACK を見た時点で自分の baud を
      戻すので、以降の行は速度が食い違って化け、その後の RX 破棄でこちらの
      コマンドまで消える。ACK をブリッジ速度で送る**最後の 1 バイト**に
      すれば縮められるはず (firmware 側で対処中)。
    """
    global _last_bridge_end
    gap = BRIDGE_GAP_S - (time.time() - _last_bridge_end)
    if gap > 0:
        print(f"[{label}] 前のブリッジの後始末を {gap:.1f}s 待つ")
        time.sleep(gap)


def send_bridge(ser: "serial.Serial", payload: bytes, label: str,
                tag: str = "upload") -> None:
    """XIAO を ubridge モードへ入れ、payload を生で流し込む。

    tag は XIAO の**状態 LED の色だけ**を決める第 3 引数 (upload/resend/commit)。
    ★古い XIAO は第 3 引数を読まないので黙って無視される (後方互換)。
      Pico へ中継される UBRIDGE には載らないので、Pico 側は無関係。
    """
    global _last_bridge_end
    wait_after_bridge(label)
    n = len(payload)
    ser.reset_input_buffer()
    ser.write(f"ubridge {n} {BRIDGE_BAUD} {tag}\n".encode())
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
    #   状態に戻るまでにもう少し時間がかかる。正確な内訳は未特定 — XIAO 側か
    #   Pico 側か、あるいは両方の後始末が絡んでいるかは分かっていない。
    #   待ちは次の send_bridge() の頭で払う (BRIDGE_GAP_S)。
    _last_bridge_end = time.time()


def complete_flag(text: str) -> bool:
    """NEEDEND まで見えたか。"""
    return "NEEDEND" in text


def reset_ota(ser: "serial.Serial") -> None:
    """走りかけの転送を捨てさせ、必ず待ち受けから始める。"""
    ser.reset_input_buffer()
    ser.write(b"SEND OTARESET\n")
    ser.flush()
    drain(ser, 0.5, hard_limit=3.0)


def query_missing(ser: "serial.Serial"):
    """まだ届いていないチャンクを Pico に聞く。戻り値 (n, seqs, complete)。

    ★★**中継の外**、常用の 115200 で聞く。同じ問い合わせは帯域内 (seq=0xFFFF
      のチャンクヘッダ) でもできるが、攻めた baud では**返事そのものが化ける** —
      そうなると「どれを再送すべきか」が分からず、再送機構ごと成立しない。
      制御は必ず化けない速度で通す、というのがここを分けている理由。
    ★小文字を送らないこと。XIAO の 1 キーショートカット ('s'=status,
      'e'=SW_INHIBIT トグル) が即時発火し、**安全フラグが検証コマンドの
      副作用で変わる**事故を実際に起こしている (2026-09-02)。
    """
    # ★★何度か聞き直すこと。**Pico がまだ中継モードから抜けていない**ことが
    #   ある — 化ける速度では受け取りバイト数が要求値に届かず、Pico は
    #   無通信 5 秒のタイムアウトでしか中継を抜けない。その間に送った
    #   OTANEED は像の一部として食われて消える。
    #   2026-09-02 実機: 460800 で「制御が壊れた」と判定したが、後から手で
    #   聞き直したら NEED n=24 (ok=85 bad=11) と正常に返ってきた。**制御が
    #   壊れていたのではなく、こちらが早すぎた**。ここを一発勝負にすると
    #   「速い baud は使えない」という誤った結論を出してしまう。
    n = seqs = complete = None
    for attempt in range(3):
        wait_after_bridge("OTANEED")
        ser.reset_input_buffer()
        ser.write(b"SEND OTANEED\n")
        ser.flush()
        # NEEDEND は成否によらず必ず出る (device 側 report_missing)。それを
        # 待てば「返事が全部届いた」が一つの合図で判定できる。
        text = drain_until(ser, b"NEEDEND", idle_seconds=2.0,
                           hard_limit=30.0).decode(errors="replace")
        # ★★"NEEDIDLE" = device 側に転送が走っていない。**成功ではない**。
        #   以前 device はこれを "NEED n=0 (チャンク転送が走っていない)" と
        #   返しており、n=0 = 全チャンク受領として読まれて**失敗が成功として
        #   報告されていた** (2026-09-02 実機)。しかも括弧の中は日本語なので
        #   XIAO の行組み立て (ASCII のみ) を通ると消え、"NEED n=0 ()" という
        #   完全に成功に見える文字列になっていた。
        if "NEEDIDLE" in text:
            print("  ★device に転送が走っていない (NEEDIDLE)。転送が途中で"
                  "畳まれた可能性が高い — 最初からやり直すこと")
            return None, [], complete_flag(text)
        n, seqs, complete = parse_need(text)
        if complete and n is not None:
            return n, seqs, complete
        if attempt < 2:
            print(f"  (OTANEED に返事が無い — Pico がまだ中継から抜けて"
                  f"いない可能性。{attempt + 2} 回目を試す)")
            time.sleep(6.0)  # 中継の無通信タイムアウト (5s) より長く
    return n, seqs, complete


def send_chunked(ser: "serial.Serial", header: bytes, chunks: list) -> bool:
    """一括で送り切り、足りない seq だけを繰り返し再送する。

    ★窓 (パイプライン) 方式ではなく **HTTP の range 再取得と同じ発想**。
      送り手は途中で ACK を待たず最後まで流し切り、受け手が全部受け終えた
      時点で「失敗した seq の一覧」を返す。窓幅・順序・タイムアウト再送の
      状態機械が要らず、ota 側も「失敗した seq を覚えておくだけ」で済む。
    ★2 ラウンド目以降は**ファイルヘッダを送らない**。XNOR ヘッダは受領
      ビットマップを消すので、送り直すと全チャンクが「未受領」に戻る。
      device はラウンドの合間 CSEEK のまま待っている。
    """
    send_bridge(ser, header + chunked_body(chunks), "upload")

    for rnd in range(1, MAX_ROUNDS + 1):
        n, seqs, complete = query_missing(ser)
        if not complete:
            print(f"\n[round {rnd}] NEED の返事が最後まで届かない "
                  f"(NEEDEND が見えない)。一覧が途中で切れている可能性が"
                  f"あるので、ここで止める — 欠けたまま commit しないこと。")
            return False
        if n is None:
            print(f"\n[round {rnd}] NEED 行が読めない。Pico のファームが "
                  f"チャンク再送 (XNOR) に対応していない可能性がある "
                  f"(--legacy で従来の XNOZ 経路に落とせる)")
            return False
        if n == 0:
            print(f"[round {rnd}] 全チャンク受領。device が読み返し CRC まで"
                  f"確認済み")
            return True
        if rnd == MAX_ROUNDS:
            print(f"\n{n} チャンクが {MAX_ROUNDS} ラウンドでも埋まらない。"
                  f"配線かこの baud が無理筋 — --baud= を下げて再試行すること")
            return False
        print(f"[round {rnd}] {n} チャンクが未達 → 再送する "
              f"({len(seqs)} 個を詰め直し)")
        send_bridge(ser, chunked_body(chunks, seqs), f"resend{rnd}",
                    tag="resend")
    return False


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
    if len(positional) > 1 or not flags <= {"--commit", "--raw", "--legacy"}:
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
    legacy = "--legacy" in flags

    image = load_image(image_path)
    crc = zlib.crc32(image) & 0xFFFFFFFF
    chunks = None
    if raw_upload:
        body = image
        header = b"XNOU" + struct.pack("<II", len(image), crc)
    elif legacy:
        # ★従来の XNOZ 経路。**回復のために残してある** — 新形式 (XNOR) に
        #   バグがあっても、この経路で板へ焼き直せる。チャンク再送は効かない。
        body = deflate_chunks(image)
        header = b"XNOZ" + struct.pack("<II", len(image), crc)
    else:
        chunks = deflate_chunk_list(image)
        body = chunked_body(chunks)
        header = b"XNOR" + struct.pack("<II", len(image), crc)
    print(f"image: {image_path}  {len(image)} bytes  crc32={crc:08x}")
    if not raw_upload:
        mode = "XNOZ (legacy)" if legacy else f"XNOR ({len(chunks)} chunks)"
        print(f"  deflate: {len(body)} bytes on the wire "
              f"({len(body) / len(image) * 100:.1f}%)  {mode}")

    port = find_port()
    print(f"opening {port} @ {BAUD} baud (中継中は {BRIDGE_BAUD} baud)")
    ser = serial.Serial(port, BAUD, timeout=0.1)
    time.sleep(0.3)
    # 開始前の掃除。相手が定期送信していると idle にならないので上限は短く。
    drain(ser, 0.3, hard_limit=1.0)

    # ★★どの経路でも、送る前に走りかけの転送を捨てさせる。
    #   **--legacy にも要る**。前の campaign を諦めた直後の ota は CSEEK に
    #   居座っており、そこへ XNOZ を流すと**ストリーム全体がチャンク探索の
    #   ゴミとして食われる** (2026-09-02 実機で踏んだ: done: も ACK も出ず、
    #   次の commit 用ブリッジが UBRIDGE_READY を返せなくなった)。
    #   最初これを chunked 経路にだけ入れていたのが誤り — **回復用の経路こそ
    #   前提を仮定してはいけない**。待ち受けから始まっていれば何も起きない。
    reset_ota(ser)

    if chunks is not None:
        if not send_chunked(ser, header, chunks):
            print("RESULT: staging failed (本体は無傷)")
            ser.close()
            return 1
    else:
        send_bridge(ser, header + body, "upload")

    if do_commit:
        commit_cmd = b"XNOC" + struct.pack("<II", len(image), crc)
        print("\ncommitting — ここで USB / 電源を抜かないこと")
        send_bridge(ser, commit_cmd, "commit", tag="commit")

        # ★★何度か聞き直すこと。commit は消去 + 書き込み + 再起動なので、
        #   固定の 3 秒では**間に合わないほうが普通**だった (実測: 1 回きりだと
        #   ほぼ毎回「確認できず」になる)。
        # ★小文字は XIAO の 1 キーショートカット即時実行に化ける ('s'=status,
        #   'e'=SW_INHIBIT トグル)。行頭が大文字なら以降は行バッファに入るので
        #   'SEND ...' は安全だが、**行頭を必ず大文字にする**規律は崩さない。
        print("waiting for Pico to reboot...")
        verified = False
        deadline = time.time() + 25.0
        while time.time() < deadline:
            time.sleep(2.0)
            ser.reset_input_buffer()
            ser.write(b"SEND VER\n")
            text = drain(ser, 2.0).decode(errors="replace")
            if f"crc={crc:08x}" in text:
                verified = True
                break
        if not verified:
            print("RESULT: could not verify from here — XIAO 経由の 'SEND VER' "
                  "応答に一致する crc が見えなかった。Pico の USB CDC "
                  "(もし繋げれば) か次回 BLE 接続で確認すること。")
            # ★★**成功として返さない**。ここで 0 を返していたため、
            #   flash.sh が「検証できていない commit」を「完了」と報告して
            #   いた。焼けたかどうかを言えないなら、言えないと返すこと —
            #   上位が次の手 (OTA) へ進めるかどうかの判断材料になる。
            ser.close()
            return 1
        print(f"RESULT: commit verified (running new image, crc={crc:08x})")

    ser.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
