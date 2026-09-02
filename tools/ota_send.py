#!/usr/bin/env python3
"""ファームウェアを BLE でデバイスへ送り、ステージングへ置かせる / 本体へ移す。

  python3 tools/ota_send.py --commit                   # ← これで足りる
  python3 tools/ota_send.py <image> --commit           # 送って、本体へ移す
  python3 tools/ota_send.py <image>                    # 送るだけ (ステージング)
  python3 tools/ota_send.py <image> --commit-only      # 送らず、置いてある像を移す

`<image>` は **.elf / .uf2 / .bin のどれでもよい**。省略すると bazel の既定の
出力 (bazel-bin/firmware_bazel/xno_bringup) を使う。デバイスへ流すのは常に
**生イメージ**なので、.elf は objcopy で、.uf2 はこのスクリプト内で bin に
直してから送る。★ワンライナーで済ませたかったのがこの分岐の理由 ——
「どの成果物を渡せばいいんだっけ」を毎回思い出さなくていいようにしてある。

  bazel build //firmware_bazel:xno_bringup && python3 tools/ota_send.py --commit

プロトコル (ota.hpp と一致させること):
  [0] 'X','N','O','U'   マジック
  [4] uint32 le  total  イメージ長
  [8] uint32 le  crc32  イメージ全体の CRC32 (zlib と同じ)
  [12..] 生データ

commit コマンドは同じ characteristic 上の 12 バイト:
  [0] 'X','N','O','C' / [4] total / [8] crc32
★--commit-only が成り立つのは、デバイスが RAM 上の受信状態ではなく
  **flash から読み直して** CRC を確かめるため。転送と commit を別の接続で
  やってよい (転送のあと切れてしまっても、置いた像はそこに残っている)。

進捗と結果はデバイス側が NUS の notify に行で出すので、それを表示する。

★commit は戻ってこない操作。デバイスは「commit: ...」の行を出したあと
  自分がいる領域を消して書き、書き終わると再起動する。その間 BLE は切れる。
  **コピー中に電源を落とすと BOOTSEL で焼き直すしかない** (ota.hpp 冒頭)。
"""
import asyncio
import glob
import os
import shutil
import struct
import subprocess
import sys
import tempfile
import time
import zlib

sys.path.insert(0, __file__.rsplit("/", 1)[0])
# ★★BLE の依存を import だけで要求しない。この module にはワイヤ形式の
#   組み立てや NEED のパースといった**純粋な処理**も入っていて、試験はそこ
#   だけを見る。import で bleak を要求すると、ハードウェアも外部パッケージも
#   無い環境 (Bazel の hermetic な interpreter が典型) で**試験が回らなく
#   なる**。実際に BLE を使う関数へ入った時点で落ちれば十分。
try:
    from bleak import BleakClient, BleakScanner  # noqa: E402,F401
    from shizuku_link import DEVICE_NAME, NUS_TX_UUID, find_device  # noqa: E402,F401
    _BLE_IMPORT_ERROR = None
except ImportError as _err:  # pragma: no cover - 環境依存
    BleakClient = BleakScanner = None
    DEVICE_NAME = NUS_TX_UUID = None
    find_device = None
    _BLE_IMPORT_ERROR = _err


def _require_ble() -> None:
    """BLE 経路へ入る直前に呼ぶ。落ちるなら**理由が分かる形で**落とす。"""
    if _BLE_IMPORT_ERROR is not None:
        raise SystemExit(
            f"BLE の依存が入っていません ({_BLE_IMPORT_ERROR})。"
            "  pip install bleak pyserial")

OTA_RX_UUID = "6e402002-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
# ★1 回の write に載せられるのは ATT MTU - 3。MTU は 247 に頭打ちしてある
#   (btstack_config.h の HCI_ACL_PAYLOAD_SIZE=(247+4))。
CHUNK = 244

# ---- 圧縮 -----------------------------------------------------------------
# ★リンクは 28.5 kB/s で頭打ち (macOS/CoreBluetooth 側の天井。書き込みゲートを
#   外してもオーバーシュートさせても変わらないことを実測済み)。上げられない
#   以上、**送るバイトを減らす**しかない。ARM の像は deflate で 67% に落ちる。
# ★1 チャンク = 4096 バイトの**独立した** raw deflate ストリーム。独立にすると
#   圧縮率は 63.2% → 66.9% に落ちるが (転送時間で 0.5 秒)、デバイス側は
#   「入力をまたいで再開できる展開器」が要らなくなり、展開先も既存のセクタ
#   バッファで足りる。その 0.5 秒で実装の難所を買っている (inflate.hpp 冒頭)。
ZCHUNK = 4096
ZWBITS = -12  # 窓 4KB = 1 チャンクぶん。これ以上大きくしても意味がない
ZCHUNK_MAX = ZCHUNK + 256  # デバイス側 ota.cpp の ZCHUNK_MAX と一致させること


def _deflate_one(raw: bytes) -> bytes:
    """4KB 以下の 1 ブロックを独立した raw deflate にする。"""
    c = zlib.compressobj(9, zlib.DEFLATED, ZWBITS)
    comp = c.compress(raw) + c.flush()
    if len(comp) >= len(raw) + 5:
        # ★縮まないチャンクは自分で「無圧縮ブロック」に組む。deflate の
        #   枠が付いて 4KB を超え、デバイスの上限に当たるのを防ぐ。
        #   0x01 = 最終ブロック・無圧縮、続けて LEN と ~LEN。
        #   ★ここは元々**生の 0x01 バイトがソースへ直接埋まっていた**。値は
        #     同じだが grep や差分で消えて見えるのでエスケープ表記へ直した。
        comp = b"\x01" + struct.pack("<HH", len(raw), len(raw) ^ 0xFFFF) + raw
    if len(comp) > ZCHUNK_MAX:
        raise SystemExit(f"chunk is {len(comp)}B > {ZCHUNK_MAX}B")
    return comp


def deflate_chunk_list(image: bytes) -> list:
    """像を 4KB ごとに独立圧縮したペイロードの一覧にする (seq = 添字)。"""
    return [_deflate_one(image[at : at + ZCHUNK])
            for at in range(0, len(image), ZCHUNK)]


def deflate_chunks(image: bytes) -> bytes:
    """4KB ごとに独立した raw deflate にして [u16 len][data] で並べる。

    ★XNOZ (旧形式) 用。チャンク再送には使えない — 境界が len にしか無いので、
      len の 1 ビットが化けると以降のフレーム境界が全部ずれ、「化けた seq
      だけ再送」が「最初の 1 個が化けたら以降全部」へ退化する。新しい経路は
      chunked_body() を使うこと。**この関数を消さないのは回復経路のため**:
      新形式にバグがあっても、古い形式でこの板へ焼き直せる。
    """
    out = bytearray()
    for comp in deflate_chunk_list(image):
        out += struct.pack("<H", len(comp)) + comp
    return bytes(out)


# ---- XNOR: チャンク単位の再送 ---------------------------------------------
# ★ワイヤ形式 (device 側 ota.cpp と一致させること):
#     [u32 'XNCK'][u16 seq][u16 len][u32 crc32(payload)][u16 rsv][u16 crc16(先頭14B)]
#   受け手は len を信用する前に crc16 でヘッダを検め、壊れていれば 1 バイト
#   ずつずらして magic を探し直す。この「自己同期できるヘッダ」があって初めて
#   「化けた seq だけ再送」が成立する。上乗せは 16B/4096B = 0.39%。
# ★seq == QUERY_SEQ かつ len == 0 は「今どれが足りないか教えろ」の問い合わせ。
#   チャンクと同じ枠に載せてあるので、探索も crc16 の保護もそのまま効く。
CHUNK_MAGIC = b"XNCK"
QUERY_SEQ = 0xFFFF
RESET_SEQ = 0xFFFE
MAX_ROUNDS = 5


def _crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = (((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000
                   else (crc << 1) & 0xFFFF)
    return crc


def chunk_frame(seq: int, payload: bytes) -> bytes:
    head = CHUNK_MAGIC + struct.pack(
        "<HHIH", seq, len(payload), zlib.crc32(payload) & 0xFFFFFFFF, 0)
    return head + struct.pack("<H", _crc16_ccitt(head)) + payload


def reset_frame() -> bytes:
    """走りかけの転送を捨てさせる。

    ★★諦めた転送のあと device は最大 2 分 CSEEK に居座る (ラウンドの合間を
      守るための長いタイムアウト)。その間に次の転送を始めると **XNOR の
      ファイルヘッダがチャンクデータとして食われて**先へ進まない。転送の頭で
      必ず撃つこと。
    ★device はこのフレームの**残りを捨てる**ので、**書き込み単位の最後に
      置くこと** (BLE なら 16B を単独で write する)。UART の中継のように
      バイト列が連続する経路では代わりにシェルの OTARESET を使う。
    """
    return chunk_frame(RESET_SEQ, b"")


def query_frame() -> bytes:
    """帯域内の「足りない seq を教えろ」。

    ★OTW では使わない — 攻めた baud だと返事そのものが化けて再送機構ごと
      成立しないので、あちらは中継の外から常用 115200 でシェルの OTANEED を
      叩く。BLE は baud の問題が無いのでこれでよい。
    """
    return chunk_frame(QUERY_SEQ, b"")


def chunked_body(chunks: list, seqs=None) -> bytes:
    """指定した seq のチャンクだけを並べる (省略時は全部)。

    ★チャンクが自分の seq を持っているので**順不同でよい**。送り手側に
      窓幅・順序・タイムアウト再送の状態機械が要らないのがこの方式の要点。
    """
    if seqs is None:
        seqs = range(len(chunks))
    return b"".join(chunk_frame(s, chunks[s]) for s in seqs)


def parse_need(text: str):
    """device の NEED 応答を読む。戻り値 (n, seqs, complete)。

    complete=False は「NEEDEND まで見えなかった」= 返事が化けたか届いて
    いない。★このとき n も seqs も信用してはいけない — 一覧が途中で切れて
    いるのに信じると、欠けたまま「もう要らない」と誤解して commit へ進む。
    """
    n = None
    seqs = []
    complete = False
    for line in text.splitlines():
        line = line.strip()
        # ★★XIAO は Pico から中継した行に "[UART-RX] " を前置して CDC へ出す。
        #   素の startswith では一致せず、**NEED が全部届いているのに「届いて
        #   いない」と誤判定する** (実機で踏んだ: ok=109 bad=0 と出ているのに
        #   staging failed で止まった)。先頭の "[...] " を落としてから見る。
        #   BLE 経路には前置が付かないので、これ一つで両方を扱える。
        if line.startswith("[") and "] " in line:
            line = line.split("] ", 1)[1]
        # ★★行頭にゴミが付くことがある (中継の切り替わりで残ったバイトが
        #   次の行に食い込む。実機で "dsNEED n=2 ..." を観測)。startswith で
        #   見ていると**中身は全部届いているのに読めない**と誤判定するので、
        #   目印は行のどこにあってもよいことにする。この 3 語は他の出力に
        #   現れないので、探索にしても誤検出しない。
        at = line.find("NEED n=")
        if at >= 0:
            try:
                n = int(line[at + 7:].split()[0])
            except (ValueError, IndexError):
                pass
        elif "NEEDSEQ" in line:
            # ★範囲表記 ("40-59") を受ける。ビット化けは連続したチャンクを
            #   まとめて落とすので、範囲にすると行が劇的に短くなる —
            #   これは見た目の話ではなく、**XIAO の UART RX が FIFO 32B の
            #   ポーリングで 115200 では 2.8ms で溢れる**ため。長い行を続けて
            #   流すと途中が落ちて一覧が欠け、送り手は足りない seq を知らない
            #   まま再送して永久に収束しない (2026-09-02 実機で踏んだ)。
            for tok in line[line.find("NEEDSEQ") + 7:].replace(",", " ").split():
                try:
                    if "-" in tok:
                        a, b = tok.split("-", 1)
                        seqs.extend(range(int(a), int(b) + 1))
                    else:
                        seqs.append(int(tok))
                except ValueError:
                    pass  # 化けたトークンは捨てる (complete 判定で拾う)
        elif "NEEDEND" in line:
            complete = True
    return n, seqs, complete

# ★write without response のバックプレッシャ。bleak は response=False を
#   CoreBluetooth へ**投げっぱなし**にする (backends/corebluetooth の
#   write_characteristic は非応答書き込みでは何も待たない)。そのままだと
#   396KB が一瞬でキューへ積まれ、デバイスの受信リングが溢れて切断する
#   (2026-08-24 の実測: 「0.0 秒で 396KB 送信」→ 切断)。
#   ★流量制御は CoreBluetooth 自身が持っている — `canSendWriteWithoutResponse`
#     が「まだ積める」を教えてくれる。これを見て詰まったら待つ。これは
#     リンク層まで効く本物の背圧で、デバイスが遅れれば相手のコントローラが
#     クレジットを止め、こちらの canSend が false になる。
try:
    from CoreBluetooth import CBPeripheral  # noqa: F401  (macOS のみ)
    HAVE_COREBLUETOOTH = True
except ImportError:
    HAVE_COREBLUETOOTH = False

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_IMAGE = os.path.join(ROOT, "bazel-bin/firmware_bazel/xno_bringup")

# UF2 (https://github.com/microsoft/uf2) — 512B 固定ブロック。
UF2_MAGIC0, UF2_MAGIC1, UF2_MAGIC_END = 0x0A324655, 0x9E5D5157, 0x0AB16F30
UF2_FLAG_NOT_MAIN_FLASH = 0x00000001


def _objcopy() -> str:
    """arm-none-eabi-objcopy を探す。PATH → pico-sdk の同梱ツールチェーン。"""
    found = shutil.which("arm-none-eabi-objcopy")
    if found:
        return found
    candidates = sorted(glob.glob(os.path.expanduser(
        "~/.pico-sdk/toolchain/*/bin/arm-none-eabi-objcopy")))
    if not candidates:
        sys.exit("arm-none-eabi-objcopy が見つかりません (.elf を渡すには要る)")
    return candidates[-1]


def _uf2_to_bin(blob: bytes) -> bytes:
    """UF2 を生イメージへ。★穴は 0xFF で埋める (消去後と同じ値)。

    uf2 はブロックごとに書き込み先アドレスを持つので、順番も連続性も保証が
    ない。並べ替えて最小アドレスを基点に敷き直す。
    """
    chunks = []
    for at in range(0, len(blob), 512):
        block = blob[at : at + 512]
        if len(block) < 512:
            break
        m0, m1, flags, addr, size = struct.unpack("<IIIII", block[:20])
        (mend,) = struct.unpack("<I", block[508:512])
        if m0 != UF2_MAGIC0 or m1 != UF2_MAGIC1 or mend != UF2_MAGIC_END:
            sys.exit(f"UF2 ブロックが壊れています (offset {at})")
        if flags & UF2_FLAG_NOT_MAIN_FLASH:
            continue  # flash 以外に載るブロック (メタデータ等) は送らない
        chunks.append((addr, block[32 : 32 + size]))
    if not chunks:
        sys.exit("UF2 に flash 向けのブロックがありません")
    chunks.sort()
    base = chunks[0][0]
    end = max(addr + len(data) for addr, data in chunks)
    image = bytearray(b"\xFF" * (end - base))
    for addr, data in chunks:
        image[addr - base : addr - base + len(data)] = data
    return bytes(image)


def load_image(path: str) -> bytes:
    """.elf / .uf2 / .bin のどれを渡されても生イメージにして返す。

    拡張子ではなく**中身**で見分ける (bazel の出力は拡張子なしの ELF なので、
    拡張子で判断すると既定パスが通らない)。
    """
    blob = open(path, "rb").read()
    if blob[:4] == b"\x7fELF":
        # ★出力先はテンポラリ。bazel-bin は**読み取り専用**なので、渡された
        #   ELF の隣には置けない。
        out = os.path.join(tempfile.gettempdir(),
                           os.path.basename(path) + ".ota.bin")
        subprocess.run([_objcopy(), "-O", "binary", path, out], check=True)
        print(f"  objcopy: {path} -> {out}")
        return open(out, "rb").read()
    if blob[:4] == struct.pack("<I", UF2_MAGIC0):
        # ★uf2 経由だと末尾に最大 255 バイトの 0 詰めが付く (ブロックが 256B
        #   固定のため)。中身は elf 経由と一致するが**長さと CRC は変わる**ので、
        #   `--commit-only` で後から commit するときは同じ形式で渡すこと。
        print(f"  uf2 -> bin: {path}")
        return _uf2_to_bin(blob)
    return blob


def _peripheral_of(client):
    """bleak の CoreBluetooth バックエンドが握っている CBPeripheral。

    ★公開 API ではないので、取れなかったら黙って応答付きへ落とす
      (bleak が変わったときに「速いつもりで壊れている」のが一番困る)。
    """
    if not HAVE_COREBLUETOOTH:
        return None
    backend = getattr(client, "_backend", client)
    peripheral = getattr(backend, "_peripheral", None)
    if peripheral is None or not hasattr(peripheral, "canSendWriteWithoutResponse"):
        return None
    return peripheral


async def _connect_or_explain(device):
    """繋がらなかったら**何が起きているか**を言う。例外を投げっぱなしにしない。

    ★控えたアドレスで繋ぎに行くと、掴まれているだけなら繋がるが、本当に
      落ちていれば例外になる。どちらか分からないまま traceback を出しても
      次の一手が決まらない。
    """
    try:
        client = BleakClient(device, timeout=25.0)
        await client.connect()
        return client
    except Exception as e:  # noqa: BLE001
        print(f"接続できません: {type(e).__name__}: {e}")
        print("  advertise しておらず直接も繋げない = リンクが壊れているか"
              "電源が落ちている。")
        print("  USB を挿して picotool で焼き直すか、電源を入れ直してください。")
        return None


async def main(path: str, do_upload: bool, do_commit: bool,
               with_response: bool, raw_upload: bool,
               legacy: bool = False) -> int:
    _require_ble()
    image = load_image(path)
    crc = zlib.crc32(image) & 0xFFFFFFFF
    commit_cmd = b"XNOC" + struct.pack("<II", len(image), crc)
    # ★total と crc32 は**常に展開後の像**のもの。圧縮するかどうかは転送の
    #   都合でしかなく、commit 側 (と検証) は一切変わらない。
    chunks = None
    if raw_upload:
        body = image
        header = b"XNOU" + struct.pack("<II", len(image), crc)
    elif legacy:
        # ★従来の XNOZ 経路。**回復のために残してある** — 新形式 (XNOR) に
        #   バグがあっても、この経路で板へ焼き直せる。再送は効かない。
        body = deflate_chunks(image)
        header = b"XNOZ" + struct.pack("<II", len(image), crc)
    else:
        chunks = deflate_chunk_list(image)
        body = chunked_body(chunks)
        header = b"XNOR" + struct.pack("<II", len(image), crc)
    print(f"image: {path}  {len(image)} bytes  crc32={crc:08x}")
    if not raw_upload:
        mode = "XNOZ (legacy)" if legacy else f"XNOR ({len(chunks)} chunks)"
        print(f"  deflate: {len(body)} bytes on the wire "
              f"({len(body) / len(image) * 100:.1f}%)  {mode}")

    lines: list[str] = []
    buf = bytearray()

    def on_notify(_h, data):
        buf.extend(data)
        while b"\n" in buf:
            i = buf.index(b"\n")
            line = bytes(buf[: i]).decode(errors="replace").strip()
            del buf[: i + 1]
            if not line.startswith("PICO,"):   # テレメトリは黙らせる
                lines.append(line)
                print("  device:", line, flush=True)

    # ★スキャンで見つからなくても諦めない。誰かが繋いでいる間デバイスは
    #   advertise しないが、**アドレスが分かっていれば直接繋げる**
    #   (shizuku_link.find_device 参照)。
    device = await find_device(timeout=15.0)
    if device is None:
        print("device not found")
        # ★一番ありがちな原因を名指しする。BLE のリンクは 1 本しかなく、
        #   誰かが繋いでいる間デバイスは advertise しない = 見つからない。
        #   GDB の橋は VS Code の background タスクとして生き残りやすい。
        try:
            held = subprocess.run(["pgrep", "-fl", "gdb_ble_bridge.py"],
                                  capture_output=True, text=True).stdout.strip()
        except Exception:  # noqa: BLE001
            held = ""
        if held:
            print("  ★gdb_ble_bridge.py が動いています。BLE のリンクは 1 本 "
                  "しかないので、橋が握っている間デバイスは advertise せず、"
                  "OTA は繋げません。先に落としてください:")
            for line in held.splitlines():
                print(f"    {line}")
        else:
            print("  (他に BLE で繋いでいるものが無いか確認。リンクは 1 本しか"
                  "無く、繋がれている間デバイスは advertise しません)")
        return 1

    client = await _connect_or_explain(device)
    if client is None:
        return 1
    async with client:
        await client.start_notify(NUS_TX_UUID, on_notify)
        await asyncio.sleep(1.0)

        if not do_upload:
            print("skipping upload (--commit-only)")
            return await commit(client, commit_cmd, lines)

        # ★ヘッダだけ先に送り、デバイスが消去を終えて "ready" を出すまで待つ。
        #   消去は両コアを止めて IRQ を切るので (1 ブロック ~105ms)、その間に
        #   流し込むと CYW43 の SPI が壊れる (実機で Bus error → チップごと
        #   ハング → 電源を抜くまで復帰せず)。**消し終わってから流す**。
        if chunks is not None:
            # ★16B を単独で write する (device はこのフレームの残りを捨てる)。
            #   前回の転送を諦めた直後でも、必ず待ち受けから始められる。
            await client.write_gatt_char(OTA_RX_UUID, reset_frame(),
                                         response=True)
            await asyncio.sleep(0.2)
        await client.write_gatt_char(OTA_RX_UUID, header, response=True)
        for _ in range(80):
            await asyncio.sleep(0.1)
            if any(x.startswith("ready") for x in lines):
                break
            if any(("reject" in x) or ("failed" in x) for x in lines):
                print("RESULT: device rejected the transfer")
                await quiet_stop(client)
                return 1
        else:
            # ★古いファームは "ready" を出さない。待ち続けずに進む。
            print("  (no 'ready' from the device; going ahead anyway)")
        payload = body
        # ★既定は write **without** response。1 write = 1 往復 (≒ 2 CI = 30ms)
        #   だった応答付きに対し、非応答なら 1 つの接続イベントに複数パケットが
        #   載る。詰まりの検出は下の wait_ready が受け持つ。
        peripheral = None if with_response else _peripheral_of(client)
        if not with_response and peripheral is None:
            print("  (canSendWriteWithoutResponse が使えないので応答付きにする)")
            with_response = True

        async def wait_ready() -> bool:
            # CoreBluetooth のキューが空くまで待つ。★sleep(0) では回らない
            #   (デリゲートのコールバックが別スレッドから来るので、
            #    イベントループに実際に時間を渡す必要がある)。
            # ★上限を付ける。リンクが止まったまま戻らないことがあり
            #   (2026-08-24 実測: 93% で停止)、上限が無いと**ホストが永久に
            #   回り続けて何も報告しない**。止まったなら止まったと言う。
            deadline = time.perf_counter() + 10.0
            while not peripheral.canSendWriteWithoutResponse():
                if time.perf_counter() > deadline:
                    return False
                await asyncio.sleep(0.002)
            return True

        async def blast(data: bytes, label: str) -> bool:
            """まとめて投げ切る。戻り値 False = リンクが詰まって戻らなかった。

            ★チャンク再送でも「投げ切る」流し方は変えない。窓を作って ACK を
              追いかけるのではなく、送り切ってから足りない分を聞く
              (HTTP の range 再取得と同じ発想)。
            """
            t0 = time.perf_counter()
            sent = 0
            stalls = 0
            for offset in range(0, len(data), CHUNK):
                if not with_response:
                    if not peripheral.canSendWriteWithoutResponse():
                        stalls += 1
                        if not await wait_ready():
                            print(f"\n[{label}] link stalled at "
                                  f"{sent}/{len(data)} bytes "
                                  f"({sent / len(data) * 100:.0f}%) — giving up",
                                  flush=True)
                            return False
                await client.write_gatt_char(
                    OTA_RX_UUID, data[offset : offset + CHUNK],
                    response=with_response,
                )
                await asyncio.sleep(0.005)
                sent += min(CHUNK, len(data) - offset)
                if offset % (CHUNK * 200) == 0:
                    elapsed = time.perf_counter() - t0
                    rate = sent / elapsed / 1024 if elapsed > 0 else 0
                    print(f"  [{label}] sent {sent}/{len(data)}  "
                          f"{rate:.1f} kB/s", flush=True)
            elapsed = max(time.perf_counter() - t0, 1e-9)
            print(f"[{label}] done: {sent} bytes in {elapsed:.1f}s "
                  f"({sent / elapsed / 1024:.1f} kB/s, {stalls} stalls)")
            return True

        if not await blast(payload, "upload"):
            print("RESULT: transfer stalled (本体は無傷)")
            return 1

        if chunks is not None:
            # ★BLE は baud を切り替えないので、問い合わせも**帯域内**で済む
            #   (OTW のように制御用の別経路を用意する必要が無い)。同じ 16B の
            #   チャンクヘッダに seq=0xFFFF を載せるだけ。
            # ★この枠組みが BLE でも効くのは、ACL 層の再送があってもアプリ層は
            #   「投げた」ことしか分からず「届いて書き込めた」までは見えない
            #   ため。stall や切断を**チャンク単位で切り分けられる**ようになる。
            for rnd in range(1, MAX_ROUNDS + 1):
                mark = len(lines)
                await client.write_gatt_char(OTA_RX_UUID, query_frame(),
                                             response=True)
                for _ in range(300):
                    await asyncio.sleep(0.1)
                    if any(x.startswith("NEEDEND") for x in lines[mark:]):
                        break
                n, seqs, complete = parse_need("\n".join(lines[mark:]))
                if not complete or n is None:
                    print(f"[round {rnd}] NEED の返事が最後まで届かない "
                          f"(NEEDEND が見えない)。一覧が途中で切れている可能性が"
                          f"あるので止める — 欠けたまま commit しないこと。")
                    dump_tail(lines)
                    await quiet_stop(client)
                    return 1
                if n == 0:
                    print(f"[round {rnd}] 全チャンク受領")
                    break
                if rnd == MAX_ROUNDS:
                    print(f"{n} チャンクが {MAX_ROUNDS} ラウンドでも埋まらない")
                    dump_tail(lines)
                    await quiet_stop(client)
                    return 1
                print(f"[round {rnd}] {n} チャンク未達 → 再送 ({len(seqs)} 個)")
                if not await blast(chunked_body(chunks, seqs), f"resend{rnd}"):
                    print("RESULT: transfer stalled (本体は無傷)")
                    return 1

        # デバイス側の判定 (done / CRC MISMATCH) が出るまで待つ。
        # ★固定待ちにしない — 判定は転送が終わればすぐ出る。
        for _ in range(100):
            await asyncio.sleep(0.1)
            if any(("OK (staged" in x) or ("MISMATCH" in x) or ("failed" in x)
                   for x in lines):
                break
        await asyncio.sleep(0.3)  # 内訳の行が続くので少しだけ待つ

        staged = any("OK (staged" in line for line in lines)
        if staged:
            print("RESULT: staged OK")
        elif any("MISMATCH" in line for line in lines):
            print("RESULT: CRC mismatch during staging (本体は無傷)")
        elif any("failed" in line for line in lines):
            print("RESULT: flash write failed during staging (本体は無傷)")
        else:
            print("RESULT: not confirmed (no ACK received from device)")

        if not staged:
            dump_tail(lines)
            await quiet_stop(client)
            return 1
        if not do_commit:
            await quiet_stop(client)
            return 0
        return await commit(client, commit_cmd, lines)


async def quiet_stop(client) -> None:
    """通知を止める。★失敗しても黙って進む。

    ここで例外を出すと、**その時点までに集めた診断が全部消える**。実際
    「転送は途中まで進んでいたのに、後始末の disconnected で traceback だけが
    残り、どこで死んだのか分からない」が 2 回起きた。切断は起こる前提の経路
    なので、後始末は結果に影響させない。

    ★2026-09-02 の 6c04010 で各所の `client.stop_notify(NUS_TX_UUID)` をこの
      関数へまとめた際、**本体まで自分自身の呼び出しに置き換わっていた**
      (無限再帰)。通知が止まらないだけでなく、RecursionError をすぐ下の
      except が拾って「無視」と印字するので、**表向き無症状に見える**のが
      たちが悪い。ラッパを作るときは中身が元の呼び出しのままか必ず見ること。
    """
    try:
        await client.stop_notify(NUS_TX_UUID)
    except Exception as e:  # noqa: BLE001
        print(f"  (stop_notify は無視: {type(e).__name__}: {e})")


def dump_tail(lines: list, n: int = 12) -> None:
    """デバイスから最後に届いた行を出す。どこまで進んだかの唯一の手掛かり。"""
    print(f"--- デバイスからの最後の {min(n, len(lines))} 行 ---")
    for line in lines[-n:]:
        print(f"  | {line}")
    if not lines:
        print("  | (1 行も届いていない)")


async def commit(client, commit_cmd: bytes, lines: list) -> int:
    """本体領域を書き換えさせる。**ここから先はデバイスは戻ってこない**。

    デバイスは flash から読み直して CRC を確かめ、一致したら
    「commit: ... no return」を出してから自分がいる領域を消して書き、
    書き終わると再起動する。したがって:
      * 成功の合図は「commit: が出て、そのあと黙って切れる」
      * 失敗の合図は「commit rejected: ...」が出て、切れない
    切断そのものは異常ではないので、例外は握って合図の方で判定する。
    """
    mark = len(lines)
    t0 = time.perf_counter()

    def phase(name):
        print(f"  [{time.perf_counter() - t0:5.1f}s] {name}", flush=True)

    print("committing — ここで電源を切らないこと")
    try:
        await client.write_gatt_char(OTA_RX_UUID, commit_cmd, response=True)
    except Exception as e:  # noqa: BLE001 — 書いた直後に切れることがある
        print(f"  (write returned {type(e).__name__}: {e})")
    # 「commit: 〜」が出るまで待つ (CRC の読み直しに数百 ms かかる)。
    for _ in range(200):
        await asyncio.sleep(0.1)
        new = lines[mark:]
        if any(line.startswith("commit rejected") for line in new):
            print("RESULT: commit rejected (本体は無傷)")
            return 1
        if any(line.startswith("commit:") for line in new):
            phase("device acknowledged, erasing + programming")
            break
    else:
        print("RESULT: no commit acknowledgement — 本体は無傷のはず")
        return 1

    # ここからデバイスは flash を焼いている。切れるのが正常。
    # ★固定の長い待ちをやめた。消去が 64KB ブロック単位になって commit 自体が
    #   2 秒弱で終わるので、待つのではなく**切って探しに行く**。
    try:
        await client.disconnect()
    except Exception:  # noqa: BLE001
        pass
    phase("disconnected, waiting for the device to advertise again")

    expected_size, expected_crc = struct.unpack("<II", commit_cmd[4:12])
    print(f"  expected post-OTA firmware: {expected_size} bytes, crc32={expected_crc:08x}")

    for attempt in range(8):
        device = await find_device(timeout=5.0, verbose=False)
        if device is None:
            continue
        try:
            async with BleakClient(device) as check:
                heard_crc = asyncio.Event()
                verified = [False]
                got_crc = [0]
                got_size = [0]
                rx_buf = bytearray()

                def on_line(_h, data):
                    rx_buf.extend(data)
                    while b"\n" in rx_buf:
                        idx = rx_buf.index(b"\n")
                        line_str = bytes(rx_buf[:idx]).decode(errors="replace").strip()
                        del rx_buf[:idx + 1]
                        if line_str.startswith("VER:"):
                            # VER: size=... crc=...
                            parts = line_str.split()
                            for p in parts[1:]:
                                if p.startswith("crc="):
                                    got_crc[0] = int(p.split("=")[1], 16)
                                elif p.startswith("size="):
                                    got_size[0] = int(p.split("=")[1])
                            if got_crc[0] == expected_crc:
                                verified[0] = True
                            heard_crc.set()

                await check.start_notify(NUS_TX_UUID, on_line)
                # Send 'version' command to the shell to query running image CRC
                for _ in range(3):
                    await check.write_gatt_char(NUS_RX_UUID, b"version\n", response=False)
                    try:
                        await asyncio.wait_for(heard_crc.wait(), timeout=2.0)
                        break
                    except asyncio.TimeoutError:
                        await asyncio.sleep(0.5)

                await quiet_stop(check)

                if verified[0]:
                    phase(f"verified running firmware: size={got_size[0]}B, crc32={got_crc[0]:08x} (matches uploaded image!)")
                    # ★最終行だけを見て判断されることがあるので、ここにも値を載せる。
                    #   「verified」の一語より、突き合わせた数字が出ているほうが強い。
                    print(f"RESULT: commit verified "
                          f"(size={got_size[0]}B crc32={got_crc[0]:08x}, "
                          f"rebooted + running new image, attempt {attempt + 1})")
                    return 0
                elif heard_crc.is_set():
                    print(f"RESULT: commit mismatch (running crc {got_crc[0]:08x} != expected {expected_crc:08x})")
                    return 1
        except Exception as e:  # noqa: BLE001 — 焼いている最中は繋がらない
            print(f"  (not up yet: {type(e).__name__}: {e}); retrying")
            await asyncio.sleep(1.0)
            continue
    print("RESULT: committed, but the device never answered again — "
          "USB CDC を見ること")
    return 1


if __name__ == "__main__":
    args = sys.argv[1:]
    flags = {a for a in args if a.startswith("--")}
    positional = [a for a in args if not a.startswith("--")]
    if len(positional) > 1 or not flags <= {"--commit", "--commit-only",
                                            "--with-response", "--raw",
                                            "--legacy"}:
        print(__doc__)
        raise SystemExit(2)
    image_path = positional[0] if positional else DEFAULT_IMAGE
    if not os.path.exists(image_path):
        if os.path.exists(image_path + ".elf"):
            image_path = image_path + ".elf"
        elif os.path.exists(image_path + ".uf2"):
            image_path = image_path + ".uf2"
        else:
            raise SystemExit(f"イメージがありません: {image_path}\n"
                             "  bazel build //...")
    commit_only = "--commit-only" in flags
    raise SystemExit(asyncio.run(main(
        image_path,
        do_upload=not commit_only,
        do_commit=commit_only or "--commit" in flags,
        with_response="--with-response" in flags,
        raw_upload="--raw" in flags,
        legacy="--legacy" in flags,
    )))
