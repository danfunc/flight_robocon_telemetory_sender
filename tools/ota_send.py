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
from bleak import BleakClient, BleakScanner  # noqa: E402
from shizuku_link import DEVICE_NAME, NUS_TX_UUID, find_device  # noqa: E402

OTA_RX_UUID = "6e402002-b5a3-f393-e0a9-e50e24dcca9e"
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


def deflate_chunks(image: bytes) -> bytes:
    """4KB ごとに独立した raw deflate にして [u16 len][data] で並べる。"""
    out = bytearray()
    for at in range(0, len(image), ZCHUNK):
        raw = image[at : at + ZCHUNK]
        c = zlib.compressobj(9, zlib.DEFLATED, ZWBITS)
        comp = c.compress(raw) + c.flush()
        if len(comp) >= len(raw) + 5:
            # ★縮まないチャンクは自分で「無圧縮ブロック」に組む。deflate の
            #   枠が付いて 4KB を超え、デバイスの上限に当たるのを防ぐ。
            #   0x01 = 最終ブロック・無圧縮、続けて LEN と ~LEN。
            comp = b"" + struct.pack("<HH", len(raw), len(raw) ^ 0xFFFF) + raw
        if len(comp) > ZCHUNK_MAX:
            raise SystemExit(f"chunk at {at} is {len(comp)}B > {ZCHUNK_MAX}B")
        out += struct.pack("<H", len(comp)) + comp
    return bytes(out)

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
               with_response: bool, raw_upload: bool) -> int:
    image = load_image(path)
    crc = zlib.crc32(image) & 0xFFFFFFFF
    commit_cmd = b"XNOC" + struct.pack("<II", len(image), crc)
    # ★total と crc32 は**常に展開後の像**のもの。圧縮するかどうかは転送の
    #   都合でしかなく、commit 側 (と検証) は一切変わらない。
    if raw_upload:
        body = image
        header = b"XNOU" + struct.pack("<II", len(image), crc)
    else:
        body = deflate_chunks(image)
        header = b"XNOZ" + struct.pack("<II", len(image), crc)
    print(f"image: {path}  {len(image)} bytes  crc32={crc:08x}")
    if not raw_upload:
        print(f"  deflate: {len(body)} bytes on the wire "
              f"({len(body) / len(image) * 100:.1f}%)")

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
        await client.write_gatt_char(OTA_RX_UUID, header, response=True)
        for _ in range(80):
            await asyncio.sleep(0.1)
            if any(x.startswith("ready") for x in lines):
                break
            if any(("reject" in x) or ("failed" in x) for x in lines):
                print("RESULT: device rejected the transfer")
                await client.stop_notify(NUS_TX_UUID)
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
                await asyncio.sleep(0.0005)
            return True

        t0 = time.perf_counter()
        sent = 0
        stalls = 0
        for offset in range(0, len(payload), CHUNK):
            if not with_response:
                if not peripheral.canSendWriteWithoutResponse():
                    stalls += 1
                    if not await wait_ready():
                        print(f"\nlink stalled at {sent}/{len(payload)} bytes "
                              f"({sent / len(payload) * 100:.0f}%) — giving up",
                              flush=True)
                        print("RESULT: transfer stalled (本体は無傷)")
                        return 1
            await client.write_gatt_char(
                OTA_RX_UUID, payload[offset : offset + CHUNK],
                response=with_response,
            )
            sent += min(CHUNK, len(payload) - offset)
            if offset % (CHUNK * 200) == 0:
                elapsed = time.perf_counter() - t0
                rate = sent / elapsed / 1024 if elapsed > 0 else 0
                print(f"  sent {sent}/{len(payload)}  {rate:.1f} kB/s", flush=True)
        elapsed = time.perf_counter() - t0
        print(f"transfer done: {sent} bytes in {elapsed:.1f}s "
              f"({sent / elapsed / 1024:.1f} kB/s, {stalls} stalls)")

        # デバイス側の判定 (done / CRC MISMATCH) が出るまで待つ。
        # ★固定待ちにしない — 判定は転送が終わればすぐ出る。
        for _ in range(100):
            await asyncio.sleep(0.1)
            if any(("OK (staged" in x) or ("MISMATCH" in x) or ("failed" in x)
                   for x in lines):
                break
        await asyncio.sleep(0.3)  # 内訳の行が続くので少しだけ待つ

        staged = any("OK (staged" in line for line in lines)
        print("RESULT:", "staged OK" if staged else "not confirmed")
        if not staged:
            await client.stop_notify(NUS_TX_UUID)
            return 1
        if not do_commit:
            await client.stop_notify(NUS_TX_UUID)
            return 0
        return await commit(client, commit_cmd, lines)


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

    # ★「スキャンで見つかった」を成功の合図にしない。macOS は一度見つけた
    #   ペリフェラルを覚えていて、**まだ焼いている最中でも即座に返してくる**
    #   (実測: commit 確認から 0.9 秒で「戻った」と出たが、消去と書き込みだけで
    #   1.7 秒かかるのだから戻れるはずがない)。実際に**繋いで、喋ることを
    #   確かめる** — これなら再起動して BLE が上がりきったことの証拠になる。
    for attempt in range(8):
        device = await find_device(timeout=5.0, verbose=False)
        if device is None:
            continue
        try:
            async with BleakClient(device) as check:
                heard = asyncio.Event()

                def on_line(_h, _data):
                    heard.set()

                await check.start_notify(NUS_TX_UUID, on_line)
                try:
                    await asyncio.wait_for(heard.wait(), timeout=6.0)
                except asyncio.TimeoutError:
                    print("  (connected but heard nothing; retrying)")
                    continue
                await check.stop_notify(NUS_TX_UUID)
        except Exception as e:  # noqa: BLE001 — 焼いている最中は繋がらない
            print(f"  (not up yet: {type(e).__name__}); retrying")
            await asyncio.sleep(1.0)
            continue
        phase("device is back and talking")
        print(f"RESULT: committed and rebooted (verified on attempt "
              f"{attempt + 1})")
        return 0
    print("RESULT: committed, but the device never answered again — "
          "USB CDC を見ること")
    return 1


if __name__ == "__main__":
    args = sys.argv[1:]
    flags = {a for a in args if a.startswith("--")}
    positional = [a for a in args if not a.startswith("--")]
    if len(positional) > 1 or not flags <= {"--commit", "--commit-only",
                                            "--with-response", "--raw"}:
        print(__doc__)
        raise SystemExit(2)
    image_path = positional[0] if positional else DEFAULT_IMAGE
    if not os.path.exists(image_path):
        raise SystemExit(f"イメージがありません: {image_path}\n"
                         "  bazel build //firmware_bazel:xno_bringup")
    commit_only = "--commit-only" in flags
    raise SystemExit(asyncio.run(main(
        image_path,
        do_upload=not commit_only,
        do_commit=commit_only or "--commit" in flags,
        with_response="--with-response" in flags,
        raw_upload="--raw" in flags,
    )))
