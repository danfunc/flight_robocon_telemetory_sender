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
from shizuku_link import DEVICE_NAME, NUS_TX_UUID  # noqa: E402

OTA_RX_UUID = "6e402002-b5a3-f393-e0a9-e50e24dcca9e"
# ★1 回の write に載せられるのは ATT MTU - 3。MTU は 247 に頭打ちしてある
#   (btstack_config.h の HCI_ACL_PAYLOAD_SIZE=(247+4))。
CHUNK = 244

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


async def main(path: str, do_upload: bool, do_commit: bool) -> int:
    image = load_image(path)
    crc = zlib.crc32(image) & 0xFFFFFFFF
    header = b"XNOU" + struct.pack("<II", len(image), crc)
    commit_cmd = b"XNOC" + struct.pack("<II", len(image), crc)
    print(f"image: {path}  {len(image)} bytes  crc32={crc:08x}")

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

    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=15.0)
    if device is None:
        print("device not found")
        return 1

    async with BleakClient(device) as client:
        await client.start_notify(NUS_TX_UUID, on_notify)
        await asyncio.sleep(1.0)

        if not do_upload:
            print("skipping upload (--commit-only)")
            return await commit(client, commit_cmd, lines)

        payload = header + image
        t0 = time.perf_counter()
        sent = 0
        for offset in range(0, len(payload), CHUNK):
            # ★response=True (write with response) にする。**流量制御がここに
            #   しかない** — response=False だと CoreBluetooth のキューへ
            #   一気に積まれ、デバイス側の受信リング (32 フレーム ≒ 7.8KB) が
            #   即座に溢れて切断される (実測: 396KB を「0.0 秒」で送って落ちた)。
            #   1 write = 1 往復 ≒ 1 CI (15ms) になるが、確実に届く。
            await client.write_gatt_char(
                OTA_RX_UUID, payload[offset : offset + CHUNK], response=True
            )
            sent += min(CHUNK, len(payload) - offset)
            if offset % (CHUNK * 100) == 0:
                elapsed = time.perf_counter() - t0
                rate = sent / elapsed / 1024 if elapsed > 0 else 0
                print(f"  sent {sent}/{len(payload)}  {rate:.1f} kB/s", flush=True)
        elapsed = time.perf_counter() - t0
        print(f"transfer done: {sent} bytes in {elapsed:.1f}s "
              f"({sent / elapsed / 1024:.1f} kB/s)")

        # デバイス側の判定 (done / CRC MISMATCH) が出るまで少し待つ
        await asyncio.sleep(5.0)

        staged = any("OK" in line for line in lines)
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
            break
    else:
        print("RESULT: no commit acknowledgement — 本体は無傷のはず")
        return 1

    # ここからデバイスは flash を焼いている。切れるのが正常。
    print("  erasing + programming, then rebooting ...")
    await asyncio.sleep(20.0)
    try:
        await client.disconnect()
    except Exception:  # noqa: BLE001
        pass

    # 新しい像が本当に起動したかは「また advertise してくるか」で見る。
    print("  waiting for the device to come back ...")
    for attempt in range(6):
        device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10.0)
        if device is not None:
            print(f"RESULT: committed and rebooted (found again after "
                  f"{attempt + 1} scan(s))")
            return 0
    print("RESULT: committed, but the device did not advertise again — "
          "USB CDC を見ること")
    return 1


if __name__ == "__main__":
    args = sys.argv[1:]
    flags = {a for a in args if a.startswith("--")}
    positional = [a for a in args if not a.startswith("--")]
    if len(positional) > 1 or not flags <= {"--commit", "--commit-only"}:
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
    )))
