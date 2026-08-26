#!/usr/bin/env python3
"""GDB (RSP) を BLE で運ぶ橋。GDB は TCP で繋ぎ、こちらが BLE へ中継する。

  python3 tools/gdb_ble_bridge.py            # 127.0.0.1:3333 で待つ
  python3 tools/gdb_ble_bridge.py --port 4444
  python3 tools/gdb_ble_bridge.py --stay     # GDB が切れても居座る
  python3 tools/gdb_ble_bridge.py --log rsp.txt   # RSP を記録する

別の端末から:

  arm-none-eabi-gdb bazel-bin/firmware_bazel/xno_bringup \
      -ex 'target remote :3333'

★GDB 側からは「ただの remote target」に見える。RSP の中身はこの橋も
  ble_uart も**一切解釈しない** —— 素のバイト列を運ぶだけ。解釈するのは
  Shizuku 側の gdb server ひとつ。

★NUS (テレメトリ) とは別の characteristic を使う。RSP は `$...#xx` の
  枠付きバイト列なので、CSV 行が同じチャネルに混ざると握手ごと壊れる
  (ble_uart.gatt のコメント)。

★notify を購読するまで stub は「繋がっていない」扱い
  (`gdb_link_set_connected(notify_enabled && authorized)`)。だから GDB が
  来る前に購読を済ませておく。認可されていないリンクからの RSP は
  デバイス側で捨てられる (fail-closed) —— GDB は任意のメモリ読み書きと
  レジスタ操作そのものなので、ペアリング済みでなければ繋がらない。

★書き込みは `response=True`。**流量制御がここにしかない**のは OTA と同じ理由。
  1 write = 1 往復 ≒ 1 CI (15ms) だが、RSP は 1 パケットが小さいので効く。
"""
import argparse
import asyncio
import os
import socket
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from bleak import BleakClient, BleakScanner  # noqa: E402
from shizuku_link import DEVICE_NAME, find_device  # noqa: E402

GDB_TX_UUID = "6e401003-b5a3-f393-e0a9-e50e24dcca9e"  # notify: stub → host
GDB_RX_UUID = "6e401002-b5a3-f393-e0a9-e50e24dcca9e"  # write:  host → stub
# 1 回の write に載せられるのは ATT MTU - 3。MTU は 247 に頭打ちしてある。
# デバイス側はこれを 64B の link_chunk へ割り直すので、こちらで刻む必要はない。
CHUNK = 244


def _printable(data: bytes) -> str:
    """RSP を読める形に。制御文字は <XX> で見せる (0x03 = Ctrl-C 等)。"""
    out = []
    for b in data:
        if 0x20 <= b < 0x7F:
            out.append(chr(b))
        elif b == 0x0A:
            out.append("\\n")
        else:
            out.append(f"<{b:02x}>")
    return "".join(out)


HEARTBEAT_MAX_AGE_S = 8.0


def _heartbeat_path(port: int) -> str:
    return os.path.join(tempfile.gettempdir(), f"gdb_ble_bridge.{port}.beat")


def _read_heartbeat(port: int):
    """生きている橋の心拍。無い/古い/プロセスが居ないなら None。"""
    try:
        with open(_heartbeat_path(port), encoding="utf-8") as f:
            pid_text, stamp_text = f.read().split()
        pid, stamp = int(pid_text), float(stamp_text)
        age = time.time() - stamp
        if age > HEARTBEAT_MAX_AGE_S:
            return None
        os.kill(pid, 0)  # 居るか確かめるだけ (シグナルは送らない)
        return {"pid": pid, "age": age}
    except Exception:  # noqa: BLE001 — 読めない = 生きていないとみなす
        return None


def _port_is_served(port: int) -> bool:
    """既に誰かがそのポートで待っているか。

    ★**繋いで確かめてはいけない**。この橋は繋いできた相手を GDB とみなすので、
      様子見の接続が「GDB が来て、すぐ去った」と解釈され、既定では
      そこで終了してしまう (2026-08-25 に実際に自分で殺した)。
      bind できるかどうかで見れば、相手に触らずに分かる。
    """
    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        probe.bind(("127.0.0.1", port))  # SO_REUSEADDR は付けない
        return False
    except OSError:
        return True
    finally:
        probe.close()


async def run(port: int, stay: bool, log_path: str | None) -> int:
    # ★★**先にポートを見る**。BLE を探すより前に。
    #   前のセッションの橋が残っていると、そいつが BLE リンクを握っている
    #   ので新しい橋の scan は必ず失敗する。そこで終了すると VS Code の
    #   preLaunchTask が失敗し、**デバッグが始まる前にアダプタが死ぬ**
    #   (2026-08-25 に踏んだ)。
    #   ★既に上がっているなら、それは**使える橋**なので譲って正常終了する。
    #     GDB はそちらへ繋がる。新しく立て直す必要はない。
    if _port_is_served(port):
        # ★★譲る前に**生きているか確かめる**。ポートを掴んでいるだけで BLE が
        #   死んでいる橋に譲ると、GDB は**無反応の橋に繋がって attach できない**
        #   (症状が「たまに繋がらない」になり、原因が一番見えにくい形になる)。
        #   生存は心拍ファイルで見る。古ければ居座りとみなして退かせる。
        beat = _read_heartbeat(port)
        if beat is not None:
            print(f"a bridge is already listening on 127.0.0.1:{port} "
                  f"(pid {beat['pid']}, {beat['age']:.1f}s前に心拍) — "
                  f"そちらを使うのでこのプロセスは終了します", flush=True)
            return 0
        print(f"port {port} is held but the bridge looks dead — 退かせます",
              flush=True)
        subprocess.run(["pkill", "-f", "gdb_ble_bridge.py"], check=False)
        await asyncio.sleep(2.0)
        if _port_is_served(port):
            print("まだ掴まれています。手で落としてください: "
                  "pkill -f gdb_ble_bridge.py", flush=True)
            return 1

    device = await find_device(timeout=15.0)
    if device is None:
        print("device not found", flush=True)
        print("  ★他に BLE で繋いでいるものが無いか確認 (リンクは 1 本しか無く、"
              "繋がれている間デバイスは advertise しません)。"
              "古い gdb_ble_bridge.py が残っていないか:", flush=True)
        print("    pkill -f gdb_ble_bridge.py", flush=True)
        return 1
    print(f"found: {device}", flush=True)

    async with BleakClient(device) as client:
        # ★GDB が来る前から溜めておく。stub は接続を認識した時点で停止理由を
        #   送ってくることがあり、それを取りこぼすと最初の握手で固まる。
        pending = bytearray()
        state = {"writer": None}
        finished = asyncio.Event()

        # ★RSP をそのまま記録する。**中身は解釈しない** — 誰が何を送ったかを
        #   後から読むためだけのもの。VS Code / cppdbg が実際に何を撃つかは
        #   仕様を読んでも分からない (アダプタの実装次第) ので、測る。
        t0 = time.perf_counter()
        log = open(log_path, "w", encoding="utf-8") if log_path else None

        def record(direction: str, data: bytes):
            if log is None:
                return
            log.write(f"{time.perf_counter() - t0:8.3f} {direction} "
                      f"{_printable(data)}\n")
            log.flush()

        def on_notify(_handle, data: bytearray):
            record("dev->gdb", bytes(data))
            writer = state["writer"]
            if writer is None:
                pending.extend(data)
                return
            writer.write(bytes(data))

        await client.start_notify(GDB_TX_UUID, on_notify)
        print("subscribed to the GDB characteristic", flush=True)

        async def serve(reader, writer):
            if state["writer"] is not None:
                print("  (already have a GDB; refusing the second)")
                writer.close()
                return
            sock = writer.get_extra_info("socket")
            if sock is not None:
                # ★RSP は小さいパケットの往復なので Nagle は害しかない。
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            state["writer"] = writer
            print(f"gdb connected from {writer.get_extra_info('peername')}",
                  flush=True)
            if pending:
                writer.write(bytes(pending))
                pending.clear()
            try:
                while True:
                    data = await reader.read(CHUNK)
                    if not data:
                        break
                    record("gdb->dev", data)
                    await client.write_gatt_char(GDB_RX_UUID, data, response=True)
            except (ConnectionResetError, asyncio.IncompleteReadError):
                pass
            finally:
                state["writer"] = None
                writer.close()
                print("gdb disconnected", flush=True)
                if not stay:
                    # ★BLE を離す。握ったままだと OTA が繋げない。
                    print("releasing the BLE link (--stay で居座れる)",
                          flush=True)
                    finished.set()

        # ★心拍を打ち続ける。次に起きた橋が「譲ってよい相手か」を見る材料。
        async def heartbeat():
            while True:
                try:
                    with open(_heartbeat_path(port), "w", encoding="utf-8") as f:
                        f.write(f"{os.getpid()} {time.time()}")
                except OSError:
                    pass
                await asyncio.sleep(2.0)

        beat_task = asyncio.create_task(heartbeat())

        server = await asyncio.start_server(serve, "127.0.0.1", port)
        print(f"listening on 127.0.0.1:{port}"
              f"   →  gdb: target remote :{port}", flush=True)
        async with server:
            try:
                if stay:
                    await server.serve_forever()
                else:
                    await finished.wait()
            finally:
                beat_task.cancel()
                try:
                    os.unlink(_heartbeat_path(port))
                except OSError:
                    pass
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", type=int, default=3333)
    ap.add_argument("--log", metavar="FILE",
                    help="RSP のやり取りをこのファイルへ記録する")
    ap.add_argument("--stay", action="store_true",
                    help="GDB が切れても終わらない (BLE を握り続けるので "
                         "その間 OTA は出来ない)")
    args = ap.parse_args()
    try:
        raise SystemExit(asyncio.run(run(args.port, args.stay, args.log)))
    except KeyboardInterrupt:
        print("\nbye")
