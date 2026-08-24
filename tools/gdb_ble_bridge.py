#!/usr/bin/env python3
"""GDB (RSP) を BLE で運ぶ橋。GDB は TCP で繋ぎ、こちらが BLE へ中継する。

  python3 tools/gdb_ble_bridge.py            # 127.0.0.1:3333 で待つ
  python3 tools/gdb_ble_bridge.py --port 4444

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
import socket
import sys

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from bleak import BleakClient, BleakScanner  # noqa: E402
from shizuku_link import DEVICE_NAME  # noqa: E402

GDB_TX_UUID = "6e401003-b5a3-f393-e0a9-e50e24dcca9e"  # notify: stub → host
GDB_RX_UUID = "6e401002-b5a3-f393-e0a9-e50e24dcca9e"  # write:  host → stub
# 1 回の write に載せられるのは ATT MTU - 3。MTU は 247 に頭打ちしてある。
# デバイス側はこれを 64B の link_chunk へ割り直すので、こちらで刻む必要はない。
CHUNK = 244


async def run(port: int) -> int:
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=15.0)
    if device is None:
        print("device not found", flush=True)
        return 1
    print(f"found: {device}", flush=True)

    async with BleakClient(device) as client:
        # ★GDB が来る前から溜めておく。stub は接続を認識した時点で停止理由を
        #   送ってくることがあり、それを取りこぼすと最初の握手で固まる。
        pending = bytearray()
        state = {"writer": None}

        def on_notify(_handle, data: bytearray):
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
                    await client.write_gatt_char(GDB_RX_UUID, data, response=True)
            except (ConnectionResetError, asyncio.IncompleteReadError):
                pass
            finally:
                state["writer"] = None
                writer.close()
                print("gdb disconnected", flush=True)

        server = await asyncio.start_server(serve, "127.0.0.1", port)
        print(f"listening on 127.0.0.1:{port}"
              f"   →  gdb: target remote :{port}", flush=True)
        async with server:
            await server.serve_forever()
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", type=int, default=3333)
    try:
        raise SystemExit(asyncio.run(run(ap.parse_args().port)))
    except KeyboardInterrupt:
        print("\nbye")
