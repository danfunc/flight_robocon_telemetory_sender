#!/usr/bin/env python3
"""Shizuku テレメトリ 共有リンク層 (BLE + フレーム解析)

操縦者向け計器クライアント (shizuku_pilot_client.py) と エンジニア向けグラフ
クライアント (shizuku_telemetry_client.py) の双方が import して使う、プロトコル
依存部分を 1 箇所に集約したモジュール。

含むもの:
  - NUS UUID / デバイス名 / BLE スループット上限の定数
  - バイナリ・バッチフレームの書式 (TELEMETRY_SENDER.cpp と一致)
  - crc16_ccitt / frame_unstuff / parse_batch / parse_csv_line
  - TelemetryRx: 0x0A 区切りストリームをサンプル(dict)とテキスト行に分解する
  - BleWorker: 専用スレッドで asyncio + bleak を回す BLE ワーカー
"""

import asyncio
import queue
import struct
import threading
import time

from bleak import BleakClient, BleakScanner

# ---- NUS UUID -------------------------------------------------------------
NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # notify (受信)
NUS_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # write  (送信)
DEVICE_NAME = "Shizuku UART"

# ---- BLE スループットの理論上限 -------------------------------------------
BLE_NOTIFY_PAYLOAD = 524      # MTU - 3 [byte]
BLE_EFFECTIVE_CI_MS = 30.0    # 1 notify/event の実効接続インターバル [ms]
BLE_CEILING_KBPS = BLE_NOTIFY_PAYLOAD * 8.0 / BLE_EFFECTIVE_CI_MS  # ≈ 139.7 kbps
BLE_CEILING_KBYTES = BLE_NOTIFY_PAYLOAD / BLE_EFFECTIVE_CI_MS      # ≈ 17.5 kB/s

# ---- バイナリテレメトリ バッチフレーム (TELEMETRY_SENDER.cpp と一致) -------
# ワイヤ: [0x02 マーカー][0x0A バイトスタッフィング本体][0x0A 終端]
#   本体 = batch_header(17B) + count × batch_sample(35B) + CRC16-CCITT(2B, LE)
FRAME_MARKER_BATCH = 0x02
FRAME_ESC = 0x1B
BATCH_HDR_FMT = "<BBBIIIH"               # ver,count,flags,seq0,t0_up_ms,t0_wall_s,t0_wall_ms
BATCH_HDR_SIZE = struct.calcsize(BATCH_HDR_FMT)        # 17
BATCH_SAMPLE_FMT = "<HhIiiiiHhhBBBh"     # d_ms,temp,press,altb,altf,vel,az,head,roll,pitch,calib,vstate,elev,servo
BATCH_SAMPLE_SIZE = struct.calcsize(BATCH_SAMPLE_FMT)  # 35

# ---- テレメトリ列の定義 (firmware の並びと一致) ---------------------------
#   (キー, 表示名, スケール除数, 単位)。整数値を除数で割ると物理量になる。
TELEMETRY_FIELDS = [
    ("seq",        "シーケンス",      1,    ""),
    ("up_ms",      "稼働時間",        1000, "s"),   # ms -> s
    ("temp_c",     "温度",            100,  "°C"),
    ("press_hpa",  "気圧",            100,  "hPa"),  # Pa -> hPa
    ("alt_baro_m", "気圧高度",        1000, "m"),
    ("alt_fused_m", "融合高度",       1000, "m"),
    ("vel_m_s",    "上下速度",        1000, "m/s"),
    ("az_m_s2",    "鉛直加速度",      1000, "m/s²"),
    ("heading",    "方位",            100,  "°"),
    ("roll",       "ロール",          100,  "°"),
    ("pitch",      "ピッチ",          100,  "°"),
    ("calib",      "較正(SGAM)",      1,    ""),
    ("vstate",     "上下状態",        1,    ""),
    ("elev",       "エレベータ",      1,    ""),
    ("servo_deg",  "サーボ指令",      100,  "°"),
]
VSTATE_STR = {0: "LEVEL -", 1: "ASC ▲", 2: "DESC ▼"}
ELEV_STR = {0: "NEUTRAL", 1: "UP ▲", 2: "DOWN ▼"}


# ---- フレーム解析 ----------------------------------------------------------
def crc16_ccitt(data: bytes) -> int:
    """CRC16-CCITT (poly 0x1021, init 0xFFFF)。firmware の実装と一致。"""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def frame_unstuff(data: bytes) -> bytes:
    """0x0A バイトスタッフィングを戻す (ESC 0x01->0x0A, ESC 0x02->ESC)。"""
    out = bytearray()
    i, n = 0, len(data)
    while i < n:
        b = data[i]
        if b == FRAME_ESC and i + 1 < n:
            out.append(0x0A if data[i + 1] == 0x01 else FRAME_ESC)
            i += 2
        else:
            out.append(b)
            i += 1
    return bytes(out)


def _vals_from_sample(seq, up_s, rec_fields):
    (d_ms, temp_cC, press_Pa, alt_baro_mm, alt_fused_mm, vel_mm_s, az_mm_s2,
     head_cdeg, roll_cdeg, pitch_cdeg, calib, vstate, elev, servo_cdeg) = rec_fields
    return {
        "seq": seq, "up_ms": up_s,
        "temp_c": temp_cC / 100.0, "press_hpa": press_Pa / 100.0,
        "alt_baro_m": alt_baro_mm / 1000.0, "alt_fused_m": alt_fused_mm / 1000.0,
        "vel_m_s": vel_mm_s / 1000.0, "az_m_s2": az_mm_s2 / 1000.0,
        "heading": head_cdeg / 100.0, "roll": roll_cdeg / 100.0,
        "pitch": pitch_cdeg / 100.0, "calib": calib,
        "vstate": vstate, "elev": elev, "servo_deg": servo_cdeg / 100.0,
    }


def parse_batch(seg: bytes):
    """先頭が FRAME_MARKER_BATCH のセグメントを解析。

    返り値 (samples: list[dict], last_wall: float|None, ok: bool)。
    """
    body = frame_unstuff(seg[1:])
    if len(body) < BATCH_HDR_SIZE + 2:
        return [], None, False
    payload, crc_rx = body[:-2], body[-2:]
    if crc16_ccitt(payload) != int.from_bytes(crc_rx, "little"):
        return [], None, False
    ver, count, flags, seq0, t0_up_ms, t0_wall_s, t0_wall_ms = struct.unpack(
        BATCH_HDR_FMT, payload[:BATCH_HDR_SIZE])
    if ver != 2:
        return [], None, False
    synced = bool(flags & 0x01)
    t0_wall = (t0_wall_s + t0_wall_ms / 1000.0) if synced else None
    off = BATCH_HDR_SIZE
    samples = []
    last_wall = None
    for i in range(count):
        rec = payload[off:off + BATCH_SAMPLE_SIZE]
        off += BATCH_SAMPLE_SIZE
        if len(rec) < BATCH_SAMPLE_SIZE:
            break
        fields = struct.unpack(BATCH_SAMPLE_FMT, rec)
        d_ms = fields[0]
        samples.append(_vals_from_sample(seq0 + i, (t0_up_ms + d_ms) / 1000.0, fields))
        if t0_wall is not None:
            last_wall = t0_wall + d_ms / 1000.0
    return samples, last_wall, True


def parse_csv_line(line: str):
    """CSV テレメトリ行 (F0 モード "PICO,...") を vals(dict) に。失敗 None。"""
    parts = line.split(",")
    if len(parts) != 1 + len(TELEMETRY_FIELDS):
        return None
    try:
        raw = [int(x) for x in parts[1:]]
    except ValueError:
        return None
    vals = {}
    for (key, _label, scale, _unit), r in zip(TELEMETRY_FIELDS, raw):
        vals[key] = r / scale if scale != 1 else r
    return vals


class TelemetryRx:
    """0x0A 区切りの受信ストリームを (samples, texts) に分解する。

    先頭 0x02 はバイナリ・バッチ → samples(dict) に展開。"PICO," 始まりのテキスト
    も CSV テレメトリとして samples へ。それ以外のテキスト(RSSI=, P, BEND...)は
    texts に。バッチは複数 notify に跨るため内部バッファに連結してから処理する。
    """

    def __init__(self):
        self._buf = bytearray()

    def feed(self, chunk: bytes):
        self._buf += chunk
        samples, texts = [], []
        while b"\n" in self._buf:
            seg, _, self._buf = self._buf.partition(b"\n")
            seg = seg.rstrip(b"\r")
            if not seg:
                continue
            if seg[0] == FRAME_MARKER_BATCH:
                s, _wall, ok = parse_batch(seg)
                if ok:
                    samples.extend(s)
            else:
                line = seg.decode("utf-8", errors="replace")
                if line.startswith("PICO,"):
                    v = parse_csv_line(line)
                    if v is not None:
                        samples.append(v)
                else:
                    texts.append(line)
        return samples, texts


class BleWorker:
    """専用スレッドで asyncio ループを回し bleak を駆動する。

    GUI -> worker : submit() でコルーチン投入 (run_coroutine_threadsafe)
    worker -> GUI : ui_queue へ (kind, payload) を put し、GUI が after で取得。
    """

    def __init__(self, ui_queue: "queue.Queue"):
        self.ui_queue = ui_queue
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.client: BleakClient | None = None
        self.devices: dict[str, object] = {}

    def start(self):
        self.thread.start()

    def _run(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def submit(self, coro):
        return asyncio.run_coroutine_threadsafe(coro, self.loop)

    def stop(self):
        async def _cleanup():
            if self.client and self.client.is_connected:
                await self.client.disconnect()
        try:
            self.submit(_cleanup()).result(timeout=5)
        except Exception:
            pass
        self.loop.call_soon_threadsafe(self.loop.stop)

    def post(self, kind: str, **payload):
        self.ui_queue.put((kind, payload))

    def log(self, msg: str):
        self.post("log", text=msg)

    # -- BLE 操作 -----------------------------------------------------------
    async def scan(self, timeout: float = 5.0):
        self.log(f"スキャン中… ({timeout:.0f}s)")
        self.devices.clear()
        try:
            found = await BleakScanner.discover(timeout=timeout)
        except Exception as e:  # noqa: BLE001
            self.log(f"スキャン失敗: {e}")
            self.post("scan_done", items=[])
            return
        items = []
        for d in found:
            name = d.name or "(no name)"
            self.devices[d.address] = d
            items.append((d.name == DEVICE_NAME, f"{name}  [{d.address}]", d.address))
        items.sort(key=lambda t: (not t[0], t[1]))
        self.log(f"{len(items)} 台検出")
        self.post("scan_done", items=[(lbl, addr) for _, lbl, addr in items])

    async def connect(self, address: str):
        if self.client and self.client.is_connected:
            await self.client.disconnect()
        self.log(f"接続中… {address}")

        def _on_disconnect(_c):
            self.post("disconnected")
            self.log("切断されました")

        try:
            self.client = BleakClient(address, disconnected_callback=_on_disconnect)
            await self.client.connect()
        except Exception as e:  # noqa: BLE001
            self.log(f"接続失敗: {e}")
            self.post("disconnected")
            return

        def _on_rx(_sender, data: bytearray):
            # 生バイトで渡す (バイナリフレームを utf-8 で壊さないため)。
            self.post("rx", data=bytes(data))

        try:
            await self.client.start_notify(NUS_TX_UUID, _on_rx)
        except Exception as e:  # noqa: BLE001
            self.log(f"notify 購読失敗: {e}")
            await self.client.disconnect()
            self.post("disconnected")
            return
        self.log("接続完了・notify 購読開始")
        self.post("connected", address=address)

    async def send(self, data: bytes):
        if not (self.client and self.client.is_connected):
            self.log("未接続のため送信できません")
            return
        try:
            await self.client.write_gatt_char(NUS_RX_UUID, data, response=False)
            self.post("tx", data=data.decode("utf-8", errors="replace"))
        except Exception as e:  # noqa: BLE001
            self.log(f"送信失敗: {e} (ペアリングが必要かもしれません)")

    async def send_time(self):
        if not (self.client and self.client.is_connected):
            self.log("未接続のため同期できません")
            return
        epoch_us = int(time.time() * 1_000_000)
        try:
            await self.client.write_gatt_char(
                NUS_RX_UUID, f"T{epoch_us}\n".encode("ascii"), response=False)
            self.log(f"時刻同期送信: epoch_us={epoch_us}")
        except Exception as e:  # noqa: BLE001
            self.log(f"同期失敗: {e} (ペアリングが必要かもしれません)")

    async def read_rssi(self):
        if not (self.client and self.client.is_connected):
            return
        try:
            rssi = await self.client._backend.get_rssi()
            self.post("rssi_host", value=int(rssi))
        except Exception as e:  # noqa: BLE001
            self.post("rssi_host", value=None, error=str(e))

    async def disconnect(self):
        if self.client and self.client.is_connected:
            await self.client.disconnect()


__all__ = [
    "NUS_SERVICE_UUID", "NUS_TX_UUID", "NUS_RX_UUID", "DEVICE_NAME",
    "BLE_NOTIFY_PAYLOAD", "BLE_EFFECTIVE_CI_MS", "BLE_CEILING_KBPS",
    "BLE_CEILING_KBYTES", "FRAME_MARKER_BATCH", "FRAME_ESC",
    "BATCH_HDR_FMT", "BATCH_HDR_SIZE", "BATCH_SAMPLE_FMT", "BATCH_SAMPLE_SIZE",
    "TELEMETRY_FIELDS", "VSTATE_STR", "ELEV_STR",
    "crc16_ccitt", "frame_unstuff", "parse_batch", "parse_csv_line",
    "TelemetryRx", "BleWorker",
]
