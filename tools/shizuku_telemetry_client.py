#!/usr/bin/env python3
"""Shizuku 飛行テレメトリ BLE クライアント (macOS / クロスプラットフォーム)

Pico (pico2_w) 上の Shizuku OS で動く TELEMETRY_SENDER オブジェクトが、BME280 +
BNO055 を融合して BLE UART (Nordic UART Service) の Notify で流す CSV テレメトリを
受信し、数値パネル + 高度/速度ストリップチャートで可視化する GUI クライアント。

あわせて、参考にした ble_uart_gui.py のベンチマーク機能 (RSSI グラフ・Ping/RTT・
ダウンリンク スループット試験・RTC 時刻同期) も内包する。

通信:
  bleak  (BLE, クロスプラットフォーム)
  tkinter (GUI, Python 標準ライブラリ)

NUS / Shizuku 側定数 (BLE_UART_DRIVER.cpp / ble_uart.gatt と一致):
  デバイス名      : "Shizuku UART"
  Service         : 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
  TX (Peri->Cent) : 6E400003-... (Notify)  <- 購読して受信
  RX (Cent->Peri) : 6E400002-... (Write)   <- コマンド送信

ダウンリンク テレメトリ (TELEMETRY_SENDER.cpp と一致):
  既定はバイナリフレーム (~44B/フレーム。CSV の約 1/3 で BLE の ~140kbps 上限に余裕):
    [0x01 マーカー][0x0A バイトスタッフィング本体][0x0A 終端]
    本体 = telem_packet_t(42B, LE, '<BIIhIiiiiHhhBBBh') + CRC16-CCITT(2B, LE)
  F0 コマンドで CSV テキストへ切替も可能:
    PICO,seq,up_ms,temp_cC,press_Pa,altBaro_mm,altFused_mm,vel_mm_s,az_mm_s2,
         head_cdeg,roll_cdeg,pitch_cdeg,calib,vstate,elev,servo_cdeg
  スケール: 角度/温度 = 1/100, 高度/速度/加速度 = 1/1000, 気圧 = Pa, calib = 生バイト
  ※ 制御テキスト行 (RSSI= / ping / BEND / STATS) と同じ 0x0A 区切りストリーム上で
     混在する。クライアントは各行の先頭バイトが 0x01 ならバイナリ、それ以外はテキスト。

アップリンク コマンド:
  T<epoch_us> 時刻同期   P<token> ping   S 統計   R<ms> 送信周期   B<秒> スループット試験
  F1 バイナリ送信(既定)   F0 CSV 送信

使い方:
  pip install bleak        (tkinter は標準ライブラリ)
  python shizuku_telemetry_client.py

注記:
  - 受信 (notify) はペアリング不要。送信 (RX への書き込み) は LE Secure Connections +
    Numeric Comparison を要求するため、OS の Bluetooth がペアリングを促す場合がある。
    Pico のシリアルに出る 6 桁番号と OS のダイアログを照合し、Pico 側へ 'y' を入力する。
"""

import asyncio
import collections
import csv
import os
import queue
import re
import struct
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, scrolledtext, ttk

from bleak import BleakClient, BleakScanner

# ---- NUS UUID -------------------------------------------------------------
NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # notify (受信)
NUS_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # write  (送信)
DEVICE_NAME = "Shizuku UART"

# ---- BLE スループットの理論上限 -------------------------------------------
# この cyw43/btstack ペリフェラルは 1 接続イベントあたり実質 1 notify しか
# 確実に流せない (BLE_UART_DRIVER.cpp / HELLO_WORLD.cpp の flush_tx コメント参照)。
#   payload  = MTU(527) - 3            = 524 byte / event
#   実効 CI ≈ 30 ms                    → 524B / 30ms ≈ 17.5 kB/s ≈ 140 kbps
# スループット試験はこの上限に対する達成率も併記する。CI/MTU が変われば調整する。
BLE_NOTIFY_PAYLOAD = 524      # MTU - 3 [byte]
BLE_EFFECTIVE_CI_MS = 30.0    # 1 notify/event の実効接続インターバル [ms]
BLE_CEILING_KBPS = BLE_NOTIFY_PAYLOAD * 8.0 / BLE_EFFECTIVE_CI_MS  # ≈ 139.7 kbps
BLE_CEILING_KBYTES = BLE_NOTIFY_PAYLOAD / BLE_EFFECTIVE_CI_MS      # ≈ 17.5 kB/s

# ---- バイナリテレメトリ バッチフレーム (TELEMETRY_SENDER.cpp と一致) -------
# ワイヤ: [0x02 マーカー][0x0A バイトスタッフィング本体][0x0A 終端]
#   本体 = batch_header(17B) + count × batch_sample(35B) + CRC16-CCITT(2B, LE)
FRAME_MARKER_BATCH = 0x02
FRAME_ESC = 0x1B
# C 側 batch_header_t / batch_sample_t と同じ並び (リトルエンディアン, パック)。
BATCH_HDR_FMT = "<BBBIIIH"               # ver,count,flags,seq0,t0_up_ms,t0_wall_s,t0_wall_ms
BATCH_HDR_SIZE = struct.calcsize(BATCH_HDR_FMT)        # 17
BATCH_SAMPLE_FMT = "<HhIiiiiHhhBBBh"     # d_ms,temp,press,altb,altf,vel,az,head,roll,pitch,calib,vstate,elev,servo
BATCH_SAMPLE_SIZE = struct.calcsize(BATCH_SAMPLE_FMT)  # 35


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

# ---- テレメトリ列の定義 (firmware の並びと一致) ---------------------------
#   (キー, 表示名, スケール除数, 単位)。整数値を除数で割ると物理量になる。
TELEMETRY_FIELDS = [
    ("seq",        "シーケンス",      1,    ""),
    ("up_ms",      "稼働時間",        1000, "s"),   # ms -> s
    ("temp_c",     "温度",            100,  "°C"),
    ("press_hpa",  "気圧",            100,  "hPa"),  # Pa -> hPa
    ("alt_baro_m", "気圧高度",        1000, "m"),
    ("alt_fused_m","融合高度",        1000, "m"),
    ("vel_m_s",    "上下速度",        1000, "m/s"),
    ("az_m_s2",    "鉛直加速度",      1000, "m/s²"),
    ("heading",    "方位",            100,  "°"),
    ("roll",       "ロール",          100,  "°"),
    ("pitch",      "ピッチ",          100,  "°"),
    ("calib",      "較正(SGAM)",      1,    ""),     # 生バイト。後で分解表示
    ("vstate",     "上下状態",        1,    ""),
    ("elev",       "エレベータ",      1,    ""),
    ("servo_deg",  "サーボ指令",      100,  "°"),
]
VSTATE_STR = {0: "LEVEL -", 1: "ASC ▲", 2: "DESC ▼"}
ELEV_STR = {0: "NEUTRAL", 1: "UP ▲", 2: "DOWN ▼"}


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


class StripChart(tk.Canvas):
    """複数系列を時間軸で重ね描きする軽量ストリップチャート (tk Canvas のみ)。"""

    WINDOW_SEC = 30.0

    def __init__(self, master, series, ymin, ymax, ylabel, **kw):
        super().__init__(master, height=160, bg="white",
                         highlightthickness=1, highlightbackground="#ccc", **kw)
        # series: {key: color}
        self.colors = series
        self.ymin, self.ymax, self.ylabel = ymin, ymax, ylabel
        self._data = {k: collections.deque() for k in series}
        self.bind("<Configure>", lambda _e: self.redraw())

    def add(self, key: str, value: float):
        now = time.monotonic()
        dq = self._data[key]
        dq.append((now, value))
        cutoff = now - self.WINDOW_SEC
        while dq and dq[0][0] < cutoff:
            dq.popleft()

    def clear(self):
        for dq in self._data.values():
            dq.clear()
        self.redraw()

    def _autoscale(self):
        vals = [v for dq in self._data.values() for _, v in dq]
        if not vals:
            return self.ymin, self.ymax
        lo, hi = min(vals), max(vals)
        if hi - lo < 1e-6:
            lo, hi = lo - 1, hi + 1
        m = (hi - lo) * 0.1
        return lo - m, hi + m

    def _y(self, v, y0, y1, lo, hi):
        v = max(lo, min(hi, v))
        return y0 + (hi - v) / (hi - lo) * (y1 - y0)

    def redraw(self):
        self.delete("all")
        w, h = self.winfo_width(), self.winfo_height()
        if w < 60 or h < 50:
            return
        ml, mr, mt, mb = 52, 70, 10, 16
        x0, x1, y0, y1 = ml, w - mr, mt, h - mb
        lo, hi = self._autoscale()

        for i in range(5):
            yy = y0 + i / 4 * (y1 - y0)
            val = hi - i / 4 * (hi - lo)
            self.create_line(x0, yy, x1, yy, fill="#eee")
            self.create_text(x0 - 4, yy, text=f"{val:.2f}", anchor="e",
                            fill="#999", font=("Menlo", 8))
        self.create_text((x0 + x1) / 2, h - 3,
                        text=f"直近 {self.WINDOW_SEC:.0f} 秒 ({self.ylabel})",
                        anchor="s", fill="#999", font=("Menlo", 8))

        now = time.monotonic()
        t_left = now - self.WINDOW_SEC
        legend_y = y0 + 6
        for key, color in self.colors.items():
            dq = self._data[key]
            self.create_text(x1 + 6, legend_y, text=key, anchor="w",
                            fill=color, font=("Menlo", 8))
            legend_y += 14
            if len(dq) < 2:
                continue
            pts = []
            for t, v in dq:
                x = x0 + (t - t_left) / self.WINDOW_SEC * (x1 - x0)
                pts.extend((x, self._y(v, y0, y1, lo, hi)))
            self.create_line(*pts, fill=color, width=2)


class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.ui_queue: "queue.Queue" = queue.Queue()
        self.worker = BleWorker(self.ui_queue)
        self.worker.start()
        self.connected = False
        self._rx_bytes = bytearray()  # 受信生バイトの行アセンブリ

        self._ping_seq = 0
        self._pending_pings: dict[str, float] = {}

        self._tput_active = False
        self._tput_bytes = 0
        self._tput_t0 = 0.0

        self._rtt_str = "-"
        self._tput_str = "-"
        self._host_rssi = None
        self._pico_rssi = None
        self._rssi_warned = False
        self._telem_count = 0
        self._telem_t0 = None
        self._last_telem_seq = None
        self._telem_lost = 0

        # CSV 記録 (全サンプルを host 受信時刻つきで保存し、後でオフライン解析する)
        self._rec_file = None
        self._rec_writer = None
        self._rec_count = 0
        self._rec_path = None

        # BNO055 読みモード / 較正
        self._read_split = False           # False=BLK, True=2B 個別読み
        self._ffff_reject = True            # 0xFFFF 破損破棄 (既定 ON)
        self._calib_win = None             # 較正 Toplevel
        self._calib_labels = {}            # S/G/A/M ラベル
        self._calib_status_var = None
        self._calib_save_to_file = False   # CDUMP 受信時にファイル保存するか
        self._last_calib_hex = None        # 直近に保存したオフセット(44 hex)

        root.title("Shizuku 飛行テレメトリ クライアント")
        root.geometry("860x900")
        root.minsize(720, 700)

        self._build_ui()
        self.root.after(50, self._poll_queue)
        self.root.after(1000, self._poll_rssi)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # -- UI 構築 ------------------------------------------------------------
    def _build_ui(self):
        pad = dict(padx=6, pady=3)

        top = ttk.Frame(self.root)
        top.pack(fill=tk.X, **pad)
        self.scan_btn = ttk.Button(top, text="スキャン", command=self._on_scan)
        self.scan_btn.pack(side=tk.LEFT)
        self.device_var = tk.StringVar()
        self.device_box = ttk.Combobox(top, textvariable=self.device_var,
                                       state="readonly", width=40)
        self.device_box.pack(side=tk.LEFT, padx=6, fill=tk.X, expand=True)
        self.connect_btn = ttk.Button(top, text="接続",
                                      command=self._on_connect_toggle)
        self.connect_btn.pack(side=tk.LEFT)

        self.status_var = tk.StringVar(value="● 未接続")
        ttk.Label(self.root, textvariable=self.status_var,
                 foreground="gray").pack(fill=tk.X, padx=6)
        self.metrics_var = tk.StringVar(value="RTT: -   スループット: -   テレメトリ: -")
        ttk.Label(self.root, textvariable=self.metrics_var,
                 foreground="#06c").pack(fill=tk.X, padx=6)
        self.rssi_var = tk.StringVar(value="RSSI  母艦(host): -   Pico(device): -")
        ttk.Label(self.root, textvariable=self.rssi_var,
                 foreground="#c30").pack(fill=tk.X, padx=6)
        self.sync_var = tk.StringVar(value="同期: 未")
        ttk.Label(self.root, textvariable=self.sync_var,
                 foreground="gray").pack(fill=tk.X, padx=6)

        # --- テレメトリ数値パネル ---
        panel = ttk.LabelFrame(self.root, text="テレメトリ (最新値)")
        panel.pack(fill=tk.X, padx=6, pady=4)
        self.value_vars: dict[str, tk.StringVar] = {}
        cols = 4
        for idx, (key, label, _scale, unit) in enumerate(TELEMETRY_FIELDS):
            r, c = divmod(idx, cols)
            cell = ttk.Frame(panel)
            cell.grid(row=r, column=c, padx=8, pady=2, sticky="w")
            ttk.Label(cell, text=label, foreground="#666",
                     font=("Menlo", 9)).pack(anchor="w")
            v = tk.StringVar(value="-")
            self.value_vars[key] = v
            ttk.Label(cell, textvariable=v, font=("Menlo", 13, "bold")).pack(anchor="w")
        for c in range(cols):
            panel.columnconfigure(c, weight=1)

        # --- グラフ (高度 / 速度) ---
        self.alt_chart = StripChart(
            self.root, {"alt_fused": "#06c", "alt_baro": "#0a0"},
            -2, 3, "m")
        self.alt_chart.pack(fill=tk.X, padx=6, pady=2)
        self.vel_chart = StripChart(
            self.root, {"vel": "#c30", "az": "#909"}, -2, 2, "m/s, m/s²")
        self.vel_chart.pack(fill=tk.X, padx=6, pady=2)
        self.rssi_chart = StripChart(
            self.root, {"host": "#06c", "pico": "#0a0"}, -100, -30, "dBm")
        self.rssi_chart.pack(fill=tk.X, padx=6, pady=2)

        # --- ログ ---
        self.text = scrolledtext.ScrolledText(
            self.root, height=8, state=tk.DISABLED, wrap=tk.CHAR,
            font=("Menlo", 10))
        self.text.pack(fill=tk.BOTH, expand=True, **pad)
        self.text.tag_config("rx", foreground="#0a0")
        self.text.tag_config("tx", foreground="#06c")
        self.text.tag_config("log", foreground="#888")

        # --- 送信欄 / ボタン ---
        bottom = ttk.Frame(self.root)
        bottom.pack(fill=tk.X, **pad)
        self.entry = ttk.Entry(bottom)
        self.entry.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.entry.bind("<Return>", lambda _e: self._on_send())
        self.send_btn = ttk.Button(bottom, text="送信", command=self._on_send)
        self.send_btn.pack(side=tk.LEFT, padx=4)
        self.sync_btn = ttk.Button(bottom, text="時刻同期",
                                   command=self._on_sync_time)
        self.sync_btn.pack(side=tk.LEFT)
        self.ping_btn = ttk.Button(bottom, text="Ping", command=self._on_ping)
        self.ping_btn.pack(side=tk.LEFT, padx=4)
        self.tput_btn = ttk.Button(bottom, text="スループット試験",
                                   command=self._on_throughput)
        self.tput_btn.pack(side=tk.LEFT)
        ttk.Label(bottom, text="周期ms").pack(side=tk.LEFT, padx=(8, 2))
        self.rate_entry = ttk.Entry(bottom, width=5)
        self.rate_entry.insert(0, "40")
        self.rate_entry.pack(side=tk.LEFT)
        ttk.Button(bottom, text="設定", command=self._on_set_rate).pack(side=tk.LEFT, padx=2)
        # 送信フォーマット切替 (F1=バイナリ / F0=CSV)。
        ttk.Label(bottom, text="形式").pack(side=tk.LEFT, padx=(8, 2))
        self.fmt_var = tk.StringVar(value="BIN")
        self.fmt_box = ttk.Combobox(bottom, textvariable=self.fmt_var,
                                    state="readonly", width=5,
                                    values=["BIN", "CSV"])
        self.fmt_box.pack(side=tk.LEFT)
        self.fmt_box.bind("<<ComboboxSelected>>", lambda _e: self._on_set_format())
        self.rec_btn = ttk.Button(bottom, text="記録開始", command=self._toggle_record)
        self.rec_btn.pack(side=tk.LEFT, padx=4)
        # BNO055 読みモード切替 (X0=ブロック / X1=2B 個別読み)。
        self.readmode_btn = ttk.Button(bottom, text="読み:BLK",
                                       command=self._toggle_read_mode)
        self.readmode_btn.pack(side=tk.LEFT, padx=4)
        # 0xFFFF 破損バーストの破棄 ON/OFF (Y1/Y0)。既定 ON。
        self.ffff_btn = ttk.Button(bottom, text="0xFF除外:ON",
                                   command=self._toggle_ffff_reject)
        self.ffff_btn.pack(side=tk.LEFT, padx=4)
        self.calib_btn = ttk.Button(bottom, text="較正", command=self._open_calib_window)
        self.calib_btn.pack(side=tk.LEFT, padx=4)
        ttk.Button(bottom, text="クリア", command=self._clear).pack(side=tk.LEFT, padx=4)

        self._set_connected(False)

    # -- ボタンハンドラ -----------------------------------------------------
    def _on_scan(self):
        self.scan_btn.config(state=tk.DISABLED)
        self._append("log", "スキャン開始\n")
        self.worker.submit(self.worker.scan())

    def _on_connect_toggle(self):
        if self.connected:
            self.worker.submit(self.worker.disconnect())
            return
        addr = getattr(self, "_label_to_addr", {}).get(self.device_var.get())
        if not addr:
            self._append("log", "先にスキャンしてデバイスを選択してください\n")
            return
        self.connect_btn.config(state=tk.DISABLED)
        self.worker.submit(self.worker.connect(addr))

    def _on_send(self):
        msg = self.entry.get()
        if not msg:
            return
        self.worker.submit(self.worker.send((msg + "\r\n").encode("utf-8")))
        self.entry.delete(0, tk.END)

    def _on_sync_time(self):
        self.worker.submit(self.worker.send_time())

    def _schedule_auto_sync(self):
        # 接続/ペアリング完了後に時刻信号を自動送信する。ペアリング (Numeric
        # Comparison) 完了タイミングが読めないので、数回に分けて試みる (再送は
        # 同じ offset を上書きするだけで無害)。
        for delay in (1500, 4000, 8000):
            self.root.after(delay, self._auto_sync_once)

    def _auto_sync_once(self):
        if self.connected:
            self.worker.submit(self.worker.send_time())
            self._append("log", "時刻自動同期を送信 (auto)\n")

    def _on_ping(self):
        token = str(self._ping_seq)
        self._ping_seq += 1
        self._pending_pings[token] = time.perf_counter()
        self.worker.submit(self.worker.send(f"P{token}\n".encode("ascii")))
        self._append("log", f"ping P{token} 送信\n")

    def _on_throughput(self, seconds: int = 3):
        if self._tput_active:
            return
        self._tput_active = True
        self._tput_bytes = 0
        self._tput_t0 = time.perf_counter()
        self._tput_str = "測定中…"
        self._set_metrics()
        self._append("log", f"スループット試験 {seconds}s 開始 (B{seconds})\n")
        self.worker.submit(self.worker.send(f"B{seconds}\n".encode("ascii")))

    def _on_set_rate(self):
        try:
            ms = int(self.rate_entry.get())
        except ValueError:
            self._append("log", "周期は整数(ms)で入力してください\n")
            return
        self.worker.submit(self.worker.send(f"R{ms}\n".encode("ascii")))
        self._append("log", f"送信周期変更 R{ms} 送信\n")

    def _on_set_format(self):
        # BIN=F1 (バイナリ), CSV=F0 (テキスト)。
        binary = self.fmt_var.get() == "BIN"
        self.worker.submit(self.worker.send((b"F1\n" if binary else b"F0\n")))
        self._append("log", f"送信形式変更 F{int(binary)} ({self.fmt_var.get()}) 送信\n")

    # -- 受信処理 (生バイト) ------------------------------------------------
    # ダウンリンクは 0x0A 区切りで、各セグメントが
    #   - 先頭 0x02 → バイナリ・バッチテレメトリフレーム
    #   - それ以外  → ASCII 制御テキスト (PICO CSV / RSSI= / P.. / BEND / STATS)
    # の 2 種混在。先頭バイトで振り分ける。バッチは複数 notify に跨ることがあるので
    # 0x0A まで連結してから処理する (_rx_bytes に蓄積)。
    def _consume_rx(self, chunk: bytes):
        if self._tput_active:
            self._tput_bytes += len(chunk)
        self._rx_bytes += chunk
        while b"\n" in self._rx_bytes:
            seg, _, self._rx_bytes = self._rx_bytes.partition(b"\n")
            seg = seg.rstrip(b"\r")
            if not seg:
                continue
            if seg[0] == FRAME_MARKER_BATCH:
                self._on_binary_batch(seg)
            else:
                self._on_rx_line(seg.decode("utf-8", errors="replace"))

    def _on_binary_batch(self, seg: bytes):
        body = frame_unstuff(seg[1:])  # マーカーを除いて復元
        if len(body) < BATCH_HDR_SIZE + 2:
            return
        payload, crc_rx = body[:-2], body[-2:]
        if crc16_ccitt(payload) != int.from_bytes(crc_rx, "little"):
            self._telem_crc_err = getattr(self, "_telem_crc_err", 0) + 1
            return
        ver, count, flags, seq0, t0_up_ms, t0_wall_s, t0_wall_ms = struct.unpack(
            BATCH_HDR_FMT, payload[:BATCH_HDR_SIZE])
        if ver != 2:
            return
        synced = bool(flags & 0x01)
        t0_wall = (t0_wall_s + t0_wall_ms / 1000.0) if synced else None
        off = BATCH_HDR_SIZE
        last_wall = None
        for i in range(count):
            rec = payload[off:off + BATCH_SAMPLE_SIZE]
            off += BATCH_SAMPLE_SIZE
            if len(rec) < BATCH_SAMPLE_SIZE:
                break
            (d_ms, temp_cC, press_Pa, alt_baro_mm, alt_fused_mm, vel_mm_s,
             az_mm_s2, head_cdeg, roll_cdeg, pitch_cdeg, calib, vstate, elev,
             servo_cdeg) = struct.unpack(BATCH_SAMPLE_FMT, rec)
            vals = {
                "seq": seq0 + i, "up_ms": (t0_up_ms + d_ms) / 1000.0,
                "temp_c": temp_cC / 100.0, "press_hpa": press_Pa / 100.0,
                "alt_baro_m": alt_baro_mm / 1000.0, "alt_fused_m": alt_fused_mm / 1000.0,
                "vel_m_s": vel_mm_s / 1000.0, "az_m_s2": az_mm_s2 / 1000.0,
                "heading": head_cdeg / 100.0, "roll": roll_cdeg / 100.0,
                "pitch": pitch_cdeg / 100.0, "calib": calib,
                "vstate": vstate, "elev": elev, "servo_deg": servo_cdeg / 100.0,
            }
            # パネルは最後のサンプルだけ更新 (グラフ/統計は全サンプル)。
            self._update_telemetry(vals, update_panel=(i == count - 1))
            if t0_wall is not None:
                last_wall = t0_wall + d_ms / 1000.0
        # 壁時計同期済みなら絶対時刻とドリフトを表示。
        if last_wall is not None:
            drift_ms = (time.time() - last_wall) * 1000.0
            self.sync_var.set(
                f"同期: device wall={last_wall:.3f}  "
                f"ドリフト(host-device)={drift_ms:+.1f} ms")

    def _on_rx_line(self, line: str):
        if not line:
            return

        # テレメトリ行 (CSV; F0 のとき)
        if line.startswith("PICO,"):
            self._on_telemetry(line)
            return

        # Pico 側 RSSI
        m = re.match(r"RSSI=(-?\d+)", line)
        if m:
            self._pico_rssi = int(m.group(1))
            self.rssi_chart.add("pico", self._pico_rssi)
            self._set_rssi()
            return

        # ping エコー
        if re.fullmatch(r"P\d+", line):
            token = line[1:]
            t0 = self._pending_pings.pop(token, None)
            if t0 is not None:
                rtt = (time.perf_counter() - t0) * 1000.0
                self._rtt_str = f"{rtt:.1f} ms"
                self._set_metrics()
                self._append("log", f"ping P{token} RTT={rtt:.1f} ms\n")
            return

        # スループット試験終了
        if line.startswith("BEND"):
            if self._tput_active:
                elapsed = time.perf_counter() - self._tput_t0
                kbytes = (self._tput_bytes / 1024.0) / elapsed if elapsed > 0 else 0
                kbps = (self._tput_bytes * 8.0 / 1000.0) / elapsed if elapsed > 0 else 0
                # 理論上限 (1 notify/event) に対する達成率。
                eff = 100.0 * kbps / BLE_CEILING_KBPS if BLE_CEILING_KBPS > 0 else 0
                m = re.search(r"bytes=(\d+)", line)
                sent = int(m.group(1)) if m else 0
                loss = ""
                if sent > 0:
                    lost = max(0, sent - self._tput_bytes)
                    loss = f"  loss={100.0 * lost / sent:.1f}%"
                self._tput_str = (
                    f"↓{kbps:.0f} kbps ({kbytes:.1f} kB/s, 上限比 {eff:.0f}%){loss}"
                )
                self._set_metrics()
                self._append("log",
                             f"スループット試験完了: {kbps:.1f} kbps "
                             f"({kbytes:.1f} kB/s)  理論上限 {BLE_CEILING_KBPS:.0f} kbps の "
                             f"{eff:.0f}%  recv={self._tput_bytes}B sent={sent}B{loss}\n")
                self._tput_active = False
            return

        if line.startswith("STATS"):
            self._append("log", f"{line}\n")
            return

        # 較正オフセットのダンプ / 復元 ack
        if line.startswith("CDUMP"):
            self._on_calib_dump(line)
            return
        if line.startswith("CLOAD"):
            ok = "ok=1" in line
            if self._calib_status_var is not None:
                self._calib_status_var.set("復元 " + ("成功" if ok else "失敗"))
            self._append("log", line + "\n")
            return

        # それ以外はログへ
        self._append("rx", line + "\n")

    def _on_telemetry(self, line: str):
        """CSV テレメトリ行 (F0 のとき) を解析して共通の更新へ渡す。"""
        parts = line.split(",")
        # "PICO" + 15 フィールド
        if len(parts) != 1 + len(TELEMETRY_FIELDS):
            self._append("log", f"テレメトリ列数不一致 ({len(parts)}): {line}\n")
            return
        try:
            raw = [int(x) for x in parts[1:]]
        except ValueError:
            self._append("log", f"テレメトリ解析失敗: {line}\n")
            return
        vals = {}
        for (key, _label, scale, _unit), r in zip(TELEMETRY_FIELDS, raw):
            vals[key] = r / scale if scale != 1 else r
        self._update_telemetry(vals)

    def _update_telemetry(self, vals: dict, update_panel: bool = True):
        """物理量に直したテレメトリ (CSV/バイナリ共通) でパネル・グラフ・統計を更新。

        バッチ受信では複数サンプルが一度に来るので、数値パネルは最後の 1 件だけ
        (update_panel=True)、グラフと統計は全サンプルを反映する。記録中なら全サンプルを
        CSV へ書き出す。
        """
        # CSV 記録 (全サンプル。パネル更新の有無に関わらず)
        self._record_sample(vals)

        # 較正ウィンドウが開いていれば S/G/A/M ライブ更新
        if self._calib_win is not None:
            self._calib_update_status(vals.get("calib", 0))

        # 表示更新 (パネル)
        if update_panel:
            for (key, _label, _scale, unit) in TELEMETRY_FIELDS:
                v = vals[key]
                if key == "calib":
                    b = int(v)
                    s = f"S{(b >> 6) & 3} G{(b >> 4) & 3} A{(b >> 2) & 3} M{b & 3}"
                    self.value_vars[key].set(s)
                elif key == "vstate":
                    self.value_vars[key].set(VSTATE_STR.get(int(v), str(int(v))))
                elif key == "elev":
                    self.value_vars[key].set(ELEV_STR.get(int(v), str(int(v))))
                elif key in ("seq",):
                    self.value_vars[key].set(str(int(v)))
                else:
                    self.value_vars[key].set(f"{v:.3f} {unit}".strip())

        # グラフ更新
        self.alt_chart.add("alt_fused", vals["alt_fused_m"])
        self.alt_chart.add("alt_baro", vals["alt_baro_m"])
        self.vel_chart.add("vel", vals["vel_m_s"])
        self.vel_chart.add("az", vals["az_m_s2"])

        # 受信レート / パケロス
        now = time.monotonic()
        if self._telem_t0 is None:
            self._telem_t0 = now
        self._telem_count += 1
        seq = int(vals["seq"])
        if self._last_telem_seq is not None and seq > self._last_telem_seq + 1:
            self._telem_lost += seq - self._last_telem_seq - 1
        self._last_telem_seq = seq
        dt = now - self._telem_t0
        if dt >= 1.0:
            hz = self._telem_count / dt
            self._telem_str = f"{hz:.1f} Hz (lost {self._telem_lost})"
            self._telem_count = 0
            self._telem_t0 = now
            self._set_metrics()

    # -- CSV 記録 -----------------------------------------------------------
    # 受信した全テレメトリサンプルを host 受信時刻つきで CSV に追記する。バッチで
    # 複数サンプルが来ても 1 行ずつ残るので、0.06° ディザのような微小な時系列を
    # オフラインで解析できる。GUI スレッドからのみ呼ばれるのでロック不要。
    def _toggle_record(self):
        if self._rec_file is None:
            self._start_record()
        else:
            self._stop_record()

    def _start_record(self):
        path = os.path.join(os.getcwd(), time.strftime("telemetry_%Y%m%d_%H%M%S.csv"))
        try:
            f = open(path, "w", newline="")
        except OSError as e:  # noqa: BLE001
            self._append("log", f"記録ファイルを開けません: {e}\n")
            return
        self._rec_file = f
        self._rec_writer = csv.writer(f)
        self._rec_count = 0
        self._rec_path = path
        cols = ["host_unix", "host_time"] + [k for k, _l, _s, _u in TELEMETRY_FIELDS]
        self._rec_writer.writerow(cols)
        f.flush()
        self.rec_btn.config(text="記録停止")
        self._append("log", f"記録開始: {path}\n")

    def _stop_record(self):
        if self._rec_file is not None:
            try:
                self._rec_file.flush()
                self._rec_file.close()
            except OSError:
                pass
            self._append("log",
                         f"記録停止: {self._rec_count} サンプル → {self._rec_path}\n")
        self._rec_file = None
        self._rec_writer = None
        if hasattr(self, "rec_btn"):
            self.rec_btn.config(text="記録開始")

    def _record_sample(self, vals: dict):
        if self._rec_writer is None:
            return
        now = time.time()
        ms = int((now % 1) * 1000)
        row = [f"{now:.6f}", time.strftime("%H:%M:%S", time.localtime(now)) + f".{ms:03d}"]
        row += [vals.get(k) for k, _l, _s, _u in TELEMETRY_FIELDS]
        try:
            self._rec_writer.writerow(row)
            self._rec_count += 1
            if self._rec_count % 50 == 0:  # 定期 flush でクラッシュ時の取りこぼしを抑える
                self._rec_file.flush()
        except (OSError, ValueError) as e:  # noqa: BLE001
            self._append("log", f"記録書き込み失敗: {e}\n")
            self._stop_record()

    # -- BNO055 読みモード / 較正 -------------------------------------------
    def _toggle_read_mode(self):
        self._read_split = not self._read_split
        cmd = b"X1\n" if self._read_split else b"X0\n"
        self.worker.submit(self.worker.send(cmd))
        self.readmode_btn.config(text="読み:2B" if self._read_split else "読み:BLK")
        self._append("log",
                     f"BNO読みモード -> {'2B個別' if self._read_split else 'ブロック'}\n")

    def _toggle_ffff_reject(self):
        self._ffff_reject = not self._ffff_reject
        self.worker.submit(self.worker.send(b"Y1\n" if self._ffff_reject else b"Y0\n"))
        self.ffff_btn.config(text="0xFF除外:ON" if self._ffff_reject else "0xFF除外:OFF")
        self._append("log",
                     f"0xFFFF破損破棄 -> {'ON' if self._ffff_reject else 'OFF(素通し)'}\n")

    def _open_calib_window(self):
        if self._calib_win is not None and self._calib_win.winfo_exists():
            self._calib_win.lift()
            return
        win = tk.Toplevel(self.root)
        win.title("BNO055 較正")
        win.geometry("440x380")
        self._calib_win = win
        self._calib_labels = {}

        ttk.Label(win, text="較正ステータス (各 0→3、3 で完了)",
                  font=("Menlo", 11, "bold")).pack(pady=(10, 4))
        row = ttk.Frame(win)
        row.pack(pady=4)
        for key, name in [("S", "SYS"), ("G", "GYR"), ("A", "ACC"), ("M", "MAG")]:
            cell = ttk.Frame(row)
            cell.pack(side=tk.LEFT, padx=12)
            ttk.Label(cell, text=name, foreground="#666").pack()
            lbl = ttk.Label(cell, text="-", font=("Menlo", 22, "bold"))
            lbl.pack()
            self._calib_labels[key] = lbl

        guide = ("手順 (動かしながら上の数字が 3 になるのを待つ):\n"
                 "  GYR : 数秒間そのまま静止して置く\n"
                 "  ACC : ゆっくり 6 面 (各軸 ±) で数秒ずつ静止\n"
                 "  MAG : 空中で 8 の字を描く\n\n"
                 "全部 3 になったら『保存』で書き出し。\n"
                 "次回起動後は『復元』でそのプロファイルを即適用。")
        ttk.Label(win, text=guide, justify=tk.LEFT,
                  font=("Menlo", 9)).pack(pady=8, padx=14, anchor="w")

        btns = ttk.Frame(win)
        btns.pack(pady=6)
        ttk.Button(btns, text="オフセット保存(ファイル)",
                   command=self._on_calib_save).pack(side=tk.LEFT, padx=4)
        ttk.Button(btns, text="オフセット復元(ファイル)",
                   command=self._on_calib_restore).pack(side=tk.LEFT, padx=4)

        self._calib_status_var = tk.StringVar(value="")
        ttk.Label(win, textvariable=self._calib_status_var,
                  foreground="#06c", wraplength=400).pack(pady=4)

    def _calib_update_status(self, calib_byte: int):
        if self._calib_win is None or not self._calib_win.winfo_exists():
            return
        b = int(calib_byte)
        vals = {"S": (b >> 6) & 3, "G": (b >> 4) & 3, "A": (b >> 2) & 3, "M": b & 3}
        for k, val in vals.items():
            lbl = self._calib_labels.get(k)
            if lbl is not None:
                lbl.config(text=str(val), foreground="#0a0" if val == 3 else "#c30")

    def _on_calib_save(self):
        if not self.connected:
            self._append("log", "未接続のため較正保存できません\n")
            return
        self._calib_save_to_file = True
        self.worker.submit(self.worker.send(b"CS\n"))
        if self._calib_status_var is not None:
            self._calib_status_var.set("保存要求を送信… (CDUMP 待ち)")
        self._append("log", "較正オフセット保存要求 CS 送信\n")

    def _on_calib_restore(self):
        if not self.connected:
            self._append("log", "未接続のため較正復元できません\n")
            return
        path = filedialog.askopenfilename(
            filetypes=[("calib hex", "*.hex"), ("all", "*.*")])
        if not path:
            return
        try:
            with open(path) as f:
                hexstr = "".join(f.read().split())
        except OSError as e:  # noqa: BLE001
            self._append("log", f"較正ファイル読込失敗: {e}\n")
            return
        if len(hexstr) != 44 or any(c not in "0123456789abcdefABCDEF" for c in hexstr):
            messagebox.showerror("較正復元", "ファイル形式が不正です (44 hex 文字が必要)")
            return
        self.worker.submit(self.worker.send(f"CL{hexstr}\n".encode("ascii")))
        if self._calib_status_var is not None:
            self._calib_status_var.set("復元要求を送信… (CLOAD 待ち)")
        self._append("log", f"較正オフセット復元 CL 送信 ({path})\n")

    def _on_calib_dump(self, line: str):
        """CDUMP <44hex> ok=1 を受けてファイル保存。"""
        parts = line.split()
        hexstr = parts[1] if len(parts) >= 2 else ""
        ok = "ok=1" in line
        self._last_calib_hex = hexstr if ok else None
        msg = f"CDUMP ok={int(ok)} len={len(hexstr)}"
        if ok and self._calib_save_to_file and len(hexstr) == 44:
            path = filedialog.asksaveasfilename(
                defaultextension=".hex", initialfile="bno055_calib.hex",
                filetypes=[("calib hex", "*.hex"), ("all", "*.*")])
            if path:
                try:
                    with open(path, "w") as f:
                        f.write(hexstr + "\n")
                    msg += f" -> {path}"
                    if self._calib_status_var is not None:
                        self._calib_status_var.set(f"保存しました: {path}")
                except OSError as e:  # noqa: BLE001
                    msg += f" 保存失敗:{e}"
        elif not ok and self._calib_status_var is not None:
            self._calib_status_var.set("保存失敗 (ok=0)")
        self._calib_save_to_file = False
        self._append("log", msg + "\n")

    # -- メトリクス表示 -----------------------------------------------------
    def _set_metrics(self):
        telem = getattr(self, "_telem_str", "-")
        self.metrics_var.set(
            f"RTT: {self._rtt_str}   スループット: {self._tput_str}   テレメトリ: {telem}")

    def _set_rssi(self):
        h = f"{self._host_rssi} dBm" if self._host_rssi is not None else "-"
        p = f"{self._pico_rssi} dBm" if self._pico_rssi is not None else "-"
        self.rssi_var.set(f"RSSI  母艦(host): {h}   Pico(device): {p}")

    def _poll_rssi(self):
        if self.connected:
            self.worker.submit(self.worker.read_rssi())
        self.root.after(1000, self._poll_rssi)

    def _clear(self):
        self.text.config(state=tk.NORMAL)
        self.text.delete("1.0", tk.END)
        self.text.config(state=tk.DISABLED)

    # -- worker キュー処理 --------------------------------------------------
    def _poll_queue(self):
        try:
            while True:
                kind, payload = self.ui_queue.get_nowait()
                self._handle(kind, payload)
        except queue.Empty:
            pass
        # チャートはまとめて再描画 (描画コストを抑える)
        if self.connected:
            self.alt_chart.redraw()
            self.vel_chart.redraw()
            self.rssi_chart.redraw()
        self.root.after(100, self._poll_queue)

    def _handle(self, kind: str, payload: dict):
        if kind == "log":
            self._append("log", payload["text"])
        elif kind == "rx":
            self._consume_rx(payload["data"])
        elif kind == "tx":
            self._append("tx", "» " + payload["data"])
        elif kind == "scan_done":
            self._fill_devices(payload["items"])
            self.scan_btn.config(state=tk.NORMAL)
        elif kind == "rssi_host":
            val = payload.get("value")
            if val is not None:
                self._host_rssi = val
                self.rssi_chart.add("host", val)
                self._set_rssi()
            elif not self._rssi_warned:
                self._rssi_warned = True
                self._append("log",
                             f"母艦側 RSSI 取得不可 (bleak backend 未対応): "
                             f"{payload.get('error', '')}\n")
        elif kind == "connected":
            self._set_connected(True, payload["address"])
            self._schedule_auto_sync()  # ペアリング後 ~1.5s で時刻自動同期
        elif kind == "disconnected":
            self._set_connected(False)

    def _fill_devices(self, items):
        self._label_to_addr = {lbl: addr for lbl, addr in items}
        labels = list(self._label_to_addr.keys())
        self.device_box["values"] = labels
        if labels and not self.device_var.get():
            self.device_var.set(labels[0])

    def _set_connected(self, connected: bool, address: str = ""):
        self.connected = connected
        self.connect_btn.config(state=tk.NORMAL)
        self._host_rssi = None
        self._pico_rssi = None
        self._rssi_warned = False
        self._set_rssi()
        state = tk.NORMAL if connected else tk.DISABLED
        for b in (self.send_btn, self.sync_btn, self.ping_btn, self.tput_btn):
            b.config(state=state)
        self.fmt_box.config(state="readonly" if connected else tk.DISABLED)
        self.entry.config(state=state)
        if connected:
            self.status_var.set(f"● 接続中  {address}")
            self.connect_btn.config(text="切断")
        else:
            self.status_var.set("● 未接続")
            self.connect_btn.config(text="接続")
            self.sync_var.set("同期: 未")
            self._tput_active = False
            self._pending_pings.clear()
            self._rx_bytes = bytearray()
            self._last_telem_seq = None
            self.alt_chart.clear()
            self.vel_chart.clear()
            self.rssi_chart.clear()

    def _append(self, tag: str, text: str):
        self.text.config(state=tk.NORMAL)
        if tag != "rx":
            self.text.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] ", "log")
        self.text.insert(tk.END, text, tag)
        self.text.see(tk.END)
        self.text.config(state=tk.DISABLED)

    def _on_close(self):
        self._stop_record()
        self.worker.stop()
        self.root.destroy()


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
