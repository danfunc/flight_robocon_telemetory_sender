#!/usr/bin/env python3
"""Shizuku BLE UART GUI クライアント

Pico (pico2_w) 上の Shizuku OS が公開する Nordic UART Service (NUS) に接続し、
TX characteristic の notify を受信表示し、RX characteristic へテキストを送信する
GUI クライアント。

通信: bleak (BLE, クロスプラットフォーム)
GUI : tkinter (Python 標準ライブラリ)

NUS / Shizuku 側の定数 (BLE_UART_DRIVER.cpp / ble_uart.gatt と一致):
  デバイス名        : "Shizuku UART"
  Service           : 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
  TX (Peri->Cent)   : 6E400003-... (Notify)   <- ここを購読して受信
  RX (Cent->Peri)   : 6E400002-... (Write)    <- ここへ書いて送信

使い方:
  pip install bleak      (tkinter は標準ライブラリ)
  python tools/ble_uart_gui.py

備考:
  - 受信 (Hello, World) はペアリング不要。
  - 送信 (RX への書き込み) はファーム側が LE Secure Connections + Numeric
    Comparison を要求するため、OS の Bluetooth スタックがペアリングを促す場合が
    あります。Pico のシリアルに出る6桁番号と OS のダイアログを照合し、Pico 側へ
    'y' を入力してください。
"""

import asyncio
import collections
import queue
import re
import sys
import threading
import time
import tkinter as tk
from tkinter import scrolledtext, ttk

from bleak import BleakClient, BleakScanner

# ---- NUS UUID -------------------------------------------------------------
NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # notify (受信)
NUS_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # write  (送信)
DEVICE_NAME = "Shizuku UART"


class BleWorker:
    """専用スレッドで asyncio ループを回し、bleak を駆動する。

    GUI スレッド -> worker : submit() でコルーチンを投入 (run_coroutine_threadsafe)
    worker -> GUI スレッド  : ui_queue へ (kind, payload) を put し、GUI が after で取得
    """

    def __init__(self, ui_queue: "queue.Queue"):
        self.ui_queue = ui_queue
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.client: BleakClient | None = None
        self.devices: dict[str, object] = {}  # address -> BLEDevice

    # -- ライフサイクル -----------------------------------------------------
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
            fut = self.submit(_cleanup())
            fut.result(timeout=5)
        except Exception:
            pass
        self.loop.call_soon_threadsafe(self.loop.stop)

    # -- GUI へ通知 ---------------------------------------------------------
    def post(self, kind: str, **payload):
        self.ui_queue.put((kind, payload))

    def log(self, msg: str):
        self.post("log", text=msg)

    # -- BLE 操作 (コルーチン) ---------------------------------------------
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
            label = f"{name}  [{d.address}]"
            # Shizuku を先頭に並べたいので印を付ける
            items.append((d.name == DEVICE_NAME, label, d.address))
        items.sort(key=lambda t: (not t[0], t[1]))
        self.log(f"{len(items)} 台検出")
        self.post("scan_done", items=[(lbl, addr) for _, lbl, addr in items])

    async def connect(self, address: str):
        if self.client and self.client.is_connected:
            await self.client.disconnect()

        self.log(f"接続中… {address}")

        def _on_disconnect(_client):
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
            text = bytes(data).decode("utf-8", errors="replace")
            self.post("rx", data=text)

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
            # RX は WRITE / WRITE_WITHOUT_RESPONSE。response=False で投げる。
            await self.client.write_gatt_char(NUS_RX_UUID, data, response=False)
            self.post("tx", data=data.decode("utf-8", errors="replace"))
        except Exception as e:  # noqa: BLE001
            self.log(f"送信失敗: {e} (ペアリングが必要かもしれません)")

    async def send_time(self):
        """ホストの現在時刻 (Unix エポック us) を "T<epoch_us>\\n" で送り、
        Pico 側の RTC を同調させる。RX 書き込みなのでペアリングが必要。"""
        if not (self.client and self.client.is_connected):
            self.log("未接続のため同期できません")
            return
        epoch_us = int(time.time() * 1_000_000)
        payload = f"T{epoch_us}\n".encode("ascii")
        try:
            await self.client.write_gatt_char(NUS_RX_UUID, payload, response=False)
            self.log(f"時刻同期送信: epoch_us={epoch_us}")
        except Exception as e:  # noqa: BLE001
            self.log(f"同期失敗: {e} (ペアリングが必要かもしれません)")

    async def read_rssi(self):
        """母艦 (ホスト) 側の無線が測る接続 RSSI を取得して GUI へ送る。

        bleak の公開 API には接続中 RSSI が無いため backend の get_rssi() を使う
        (macOS/CoreBluetooth では CBPeripheral.readRSSI 経由で実装済み)。WinRT
        (Windows) / BlueZ (Linux) には OS 側に相当 API が無いので unsupported=True
        を付けて返し、GUI 側で表示を「非対応」にしてポーリングを止めてもらう。"""
        if not (self.client and self.client.is_connected):
            return
        getter = getattr(self.client._backend, "get_rssi", None)
        if getter is None:
            self.post(
                "rssi_host", value=None, unsupported=True,
                error=f"{sys.platform} の bleak backend に接続中 RSSI 取得 API 無し")
            return
        try:
            rssi = await getter()
            self.post("rssi_host", value=int(rssi))
        except Exception as e:  # noqa: BLE001
            self.post("rssi_host", value=None, error=str(e))

    async def disconnect(self):
        if self.client and self.client.is_connected:
            await self.client.disconnect()


class RssiGraph(tk.Canvas):
    """RSSI を時間軸で重ね描きする軽量グラフ (matplotlib 非依存・tk Canvas のみ)。

    2 系列を描く:
      host (母艦側測定) … 青  /  pico (デバイス側測定) … 緑
    各サンプルは (monotonic 時刻, dBm) で保持し、直近 WINDOW_SEC 秒だけ表示する。
    """

    WINDOW_SEC = 60.0
    RSSI_MAX = -30   # グラフ上端 (電波が強い)
    RSSI_MIN = -100  # グラフ下端 (電波が弱い)
    COLORS = {"host": "#06c", "pico": "#0a0"}

    def __init__(self, master, **kw):
        super().__init__(
            master, height=170, bg="white",
            highlightthickness=1, highlightbackground="#ccc", **kw,
        )
        self._series = {
            "host": collections.deque(),
            "pico": collections.deque(),
        }
        self.bind("<Configure>", lambda _e: self.redraw())

    def add(self, source: str, value: int):
        now = time.monotonic()
        dq = self._series[source]
        dq.append((now, value))
        cutoff = now - self.WINDOW_SEC
        while dq and dq[0][0] < cutoff:
            dq.popleft()
        self.redraw()

    def clear(self):
        for dq in self._series.values():
            dq.clear()
        self.redraw()

    def _y(self, dbm, y0, y1):
        dbm = max(self.RSSI_MIN, min(self.RSSI_MAX, dbm))
        frac = (self.RSSI_MAX - dbm) / (self.RSSI_MAX - self.RSSI_MIN)
        return y0 + frac * (y1 - y0)

    def redraw(self):
        self.delete("all")
        w, h = self.winfo_width(), self.winfo_height()
        if w < 40 or h < 40:
            return
        ml, mr, mt, mb = 44, 10, 10, 16  # 余白 (左に dBm 目盛り、下に軸ラベル)
        x0, x1 = ml, w - mr
        y0, y1 = mt, h - mb

        # 横グリッド + dBm 目盛り
        for dbm in range(self.RSSI_MIN, self.RSSI_MAX + 1, 10):
            y = self._y(dbm, y0, y1)
            self.create_line(x0, y, x1, y, fill="#eee")
            self.create_text(
                x0 - 4, y, text=f"{dbm}", anchor="e",
                fill="#999", font=("Menlo", 8),
            )
        self.create_text(
            (x0 + x1) / 2, h - 3,
            text=f"直近 {self.WINDOW_SEC:.0f} 秒 (dBm)", anchor="s",
            fill="#999", font=("Menlo", 8),
        )

        now = time.monotonic()
        t_left = now - self.WINDOW_SEC
        for src, dq in self._series.items():
            if not dq:
                continue
            pts = []
            for t, v in dq:
                x = x0 + (t - t_left) / self.WINDOW_SEC * (x1 - x0)
                pts.extend((x, self._y(v, y0, y1)))
            if len(pts) >= 4:
                self.create_line(*pts, fill=self.COLORS[src], width=2)
            lx, ly = pts[-2], pts[-1]  # 最新点のマーカー
            self.create_oval(
                lx - 3, ly - 3, lx + 3, ly + 3,
                fill=self.COLORS[src], outline="",
            )


class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.ui_queue: "queue.Queue" = queue.Queue()
        self.worker = BleWorker(self.ui_queue)
        self.worker.start()
        self.connected = False
        self._rx_buf = ""  # 受信行アセンブリ (wall= / コマンド応答抽出用)

        # ping 計測
        self._ping_seq = 0
        self._pending_pings: dict[str, float] = {}  # token -> 送信時刻

        # スループット計測 (ダウンリンク)
        self._tput_active = False
        self._tput_bytes = 0
        self._tput_t0 = 0.0

        # メトリクス表示 (RTT / スループット) の最新値
        self._rtt_str = "-"
        self._tput_str = "-"

        # RSSI (母艦=host 側測定 / Pico=device 側測定) の最新値。
        self._host_rssi = None
        self._pico_rssi = None
        self._rssi_warned = False  # 母艦側 RSSI 取得不可の警告は一度だけ
        # OS の bleak backend が接続中 RSSI 取得に非対応 (Windows/Linux)。
        # 一度検出したら以後ポーリングしない (プラットフォームは変わらないため)。
        self._host_rssi_unsupported = False

        root.title("Shizuku BLE UART クライアント")
        root.geometry("700x760")
        root.minsize(560, 560)

        self._build_ui()
        self.root.after(50, self._poll_queue)
        self.root.after(1000, self._poll_rssi)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # -- UI 構築 ------------------------------------------------------------
    def _build_ui(self):
        pad = dict(padx=6, pady=4)

        top = ttk.Frame(self.root)
        top.pack(fill=tk.X, **pad)

        self.scan_btn = ttk.Button(top, text="スキャン", command=self._on_scan)
        self.scan_btn.pack(side=tk.LEFT)

        self.device_var = tk.StringVar()
        self.device_box = ttk.Combobox(
            top, textvariable=self.device_var, state="readonly", width=44
        )
        self.device_box.pack(side=tk.LEFT, padx=6, fill=tk.X, expand=True)

        self.connect_btn = ttk.Button(
            top, text="接続", command=self._on_connect_toggle
        )
        self.connect_btn.pack(side=tk.LEFT)

        self.status_var = tk.StringVar(value="● 未接続")
        status = ttk.Label(self.root, textvariable=self.status_var, foreground="gray")
        status.pack(fill=tk.X, padx=6)

        self.sync_var = tk.StringVar(value="同期: 未")
        ttk.Label(self.root, textvariable=self.sync_var, foreground="gray").pack(
            fill=tk.X, padx=6
        )

        self.metrics_var = tk.StringVar(value="RTT: -    スループット: -")
        ttk.Label(
            self.root, textvariable=self.metrics_var, foreground="#06c"
        ).pack(fill=tk.X, padx=6)

        # RSSI 表示 (数値) + グラフ。母艦=青、Pico=緑 でグラフと色を合わせる。
        self.rssi_var = tk.StringVar(value="RSSI  母艦(host): -    Pico(device): -")
        ttk.Label(
            self.root, textvariable=self.rssi_var, foreground="#c30"
        ).pack(fill=tk.X, padx=6)
        self.rssi_graph = RssiGraph(self.root)
        self.rssi_graph.pack(fill=tk.X, padx=6, pady=(2, 4))

        # 受信/ログ表示
        self.text = scrolledtext.ScrolledText(
            self.root, height=18, state=tk.DISABLED, wrap=tk.CHAR,
            font=("Menlo", 11),
        )
        self.text.pack(fill=tk.BOTH, expand=True, **pad)
        self.text.tag_config("rx", foreground="#0a0")
        self.text.tag_config("tx", foreground="#06c")
        self.text.tag_config("log", foreground="#888")

        # 送信欄
        bottom = ttk.Frame(self.root)
        bottom.pack(fill=tk.X, **pad)

        self.entry = ttk.Entry(bottom)
        self.entry.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.entry.bind("<Return>", lambda _e: self._on_send())

        self.newline_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(bottom, text="改行付与", variable=self.newline_var).pack(
            side=tk.LEFT, padx=6
        )

        self.send_btn = ttk.Button(bottom, text="送信", command=self._on_send)
        self.send_btn.pack(side=tk.LEFT)

        self.sync_btn = ttk.Button(
            bottom, text="時刻同期", command=self._on_sync_time
        )
        self.sync_btn.pack(side=tk.LEFT, padx=6)

        self.ping_btn = ttk.Button(bottom, text="Ping", command=self._on_ping)
        self.ping_btn.pack(side=tk.LEFT)

        self.tput_btn = ttk.Button(
            bottom, text="スループット試験", command=self._on_throughput
        )
        self.tput_btn.pack(side=tk.LEFT, padx=6)

        ttk.Button(bottom, text="クリア", command=self._clear).pack(
            side=tk.LEFT, padx=6
        )

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
        label = self.device_var.get()
        addr = getattr(self, "_label_to_addr", {}).get(label)
        if not addr:
            self._append("log", "先にスキャンしてデバイスを選択してください\n")
            return
        self.connect_btn.config(state=tk.DISABLED)
        self.worker.submit(self.worker.connect(addr))

    def _on_send(self):
        msg = self.entry.get()
        if not msg:
            return
        if self.newline_var.get():
            msg = msg + "\r\n"
        self.worker.submit(self.worker.send(msg.encode("utf-8")))
        self.entry.delete(0, tk.END)

    def _on_sync_time(self):
        self.worker.submit(self.worker.send_time())

    def _set_metrics(self):
        self.metrics_var.set(
            f"RTT: {self._rtt_str}    スループット: {self._tput_str}"
        )

    def _set_rssi(self):
        if self._host_rssi is not None:
            h = f"{self._host_rssi} dBm"
        elif self._host_rssi_unsupported:
            h = "非対応(このOS)"
        else:
            h = "-"
        p = f"{self._pico_rssi} dBm" if self._pico_rssi is not None else "-"
        self.rssi_var.set(f"RSSI  母艦(host): {h}    Pico(device): {p}")

    def _poll_rssi(self):
        # 接続中は母艦側 RSSI を周期取得 (Pico 側は notify で勝手に届く)。
        if self.connected and not self._host_rssi_unsupported:
            self.worker.submit(self.worker.read_rssi())
        self.root.after(1000, self._poll_rssi)

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

    def _consume_rx(self, chunk: str):
        """受信ストリームを行に組み立て、"wall=<sec>.<us>" を見つけたら
        ホスト時刻との差(ドリフト)を表示する。"""
        self._rx_buf += chunk
        while "\n" in self._rx_buf:
            line, self._rx_buf = self._rx_buf.split("\n", 1)
            line = line.rstrip("\r")
            self._on_rx_line(line)

    def _on_rx_line(self, line: str):
        # Pico 側が測った接続 RSSI: "RSSI=<dBm>"
        m = re.match(r"RSSI=(-?\d+)", line)
        if m:
            self._pico_rssi = int(m.group(1))
            self.rssi_graph.add("pico", self._pico_rssi)
            self._set_rssi()
            return

        # ping エコー: "P<token>"
        if line.startswith("P"):
            token = line[1:].strip()
            t0 = self._pending_pings.pop(token, None)
            if t0 is not None:
                rtt_ms = (time.perf_counter() - t0) * 1000.0
                self._rtt_str = f"{rtt_ms:.1f} ms"
                self._set_metrics()
                self._append("log", f"ping P{token} RTT={rtt_ms:.1f} ms\n")
            return

        # スループット試験終了: "BEND seq=<n> bytes=<m>"
        if line.startswith("BEND"):
            if self._tput_active:
                elapsed = time.perf_counter() - self._tput_t0
                kbps = (self._tput_bytes / 1024.0) / elapsed if elapsed > 0 else 0
                m = re.search(r"seq=(\d+)\s+bytes=(\d+)", line)
                sent = int(m.group(2)) if m else 0
                loss = ""
                if sent > 0:
                    lost = max(0, sent - self._tput_bytes)
                    loss = f"  loss={100.0 * lost / sent:.1f}%"
                self._tput_str = (
                    f"↓{kbps:.1f} kB/s ({self._tput_bytes}B/{elapsed:.2f}s){loss}"
                )
                self._set_metrics()
                self._append(
                    "log",
                    f"スループット試験完了: {kbps:.1f} kB/s "
                    f"recv={self._tput_bytes}B sent={sent}B{loss}\n",
                )
                self._tput_active = False
            return

        # 統計返信: "STATS rx=<n>"
        if line.startswith("STATS"):
            self._append("log", f"{line}\n")
            return

        # ウォールクロック (RTC 同調) のドリフト表示
        m = re.search(r"wall=(\d+)\.(\d+)", line)
        if m:
            device_wall = int(m.group(1)) + int(m.group(2)) / 1_000_000
            drift_ms = (time.time() - device_wall) * 1000.0
            self.sync_var.set(
                f"同期: device wall={device_wall:.3f}  "
                f"ドリフト(host-device)={drift_ms:+.1f} ms"
            )

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
        self.root.after(50, self._poll_queue)

    def _handle(self, kind: str, payload: dict):
        if kind == "log":
            self._append("log", payload["text"] + "\n")
        elif kind == "rx":
            data = payload["data"]
            if self._tput_active:
                self._tput_bytes += len(data.encode("utf-8", errors="replace"))
            self._append("rx", data)
            self._consume_rx(data)
        elif kind == "tx":
            self._append("tx", "» " + payload["data"])
        elif kind == "scan_done":
            self._fill_devices(payload["items"])
            self.scan_btn.config(state=tk.NORMAL)
        elif kind == "rssi_host":
            val = payload.get("value")
            if val is not None:
                self._host_rssi = val
                self.rssi_graph.add("host", val)
                self._set_rssi()
            elif payload.get("unsupported"):
                if not self._host_rssi_unsupported:
                    self._host_rssi_unsupported = True
                    self._set_rssi()
                    self._append(
                        "log",
                        f"母艦側 RSSI はこの OS では取得できません "
                        f"(Pico 側 RSSI で電波状況を確認してください): "
                        f"{payload.get('error', '')}\n",
                    )
            elif not self._rssi_warned:
                self._rssi_warned = True
                self._append(
                    "log",
                    f"母艦側 RSSI 取得不可 (このプラットフォームの bleak backend は "
                    f"未対応): {payload.get('error', '')}\n",
                )
        elif kind == "connected":
            self._set_connected(True, payload["address"])
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
        # 接続状態が変わったら RSSI 表示とグラフは仕切り直す。
        self._host_rssi = None
        self._pico_rssi = None
        self._rssi_warned = False
        self._set_rssi()
        self.rssi_graph.clear()
        if connected:
            self.status_var.set(f"● 接続中  {address}")
            self.connect_btn.config(text="切断")
            self.send_btn.config(state=tk.NORMAL)
            self.sync_btn.config(state=tk.NORMAL)
            self.ping_btn.config(state=tk.NORMAL)
            self.tput_btn.config(state=tk.NORMAL)
            self.entry.config(state=tk.NORMAL)
        else:
            self.status_var.set("● 未接続")
            self.connect_btn.config(text="接続")
            self.send_btn.config(state=tk.DISABLED)
            self.sync_btn.config(state=tk.DISABLED)
            self.ping_btn.config(state=tk.DISABLED)
            self.tput_btn.config(state=tk.DISABLED)
            self.entry.config(state=tk.DISABLED)
            self.sync_var.set("同期: 未")
            self._tput_active = False
            self._pending_pings.clear()
            self._rx_buf = ""

    def _append(self, tag: str, text: str):
        self.text.config(state=tk.NORMAL)
        if tag != "rx":
            ts = time.strftime("%H:%M:%S")
            self.text.insert(tk.END, f"[{ts}] ", "log")
        self.text.insert(tk.END, text, tag)
        self.text.see(tk.END)
        self.text.config(state=tk.DISABLED)

    def _on_close(self):
        self.worker.stop()
        self.root.destroy()


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
