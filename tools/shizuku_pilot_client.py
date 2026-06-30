#!/usr/bin/env python3
"""Shizuku 操縦者向け 計器クライアント (グラスコックピット)

操縦者がひと目で機体姿勢を把握するための計器表示に特化したクライアント。
中央に人工水平儀 (アティチュード・インジケータ: ロール/ピッチ + 方位テープ)、
周囲に高度・上下速度・方位・リンク状態などのデジタル計器を並べる。

エンジニア向けのグラフ/デバッグ表示は shizuku_telemetry_client.py の方。
両者はプロトコル層 shizuku_link.py を共有する。

  pip install bleak     (tkinter は標準ライブラリ)
  python shizuku_pilot_client.py
"""

import math
import os
import queue
import subprocess
import sys
import time
import tkinter as tk
from tkinter import ttk

from shizuku_link import BleWorker, TelemetryRx, VSTATE_STR

# 計器の符号調整 (機体/座標系に合わせて反転したいとき用)。
ROLL_SIGN = -1.0   # 人工水平儀の地平線はロールと逆回転
PITCH_SIGN = 1.0   # ピッチ↑で地平線が下がる

# 配色
C_SKY = "#2f7fd0"
C_GND = "#7a5230"
C_LINE = "#ffffff"
C_BEZEL = "#0a0a0a"
C_SYMBOL = "#ffd400"
C_READOUT = "#21e07a"
C_LABEL = "#8a8a8a"
C_WARN = "#ff5252"


class AttitudeIndicator(tk.Canvas):
    """人工水平儀。roll/pitch で地平線・ピッチラダー、上端にバンク目盛、下端に方位テープ。"""

    def __init__(self, master, size=440, **kw):
        super().__init__(master, width=size, height=size, bg=C_BEZEL,
                         highlightthickness=0, **kw)
        self.size = size
        self.roll = 0.0
        self.pitch = 0.0
        self.heading = 0.0
        self.valid = False
        self.bind("<Configure>", lambda _e: self.redraw())

    def update_attitude(self, roll, pitch, heading, valid=True):
        self.roll, self.pitch, self.heading, self.valid = roll, pitch, heading, valid

    # 点 (px,py) を中心 (cx,cy) まわりに ang[deg] 回転。
    @staticmethod
    def _rot(px, py, cx, cy, ang):
        a = math.radians(ang)
        ca, sa = math.cos(a), math.sin(a)
        dx, dy = px - cx, py - cy
        return (cx + dx * ca - dy * sa, cy + dx * sa + dy * ca)

    def redraw(self):
        self.delete("all")
        w = self.winfo_width() or self.size
        h = self.winfo_height() or self.size
        S = min(w, h)
        cx, cy = w / 2.0, h / 2.0
        ppd = S * 0.014                 # ピッチ pixels/deg
        ang = ROLL_SIGN * self.roll     # 地平線の回転
        horizon_y = cy + PITCH_SIGN * self.pitch * ppd
        R = S * 1.7

        def rp(px, py):
            return self._rot(px, py, cx, cy, ang)

        # --- 空と地面 (回転した大きな矩形) ---
        sky = [rp(cx - R, cy - R), rp(cx + R, cy - R),
               rp(cx + R, horizon_y), rp(cx - R, horizon_y)]
        gnd = [rp(cx - R, horizon_y), rp(cx + R, horizon_y),
               rp(cx + R, cy + R), rp(cx - R, cy + R)]
        self.create_polygon(*[c for pt in sky for c in pt], fill=C_SKY, outline="")
        self.create_polygon(*[c for pt in gnd for c in pt], fill=C_GND, outline="")
        # 地平線
        p1, p2 = rp(cx - R, horizon_y), rp(cx + R, horizon_y)
        self.create_line(*p1, *p2, fill=C_LINE, width=2)

        # --- ピッチラダー ---
        for p in range(-30, 31, 10):
            if p == 0:
                continue
            ry = cy + PITCH_SIGN * (self.pitch - p) * ppd
            half = S * (0.11 if p % 20 == 0 else 0.07)
            a1, a2 = rp(cx - half, ry), rp(cx + half, ry)
            self.create_line(*a1, *a2, fill=C_LINE, width=1)
            txt = str(abs(p))
            le = rp(cx - half - S * 0.02, ry)
            re = rp(cx + half + S * 0.02, ry)
            self.create_text(*le, text=txt, fill=C_LINE, font=("Menlo", int(S * 0.025)))
            self.create_text(*re, text=txt, fill=C_LINE, font=("Menlo", int(S * 0.025)))

        # --- バンク目盛 (固定) + ポインタ (ロールで移動) ---
        rr = S * 0.44
        for b in (-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60):
            aa = math.radians(-90 + b)
            x0 = cx + rr * math.cos(aa)
            y0 = cy + rr * math.sin(aa)
            tl = S * (0.045 if b % 30 == 0 else 0.025)
            x1 = cx + (rr - tl) * math.cos(aa)
            y1 = cy + (rr - tl) * math.sin(aa)
            self.create_line(x0, y0, x1, y1, fill=C_LINE, width=2 if b == 0 else 1)
        # ロールポインタ (三角形)
        pa = math.radians(-90 + self.roll)
        tip = (cx + rr * math.cos(pa), cy + rr * math.sin(pa))
        bl = (cx + (rr + S * 0.04) * math.cos(pa - 0.035),
              cy + (rr + S * 0.04) * math.sin(pa - 0.035))
        br = (cx + (rr + S * 0.04) * math.cos(pa + 0.035),
              cy + (rr + S * 0.04) * math.sin(pa + 0.035))
        self.create_polygon(*tip, *bl, *br, fill=C_SYMBOL, outline="")

        # --- 固定の機体シンボル ---
        wing = S * 0.16
        gap = S * 0.04
        self.create_line(cx - wing, cy, cx - gap, cy, fill=C_SYMBOL, width=4)
        self.create_line(cx + gap, cy, cx + wing, cy, fill=C_SYMBOL, width=4)
        self.create_line(cx - gap, cy, cx - gap, cy + gap, fill=C_SYMBOL, width=4)
        self.create_line(cx + gap, cy, cx + gap, cy + gap, fill=C_SYMBOL, width=4)
        self.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, fill=C_SYMBOL, outline="")

        # --- 方位テープ (下端) ---
        self._draw_heading_tape(w, h, S)

        if not self.valid:
            self.create_text(cx, cy - S * 0.22, text="NO ATTITUDE", fill=C_WARN,
                             font=("Menlo", int(S * 0.045), "bold"))

    def _draw_heading_tape(self, w, h, S):
        ty = h - S * 0.07
        self.create_rectangle(0, ty - S * 0.04, w, h, fill="#101010", outline="")
        cx = w / 2.0
        vr = 45.0                       # 表示範囲 ±deg
        hpp = (w * 0.5) / vr
        names = {0: "N", 90: "E", 180: "S", 270: "W"}
        base = round(self.heading)
        for d in range(base - int(vr), base + int(vr) + 1):
            if d % 5 != 0:
                continue
            dd = ((d - self.heading + 180) % 360) - 180
            x = cx + dd * hpp
            if x < 4 or x > w - 4:
                continue
            major = (d % 10 == 0)
            self.create_line(x, ty - S * 0.025, x, ty, fill=C_LINE,
                             width=2 if major else 1)
            dn = d % 360
            if dn in names:
                self.create_text(x, ty + S * 0.022, text=names[dn], fill=C_SYMBOL,
                                 font=("Menlo", int(S * 0.03), "bold"))
            elif d % 30 == 0:
                self.create_text(x, ty + S * 0.022, text=f"{dn:03d}", fill=C_LINE,
                                 font=("Menlo", int(S * 0.022)))
        # 中央インデックス
        self.create_polygon(cx, ty - S * 0.04, cx - S * 0.02, ty - S * 0.065,
                            cx + S * 0.02, ty - S * 0.065, fill=C_SYMBOL, outline="")
        self.create_text(cx, h - S * 0.018, text=f"{int(round(self.heading)) % 360:03d}°",
                         fill=C_READOUT, font=("Menlo", int(S * 0.03), "bold"))


class Readout(ttk.Frame):
    """ラベル付きデジタル計器 1 個。"""

    def __init__(self, master, title, unit="", big=22):
        super().__init__(master)
        tk.Label(self, text=title, fg=C_LABEL, bg=C_BEZEL,
                 font=("Menlo", 10)).pack(anchor="w")
        self.var = tk.StringVar(value="--")
        row = tk.Frame(self, bg=C_BEZEL)
        row.pack(anchor="w")
        self.val = tk.Label(row, textvariable=self.var, fg=C_READOUT, bg=C_BEZEL,
                            font=("Menlo", big, "bold"))
        self.val.pack(side=tk.LEFT)
        if unit:
            tk.Label(row, text=" " + unit, fg=C_LABEL, bg=C_BEZEL,
                     font=("Menlo", 11)).pack(side=tk.LEFT, anchor="s", pady=(0, 4))

    def set(self, text, warn=False):
        self.var.set(text)
        self.val.config(fg=C_WARN if warn else C_READOUT)


class PilotApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.ui_queue: "queue.Queue" = queue.Queue()
        self.worker = BleWorker(self.ui_queue)
        self.worker.start()
        self.rx = TelemetryRx()
        self.connected = False
        self.state = {}                 # 最新テレメトリ vals
        self.host_rssi = None
        self._rate_n = 0
        self._rate_t = None
        self._rate = 0.0
        self._last_seq = None
        self._lost = 0

        root.title("Shizuku 操縦計器")
        root.configure(bg=C_BEZEL)
        root.geometry("880x560")
        root.minsize(760, 480)

        self._build_ui()
        self.root.after(33, self._tick)         # ~30fps: キュー処理 + 再描画
        self.root.after(1000, self._poll_rssi)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self):
        # 上部バー
        top = tk.Frame(self.root, bg=C_BEZEL)
        top.pack(fill=tk.X, padx=8, pady=6)
        self.scan_btn = ttk.Button(top, text="スキャン", command=self._on_scan)
        self.scan_btn.pack(side=tk.LEFT)
        self.device_var = tk.StringVar()
        self.device_box = ttk.Combobox(top, textvariable=self.device_var,
                                       state="readonly", width=34)
        self.device_box.pack(side=tk.LEFT, padx=6)
        self.connect_btn = ttk.Button(top, text="接続", command=self._on_connect_toggle)
        self.connect_btn.pack(side=tk.LEFT)
        ttk.Button(top, text="時刻同期",
                   command=lambda: self.worker.submit(self.worker.send_time())
                   ).pack(side=tk.LEFT, padx=6)
        ttk.Button(top, text="エンジニア画面", command=self._launch_engineer).pack(side=tk.LEFT)
        self.status_var = tk.StringVar(value="● 未接続")
        tk.Label(top, textvariable=self.status_var, fg=C_LABEL, bg=C_BEZEL,
                 font=("Menlo", 11)).pack(side=tk.RIGHT)

        body = tk.Frame(self.root, bg=C_BEZEL)
        body.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)

        # 中央: 人工水平儀
        self.ai = AttitudeIndicator(body, size=460)
        self.ai.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # 右: デジタル計器盤
        panel = tk.Frame(body, bg=C_BEZEL)
        panel.pack(side=tk.LEFT, fill=tk.Y, padx=(10, 0))
        self.r_alt = Readout(panel, "融合高度 ALT", "m", big=30)
        self.r_alt.pack(anchor="w", pady=6)
        self.r_vs = Readout(panel, "上下速度 V/S", "m/s", big=26)
        self.r_vs.pack(anchor="w", pady=6)
        self.r_hdg = Readout(panel, "方位 HDG", "°", big=26)
        self.r_hdg.pack(anchor="w", pady=6)
        self.r_rp = Readout(panel, "ロール / ピッチ", "°", big=20)
        self.r_rp.pack(anchor="w", pady=6)
        self.r_vstate = Readout(panel, "上下状態", big=18)
        self.r_vstate.pack(anchor="w", pady=6)

        ttk.Separator(panel, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=8)
        self.r_link = Readout(panel, "リンク (RSSI / レート)", big=16)
        self.r_link.pack(anchor="w", pady=4)
        self.r_calib = Readout(panel, "較正 S/G/A/M", big=18)
        self.r_calib.pack(anchor="w", pady=4)

    # -- 操作 ---------------------------------------------------------------
    def _on_scan(self):
        self.scan_btn.config(state=tk.DISABLED)
        self.worker.submit(self.worker.scan())

    def _on_connect_toggle(self):
        if self.connected:
            self.worker.submit(self.worker.disconnect())
            return
        addr = getattr(self, "_label_to_addr", {}).get(self.device_var.get())
        if not addr:
            self.status_var.set("先にスキャン/選択してください")
            return
        self.connect_btn.config(state=tk.DISABLED)
        self.worker.submit(self.worker.connect(addr))

    def _launch_engineer(self):
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "shizuku_telemetry_client.py")
        try:
            subprocess.Popen([sys.executable, path])
        except OSError as e:  # noqa: BLE001
            self.status_var.set(f"起動失敗: {e}")

    # -- キュー処理 + 再描画 (30fps) ----------------------------------------
    def _tick(self):
        try:
            while True:
                kind, payload = self.ui_queue.get_nowait()
                self._handle(kind, payload)
        except queue.Empty:
            pass
        self._refresh()
        self.root.after(33, self._tick)

    def _handle(self, kind, payload):
        if kind == "rx":
            samples, _texts = self.rx.feed(payload["data"])
            for v in samples:
                self._ingest(v)
        elif kind == "scan_done":
            self._label_to_addr = {lbl: addr for lbl, addr in payload["items"]}
            labels = list(self._label_to_addr.keys())
            self.device_box["values"] = labels
            if labels and not self.device_var.get():
                self.device_var.set(labels[0])
            self.scan_btn.config(state=tk.NORMAL)
        elif kind == "rssi_host":
            v = payload.get("value")
            if v is not None:
                self.host_rssi = v
        elif kind == "connected":
            self.connected = True
            self.connect_btn.config(state=tk.NORMAL, text="切断")
            self.status_var.set(f"● 接続中 {payload.get('address','')}")
            for d in (1500, 4000, 8000):     # ペアリング後の自動時刻同期
                self.root.after(d, lambda: self.connected and
                                self.worker.submit(self.worker.send_time()))
        elif kind == "disconnected":
            self.connected = False
            self.connect_btn.config(state=tk.NORMAL, text="接続")
            self.status_var.set("● 未接続")
            self.host_rssi = None
            self._last_seq = None

    def _ingest(self, v):
        self.state = v
        self.ai.update_attitude(v.get("roll", 0.0), v.get("pitch", 0.0),
                                v.get("heading", 0.0), valid=True)
        # 受信レート / ロス
        now = time.monotonic()
        if self._rate_t is None:
            self._rate_t = now
        self._rate_n += 1
        seq = int(v.get("seq", 0))
        if self._last_seq is not None and seq > self._last_seq + 1:
            self._lost += seq - self._last_seq - 1
        self._last_seq = seq
        if now - self._rate_t >= 1.0:
            self._rate = self._rate_n / (now - self._rate_t)
            self._rate_n = 0
            self._rate_t = now

    def _refresh(self):
        self.ai.redraw()
        s = self.state
        if not s:
            return
        self.r_alt.set(f"{s.get('alt_fused_m', 0.0):+.2f}")
        vs = s.get("vel_m_s", 0.0)
        self.r_vs.set(f"{vs:+.2f}", warn=abs(vs) > 2.0)
        self.r_hdg.set(f"{int(round(s.get('heading', 0.0))) % 360:03d}")
        self.r_rp.set(f"{s.get('roll', 0.0):+.1f} / {s.get('pitch', 0.0):+.1f}")
        self.r_vstate.set(VSTATE_STR.get(int(s.get("vstate", 0)), "?"))
        b = int(s.get("calib", 0))
        sgam = f"{(b>>6)&3} {(b>>4)&3} {(b>>2)&3} {b&3}"
        self.r_calib.set(sgam, warn=((b >> 2) & 3) < 3)  # ACC 未較正を警告色
        rssi = f"{self.host_rssi}dBm" if self.host_rssi is not None else "--"
        self.r_link.set(f"{rssi} / {self._rate:.0f}Hz L{self._lost}",
                        warn=(not self.connected))

    def _poll_rssi(self):
        if self.connected:
            self.worker.submit(self.worker.read_rssi())
        self.root.after(1000, self._poll_rssi)

    def _on_close(self):
        self.worker.stop()
        self.root.destroy()


def main():
    root = tk.Tk()
    PilotApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
