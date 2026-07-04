#!/usr/bin/env python3
"""Shizuku 操縦者向け 計器クライアント (エアバス風 グラスコックピット PFD)

操縦者がひと目で機体姿勢を把握するための計器表示に特化したクライアント。
Airbus の PFD (Primary Flight Display) を模し、中央に人工水平儀
(アティチュード・インジケータ: ロール/ピッチ + バンク角スケール + スリップ計)、
右に高度テープと上下速度スケール、下端に方位テープを配置する。

エンジニア向けのグラフ/デバッグ表示は shizuku_telemetry_client.py の方。
両者はプロトコル層 shizuku_link.py を共有する。

  pip install bleak     (tkinter は標準ライブラリ)
  python shizuku_pilot_client.py
"""

import math
import queue
import re
import time
import tkinter as tk
from tkinter import ttk

from shizuku_link import BleWorker, TelemetryRx, VSTATE_STR

# 計器の符号調整 (機体/座標系に合わせて反転したいとき用)。
ROLL_SIGN = -1.0   # 人工水平儀の地平線はロールと逆回転
PITCH_SIGN = 1.0   # ピッチ↑で地平線が下がる

# ---- Airbus PFD 配色 ------------------------------------------------------
C_SKY = "#1076c4"        # Airbus ブルー (空)
C_GND = "#7d5a2b"        # Airbus ブラウン (地面)
C_LINE = "#ffffff"       # 地平線・ピッチラダー
C_BEZEL = "#000000"      # ディスプレイ地色
C_PANEL = "#0b0b0b"      # 計器ボックス地色
C_FRAME = "#3a3a3a"      # ボックス枠
C_SYMBOL = "#ffe000"     # 機体シンボル / バンクポインタ (Airbus イエロー)
C_READOUT = "#12e21a"    # 数値 (Airbus グリーン)
C_MAGENTA = "#ff35ff"    # 目標値 (Airbus マゼンタ)
C_LABEL = "#c8c8c8"      # ラベル (白グレー)
C_AMBER = "#ff9d00"      # 注意 (Airbus アンバー)
C_WARN = "#ff3b30"       # 警告 (赤)

FONT = "Menlo"


class AttitudeIndicator(tk.Canvas):
    """Airbus 風 PFD。姿勢球 + バンク角スケール + 機体シンボル + スリップ計、
    右に高度テープ・上下速度スケール、下に方位テープ。"""

    def __init__(self, master, size=440, **kw):
        super().__init__(master, width=size, height=size, bg=C_BEZEL,
                         highlightthickness=0, **kw)
        self.size = size
        self.roll = 0.0
        self.pitch = 0.0
        self.heading = 0.0
        self.alt = 0.0
        self.vs = 0.0
        self.spd = 0.0
        self.valid = False
        self.bind("<Configure>", lambda _e: self.redraw())

    def update_attitude(self, roll, pitch, heading, alt=0.0, vs=0.0, spd=0.0,
                        valid=True):
        self.roll, self.pitch, self.heading = roll, pitch, heading
        self.alt, self.vs, self.spd, self.valid = alt, vs, spd, valid

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

        # 左に速度テープ、右に高度テープ＋上下速度スケール、下に方位テープを確保し、
        # 姿勢球は残りの中央に置く。
        self.spd_x0 = S * 0.03                     # 速度テープ左端
        self.spd_w = S * 0.13
        self.spd_x1 = self.spd_x0 + self.spd_w
        self.vs_x = w - S * 0.045                  # 上下速度スケール軸
        self.alt_x1 = self.vs_x - S * 0.055        # 高度テープ右端
        self.alt_w = S * 0.13
        self.alt_x0 = self.alt_x1 - self.alt_w
        self.hdg_h = S * 0.11                       # 方位テープ高
        ball_left = self.spd_x1 + S * 0.02
        ball_right = self.alt_x0 - S * 0.02
        cx, cy = (ball_left + ball_right) / 2.0, (h - self.hdg_h) * 0.46

        self._draw_ball(w, h, S, cx, cy, ball_right)
        self._draw_pitch_ladder(S, cx, cy)
        self._draw_roll_scale(S, cx, cy)
        self._draw_symbol(S, cx, cy)
        # テープ類は姿勢球の上に不透明に重ねる。
        self._draw_spd_tape(w, h, S)
        self._draw_alt_tape(w, h, S)
        self._draw_vs_scale(w, h, S)
        self._draw_heading_tape(w, h, S, cx, ball_left, ball_right)

        if not self.valid:
            self.create_rectangle(cx - S * 0.16, cy - S * 0.06,
                                  cx + S * 0.16, cy + S * 0.06,
                                  outline=C_WARN, width=2, fill="")
            self.create_text(cx, cy, text="ATT", fill=C_WARN,
                             font=(FONT, int(S * 0.05), "bold"))

    # -- 空/地面/地平線 -----------------------------------------------------
    def _draw_ball(self, w, h, S, cx, cy, ball_right):
        ppd = S * 0.014                 # ピッチ pixels/deg
        ang = ROLL_SIGN * self.roll     # 地平線の回転
        horizon_y = cy + PITCH_SIGN * self.pitch * ppd
        R = S * 1.8

        def rp(px, py):
            return self._rot(px, py, cx, cy, ang)

        sky = [rp(cx - R, cy - R), rp(cx + R, cy - R),
               rp(cx + R, horizon_y), rp(cx - R, horizon_y)]
        gnd = [rp(cx - R, horizon_y), rp(cx + R, horizon_y),
               rp(cx + R, cy + R), rp(cx - R, cy + R)]
        self.create_polygon(*[c for pt in sky for c in pt], fill=C_SKY, outline="")
        self.create_polygon(*[c for pt in gnd for c in pt], fill=C_GND, outline="")
        p1, p2 = rp(cx - R, horizon_y), rp(cx + R, horizon_y)
        self.create_line(*p1, *p2, fill=C_LINE, width=3)
        self._horizon = (cx, cy, ang, ppd, horizon_y)

    def _draw_pitch_ladder(self, S, cx, cy):
        cx, cy, ang, ppd, _ = self._horizon

        def rp(px, py):
            return self._rot(px, py, cx, cy, ang)

        for p in range(-30, 31, 5):
            if p == 0:
                continue
            ry = cy + PITCH_SIGN * (self.pitch - p) * ppd
            major = (p % 10 == 0)
            half = S * (0.11 if major else 0.05)
            a1, a2 = rp(cx - half, ry), rp(cx + half, ry)
            self.create_line(*a1, *a2, fill=C_LINE, width=2 if major else 1)
            if major:
                txt = str(abs(p))
                le = rp(cx - half - S * 0.028, ry)
                re = rp(cx + half + S * 0.028, ry)
                self.create_text(*le, text=txt, fill=C_LINE,
                                 font=(FONT, int(S * 0.026)))
                self.create_text(*re, text=txt, fill=C_LINE,
                                 font=(FONT, int(S * 0.026)))

    # -- バンク角スケール (Airbus: 上部固定アーク + 可動ポインタ + スリップ計) --
    def _draw_roll_scale(self, S, cx, cy):
        rr = S * 0.40
        for b in (-60, -45, -30, -20, -10, 10, 20, 30, 45, 60):
            aa = math.radians(-90 + b)
            big = b % 30 == 0
            tl = S * (0.05 if big else 0.028)
            x0, y0 = cx + rr * math.cos(aa), cy + rr * math.sin(aa)
            x1, y1 = cx + (rr - tl) * math.cos(aa), cy + (rr - tl) * math.sin(aa)
            self.create_line(x0, y0, x1, y1, fill=C_LINE, width=2 if big else 1)
        # 0° 固定基準三角 (上端に頂点を下向き)
        top = (cx, cy - rr)
        self.create_polygon(top[0], top[1],
                            cx - S * 0.022, cy - rr - S * 0.035,
                            cx + S * 0.022, cy - rr - S * 0.035,
                            fill=C_LINE, outline="")

        # 可動バンクポインタ (イエロー三角, ロールで回転)
        pa = math.radians(-90 + self.roll)
        tipr, baser = rr - S * 0.008, rr - S * 0.055

        def polar(r, dth):
            return (cx + r * math.cos(pa + dth), cy + r * math.sin(pa + dth))

        tip = polar(tipr, 0.0)
        bl = polar(baser, -0.05)
        br = polar(baser, 0.05)
        self.create_polygon(*tip, *bl, *br, fill=C_SYMBOL, outline="")
        # スリップ計 (ポインタ下の台形, 協調飛行を表す)
        s_out, s_in = baser - S * 0.006, baser - S * 0.04
        trap = [polar(s_out, -0.055), polar(s_out, 0.055),
                polar(s_in, 0.04), polar(s_in, -0.04)]
        self.create_polygon(*[c for pt in trap for c in pt],
                            outline=C_LINE, width=2, fill="")

    # -- 固定の機体シンボル (Airbus 風の縁取りウィング) ---------------------
    def _draw_symbol(self, S, cx, cy):
        wing = S * 0.17
        gap = S * 0.045
        drop = S * 0.035

        def bar(x0, y0, x1, y1):
            self.create_line(x0, y0, x1, y1, fill=C_BEZEL,
                             width=8, capstyle=tk.PROJECTING)
            self.create_line(x0, y0, x1, y1, fill=C_SYMBOL,
                             width=4, capstyle=tk.PROJECTING)

        bar(cx - wing, cy, cx - gap, cy)
        bar(cx - gap, cy, cx - gap, cy + drop)
        bar(cx + gap, cy, cx + wing, cy)
        bar(cx + gap, cy, cx + gap, cy + drop)
        self.create_rectangle(cx - 3, cy - 3, cx + 3, cy + 3,
                              fill=C_SYMBOL, outline=C_BEZEL)

    # -- 速度テープ (左, Airbus 風 対地速度) -------------------------------
    def _draw_spd_tape(self, w, h, S):
        x0, x1 = self.spd_x0, self.spd_x1
        ty0, ty1 = S * 0.05, h - self.hdg_h
        cyt = (ty0 + ty1) / 2.0
        pxps = S * 0.16                  # pixels/(m/s)
        self.create_rectangle(x0, ty0, x1, ty1, fill=C_PANEL, outline=C_FRAME)

        half = (cyt - ty0) / pxps
        # 0.5 m/s 刻みで目盛、整数で数字。速度は負にならないので 0 未満は描かない。
        k0 = int(math.floor((self.spd - half) / 0.5))
        k1 = int(math.ceil((self.spd + half) / 0.5))
        for k in range(k0, k1 + 1):
            val = k * 0.5
            if val < 0:
                continue
            y = cyt - (val - self.spd) * pxps
            if y < ty0 + 2 or y > ty1 - 2:
                continue
            if k % 2 == 0:               # 整数 m/s
                self.create_line(x1, y, x1 - S * 0.03, y, fill=C_LINE, width=2)
                self.create_text(x0 + S * 0.008, y, text=str(k // 2), anchor="w",
                                 fill=C_LINE, font=(FONT, int(S * 0.026)))
            else:
                self.create_line(x1, y, x1 - S * 0.016, y, fill=C_LINE, width=1)

        # 現在速度リードアウト (黒箱 + グリーン数字 + 右向きノッチ)
        bh = S * 0.06
        self.create_rectangle(x0, cyt - bh / 2, x1 + S * 0.012, cyt + bh / 2,
                              fill=C_BEZEL, outline=C_LINE, width=2)
        self.create_polygon(x1 + S * 0.012, cyt,
                            x1 + S * 0.04, cyt - S * 0.02,
                            x1 + S * 0.04, cyt + S * 0.02,
                            fill=C_BEZEL, outline=C_LINE, width=2)
        self.create_text((x0 + x1) / 2, cyt, text=f"{self.spd:.1f}",
                         fill=C_READOUT, font=(FONT, int(S * 0.034), "bold"))
        self.create_text((x0 + x1) / 2, ty0 - S * 0.02, text="SPD m/s",
                         fill=C_LABEL, font=(FONT, int(S * 0.022)))

    # -- 高度テープ (右, Airbus 風) ----------------------------------------
    def _draw_alt_tape(self, w, h, S):
        x0, x1 = self.alt_x0, self.alt_x1
        ty0, ty1 = S * 0.05, h - self.hdg_h
        cyt = (ty0 + ty1) / 2.0
        pxpm = S * 0.045                 # pixels/m
        self.create_rectangle(x0, ty0, x1, ty1, fill=C_PANEL, outline=C_FRAME)

        half_m = (cyt - ty0) / pxpm
        lo = int(math.floor(self.alt - half_m))
        hi = int(math.ceil(self.alt + half_m))
        for m in range(lo, hi + 1):
            y = cyt - (m - self.alt) * pxpm
            if y < ty0 + 2 or y > ty1 - 2:
                continue
            if m % 5 == 0:
                self.create_line(x0, y, x0 + S * 0.03, y, fill=C_LINE, width=2)
                self.create_text(x1 - S * 0.008, y, text=str(m), anchor="e",
                                 fill=C_LINE, font=(FONT, int(S * 0.026)))
            elif m % 1 == 0:
                self.create_line(x0, y, x0 + S * 0.016, y, fill=C_LINE, width=1)

        # 現在高度リードアウト (黒箱 + グリーン数字 + 左向きノッチ)
        bh = S * 0.06
        self.create_rectangle(x0 - S * 0.012, cyt - bh / 2, x1, cyt + bh / 2,
                              fill=C_BEZEL, outline=C_LINE, width=2)
        self.create_polygon(x0 - S * 0.012, cyt,
                            x0 - S * 0.04, cyt - S * 0.02,
                            x0 - S * 0.04, cyt + S * 0.02,
                            fill=C_BEZEL, outline=C_LINE, width=2)
        self.create_text((x0 + x1) / 2, cyt, text=f"{self.alt:+.1f}",
                         fill=C_READOUT, font=(FONT, int(S * 0.034), "bold"))
        self.create_text((x0 + x1) / 2, ty0 - S * 0.02, text="ALT m",
                         fill=C_LABEL, font=(FONT, int(S * 0.022)))

    # -- 上下速度スケール (最右, Airbus VSI 風) ----------------------------
    def _draw_vs_scale(self, w, h, S):
        ax = self.vs_x
        ty0, ty1 = S * 0.10, h - self.hdg_h - S * 0.02
        cyt = (ty0 + ty1) / 2.0
        vspp = (cyt - ty0) / 3.2         # pixels per m/s (フルスケール ~3 m/s)
        self.create_line(ax, ty0, ax, ty1, fill=C_FRAME, width=1)
        for v in (-3, -2, -1, 1, 2, 3):
            y = cyt - v * vspp
            self.create_line(ax, y, ax - S * 0.02, y, fill=C_LINE, width=1)
            self.create_text(ax - S * 0.03, y, text=str(abs(v)), anchor="e",
                             fill=C_LABEL, font=(FONT, int(S * 0.02)))
        self.create_line(ax - S * 0.024, cyt, ax, cyt, fill=C_LINE, width=2)
        # 可動ポインタ
        vy = cyt - max(-3.4, min(3.4, self.vs)) * vspp
        col = C_AMBER if abs(self.vs) > 2.0 else C_READOUT
        self.create_line(ax, cyt, ax - S * 0.05, vy, fill=col, width=3)
        self.create_text(ax - S * 0.02, ty0 - S * 0.028, text="V/S",
                         fill=C_LABEL, font=(FONT, int(S * 0.02)))
        self.create_text(ax - S * 0.02, ty0 - S * 0.006,
                         text=f"{self.vs:+.1f}", fill=col,
                         font=(FONT, int(S * 0.024), "bold"))

    # -- 方位テープ (下端) -------------------------------------------------
    def _draw_heading_tape(self, w, h, S, cx, x_left, x_right):
        ty = h - self.hdg_h
        self.create_rectangle(x_left, ty, x_right, h, fill=C_PANEL, outline=C_FRAME)
        vr = 45.0                       # 表示範囲 ±deg
        hpp = ((x_right - x_left) * 0.5) / vr
        names = {0: "N", 90: "E", 180: "S", 270: "W"}
        base = round(self.heading)
        for d in range(base - int(vr), base + int(vr) + 1):
            if d % 5 != 0:
                continue
            dd = ((d - self.heading + 180) % 360) - 180
            x = cx + dd * hpp
            if x < x_left + 4 or x > x_right - 4:
                continue
            major = (d % 10 == 0)
            self.create_line(x, ty + S * 0.006, x, ty + S * 0.032, fill=C_LINE,
                             width=2 if major else 1)
            dn = d % 360
            if dn in names:
                self.create_text(x, ty + S * 0.055, text=names[dn], fill=C_SYMBOL,
                                 font=(FONT, int(S * 0.03), "bold"))
            elif d % 30 == 0:
                self.create_text(x, ty + S * 0.052, text=f"{dn:03d}", fill=C_LINE,
                                 font=(FONT, int(S * 0.022)))
        # 中央インデックス (現在方位リードアウト)
        bw, bh = S * 0.07, S * 0.045
        self.create_rectangle(cx - bw, ty - bh, cx + bw, ty, fill=C_BEZEL,
                              outline=C_LINE, width=2)
        self.create_polygon(cx, ty + S * 0.008, cx - S * 0.02, ty - S * 0.012,
                            cx + S * 0.02, ty - S * 0.012, fill=C_SYMBOL,
                            outline="")
        self.create_text(cx, ty - bh / 2,
                         text=f"{int(round(self.heading)) % 360:03d}",
                         fill=C_READOUT, font=(FONT, int(S * 0.03), "bold"))


class Readout(tk.Frame):
    """Airbus 風のラベル付きデジタル計器 1 個 (枠付きボックス)。"""

    def __init__(self, master, title, unit="", big=22):
        super().__init__(master, bg=C_PANEL, highlightbackground=C_FRAME,
                         highlightthickness=1)
        tk.Label(self, text=title, fg=C_LABEL, bg=C_PANEL,
                 font=(FONT, 10)).pack(anchor="w", padx=8, pady=(4, 0))
        self.var = tk.StringVar(value="--")
        row = tk.Frame(self, bg=C_PANEL)
        row.pack(anchor="w", padx=8, pady=(0, 4))
        self.val = tk.Label(row, textvariable=self.var, fg=C_READOUT, bg=C_PANEL,
                            font=(FONT, big, "bold"))
        self.val.pack(side=tk.LEFT)
        if unit:
            tk.Label(row, text=" " + unit, fg=C_LABEL, bg=C_PANEL,
                     font=(FONT, 11)).pack(side=tk.LEFT, anchor="s", pady=(0, 4))

    def set(self, text, warn=False):
        self.var.set(text)
        self.val.config(fg=C_AMBER if warn else C_READOUT)


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
        self.pico_rssi = None               # 機体側測定 ("RSSI=" 行の notify)
        # OS の bleak backend が接続中 RSSI 取得に非対応 (Windows/Linux)。
        # 検出後はポーリングをやめ、LINK 表示は Pico 側 RSSI で代用する。
        self._host_rssi_unsupported = False
        self._rate_n = 0
        self._rate_t = None
        self._rate = 0.0
        self._last_seq = None
        self._lost = 0

        root.title("Shizuku PFD")
        root.configure(bg=C_BEZEL)
        root.geometry("980x600")
        root.minsize(820, 520)

        self._init_style()
        self._build_ui()
        self.root.after(33, self._tick)         # ~30fps: キュー処理 + 再描画
        self.root.after(1000, self._poll_rssi)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _init_style(self):
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("Airbus.TButton", background="#1a1a1a",
                        foreground=C_LABEL, borderwidth=1, focuscolor=C_FRAME,
                        font=(FONT, 11))
        style.map("Airbus.TButton",
                  background=[("active", "#2a2a2a")],
                  foreground=[("active", C_LINE)])
        style.configure("Airbus.TCombobox", fieldbackground=C_PANEL,
                        background="#1a1a1a", foreground=C_READOUT,
                        arrowcolor=C_LABEL)

    def _build_ui(self):
        # 上部バー (FMA 風の黒帯)
        top = tk.Frame(self.root, bg="#111111")
        top.pack(fill=tk.X, padx=8, pady=(8, 4))
        tk.Label(top, text="SHIZUKU PFD", fg=C_SYMBOL, bg="#111111",
                 font=(FONT, 13, "bold")).pack(side=tk.LEFT, padx=(4, 12))
        self.scan_btn = ttk.Button(top, text="スキャン", style="Airbus.TButton",
                                   command=self._on_scan)
        self.scan_btn.pack(side=tk.LEFT)
        self.device_var = tk.StringVar()
        self.device_box = ttk.Combobox(top, textvariable=self.device_var,
                                       state="readonly", width=30,
                                       style="Airbus.TCombobox")
        self.device_box.pack(side=tk.LEFT, padx=6)
        self.connect_btn = ttk.Button(top, text="接続", style="Airbus.TButton",
                                      command=self._on_connect_toggle)
        self.connect_btn.pack(side=tk.LEFT)
        ttk.Button(top, text="時刻同期", style="Airbus.TButton",
                   command=lambda: self.worker.submit(self.worker.send_time())
                   ).pack(side=tk.LEFT, padx=6)
        self.status_var = tk.StringVar(value="● 未接続")
        tk.Label(top, textvariable=self.status_var, fg=C_AMBER, bg="#111111",
                 font=(FONT, 11, "bold")).pack(side=tk.RIGHT, padx=6)

        body = tk.Frame(self.root, bg=C_BEZEL)
        body.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)

        # 中央: PFD (人工水平儀 + 高度/上下速度/方位テープ)
        self.ai = AttitudeIndicator(body, size=520)
        self.ai.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # 右: 補助デジタル計器盤
        panel = tk.Frame(body, bg=C_BEZEL)
        panel.pack(side=tk.LEFT, fill=tk.Y, padx=(10, 0))
        self.r_rp = Readout(panel, "ROLL / PITCH", "°", big=20)
        self.r_rp.pack(fill=tk.X, pady=5)
        self.r_hdg = Readout(panel, "HEADING HDG", "°", big=26)
        self.r_hdg.pack(fill=tk.X, pady=5)
        self.r_vstate = Readout(panel, "上下状態 V-STATE", big=18)
        self.r_vstate.pack(fill=tk.X, pady=5)

        ttk.Separator(panel, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=8)
        self.r_link = Readout(panel, "LINK  RSSI / RATE", big=16)
        self.r_link.pack(fill=tk.X, pady=4)
        self.r_calib = Readout(panel, "CALIB  S/G/A/M", big=18)
        self.r_calib.pack(fill=tk.X, pady=4)

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
            samples, texts = self.rx.feed(payload["data"])
            for v in samples:
                self._ingest(v)
            for line in texts:
                m = re.match(r"RSSI=(-?\d+)", line)
                if m:
                    self.pico_rssi = int(m.group(1))
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
            elif payload.get("unsupported"):
                self._host_rssi_unsupported = True
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
            err = payload.get("error")
            self.status_var.set(f"● 接続失敗: {err}" if err else "● 未接続")
            self.host_rssi = None
            self.pico_rssi = None
            self._last_seq = None

    def _ingest(self, v):
        self.state = v
        self.ai.update_attitude(v.get("roll", 0.0), v.get("pitch", 0.0),
                                v.get("heading", 0.0),
                                alt=v.get("alt_fused_m", 0.0),
                                vs=v.get("vel_m_s", 0.0),
                                spd=v.get("speed_m_s", 0.0), valid=True)
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
        self.r_hdg.set(f"{int(round(s.get('heading', 0.0))) % 360:03d}")
        self.r_rp.set(f"{s.get('roll', 0.0):+.1f} / {s.get('pitch', 0.0):+.1f}")
        self.r_vstate.set(VSTATE_STR.get(int(s.get("vstate", 0)), "?"))
        b = int(s.get("calib", 0))
        sgam = f"{(b>>6)&3} {(b>>4)&3} {(b>>2)&3} {b&3}"
        self.r_calib.set(sgam, warn=((b >> 2) & 3) < 3)  # ACC 未較正を警告色
        # 母艦側 RSSI が取れない OS (Windows/Linux) では機体側測定で代用し、
        # 区別のため (P) を付ける。リンクはほぼ対称なので目安としては十分。
        if self.host_rssi is not None:
            rssi = f"{self.host_rssi}dBm"
        elif self.pico_rssi is not None:
            rssi = f"{self.pico_rssi}dBm(P)"
        else:
            rssi = "--"
        self.r_link.set(f"{rssi} / {self._rate:.0f}Hz L{self._lost}",
                        warn=(not self.connected))

    def _poll_rssi(self):
        if self.connected and not self._host_rssi_unsupported:
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
