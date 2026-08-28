#!/usr/bin/env python3
"""Shizuku Dynamic Object Hot-Reload (RAM Module Injector)

単一の C++ オブジェクトソースをコンパイルし、BLE GDB bridge または USB CDC シリアルを通じて
実機 (RP2350) の SRAM へミリ秒単位で転送・動的起動する。

使い方:
  # 1. コンパイルして実機へ注入・起動 (BLE bridge 経由)
  python3 tools/shizuku_hot_reload.py modules_dyn/fast_blink.cpp

  # 2. USB シリアル (プローブ不要) 経由で直接注入・起動
  python3 tools/shizuku_hot_reload.py modules_dyn/fast_blink.cpp --serial /dev/cu.usbmodem103

  # オプション:
  python3 tools/shizuku_hot_reload.py modules_dyn/fast_blink.cpp --port 3333
  python3 tools/shizuku_hot_reload.py modules_dyn/fast_blink.cpp --compile-only
  python3 tools/shizuku_hot_reload.py modules_dyn/fast_blink.cpp --dump-asm
"""

from __future__ import annotations

import argparse
import glob
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import time
try:
    import serial
except ImportError:
    serial = None


HOME = os.path.expanduser("~")
TOOLCHAIN_GLOB = os.path.join(HOME, ".pico-sdk/toolchain/*/bin")

# 静的 blink オブジェクト ID (xno_object_id::blink = 36)
STATIC_BLINK_OBJECT_ID = 36


def find_tool(name: str) -> str:
    """ツールチェーンの実行バイナリを探す (~/.pico-sdk または PATH)。"""
    for bin_dir in sorted(glob.glob(TOOLCHAIN_GLOB), reverse=True):
        candidate = os.path.join(bin_dir, f"arm-none-eabi-{name}")
        if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
            return candidate
    which_path = shutil.which(f"arm-none-eabi-{name}")
    if which_path:
        return which_path
    sys.exit(f"エラー: arm-none-eabi-{name} が見つかりません。~/.pico-sdk/toolchain を確認してください。")


def compile_module(source_path: str, output_bin: str, dump_asm: bool = False) -> tuple[int, str, int]:
    """C++ ソースを位置独立 Thumb-2 バイナリ (.bin) にコンパイルし、(サイズ, 逆アセンブル, エントリーオフセット) を返す。"""
    gxx = find_tool("g++")
    objcopy = find_tool("objcopy")
    objdump = find_tool("objdump")
    readelf = find_tool("readelf")

    source_dir = os.path.dirname(os.path.abspath(source_path))
    repo_root = os.path.dirname(source_dir) if ("modules_dyn" in source_dir or "tools" in source_dir) else source_dir
    include_dyn = os.path.join(repo_root, "modules_dyn")
    
    # Auto-find Shizuku internal headers from bazel external or local
    shizuku_headers = []
    for candidate in [
        os.path.join(repo_root, "bazel-flight_robocon_telemetory_sender/external/shizuku+/internal_headers"),
        os.path.join(repo_root, "bazel-flight_robocon_telemetory_sender/external/shizuku+/modules/pico_sdk_support/internal_headers"),
        os.path.join(repo_root, "bazel-flight_robocon_telemetory_sender/external/shizuku+/bazel-bin/configs/internal_headers"),
        os.path.join(repo_root, "bazel-flight_robocon_telemetory_sender/external/shizuku+/bazel-bin/generated_object_ids"),
        "/Users/ishigakiyua/github/Shizuku/internal_headers",
        "/Users/ishigakiyua/github/Shizuku/modules/pico_sdk_support/internal_headers",
    ]:
        if os.path.isdir(candidate):
            shizuku_headers.append(f"-I{candidate}")

    with tempfile.NamedTemporaryFile(suffix=".elf", delete=False) as tmp_elf:
        elf_path = tmp_elf.name

    try:
        # 位置独立(PIC/PIE), 外部依存なし(nostdlib), Thumb-2 (Cortex-M33)
        cmd_compile = [
            gxx,
            "-mcpu=cortex-m33",
            "-mthumb",
            "-fPIC",
            "-fPIE",
            "-Os",
            "-nostdlib",
            "-ffreestanding",
            "-fno-exceptions",
            "-fno-rtti",
            "-fno-unwind-tables",
            "-fno-asynchronous-unwind-tables",
            "-Wl,-Ttext=0x0",
            "-Wl,-N",
            "-Wl,--entry=dynamic_module_main",
            "-Wl,--gc-sections",
            "-DSHIZUKU_DYNAMIC_MODULE=1",
            f"-I{repo_root}",
            f"-I{include_dyn}",
            f"-I{source_dir}",
            *shizuku_headers,
            "-o", elf_path,
            source_path,
        ]
        res = subprocess.run(cmd_compile, capture_output=True, text=True)
        if res.returncode != 0:
            sys.exit(f"コンパイルエラー:\n{res.stderr}")

        # エントリーポイントのオフセットを取得 (readelf -h)
        entry_offset = 0
        res_readelf = subprocess.run([readelf, "-h", elf_path], capture_output=True, text=True)
        for line in res_readelf.stdout.splitlines():
            if "Entry point address:" in line:
                try:
                    addr_val = int(line.split(":", 1)[1].strip(), 16)
                    entry_offset = addr_val & ~1
                    break
                except ValueError:
                    pass

        # .text セクションを生バイナリへ抽出
        cmd_copy = [objcopy, "-O", "binary", elf_path, output_bin]
        res = subprocess.run(cmd_copy, capture_output=True, text=True)
        if res.returncode != 0:
            sys.exit(f"objcopy エラー:\n{res.stderr}")

        bin_size = os.path.getsize(output_bin)
        asm_output = ""
        if dump_asm:
            res = subprocess.run([objdump, "-d", elf_path], capture_output=True, text=True)
            asm_output = res.stdout

        return bin_size, asm_output, entry_offset
    finally:
        if os.path.exists(elf_path):
            os.unlink(elf_path)


# ===========================================================================
#  GDB RSP (Remote Serial Protocol) クライアント
# ===========================================================================
def rsp_checksum(data: bytes) -> str:
    """GDB RSP チェックサムを計算する (モジュロ 256 の 2 桁 16 進)。"""
    return f"{sum(data) % 256:02x}"


def rsp_encode_packet(payload: str) -> bytes:
    """RSP パケット ($<payload>#<checksum>) を作成。"""
    raw = payload.encode("ascii")
    chk = rsp_checksum(raw)
    return b"$" + raw + b"#" + chk.encode("ascii")


class SerialRspClient:
    """USB CDC シリアルポートと直接 RSP 通信するクライアント。"""
    def __init__(self, port: str = "/dev/cu.usbmodem103", timeout: float = 2.0):
        self.port = port
        self.timeout = timeout
        self.ser = None

    def connect(self):
        if serial is None:
            raise RuntimeError("pyserial がインストールされていません")
        self.ser = serial.Serial(self.port, 115200, timeout=self.timeout)
        time.sleep(0.1)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.ser.write(b"+")
        self.ser.flush()
        time.sleep(0.05)

    def close(self):
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def send_cmd(self, payload: str) -> str:
        if not self.ser:
            raise RuntimeError("未接続です")
        # 以前の滞留データをクリア
        while self.ser.in_waiting > 0:
            self.ser.read(self.ser.in_waiting)
        packet = rsp_encode_packet(payload)
        self.ser.write(packet)
        self.ser.flush()
        
        # 応答の受信
        buf = bytearray()
        t0 = time.time()
        while time.time() - t0 < self.timeout:
            chunk = self.ser.read(self.ser.in_waiting or 1)
            if not chunk:
                continue
            buf.extend(chunk)
            if b"$" in buf and b"#" in buf:
                start = buf.find(b"$")
                hash_idx = buf.find(b"#", start)
                if hash_idx != -1 and len(buf) >= hash_idx + 3:
                    self.ser.write(b"+")
                    self.ser.flush()
                    return buf[start + 1:hash_idx].decode("ascii", errors="replace")
        return buf.decode("ascii", errors="replace")

    def qrcmd(self, cmd_text: str) -> str:
        hex_cmd = cmd_text.encode("utf-8").hex()
        resp = self.send_cmd(f"qRcmd,{hex_cmd}").strip()
        if resp.startswith("$"):
            resp = resp[1:].split("#")[0]
        try:
            return bytes.fromhex(resp).decode("utf-8", errors="replace")
        except ValueError:
            return resp

    def memory_write(self, addr: int, data: bytes, chunk_size: int = 64) -> bool:
        for offset in range(0, len(data), chunk_size):
            chunk = data[offset : offset + chunk_size]
            cur_addr = addr + offset
            hex_data = chunk.hex()
            resp = self.send_cmd(f"M{cur_addr:x},{len(chunk):x}:{hex_data}")
            if resp != "OK":
                return False
        return True


def _heartbeat_path(port: int) -> str:
    return os.path.join(tempfile.gettempdir(), f"gdb_ble_bridge.{port}.beat")

def _read_heartbeat(port: int):
    try:
        with open(_heartbeat_path(port), encoding="utf-8") as f:
            pid_text, stamp_text = f.read().split()
        pid, stamp = int(pid_text), float(stamp_text)
        if time.time() - stamp > 8.0:
            return None
        os.kill(pid, 0)
        return {"pid": pid, "age": time.time() - stamp}
    except Exception:
        return None

def ensure_gdb_bridge(port: int = 3333, timeout: float = 25.0) -> subprocess.Popen | None:
    """GDB BLE Bridge を確実に起動・接続する。"""
    beat = _read_heartbeat(port)
    if beat is not None:
        return None

    subprocess.run(["pkill", "-f", "gdb_ble_bridge.py"], capture_output=True)
    time.sleep(0.3)

    bridge_script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gdb_ble_bridge.py")
    python_bin = "/opt/homebrew/bin/python3" if os.path.exists("/opt/homebrew/bin/python3") else sys.executable
    log_file = open("/tmp/gdb_ble_bridge.log", "w", encoding="utf-8")
    bridge = subprocess.Popen(
        [python_bin, "-u", bridge_script, "--port", str(port), "--stay"],
        stdout=log_file,
        stderr=subprocess.STDOUT,
    )

    t0 = time.time()
    while time.time() - t0 < timeout:
        if bridge.poll() is not None:
            log_file.close()
            with open("/tmp/gdb_ble_bridge.log", "r", encoding="utf-8") as f:
                err = f.read()
            sys.exit(f"エラー: GDB BLE Bridge が異常終了しました:\n{err}")
        
        beat = _read_heartbeat(port)
        if beat is not None:
            time.sleep(0.2)
            return bridge
        time.sleep(0.2)
    sys.exit("エラー: GDB BLE Bridge の起動待機がタイムアウトしました。")

class RspClient:
    """GDB BLE bridge (TCP ソケット) と直接 RSP 通信する軽量クライアント。"""

    def __init__(self, host: str = "127.0.0.1", port: int = 3333, timeout: float = 10.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock: socket.socket | None = None

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        self.sock.connect((self.host, self.port))
        
        # 接続直後の滞留データ (前セッションの残りパケット等) をクリア
        self.sock.setblocking(False)
        time.sleep(0.05)
        try:
            while True:
                data = self.sock.recv(512)
                if not data:
                    break
        except (BlockingIOError, socket.error):
            pass
        self.sock.setblocking(True)
        self.sock.settimeout(self.timeout)

        # 最初の握手 ACK ('+')
        self.sock.sendall(b"+")

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

    def send_cmd(self, payload: str) -> str:
        """RSP コマンドを送信し、返信を受信する。"""
        if not self.sock:
            raise RuntimeError("未接続です")
        packet = rsp_encode_packet(payload)
        self.sock.sendall(packet)

        # 応答の受信
        buf = bytearray()
        while True:
            chunk = self.sock.recv(512)
            if not chunk:
                break
            buf.extend(chunk)
            # '+' (ACK) を読み飛ばし、$ から #xx までのパケットを待つ
            if b"$" in buf and b"#" in buf:
                start = buf.find(b"$")
                hash_idx = buf.find(b"#", start)
                if hash_idx != -1 and len(buf) >= hash_idx + 3:
                    # ACK を返す
                    self.sock.sendall(b"+")
                    resp = buf[start + 1:hash_idx].decode("ascii", errors="replace")
                    return resp
        return buf.decode("ascii", errors="replace")

    def qrcmd(self, cmd_text: str) -> str:
        """monitor コマンド (qRcmd) を送信し、実機コンソール応答 (hex decode) を返す。"""
        hex_cmd = cmd_text.encode("utf-8").hex()
        resp = self.send_cmd(f"qRcmd,{hex_cmd}").strip()
        if resp.startswith("$"):
            resp = resp[1:].split("#")[0]
        try:
            return bytes.fromhex(resp).decode("utf-8", errors="replace")
        except ValueError:
            return resp

    def memory_write(self, addr: int, data: bytes, chunk_size: int = 64) -> bool:
        """実機 RAM にバイナリデータを書き込む ($M<addr>,<len>:<hex>)."""
        for offset in range(0, len(data), chunk_size):
            chunk = data[offset : offset + chunk_size]
            cur_addr = addr + offset
            hex_data = chunk.hex()
            resp = self.send_cmd(f"M{cur_addr:x},{len(chunk):x}:{hex_data}")
            if resp != "OK":
                return False
        return True


# ===========================================================================
#  メイン処理: ホットリロード
# ===========================================================================
def hot_reload(source_path: str, port: int = 3333, serial_port: str | None = None, dump_asm: bool = False, compile_only: bool = False):
    print(f"=== Shizuku Dynamic Object Hot-Reload ===")
    print(f"対象ソース: {source_path}")

    # 1. コンパイル
    t0 = time.perf_counter()
    with tempfile.NamedTemporaryFile(suffix=".bin", delete=False) as tmp_bin:
        bin_path = tmp_bin.name

    try:
        bin_size, asm_text, entry_offset = compile_module(source_path, bin_path, dump_asm=dump_asm)
        t_compile = time.perf_counter() - t0
        print(f"[1/4] コンパイル完了: {bin_size} bytes (entry offset: +0x{entry_offset:x}, 所要時間: {t_compile * 1000:.1f}ms)")

        if dump_asm and asm_text:
            print("\n--- 逆アセンブル ---")
            print(asm_text)
            print("--------------------\n")

        if compile_only:
            print("コンパイル検証のみモードのため終了します。")
            return

        with open(bin_path, "rb") as f:
            bin_data = f.read()

        # 2. 接続確立 (USB シリアル または BLE TCP)
        bridge_proc = None
        if serial_port:
            print(f"[2/4] USB CDC シリアルポート ({serial_port}) へ直接接続中…")
            rsp = SerialRspClient(port=serial_port)
            try:
                rsp.connect()
            except Exception as e:
                sys.exit(f"エラー: シリアルポート {serial_port} への接続に失敗しました: {e}")
        else:
            print(f"[2/4] GDB BLE Bridge (127.0.0.1:{port}) へ接続中…")
            bridge_proc = ensure_gdb_bridge(port=port) if port == 3333 else None
            rsp = RspClient(port=port)
            try:
                rsp.connect()
            except Exception as e:
                sys.exit(f"エラー: 127.0.0.1:{port} への接続に失敗しました: {e}")

        try:
            # 既存の静的 blink (object_id=36) が動いていれば停止して競合を防ぐ
            rsp.qrcmd(f"call {STATIC_BLINK_OBJECT_ID} 5 0")
            time.sleep(0.05)

            # 3. 実機 RAM 確保 (monitor alloc <size>)
            # アラインメントと余裕のため 16 バイト単位で切り上げ
            alloc_size = ((bin_size + 15) // 16) * 16
            print(f"[3/4] 実機 SRAM 領域を確保中 ({alloc_size} bytes)...")
            alloc_resp = rsp.qrcmd(f"alloc {alloc_size}")
            # 応答から 0x20xxxxxx をパース
            ram_addr = None
            for line in alloc_resp.splitlines():
                line = line.strip()
                if line.startswith("0x") or line.startswith("20"):
                    try:
                        ram_addr = int(line, 16)
                        break
                    except ValueError:
                        pass
            if ram_addr is None:
                sys.exit(f"エラー: RAM 確保に失敗しました。応答: {alloc_resp}")
            print(f"      確保アドレス: 0x{ram_addr:08x}")

            # 4. バイナリを実機 RAM に転送
            print(f"[4/4] SRAM (0x{ram_addr:08x}) へバイナリ書き込み中...")
            t_write_start = time.perf_counter()
            ok = rsp.memory_write(ram_addr, bin_data)
            t_write = time.perf_counter() - t_write_start
            if not ok:
                sys.exit("エラー: メモリ書き込みに失敗しました。")
            print(f"      転送完了 ({t_write * 1000:.1f}ms)")

            # 5. 動的スレッド起動 (monitor spawn <addr>)
            # ARM Thumb モードのため最下位ビットを 1 にする
            entry_thumb = (ram_addr + entry_offset) | 1
            print(f"      動的スレッド起動中 (entry: 0x{entry_thumb:08x}, offset: +0x{entry_offset:x})...")
            spawn_resp = rsp.qrcmd(f"spawn 0x{entry_thumb:08x}")
            print(f"      実機応答: {spawn_resp.strip()}")

            # 6. GDB スタブから Detach ('D') して動的スレッドを実行再開
            try:
                rsp.send_cmd("D")
            except Exception:
                pass

            total_ms = (time.perf_counter() - t0) * 1000
            print(f"\n✨ ホットリロード成功！ 合計所要時間: {total_ms:.1f}ms")
            print(f"   (全ファームウェア焼き直しの約50秒に対して、{total_ms / 1000:.2f}秒で即時反映)")

        finally:
            rsp.close()
            if bridge_proc:
                try:
                    bridge_proc.terminate()
                except Exception:
                    pass

    finally:
        if os.path.exists(bin_path):
            os.unlink(bin_path)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", help="動的モジュールの C++ ソースファイル (.cpp)")
    parser.add_argument("--port", type=int, default=3333, help="GDB BLE bridge のポート番号 (既定: 3333)")
    parser.add_argument("--serial", type=str, default=None, help="USB CDC シリアルデバイス (例: /dev/cu.usbmodem103)")
    parser.add_argument("--dump-asm", action="store_true", help="逆アセンブルを出力する")
    parser.add_argument("--compile-only", action="store_true", help="コンパイルとバイナリ検証のみ実行する")
    args = parser.parse_args()

    hot_reload(args.source, port=args.port, serial_port=args.serial, dump_asm=args.dump_asm, compile_only=args.compile_only)
