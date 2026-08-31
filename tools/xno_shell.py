#!/usr/bin/env python3
"""Shizuku OS Interactive Shell & Flash FS Manager

使用例:
  # 1. 対話型シリアルコンソールを開く
  python3 tools/xno_shell.py console

  # 2. C++ ソースをコンパイルして Flash FS へ転送し、即座にロードして起動
  python3 tools/xno_shell.py upload blink.cpp /bin/blink.bin --load

  # 3. 単発のシェルコマンドを実行
  python3 tools/xno_shell.py cmd "ls"
  python3 tools/xno_shell.py cmd "ps"
  python3 tools/xno_shell.py cmd "load /bin/blink.bin"
  python3 tools/xno_shell.py cmd "unload blink"
"""

from __future__ import annotations

import argparse
import getpass
import glob
import os
import subprocess
import sys
import tempfile
import time

# ---- ホスト依存のパス -------------------------------------------------------
# ★絶対パスを直書きしない。**公開リポジトリに $HOME (= ユーザー名) が載る**のと、
#   他のマシンで動かなくなるのが同じ 1 つの原因から来ている。環境変数で
#   上書きでき、無ければ既定値へ落とす形にしておけば両方まとめて消える。
SHIZUKU = os.environ.get(
    "SHIZUKU_ROOT",
    os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                 os.pardir, "Shizuku"),
)
SHIZUKU = os.path.normpath(SHIZUKU)
PICO_SDK_ROOT = os.environ.get(
    "PICO_SDK_ROOT", os.path.expanduser("~/.pico-sdk"))
# Bazel の出力ベースはユーザー名を含む (_bazel_<user>)。getuser() から組み立てる。
BAZEL_TMP = os.environ.get(
    "BAZEL_TMP", "/private/var/tmp/_bazel_" + getpass.getuser())


try:
    import serial
except ImportError:
    serial = None


def find_serial_port(preferred: str | None = None) -> str:
    if preferred and os.path.exists(preferred):
        return preferred
    # macOS の場合: CDC Channel 0 (/dev/cu.usbmodem101) がシェル
    ports = sorted(glob.glob("/dev/cu.usbmodem*"))
    if ports:
        return ports[0]
    return "/dev/cu.usbmodem101"


def compile_cpp_to_bin(src_path: str, out_bin: str) -> int:
    """C++ ソースを PIC/PIE Thumb-2 バイナリへコンパイル"""
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    include_dyn = SHIZUKU + "/internal_headers"
    include_pico = SHIZUKU + "/modules/pico_sdk_support/internal_headers"
    source_dir = SHIZUKU + "/source"

    gxx_candidates = [
        PICO_SDK_ROOT + "/toolchain/14_2_Rel1/bin/arm-none-eabi-g++",
        "/opt/homebrew/bin/arm-none-eabi-g++",
        "arm-none-eabi-g++",
    ]
    gxx = next((c for c in gxx_candidates if os.path.exists(c) or subprocess.run(["which", c], capture_output=True).returncode == 0), "arm-none-eabi-g++")

    with tempfile.NamedTemporaryFile(suffix=".elf", delete=False) as tmp_elf:
        elf_path = tmp_elf.name
    with tempfile.NamedTemporaryFile(mode="w", suffix=".ld", delete=False) as tmp_ld:
        tmp_ld.write("SECTIONS { . = 0x0; .text : { KEEP(*(.text.entry)) *(.text*) *(.rodata*) *(.data*) *(.bss*) } }\n")
        ld_path = tmp_ld.name

    gen_ids_dirs = glob.glob(
        BAZEL_TMP + "/*/execroot/_main/bazel-out/darwin_arm64-fastbuild/bin/external/shizuku+/generated_object_ids"
    ) + glob.glob(os.path.join(repo_root, "bazel-bin/external/shizuku+/generated_object_ids"))
    gen_ids = gen_ids_dirs[0] if gen_ids_dirs else ""

    cmd_compile = [
        gxx,
        "-mcpu=cortex-m33",
        "-mthumb",
        "-Os",
        "-fPIC",
        "-fPIE",
        "-ffreestanding",
        "-nostdlib",
        "-fno-exceptions",
        "-fno-rtti",
        f"-Wl,-T,{ld_path}",
        "-DSHIZUKU_DYNAMIC_MODULE=1",
        f"-I{repo_root}",
        f"-I{include_dyn}",
        f"-I{include_pico}",
        f"-I{source_dir}",
    ]
    if gen_ids:
        cmd_compile.append(f"-I{gen_ids}")
    cmd_compile.extend([src_path, "-o", elf_path])

    res = subprocess.run(cmd_compile, capture_output=True, text=True)
    if os.path.exists(ld_path):
        os.unlink(ld_path)
    if res.returncode != 0:
        if os.path.exists(elf_path):
            os.unlink(elf_path)
        sys.exit(f"コンパイルエラー:\n{res.stderr}")

    objcopy_candidates = [
        PICO_SDK_ROOT + "/toolchain/14_2_Rel1/bin/arm-none-eabi-objcopy",
        "/opt/homebrew/bin/arm-none-eabi-objcopy",
        "arm-none-eabi-objcopy",
    ]
    objcopy = next((c for c in objcopy_candidates if os.path.exists(c) or subprocess.run(["which", c], capture_output=True).returncode == 0), "arm-none-eabi-objcopy")

    cmd_copy = [objcopy, "-O", "binary", elf_path, out_bin]
    res_copy = subprocess.run(cmd_copy, capture_output=True, text=True)
    if os.path.exists(elf_path):
        os.unlink(elf_path)

    if res_copy.returncode != 0:
        sys.exit(f"objcopy エラー:\n{res_copy.stderr}")

    return os.path.getsize(out_bin)


class ShizukuShellClient:
    def __init__(self, port: str | None = None, baudrate: int = 115200, timeout: float = 1.0):
        if serial is None:
            sys.exit("エラー: pyserial が必要です (pip install pyserial)")
        self.port = port or find_serial_port()
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        self.ser.dtr = True
        self.ser.rts = True
        time.sleep(0.15)

    def send_command(self, cmd: str, wait_prompt: bool = True) -> str:
        self.ser.reset_input_buffer()
        self.ser.write(f" {cmd.strip()}\r\n".encode("utf-8"))
        self.ser.flush()

        if not wait_prompt:
            return ""

        buf = bytearray()
        t0 = time.time()
        while time.time() - t0 < self.timeout:
            chunk = self.ser.read(self.ser.in_waiting or 1)
            if chunk:
                buf.extend(chunk)
                if buf.endswith(b"shizuku> ") or b"\nshizuku> " in buf:
                    break
        return buf.decode("utf-8", errors="replace")

    def upload_file(self, dest_path: str, data: bytes) -> bool:
        size = len(data)
        self.ser.reset_input_buffer()
        cmd = f" upload {dest_path} {size}\r\n"
        self.ser.write(cmd.encode("utf-8"))
        self.ser.flush()

        # "READY <size>" を待つ
        buf = bytearray()
        t0 = time.time()
        ready = False
        while time.time() - t0 < 3.0:
            chunk = self.ser.read(self.ser.in_waiting or 1)
            if chunk:
                buf.extend(chunk)
                if b"READY" in buf:
                    ready = True
                    break

        if not ready:
            print(f"エラー: シェルから READY 応答がありませんでした: {buf.decode('utf-8', errors='replace')}")
            return False

        time.sleep(0.05)  # ファームウェア側の CR/LF ドレイン待ち (5ms sleep + マージン)
        self.ser.reset_input_buffer()  # READY 応答のエコーをクリア
        # 生バイナリデータを送信
        self.ser.write(data)
        self.ser.flush()

        # "UPLOAD_OK" を待つ
        buf = bytearray()
        t0 = time.time()
        ok = False
        while time.time() - t0 < 5.0:
            chunk = self.ser.read(self.ser.in_waiting or 1)
            if chunk:
                buf.extend(chunk)
                if b"UPLOAD_OK" in buf:
                    ok = True
                    break

        resp = buf.decode("utf-8", errors="replace")
        if ok:
            print(f"✨ アップロード成功: {dest_path} ({size} bytes)")
            return True
        else:
            print(f"エラー: アップロード失敗: {resp}")
            return False

    def interactive_console(self):
        print(f"=== Shizuku Interactive Console ({self.port}) ===")
        print("終了するには Ctrl+C を押してください。\n")
        self.ser.write(b"\r\n")
        self.ser.flush()

        import threading

        running = True

        def reader():
            while running:
                try:
                    chunk = self.ser.read(self.ser.in_waiting or 1)
                    if chunk:
                        sys.stdout.write(chunk.decode("utf-8", errors="replace"))
                        sys.stdout.flush()
                except Exception:
                    break

        t = threading.Thread(target=reader, daemon=True)
        t.start()

        try:
            while True:
                line = input()
                self.ser.write((line + "\r\n").encode("utf-8"))
                self.ser.flush()
        except (KeyboardInterrupt, EOFError):
            print("\nコンソールを終了します。")
        finally:
            running = False
            self.ser.close()

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    subparsers = parser.add_subparsers(dest="subcommand", help="サブコマンド")

    # upload サブコマンド
    p_up = subparsers.add_parser("upload", help="C++ ソースをコンパイルして Flash FS へ転送")
    p_up.add_argument("source", help="ソースコード (.cpp) またはバイナリ (.bin)")
    p_up.add_argument("dest", help="Flash FS 上の保存先パス (例: /bin/blink.bin)")
    p_up.add_argument("--load", action="store_true", help="転送後に自動で load を実行")
    p_up.add_argument("--port", type=str, default=None, help="シリアルポート")

    # cmd サブコマンド
    p_cmd = subparsers.add_parser("cmd", help="単発シェルコマンドの実行")
    p_cmd.add_argument("command", help="実行するシェルコマンド (例: ls, ps, load /bin/blink.bin)")
    p_cmd.add_argument("--port", type=str, default=None, help="シリアルポート")

    # console サブコマンド
    p_con = subparsers.add_parser("console", help="対話型ターミナルコンソールを開く")
    p_con.add_argument("--port", type=str, default=None, help="シリアルポート")

    args = parser.parse_args()
    if not args.subcommand:
        parser.print_help()
        return

    port = find_serial_port(getattr(args, "port", None))

    if args.subcommand == "console":
        client = ShizukuShellClient(port=port)
        client.interactive_console()

    elif args.subcommand == "cmd":
        client = ShizukuShellClient(port=port)
        resp = client.send_command(args.command)
        print(resp)
        client.close()

    elif args.subcommand == "upload":
        src = args.source
        if src.endswith(".cpp"):
            with tempfile.NamedTemporaryFile(suffix=".bin", delete=False) as tmp:
                bin_path = tmp.name
            print(f"[1/3] コンパイル中: {src} -> {bin_path}...")
            t0 = time.perf_counter()
            size = compile_cpp_to_bin(src, bin_path)
            t_comp = time.perf_counter() - t0
            print(f"      コンパイル完了: {size} bytes ({t_comp * 1000:.1f}ms)")
            with open(bin_path, "rb") as f:
                data = f.read()
            if os.path.exists(bin_path):
                os.unlink(bin_path)
        else:
            with open(src, "rb") as f:
                data = f.read()

        print(f"[2/3] Flash FS へ転送中 ({port} -> {args.dest})...")
        client = ShizukuShellClient(port=port)
        ok = client.upload_file(args.dest, data)
        if not ok:
            client.close()
            sys.exit(1)

        if args.load:
            print(f"[3/3] ロード＆起動中: {args.dest}...")
            resp = client.send_command(f"load {args.dest}")
            print(resp)

        client.close()


if __name__ == "__main__":
    main()
