#!/usr/bin/env python3
"""Flash FS 上の動的 ELF / バイナリモジュール ホットスワップツール

Flash FS（/bin/...）上に配置されたモジュール（ELFまたはPIC生バイナリ）を
実行中にカーネルを再起動することなく安全に差し替えて即座に新アルゴリズムを起動します。

使用例:
  # 1. user_apps/pulse_blink.cpp をコンパイルし、ELF 形式で Flash FS へアップロードしてホットスワップ
  python3 tools/hotswap_module.py --src user_apps/pulse_blink.cpp --dest /bin/pulse_blink.elf --format elf

  # 2. 既存の .bin または .elf ファイルを直接ホットスワップ
  python3 tools/hotswap_module.py --file bazel-bin/user_apps/pulse_blink.bin --dest /bin/pulse_blink.bin
"""

from __future__ import annotations

import argparse
import glob
import os
import subprocess
import sys
import time

try:
    import serial
except ImportError:
    serial = None


def find_serial_port(preferred: str | None = None) -> str:
    if preferred and os.path.exists(preferred):
        return preferred
    ports = sorted(glob.glob("/dev/cu.usbmodem*"))
    if ports:
        return ports[0]
    return "/dev/cu.usbmodem101"


def compile_to_elf_or_bin(src_path: str, fmt: str = "elf") -> bytes:
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    include_dyn = "/Users/ishigakiyua/github/Shizuku/internal_headers"
    include_pico = "/Users/ishigakiyua/github/Shizuku/modules/pico_sdk_support/internal_headers"
    source_dir = "/Users/ishigakiyua/github/Shizuku/source"

    gxx_candidates = [
        "/Users/ishigakiyua/.pico-sdk/toolchain/14_2_Rel1/bin/arm-none-eabi-g++",
        "/opt/homebrew/bin/arm-none-eabi-g++",
        "arm-none-eabi-g++",
    ]
    gxx = next((c for c in gxx_candidates if os.path.exists(c) or subprocess.run(["which", c], capture_output=True).returncode == 0), "arm-none-eabi-g++")
    ld_script = os.path.join(repo_root, "user_apps/dyn_module.ld")

    import tempfile
    with tempfile.NamedTemporaryFile(suffix=".elf", delete=False) as tmp_elf:
        elf_path = tmp_elf.name

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
        f"-Wl,-T,{ld_script}",
        "-DSHIZUKU_DYNAMIC_MODULE=1",
        f"-I{repo_root}",
        f"-I{include_dyn}",
        f"-I{include_pico}",
        f"-I{source_dir}",
        src_path,
        "-o",
        elf_path,
    ]

    res = subprocess.run(cmd_compile, capture_output=True, text=True)
    if res.returncode != 0:
        if os.path.exists(elf_path):
            os.unlink(elf_path)
        sys.exit(f"コンパイルエラー:\n{res.stderr}")

    if fmt == "elf":
        with open(elf_path, "rb") as f:
            data = f.read()
        os.unlink(elf_path)
        return data

    # bin 形式
    with tempfile.NamedTemporaryFile(suffix=".bin", delete=False) as tmp_bin:
        bin_path = tmp_bin.name

    objcopy_candidates = [
        "/Users/ishigakiyua/.pico-sdk/toolchain/14_2_Rel1/bin/arm-none-eabi-objcopy",
        "/opt/homebrew/bin/arm-none-eabi-objcopy",
        "arm-none-eabi-objcopy",
    ]
    objcopy = next((c for c in objcopy_candidates if os.path.exists(c) or subprocess.run(["which", c], capture_output=True).returncode == 0), "arm-none-eabi-objcopy")

    res_copy = subprocess.run([objcopy, "-O", "binary", elf_path, bin_path], capture_output=True, text=True)
    os.unlink(elf_path)
    if res_copy.returncode != 0:
        if os.path.exists(bin_path):
            os.unlink(bin_path)
        sys.exit(f"objcopy エラー:\n{res_copy.stderr}")

    with open(bin_path, "rb") as f:
        data = f.read()
    os.unlink(bin_path)
    return data


def perform_hotswap(port: str, dest_path: str, data: bytes, target_name: str = "blink", core_id: int = 0):
    if serial is None:
        sys.exit("pyserial が必要です: pip install pyserial")

    print(f"🔌 シリアルポート {port} に接続中...")
    ser = serial.Serial(port, 115200, timeout=1.0)
    ser.dtr = True
    ser.rts = True
    time.sleep(0.15)

    def send_cmd(cmd: str) -> str:
        ser.reset_input_buffer()
        ser.write(f" {cmd.strip()}\r\n".encode("utf-8"))
        ser.flush()
        buf = bytearray()
        t0 = time.time()
        while time.time() - t0 < 1.5:
            chunk = ser.read(ser.in_waiting or 1)
            if chunk:
                buf.extend(chunk)
                if buf.endswith(b"shizuku> ") or b"\nshizuku> " in buf:
                    break
        return buf.decode("utf-8", errors="replace")

    # 1. 接続確認
    resp = send_cmd("help")
    if "Shizuku" not in resp and "shizuku>" not in resp:
        print("⚠️ シェル応答待機中...")

    # 2. Flash FS へのアップロード
    size = len(data)
    print(f"📦 Flash FS へ書き込み中: {dest_path} ({size} bytes)...")
    ser.reset_input_buffer()
    ser.write(f" upload {dest_path} {size}\r\n".encode("utf-8"))
    ser.flush()

    buf = bytearray()
    t0 = time.time()
    ready = False
    while time.time() - t0 < 3.0:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf.extend(chunk)
            if b"READY" in buf:
                ready = True
                break

    if not ready:
        sys.exit(f"❌ upload READY 応答がありませんでした: {buf.decode('utf-8', errors='replace')}")

    time.sleep(0.05)
    ser.reset_input_buffer()
    ser.write(data)
    ser.flush()

    buf = bytearray()
    t0 = time.time()
    ok = False
    while time.time() - t0 < 5.0:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf.extend(chunk)
            if b"UPLOAD_OK" in buf:
                ok = True
                break

    if not ok:
        sys.exit(f"❌ アップロード失敗: {buf.decode('utf-8', errors='replace')}")
    print("✅ アップロード完了 (Flash FS に保存されました)")

    # 3. ホットスワップ実行
    print(f"🔄 ホットスワップ実行: '{target_name}' を停止し '{dest_path}' を起動中...")
    resp_swap = send_cmd(f"swap {target_name} {dest_path} {core_id}")
    print(resp_swap)

    # 4. 実行スレッド一覧取得
    time.sleep(0.3)
    resp_ps = send_cmd("ps")
    print(resp_ps)


def main():
    parser = argparse.ArgumentParser(description="Dynamic Module Hot-Swap Tool")
    parser.add_argument("--src", help="C++ source file to compile and hot-swap")
    parser.add_argument("--file", help="Pre-built .bin or .elf file to hot-swap")
    parser.add_argument("--dest", default="/bin/pulse_blink.elf", help="Flash FS destination path (default: /bin/pulse_blink.elf)")
    parser.add_argument("--target", default="blink", help="Target object name to replace (default: blink)")
    parser.add_argument("--format", choices=["elf", "bin"], default="elf", help="Module format (elf or bin)")
    parser.add_argument("--port", help="Serial port")
    parser.add_argument("--core", type=int, default=0, help="Core ID (0 or 1)")

    args = parser.parse_args()

    port = find_serial_port(args.port)

    if args.src:
        print(f"🛠️  コンパイル中: {args.src} -> {args.format.upper()} 形式...")
        data = compile_to_elf_or_bin(args.src, fmt=args.format)
    elif args.file:
        with open(args.file, "rb") as f:
            data = f.read()
    else:
        # デフォルトで user_apps/pulse_blink.cpp を使用
        default_src = "user_apps/pulse_blink.cpp"
        if os.path.exists(default_src):
            print(f"🛠️  デフォルトソースをコンパイル: {default_src} -> {args.format.upper()} 形式...")
            data = compile_to_elf_or_bin(default_src, fmt=args.format)
        else:
            sys.exit("エラー: --src または --file を指定してください")

    perform_hotswap(port, args.dest, data, target_name=args.target, core_id=args.core)


if __name__ == "__main__":
    main()
