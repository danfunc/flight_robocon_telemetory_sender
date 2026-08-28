#!/usr/bin/env python3
"""Flash FS イメージ生成ツール (A/B ピンポン電源断保護対応)

指定されたバイナリ群を Shizuku / XNO の Flash FS 構造（0x00200000〜）に従ってパックし、
生バイナリ（.bin）または UF2（.uf2）として出力する。

使用例:
  python3 tools/make_flash_fs.py --out flash_fs.bin \
      /bin/blink.bin=path/to/blink.bin \
      /bin/user_app.bin=path/to/user_app.bin
"""

import argparse
import os
import struct
import sys
import zlib

FS_MAGIC = 0x584E4F46   # 'XNOF' (file_entry)
DIR_MAGIC = 0x584E4F44  # 'XNOD' (dir_header)
SECTOR_SIZE = 4096
MAX_FILES = 64
MAX_FILENAME_LEN = 32
DIR_BANKS = 2           # Bank A (0x0000), Bank B (0x1000)
DATA_OFFSET = DIR_BANKS * SECTOR_SIZE  # 0x2000 (8192 bytes)


def pack_flash_fs(file_map: list[tuple[str, str]]) -> bytes:
    """Flash FS の生イメージ（A/B ディレクトリ + 各データセクタ）を構築する。"""
    if len(file_map) > MAX_FILES:
        raise ValueError(f"Too many files ({len(file_map)} > {MAX_FILES})")

    # 64 エントリ分の file_entry 配列バッファ (64 * 60 = 3840 bytes)
    entries_buf = bytearray(b"\x00" * (MAX_FILES * 60))
    data_sectors = bytearray()

    current_data_offset = DATA_OFFSET

    for idx, (vpath, src_path) in enumerate(file_map):
        if not os.path.exists(src_path):
            raise FileNotFoundError(f"Source file not found: {src_path}")

        with open(src_path, "rb") as f:
            data = f.read()

        size = len(data)
        crc32 = zlib.crc32(data) & 0xFFFFFFFF

        # 32 バイト固定ファイル名 (null 終端)
        name_encoded = vpath.encode("utf-8")[: MAX_FILENAME_LEN - 1]
        name_padded = name_encoded + b"\x00" * (MAX_FILENAME_LEN - len(name_encoded))

        # 4KB アラインでデータ領域を確保
        flash_offset = current_data_offset
        flags = 0x03  # VALID | EXEC

        # struct file_entry (60 bytes)
        entry_bytes = struct.pack(
            "<I32sIIII2I",
            FS_MAGIC,
            name_padded,
            size,
            crc32,
            flash_offset,
            flags,
            0,
            0,
        )

        entry_offset = idx * 60
        entries_buf[entry_offset : entry_offset + len(entry_bytes)] = entry_bytes

        # データセクタを 4KB 単位で詰める
        data_sectors.extend(data)
        pad_len = (SECTOR_SIZE - (size % SECTOR_SIZE)) % SECTOR_SIZE
        if pad_len > 0:
            data_sectors.extend(b"\xFF" * pad_len)

        current_data_offset += size + pad_len

    # Bank A (Bank 0) ヘッダ作成
    entries_crc32 = zlib.crc32(entries_buf) & 0xFFFFFFFF
    header_bytes = struct.pack(
        "<IIII",
        DIR_MAGIC,
        1,  # sequence = 1
        len(file_map),  # file_count
        entries_crc32,  # crc32
    )

    bank_a = bytearray(b"\xFF" * SECTOR_SIZE)
    bank_a[0 : len(header_bytes)] = header_bytes
    bank_a[len(header_bytes) : len(header_bytes) + len(entries_buf)] = entries_buf

    # Bank B (Bank 1) は未書き込み (0xFF)
    bank_b = bytearray(b"\xFF" * SECTOR_SIZE)

    return bytes(bank_a + bank_b + data_sectors)


def main():
    parser = argparse.ArgumentParser(description="Generate Flash FS image for XNO")
    parser.add_argument("--out", required=True, help="Output binary path (.bin)")
    parser.add_argument("files", nargs="+", help="Files to pack in format /vpath=local_path")
    args = parser.parse_args()

    file_map = []
    for item in args.files:
        if "=" not in item:
            raise ValueError(f"Invalid format: '{item}'. Expected /vpath=local_path")
        vpath, local_path = item.split("=", 1)
        file_map.append((vpath, local_path))

    fs_image = pack_flash_fs(file_map)
    with open(args.out, "wb") as f:
        f.write(fs_image)

    print(f"Generated Flash FS image: {args.out} ({len(fs_image)} bytes, {len(file_map)} files, A/B dual-bank)")


if __name__ == "__main__":
    main()
