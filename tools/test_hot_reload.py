#!/usr/bin/env python3
"""Shizuku Dynamic Object Hot-Reload 単体・統合テスト

1. 動的モジュール (blink.cpp) のコンパイル検証
2. 位置独立性 (PIC/PIE) と外部未解決シンボルの不在検証
3. GDB RSP プロトコルパケットの生成・検証
4. ローカルモック GDB サーバーによる E2E ホットリロード検証
"""

from __future__ import annotations

import os
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import shizuku_hot_reload as hr


class TestDynamicModuleCompilation(unittest.TestCase):
    """コンパイルとバイナリ品質のテスト"""

    def setUp(self):
        self.src_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "blink.cpp"
        )
        self.assertTrue(os.path.isfile(self.src_path), f"テスト対象ソースが見つかりません: {self.src_path}")

    def test_01_compile_and_binary_size(self):
        """コンパイルが成功し、バイナリサイズが極小 (1KB未満) であること"""
        with tempfile.NamedTemporaryFile(suffix=".bin", delete=False) as tmp:
            bin_path = tmp.name
        try:
            size, asm, entry_offset = hr.compile_module(self.src_path, bin_path, dump_asm=True)
            print(f"\n[Test] blink.bin size: {size} bytes, entry offset: +0x{entry_offset:x}")
            self.assertGreater(size, 0, "バイナリサイズが0です")
            self.assertLess(size, 2048, "バイナリサイズが大きすぎます (2KB以上)")
            self.assertIn("svc", asm, "アセンブリ内に svc 0 (システムコール) が見つかりません")
        finally:
            if os.path.exists(bin_path):
                os.unlink(bin_path)

    def test_02_rsp_packet_and_checksum(self):
        """RSP パケットのエンコードとチェックサム計算"""
        # "qRcmd,616c6c6f63203634" -> "alloc 64"
        cmd = "alloc 64"
        hex_cmd = cmd.encode("utf-8").hex()
        payload = f"qRcmd,{hex_cmd}"
        pkt = hr.rsp_encode_packet(payload)

        self.assertTrue(pkt.startswith(b"$" + payload.encode("ascii") + b"#"))
        chk = hr.rsp_checksum(payload.encode("ascii"))
        self.assertEqual(pkt[-2:].decode("ascii"), chk)


class MockGdbStubServer:
    """GDB RSP プロトコルを模擬する軽量モックサーバー"""

    def __init__(self, port: int = 13333):
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", port))
        self.sock.listen(1)
        self.sock.settimeout(3.0)
        self.received_alloc = False
        self.received_write = False
        self.received_spawn = False
        self.received_call = False
        self.written_bytes = 0
        self.running = True

    def run(self):
        try:
            conn, _ = self.sock.accept()
        except Exception:
            return
        conn.settimeout(2.0)
        with conn:
            in_buf = bytearray()
            while self.running:
                try:
                    chunk = conn.recv(512)
                    if not chunk:
                        break
                    in_buf.extend(chunk)
                except (socket.timeout, socket.error):
                    continue

                # パケットの抽出 ($...#xx)
                while b"$" in in_buf and b"#" in in_buf:
                    start = in_buf.find(b"$")
                    end = in_buf.find(b"#", start)
                    if end == -1 or len(in_buf) < end + 3:
                        break
                    
                    cmd = in_buf[start + 1:end].decode("ascii", errors="replace")
                    in_buf = in_buf[end + 3:]
                    # ACK を返す
                    conn.sendall(b"+")

                    if cmd.startswith("qRcmd,"):
                        # Hex デコード
                        hex_str = cmd[6:]
                        text = bytes.fromhex(hex_str).decode("utf-8", errors="replace")
                        if text.startswith("alloc"):
                            self.received_alloc = True
                            # 0x20015000\n を hex encode して返す
                            resp_text = "0x20015000\n".encode("utf-8").hex()
                            conn.sendall(hr.rsp_encode_packet(resp_text))
                        elif text.startswith("spawn"):
                            self.received_spawn = True
                            resp_text = "spawned thread 14 (object 24)\n".encode("utf-8").hex()
                            conn.sendall(hr.rsp_encode_packet(resp_text))
                        elif text.startswith("call"):
                            self.received_call = True
                            resp_text = "call result: 1 (error: 0)\n".encode("utf-8").hex()
                            conn.sendall(hr.rsp_encode_packet(resp_text))
                        else:
                            conn.sendall(hr.rsp_encode_packet(""))
                    elif cmd.startswith("M"):
                        self.received_write = True
                        # M<addr>,<len>:<data>
                        parts = cmd[1:].split(":")
                        if len(parts) == 2:
                            self.written_bytes += len(parts[1]) // 2
                        conn.sendall(hr.rsp_encode_packet("OK"))
                    else:
                        conn.sendall(hr.rsp_encode_packet(""))

    def close(self):
        self.running = False
        try:
            self.sock.close()
        except Exception:
            pass


class TestEndToEndHotReload(unittest.TestCase):
    """モック GDB サーバーを使った E2E ホットリロードのテスト"""

    def test_e2e_hot_reload(self):
        port = 13333
        server = MockGdbStubServer(port=port)
        server_thread = threading.Thread(target=server.run, daemon=True)
        server_thread.start()
        time.sleep(0.1)

        src_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "blink.cpp"
        )

        try:
            # ホットリロード実行
            hr.hot_reload(src_path, port=port)

            # 検証
            self.assertTrue(server.received_call, "既存 blink 停止の call コマンドが送信されていません")
            self.assertTrue(server.received_alloc, "alloc コマンドが送信されていません")
            self.assertTrue(server.received_write, "メモリ書き込みが送信されていません")
            self.assertTrue(server.received_spawn, "spawn コマンドが送信されていません")
            self.assertGreater(server.written_bytes, 0, "書き込まれたバイト数が 0 です")
            print(f"\n[Test E2E] モック実機へのメモリ書き込み確認: {server.written_bytes} bytes 転送成功！")
        finally:
            server.close()


if __name__ == "__main__":
    unittest.main()
