#!/usr/bin/env python3
"""XNOR (チャンク単位再送) のワイヤ形式と、受け手の再同期の試験。

★★**この試験は実機に持ち込む前にバグを 1 つ捕まえている。** 受信部を
  ホスト側へ写して無傷の入力を流したところ、毎ラウンド半分しか受理されず、
  CSEEK の窓が「4 バイト目を埋めた回に magic を照合していない」ことが
  分かった (C 側も同じロジックだったので同じバグ)。実機だけで追っていたら
  何時間も溶かしていた。**だから消さずに残す。**

★device 側 (Shizuku の ota.cpp) の受信部をここに写してある。**片方だけ直すと
  意味が無くなる**ので、ota.cpp の CSEEK/CHDR/CDATA を触ったらここも直すこと。
  写しである以上ズレうるが、ズレたまま気づかないよりは、写しがあって
  「両方直す」を意識できるほうがよい。

    python3 tools/test_ota_framing.py          # 単体で走る
    bazel test //tools:test_ota_framing        # Bazel から
"""
import os
import random
import struct
import sys
import unittest
import zlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ota_send as o  # noqa: E402

SECTOR = 4096
QUERY_SEQ = 0xFFFF


class Device:
    """ota.cpp の XNOR 受信部の写し。CSEEK/CHDR/CDATA の遷移を同じ順序で辿る。"""

    def __init__(self, total):
        self.total = total
        self.nchunks = (total + SECTOR - 1) // SECTOR
        self.ok = set()
        self.state = "CSEEK"
        self.win = bytearray(4)
        self.win_filled = 0
        self.hdr = bytearray(16)
        self.hdr_got = 0
        self.seq = self.len = self.crc = 0
        self.buf = bytearray()
        self.bad = 0

    def feed(self, data: bytes):
        i, n = 0, len(data)
        while i < n:
            if self.state == "CSEEK":
                self.win[0:3] = self.win[1:4]
                self.win[3] = data[i]
                i += 1
                # ★★窓が満ちた**その回に**照合する。「満ちるまで continue」に
                #   すると 4 バイト目を入れた回を飛ばし、その窓は次のバイトで
                #   先頭が押し出されて二度と一致しない = チャンクを 1 個おきに
                #   取り逃がす。これが実機前に捕まえたバグ。
                if self.win_filled < 4:
                    self.win_filled += 1
                if self.win_filled < 4:
                    continue
                if bytes(self.win) != o.CHUNK_MAGIC:
                    continue
                self.hdr[0:4] = o.CHUNK_MAGIC
                self.hdr_got = 4
                self.state = "CHDR"
                continue
            if self.state == "CHDR":
                while i < n and self.hdr_got < 16:
                    self.hdr[self.hdr_got] = data[i]
                    self.hdr_got += 1
                    i += 1
                if self.hdr_got < 16:
                    return
                self.seq, self.len = struct.unpack("<HH", self.hdr[4:8])
                self.crc = struct.unpack("<I", self.hdr[8:12])[0]
                hdr_ok = o._crc16_ccitt(bytes(self.hdr[:14])) == \
                    struct.unpack("<H", self.hdr[14:16])[0]
                self.win_filled = 0
                self.hdr_got = 0
                if hdr_ok and self.seq == QUERY_SEQ and self.len == 0:
                    self.state = "CSEEK"
                    continue
                if (not hdr_ok) or self.seq >= self.nchunks or self.len == 0 \
                        or self.len > SECTOR + 256:
                    # ヘッダ破損 / データ中の偶然の 'XNCK'。探索へ戻るだけ。
                    self.bad += 1
                    self.state = "CSEEK"
                    continue
                self.buf = bytearray()
                self.state = "CDATA"
                continue
            if self.state == "CDATA":
                take = min(self.len - len(self.buf), n - i)
                self.buf += data[i:i + take]
                i += take
                if len(self.buf) < self.len:
                    return
                if (zlib.crc32(bytes(self.buf)) & 0xFFFFFFFF) != self.crc:
                    self.bad += 1          # 化けた。記録だけして先へ
                else:
                    at = self.seq * SECTOR
                    want = min(SECTOR, self.total - at)
                    try:
                        raw = zlib.decompressobj(-12).decompress(bytes(self.buf))
                    except zlib.error:
                        raw = b""
                    if len(raw) == want:
                        self.ok.add(self.seq)
                    else:
                        self.bad += 1
                self.win_filled = 0
                self.hdr_got = 0
                self.state = "CSEEK"
                continue

    def missing(self):
        return sorted(set(range(self.nchunks)) - self.ok)


def corrupt(data: bytes, ber: float, rng) -> bytes:
    """1 ビットあたり ber の確率で反転させる (物理的なビット化けの模擬)。"""
    b = bytearray(data)
    nbits = len(b) * 8
    k = min(nbits, max(0, int(rng.gauss(nbits * ber, (nbits * ber) ** 0.5))))
    for _ in range(k):
        pos = rng.randrange(nbits)
        b[pos >> 3] ^= 1 << (pos & 7)
    return bytes(b)


def run_campaign(image, ber, seed=0, max_rounds=5):
    """(ラウンド数, 送信総量, 収束したか) を返す。"""
    rng = random.Random(seed)
    chunks = o.deflate_chunk_list(image)
    dev = Device(len(image))
    body = o.chunked_body(chunks)
    dev.feed(corrupt(body, ber, rng))
    sent = len(body)
    for rnd in range(1, max_rounds + 1):
        miss = dev.missing()
        if not miss:
            return rnd - 1, sent, True
        if rnd == max_rounds:
            return rnd, sent, False
        rb = o.chunked_body(chunks, miss)
        sent += len(rb)
        dev.feed(corrupt(rb, ber, rng))
    return max_rounds, sent, False


# ★実物の像は使わない (ビルド成果物に依存すると試験が環境で落ちる)。
#   deflate が効く程度に規則性のある擬似データを決め打ちの種で作る。
def make_image(nbytes=445312, seed=12345):
    rng = random.Random(seed)
    out = bytearray()
    while len(out) < nbytes:
        out += bytes(rng.randrange(256) for _ in range(16)) * rng.randrange(1, 40)
    return bytes(out[:nbytes])


class TestWireFormat(unittest.TestCase):
    def test_crc16_known_vector(self):
        # CRC-16/CCITT-FALSE。★device 側 (ota.cpp の crc16_ccitt) と一致させる
        #   ための固定点。ここがズレると全チャンクがヘッダ検査で落ちる。
        self.assertEqual(o._crc16_ccitt(b"123456789"), 0x29B1)

    def test_chunk_frame_layout(self):
        f = o.chunk_frame(7, b"hello")
        self.assertEqual(len(f), 16 + 5)
        magic, seq, ln, c32, rsv, c16 = struct.unpack("<4sHHIHH", f[:16])
        self.assertEqual((magic, seq, ln, rsv), (b"XNCK", 7, 5, 0))
        self.assertEqual(c32, zlib.crc32(b"hello") & 0xFFFFFFFF)
        self.assertEqual(c16, o._crc16_ccitt(f[:14]))

    def test_control_frames(self):
        q = o.query_frame()
        self.assertEqual(len(q), 16)
        self.assertEqual(struct.unpack("<H", q[4:6])[0], o.QUERY_SEQ)
        r = o.reset_frame()
        self.assertEqual(struct.unpack("<H", r[4:6])[0], o.RESET_SEQ)


class TestNeedParsing(unittest.TestCase):
    def test_plain(self):
        n, seqs, done = o.parse_need(
            "NEED n=2 of=109 ok=107 bad=1 r=1\nNEEDSEQ 3,17\nNEEDEND")
        self.assertEqual((n, seqs, done), (2, [3, 17], True))

    def test_xiao_prefix_and_leading_garbage(self):
        # ★XIAO は中継した行に "[UART-RX] " を前置し、中継の切り替わりで残った
        #   バイトが行頭に食い込むことがある。実機で "dsNEED n=2 ..." を観測。
        n, seqs, done = o.parse_need(
            "[UART-RX] dsNEED n=2 of=109 ok=107 bad=1 r=1\n"
            "[UART-RX] NEEDSEQ 3,17\n[UART-RX] NEEDEND")
        self.assertEqual((n, seqs, done), (2, [3, 17], True))

    def test_ranges(self):
        # ★ビット化けは連続したチャンクをまとめて落とすので、一覧は範囲で来る。
        n, seqs, done = o.parse_need(
            "NEED n=5 of=109 ok=104 bad=0 r=1\nNEEDSEQ 39-42,98\nNEEDEND")
        self.assertEqual((n, seqs, done), (5, [39, 40, 41, 42, 98], True))

    def test_incomplete_is_not_trusted(self):
        # ★NEEDEND が見えなければ n を信用してはいけない。一覧が途中で切れて
        #   いるのに信じると、欠けたまま commit へ進む。
        _, _, done = o.parse_need("NEED n=3 of=109 ok=106 bad=0 r=1\nNEEDSEQ 3")
        self.assertFalse(done)


class TestConvergence(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.image = make_image()

    def test_clean_input_accepts_everything(self):
        """★★これが CSEEK の窓バグを捕まえた試験。

        無傷の入力なら再送は 1 回も要らない。バグがあると毎ラウンド半分ずつ
        しか受理されず、欠損が半減していくだけで収束しない。
        """
        rounds, _, ok = run_campaign(self.image, 0.0)
        self.assertTrue(ok)
        self.assertEqual(rounds, 0)

    def test_no_chunk_is_skipped_on_clean_input(self):
        """1 個おきの取りこぼしを直接見る (上の試験の意図を明示する)。"""
        chunks = o.deflate_chunk_list(self.image)
        dev = Device(len(self.image))
        dev.feed(o.chunked_body(chunks))
        self.assertEqual(dev.missing(), [])
        self.assertEqual(dev.bad, 0)

    def test_bit_errors_converge(self):
        for ber in (1e-7, 1e-6, 5e-6):
            with self.subTest(ber=ber):
                _, _, ok = run_campaign(self.image, ber, seed=1)
                self.assertTrue(ok, f"BER={ber} で収束しなかった")

    def test_corruption_is_localised(self):
        """★化けたチャンクだけが欠けること (以降が巻き添えにならないこと)。

        これが成り立たないと「失敗した seq だけ再送」が意味を失い、
        「最初の 1 個が化けたら以降全部」に退化する = 自己同期ヘッダの存在意義。
        """
        chunks = o.deflate_chunk_list(self.image)
        body = bytearray(o.chunked_body(chunks))
        # 先頭付近のチャンクのペイロードを 1 ビット壊す
        body[100] ^= 0x01
        dev = Device(len(self.image))
        dev.feed(bytes(body))
        miss = dev.missing()
        self.assertLessEqual(len(miss), 2, f"巻き添えが大きい: {miss[:10]}")

    def test_query_frame_is_transparent(self):
        """問い合わせフレームが混ざってもチャンクの受理を妨げないこと。"""
        chunks = o.deflate_chunk_list(self.image)
        body = o.chunked_body(chunks[:5]) + o.query_frame() + \
            o.chunked_body(chunks, range(5, len(chunks)))
        dev = Device(len(self.image))
        dev.feed(body)
        self.assertEqual(dev.missing(), [])


if __name__ == "__main__":
    unittest.main(verbosity=2)
