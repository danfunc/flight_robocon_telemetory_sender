// ===========================================================================
//  stream_selftest — ストリーム API (include/stream.hpp) の end-to-end 実動確認
// ===========================================================================
//  STREAM_TEST オブジェクトに producer / consumer 2 スレッドを置き、共有メモリ
//  ストリーム 1 本で繋ぐ。制御プレーン (create / open / bind の SVC) と
//  データプレーン (push / pop のライブラリ) が実機で動くことを確認する。
//  producer は連番 seq を push、consumer は open→pop して単調増加 (lossy ドロップで
//  前進はしても後退しない) を検証し、件数 / lost / 順序 OK を 1 定期 printf する。
//  core0 上で協調的に走る (affinity 既定 = core0)。SHIZU_STREAM_SELFTEST で有効化。
// ===========================================================================
#include <cstdint>
#include <cstdio>
#include <kernel.hpp>
#include <obj_api.hpp>
#include <object_id.hpp>
#include <pico/stdlib.h>
#include <stream.hpp>

namespace shizu {
namespace {
namespace st = stream;

struct test_rec_t {
  uint32_t seq;
};
constexpr uint32_t TEST_STREAM_ID = 0;
// connect (DMA ポンプ) テスト用: SRC → [カーネル DMA] → DST の 2 本。
constexpr uint32_t CONN_SRC_ID = 1;
constexpr uint32_t CONN_DST_ID = 2;

// 共有メモリ (既定 lossy)。producer(owner) と consumer が同一実体を参照する。
st::storage<test_rec_t, 64> g_test_stream;
// connect テスト: LOSSLESS にして「DMA ポンプ経由でも 1 件も落ちない・順序が保たれる」
// を厳密に検証する (lossy だと落ちが正常系に混ざり判定が緩む)。
st::storage<test_rec_t, 64, st::LOSSLESS> g_conn_src;
st::storage<test_rec_t, 64, st::LOSSLESS> g_conn_dst;

// producer: ストリームを登録して連番を push し続ける。
void stream_test_producer() {
  auto e = st::create(TEST_STREAM_ID, &g_test_stream.desc);
  if (e.is_err())
    printf("[STREAMTEST] create err=%lu\n", (unsigned long)e.raw());
  st::bind(TEST_STREAM_ID, st::role::PRODUCER);
  auto tx = g_test_stream.hdl();
  uint32_t seq = 0;
  while (true) {
    test_rec_t r{seq++};
    tx.push(r);
    obj_api::yield();
  }
}

// consumer: producer が create するまで open を待ち、pop して単調性を検証する。
void stream_test_consumer() {
  st::handle<test_rec_t> rx;
  while (!rx.valid()) { // producer の create を待つ (discovery)
    rx = st::open<test_rec_t>(TEST_STREAM_ID);
    obj_api::yield();
  }
  st::bind(TEST_STREAM_ID, st::role::CONSUMER);
  uint32_t got = 0, lost = 0, last = 0, next_print = 5000;
  bool started = false, order_ok = true;
  while (true) {
    test_rec_t r;
    while (rx.pop(&r, &lost)) {
      // lossy: ドロップで seq が飛ぶ (前進) のは正常。後退したら異常。
      if (started && r.seq <= last)
        order_ok = false;
      last = r.seq;
      started = true;
      got++;
    }
    if (got >= next_print) {
      printf("[STREAMTEST] core=%u got=%lu lost=%lu last_seq=%lu order_ok=%d\n",
             get_core_num(), (unsigned long)got, (unsigned long)lost,
             (unsigned long)last, order_ok ? 1 : 0);
      next_print += 5000;
    }
    obj_api::yield();
  }
}

// connect producer: SRC/DST を登録して connect(SRC→DST) し、SRC へ連番を push。
// 以後このオブジェクトは DST に一切触らない — レコードはカーネルの DMA ポンプ
// だけで DST へ渡る (= 「接続中はオブジェクトによるコピー不要」の実動確認)。
void conn_test_producer() {
  st::create(CONN_SRC_ID, &g_conn_src.desc);
  st::create(CONN_DST_ID, &g_conn_dst.desc);
  st::bind(CONN_SRC_ID, st::role::PRODUCER);
  auto e = st::connect(CONN_SRC_ID, CONN_DST_ID);
  printf("[CONNTEST] connect err=%lu (0=OK)\n", (unsigned long)e.raw());
  auto tx = g_conn_src.hdl();
  uint32_t seq = 0;
  while (true) {
    test_rec_t r{seq};
    if (tx.push(r)) // LOSSLESS: 満杯 (ポンプ待ち) なら失敗 → yield して再試行
      seq++;
    obj_api::yield();
  }
}

// connect consumer: DST 側だけを見る。LOSSLESS × DMA ポンプなので「欠落ゼロ +
// 連番完全一致」が合格条件 (通常ストリームの単調性検証より厳しい)。
void conn_test_consumer() {
  st::handle<test_rec_t> rx;
  while (!rx.valid()) {
    rx = st::open<test_rec_t>(CONN_DST_ID);
    obj_api::yield();
  }
  st::bind(CONN_DST_ID, st::role::CONSUMER);
  uint32_t got = 0, expect = 0, bad = 0, next_print = 5000;
  while (true) {
    test_rec_t r;
    while (rx.pop(&r)) {
      if (r.seq != expect)
        bad++;
      expect = r.seq + 1;
      got++;
    }
    if (got >= next_print) {
      printf("[CONNTEST] got=%lu bad=%lu (bad=0 = DMA pump PASS)\n",
             (unsigned long)got, (unsigned long)bad);
      next_print += 5000;
    }
    obj_api::yield();
  }
}

} // namespace

// core0 (kernel_object_main) から呼ぶ。STREAM_TEST オブジェクト + producer/consumer
// スレッドを作る (両者 affinity 既定 = core0 で協調実行)。connect テストは同じ
// オブジェクトにもう 1 対: SRC→DST をカーネル DMA ポンプで直結して検証する。
void stream_selftest_launch() {
  // create_object + async_call ×2 (producer/consumer)。tid は自動確保。
  FOR_KERNEL_OBJECT::create_object((uint32_t)object_ids::STREAM_TEST);
  FOR_KERNEL_OBJECT::async_call((uint32_t)object_ids::STREAM_TEST,
                                (method_t)stream_test_producer);
  FOR_KERNEL_OBJECT::async_call((uint32_t)object_ids::STREAM_TEST,
                                (method_t)stream_test_consumer);
  FOR_KERNEL_OBJECT::async_call((uint32_t)object_ids::STREAM_TEST,
                                (method_t)conn_test_producer);
  FOR_KERNEL_OBJECT::async_call((uint32_t)object_ids::STREAM_TEST,
                                (method_t)conn_test_consumer);
}

} // namespace shizu
