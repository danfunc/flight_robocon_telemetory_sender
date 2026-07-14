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

// 共有メモリ (既定 lossy)。producer(owner) と consumer が同一実体を参照する。
st::storage<test_rec_t, 64> g_test_stream;

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

} // namespace

// core0 (kernel_object_main) から呼ぶ。STREAM_TEST オブジェクト + producer/consumer
// スレッドを作る (両者 affinity 既定 = core0 で協調実行)。
void stream_selftest_launch() {
  // create_object + async_call ×2 (producer/consumer)。tid は自動確保。
  FOR_KERNEL_OBJECT::create_object((uint32_t)object_ids::STREAM_TEST);
  FOR_KERNEL_OBJECT::async_call((uint32_t)object_ids::STREAM_TEST,
                                (method_t)stream_test_producer);
  FOR_KERNEL_OBJECT::async_call((uint32_t)object_ids::STREAM_TEST,
                                (method_t)stream_test_consumer);
}

} // namespace shizu
