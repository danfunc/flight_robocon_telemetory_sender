// ===========================================================================
//  logger — 出ていく行の合流点
// ===========================================================================
//  設計の理由は logger.hpp 冒頭。
#include "logger.hpp"
#include "shizuku/kernel.hpp"
#include "shizuku/object_api.hpp"
#include "shizuku/stream.hpp"
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace logger {
namespace {

using ARCH = shizuku::KERNEL::ARCH;
using BOARD = shizuku::KERNEL::BOARD;

struct api_result {
  uintptr_t error;
  uintptr_t value;
};

api_result api(shizuku::object_api number, uintptr_t a1 = 0, uintptr_t a2 = 0,
               uintptr_t a3 = 0) {
  const auto result = ARCH::syscall((uintptr_t)number, a1, a2, a3);
  return {result.error, result.value};
}

uintptr_t export_method(method m, uintptr_t entry) {
  return api(shizuku::object_api::EXPORT_METHOD, (uintptr_t)m, entry).error;
}

// ---- 出口: 束ねた 1 本 --------------------------------------------------------
// ★容量は入力より大きめに取る。ここが詰まると全ソースの行が落ちるので、
//   ble_uart が notify を出す間 (接続直後や CI が伸びた瞬間) の凸を吸わせる。
shizuku::stream::storage<frame_t, 48> g_out;
uintptr_t g_out_id = 0;

// ---- 入口 --------------------------------------------------------------------
struct input {
  uintptr_t stream_id;
  uint32_t priority;
  shizuku::stream::handle<frame_t> handle;
};
input g_inputs[MAX_INPUTS];
uint32_t g_input_n = 0;

uint32_t g_lost_in = 0;
uint32_t g_lost_out = 0;
uint32_t g_forwarded = 0;

// ---- エクスポートするメソッド --------------------------------------------------
uintptr_t method_add_input(uintptr_t argument, uintptr_t, uintptr_t, uintptr_t) {
  if (g_input_n >= MAX_INPUTS)
    return 0;
  const uintptr_t id = argument >> 16;
  const uint32_t priority = (uint32_t)(argument & 0xFFFFu);
  // ★0 は正当なストリーム番号なので弾かない (tx_frame.hpp の NO_STREAM 参照)。
  if (id == xno::NO_STREAM)
    return 0;
  g_inputs[g_input_n].stream_id = id;
  g_inputs[g_input_n].priority = priority;
  ++g_input_n;
  return 1;
}

uintptr_t method_get_stream(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  return g_out_id;
}

// ---- 合流ループ ---------------------------------------------------------------
constexpr uint32_t PERIOD_US = 2000; // 500Hz

uintptr_t poll_loop(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  api(shizuku::object_api::STREAM_BIND, g_out_id,
      (uintptr_t)shizuku::stream::role::PRODUCER);

  for (uint32_t i = 0; i < g_input_n; ++i) {
    const auto opened =
        api(shizuku::object_api::STREAM_OPEN, g_inputs[i].stream_id);
    if (opened.error != 0 || opened.value == 0) {
      BOARD::diag_printf("[LOG] input stream %lu could not be opened (%lu)\n",
                         (unsigned long)g_inputs[i].stream_id,
                         (unsigned long)opened.error);
      continue;
    }
    g_inputs[i].handle = shizuku::stream::handle<frame_t>(
        (shizuku::stream::descriptor *)opened.value);
    api(shizuku::object_api::STREAM_BIND, g_inputs[i].stream_id,
        (uintptr_t)shizuku::stream::role::CONSUMER);
    BOARD::diag_printf("[LOG] input stream %lu (prio %lu) bound\n",
                       (unsigned long)g_inputs[i].stream_id,
                       (unsigned long)g_inputs[i].priority);
  }

  BOARD::diag_printf("[LOG] merging %lu inputs -> stream %lu\n",
                     (unsigned long)g_input_n, (unsigned long)g_out_id);

  uint64_t next = BOARD::time_us() + PERIOD_US;
  while (true) {
    const int64_t remaining = (int64_t)(next - BOARD::time_us());
    if (remaining > 0)
      api(shizuku::object_api::SLEEP_US, (uintptr_t)remaining);
    next += PERIOD_US;

    // ★優先度の低い数から順に汲む = 小さいほど先に出る。要素数は 4 なので
    //   毎回線形に選ぶ (並べ替えは持たない — 表が小さいうちは走査の方が安い)。
    uint32_t done = 0;
    while (done < g_input_n) {
      uint32_t best = MAX_INPUTS;
      uint32_t best_prio = 0xFFFFFFFFu;
      for (uint32_t i = 0; i < g_input_n; ++i) {
        if (!g_inputs[i].handle.valid())
          continue;
        if (g_inputs[i].handle.available() == 0)
          continue;
        if (g_inputs[i].priority < best_prio) {
          best_prio = g_inputs[i].priority;
          best = i;
        }
      }
      if (best == MAX_INPUTS)
        break; // どこにも残っていない
      frame_t f{};
      uint32_t lost = 0;
      if (!g_inputs[best].handle.pop(&f, &lost)) {
        g_lost_in += lost;
        ++done; // 取れなかった相手は次の周回へ回す
        continue;
      }
      g_lost_in += lost;
      if (g_out.hdl().push(f))
        ++g_forwarded;
      else
        ++g_lost_out;
    }
  }
  return 0;
}

uintptr_t logger_main(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  uintptr_t failures =
      api(shizuku::object_api::DECLARE_NAME, (uintptr_t) "logger").error;
  failures += export_method(method::ADD_INPUT, (uintptr_t)&method_add_input);
  failures += export_method(method::GET_STREAM, (uintptr_t)&method_get_stream);
  failures += export_method(method::POLL, (uintptr_t)&poll_loop);

  g_out.init();
  const auto created =
      api(shizuku::object_api::STREAM_CREATE, (uintptr_t)&g_out.desc);
  failures += created.error;
  g_out_id = created.value;
  return failures;
}

} // namespace

uint32_t register_logger() {
  const auto created = api(shizuku::object_api::CREATE_OBJECT, OBJECT,
                           (uintptr_t)&logger_main, shizuku::OBJECT_PRIVILEGED);
  const auto started = api(shizuku::object_api::CALL_METHOD, OBJECT, 0, 0);
  if (created.error != 0 || started.error != 0 || started.value != 0) {
    BOARD::diag_printf("[LOG] FAILED: create=%lu call=%lu exports_failed=%lu\n",
                       (unsigned long)created.error,
                       (unsigned long)started.error,
                       (unsigned long)started.value);
    return 1;
  }
  BOARD::diag_printf("[LOG] registered (object %lu, stream %lu)\n",
                     (unsigned long)OBJECT, (unsigned long)g_out_id);
  return 0;
}

uint32_t start_logger() {
  const auto spawned =
      api(shizuku::object_api::SPAWN, OBJECT, (uintptr_t)method::POLL, 0);
  if (spawned.error != 0) {
    BOARD::diag_printf("[LOG] could not spawn (%lu)\n",
                       (unsigned long)spawned.error);
    return 1;
  }
  BOARD::diag_printf("[LOG] thread %lu started\n",
                     (unsigned long)spawned.value);
  return 0;
}

} // namespace logger
