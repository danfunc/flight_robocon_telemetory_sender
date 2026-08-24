// ===========================================================================
//  flight_controller — 上りコマンドの解釈と機体の判断
// ===========================================================================
//  設計の理由は flight_controller.hpp 冒頭。
#include "flight_controller.hpp"
#include "shizuku/kernel.hpp"
#include "shizuku/object_api.hpp"
#include "shizuku/stream.hpp"
#include "telemetry.hpp" // SET_RATE (R<ms> コマンドの転送先)
#include <cstdint>
#include <cstdio>
#include <cstdlib> // atoi
#include <cstring>

namespace flight_controller {
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

// ---- 出口: 応答行 1 本 -------------------------------------------------------
shizuku::stream::storage<frame_t, 16> g_out;
uintptr_t g_out_id = 0;

// ---- 入口: ble_uart の RX ----------------------------------------------------
uintptr_t g_rx_id = xno::NO_STREAM;
shizuku::stream::handle<frame_t> g_rx;

uint32_t g_commands = 0;
uint32_t g_lost_in = 0;
uint32_t g_lost_out = 0;

void reply(const char *text) {
  frame_t f{};
  uint32_t n = (uint32_t)strlen(text);
  if (n > sizeof(f.data))
    n = sizeof(f.data);
  f.len = (uint16_t)n;
  memcpy(f.data, text, n);
  if (!g_out.hdl().push(f))
    ++g_lost_out;
}

// ---- コマンド解釈 -------------------------------------------------------------
// 1 行 = 1 コマンド。書式は tools/shizuku_telemetry_client.py の上りコマンドに
// 合わせてある (同ファイル冒頭の docstring が正)。
void handle_command(const frame_t &f) {
  char buf[96];
  uint32_t n = f.len < (sizeof(buf) - 1) ? f.len : (uint32_t)(sizeof(buf) - 1);
  memcpy(buf, f.data, n);
  buf[n] = '\0';
  while (n > 0 && (buf[n - 1] == '\n' || buf[n - 1] == '\r'))
    buf[--n] = '\0';
  if (n == 0)
    return;
  ++g_commands;
  BOARD::diag_printf("[FC] command: \"%s\"\n", buf);

  // P<token> : ping。**同じ文字列を返す**のが約束 (クライアントは `P\d+` の
  //            完全一致で RTT を確定する)。
  if (buf[0] == 'P' && n >= 2) {
    char line[104];
    if (snprintf(line, sizeof(line), "%s\n", buf) > 0)
      reply(line);
    return;
  }
  // R<ms> : テレメトリ送出周期。★周期を持っているのは telemetry なので、
  //         ここは**決めて頼むだけ** (状態を二重に持たない)。
  if (buf[0] == 'R' && n >= 2) {
    const int ms = atoi(buf + 1);
    if (ms <= 0) {
      reply("STATS rate rejected (must be > 0)\n");
      return;
    }
    const auto set = api(shizuku::object_api::CALL_METHOD, telemetry::OBJECT,
                         (uintptr_t)telemetry::method::SET_RATE, (uintptr_t)ms);
    char line[80];
    if (snprintf(line, sizeof(line), "STATS rate=%dms ok=%lu\n", ms,
                 (unsigned long)(set.error == 0 ? set.value : 0)) > 0)
      reply(line);
    return;
  }
  // F0 = CSV / F1 = バイナリ。★バイナリは**まだ実装が無い**。F1 に黙って CSV を
  //      返すとクライアントはバイナリを待ち続けて「無反応」に見えるので明示する。
  if (strcmp(buf, "F0") == 0) {
    reply("STATS format=csv\n");
    return;
  }
  if (strcmp(buf, "F1") == 0) {
    reply("STATS format=csv (binary not implemented)\n");
    return;
  }
  // S : 統計。クライアントは "STATS" 始まりの行をログへ出す。
  if (strcmp(buf, "S") == 0) {
    char line[128];
    if (snprintf(line, sizeof(line),
                 "STATS cmds=%lu lost_in=%lu lost_out=%lu\n",
                 (unsigned long)g_commands, (unsigned long)g_lost_in,
                 (unsigned long)g_lost_out) > 0)
      reply(line);
    return;
  }
  // 手で叩くとき用の別名 (クライアントは使わない)。
  if (strcmp(buf, "ping") == 0) {
    reply("pong\n");
    return;
  }

  char line[128];
  if (snprintf(line, sizeof(line), "unknown command: %s\n", buf) > 0)
    reply(line);
}

// ---- エクスポートするメソッド --------------------------------------------------
uintptr_t method_set_rx_stream(uintptr_t argument, uintptr_t, uintptr_t,
                               uintptr_t) {
  g_rx_id = argument;
  return 1;
}

uintptr_t method_get_stream(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  return g_out_id;
}

// ---- 主ループ -----------------------------------------------------------------
constexpr uint32_t PERIOD_US = 2000; // 500Hz (対話の待ち時間を短く保つ)

uintptr_t poll_loop(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  api(shizuku::object_api::STREAM_BIND, g_out_id,
      (uintptr_t)shizuku::stream::role::PRODUCER);

  if (g_rx_id != xno::NO_STREAM) {
    const auto opened = api(shizuku::object_api::STREAM_OPEN, g_rx_id);
    if (opened.error == 0 && opened.value != 0) {
      g_rx = shizuku::stream::handle<frame_t>(
          (shizuku::stream::descriptor *)opened.value);
      api(shizuku::object_api::STREAM_BIND, g_rx_id,
          (uintptr_t)shizuku::stream::role::CONSUMER);
    } else {
      BOARD::diag_printf("[FC] rx stream %lu could not be opened (%lu)\n",
                         (unsigned long)g_rx_id, (unsigned long)opened.error);
    }
  } else {
    BOARD::diag_printf("[FC] no rx stream wired — コマンドは受けられない\n");
  }

  BOARD::diag_printf("[FC] running (rx %lu -> reply stream %lu)\n",
                     (unsigned long)g_rx_id, (unsigned long)g_out_id);

  uint64_t next = BOARD::time_us() + PERIOD_US;
  while (true) {
    const int64_t remaining = (int64_t)(next - BOARD::time_us());
    if (remaining > 0)
      api(shizuku::object_api::SLEEP_US, (uintptr_t)remaining);
    next += PERIOD_US;

    if (!g_rx.valid())
      continue;
    frame_t f{};
    uint32_t lost = 0;
    while (g_rx.pop(&f, &lost)) {
      g_lost_in += lost;
      handle_command(f);
    }
    g_lost_in += lost;
  }
  return 0;
}

uintptr_t flight_controller_main(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  uintptr_t failures =
      api(shizuku::object_api::DECLARE_NAME, (uintptr_t) "flight_controller")
          .error;
  failures +=
      export_method(method::SET_RX_STREAM, (uintptr_t)&method_set_rx_stream);
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

uint32_t register_flight_controller() {
  // ★affinity は指定しない。cyw43/btstack には一切触れないので、どのコアでもよい。
  const auto created =
      api(shizuku::object_api::CREATE_OBJECT, OBJECT,
          (uintptr_t)&flight_controller_main, shizuku::OBJECT_PRIVILEGED);
  const auto started = api(shizuku::object_api::CALL_METHOD, OBJECT, 0, 0);
  if (created.error != 0 || started.error != 0 || started.value != 0) {
    BOARD::diag_printf("[FC] FAILED: create=%lu call=%lu exports_failed=%lu\n",
                       (unsigned long)created.error,
                       (unsigned long)started.error,
                       (unsigned long)started.value);
    return 1;
  }
  BOARD::diag_printf("[FC] registered (object %lu, reply stream %lu)\n",
                     (unsigned long)OBJECT, (unsigned long)g_out_id);
  return 0;
}

uint32_t start_flight_controller() {
  const auto spawned =
      api(shizuku::object_api::SPAWN, OBJECT, (uintptr_t)method::POLL, 0);
  if (spawned.error != 0) {
    BOARD::diag_printf("[FC] could not spawn (%lu)\n",
                       (unsigned long)spawned.error);
    return 1;
  }
  BOARD::diag_printf("[FC] thread %lu started\n",
                     (unsigned long)spawned.value);
  return 0;
}

} // namespace flight_controller
