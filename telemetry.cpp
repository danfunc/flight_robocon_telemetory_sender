// ===========================================================================
//  telemetry — サンプル → テレメトリ行
// ===========================================================================
//  設計の理由は telemetry.hpp 冒頭。
#include "telemetry.hpp"
#include "flight_controller.hpp"
#include "fw_version.hpp"
#include "shizuku/objects/ota.hpp"
#include "shizuku/kernel.hpp"
#include "shizuku/object_api.hpp"
#include "shizuku/stream.hpp"
#include <cmath> // powf (気圧高度)
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace telemetry {
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

// ---- 出口: 行 1 本 -----------------------------------------------------------
shizuku::stream::storage<frame_t, 16> g_out;
uintptr_t g_out_id = 0;

// ---- 入口: センサごとに 1 本 --------------------------------------------------
uintptr_t g_input_ids[MAX_INPUTS] = {};
shizuku::stream::handle<sample_t> g_inputs[MAX_INPUTS];
uint32_t g_input_n = 0;

// ---- 最新値 (符号化に必要な状態) ----------------------------------------------
int32_t g_eul[3] = {}; // BNO055 raw (1/16 deg)
int32_t g_lia[3] = {}; // BNO055 raw (1/100 m/s^2)
int32_t g_grv[3] = {}; // BNO055 raw (1/100 m/s^2)
int32_t g_press_pa = 0;
int32_t g_temp_cc = 0;
bool g_have_imu = false;
bool g_have_baro = false;

uint32_t g_seq = 0;
uint32_t g_lost_in = 0;
uint32_t g_lost_out = 0;
uint32_t g_period_us = 100000; // 既定 10Hz

// 気圧高度 [mm]。国際標準大気の素直な式、海面基準 101325 Pa 固定。
// ★QNH 補正は無い ので絶対高度としては信用しないこと (離陸地点との差分を見る用途)。
int32_t altitude_mm(int32_t press_pa) {
  if (press_pa <= 0)
    return 0;
  const float ratio = (float)press_pa / 101325.0f;
  const float meters = 44330.0f * (1.0f - powf(ratio, 1.0f / 5.255f));
  return (int32_t)(meters * 1000.0f);
}

// BNO055 の生値 → クライアントの倍率へ。
//   euler : 1 LSB = 1/16 deg      → cdeg (x100)   = raw * 100 / 16
//   accel : 1 LSB = 1/100 m/s^2   → mm/s^2 (x1000) = raw * 10
inline int32_t eul_to_cdeg(int32_t raw) { return raw * 100 / 16; }
inline int32_t acc_to_mm_s2(int32_t raw) { return raw * 10; }

void push_line(const char *text, uint32_t len) {
  // ★★焼いている最中は BLE へ何も積まない。消去・書き込みは IRQ を止めて
  //   走るので、その間の BLE トラフィックは CYW43 の SPI/PIO を壊す。
  //   テレメトリは 20ms 周期で最も量が多い口なので、ここを塞ぐのが一番効く。
  //   ★捨てた行は g_lost_out に数えず黙って落とす —— 数えると「取りこぼし」に
  //     見えるが、これは仕様として止めているので別物。
  if (shizuku::objects::ota::flash_busy())
    return;
  frame_t f{};
  if (len > sizeof(f.data))
    len = sizeof(f.data);
  f.len = (uint16_t)len;
  memcpy(f.data, text, len);
  if (!g_out.hdl().push(f))
    ++g_lost_out;
}

// ---- 1 行組み立て (tools/shizuku_link.py の TELEMETRY_FIELDS 順) --------------
// 走っている像の素性を 1 行流す。
//  ★"PICO," 行に列を足さないこと。ホスト側 (tools/shizuku_link.py) は
//    列数が合わない PICO 行を捨てるので、足した瞬間にテレメトリが全部消える。
//    別種の行にすれば、知らないホストは黙って無視できる。
//  ★1 回だけでなく時々流す。ホストが後から繋いだときに見逃さないため
//    (値は控えてあるので、流すこと自体は安い)。
// 何行に 1 回、素性を挟むか。★毎行は無駄、1 回きりだと後から繋いだホストが
//   見逃す。テレメトリが 20ms 周期なら 200 行 = およそ 4 秒に 1 回。
constexpr uint32_t VERSION_EVERY = 200;
uint32_t g_since_version = 0;

void emit_version() {
  const auto &id = xno::firmware_id();
  char line[64];
  const int written = snprintf(line, sizeof(line), "PICOVER,%lu,%08lx\n",
                               (unsigned long)id.bytes, (unsigned long)id.crc32);
  if (written > 0)
    push_line(line, (uint32_t)written);
}

void emit_line() {
  // まだ何も測れていないなら送らない (0 だけの行を流さない)。
  if (!g_have_imu && !g_have_baro)
    return;
  const int32_t alt_mm = altitude_mm(g_press_pa);

  // flight_controller から最新の制御状態を取ってくる。
  //  ★★グローバルを直に読まないこと。以前は flight_controller が公開した
  //    ミラー変数を参照していたが、それは (1) オブジェクト境界を越えた素の
  //    メモリ参照で、FLIGHT_CONTROLLER を非特権 + per-object arena へ移した
  //    瞬間に fault し、(2) 本体の値とミラーの二重管理でいつでもズレうる。
  //    メソッドで訊けば、答える側が唯一の持ち主のままでいられる。
  ::flight_controller::control_state cs{};
  api(shizuku::object_api::CALL_METHOD, ::flight_controller::OBJECT,
      (uintptr_t)::flight_controller::method::GET_CONTROL_STATE, (uintptr_t)&cs);

  char line[256];
  const int written = snprintf(
      line, sizeof(line),
      "PICO,%lu,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,"
      "%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\n",
      (unsigned long)++g_seq,                      // seq
      (unsigned long)(BOARD::time_us() / 1000ull), // up_ms
      (long)g_temp_cc,                             // temp   (x100 degC)
      (long)g_press_pa,                            // press  (Pa; /100 = hPa)
      (long)alt_mm,                                // alt_baro (mm)
      (long)(cs.h_est * 1000.0f),                  // alt_fused (mm)
      (long)(cs.v_est * 1000.0f),                  // vel (mm/s)
      (long)0,                                     // speed
      (long)acc_to_mm_s2(g_lia[2]),                // az ≒ LIA z
      (long)acc_to_mm_s2(g_lia[0]),                // lax
      (long)acc_to_mm_s2(g_lia[1]),                // lay
      (long)acc_to_mm_s2(g_lia[2]),                // laz
      (long)acc_to_mm_s2(g_grv[0]),                // gx
      (long)acc_to_mm_s2(g_grv[1]),                // gy
      (long)acc_to_mm_s2(g_grv[2]),                // gz
      (long)eul_to_cdeg(g_eul[0]),                 // heading
      (long)eul_to_cdeg(g_eul[2]),                 // roll  (BNO055: eul[2])
      (long)eul_to_cdeg(g_eul[1]),                 // pitch (BNO055: eul[1])
      (long)0,                                     // calib
      (long)cs.vstate,                             // vstate
      (long)(cs.elevator * 100.0f),                // elev (x100 deg)
      (long)0,                                     // servo
      (long)(cs.rudder * 100.0f),                  // rudder (x100 deg)
      (long)(cs.throttle * 1000.0f));              // throttle (x1000: 0..1000)
  if (written > 0)
    push_line(line, (uint32_t)written);
}

void absorb(const sample_t &s) {
  switch (s.kind) {
  case xno::sample_kind::IMU:
    for (uint32_t i = 0; i < 3; ++i) {
      g_eul[i] = s.value[i];
      g_lia[i] = s.value[3 + i];
      g_grv[i] = s.value[6 + i];
    }
    g_have_imu = true;
    break;
  case xno::sample_kind::BARO:
    g_press_pa = s.value[0];
    g_temp_cc = s.value[1];
    g_have_baro = true;
    break;
  default:
    break;
  }
}

// ---- エクスポートするメソッド --------------------------------------------------
uintptr_t method_add_input(uintptr_t argument, uintptr_t, uintptr_t, uintptr_t) {
  // ★0 は正当なストリーム番号なので弾かない (tx_frame.hpp の NO_STREAM 参照)。
  if (g_input_n >= MAX_INPUTS || argument == xno::NO_STREAM)
    return 0;
  g_input_ids[g_input_n++] = argument;
  return 1;
}

uintptr_t method_get_stream(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  return g_out_id;
}

uintptr_t method_set_rate(uintptr_t argument, uintptr_t, uintptr_t, uintptr_t) {
  if (argument == 0)
    return 0;
  g_period_us = (uint32_t)argument * 1000u;
  return 1;
}

// ---- 符号化ループ -------------------------------------------------------------
// ★入力を汲む周期は送出周期より細かくする。サンプルはリングに溜まるので
//   「送出のたびに全部汲む」でも動くが、細かく汲む方が取りこぼしに強い。
constexpr uint32_t DRAIN_PERIOD_US = 5000; // 200Hz

uintptr_t poll_loop(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  api(shizuku::object_api::STREAM_BIND, g_out_id,
      (uintptr_t)shizuku::stream::role::PRODUCER);

  for (uint32_t i = 0; i < g_input_n; ++i) {
    const auto opened = api(shizuku::object_api::STREAM_OPEN, g_input_ids[i]);
    if (opened.error != 0 || opened.value == 0) {
      BOARD::diag_printf("[TELEM] input stream %lu could not be opened (%lu)\n",
                         (unsigned long)g_input_ids[i],
                         (unsigned long)opened.error);
      continue;
    }
    g_inputs[i] = shizuku::stream::handle<sample_t>(
        (shizuku::stream::descriptor *)opened.value);
    api(shizuku::object_api::STREAM_BIND, g_input_ids[i],
        (uintptr_t)shizuku::stream::role::CONSUMER);
  }

  BOARD::diag_printf("[TELEM] encoding %lu inputs -> stream %lu (%luHz)\n",
                     (unsigned long)g_input_n, (unsigned long)g_out_id,
                     (unsigned long)(1000000u / g_period_us));

  uint64_t next_drain = BOARD::time_us() + DRAIN_PERIOD_US;
  uint64_t next_emit = BOARD::time_us() + g_period_us;
  while (true) {
    const int64_t remaining = (int64_t)(next_drain - BOARD::time_us());
    if (remaining > 0)
      api(shizuku::object_api::SLEEP_US, (uintptr_t)remaining);
    next_drain += DRAIN_PERIOD_US;

    for (uint32_t i = 0; i < g_input_n; ++i) {
      if (!g_inputs[i].valid())
        continue;
      sample_t s{};
      uint32_t lost = 0;
      while (g_inputs[i].pop(&s, &lost)) {
        g_lost_in += lost;
        absorb(s);
      }
      g_lost_in += lost;
    }

    if ((int64_t)(BOARD::time_us() - next_emit) >= 0) {
      next_emit += g_period_us;
      // ★素性は最初の 1 回と、以後たまに。★★最初の 1 回はここで払う —
      //   CRC は像を丸ごとなめるので数ミリ秒かかり、割り当てを 1 回はみ出す。
      //   起動経路 (BLE を上げる前) でこれをやると、その分だけ BLE の
      //   立ち上がりが遅れるので、周期スレッドが回り始めてからにしている。
      if (g_since_version == 0)
        emit_version();
      if (++g_since_version >= VERSION_EVERY)
        g_since_version = 0;
      emit_line();
    }
  }
  return 0;
}

uintptr_t telemetry_main(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  uintptr_t failures =
      api(shizuku::object_api::DECLARE_NAME, (uintptr_t) "telemetry").error;
  failures += export_method(method::ADD_INPUT, (uintptr_t)&method_add_input);
  failures += export_method(method::GET_STREAM, (uintptr_t)&method_get_stream);
  failures += export_method(method::SET_RATE, (uintptr_t)&method_set_rate);
  failures += export_method(method::POLL, (uintptr_t)&poll_loop);

  g_out.init();
  const auto created =
      api(shizuku::object_api::STREAM_CREATE, (uintptr_t)&g_out.desc);
  failures += created.error;
  g_out_id = created.value;
  return failures;
}

} // namespace

uint32_t register_telemetry() {
  // ★affinity は指定しない。cyw43/btstack には一切触れないので、どのコアでもよい。
  const auto created = api(shizuku::object_api::CREATE_OBJECT, OBJECT,
                           (uintptr_t)&telemetry_main, shizuku::OBJECT_PRIVILEGED);
  const auto started = api(shizuku::object_api::CALL_METHOD, OBJECT, 0, 0);
  if (created.error != 0 || started.error != 0 || started.value != 0) {
    BOARD::diag_printf("[TELEM] FAILED: create=%lu call=%lu exports_failed=%lu\n",
                       (unsigned long)created.error,
                       (unsigned long)started.error,
                       (unsigned long)started.value);
    return 1;
  }
  BOARD::diag_printf("[TELEM] registered (object %lu, stream %lu)\n",
                     (unsigned long)OBJECT, (unsigned long)g_out_id);
  return 0;
}

uint32_t start_telemetry() {
  const auto spawned =
      api(shizuku::object_api::SPAWN, OBJECT, (uintptr_t)method::POLL, 0);
  if (spawned.error != 0) {
    BOARD::diag_printf("[TELEM] could not spawn (%lu)\n",
                       (unsigned long)spawned.error);
    return 1;
  }
  BOARD::diag_printf("[TELEM] thread %lu started\n",
                     (unsigned long)spawned.value);
  return 0;
}

} // namespace telemetry
