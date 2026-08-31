#include "blink.hpp"
#include "object_ids.hpp"
#include "props.hpp"
#include "shizuku/object_api.hpp"
#include "shizuku/objects/peripherals.hpp"
#include "shizuku/objects/usb_cdc.hpp"
#include <cstdint>
#include <cstdio>

namespace blink {
namespace {

// ---- 純粋なユーザ空間 syscall (svc 0) ----
struct api_result {
  uintptr_t error;
  uintptr_t value;
};

static inline api_result api(shizuku::object_api number, uintptr_t a1 = 0,
                             uintptr_t a2 = 0, uintptr_t a3 = 0,
                             uintptr_t a4 = 0) {
  register uintptr_t r0 asm("r0") = (uintptr_t)number;
  register uintptr_t r1 asm("r1") = a1;
  register uintptr_t r2 asm("r2") = a2;
  register uintptr_t r3 asm("r3") = a3;
  register uintptr_t r12 asm("r12") = a4;
  asm volatile("svc 0"
               : "+r"(r0), "+r"(r1)
               : "r"(r2), "r"(r3), "r"(r12)
               : "memory");
  return {r0, r1};
}

static inline void set_led_hw(bool on) {
  shizuku::objects::led_request req{on ? 1u : 0u};
  api(shizuku::object_api::CALL_METHOD, shizuku::objects::LED_OBJECT,
      (uintptr_t)shizuku::objects::led_method::WRITE, (uintptr_t)&req);
}

// パターン定義テーブル
struct pattern_step {
  uint16_t duration_ms;
  uint8_t level;
};

// Pattern 0: プレースホルダ (DYNAMIC_SWEEP は専用掃引アルゴリズムで実行)
constexpr pattern_step P0[] = {{200, 1}, {200, 0}};
// Pattern 1: 航空機アビオニクス・ダブルハートビート (60ms ON, 120ms OFF, 60ms
// ON, 760ms OFF)
constexpr pattern_step P1[] = {{60, 1}, {120, 0}, {60, 1}, {760, 0}};
// Pattern 2: トリプルフラッシュ・ビーコン (40ms ON, 60ms OFF, 40ms ON, 60ms
// OFF, 40ms ON, 760ms OFF)
constexpr pattern_step P2[] = {{40, 1}, {60, 0}, {40, 1},
                               {60, 0}, {40, 1}, {760, 0}};
// Pattern 3: モールス S 信号 (100ms ON, 100ms OFF, 100ms ON, 100ms OFF, 100ms
// ON, 500ms OFF)
constexpr pattern_step P3[] = {{100, 1}, {100, 0}, {100, 1},
                               {100, 0}, {100, 1}, {500, 0}};
// Pattern 4: 高速アクティビティ・ストロボ (30ms ON, 30ms OFF: OTA受信中など)
constexpr pattern_step P4[] = {{30, 1}, {30, 0}};
// Pattern 5: プログレッシブ・バースト (1発, 2発, 3発, 4発の加速パルス)
constexpr pattern_step P5[] = {{50, 1},  {150, 0}, {50, 1}, {50, 0}, {50, 1},
                               {150, 0}, {50, 1},  {50, 0}, {50, 1}, {50, 0},
                               {50, 1},  {150, 0}, {50, 1}, {50, 0}, {50, 1},
                               {50, 0},  {50, 1},  {50, 0}, {50, 1}, {600, 0}};

struct pattern_info {
  const pattern_step *steps;
  uint8_t count;
};

constexpr pattern_info PATTERNS[] = {
    {P0, sizeof(P0) / sizeof(P0[0])}, {P1, sizeof(P1) / sizeof(P1[0])},
    {P2, sizeof(P2) / sizeof(P2[0])}, {P3, sizeof(P3) / sizeof(P3[0])},
    {P4, sizeof(P4) / sizeof(P4[0])}, {P5, sizeof(P5) / sizeof(P5[0])},
};
constexpr uint32_t NUM_PATTERNS = sizeof(PATTERNS) / sizeof(PATTERNS[0]);

namespace {
volatile uint8_t g_pattern_id = (uint8_t)
    pattern_id::DYNAMIC_SWEEP; // 既定: 0 (OTA検証用ダイナミックスイープ)
volatile uint8_t g_speed_pct = 100;
volatile int32_t g_interval_ms = 1000;
volatile uint32_t g_toggle_count = 0;
volatile bool g_enabled = true;
volatile bool g_running = false;
volatile uint8_t g_led_value = 0;
volatile uint32_t g_thread_id = 0;
} // namespace

// ---- 保存する設定 ----------------------------------------------------------
//  ★flash FS の上に置く (実体は Shizuku の flashfs オブジェクト)。
//    名前は媒体側で 24 バイト固定なので、パス風の長い名前は付けられない。
constexpr char CONFIG_NAME[] = "blink.cfg";
constexpr uint32_t CONFIG_MAGIC = 0x4B4E4C42; // 'BLNK'
constexpr uint16_t CONFIG_VERSION = 1;

// ★flash に焼く形。頭を付けるのは、ファーム更新で形が変わったときに
//   「古い版を新しい構造体として読む」のを止めるため (props.hpp 参照)。
struct persisted {
  xno::props::header head;
  uint8_t pattern_id;
  uint8_t speed_pct;
  uint16_t reserved;
};

void load_persisted_config() {
  persisted cfg{};
  if (!xno::props::load(CONFIG_NAME, CONFIG_MAGIC, CONFIG_VERSION, &cfg,
                        sizeof(cfg)))
    return; // 無い / 古い / 形が違う — どれも「既定値で始める」で同じ
  // ★焼いてある値でも範囲は見る。媒体の中身は「前のファームが書いたもの」で、
  //   そのファームの範囲が今と同じとは限らない。
  if (cfg.pattern_id < NUM_PATTERNS)
    g_pattern_id = cfg.pattern_id;
  if (cfg.speed_pct >= 10 && cfg.speed_pct <= 300)
    g_speed_pct = cfg.speed_pct;
}

// ★★点滅ループから呼ばないこと。1 セクタの消去に約 33ms かかり、その間
//   XIP が止まる = 系全体が止まる。呼んでよいのはシェル経由 (SAVE_CONFIG)
//   だけ。
bool save_persisted_config() {
  persisted cfg{};
  cfg.head.magic = CONFIG_MAGIC;
  cfg.head.version = CONFIG_VERSION;
  cfg.head.bytes = (uint16_t)sizeof(cfg);
  cfg.pattern_id = g_pattern_id;
  cfg.speed_pct = g_speed_pct;
  return xno::props::store(CONFIG_NAME, &cfg, sizeof(cfg));
}

// ---- 点滅ループ (Core 0 固定) ----
//  ★Pattern 0: OTA (commit) の実証用に、200ms〜1400ms
//  を往復する掃引アルゴリズム。
//    速くなって遅くなってを繰り返すことで、一目で「新しい像が動いている」と分かる。
//    GDB で attach してレジスタ interval_ms
//    を読んでも値が動いていることが確認できる。
uintptr_t poll_loop(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  load_persisted_config();
  g_running = true;
  g_enabled = true;
  uint32_t step_index = 0;

  constexpr int32_t MIN_MS = 1000;
  constexpr int32_t MAX_MS = 2000;
  constexpr int32_t STEP_MS = 10;
  int32_t sweep_interval_ms = MIN_MS;
  int32_t sweep_step_ms = STEP_MS;

  while (g_running && g_enabled) {
    uint8_t pid = g_pattern_id;
    if (pid >= NUM_PATTERNS) {
      pid = 0;
    }

    if (pid == (uint8_t)pattern_id::DYNAMIC_SWEEP) {
      // ---- Pattern 0: OTA 検証用ダイナミックスイープ (200ms〜1400ms 往復掃引)
      // ----
      g_led_value ^= 1;
      set_led_hw(g_led_value != 0);
      g_toggle_count = g_toggle_count + 1;

      uint32_t dur =
          (g_speed_pct > 0)
              ? (((uint32_t)sweep_interval_ms * 100) / (uint32_t)g_speed_pct)
              : (uint32_t)sweep_interval_ms;
      if (dur < 5)
        dur = 5;
      g_interval_ms = (int32_t)dur;

      api(shizuku::object_api::SLEEP_US, (uintptr_t)(dur * 1000));

      sweep_interval_ms += sweep_step_ms;
      if (sweep_interval_ms >= MAX_MS) {
        sweep_interval_ms = MAX_MS;
        sweep_step_ms = -STEP_MS;
      } else if (sweep_interval_ms <= MIN_MS) {
        sweep_interval_ms = MIN_MS;
        sweep_step_ms = STEP_MS;
      }
      step_index = 0;
    } else {
      // ---- Pattern 1..5: テーブル定義パターン ----
      const auto &p = PATTERNS[pid];
      const auto &step = p.steps[step_index % p.count];
      step_index = (step_index + 1) % p.count;

      g_led_value = step.level;
      set_led_hw(g_led_value != 0);
      g_toggle_count = g_toggle_count + 1;

      // スピード補正 (speed_pct: 100 = 1.0x, 200 = 2.0x 高速, 50 = 0.5x 低速)
      uint32_t dur =
          (g_speed_pct > 0)
              ? (((uint32_t)step.duration_ms * 100) / (uint32_t)g_speed_pct)
              : (uint32_t)step.duration_ms;
      if (dur < 5)
        dur = 5;
      g_interval_ms = (int32_t)dur;

      api(shizuku::object_api::SLEEP_US, (uintptr_t)(dur * 1000));
    }

    if (!g_running || !g_enabled) {
      break;
    }
  }

  set_led_hw(false);
  api(shizuku::object_api::EXIT_THREAD);
  return 0;
}

uintptr_t export_method(method m, uintptr_t entry) {
  return api(shizuku::object_api::EXPORT_METHOD, (uintptr_t)m, entry).error;
}

// ---- エクスポートするメソッド群 ----
uintptr_t method_set_interval(uintptr_t ms, uintptr_t, uintptr_t, uintptr_t) {
  if (ms >= 10 && ms <= 10000) {
    g_interval_ms = (int32_t)ms;
    return 1;
  }
  return 0;
}

uintptr_t method_set_enabled(uintptr_t en, uintptr_t, uintptr_t, uintptr_t) {
  g_enabled = (en != 0);
  if (!g_enabled) {
    g_led_value = 0;
    set_led_hw(false);
  }
  return 1;
}

uintptr_t method_get_state(uintptr_t arg, uintptr_t, uintptr_t, uintptr_t) {
  if (arg == 0)
    return 0;
  auto *out = (state *)arg;
  out->interval_ms = (uint32_t)g_interval_ms;
  out->toggle_count = (uint32_t)g_toggle_count;
  out->enabled = g_enabled ? 1 : 0;
  out->led_value = g_led_value;
  out->pattern_id = g_pattern_id;
  out->speed_pct = g_speed_pct;
  return 1;
}

uintptr_t method_stop(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  g_running = false;
  g_enabled = false;
  set_led_hw(false);
  return 1;
}

uintptr_t method_set_pattern(uintptr_t pat, uintptr_t, uintptr_t, uintptr_t) {
  if (pat < NUM_PATTERNS) {
    g_pattern_id = (uint8_t)pat;
    return 1;
  }
  return 0;
}

uintptr_t method_set_speed(uintptr_t pct, uintptr_t, uintptr_t, uintptr_t) {
  if (pct >= 10 && pct <= 1000) {
    g_speed_pct = (uint8_t)pct;
    return 1;
  }
  return 0;
}

uintptr_t method_save_config(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  return save_persisted_config() ? 1 : 0;
}

uintptr_t method_load_config(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  load_persisted_config();
  return 1;
}

// メソッド 0: オブジェクト初期化 (メソッド群を export して即復帰)
uintptr_t blink_main(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  uintptr_t failures =
      api(shizuku::object_api::DECLARE_NAME, (uintptr_t)"blink").error;
  failures += export_method(method::POLL, (uintptr_t)&poll_loop);
  failures +=
      export_method(method::SET_INTERVAL, (uintptr_t)&method_set_interval);
  failures +=
      export_method(method::SET_ENABLED, (uintptr_t)&method_set_enabled);
  failures += export_method(method::GET_STATE, (uintptr_t)&method_get_state);
  failures += export_method(method::STOP, (uintptr_t)&method_stop);
  failures +=
      export_method(method::SET_PATTERN, (uintptr_t)&method_set_pattern);
  failures += export_method(method::SET_SPEED, (uintptr_t)&method_set_speed);
  failures +=
      export_method(method::SAVE_CONFIG, (uintptr_t)&method_save_config);
  failures +=
      export_method(method::LOAD_CONFIG, (uintptr_t)&method_load_config);
  return failures;
}

} // namespace

uint32_t register_blink(uintptr_t obj_id) {
  // LED (CYW43) は Core 0 で初期化されているため、必ず Core 0 にバインドする
  const auto created = api(shizuku::object_api::CREATE_OBJECT, obj_id,
                           (uintptr_t)&blink_main, shizuku::OBJECT_ON_CORE(0));
  const auto exported = api(shizuku::object_api::CALL_METHOD, obj_id, 0, 0);
  if (created.error != 0 || exported.error != 0 || exported.value != 0) {
    return 1;
  }
  return 0;
}

uint32_t start_blink(uintptr_t obj_id) {
  const auto spawned =
      api(shizuku::object_api::SPAWN, obj_id, (uintptr_t)method::POLL, 0);
  if (spawned.error != 0) {
    return 1;
  }
  g_thread_id = (uint32_t)spawned.value;
  return 0;
}

uint32_t stop_blink(uintptr_t obj_id) {
  api(shizuku::object_api::CALL_METHOD, obj_id, (uintptr_t)method::STOP, 0);
  return 0;
}

uint32_t get_thread_id() { return (uint32_t)g_thread_id; }

} // namespace blink
