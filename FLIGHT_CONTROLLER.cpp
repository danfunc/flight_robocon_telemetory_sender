// ===========================================================================
//  flight_controller — 自動操縦アルゴリズム (プロトタイプ) & コマンド解釈
// ===========================================================================
#include "flight_controller.hpp"
#include "blink.hpp"
#include "flash_fs.hpp"
#include "shizuku_loader.hpp"
#include "shizuku/kernel.hpp"
#include "shizuku/object_api.hpp"
#include "shizuku/stream.hpp"
#include "telemetry.hpp" // SET_RATE (R<ms> コマンドの転送先)
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
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

// ---- 入口: センササンプルストリーム (BNO055 / BME280) -------------------------
constexpr uint32_t MAX_SENSOR_INPUTS = 4;
uintptr_t g_sensor_ids[MAX_SENSOR_INPUTS] = {};
shizuku::stream::handle<sample_t> g_sensors[MAX_SENSOR_INPUTS];
uint32_t g_sensor_n = 0;

uint32_t g_commands = 0;
uint32_t g_lost_in = 0;
uint32_t g_lost_out = 0;

// Flash FS ワイヤレスアップロード用バッファ (最大 8KB)
static char g_upload_path[32] = "";
static size_t g_upload_expected = 0;
static size_t g_upload_received = 0;
static uint8_t g_upload_buf[8192];

// ===========================================================================
//  自動操縦アルゴリズム (パラメータ & ゲインスケジュール)
// ===========================================================================
// スロットル連動ゲインスケジュール [0.0, 0.5, 1.0]
static constexpr int KN = 3;
static constexpr float THR_BP[KN] = {0.0f, 0.5f, 1.0f};
static constexpr float K_PITCH[KN] = {0.6f, 1.0f, 1.4f};
static constexpr float K_YAW[KN] = {0.5f, 1.0f, 1.5f};

// ピッチ内側ループ (姿勢安定化)
static constexpr float KP_PITCH = 2.0f;       // [elev-deg / deg]
static constexpr float KD_PITCH = 0.4f;       // [elev-deg / (deg/s)]
static constexpr float KI_PITCH = 0.3f;       // [elev-deg / (deg*s)]
static constexpr float M_PROP = 0.0f;         // プロペラ後流補償
static constexpr float PITCH_I_LIMIT = 10.0f; // [deg]
static constexpr float ELEV_LIMIT_DEG = 30.0f;// [deg]

// ヨー方位保持ループ
static constexpr float KP_YAW = 1.2f;         // [rud-deg / deg]
static constexpr float KD_YAW = 0.25f;        // [rud-deg / (deg/s)]
static constexpr float RUD_LIMIT_DEG = 30.0f; // [deg]

// 高度外側ループ (スロットル制御)
static constexpr float KP_ALT = 0.15f;        // [thr / m]
static constexpr float KI_ALT = 0.03f;        // [thr / (m*s)]
static constexpr float KD_ALT = 0.10f;        // [thr / (m/s)]
static constexpr float ALT_I_LIMIT = 0.4f;

// 相補フィルタ時定数
static constexpr float ALPHA_H = 0.95f;
static constexpr float ALPHA_V = 0.90f;
static constexpr float A_Z_CLIP = 15.0f;

// 上昇/下降状態ヒステリシス
static constexpr float V_ENTER_ASC = +0.30f, V_LEAVE_ASC = +0.10f;
static constexpr float V_ENTER_DESC = -0.30f, V_LEAVE_DESC = -0.10f;
enum VState : uint8_t { ST_LEVEL = 0, ST_ASC = 1, ST_DESC = 2 };

// ---- 設定 / 目標 -------------------------------------------------------------
static bool g_armed = false;
static float g_pitch_ref = 0.0f;     // ピッチ目標 [deg]
static float g_heading_ref = 0.0f;   // 方位目標 [deg]
static bool g_heading_ref_set = false;
static float g_alt_ref = 1.5f;       // 高度目標 [m]
static float g_thr_trim = 0.4f;      // スロットルトリム [0..1]

// ---- 推定・内部状態 ----------------------------------------------------------
static float g_h_est = 0.0f;         // 推定高度 [m]
static float g_v_est = 0.0f;         // 推定鉛直速度 [m/s]
static float g_last_a_z = 0.0f;
static float g_h_baro_prev = 0.0f;
static uint64_t g_last_baro_us = 0;
static uint64_t g_last_imu_us = 0;

static float g_pitch_i = 0.0f, g_alt_i = 0.0f;
static float g_pitch_prev = 0.0f, g_heading_prev = 0.0f;
static bool g_primed = false;
static VState g_vstate = ST_LEVEL;
static control_state g_state = {};

// ---- 算術ユーティリティ -------------------------------------------------------
static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

static inline float clamp_sym(float x, float lim) {
  return clampf(x, -lim, lim);
}

static inline float wrap180(float d) {
  while (d > 180.0f)
    d -= 360.0f;
  while (d < -180.0f)
    d += 360.0f;
  return d;
}

static float k_lookup(const float *bp, const float *val, int n, float x) {
  if (x <= bp[0])
    return val[0];
  if (x >= bp[n - 1])
    return val[n - 1];
  for (int i = 1; i < n; ++i) {
    if (x <= bp[i]) {
      float t = (x - bp[i - 1]) / (bp[i] - bp[i - 1]);
      return val[i - 1] + t * (val[i] - val[i - 1]);
    }
  }
  return val[n - 1];
}

static VState next_state(VState cur, float v) {
  switch (cur) {
  case ST_LEVEL:
    if (v > V_ENTER_ASC)
      return ST_ASC;
    if (v < V_ENTER_DESC)
      return ST_DESC;
    return ST_LEVEL;
  case ST_ASC:
    return (v < V_LEAVE_ASC) ? ST_LEVEL : ST_ASC;
  case ST_DESC:
    return (v > V_LEAVE_DESC) ? ST_LEVEL : ST_DESC;
  }
  return ST_LEVEL;
}

// 応答送信
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

// ===========================================================================
//  センササンプル処理 (高度推定 & 制御則計算)
// ===========================================================================
void process_sample(const sample_t &s) {
  uint64_t now = BOARD::time_us();

  if (s.kind == xno::sample_kind::IMU) {
    // BNO055: value[0..2]=heading/roll/pitch (1/16 deg), value[3..5]=lax/lay/laz, value[6..8]=gx/gy/gz
    float heading = s.value[0] / 16.0f;
    float roll = s.value[1] / 16.0f;
    float pitch = s.value[2] / 16.0f;
    float lax = s.value[3] / 100.0f;
    float lay = s.value[4] / 100.0f;
    float laz = s.value[5] / 100.0f;
    float gx = s.value[6] / 100.0f;
    float gy = s.value[7] / 100.0f;
    float gz = s.value[8] / 100.0f;

    float dt = g_last_imu_us > 0 ? (float)(now - g_last_imu_us) / 1e6f : 0.02f;
    if (dt <= 0.0f || dt > 0.2f)
      dt = 0.02f;
    g_last_imu_us = now;

    // 鉛直加速度 (重力ベクトルへの線形加速度の射影)
    float gn = sqrtf(gx * gx + gy * gy + gz * gz);
    float a_z = 0.0f;
    if (gn > 1.0f) {
      a_z = (lax * gx + lay * gy + laz * gz) / gn;
      a_z = clamp_sym(a_z, A_Z_CLIP);
    }
    g_last_a_z = a_z;

    // 慣性航法 (高度・鉛直速度の事前予測)
    g_h_est += g_v_est * dt + 0.5f * a_z * dt * dt;
    g_v_est += a_z * dt;

    // 自動操縦 制御ループ計算
    if (!g_armed) {
      g_pitch_i = 0.0f;
      g_alt_i = 0.0f;
      g_pitch_prev = pitch;
      g_heading_prev = heading;
      g_primed = true;

      g_state.elevator = 0.0f;
      g_state.rudder = 0.0f;
      g_state.throttle = 0.0f;
      g_state.pitch_ref = g_pitch_ref;
      g_state.heading_ref = g_heading_ref;
      g_state.alt_ref = g_alt_ref;
      g_state.h_est = g_h_est;
      g_state.v_est = g_v_est;
      g_state.vstate = (uint8_t)g_vstate;
      g_state.armed = 0;
      return;
    }

    // 初回 arm 時に方位目標を捕捉
    if (!g_heading_ref_set) {
      g_heading_ref = heading;
      g_heading_ref_set = true;
    }

    // ① 高度外側ループ → スロットル
    float h_err = g_alt_ref - g_h_est;
    g_alt_i = clamp_sym(g_alt_i + h_err * dt, ALT_I_LIMIT);
    float u_thr = g_thr_trim + KP_ALT * h_err + KI_ALT * g_alt_i - KD_ALT * g_v_est;
    u_thr = clampf(u_thr, 0.0f, 1.0f);

    // スロットルに基づくゲインスケジュール補間
    float kp_eff = k_lookup(THR_BP, K_PITCH, KN, u_thr);
    float ky_eff = k_lookup(THR_BP, K_YAW, KN, u_thr);

    // ② ピッチ内側ループ (姿勢一定 & 安定化)
    float th_rate = g_primed ? (pitch - g_pitch_prev) / dt : 0.0f;
    g_pitch_prev = pitch;
    float e_p = g_pitch_ref - pitch;
    g_pitch_i = clamp_sym(g_pitch_i + e_p * dt, PITCH_I_LIMIT);
    float v_p = KP_PITCH * e_p - KD_PITCH * th_rate + KI_PITCH * g_pitch_i;
    v_p -= M_PROP * u_thr;
    float elev = clamp_sym(v_p / kp_eff, ELEV_LIMIT_DEG);

    // ③ ヨーループ (方位保持)
    float psi_rate = g_primed ? wrap180(heading - g_heading_prev) / dt : 0.0f;
    g_heading_prev = heading;
    float e_y = wrap180(g_heading_ref - heading);
    float v_y = KP_YAW * e_y - KD_YAW * psi_rate;
    float rud = clamp_sym(v_y / ky_eff, RUD_LIMIT_DEG);

    g_primed = true;
    g_vstate = next_state(g_vstate, g_v_est);

    g_state.elevator = elev;
    g_state.rudder = rud;
    g_state.throttle = u_thr;
    g_state.pitch_ref = g_pitch_ref;
    g_state.heading_ref = g_heading_ref;
    g_state.alt_ref = g_alt_ref;
    g_state.h_est = g_h_est;
    g_state.v_est = g_v_est;
    g_state.vstate = (uint8_t)g_vstate;
    g_state.armed = 1;
  } else if (s.kind == xno::sample_kind::BARO) {
    // BME280: value[0]=press [Pa], value[1]=temp [0.01 degC]
    int32_t press_pa = s.value[0];
    if (press_pa <= 0)
      return;
    float alt_baro = 44330.0f * (1.0f - powf((float)press_pa / 101325.0f, 0.190295f));

    float dt = g_last_baro_us > 0 ? (float)(now - g_last_baro_us) / 1e6f : 0.05f;
    if (dt <= 0.0f || dt > 0.5f)
      dt = 0.05f;
    g_last_baro_us = now;

    // 気圧高度・鉛直速度の相補フィルタ補正
    g_h_est = ALPHA_H * g_h_est + (1.0f - ALPHA_H) * alt_baro;
    if (g_h_baro_prev != 0.0f) {
      float v_baro = (alt_baro - g_h_baro_prev) / dt;
      g_v_est = ALPHA_V * g_v_est + (1.0f - ALPHA_V) * v_baro;
    }
    g_h_baro_prev = alt_baro;
  }
}

// ===========================================================================
//  コマンド解釈
// ===========================================================================
void handle_command(const frame_t &f) {
  char buf[200];
  uint32_t n = f.len < (sizeof(buf) - 1) ? f.len : (uint32_t)(sizeof(buf) - 1);
  memcpy(buf, f.data, n);
  buf[n] = '\0';
  while (n > 0 && (buf[n - 1] == '\n' || buf[n - 1] == '\r'))
    buf[--n] = '\0';
  if (n == 0)
    return;
  ++g_commands;
  BOARD::diag_printf("[FC] command: \"%s\"\n", buf);

  // P<token> : ping
  if (buf[0] == 'P' && n >= 2) {
    char line[104];
    if (snprintf(line, sizeof(line), "%s\n", buf) > 0)
      reply(line);
    return;
  }
  // R<ms> : テレメトリ送出周期
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
  // A1 / A0 : 自動操縦 Arm / Disarm
  if (buf[0] == 'A' && n >= 2) {
    if (buf[1] == '1' || strcmp(buf, "ARM") == 0) {
      g_armed = true;
      g_heading_ref_set = false;
      g_pitch_i = 0.0f;
      g_alt_i = 0.0f;
      reply("AUTOPILOT armed\n");
      return;
    } else if (buf[1] == '0' || strcmp(buf, "DISARM") == 0) {
      g_armed = false;
      reply("AUTOPILOT disarmed\n");
      return;
    } else if (buf[1] == 'L') { // AL<m> (目標高度)
      g_alt_ref = (float)atof(buf + 2);
      char line[64];
      snprintf(line, sizeof(line), "AUTOPILOT alt_ref=%.2fm\n", (double)g_alt_ref);
      reply(line);
      return;
    } else if (buf[1] == 'P') { // AP<deg> (目標ピッチ)
      g_pitch_ref = (float)atof(buf + 2);
      char line[64];
      snprintf(line, sizeof(line), "AUTOPILOT pitch_ref=%.1fdeg\n", (double)g_pitch_ref);
      reply(line);
      return;
    } else if (buf[1] == 'H') { // AH<deg> (目標方位)
      g_heading_ref = (float)atof(buf + 2);
      g_heading_ref_set = true;
      char line[64];
      snprintf(line, sizeof(line), "AUTOPILOT heading_ref=%.1fdeg\n", (double)g_heading_ref);
      reply(line);
      return;
    } else if (buf[1] == 'T') { // AT<thr> (スロットルトリム 0..1)
      g_thr_trim = clampf((float)atof(buf + 2), 0.0f, 1.0f);
      char line[64];
      snprintf(line, sizeof(line), "AUTOPILOT thr_trim=%.2f\n", (double)g_thr_trim);
      reply(line);
      return;
    }
  }
  // L : LED 点滅制御 (LP<pattern_id>, LS<speed_pct>, LE<1|0>)
  if (buf[0] == 'L' && n >= 2) {
    if (buf[1] == 'P') { // LP<pat>
      const int pat = atoi(buf + 2);
      const auto set = api(shizuku::object_api::CALL_METHOD, blink::OBJECT,
                           (uintptr_t)blink::method::SET_PATTERN, (uintptr_t)pat);
      char line[64];
      snprintf(line, sizeof(line), "LED pattern=%d ok=%lu\n", pat,
               (unsigned long)(set.error == 0 ? set.value : 0));
      reply(line);
      return;
    } else if (buf[1] == 'S') { // LS<pct>
      const int pct = atoi(buf + 2);
      const auto set = api(shizuku::object_api::CALL_METHOD, blink::OBJECT,
                           (uintptr_t)blink::method::SET_SPEED, (uintptr_t)pct);
      char line[64];
      snprintf(line, sizeof(line), "LED speed=%d%% ok=%lu\n", pct,
               (unsigned long)(set.error == 0 ? set.value : 0));
      reply(line);
      return;
    } else if (buf[1] == 'E') { // LE<1|0>
      const int en = atoi(buf + 2);
      const auto set = api(shizuku::object_api::CALL_METHOD, blink::OBJECT,
                           (uintptr_t)blink::method::SET_ENABLED, (uintptr_t)en);
      char line[64];
      snprintf(line, sizeof(line), "LED enabled=%d ok=%lu\n", en,
               (unsigned long)(set.error == 0 ? set.value : 0));
      reply(line);
      return;
    } else if (buf[1] == 'W') { // LW : 設定を Flash FS (/cfg/blink.conf) へ永続化
      const auto saved = api(shizuku::object_api::CALL_METHOD, blink::OBJECT,
                             (uintptr_t)blink::method::SAVE_CONFIG, 0);
      char line[64];
      snprintf(line, sizeof(line), "LED saved_to_flash ok=%lu\n",
               (unsigned long)(saved.error == 0 ? saved.value : 0));
      reply(line);
      return;
    } else if (buf[1] == 'R') { // LR : Flash FS (/cfg/blink.conf) から設定再読み込み
      const auto loaded = api(shizuku::object_api::CALL_METHOD, blink::OBJECT,
                              (uintptr_t)blink::method::LOAD_CONFIG, 0);
      char line[64];
      snprintf(line, sizeof(line), "LED loaded_from_flash ok=%lu\n",
               (unsigned long)(loaded.error == 0 ? loaded.value : 0));
      reply(line);
      return;
    }
  }

  // SW<path> : 動的モジュールのホットスワップ (BLE経由)
  if (buf[0] == 'S' && buf[1] == 'W' && n >= 3) {
    const char *path = buf + 2;
    xno::loader::loaded_info info{};
    bool ok = xno::loader::hot_swap("blink", path, 0, &info);
    char line[128];
    if (ok) {
      snprintf(line, sizeof(line), "SWAP ok obj=%lu tid=%lu (%s)\n",
               (unsigned long)info.object_id, (unsigned long)info.thread_id,
               info.is_xip ? "XIP" : "RAM");
    } else {
      snprintf(line, sizeof(line), "SWAP failed: %s\n", path);
    }
    reply(line);
    return;
  }

  // FB<path> <size> : Flash FS アップロード開始 (BLE経由)
  if (buf[0] == 'F' && buf[1] == 'B' && n >= 4) {
    char path[32];
    size_t size = 0;
    if (sscanf(buf + 2, "%31s %zu", path, &size) == 2 && size <= sizeof(g_upload_buf)) {
      strncpy(g_upload_path, path, sizeof(g_upload_path) - 1);
      g_upload_expected = size;
      g_upload_received = 0;
      char line[64];
      snprintf(line, sizeof(line), "FB ready %zu\n", size);
      reply(line);
    } else {
      reply("FB err\n");
    }
    return;
  }

  // FA<hex> : アップロードデータ追記
  if (buf[0] == 'F' && buf[1] == 'A' && n >= 3) {
    const char *hex = buf + 2;
    while (*hex == ' ') ++hex;
    size_t hex_len = strlen(hex);
    for (size_t i = 0; i + 1 < hex_len && g_upload_received < sizeof(g_upload_buf); i += 2) {
      auto hex_digit = [](char c) -> int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        return -1;
      };
      int h1 = hex_digit(hex[i]);
      int h2 = hex_digit(hex[i + 1]);
      if (h1 >= 0 && h2 >= 0) {
        g_upload_buf[g_upload_received++] = (uint8_t)((h1 << 4) | h2);
      }
    }
    char line[64];
    snprintf(line, sizeof(line), "FA rx=%zu/%zu\n", g_upload_received, g_upload_expected);
    reply(line);
    return;
  }

  // FC : アップロード完了 ＆ Flash FS へコミット
  if (buf[0] == 'F' && buf[1] == 'C') {
    if (g_upload_received == g_upload_expected && g_upload_path[0] != '\0') {
      bool ok = xno::fs::write_file(g_upload_path, g_upload_buf, g_upload_received, 3 /* VALID | EXEC */);
      char line[64];
      snprintf(line, sizeof(line), "FC %s written=%zu\n", ok ? "ok" : "err", g_upload_received);
      reply(line);
    } else {
      char line[64];
      snprintf(line, sizeof(line), "FC err rx=%zu exp=%zu\n", g_upload_received, g_upload_expected);
      reply(line);
    }
    return;
  }

  // FL : Flash FS ファイル一覧
  if (buf[0] == 'F' && buf[1] == 'L') {
    xno::fs::stat_t st_list[16];
    size_t count = xno::fs::list_files(st_list, 16);
    char line[128];
    snprintf(line, sizeof(line), "FL count=%zu\n", count);
    reply(line);
    for (size_t i = 0; i < count; ++i) {
      snprintf(line, sizeof(line), "  %s (%zu bytes)\n", st_list[i].name, st_list[i].size);
      reply(line);
    }
    return;
  }

  // FF : Flash FS フォーマット
  if (buf[0] == 'F' && buf[1] == 'F') {
    bool ok = xno::fs::format();
    char line[64];
    snprintf(line, sizeof(line), "FF %s\n", ok ? "ok" : "err");
    reply(line);
    return;
  }

  // FR<path> : ファイル削除
  if (buf[0] == 'F' && buf[1] == 'R' && n >= 3) {
    const char *path = buf + 2;
    while (*path == ' ') ++path;
    bool ok = xno::fs::remove_file(path);
    char line[64];
    snprintf(line, sizeof(line), "FR %s %s\n", ok ? "ok" : "err", path);
    reply(line);
    return;
  }

  // LD<path> [core] : 動的オブジェクトのロード & 起動
  if (buf[0] == 'L' && buf[1] == 'D' && n >= 3) {
    char path[32];
    uint32_t core = 0;
    if (sscanf(buf + 2, "%31s %lu", path, (unsigned long *)&core) >= 1) {
      xno::loader::loaded_info info{};
      bool ok = xno::loader::load_object(path, core, true /* XIP */, &info);
      char line[128];
      if (ok) {
        snprintf(line, sizeof(line), "LD ok obj=%lu tid=%lu (%s)\n",
                 (unsigned long)info.object_id, (unsigned long)info.thread_id,
                 info.is_xip ? "XIP" : "RAM");
      } else {
        snprintf(line, sizeof(line), "LD err: %s\n", path);
      }
      reply(line);
    } else {
      reply("LD err arg\n");
    }
    return;
  }

  // UN<name> : オブジェクトのアンロード
  if (buf[0] == 'U' && buf[1] == 'N' && n >= 3) {
    const char *name = buf + 2;
    while (*name == ' ') ++name;
    bool ok = xno::loader::unload_object_by_name(name);
    char line[64];
    snprintf(line, sizeof(line), "UN %s %s\n", ok ? "ok" : "err", name);
    reply(line);
    return;
  }

  // PS : ロード済み動的オブジェクト一覧
  if (strcmp(buf, "PS") == 0) {
    xno::loader::loaded_info loaded[8];
    size_t count = xno::loader::list_loaded(loaded, 8);
    char line[128];
    snprintf(line, sizeof(line), "PS count=%zu\n", count);
    reply(line);
    for (size_t i = 0; i < count; ++i) {
      if (loaded[i].is_active) {
        snprintf(line, sizeof(line), "  obj=%lu tid=%lu name=%s path=%s (%s)\n",
                 (unsigned long)loaded[i].object_id,
                 (unsigned long)loaded[i].thread_id,
                 loaded[i].name,
                 loaded[i].path,
                 loaded[i].is_xip ? "XIP" : "RAM");
        reply(line);
      }
    }
    return;
  }

  // S / STATS : 統計 & 制御状態
  if (strcmp(buf, "S") == 0 || strcmp(buf, "STATS") == 0) {
    char line[160];
    if (snprintf(line, sizeof(line),
                 "STATS cmds=%lu armed=%u elev=%.1f rud=%.1f thr=%.2f h=%.2f v=%.2f\n",
                 (unsigned long)g_commands, (unsigned)g_state.armed, (double)g_state.elevator,
                 (double)g_state.rudder, (double)g_state.throttle, (double)g_state.h_est, (double)g_state.v_est) > 0)
      reply(line);
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

uintptr_t method_add_sensor_stream(uintptr_t argument, uintptr_t, uintptr_t,
                                   uintptr_t) {
  if (g_sensor_n >= MAX_SENSOR_INPUTS)
    return 0;
  g_sensor_ids[g_sensor_n++] = argument;
  return g_sensor_n;
}

uintptr_t method_get_control_state(uintptr_t dest_ptr, uintptr_t, uintptr_t,
                                   uintptr_t) {
  if (dest_ptr == 0)
    return 0;
  memcpy((void *)dest_ptr, &g_state, sizeof(g_state));
  return sizeof(g_state);
}

// ---- 主ループ -----------------------------------------------------------------
constexpr uint32_t PERIOD_US = 2000; // 500Hz

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
    }
  }

  for (uint32_t i = 0; i < g_sensor_n; ++i) {
    if (g_sensor_ids[i] == 0 || g_sensor_ids[i] == xno::NO_STREAM)
      continue;
    const auto opened = api(shizuku::object_api::STREAM_OPEN, g_sensor_ids[i]);
    if (opened.error == 0 && opened.value != 0) {
      g_sensors[i] = shizuku::stream::handle<sample_t>(
          (shizuku::stream::descriptor *)opened.value);
      api(shizuku::object_api::STREAM_BIND, g_sensor_ids[i],
          (uintptr_t)shizuku::stream::role::CONSUMER);
    }
  }

  BOARD::diag_printf("[FC] autopilot running (rx %lu, %lu sensors -> out %lu)\n",
                     (unsigned long)g_rx_id, (unsigned long)g_sensor_n,
                     (unsigned long)g_out_id);

  uint64_t next = BOARD::time_us() + PERIOD_US;
  while (true) {
    const int64_t remaining = (int64_t)(next - BOARD::time_us());
    if (remaining > 0)
      api(shizuku::object_api::SLEEP_US, (uintptr_t)remaining);
    next += PERIOD_US;

    // 1. センササンプル処理 (高度推定 & 制御則)
    for (uint32_t i = 0; i < g_sensor_n; ++i) {
      if (!g_sensors[i].valid())
        continue;
      sample_t s{};
      uint32_t lost = 0;
      while (g_sensors[i].pop(&s, &lost)) {
        process_sample(s);
      }
    }

    // 2. RX コマンド処理
    if (g_rx.valid()) {
      frame_t f{};
      uint32_t lost = 0;
      while (g_rx.pop(&f, &lost)) {
        g_lost_in += lost;
        handle_command(f);
      }
      g_lost_in += lost;
    }
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
  failures += export_method(method::ADD_SENSOR_STREAM,
                            (uintptr_t)&method_add_sensor_stream);
  failures += export_method(method::GET_CONTROL_STATE,
                            (uintptr_t)&method_get_control_state);
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
