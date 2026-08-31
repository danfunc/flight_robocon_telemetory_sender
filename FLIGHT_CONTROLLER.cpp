// ===========================================================================
//  flight_controller — 自動操縦アルゴリズム (PID / センサフュージョン)
// ===========================================================================
#include "flight_controller.hpp"
#include "sensor_sample.hpp"
#include "tx_frame.hpp"
#include "shizuku/kernel.hpp"
#include "shizuku/object_api.hpp"
#include "shizuku/stream.hpp"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace flight_controller {
namespace {

using ARCH = shizuku::KERNEL::ARCH;
using BOARD = shizuku::KERNEL::BOARD;
using frame_t = xno::tx_frame;
using sample_t = xno::sensor_sample;

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

// 出力ストリーム (制御テレメトリ)
shizuku::stream::storage<frame_t, 16> g_out;
uintptr_t g_out_id = 0;

// 入口: センササンプルストリーム (BNO055 / BME280)
constexpr uint32_t MAX_SENSOR_INPUTS = 4;
uintptr_t g_sensor_ids[MAX_SENSOR_INPUTS] = {};
shizuku::stream::handle<sample_t> g_sensors[MAX_SENSOR_INPUTS];
uint32_t g_sensor_n = 0;

// ===========================================================================
//  自動操縦アルゴリズム (パラメータ & ゲインスケジュール)
// ===========================================================================
static constexpr int KN = 3;
static constexpr float THR_BP[KN] = {0.0f, 0.5f, 1.0f};
static constexpr float K_PITCH[KN] = {0.6f, 1.0f, 1.4f};
static constexpr float K_YAW[KN] = {0.5f, 1.0f, 1.5f};

static constexpr float KP_PITCH = 2.0f;
static constexpr float KD_PITCH = 0.4f;
static constexpr float KI_PITCH = 0.3f;
static constexpr float PITCH_I_LIMIT = 10.0f;
static constexpr float ELEV_LIMIT_DEG = 30.0f;

static constexpr float KP_YAW = 1.2f;
static constexpr float KD_YAW = 0.25f;
static constexpr float RUD_LIMIT_DEG = 30.0f;

static constexpr float KP_ALT = 0.15f;
static constexpr float KI_ALT = 0.03f;
static constexpr float KD_ALT = 0.10f;
static constexpr float ALT_I_LIMIT = 0.4f;

static constexpr float ALPHA_H = 0.95f;
static constexpr float ALPHA_V = 0.90f;
static constexpr float A_Z_CLIP = 15.0f;

enum VState : uint8_t { ST_LEVEL = 0, ST_ASC = 1, ST_DESC = 2 };

// 内部状態
static uint8_t g_armed = 1;
static float g_pitch_ref = 0.0f;
static float g_heading_ref = 0.0f;
static float g_alt_ref = 10.0f;
static float g_throttle_base = 0.5f;

static float g_pitch = 0.0f;
static float g_pitch_rate = 0.0f;
static float g_heading = 0.0f;
static float g_yaw_rate = 0.0f;
static float g_accel_z = 0.0f;
static float g_pressure_alt = 0.0f;

static float g_prev_pitch = 0.0f;
static float g_prev_heading = 0.0f;
static bool g_attitude_init = false;

static float g_h_est = 0.0f;
static float g_v_est = 0.0f;
static bool g_alt_init = false;

static float g_pitch_i = 0.0f;
static float g_alt_i = 0.0f;
static float g_elevator = 0.0f;
static float g_rudder = 0.0f;
static float g_throttle = 0.0f;
static VState g_vstate = ST_LEVEL;

} // namespace


namespace {

static float interp(const float *x, const float *y, int n, float val) {
  if (val <= x[0]) return y[0];
  if (val >= x[n - 1]) return y[n - 1];
  for (int i = 0; i < n - 1; ++i) {
    if (val >= x[i] && val <= x[i + 1]) {
      float t = (val - x[i]) / (x[i + 1] - x[i]);
      return y[i] + t * (y[i + 1] - y[i]);
    }
  }
  return y[0];
}

static float clamp(float v, float min_val, float max_val) {
  if (v < min_val) return min_val;
  if (v > max_val) return max_val;
  return v;
}

static void update_sensors() {
  for (uint32_t i = 0; i < g_sensor_n; ++i) {
    if (!g_sensors[i].valid()) continue;
    sample_t s{};
    uint32_t lost = 0;
    while (g_sensors[i].pop(&s, &lost)) {
      if (s.kind == xno::sample_kind::IMU) {
        // value[0]=heading (1/16 deg), value[1]=roll, value[2]=pitch
        g_heading = s.value[0] * (1.0f / 16.0f);
        g_pitch = s.value[2] * (1.0f / 16.0f);
        g_accel_z = s.value[5] * 0.01f; // linear accel z
      } else if (s.kind == xno::sample_kind::BARO) {
        float p = (float)s.value[0]; // Pa
        if (p > 30000.0f && p < 120000.0f) {
          g_pressure_alt = 44330.0f * (1.0f - powf(p / 101325.0f, 0.190295f));
          if (!g_alt_init) {
            g_h_est = g_pressure_alt;
            g_v_est = 0.0f;
            g_alt_init = true;
          }
        }
      }
    }
  }
}

static void compute_control(float dt) {
  if (dt <= 0.001f || dt > 0.5f) dt = 0.02f;

  // 0. 角速度（ピッチレート・ヨーレート）の算出
  if (!g_attitude_init) {
    g_prev_pitch = g_pitch;
    g_prev_heading = g_heading;
    g_attitude_init = true;
  }
  float d_pitch = g_pitch - g_prev_pitch;
  g_pitch_rate = (dt > 0.0001f) ? (d_pitch / dt) : 0.0f;
  g_prev_pitch = g_pitch;

  float d_yaw = g_heading - g_prev_heading;
  while (d_yaw > 180.0f) d_yaw -= 360.0f;
  while (d_yaw < -180.0f) d_yaw += 360.0f;
  g_yaw_rate = (dt > 0.0001f) ? (d_yaw / dt) : 0.0f;
  g_prev_heading = g_heading;

  // 1. 高度・昇降速度相補フィルタ
  float az_net = clamp(g_accel_z - 9.80665f, -A_Z_CLIP, A_Z_CLIP);
  g_v_est += az_net * dt;
  g_h_est += g_v_est * dt;
  if (g_alt_init) {
    g_h_est = ALPHA_H * g_h_est + (1.0f - ALPHA_H) * g_pressure_alt;
  }

  // 昇降状態判定 (vstate)
  if (g_v_est > 0.3f) {
    g_vstate = ST_ASC;
  } else if (g_v_est < -0.3f) {
    g_vstate = ST_DESC;
  } else {
    g_vstate = ST_LEVEL;
  }

  if (!g_armed) {
    g_elevator = 0.0f;
    g_rudder = 0.0f;
    g_throttle = 0.0f;
    g_pitch_i = 0.0f;
    g_alt_i = 0.0f;
  } else {
    // 2. 高度制御 (スロットル)
    float alt_err = g_alt_ref - g_h_est;
    g_alt_i = clamp(g_alt_i + alt_err * dt, -ALT_I_LIMIT, ALT_I_LIMIT);
    float d_alt = -g_v_est;
    float thr_cmd = g_throttle_base + (KP_ALT * alt_err + KI_ALT * g_alt_i + KD_ALT * d_alt);
    g_throttle = clamp(thr_cmd, 0.0f, 1.0f);

    // 3. ゲインスケジュール
    float k_p = interp(THR_BP, K_PITCH, KN, g_throttle);
    float k_y = interp(THR_BP, K_YAW, KN, g_throttle);

    // 4. ピッチ姿勢制御
    float pitch_err = g_pitch_ref - g_pitch;
    g_pitch_i = clamp(g_pitch_i + pitch_err * dt, -PITCH_I_LIMIT, PITCH_I_LIMIT);
    float elev_cmd = k_p * (KP_PITCH * pitch_err + KI_PITCH * g_pitch_i - KD_PITCH * g_pitch_rate);
    g_elevator = clamp(elev_cmd, -ELEV_LIMIT_DEG, ELEV_LIMIT_DEG);

    // 5. ヨー方位制御
    float yaw_err = g_heading_ref - g_heading;
    while (yaw_err > 180.0f) yaw_err -= 360.0f;
    while (yaw_err < -180.0f) yaw_err += 360.0f;
    float rud_cmd = k_y * (KP_YAW * yaw_err - KD_YAW * g_yaw_rate);
    g_rudder = clamp(rud_cmd, -RUD_LIMIT_DEG, RUD_LIMIT_DEG);
  }

}

// Method 0: MAIN
uintptr_t method_main(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  return 0;
}

// Method 2: GET_STREAM
uintptr_t method_get_stream(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  return g_out_id;
}

// Method 4: ADD_SENSOR_STREAM
uintptr_t method_add_sensor_stream(uintptr_t stream_id, uintptr_t, uintptr_t, uintptr_t) {
  if (g_sensor_n >= MAX_SENSOR_INPUTS || stream_id == xno::NO_STREAM)
    return 0;
  g_sensor_ids[g_sensor_n++] = stream_id;
  return 1;
}

// Method 5: GET_CONTROL_STATE
uintptr_t method_get_control_state(uintptr_t ptr, uintptr_t, uintptr_t, uintptr_t) {
  if (ptr != 0) {
    auto *st = (control_state *)ptr;
    st->elevator = g_elevator;
    st->rudder = g_rudder;
    st->throttle = g_throttle;
    st->pitch_ref = g_pitch_ref;
    st->heading_ref = g_heading_ref;
    st->alt_ref = g_alt_ref;
    st->h_est = g_h_est;
    st->v_est = g_v_est;
    st->vstate = (uint8_t)g_vstate;
    st->armed = g_armed;
  }
  return 0;
}

// Method 6: SET_CONTROL_STATE
uintptr_t method_set_control_state(uintptr_t ptr, uintptr_t, uintptr_t, uintptr_t) {
  if (ptr != 0) {
    const auto *st = (const control_state *)ptr;
    g_elevator = st->elevator;
    g_rudder = st->rudder;
    g_throttle = st->throttle;
    g_pitch_ref = st->pitch_ref;
    g_heading_ref = st->heading_ref;
    g_alt_ref = st->alt_ref;
    g_h_est = st->h_est;
    g_v_est = st->v_est;
    g_vstate = (VState)st->vstate;
    g_armed = st->armed;
  }
  return 0;
}

// Method 7: ARM (1=arm, 0=disarm)
uintptr_t method_arm(uintptr_t val, uintptr_t, uintptr_t, uintptr_t) {
  g_armed = val ? 1 : 0;
  return 0;
}

// Method 8: SET_PITCH_REF (a1 = pitch_cdeg)
uintptr_t method_set_pitch_ref(uintptr_t pitch_cdeg, uintptr_t, uintptr_t, uintptr_t) {
  g_pitch_ref = (float)(int32_t)pitch_cdeg * 0.01f;
  return 0;
}

// Method 9: SET_HEADING_REF (a1 = heading_cdeg)
uintptr_t method_set_heading_ref(uintptr_t heading_cdeg, uintptr_t, uintptr_t, uintptr_t) {
  g_heading_ref = (float)(int32_t)heading_cdeg * 0.01f;
  return 0;
}

// Method 10: SET_ALT_REF (a1 = alt_mm)
uintptr_t method_set_alt_ref(uintptr_t alt_mm, uintptr_t, uintptr_t, uintptr_t) {
  g_alt_ref = (float)(int32_t)alt_mm * 0.001f;
  return 0;
}

// Method 3 (POLL): 制御ループ (50Hz = 20ms 周期)
uintptr_t poll_loop(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  api(shizuku::object_api::STREAM_BIND, g_out_id,
      (uintptr_t)shizuku::stream::role::PRODUCER);

  for (uint32_t i = 0; i < g_sensor_n; ++i) {
    const auto opened = api(shizuku::object_api::STREAM_OPEN, g_sensor_ids[i]);
    if (opened.error != 0 || opened.value == 0) continue;
    g_sensors[i] = shizuku::stream::handle<sample_t>((shizuku::stream::descriptor *)opened.value);
    api(shizuku::object_api::STREAM_BIND, g_sensor_ids[i],
        (uintptr_t)shizuku::stream::role::CONSUMER);
  }

  uint32_t last_time = (uint32_t)BOARD::time_us();

  while (true) {
    uint32_t now = (uint32_t)BOARD::time_us();
    float dt = (now - last_time) * 1e-6f;
    last_time = now;

    update_sensors();
    compute_control(dt);

    api(shizuku::object_api::SLEEP_US, 20000); // 20ms (50Hz)
  }
  return 0;
}

uintptr_t fc_main(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  uintptr_t failures =
      api(shizuku::object_api::DECLARE_NAME, (uintptr_t)"flight_controller").error;
  failures += export_method(method::MAIN, (uintptr_t)&method_main);
  failures += export_method(method::GET_STREAM, (uintptr_t)&method_get_stream);
  failures += export_method(method::POLL, (uintptr_t)&poll_loop);
  failures += export_method(method::ADD_SENSOR_STREAM, (uintptr_t)&method_add_sensor_stream);
  failures += export_method(method::GET_CONTROL_STATE, (uintptr_t)&method_get_control_state);
  failures += export_method(method::SET_CONTROL_STATE, (uintptr_t)&method_set_control_state);
  failures += export_method(method::ARM, (uintptr_t)&method_arm);
  failures += export_method(method::SET_PITCH_REF, (uintptr_t)&method_set_pitch_ref);
  failures += export_method(method::SET_HEADING_REF, (uintptr_t)&method_set_heading_ref);
  failures += export_method(method::SET_ALT_REF, (uintptr_t)&method_set_alt_ref);


  g_out.init();
  const auto created =
      api(shizuku::object_api::STREAM_CREATE, (uintptr_t)&g_out.desc);
  failures += created.error;
  g_out_id = created.value;
  return failures;
}

} // namespace

uint32_t register_flight_controller() {
  const auto created = api(shizuku::object_api::CREATE_OBJECT, OBJECT,
                           (uintptr_t)&fc_main,
                           shizuku::OBJECT_PRIVILEGED | shizuku::OBJECT_ON_CORE(0));
  if (created.error != 0) return 0;
  const auto inited = api(shizuku::object_api::CALL_METHOD, OBJECT, 0, 0);
  return inited.error == 0 ? 1 : 0;
}

uint32_t start_flight_controller() {
  const auto res = api(shizuku::object_api::SPAWN, OBJECT, (uintptr_t)method::POLL, 0);
  return (uint32_t)res.value;
}

} // namespace flight_controller
