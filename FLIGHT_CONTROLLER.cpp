// ===========================================================================
//  FLIGHT_CONTROLLER — 飛行制御オブジェクト (計算のみ; PWM は駆動しない)
// ===========================================================================
//  エルロン無し機体。制御軸 = ピッチ(エレベータ)/ヨー(ラダー)/スロットル。
//
//  設計の流れ (ユーザ指定: 操舵反応量 → 運動 → 制御則):
//
//  1) 操舵反応量モデル k(スロットル)  [経験テーブル, 後日同定]
//     プロペラと動翼が近接し、操舵の効きはプロペラ後流 = 指令スロットル u_thr に
//     強く依存する。軸ごとの効き k_pitch(u_thr) / k_yaw(u_thr) を折れ線テーブル+
//     線形補間で表す。値は同定前のプレースホルダ (下記 K_PITCH/K_YAW)。
//
//  2) 運動モデル (設計根拠; ここでは制御則の前提として明記)
//     ピッチ: theta_ddot = k_pitch(u_thr)*elev + M_ALPHA*theta
//                          - c_p*theta_dot + m_prop*u_thr
//       ・M_ALPHA … 静安定余裕の符号付き係数。現行機体は空力中心が重心後方に
//         是正済みで M_ALPHA<0 (復元モーメント = 静安定)。
//         (履歴: 初期機体は空力中心が重心前方で M_ALPHA>0 の静的不安定 —
//          放置で急ピッチアップ — であり、本制御則はその能動安定化を前提に
//          設計された。改修後も構造は変えていない。)
//       ・m_prop*u_thr … 後流がエレベータを直撃するスロットル起因のピッチ外乱
//     ヨー:   psi_ddot   = k_yaw(u_thr)*rud - c_y*psi_dot
//     高度:   h_dot      ~= gamma*(u_thr - thr_trim)   (ピッチ一定下, backside)
//
//  3) 制御則 (軸別, スロットルでゲインスケジュール)
//     ・ピッチ内側: 姿勢一定 PD(+I)。現行機体は静安定なので役割は姿勢保持と
//       外乱抑圧。後流外乱を前置補償し、効き正規化 elev = v_p / k_pitch(u_thr)
//       で throttle 依存を打ち消す (ループ利得を一定化)。
//     ・ヨー: 方位保持 PD、rud = v_y / k_yaw(u_thr)。
//     ・高度外側: スロットル PI(+上下速度ダンピング)。この u_thr を上2軸の
//       スケジュール変数に使い、機体固有の相互結合を陽に扱う。
//
//  本来は対気速度保持が正道だがピトー管未装備のため当面 スロットル→高度 で代替。
//  出力はテレメトリに載せるのみ (PWM 未駆動)。TELEMETRY_SENDER が on_state で融合
//  状態を push し、read_control で最新指令を読む。set_command で arm/目標を設定。
//
//  ★ ゲイン(KP_*/KD_*/KI_*) と k テーブルは同定前のプレースホルダ。実測(ステップ
//    応答)から k(スロットル) と各ゲインを同定して差し替えること。
// ===========================================================================
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <export_method.hpp>
#include <obj_api.hpp>
#include <object_headers/FLIGHT_CONTROLLER.hpp>
#include <object_id.hpp>

namespace shizu {

// ---- 操舵反応量テーブル k(スロットル) [後日同定のプレースホルダ] -----------
// throttle=0.5 付近を ~1.0 に正規化した相対効き。絶対スケールはゲインへ畳み込まれる
// ので、重要なのは「throttle と共に増える形」。floor を残し 0 割りを避ける。
static constexpr int KN = 5;
static const float THR_BP[KN] = {0.00f, 0.25f, 0.50f, 0.75f, 1.00f};
static const float K_PITCH[KN] = {0.35f, 0.65f, 1.00f, 1.45f, 1.85f};
static const float K_YAW[KN]   = {0.30f, 0.55f, 1.00f, 1.40f, 1.70f};

// ---- 制御ゲイン [プレースホルダ, 要チューニング] ---------------------------
// ピッチ内側 (姿勢一定)。機体静安定化 (M_ALPHA<0) により旧安定条件
// 「KP_PITCH > M_ALPHA 相当」は拘束でなくなった — ゲインは応答整形だけで決めてよい。
static constexpr float KP_PITCH = 2.0f;      // [elev-authority / deg]
static constexpr float KD_PITCH = 0.4f;      // [elev-authority / (deg/s)]
static constexpr float KI_PITCH = 0.3f;      // [elev-authority / (deg*s)]
static constexpr float M_PROP = 0.0f;        // 後流ピッチ外乱の前置補償 [authority/throttle]。TBD, 既定0
static constexpr float PITCH_I_LIMIT = 10.0f;
static constexpr float ELEV_LIMIT_DEG = 30.0f;

// ヨー (方位保持)
static constexpr float KP_YAW = 1.2f;        // [rud-authority / deg]
static constexpr float KD_YAW = 0.25f;       // [rud-authority / (deg/s)]
static constexpr float RUD_LIMIT_DEG = 30.0f;

// 高度外側 (スロットル)
static constexpr float KP_ALT = 0.15f;       // [throttle / m]
static constexpr float KI_ALT = 0.03f;       // [throttle / (m*s)]
static constexpr float KD_ALT = 0.10f;       // [throttle / (m/s)] 上下速度ダンピング
static constexpr float ALT_I_LIMIT = 0.4f;

// 上昇/下降ヒステリシス閾値 [m/s]
static constexpr float V_ENTER_ASC = +0.30f, V_LEAVE_ASC = +0.10f;
static constexpr float V_ENTER_DESC = -0.30f, V_LEAVE_DESC = -0.10f;
enum VState { ST_LEVEL = 0, ST_ASC = 1, ST_DESC = 2 };

// ---- 設定/目標 (set_command で更新) ----------------------------------------
static bool g_armed = false;
static float g_pitch_ref = 0.0f;    // ピッチ目標(トリム姿勢) [deg]
static float g_heading_ref = 0.0f;  // 方位目標 [deg]
static bool g_heading_ref_set = false;
static float g_alt_ref = 1.0f;      // 高度目標 [m]
static float g_thr_trim = 0.4f;     // スロットル基準 [0..1]

// ---- 制御内部状態 ----------------------------------------------------------
static float g_pitch_i = 0.f, g_alt_i = 0.f;
static float g_pitch_prev = 0.f, g_heading_prev = 0.f;
static bool g_primed = false;
static VState g_vstate = ST_LEVEL;
static control_out_t g_out = {};

// ---- 小道具 (soft-float 安全: libm を呼ばない) -----------------------------
static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}
static inline float clamp_sym(float x, float lim) { return clampf(x, -lim, lim); }

// 角度差を [-180,180] へ (heading 0..360 の差は高々 ±360 なので while で十分)。
static inline float wrap180(float d) {
  while (d > 180.f)
    d -= 360.f;
  while (d < -180.f)
    d += 360.f;
  return d;
}

// 折れ線テーブルの線形補間。
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

// ---- disarm/無効時の中立出力 -----------------------------------------------
static void neutral_out(const flight_state_t &s) {
  g_pitch_i = 0.f;
  g_alt_i = 0.f;
  g_pitch_prev = s.pitch;
  g_heading_prev = s.heading;
  g_primed = true;
  g_out.elevator = 0.f;
  g_out.rudder = 0.f;
  g_out.throttle = g_armed ? g_thr_trim : 0.f;
  g_out.pitch_ref = g_pitch_ref;
  g_out.vstate = (uint8_t)g_vstate;
  g_out.armed = g_armed ? 1u : 0u;
}

// ===========================================================================
//  push: 融合状態を受けて制御則を 1 ステップ回す (TELEMETRY から同期呼び出し)
// ===========================================================================
static void handle_state(uint32_t, uint32_t, uint32_t ptr, uint32_t) {
  if (ptr == 0)
    return;
  flight_state_t s;
  memcpy(&s, (const void *)(uintptr_t)ptr, sizeof(s));

  float dt = s.dt;
  if (dt <= 0.f || dt > 0.5f)
    dt = 0.02f; // 異常 dt ガード

  if (!g_armed || !s.valid) {
    neutral_out(s);
    return;
  }

  // arm 後の初回に方位目標を捕捉 (set_command で明示設定済みなら尊重)。
  if (!g_heading_ref_set) {
    g_heading_ref = s.heading;
    g_heading_ref_set = true;
  }

  // --- 高度外側ループ → スロットル (これがスケジュール変数にもなる) ---
  float h_err = g_alt_ref - s.alt;
  g_alt_i = clamp_sym(g_alt_i + h_err * dt, ALT_I_LIMIT);
  float u_thr = g_thr_trim + KP_ALT * h_err + KI_ALT * g_alt_i - KD_ALT * s.vel;
  u_thr = clampf(u_thr, 0.f, 1.f);

  // 指令スロットルで各軸の効きをスケジュール。
  float kp_eff = k_lookup(THR_BP, K_PITCH, KN, u_thr);
  float ky_eff = k_lookup(THR_BP, K_YAW, KN, u_thr);

  // --- ピッチ内側ループ (姿勢一定) ---
  float th_rate = g_primed ? (s.pitch - g_pitch_prev) / dt : 0.f;
  g_pitch_prev = s.pitch;
  float e_p = g_pitch_ref - s.pitch;
  g_pitch_i = clamp_sym(g_pitch_i + e_p * dt, PITCH_I_LIMIT);
  float v_p = KP_PITCH * e_p - KD_PITCH * th_rate + KI_PITCH * g_pitch_i;
  v_p -= M_PROP * u_thr;               // 後流ピッチ外乱の前置補償
  float elev = clamp_sym(v_p / kp_eff, ELEV_LIMIT_DEG); // 効き正規化

  // --- ヨーループ (方位保持) ---
  float psi_rate = g_primed ? wrap180(s.heading - g_heading_prev) / dt : 0.f;
  g_heading_prev = s.heading;
  float e_y = wrap180(g_heading_ref - s.heading);
  float v_y = KP_YAW * e_y - KD_YAW * psi_rate;
  float rud = clamp_sym(v_y / ky_eff, RUD_LIMIT_DEG);

  g_primed = true;
  g_vstate = next_state(g_vstate, s.vel);

  g_out.elevator = elev;
  g_out.rudder = rud;
  g_out.throttle = u_thr;
  g_out.pitch_ref = g_pitch_ref;
  g_out.vstate = (uint8_t)g_vstate;
  g_out.armed = 1u;
}

// ===========================================================================
//  read_control: 最新の制御出力を呼び出し元バッファへコピー (TELEMETRY が読む)
// ===========================================================================
static void handle_read_control(uint32_t, uint32_t, uint32_t ptr, uint32_t) {
  if (ptr == 0)
    return;
  memcpy((void *)(uintptr_t)ptr, &g_out, sizeof(g_out));
}

// ===========================================================================
//  set_command: arm/目標値の設定
// ===========================================================================
static void handle_set_command(uint32_t, uint32_t, uint32_t ptr, uint32_t) {
  if (ptr == 0)
    return;
  flight_cmd_t c;
  memcpy(&c, (const void *)(uintptr_t)ptr, sizeof(c));
  switch (c.kind) {
  case 0: // disarm
    g_armed = false;
#if !SHIZU_STEP1_UNPRIV_FLIGHT_CONTROLLER
    // unprivileged ビルドでは呼ばない: pico-sdk の stdio 内部が critical section で
    // PRIMASK 経由の CPSID を使うことがあり、CPS 系命令は ARMv8-M で unprivileged
    // 実行時に NOP 化される (フォールトせず割り込み禁止だけがサイレントに効かなく
    // なる) — kernel.hpp SHIZU_STEP1_UNPRIV_FLIGHT_CONTROLLER のコメント参照。
    printf("[FLIGHT] disarm\n");
#endif
    break;
  case 1: // arm (方位目標を再捕捉, 積分リセット)
    g_armed = true;
    g_heading_ref_set = false;
    g_pitch_i = 0.f;
    g_alt_i = 0.f;
#if !SHIZU_STEP1_UNPRIV_FLIGHT_CONTROLLER
    printf("[FLIGHT] arm\n");
#endif
    break;
  case 2: // alt_ref [m]
    g_alt_ref = c.value;
    break;
  case 3: // pitch_ref [deg]
    g_pitch_ref = c.value;
    break;
  case 4: // heading_ref [deg]
    g_heading_ref = c.value;
    g_heading_ref_set = true;
    break;
  case 5: // thr_trim [0..1]
    g_thr_trim = clampf(c.value, 0.f, 1.f);
    break;
  default:
    break;
  }
}

// ===========================================================================
//  オブジェクトエントリ
// ===========================================================================
void FLIGHT_CONTROLLER::main() {
#if !SHIZU_STEP1_UNPRIV_FLIGHT_CONTROLLER
  printf("[FLIGHT] main\n");
#endif
  // 制御はすべて on_state の push (TELEMETRY からの同期呼び出し) で駆動する。
  export_method<handle_state>(FLIGHT_CONTROLLER::METHOD_IDs::on_state);
  export_method<handle_read_control>(FLIGHT_CONTROLLER::METHOD_IDs::read_control);
  export_method<handle_set_command>(FLIGHT_CONTROLLER::METHOD_IDs::set_command);
  while (true) {
    obj_api::yield_us(1000000); // ハートビート (実処理は push ハンドラ)
  }
}

} // namespace shizu
