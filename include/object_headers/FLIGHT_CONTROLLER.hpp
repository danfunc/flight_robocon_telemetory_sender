#ifndef SHIZU_OBJECT_HEADERS_FLIGHT_CONTROLLER_HPP
#define SHIZU_OBJECT_HEADERS_FLIGHT_CONTROLLER_HPP
#include <cstdint>

namespace shizu {

// ===========================================================================
//  FLIGHT_CONTROLLER — 飛行制御オブジェクト (計算のみ; PWM は駆動しない)
// ---------------------------------------------------------------------------
//  エルロン無しの機体向け。制御軸は ピッチ(エレベータ)/ヨー(ラダー)/スロットル。
//   ・エレベータでピッチ姿勢を一定に保持 (機体は改修済みで静安定 — 空力中心が
//     重心後方。旧機体の静的不安定を能動安定化する前提は撤廃、役割は姿勢保持のみ)
//   ・スロットルで高度保持 (本来は対気速度保持だがピトー管未装備のため当面代替)
//   ・ラダーで方位保持
//  操舵の効き(操舵反応量)はプロペラ後流=スロットルに強く依存するため、指令スロットルで
//  各軸をゲインスケジュールする (k テーブルは後日同定)。
//
//  TELEMETRY_SENDER が融合済み状態を on_state で push し、read_control で最新指令を
//  読み出してテレメトリに載せる。set_command で arm/目標値を設定する。
// ===========================================================================

// TELEMETRY_SENDER → FLIGHT_CONTROLLER へ渡す融合済み機体状態。
struct flight_state_t {
  float dt;      // 前サンプルからの経過 [s]
  float pitch;   // ピッチ角 [deg] (+ 機首上げ)
  float roll;    // ロール角 [deg]
  float heading; // 方位角 [deg] 0..360
  float alt;     // 融合高度 [m]
  float vel;     // 上下速度 [m/s] (+ 上昇)
  bool valid;    // 姿勢が有効か
};

// FLIGHT_CONTROLLER の制御出力 (TELEMETRY_SENDER が read_control で読む)。
struct control_out_t {
  float elevator;  // エレベータ指令 [deg] (+ 機首上げ)
  float rudder;    // ラダー指令 [deg] (+ 機首右)
  float throttle;  // スロットル指令 [0..1]
  float pitch_ref; // 現在のピッチ目標 [deg] (監視用)
  uint8_t vstate;  // 上下状態 (0 LEVEL / 1 ASC / 2 DESC)
  uint8_t armed;   // 制御 arm 状態 (0/1)
};

// set_command で渡す目標/コマンド (arg0 = flight_cmd_t*)。
struct flight_cmd_t {
  uint8_t kind; // 0 disarm, 1 arm, 2 alt_ref[m], 3 pitch_ref[deg],
                // 4 heading_ref[deg], 5 thr_trim[0..1]
  float value;  // kind に応じた値
};

class FLIGHT_CONTROLLER {
public:
  FLIGHT_CONTROLLER() {};
  ~FLIGHT_CONTROLLER() {};
  static void main(); // オブジェクトスレッドのエントリ

  enum METHOD_IDs : uint32_t {
    // arg0 = flight_state_t*。TELEMETRY が融合状態を push (制御則を 1 ステップ回す)。
    on_state = 0,
    // arg0 = control_out_t*。最新の制御出力を呼び出し元バッファへコピーする。
    read_control = 1,
    // arg0 = flight_cmd_t*。arm/目標高度/ピッチトリム/方位/スロットルトリムを設定。
    set_command = 2,
  };
};

} // namespace shizu

#endif // SHIZU_OBJECT_HEADERS_FLIGHT_CONTROLLER_HPP
