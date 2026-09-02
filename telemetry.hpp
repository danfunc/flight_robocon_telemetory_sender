#ifndef FLIGHT_ROBOCON_TELEMETRY_HPP
#define FLIGHT_ROBOCON_TELEMETRY_HPP
#include <cstdint>
#include "object_ids.hpp"
#include "sensor_sample.hpp"
#include "tx_frame.hpp"

// ===========================================================================
//  telemetry — センサのサンプルをテレメトリ行に符号化する
// ===========================================================================
//  ★仕事はひとつ: **サンプル → 行**。読む相手 (BLE / flash / UART) も、
//    どの行を優先するかも知らない。出口は行のストリーム 1 本だけ。
//
//  ★複数のセンサ入力を受けるのは「符号化」に内在する仕事だから — 1 行に
//    複数センサの値が入るので、合流しないと符号化できない。syslogd 型の
//    「行の合流」(どれを先に出すか) は別objectの logger が持つ。
//
//  ★書式は tools/shizuku_telemetry_client.py が読める形:
//    `PICO,` + 24 フィールドの CSV (順序と倍率は tools/shizuku_link.py の
//    TELEMETRY_FIELDS が正)。センサ由来の欄は実測で埋め、融合・制御系
//    (alt_fused / vel / speed / vstate / 舵) は **0 を出す** —
//    埋められない欄を「それらしい数」で埋めない (計測事故になる)。
//
//  ★各センサは違う周期で来る (IMU 10Hz / 気圧 5Hz)。1 行の全欄が同じ瞬間の
//    値とは限らない (欄ごとに最新が入る)。**厳密な同時刻が要る用途に
//    この行を使ってはいけない** — 要るならサンプル自体に時刻を持たせること。
namespace telemetry {

constexpr uintptr_t OBJECT = xno_object_id::telemetry;

constexpr uint32_t MAX_INPUTS = 4;

enum struct method : uintptr_t {
  MAIN = 0,
  // a0 = センサのサンプルストリーム番号。センサ 1 つにつき 1 回。
  ADD_INPUT = 1,
  // 戻り値 = 符号化した行のストリーム番号。
  GET_STREAM = 2,
  // a0 = 送出周期 [ms]。0 は弾く (黙って止まると故障と区別できない)。
  SET_RATE = 3,
  POLL = 4,
  // a0 = 高度の基準気圧 [Pa]。0 なら「起動時捕捉をやり直す」。
  // ★flight_controller 側と**同じ値を入れること**。別々に持たせているのは
  //   サンプルごとにオブジェクトを跨いで問い合わせないためで、意味が
  //   分かれてよいという話ではない。シェルの QNH コマンドが両方へ配る。
  SET_REF_PA = 5,
};

using frame_t = xno::tx_frame;
using sample_t = xno::sensor_sample;

uint32_t register_telemetry();
uint32_t start_telemetry();

} // namespace telemetry
#endif // FLIGHT_ROBOCON_TELEMETRY_HPP
