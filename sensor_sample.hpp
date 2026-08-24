#ifndef FLIGHT_ROBOCON_SENSOR_SAMPLE_HPP
#define FLIGHT_ROBOCON_SENSOR_SAMPLE_HPP
#include <cstdint>

// ===========================================================================
//  センサ → ハブ (flight_controller) が運ぶ 1 サンプル
// ===========================================================================
//  ★**数値で渡す**。以前はセンサ側が "EUL=..." のようなテキストを作って流して
//    いたが、それだと合流点が「文字列を混ぜるだけ」しかできない — テレメトリの
//    1 行に複数センサの値を**混ぜて 1 レコードにする**ことができず、ハブが
//    ハブとして機能しない (受け取った側が再パースする羽目になる)。
//    整形はハブの仕事 (D46: どう混ぜるかの方針はハブの中に置く) なので、
//    センサは生の数値だけを出す。
//
//  ★中立ヘッダに置く理由は tx_frame.hpp と同じ — センサが ble_uart も
//    flight_controller も include しなくて済むようにするため。
namespace xno {

enum struct sample_kind : uint32_t {
  NONE = 0,
  IMU = 1,  // BNO055: value[0..2]=euler, [3..5]=linear accel, [6..8]=gravity
  BARO = 2, // BME280: value[0]=press [Pa], value[1]=temp [0.01 degC]
};

// ★固定長の素朴な入れ物にしてある。センサごとに型を分けると、ハブが
//   「どの入力が何型か」を知る必要が出てストリームの本数と型が結合する。
//   kind を見て解釈する方が、入力を足すときにハブの表だけで済む。
struct sensor_sample {
  sample_kind kind;
  uint32_t sequence; // そのセンサが何個目に出したか (取りこぼしの検出用)
  int32_t value[9];
};

} // namespace xno
#endif // FLIGHT_ROBOCON_SENSOR_SAMPLE_HPP
