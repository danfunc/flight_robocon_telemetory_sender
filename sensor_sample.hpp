#ifndef FLIGHT_ROBOCON_SENSOR_SAMPLE_HPP
#define FLIGHT_ROBOCON_SENSOR_SAMPLE_HPP
#include <cmath>
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


// 気圧から高度 [m]。国際標準大気の素直な式。
//
// ★★**基準圧 ref_pa の意味づけが、そのまま高度の意味になる**:
//     ref_pa = 離陸地点で測った気圧 (QFE 相当) → 離陸地点からの高度。起動時 0
//     ref_pa = 海面更正気圧 (QNH)              → 海抜高度
//   どちらも同じ式で、違うのは基準だけ。だから変数 1 本で両対応できる。
//
// ★★以前は 101325 Pa 固定で、**絶対高度のつもりの値を制御に使っていた**。
//   flight_controller の目標高度 `g_alt_ref = 10.0f` は「標高 10m (ISA 絶対)」を
//   意味してしまい、標高 50m の場所では地面より 40m 下を狙う。`alt <m>` で
//   打つ値も同じ。**離陸地点基準にして初めて、この 10 が見た目どおりになる。**
//
// ★離陸地点基準は「電源投入時の高度が 0」であって「地面が 0」ではない。
//   飛行中に再起動すると**その高度が新しい 0 になる**。基準を持ち回れない
//   以上これは避けられないので、再起動を伴う運用では必ず意識すること。
inline float altitude_m(float press_pa, float ref_pa) {
  if (press_pa <= 0.0f || ref_pa <= 0.0f)
    return 0.0f;
  return 44330.0f * (1.0f - powf(press_pa / ref_pa, 1.0f / 5.255f));
}

// 起動時に基準圧を捕まえるときの平均サンプル数。★1 点で決めない —
//   BME280 の 1 サンプルのノイズがそのまま「地面の高さ」になるため。
constexpr uint32_t BARO_REF_SAMPLES = 8;

} // namespace xno
#endif // FLIGHT_ROBOCON_SENSOR_SAMPLE_HPP
