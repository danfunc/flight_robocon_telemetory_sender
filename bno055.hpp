#ifndef FLIGHT_ROBOCON_BNO055_HPP
#define FLIGHT_ROBOCON_BNO055_HPP
#include <cstdint>
#include "object_ids.hpp"
#include "sensor_sample.hpp"

// ===========================================================================
//  BNO055 (IMU) — Shizuku オブジェクト版 (v1: 読めて送れるかの確認用)
// ===========================================================================
//  移植元の知見: flight_robocon_telemetory_sender/core1_io.cpp
//  (BNO055_DRIVER.cpp から移植された整数ロジック部)。
//
//  ★v1 で見送ったもの (旧実装にはあるが、ここには無い):
//    - split-read/burst-read の A/B 切替、0xFFFF 破損の据え置き再送、
//      オイラー角デッドバンド → まず「読めて送れるか」を確認するのが目的
//      なので、素の burst read 1 回だけ
//    - キャリブレーション状態の読み出し・保存/復元
//    - core1 ピン留め (BNO055 自体は cyw43 のような単一コア制約が無いので
//      不要。I2C オブジェクトを共有する別ドライバが増えたら要再検討)
//    - 100Hz 駆動 (旧実装は core1 専有だった。ここでは BLE の poll と
//      同じ core0 を共有するので 10Hz に落としてある)
namespace bno055 {

constexpr uintptr_t OBJECT = xno_object_id::bno055;

enum struct method : uintptr_t {
  MAIN = 0,
  POLL = 1, // SPAWN 専用。CALL_METHOD で直接呼ばないこと (無限ループ)。
  // 戻り値 = このセンサの出力ストリーム番号。ハブ (flight_controller) が
  // STREAM_OPEN で購読する。★席は 1 つなので読み手はちょうど 1 オブジェクト。
  GET_STREAM = 2,
};

using sample_t = xno::sensor_sample;

// 生成 + export + ストリーム作成まで。poll はまだ起こさない。
uint32_t register_bno055();
// 配線後に呼ぶ。
uint32_t start_bno055();

} // namespace bno055
#endif // FLIGHT_ROBOCON_BNO055_HPP
