#ifndef FLIGHT_ROBOCON_BME280_HPP
#define FLIGHT_ROBOCON_BME280_HPP
#include <cstdint>
#include "object_ids.hpp"
#include "sensor_sample.hpp"

// ===========================================================================
//  BME280 (気圧/温度) — Shizuku オブジェクト版 (v1: 読めて送れるかの確認用)
// ===========================================================================
//  移植元の知見: flight_robocon_telemetory_sender/core1_io.cpp
//  (BME280_DRIVER.cpp から移植された整数補償部)。BNO055 と同じ I2C バスを
//  共有する (I2C0/GP4-5、bno055.cpp が CONFIGURE 済みならそれを流用)。
//
//  ★v1 で見送ったもの: 地上気圧較正 (逐次平均)、湿度 (このセンサは温度/気圧
//    のみのボード品番を想定 — 湿度レジスタがあれば later)。
namespace bme280 {

constexpr uintptr_t OBJECT = xno_object_id::bme280;

enum struct method : uintptr_t {
  MAIN = 0,
  POLL = 1, // SPAWN 専用
  // 戻り値 = このセンサの出力ストリーム番号 (ハブが STREAM_OPEN で購読)。
  GET_STREAM = 2,
};

using sample_t = xno::sensor_sample;

// 生成 + export + ストリーム作成まで。poll はまだ起こさない。
uint32_t register_bme280();
// 配線後に呼ぶ。
uint32_t start_bme280();

} // namespace bme280
#endif // FLIGHT_ROBOCON_BME280_HPP
