#ifndef FLIGHT_ROBOCON_FLIGHT_CONTROLLER_HPP
#define FLIGHT_ROBOCON_FLIGHT_CONTROLLER_HPP
#include <cstdint>
#include "object_ids.hpp"
#include "sensor_sample.hpp"
#include "tx_frame.hpp"

// ===========================================================================
//  flight_controller — 自動操縦アルゴリズム & コマンド解釈
// ===========================================================================
//  ★役割:
//    1. BLE UART からの上りコマンド (Arm/Disarm, 目標高度/姿勢/方位設定, Ping等) の解釈
//    2. センサ値 (IMU / 気圧) を受け取って高度・速度の相補フィルタ推定
//    3. ゲインスケジュール付きピッチ・ヨー・高度 PID 制御則の計算
//    4. 制御出力 (Elevator / Rudder / Throttle / 状態) の生成と応答
namespace flight_controller {

constexpr uintptr_t OBJECT = xno_object_id::flight_controller;

enum struct method : uintptr_t {
  MAIN = 0,
  SET_RX_STREAM = 1,
  GET_STREAM = 2,
  POLL = 3,
  ADD_SENSOR_STREAM = 4,
  GET_CONTROL_STATE = 5,
  SET_CONTROL_STATE = 6,
  ARM = 7,             // 1=arm, 0=disarm
  SET_PITCH_REF = 8,   // a1: pitch_cdeg (deg * 100)
  SET_HEADING_REF = 9, // a1: heading_cdeg (deg * 100)
  SET_ALT_REF = 10,    // a1: alt_mm (m * 1000)
};

using frame_t = xno::tx_frame;
using sample_t = xno::sensor_sample;

struct control_state {
  float elevator;    // [deg] ±30
  float rudder;      // [deg] ±30
  float throttle;    // [0..1]
  float pitch_ref;   // [deg]
  float heading_ref; // [deg]
  float alt_ref;     // [m]
  float h_est;       // [m]
  float v_est;       // [m/s]
  uint8_t vstate;    // 0=level, 1=asc, 2=desc
  uint8_t armed;     // 1=armed, 0=disarmed
};

uint32_t register_flight_controller();
uint32_t start_flight_controller();

} // namespace flight_controller
#endif // FLIGHT_ROBOCON_FLIGHT_CONTROLLER_HPP
