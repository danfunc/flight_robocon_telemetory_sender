#ifndef FLIGHT_ROBOCON_BLINK_HPP
#define FLIGHT_ROBOCON_BLINK_HPP
#include <cstdint>
#include "object_ids.hpp"

// ===========================================================================
//  blink — ホットプラグ・ライフサイクル対応 LED 点滅オブジェクト
// ===========================================================================
//  ★GDB で「止まったことが目で見える」ための対象。
//    独立した Shizuku オブジェクトとしてライフサイクル（登録・起動・停止・動的設定）
//    を管理し、ホットプラグ（動的アンロード・再起動・パラメータ変更）に対応する。
namespace blink {

constexpr uintptr_t OBJECT = xno_object_id::blink;

enum struct method : uintptr_t {
  MAIN = 0,
  POLL = 1,
  SET_INTERVAL = 2,
  SET_ENABLED = 3,
  GET_STATE = 4,
  STOP = 5,
  SET_PATTERN = 6,
  SET_SPEED = 7,
  SAVE_CONFIG = 8,
  LOAD_CONFIG = 9,
};

enum struct pattern_id : uint8_t {
  DYNAMIC_SWEEP = 0,     // ★既定: OTA検証用ダイナミックスイープ (200ms〜1400ms 往復掃引)
  DOUBLE_HEARTBEAT = 1,  // 航空機アビオニクス (ダブルパルス)
  TRIPLE_BEACON = 2,     // 3連フラッシュ・ビーコン
  MORSE_S = 3,           // モールス S 信号
  FAST_STROBE = 4,       // 高速ストロボ (30ms/30ms: OTA受信中・ビジー)
  PROGRESSIVE_BURST = 5, // プログレッシブ・バースト
  COUNT = 6,
};

struct config {
  uint32_t magic;      // 'BLNK' (0x4B4E4C42)
  uint8_t pattern_id;  // 0..5
  uint8_t speed_pct;   // 20..300
  uint16_t reserved;
};

struct state {
  uint32_t interval_ms;
  uint32_t toggle_count;
  uint8_t enabled;
  uint8_t led_value;
  uint8_t pattern_id;
  uint8_t speed_pct;
};

uint32_t register_blink(uintptr_t obj_id = OBJECT);
uint32_t start_blink(uintptr_t obj_id = OBJECT);
uint32_t stop_blink(uintptr_t obj_id = OBJECT);
uint32_t get_thread_id();

} // namespace blink
#endif // FLIGHT_ROBOCON_BLINK_HPP
