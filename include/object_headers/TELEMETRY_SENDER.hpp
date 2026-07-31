#ifndef SHIZU_OBJECT_HEADERS_TELEMETRY_SENDER_HPP
#define SHIZU_OBJECT_HEADERS_TELEMETRY_SENDER_HPP
#include <cstdint>

namespace shizu {

// 飛行テレメトリ送信オブジェクト。
//   BME280_DRIVER / BNO055_DRIVER を read_latest で購読し、相補フィルタで高度/上下
//   速度を融合、姿勢・状態を付けて 1 行 CSV にまとめ、BLE_UART_DRIVER 経由で母艦
//   (mac) へ送る。受信 (RX) は HELLO_WORLD と同じく rx_sink 登録でコマンドを受け、
//   時刻同期 (T)・ping (P)・統計 (S)・送信周期変更 (R)・スループット試験 (B) に応える。
//
//   ダウンリンク 1 行フォーマット (すべて整数。nano printf の %f 非依存):
//     PICO,seq,up_ms,temp_cC,press_Pa,altBaro_mm,altFused_mm,vel_mm_s,
//          az_mm_s2,head_cdeg,roll_cdeg,pitch_cdeg,calib,vstate,elev,servo_cdeg
//   スケール: 角度/温度=1/100, 高度/速度/加速度=1/1000, 気圧=Pa。
class TELEMETRY_SENDER {
public:
  TELEMETRY_SENDER() {};
  ~TELEMETRY_SENDER() {};
  static void main(); // オブジェクトスレッドのエントリ

  enum METHOD_IDs : uint32_t {
    // BLE_UART の rx_sink から 1 バイトずつ呼ばれる (コマンド行の組み立て)。
    rx_byte = 0,
    // 1 = on_bno_sample / 2 = on_bme_sample / 3 = on_calib_result は退役
    // (番号は再利用しない)。センサ標本と較正結果は各ドライバが Shizuku ストリームへ
    // push し、本オブジェクトが自分のスレッドで drain して融合する
    // (driver_streams.hpp の ID_BNO_SAMPLE / ID_BME_SAMPLE / ID_CALIB_RESULT)。
  };
};
} // namespace shizu

#endif // SHIZU_OBJECT_HEADERS_TELEMETRY_SENDER_HPP
