#ifndef SHIZU_OBJECT_HEADERS_BLE_UART_DRIVER_HPP
#define SHIZU_OBJECT_HEADERS_BLE_UART_DRIVER_HPP
#include <cstdint>

namespace shizu {

// send_buf に渡す送信バッファ記述子。呼び出し元のバッファへのポインタと長さを
// 1 個のポインタで渡す(センサの read_latest 等と同じアドレス渡し)。data は
// メソッド呼び出しが返るまで有効であればよい(同期コピーされる)。
struct ble_tx_buf_t {
  const uint8_t *data;
  uint32_t len;
};

// Nordic UART Service (NUS) ペリフェラルを Shizuku オブジェクト化したもの。
//   RX char (6E400002, Write/WriteWithoutResponse) : Central -> Peripheral
//   TX char (6E400003, Notify)                     : Peripheral -> Central
// 受信バイトは set_rx_sink で登録した (obj_id, method_id) へ call_method で配送し、
// 送信は send_byte で内部 TX リングへ積み、poll ループが notify で flush する。
class BLE_UART_DRIVER {
public:
  BLE_UART_DRIVER() {};
  ~BLE_UART_DRIVER() {};
  static void init(); // オブジェクトスレッドのエントリ

  enum METHOD_IDs : uint32_t {
    // arg0 の下位 8bit を 1 バイトとして TX リングに積む。
    send_byte = 0,
    // arg0 = (sink_obj_id << 16) | sink_method_id。
    // 受信 1 バイトごとに call_method(sink_obj_id, sink_method_id, byte) を呼ぶ。
    // sink_obj_id == 0xFFFF で無効化。
    set_rx_sink = 1,
    // arg0 = ble_tx_buf_t* 。バッファをまとめて TX リングへ積む(1 バイトずつ
    // call_method する代わりの一括送信)。send_byte と同じ行フレーミング(\n で確定)。
    send_buf = 2,
    // arg0 = uint32_t* 。TX リングの現在の空きバイト数を書き込む。供給側の
    // バックプレッシャ用 (ブラストが「入る分だけ生成」するために使う)。
    get_tx_free = 3,
  };
};
} // namespace shizu

#endif // SHIZU_OBJECT_HEADERS_BLE_UART_DRIVER_HPP
