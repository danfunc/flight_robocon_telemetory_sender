#ifndef SHIZU_OBJECT_HEADERS_BME280_DRIVER_HPP
#define SHIZU_OBJECT_HEADERS_BME280_DRIVER_HPP
#include <cstdint>

namespace shizu {

// BME280 ドライバが公開する最新サンプル。read_latest メソッドにこの構造体への
// ポインタを渡すと、ドライバが内部キャッシュをここへ memcpy する。
// 単一アドレス空間なのでポインタ渡しでオブジェクト間の値受け渡しが成立する。
struct bme280_sample_t {
  uint32_t seq;     // 読み出しごとに +1 (鮮度確認用)
  float temp_c;     // 温度 [℃]
  float press_hpa;  // 気圧 [hPa]
  float alt_m;      // 地上気圧基準の相対高度 [m] (init 時にキャリブレーション)
  bool valid;       // 直近の I2C 読み出しが成功したか
};

// BME280 (気圧/温度) を Shizuku オブジェクト化したドライバ。
// 専用スレッドが I2C で周期サンプリングし、補償演算後の値を内部にキャッシュする。
// 他オブジェクト (TELEMETRY_SENDER 等) は read_latest を call_method で叩いて
// スナップショットを受け取る (I2C を触るのは本オブジェクトのスレッドだけ)。
class BME280_DRIVER {
public:
  BME280_DRIVER() {};
  ~BME280_DRIVER() {};
  static void init(); // オブジェクトスレッドのエントリ

  enum METHOD_IDs : uint32_t {
    // arg0 = bme280_sample_t* (呼び出し元のバッファ)。最新値をコピーする。
    read_latest = 0,
    // arg0 = 無視。地上気圧を現在値で取り直す (高度ゼロ点の再較正)。
    rezero = 1,
    // arg0 = (obj_id<<16)|method_id。新サンプル push 先(sink)を登録。0xFFFF... で無効。
    set_sample_sink = 2,
  };
};
} // namespace shizu

#endif // SHIZU_OBJECT_HEADERS_BME280_DRIVER_HPP
