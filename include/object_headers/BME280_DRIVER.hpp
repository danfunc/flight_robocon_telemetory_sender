#ifndef SHIZU_OBJECT_HEADERS_BME280_DRIVER_HPP
#define SHIZU_OBJECT_HEADERS_BME280_DRIVER_HPP
#include <cstdint>

namespace shizu {

// BME280 ドライバが公開する最新サンプル。read_latest メソッドにこの構造体への
// ポインタを渡すと、ドライバが内部キャッシュをここへ memcpy する。
// 単一アドレス空間なのでポインタ渡しでオブジェクト間の値受け渡しが成立する。
struct bme280_sample_t {
  uint32_t seq; // 読み出しごとに +1 (鮮度確認用)
  // 標本時刻 (core1 の time_us_64 下位 32bit)。bno055_sample_t と同じ理由で、
  // 気圧微分 (v_baro) の dt は排出時刻ではなくこの値の差分で取ること。
  uint32_t t_us;
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
    // 2 = set_sample_sink は退役 (番号は再利用しない)。サンプルは Shizuku ストリーム
    // で配る — driver_streams.hpp の ID_BME_SAMPLE を consumer 側が open_wait すること。
    // arg0 = 0:サンプリング再開 / 非0:一時停止(スループット試験中に I2C を空ける)。
    set_paused = 3,
  };
};
} // namespace shizu

#endif // SHIZU_OBJECT_HEADERS_BME280_DRIVER_HPP
