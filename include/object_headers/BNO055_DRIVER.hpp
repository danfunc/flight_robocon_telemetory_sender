#ifndef SHIZU_OBJECT_HEADERS_BNO055_DRIVER_HPP
#define SHIZU_OBJECT_HEADERS_BNO055_DRIVER_HPP
#include <cstdint>

namespace shizu {

// BNO055 ドライバが公開する最新サンプル。read_latest にこの構造体へのポインタを
// 渡すと、ドライバが内部キャッシュをコピーする (単一アドレス空間でのポインタ渡し)。
struct bno055_sample_t {
  uint32_t seq;               // 読み出しごとに +1
  float gx, gy, gz;           // 重力ベクトル [m/s^2]
  float lax, lay, laz;        // 線形加速度 (重力除去済) [m/s^2]
  float heading, roll, pitch; // オイラー角 [deg]
  uint8_t calib;              // キャリブレーション状態 (SYS<<6|GYR<<4|ACC<<2|MAG)
  bool valid;                 // 直近の I2C 読み出しが成功したか
};

// 較正オフセットプロファイル (BNO055 レジスタ 0x55..0x6A の 22 バイト)。calib_get で
// ドライバ→呼び出し元へ、現在の保存状態とダンプを渡す。calib_load は data[22] のみ使う。
static constexpr int BNO055_CALIB_PROFILE_LEN = 22;
struct bno055_calib_xfer_t {
  uint8_t done; // 1=直近の save/load が完了
  uint8_t ok;   // 1=成功 (I2C 成功)
  uint8_t data[BNO055_CALIB_PROFILE_LEN]; // オフセットプロファイル
};

// BNO055 (9 軸 IMU、NDOF フュージョン) を Shizuku オブジェクト化したドライバ。
// 専用スレッドが I2C で姿勢/加速度を周期サンプリングし内部にキャッシュする。
class BNO055_DRIVER {
public:
  BNO055_DRIVER() {};
  ~BNO055_DRIVER() {};
  static void init(); // オブジェクトスレッドのエントリ

  enum METHOD_IDs : uint32_t {
    // arg0 = bno055_sample_t* (呼び出し元のバッファ)。最新値をコピーする。
    read_latest = 0,
    // arg0 = 0:26B ブロック読み(既定) / 1:16bit 値ごとの 2B 個別読み。
    set_read_mode = 1,
    // arg0 無視。現オフセットプロファイルを内部バッファへ吸い出す(非同期、calib_get で取得)。
    calib_save = 2,
    // arg0 = uint8_t[22] への ptr。そのオフセットプロファイルを書き戻す(非同期)。
    calib_load = 3,
    // arg0 = bno055_calib_xfer_t* 。done/ok とダンプをコピーする(save/load の結果取得)。
    calib_get = 4,
    // arg0 = 0:0xFFFF 破損バーストをそのまま公開 / 1:検出して破棄(既定)。
    set_ffff_reject = 5,
    // arg0 = (obj_id<<16)|method_id。新サンプル push 先(sink)を登録。0xFFFF.. で無効。
    set_sample_sink = 6,
    // arg0 = (obj_id<<16)|method_id。較正 save/load 完了の push 先を登録。
    set_calib_sink = 7,
    // arg0 = 0:サンプリング再開 / 非0:一時停止(スループット試験中に I2C を空ける)。
    set_paused = 8,
  };
};
} // namespace shizu

#endif // SHIZU_OBJECT_HEADERS_BNO055_DRIVER_HPP
