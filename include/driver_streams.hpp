#ifndef SHIZU_DRIVER_STREAMS_HPP
#define SHIZU_DRIVER_STREAMS_HPP
// ===========================================================================
//  ドライバ間ストリームの ID レジストリ (stream.hpp の利用側規約)
// ===========================================================================
//  Shizuku のストリーム ID 空間 (0 .. stream::MAX_STREAMS-1 = 0..31) は全システムで
//  ひとつなので、誰が何番を取るかをこの 1 枚に集約する。ID が衝突すると create が
//  ALREADY_BOUND を返す (先着が勝つ) か、最悪 rec_size 違いの stream を掴む —
//  後者は open_wait の rec_size 検査で invalid に落ちるが、そもそも衝突させないこと。
//
//  【カーネル登録型 (create/open_wait/bind を撃つ = core0 のオブジェクト間)】
//    1  ble_tx::STREAM_BULK    TELEMETRY  → BLE_UART   frame_t    LOSSLESS
//    2  ble_tx::STREAM_CTRL    TELEMETRY  → BLE_UART   frame_t    LOSSLESS
//    3  ID_BNO_SAMPLE          BNO055     → TELEMETRY  bno055_sample_t
//    4  ID_BARO_RAW            BNO055     → BME280     baro_raw_t
//    5  ID_BME_SAMPLE          BME280     → TELEMETRY  bme280_sample_t
//    6  ID_CALIB_RESULT        BNO055     → TELEMETRY  bno055_calib_xfer_t
//    30 stream_selftest        自己テスト (SHIZU_STREAM_SELFTEST)
//    31 stream_selftest        connect 自己テストの dst
//
//  【直接ハンドル型 (ID を取らない = コア間。core_ring.hpp が storage を持つ)】
//    core_ring::g_data_stream  core1 SENSOR_IO → core0 BNO055   record_t   LOSSY
//    core_ring::g_cmd_stream   core0 (BNO/BME) → core1          cmd_rec_t  LOSSLESS/MP
//    core_ring::g_calib_req    core0 BNO055    → core1          calib_req_t
//    core_ring::g_calib_resp   core1           → core0 BNO055   calib_resp_t
//  core1 は TU 制約 (core1_io.cpp 冒頭) で Shizuku API を sleep_us しか撃てないため、
//  コア間は create/bind を通さず共有 storage を両端が .hdl() で直接触る。
// ===========================================================================
#include <cstdint>
#include <cstdio>
#include <stream.hpp>

namespace shizu {
namespace drv_stream {

// ---- カーネル登録型ストリームの ID ------------------------------------------
// 1/2 は ble_tx_stream.hpp (STREAM_BULK/STREAM_CTRL) が使用済み。
constexpr uint32_t ID_BNO_SAMPLE = 3;   // BNO055 → TELEMETRY (融合を駆動)
constexpr uint32_t ID_BARO_RAW = 4;     // BNO055 (リング demux) → BME280
constexpr uint32_t ID_BME_SAMPLE = 5;   // BME280 → TELEMETRY (気圧高度)
constexpr uint32_t ID_CALIB_RESULT = 6; // BNO055 → TELEMETRY (較正 save/load 完了)

// ---- BNO055 の demux → BME280 に渡す生の気圧レコード --------------------------
// core1 が整数補償まで済ませた値をそのまま運ぶ (float 換算と測高公式は BME280 側)。
// CH_BARO (定常) と CH_GROUND (高度ゼロ点の基準) を is_ground で区別して 1 本に載せる
// — 別ストリームに割ると「ground が baro を追い越す」順序逆転が起き得るため。
struct baro_raw_t {
  uint32_t press_pa; // 整数 Pa
  uint32_t t_us;     // core1 のタイムスタンプ (time_us_64 下位 32bit)
  int16_t temp_cc;   // 0.01 ℃
  uint8_t is_ground; // 0 = 定常 BARO / 1 = GROUND (ゼロ点較正結果)
  uint8_t _rsv;
};
static_assert(sizeof(baro_raw_t) == 12, "baro_raw_t layout");

// ---- 登録ヘルパ -------------------------------------------------------------
//  カーネル登録型の定型 (producer: create+bind / consumer: open_wait+bind) を畳み、
//  失敗を必ず鳴らす。ここを握り潰すと「push しているのに誰も読んでいない」「pop して
//  いるのに永久に空」という、実機で最も切り分けにくい壊れ方をする。

// producer 側。失敗要因は ID 衝突 (別の desc が同じ ID を先に取った)、stream_table
// 満杯、bind 済み (二人目の producer) など。
inline bool publish(uint32_t id, stream::stream_desc_t *d, const char *who) {
  auto ce = stream::create(id, d);
  if (ce.is_err()) {
    printf("[%s] stream create id=%lu failed (err=%lu)\n", who,
           (unsigned long)id, (unsigned long)ce.raw());
    return false;
  }
  auto be = stream::bind(id, stream::role::PRODUCER);
  if (be.is_err()) {
    printf("[%s] stream bind(PRODUCER) id=%lu failed (err=%lu)\n", who,
           (unsigned long)id, (unsigned long)be.raw());
    return false;
  }
  return true;
}

// consumer 側。open_wait なので producer との起動順は問わない。戻りが invalid の
// ときは呼び出し側が縮退動作すること (固まらせない)。
template <typename REC>
inline stream::handle<REC> subscribe(uint32_t id, const char *who) {
  stream::handle<REC> h = stream::open_wait<REC>(id);
  if (!h.valid()) {
    printf("[%s] stream id=%lu not registered (degraded)\n", who,
           (unsigned long)id);
    return h;
  }
  auto be = stream::bind(id, stream::role::CONSUMER);
  if (be.is_err())
    printf("[%s] stream bind(CONSUMER) id=%lu failed (err=%lu)\n", who,
           (unsigned long)id, (unsigned long)be.raw());
  return h;
}

} // namespace drv_stream
} // namespace shizu

#endif // SHIZU_DRIVER_STREAMS_HPP
