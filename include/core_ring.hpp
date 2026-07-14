#ifndef SHIZU_CORE_RING_HPP
#define SHIZU_CORE_RING_HPP
// ===========================================================================
//  コア間 SPSC リング (docs/sensor_stream_protocol.md §4)
// ===========================================================================
//  core1 (ベアメタル I/O コア) → core0 (Shizuku) のセンサレコードストリームと、
//  core0 → core1 の小さなコマンドリング、および較正プロファイル (22B、レア) 用の
//  サイドバンド共有構造体を定義する。
//
//  ・SPSC: 各リングとも producer / consumer は 1 つずつ。インデックスは u32
//    モノトニック (mod N でスロット化)。producer は payload を書いてから
//    __dmb() して wr を公開する。RP2350 の SRAM はデータキャッシュ無しなので
//    volatile + DMB で可視性は足りる。
//  ・データリングは「上書き式 (lossy)」: producer は決してブロックせず常に
//    書き進む。溢れは consumer 側が wr-rd の距離で検出し、古い方を捨てて
//    ドロップ数を数える (設計書 §4「溢れたら古い方を捨てる」)。
//  ・コマンドリング producer は core0 の複数スレッドから呼ばれ得るが、Shizuku は
//    協調型単一コアで push 中に yield しないため実質直列化されており SPSC 侵害は
//    起きない。
// ===========================================================================
#include <cstdint>
#include <cstring>
#include <hardware/sync.h> // __dmb
#include <pico/platform.h> // __not_in_flash_func
#include <stream.hpp>      // データリングを stream API の storage として正式化

namespace shizu {
namespace core_ring {

// ---- チャンネル ID ---------------------------------------------------------
// 0x01..0x04, 0x7E/0x7F は設計書 §1。0x11..0x13 / 0x05 は step2 (現行 NDOF の
// まま core1 化) の一時チャンネル (設計書 §4.1)。AMG 化 (step4) で raw ACC/GYR/MAG
// チャンネルへ置き換わる予定。
enum ch_id : uint8_t {
  CH_BARO = 0x04,   // press_pa u32 LE + temp i16 (0.01 ℃) — core1 で整数補償済み
  CH_GROUND = 0x05, // BARO と同形式。地上基準 (起動時 40 平均 / rezero 20 平均)
  CH_EUL = 0x11,    // オイラー角 h/r/p 3×int16 (1/16 deg, デッドバンド適用済み)
  CH_LIA = 0x12,    // 線形加速度 x/y/z 3×int16 (1/100 m/s^2)
  CH_GRV = 0x13,    // 重力ベクトル x/y/z 3×int16 (1/100 m/s^2)
  CH_DIAG = 0x7D,   // 1Hz: 0xFFFF 破損率 A/B [0]=read_mode(0blk/1split),[1]=reject,[2:3]=reads u16,[4:5]=ffff u16
  CH_STATUS = 0x7E, // 1Hz: [0]=calib,[1]=health,[2:3]=i2c_fail u16,[4]=recover,[5]=reinit
};

// STATUS payload [1] (health) のビット。
constexpr uint8_t HEALTH_BNO_OK = 1 << 0;
constexpr uint8_t HEALTH_BME_OK = 1 << 1;
constexpr uint8_t HEALTH_BNO_PAUSED = 1 << 2;
constexpr uint8_t HEALTH_BME_PAUSED = 1 << 3;

// record_t.flags
constexpr uint8_t FLAG_OFF_GRID = 1 << 0; // グリッド外サンプル (リトライ後など)
constexpr uint8_t FLAG_GAP = 1 << 1;      // 直前に欠落あり

// ---- レコード (12B 固定, 設計書 §4) ----------------------------------------
// フィールド順は設計書どおり (t_us がオフセット 2 に来るため packed。M33 は
// 非アラインアクセス可)。
struct __attribute__((packed)) record_t {
  uint8_t ch_id;
  uint8_t flags;
  uint32_t t_us;      // time_us_64() 下位 32bit (両コア共通タイマ)
  uint8_t payload[6]; // チャンネル固有
  uint8_t _pad[4];    // DMA_RING 用: 16B(2冪)へパディング。未使用 (consumer は無視)
};
static_assert(sizeof(record_t) == 16, "record_t padded to 16B for DMA_RING ring");

// ---- SPSC リング (上書き式) -------------------------------------------------
template <typename REC, uint32_t N> struct spsc_ring_t {
  volatile uint32_t wr; // producer 専有 (publish 済みレコード数)
  volatile uint32_t rd; // consumer 専有
  REC buf[N];

  // 消費者が overrun 検出後に巻き戻す位置の安全マージン。producer は publish 前の
  // スロット (wr % N) を書いている最中かもしれないので、wr-N ちょうどまで戻ると
  // 次の pop で再び torn になり得る。数レコード新しい側へ寄せて回避する。
  static constexpr uint32_t RESYNC_MARGIN = 8;

  // producer (唯一): 常に成功する。満杯なら最古を黙って上書き (consumer が検出)。
  // push/pop は両コアのホットパスから毎周期呼ばれるため SRAM (.time_critical) に
  // 置く (core1 はフラッシュ実行だと XIP バスを core0 の BTstack と取り合う)。
  void __not_in_flash_func(push)(const REC &r) {
    uint32_t w = wr;
    buf[w % N] = r;
    __dmb();   // payload の書き込みを wr 公開より先に完了させる
    wr = w + 1;
  }

  // consumer (唯一): 1 件取り出す。上書きで失われた件数を *lost に加算する。
  // 戻り false = 空 (または torn 検出でリトライ待ち)。
  bool __not_in_flash_func(pop)(REC *out, uint32_t *lost) {
    uint32_t w = wr;
    __dmb(); // wr 観測 → buf 読みの順序を保証
    uint32_t r = rd;
    if (r == w)
      return false;
    if (w - r >= N) { // 追い越された: 古い方を捨てて前へ
      uint32_t nr = w - N + RESYNC_MARGIN;
      if (lost)
        *lost += nr - r;
      r = nr;
    }
    REC tmp = buf[r % N];
    __dmb(); // buf 読み終わり → wr 再検証の順序を保証
    // コピー中に producer が一周してきたら tmp は破損の可能性 (torn) → 捨てる。
    // (producer は publish 前にスロットを書くので、閾値は > ではなく >= N)
    if (wr - r >= N) {
      uint32_t nr = wr - N + RESYNC_MARGIN;
      if (lost)
        *lost += nr - r;
      rd = nr;
      return false;
    }
    rd = r + 1;
    *out = tmp;
    return true;
  }
};

// ---- コマンド (core0 → core1) ----------------------------------------------
enum cmd_op : uint8_t {
  CMD_SET_PAUSED_BNO = 1, // arg: 0=再開 / 非0=停止
  CMD_SET_PAUSED_BME = 2, // arg: 同上
  CMD_SET_READ_MODE = 3,  // arg: 0=26B ブロック読み / 非0=2B 個別読み
  CMD_SET_FFFF_REJECT = 4,// arg: 0=素通し / 非0=0xFFFF 破損を据え置き (既定)
  CMD_REZERO = 5,         // 地上気圧再較正 (20 サンプル平均 → CH_GROUND で返る)
  CMD_SET_FAIL_BACKOFF = 6, // arg: N=連続失敗 N 回目で 20Hz 退避 (1=毎回退避=旧, 既定5)
};
struct cmd_rec_t {
  uint8_t op;
  uint8_t arg;
  uint8_t _rsv[2];
  uint32_t arg32;
};
static_assert(sizeof(cmd_rec_t) == 8, "cmd_rec_t must be 8 bytes");

// ---- 較正プロファイル サイドバンド (22B、レア) ------------------------------
// ストリームには乗せない (設計書 §4.2)。ハンドシェイク:
//   core0: (load なら data を書き) op を書く → __dmb() → req_seq++ で発行
//   core1: req_seq != ack_seq を検出 → 実行 → (save なら data,) ok を書く
//          → __dmb() → ack_seq = req_seq で完了通知
// core0 は ack_seq の変化で完了を知る。多重発行時は core1 が最後の req のみ処理。
struct calib_sideband_t {
  volatile uint32_t req_seq;
  volatile uint32_t ack_seq;
  volatile uint8_t op; // 1=save (BNO→data) / 2=load (data→BNO)
  volatile uint8_t ok; // 1=I2C 成功
  uint8_t data[22];    // BNO055 較正オフセットプロファイル (0x55..0x6A)
};

// ---- リングの実体 (inline 変数、.bss でゼロ初期化) ---------------------------
using cmd_ring_t = spsc_ring_t<cmd_rec_t, 16>;

// データストリーム (core1 SENSOR_IO producer → core0 consumer)。stream API の storage
// として正式化 + DMA_RING レイアウト化した。16B × 512 = 8192B (2冪) で、バッファは
// その境界にアラインされる (RP2350 DMA のリングラップ要件)。実 DMA チャネルは未接続
// だが、将来 producer/consumer を DMA 化できる配置。定常 ~323 rec/s の ~1.6 秒分。
// アルゴリズムは従来の spsc_ring_t と同一 (LOSSY 上書き)。両端は g_data_stream.hdl()
// の push/pop を使う (SVC を通らないライブラリ)。cmd/calib は従来の機構のまま。
inline stream::storage<record_t, 512, stream::DMA_RING> g_data_stream;
inline cmd_ring_t g_cmd_ring;
inline calib_sideband_t g_calib_xfer;

} // namespace core_ring

// core1 の I/O ループを起動する (core1_io.cpp、ベアメタル経路 = POC=0)。cyw43/Shizuku
// 初期化より前に main() から 1 回だけ呼ぶ。
void core1_io_launch();

// センサ I/O ループ本体。Shizuku の SENSOR_IO スレッド entry (core1 ピン留め、POC=1)
// 兼ベアメタル core1 entry。core1_boot.cpp が method_t として起動する。
[[noreturn]] void sensor_io_main();

// core0 側ディスパッチフック: データリングの唯一の consumer (BNO055_DRIVER の
// スレッド) が BARO/GROUND レコードを BME280 モジュールへ渡す (BME280_DRIVER.cpp)。
// 協調スケジューラなので BNO スレッドからの直接呼び出しで競合しない。
void bme280_on_baro(uint32_t press_pa, int16_t temp_cc, uint32_t t_us);
void bme280_on_ground(uint32_t press_pa, int16_t temp_cc);

} // namespace shizu

#endif // SHIZU_CORE_RING_HPP
