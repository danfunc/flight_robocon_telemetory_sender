#ifndef SHIZU_CORE_RING_HPP
#define SHIZU_CORE_RING_HPP
// ===========================================================================
//  コア間ストリーム (docs/sensor_stream_protocol.md §4)
// ===========================================================================
//  core1 (センサ I/O コア) と core0 (Shizuku) の間を流れる 4 本を定義する。全部
//  Shizuku ストリーム API (include/stream.hpp) の storage で、両端は .hdl() で直接
//  push/pop する「直接ハンドル型」— core1 には TU 制約 (core1_io.cpp 冒頭: Shizuku
//  API は sleep_us のみ許可) があり create/open/bind の SVC を撃てないため、カーネル
//  登録は通さず共有 storage を両端が直に触る。どちらが producer かはカーネルが強制
//  できないので下表の規約で守ること。
//
//    g_data_stream   core1 → core0   センサレコード       LOSSY (上書き)
//    g_cmd_stream    core0 → core1   設定コマンド         LOSSLESS + MP_PROD
//    g_calib_req     core0 → core1   較正 save/load 要求  LOSSLESS
//    g_calib_resp    core1 → core0   その結果             LOSSLESS
//
//  ・データは上書き式 (LOSSY): producer は決してブロックせず常に書き進む。溢れは
//    consumer 側が wr-rd の距離で検出し、古い方を捨ててドロップ数を数える (設計書 §4
//    「溢れたら古い方を捨てる」)。この論理は stream::handle::pop が実装している。
//  ・コマンドは取りこぼすと設定が食い違うので LOSSLESS。producer が core0 の複数
//    スレッド (BNO055 / BME280) なので MP_PROD を立て、push_mp の CAS で直列化する
//    (旧実装は「協調型単一コアだから実質直列」に依存していた = デュアルコア化で崩れる)。
//  ・較正は片方向 2 本 (req / resp) で表す。旧 req_seq/ack_seq のサイドバンド握手を
//    畳んだもので、多重要求も落とさず順に処理される。
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

// ---- 較正プロファイル (22B、レア) -------------------------------------------
// BNO055 の較正オフセットプロファイル (レジスタ 0x55..0x6A)。
constexpr uint32_t CALIB_PROFILE_LEN = 22;

// 要求 (core0 BNO055 → core1)。旧サイドバンドの op/data に相当。
struct calib_req_t {
  uint8_t op; // 1 = save (BNO → data) / 2 = load (data → BNO)
  uint8_t _rsv[3];
  uint8_t data[CALIB_PROFILE_LEN]; // load のときだけ有効
};

// 結果 (core1 → core0 BNO055)。旧サイドバンドの ok/data + ack に相当。
struct calib_resp_t {
  uint8_t op; // 実行した op をそのまま返す (要求と結果の対応付け)
  uint8_t ok; // 1 = I2C 成功
  uint8_t _rsv[2];
  uint8_t data[CALIB_PROFILE_LEN]; // save なら吸い出した値、load なら要求のエコー
};

// ---- ストリームの実体 (inline 変数、.bss でゼロ初期化) -----------------------
// データ: core1 SENSOR_IO producer → core0 BNO055 consumer。16B × 512 = 8192B (2冪)
// で、バッファはその境界にアラインされる (RP2350 DMA のリングラップ要件)。実 DMA
// チャネルは未接続だが、将来 producer/consumer を DMA 化できる配置。定常 ~323 rec/s
// の ~1.6 秒分。LOSSY (DMA_RING のみ = LOSSY は 0)。
inline stream::storage<record_t, 512, stream::DMA_RING> g_data_stream;

// コマンド: core0 の BNO055 / BME280 スレッドが producer なので MP_PROD (push_mp の
// CAS で直列化)。取りこぼすと core1 の設定が食い違うので LOSSLESS。
inline stream::storage<cmd_rec_t, 16, stream::LOSSLESS | stream::MP_PROD>
    g_cmd_stream;

// 較正: 片方向 2 本。どちらも単一 producer / 単一 consumer で、落とせないので LOSSLESS。
inline stream::storage<calib_req_t, 4, stream::LOSSLESS> g_calib_req;
inline stream::storage<calib_resp_t, 4, stream::LOSSLESS> g_calib_resp;

} // namespace core_ring

// core1 の I/O ループを起動する (core1_io.cpp、ベアメタル経路 = POC=0)。cyw43/Shizuku
// 初期化より前に main() から 1 回だけ呼ぶ。
void core1_io_launch();

// センサ I/O ループ本体。Shizuku の SENSOR_IO スレッド entry (core1 ピン留め、POC=1)
// 兼ベアメタル core1 entry。core1_boot.cpp が method_t として起動する。
[[noreturn]] void sensor_io_main();

} // namespace shizu

#endif // SHIZU_CORE_RING_HPP
