#ifndef SHIZU_STREAM_HPP
#define SHIZU_STREAM_HPP
// ===========================================================================
//  Shizuku ストリーム API — オブジェクト間 / コア間の一級ストリーム抽象
// ===========================================================================
//  今まで「協調型単一コアで yield しない = 暗黙のクリティカルセクション」で担保
//  していたオブジェクト間データ受け渡し (core_ring.hpp の手書き SPSC リング) を、
//  正式なユーザ API として作り込む。デュアルコア化後もこの API がコア間通信の
//  唯一の正道になる。
//
//  【設計の柱】
//  1. 制御プレーン / データプレーン分離:
//       - 制御プレーン (create/open/bind/wait/notify) だけ SVC でカーネルに委ねる
//         (登録・discovery・単一 producer/consumer 強制・待ち/起こし)。
//       - データプレーン (push/pop) は SVC を通らない**ライブラリ**。MMU が無い
//         単一アドレス空間では push/pop を SVC 化しても保護は 1bit も増えず、
//         レコードあたり 100+ サイクルの体裁代を払うだけなので通さない。
//         既存 core1 ベアメタル producer はコード無改造で外部 producer になれる。
//  2. backing は実装詳細、interface に漏らさない:
//       - 既定は**共有メモリ** (CPU SPSC リング。core_ring と同一アルゴリズム)。
//       - DMA_RING を立てたときだけ「DMA チャネルが直接指せる連続バッファ」
//         (RP2350 の DMA リングラップ制約 = バイトサイズ 2冪 + 境界アライン) を課す。
//         DMA 非対応なら共有メモリで足りる、という前提をそのまま型で表す。
//  3. MPU 前提: ディスクリプタ (制御) と データ領域 (base) を分離配置。将来 MPU で
//       データ領域だけを region 保護できる (ディスクリプタはカーネルが持つ)。
//
//  【使い方 (owner = producer 側)】
//    static shizu::stream::storage<record_t, 682> g_sensors;      // 共有メモリ既定
//    // object main 冒頭:
//    shizu::stream::create(STREAM_SENSORS, &g_sensors.desc);
//    shizu::stream::bind(STREAM_SENSORS, shizu::stream::role::PRODUCER);
//    auto tx = g_sensors.hdl();
//    // ホットループ: tx.push(rec);   // ライブラリ、SVC 無し
//
//  【使い方 (consumer 側、別オブジェクト)】
//    auto rx = shizu::stream::open<record_t>(STREAM_SENSORS);
//    shizu::stream::bind(STREAM_SENSORS, shizu::stream::role::CONSUMER);
//    record_t r; uint32_t lost = 0;
//    while (rx.pop(&r, &lost)) { ... }         // ポーリング
//    // または shizu::stream::wait(STREAM_SENSORS); でブロック (実装後)
// ===========================================================================
#include <cstddef>
#include <cstdint>
#include <obj_api.hpp>
#include <svc.hpp> // error_t / svc_result_t

// __dmb: 実機は hardware/sync.h。ホスト構文検査用にコンパイラバリアで代替する。
#if __has_include(<hardware/sync.h>)
#include <hardware/sync.h>
#else
static inline void __dmb() { __asm__ __volatile__("" ::: "memory"); }
#endif

namespace shizu {
namespace stream {

// ---- flags (ビット和) -------------------------------------------------------
enum flags : uint32_t {
  // 消失方針 (排他)。既定 LOSSY = 満杯で最古を上書き (producer 非ブロック)。
  LOSSY = 0,
  LOSSLESS = 1u << 0, // 満杯で push 失敗を返す (呼び出し側が yield して再試行)
  // producer 多重度。既定 SPSC (単一 producer)。
  MP_PROD = 1u << 1, // 複数 producer。push_mp が prod_lock (CAS) を取る
  // backing。既定は共有メモリ (CPU SPSC)。
  DMA_RING = 1u << 2, // データ領域を DMA チャネルが直接指す (2冪バイト + アライン)
  EXT_PROD = 1u << 3, // wr を CPU 以外 (DMA 等) が進める (push を使わない)
  EXT_CONS = 1u << 4, // rd を CPU 以外 (DMA 等) が進める (pop を使わない)
};

// ---- stream 系 SVC の型付きエラー (error_t 用, 0 == OK) ----------------------
enum struct error : uint32_t {
  OK = 0,
  NO_SLOT,       // stream_table 満杯
  BAD_ID,        // 未登録 / 範囲外
  ALREADY_BOUND, // その role は既にバインド済み (SPSC で2人目)
  NOT_BOUND,     // bind 前に wait/notify した
  WRONG_ROLE,    // producer 用操作を consumer が呼んだ等
  MISMATCH,      // connect: rec_size が一致しない 2 本は繋げない
  NO_DMA,        // connect: 空き DMA チャネルが無い
};

enum struct role : uint32_t { PRODUCER = 0, CONSUMER = 1 };

constexpr uint32_t NO_WAITER = 0xFFFFFFFFu;

// ストリーム ID 空間の上限 (カーネル stream_table のスロット数と一致させること)。
constexpr uint32_t MAX_STREAMS = 32;

// consumer が overrun 検出後に巻き戻す安全マージン (core_ring と同値)。
constexpr uint32_t RESYNC_MARGIN = 8;

// ---- ディスクリプタ (制御プレーン。カーネルはこのポインタだけを保持) ---------
//  データ領域 (base) とは分離配置する (MPU で base 側だけ保護できるように)。
//  POD のまま (std::atomic を埋め込まない) にして、コア間共有・DMA・将来の MPU
//  region 配置を素直にする。原子操作は __atomic ビルトイン (M33 で LDREX/STREX)
//  を prod_lock に対してだけ使う。wr/rd は core_ring 同様 volatile + __dmb で扱う。
struct stream_desc_t {
  void *base;              // データ領域先頭 = REC buf[capacity]。DMA の指す先
  uint32_t rec_size;       // 1 レコードのバイト数 (= sizeof(REC))
  uint32_t capacity;       // レコード数 (DMA_RING 時は buf バイト長が 2冪)
  uint32_t flags;          // enum flags のビット和
  volatile uint32_t wr;    // publish 済みレコード数 (producer / DMA が進める)
  volatile uint32_t rd;    // 消費済みレコード数 (consumer / DMA が進める)
  volatile uint32_t waiter;// NO_WAITER or 待機中 consumer の thread_id
  uint32_t prod_lock;      // MP_PROD 時のみ: producer 側 CAS スピンロック
};

// ---- ハンドル (データプレーン。push/pop はライブラリ、SVC を通らない) ---------
// データプレーン (push/pop) は always_inline で呼び出し元へ畳み込む。これにより
// 関数自身のセクションを持たず「呼び出し元の配置を継承」する — 例えば core1 の
// __not_in_flash_func な producer から呼べば push も SRAM に入り、XIP 競合を避けられる
// (別関数のまま flash に置かれると core0 の BLE と XIP バスを取り合う)。
#define SHIZU_STREAM_AINLINE [[gnu::always_inline]] inline

template <typename REC> class handle {
  stream_desc_t *d_ = nullptr;

  SHIZU_STREAM_AINLINE REC &at(uint32_t i) {
    return static_cast<REC *>(d_->base)[i % d_->capacity];
  }
  // 単一 producer の実書き込み (SPSC / MP 共通の中核)。
  SHIZU_STREAM_AINLINE bool push_core(const REC &r) {
    const uint32_t N = d_->capacity;
    uint32_t w = d_->wr;
    if (d_->flags & LOSSLESS) {
      if ((uint32_t)(w - d_->rd) >= N)
        return false; // 満杯: lossless は失敗を返す
    }
    at(w) = r;   // lossy は満杯でも最古を上書き (consumer が overrun 検出)
    __dmb();     // payload の書き込みを wr 公開より先に完了させる
    d_->wr = w + 1;
    return true;
  }

public:
  handle() = default;
  explicit handle(stream_desc_t *d) : d_(d) {}
  bool valid() const { return d_ != nullptr; }
  stream_desc_t *desc() const { return d_; }
  bool empty() const { return d_->wr == d_->rd; }

  // producer (単一, SPSC)。lossy=常に true / lossless=満杯で false。
  SHIZU_STREAM_AINLINE bool push(const REC &r) { return push_core(r); }

  // producer: 現在 push できる空きレコード数 (LOSSLESS の残容量)。単一 producer が
  // マルチレコードのメッセージを「全部入るときだけ push」する事前チェックに使う。
  // consumer は空きを増やす方向にしか動かないので、単一 producer 文脈では安全。
  SHIZU_STREAM_AINLINE uint32_t writable_slots() const {
    return d_->capacity - (d_->wr - d_->rd);
  }

  // consumer: 先頭レコードを覗く (rd は進めない)。空なら false。送信成功後に drop()
  // で消費確定する「成功してから消費」方式用 (credit 切れで送れなかったフレームを
  // 失わない)。単一 consumer 文脈で使うこと。
  SHIZU_STREAM_AINLINE bool peek(REC *out) {
    uint32_t w = d_->wr;
    __dmb(); // wr 観測 → buf 読みの順序を保証
    if (d_->rd == w)
      return false; // 空
    *out = at(d_->rd);
    return true;
  }

  // consumer: peek した 1 件を消費確定 (rd を 1 進める)。peek が true を返した後のみ。
  SHIZU_STREAM_AINLINE void drop() { d_->rd = d_->rd + 1; }

  // producer (複数, MP_PROD)。prod_lock を CAS で取ってから push。
  SHIZU_STREAM_AINLINE bool push_mp(const REC &r) {
    uint32_t expected = 0;
    while (!__atomic_compare_exchange_n(&d_->prod_lock, &expected, 1u, true,
                                        __ATOMIC_ACQUIRE, __ATOMIC_RELAXED))
      expected = 0; // 競り負け → 取り直し
    bool ok = push_core(r);
    __atomic_store_n(&d_->prod_lock, 0u, __ATOMIC_RELEASE);
    return ok;
  }

  // consumer (単一)。戻り false = 空 (lossy は torn 検出でのリトライ待ちも含む)。
  //
  // ラップ/torn 検出は **lossy 専用の論理**であることに注意。lossy は producer が
  // 満杯を無視して上書きするので consumer が追い越され得る (w-r>=N を検出して古い方を
  // 捨て、*lost に加算)。lossless は producer が満杯 (w-rd==N) で止まるので追い越しは
  // 起きず、w-r==N は「満杯だが全件有効」= drop してはいけない。両者を flags で分ける。
  // (LOSSY はビット 0 = 無印なので判定は必ず & LOSSLESS 側で行う。)
  SHIZU_STREAM_AINLINE bool pop(REC *out, uint32_t *lost = nullptr) {
    const uint32_t N = d_->capacity;
    uint32_t w = d_->wr;
    __dmb(); // wr 観測 → buf 読みの順序を保証
    uint32_t r = d_->rd;
    if (r == w)
      return false; // 空

    if (d_->flags & LOSSLESS) {
      // 追い越し無し: 満杯でも全件有効。素朴な FIFO 取り出しで足りる。
      REC tmp = at(r);
      d_->rd = r + 1;
      *out = tmp;
      return true;
    }

    // ---- lossy: 追い越し検出 + torn 検出 (core_ring::pop と同一論理) ----
    if (w - r >= N) { // 追い越された: 古い方を捨てて前へ
      uint32_t nr = w - N + RESYNC_MARGIN;
      if (lost)
        *lost += nr - r;
      r = nr;
    }
    REC tmp = at(r);
    __dmb(); // buf 読み終わり → wr 再検証の順序を保証
    if (d_->wr - r >= N) { // コピー中に producer が一周 (torn) → 捨てる
      uint32_t nr = d_->wr - N + RESYNC_MARGIN;
      if (lost)
        *lost += nr - r;
      d_->rd = nr;
      return false;
    }
    d_->rd = r + 1;
    *out = tmp;
    return true;
  }
};

// ---- ストレージ (owner が置く。バッファ + ディスクリプタを所有) ----------------
//  DMA_RING のときだけ RP2350 の DMA リングラップ制約 (バッファのバイト長が 2冪、
//  その境界にアライン) を静的に課す。それ以外 (共有メモリ) は素のアラインでよい
//  ── 「DMA 非対応なら共有メモリで足りる」をそのまま型で表現している。
template <typename REC, uint32_t CAPACITY, uint32_t FLAGS = LOSSY> struct storage {
  static constexpr uint32_t TOTAL_BYTES = sizeof(REC) * CAPACITY;
  static constexpr bool is_pow2(uint32_t x) { return x && !(x & (x - 1)); }
  static_assert(CAPACITY >= 2, "stream capacity must be >= 2");
  static_assert(!(FLAGS & DMA_RING) || is_pow2(TOTAL_BYTES),
                "DMA_RING: buffer byte size must be power-of-two (pad REC to a "
                "power-of-two size). RP2350 DMA ring wrap requires it.");

  alignas(FLAGS & DMA_RING ? TOTAL_BYTES : alignof(REC)) REC buf[CAPACITY];
  stream_desc_t desc;

  storage() {
    desc.base = buf;
    desc.rec_size = sizeof(REC);
    desc.capacity = CAPACITY;
    desc.flags = FLAGS;
    desc.wr = 0;
    desc.rd = 0;
    desc.waiter = NO_WAITER;
    desc.prod_lock = 0;
  }

  handle<REC> hdl() { return handle<REC>(&desc); }
  REC *data() { return buf; }                       // DMA チャネルに渡す先頭
  static constexpr uint32_t data_bytes() { return TOTAL_BYTES; }
};

// ===========================================================================
//  制御プレーン (obj_api SVC 経由)。カーネルハンドラは別ステップで実装する。
//  戻りエラーは r1(value) に stream::error を載せる規約とする (実装時に合わせる)。
// ===========================================================================
// owner がディスクリプタをカーネルへ登録する。
inline error_t<error> create(uint32_t id, stream_desc_t *d) {
  auto r = obj_api::svci<obj_api::svc_num::CREATE_STREAM>(
      id, reinterpret_cast<uintptr_t>(d), d->flags);
  return error_t<error>{static_cast<error>(r.value)};
}

// consumer が id からディスクリプタを引く (単一アドレス空間なので extern 直参照も
// 可だが、疎結合な discovery 用)。戻りが nullptr = 未登録。
template <typename REC> inline handle<REC> open(uint32_t id) {
  auto r = obj_api::svci<obj_api::svc_num::OPEN_STREAM>(id);
  return handle<REC>(reinterpret_cast<stream_desc_t *>(r.value));
}

// 役割をバインドする。カーネルが単一 producer / 単一 consumer を強制する
// (MP_PROD なら producer 多重を許す)。
inline error_t<error> bind(uint32_t id, role rl) {
  auto r = obj_api::svci<obj_api::svc_num::BIND_STREAM>(
      id, static_cast<uintptr_t>(rl));
  return error_t<error>{static_cast<error>(r.value)};
}

// src の consumer 端と dst の producer 端をカーネルの DMA ポンプで直結する。
// 以後 src へ push されたレコードは「オブジェクトによるコピー無し」で dst へ流れる
// (従来: 中間オブジェクトが pop → push でコピーしていた段が丸ごと消える)。
// 前提: 両ストリーム create 済み / rec_size 一致 / src の consumer 席と dst の
// producer 席が未バインド (接続がその席を占有し、以後オブジェクトは bind できない)。
// DMA チャネルは接続ごとにカーネルが 1 本 claim する (オブジェクトへは渡さない)。
// dst へは空きにしか書かない (溢れは src 側に滞留 → src が lossy なら最古が落ちる)。
inline error_t<error> connect(uint32_t src_id, uint32_t dst_id) {
  auto r = obj_api::svci<obj_api::svc_num::CONNECT_STREAM>(src_id, dst_id);
  return error_t<error>{static_cast<error>(r.value)};
}

// consumer: 空なら SUSPEND (カーネルが SVC 内で空を再検査してから寝る =
// lost-wakeup 防止)。非空なら即戻る。実装は blocking-pop ステップで。
inline void wait(uint32_t id) {
  obj_api::svci<obj_api::svc_num::STREAM_WAIT>(id);
}

// producer: push 後、waiter が居るときだけ呼ぶ稀パス (desc->waiter を見てから)。
inline void notify(uint32_t id) {
  obj_api::svci<obj_api::svc_num::STREAM_NOTIFY>(id);
}

} // namespace stream
} // namespace shizu

#endif // SHIZU_STREAM_HPP
