
#include <cstdio>
#include <cstdlib>
#include <hardware/dma.h> // ストリーム接続ポンプ (DMA 設定はカーネル専有)
#include <kernel_object.hpp>
#include <obj_api.hpp>
#include <object_headers/IO_CONTROLLER.hpp>
#include <object_id.hpp>
#include <pico/stdlib.h>
#include <shizu.hpp>
#include <stream.hpp>
#include <svc.hpp>

namespace shizu {

namespace obj_api {} // namespace obj_api

void exported_method() {
  printf("exported_method called\n");
  // for_userland::exit_method(54);
}
void thread_main() {
  while (1) {
    printf("thread_switched\n");
    sleep_ms(500);
    printf("return_to_boot_thread\n");
    svc<(uint32_t)shizu::kernel_object_svc_num::SWITCH_THREAD>(0, 0, 0, 0);
  }
}

void worker_thread1() {
  while (1) {
    printf("worker_thread1\n");
    sleep_ms(500);
    printf("switch_to_worker2\n");
    svc<(uint32_t)shizu::kernel_object_svc_num::SWITCH_THREAD>(2, 0, 0, 0);
  }
}
void worker_thread2() {
  while (1) {
    printf("worker_thread2\n");
    sleep_ms(500);
    printf("switch_to_app_object\n");
    svc<(uint32_t)shizu::kernel_object_svc_num::SWITCH_THREAD>(3, 0, 0, 0);
  }
}

struct method_descriptor_t {
  uint32_t object_id;
  uint32_t method_id;
};

// ---- md / obj-memory の固定フラット表 (旧: std::map) -----------------------
// std::map は挿入で malloc し木をリバランスする。SVC ハンドラ (トランポリン先) は
// スレッドモードとはいえ、(i) 2 コア同時進入でヒープ/木を壊し得る、(ii) PendSV
// プリエンプトに対して非再入、(iii) malloc mutex の owner がコア番号でスレッドを
// 区別しない — の 3 点で SMP 化と両立しない (Fable レビュー)。object 空間が [128]
// 固定なのだから表も固定配列に焼き、SVC 経路から malloc を根絶する。
// スロット幅 16 は現行アプリの使用数 (md/メモリとも数個) の 4 倍マージン。
// ゼロ初期化 = 旧 map の operator[] デフォルト {0} と同じ既定値挙動。
constexpr uint32_t MD_SLOTS = 16;  // オブジェクトごとの method descriptor 数
constexpr uint32_t MEM_SLOTS = 16; // オブジェクトごとの共有メモリ語数
static method_descriptor_t md_table[128][MD_SLOTS];
static uint32_t obj_mem[128][MEM_SLOTS];
// 範囲外アクセスは黙って捨てず数える (STATS 等で異常を検知できるように)。
static uint32_t g_flat_oob = 0;
// METHOD_CALL のエラー (UNDECLARED_METHOD/BAD_OBJECT) 累計。core1 は printf 禁止
// なのでカウンタが唯一の計装 (core0 呼び出しは printf も出す)。
static uint32_t g_call_errors = 0;

static inline bool md_ok(uint32_t obj, uint32_t slot) {
  if (obj < 128 && slot < MD_SLOTS)
    return true;
  ++g_flat_oob;
  return false;
}
static inline bool mem_ok(uint32_t obj, uint32_t slot) {
  if (obj < 128 && slot < MEM_SLOTS)
    return true;
  ++g_flat_oob;
  return false;
}

// ---- ストリーム登録表 (制御プレーン) ---------------------------------------
// 固定配列 (SVC パスは脱 malloc 方針。std::map は使わない)。SMP では
// create/bind の 書き込みを ktab_lock で保護する予定。open の読みはポインタ
// write-once なので lock-free で足りる。desc==nullptr が空きスロット。
constexpr uint32_t NO_OBJ = 0xFFFFFFFFu;
struct stream_reg_t {
  stream::stream_desc_t *desc; // nullptr = 空き
  uint32_t owner_obj;          // create した obj
  uint32_t producer_obj;       // NO_OBJ = 未バインド
  uint32_t consumer_obj;       // NO_OBJ = 未バインド
};
static stream_reg_t stream_table[stream::MAX_STREAMS];

// ---- ストリーム接続 (CONNECT_STREAM) + カーネル DMA ポンプ ------------------
// 「src へ push されたレコードを、中間オブジェクトのコピー無しで dst へ流す」。
// 接続は src の consumer 席と dst の producer 席を占有し (SPSC 維持)、以後は
// core0 の scheduler idle ループが回す stream_pump_core0 が、src の連続可読
// チャンク × dst の連続可書チャンクの min を 1 DMA 転送として発行する。
// DMA 設定はカーネル専有 (HANDOFF §6: DMA は MPU を素通りするため、チャネルを
// オブジェクトに渡さない)。ポンプは非ブロッキング: 発行したら次の周回で完了を
// 見て index を publish する。dst へは空きにしか書かない (接続ポンプは dst を
// lossless 扱い — 溢れは src 側に滞留し、src が lossy なら src 側で最古が落ちる)。
struct stream_conn_t {
  volatile uint32_t active; // 0=空き。設定完了後に release-store で 1 (公開)
  uint32_t src_id, dst_id;
  int dma_ch;        // claim 済みチャネル (接続ごとに 1 本)
  uint32_t inflight; // DMA 転送中のレコード数 (0=idle)
  uint32_t rd_snap;  // 転送開始時の src.rd (完了時に publish する基準)
  uint32_t wr_snap;  // 転送開始時の dst.wr
  uint32_t moved;    // 累計転送レコード (計装)
  uint32_t lost;     // src lossy 追い越しで捨てたレコード (計装)
};
constexpr uint32_t MAX_STREAM_CONNS = 4;
static stream_conn_t stream_conns[MAX_STREAM_CONNS];
// 接続がバインド席に座るときの仮想オブジェクト ID (実オブジェクトと衝突しない)。
constexpr uint32_t CONN_OBJ = 0xFFFFFFFEu;

// CONNECT_STREAM の本体 (トランポリン先スレッドモードで走る。dma_claim は
// malloc しない hw_claim なので SVC パス脱 malloc 方針と両立)。
static stream::error connect_streams(uint32_t src_id, uint32_t dst_id) {
  using stream::error;
  if (src_id >= stream::MAX_STREAMS || dst_id >= stream::MAX_STREAMS ||
      src_id == dst_id || stream_table[src_id].desc == nullptr ||
      stream_table[dst_id].desc == nullptr)
    return error::BAD_ID;
  stream::stream_desc_t *s = stream_table[src_id].desc;
  stream::stream_desc_t *d = stream_table[dst_id].desc;
  if (s->rec_size != d->rec_size)
    return error::MISMATCH; // レコード型が違うものは繋げない
  // 接続は src の consumer / dst の producer の席を取る (SPSC を保つ)。
  if (stream_table[src_id].consumer_obj != NO_OBJ ||
      stream_table[dst_id].producer_obj != NO_OBJ)
    return error::ALREADY_BOUND;
  uint32_t slot = MAX_STREAM_CONNS;
  for (uint32_t i = 0; i < MAX_STREAM_CONNS; ++i)
    if (stream_conns[i].active == 0) {
      slot = i;
      break;
    }
  if (slot == MAX_STREAM_CONNS)
    return error::NO_SLOT;
  int ch = dma_claim_unused_channel(false);
  if (ch < 0)
    return error::NO_DMA;
  stream_table[src_id].consumer_obj = CONN_OBJ;
  stream_table[dst_id].producer_obj = CONN_OBJ;
  stream_conn_t &c = stream_conns[slot];
  c.src_id = src_id;
  c.dst_id = dst_id;
  c.dma_ch = ch;
  c.inflight = 0;
  c.moved = c.lost = 0;
  // 公開は最後 (ポンプは core0 idle ループ = 別コアから見得るので release で)。
  __atomic_store_n(&c.active, 1u, __ATOMIC_RELEASE);
  return error::OK;
}

// core0 の scheduler_idle_loop から毎周回呼ばれる非ブロッキングポンプ。
// 接続 0 本なら acquire ロード×4 で即戻る。single-caller (core0 idle のみ) なので
// conn の inflight/snap はロック不要。producer/consumer 側とは SPSC の wr/rd 規約
// (単調増加 + __dmb) だけで同期する — ポンプは src の唯一 consumer / dst の唯一
// producer として振る舞う。
static void stream_pump_core0() {
  for (uint32_t i = 0; i < MAX_STREAM_CONNS; ++i) {
    if (!__atomic_load_n(&stream_conns[i].active, __ATOMIC_ACQUIRE))
      continue;
    stream_conn_t &c = stream_conns[i];
    stream::stream_desc_t *s = stream_table[c.src_id].desc;
    stream::stream_desc_t *d = stream_table[c.dst_id].desc;
    if (c.inflight) {
      if (dma_channel_is_busy(c.dma_ch))
        continue; // 転送中 → 次の周回で
      __dmb();    // DMA 完了の観測 → payload 可視 → index publish の順序
      // src が lossy の場合、DMA 読み中に producer が一周していたらチャンクは
      // torn の可能性がある (handle::pop と同じ論理)。dst へ publish せず捨てて
      // resync する (dst には書いてあるが wr を進めない = 存在しない扱い)。
      if (!(s->flags & stream::LOSSLESS) &&
          (uint32_t)(s->wr - c.rd_snap) >= s->capacity) {
        uint32_t nr = s->wr - s->capacity + stream::RESYNC_MARGIN;
        c.lost += nr - c.rd_snap;
        s->rd = nr;
      } else {
        s->rd = c.rd_snap + c.inflight;
        d->wr = c.wr_snap + c.inflight; // ここで初めて consumer から見える
        c.moved += c.inflight;
      }
      c.inflight = 0;
    }
    // 次チャンク = min(src 可読, dst 空き, src 連続域, dst 連続域)。
    const uint32_t rd = s->rd, wr = s->wr;
    uint32_t n = wr - rd;
    if (n == 0)
      continue;
    const uint32_t dwr = d->wr;
    const uint32_t space = d->capacity - (dwr - d->rd);
    if (space == 0)
      continue; // dst 満杯: src 側に滞留させる (dst へは空きにしか書かない)
    if (n > space)
      n = space;
    const uint32_t src_run = s->capacity - (rd % s->capacity);
    const uint32_t dst_run = d->capacity - (dwr % d->capacity);
    if (n > src_run)
      n = src_run;
    if (n > dst_run)
      n = dst_run;
    uint8_t *src_p = (uint8_t *)s->base + (rd % s->capacity) * s->rec_size;
    uint8_t *dst_p = (uint8_t *)d->base + (dwr % d->capacity) * d->rec_size;
    const uint32_t bytes = n * s->rec_size;
    c.rd_snap = rd;
    c.wr_snap = dwr;
    c.inflight = n;
    dma_channel_config cfg = dma_channel_get_default_config(c.dma_ch);
    // 4B アラインが揃うなら 32bit 転送、そうでなければ 8bit (どちらでも正しい)。
    const bool word_ok =
        ((((uintptr_t)src_p | (uintptr_t)dst_p | bytes) & 3u) == 0);
    channel_config_set_transfer_data_size(&cfg,
                                          word_ok ? DMA_SIZE_32 : DMA_SIZE_8);
    channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, true);
    dma_channel_configure(c.dma_ch, &cfg, dst_p, src_p,
                          word_ok ? bytes / 4 : bytes, true);
  }
}

// スケジューラ中核 (budget 付き round-robin)。「今走らせてよい」スレッド = affinity
// 一致 && READY && wake_at<=now を探し、budget に応じて 2 通りで CPU を渡す:
//   budget == 0 → try_switch_thread (従来のバトンパス: 無限移譲、自分は READY へ)
//   budget > 0  → GRANT_CPU (host: 自分は WAIT_GRANT で待ち、guest の yield か期限
//                 (≤budget) で戻って続行) = 凍結ウォッチドッグ。
// 不変条件: budget>0 のスレッドは guest としてしか走らないので、yield を怠っても
// 期限で必ず回収される。バトンを受け取れるのは budget 0 組だけ。
//
// スキャン起点は per-core rotor (最後に CPU を渡した tid)。host-guest 化では自分が
// 復帰後も走り続けるため、self 起点だと「self の直後の tid」だけが毎回選ばれて他が
// 飢餓する。rotor 起点でリング全体を公平に巡回させる。
// state/wake_at は素のロード (advisory) で、確定は claim の CAS。affinity を先に
// 見るので、他コア専用スレッドの wake_at(64bit) を跨いで torn 読みしない。
static uint32_t sched_rotor[2] = {0, 0}; // per-core: 最後に CPU を渡した tid
static bool sched_pick_next(uint32_t self, uint32_t core, uint64_t now) {
  const uint32_t start = sched_rotor[core];
  for (uint32_t k = 1; k < 128; ++k) {
    uint32_t i = (start + k) & 127u;
    if (i == self)
      continue; // 自分は候補外 (GRANT_CPU(self) は no-op になるだけ)
    shizu::thread_t &t = shizu::thread_table[i];
    if (!(t.affinity & (1u << core)))
      continue;
    if (t.state != shizu::thread_t::state_t::READY)
      continue;
    if (t.wake_at > now)
      continue; // sleep 中 → スキップ
    const uint32_t budget = t.grant_budget_us;
    if (budget == 0) {
      // バトンパス (無限移譲)。成功 = 自分は READY になり、切替わって戻ってきた。
      if (shizu::try_switch_thread(i).error.is_ok()) {
        sched_rotor[core] = i;
        return true;
      }
    } else {
      // host: guest の完了 (yield/期限) までこの SVC がブロックする。戻り r0 が
      // OK なら guest は走った。NOT_READY (claim 負け) は次候補へ。
      result_t<uint32_t> r =
          ::svc<(uint32_t)shizu::kernel_object_svc_num::GRANT_CPU>(i, budget,
                                                                   0, 0);
      if ((uint32_t)r.result == (uint32_t)shizu::grant_error::OK) {
        sched_rotor[core] = i;
        return true;
      }
    }
  }
  return false;
}

// 両コア共通のスケジューラ idle ループ (このファイルは既に namespace shizu 内)。
// core0=thread0 / core1=idle スレッドが自分の tid で呼び、以後この 1 実装だけが両コアで
// 回る (core1 専用 idle は廃止)。使用率会計: sched_pick_next が実務へ渡して戻ってきた
// 経過を cpu_busy_us へ足す (guest は grant 期限/yield で、baton は相手の yield で戻る)。
// スピンした分は足さない → 使用率 = Δcpu_busy_us/Δwall。
[[noreturn]] void scheduler_idle_loop(uint32_t self) {
  const uint32_t core = get_core_num();
  while (1) {
    // ストリーム接続の DMA ポンプ (core0 のみ = single-caller でロック不要)。
    // 非ブロッキング: 発行済み転送の完了確認と次チャンクの発行だけして戻る。
    if (core == 0)
      stream_pump_core0();
    uint64_t t0 = time_us_64();
    if (sched_pick_next(self, core, t0))
      cpu_busy_us[core] += time_us_64() - t0; // スケジューラが実務へ渡した時間
    else
      tight_loop_contents(); // 誰も runnable でない = idle (busy に足さない)
  }
}

// カーネルオブジェクトの SVC ハンドラ。一般オブジェクトが発行した obj_api の
// svc 0x00 はカーネル (svc_cpp_handler の else 枝) からここへトランポリンされる。
//   r0..r3 = 呼び出し元が渡した引数 (r0 = obj_api::svc_num)
//   r5     = 呼び出し元オブジェクト ID,  r6 = 呼び出し元スレッド ID
// メモリ API (SET/GET_OBJ_MEMORY) は obj_mem[呼び出し元 obj][slot] に格納するため、
// オブジェクトごとに名前空間が分かれる (他オブジェクトのメモリは直接読めない)。
// よってオブジェクト間のデータ受け渡しは「メソッド呼び出し + ポインタ渡し」で行う。
uint32_t kernel_obj_svc_handler(uint32_t r0, uint32_t r1, uint32_t r2,
                                uint32_t r3, uint32_t r4, uint32_t r5,
                                uint32_t r6, uint32_t r12) {
  // printf("KERNEL_OBJ_SVC_HANDLER_ENTRY: r0=%lx, r1=%lx, r2=%lx, "
  //       "r3=%lx,r4=%lx,r5=%lx,r6=%lx\n",
  //       r0, r1, r2, r3, r4, r5, r6);
  // 以下は臨時のset_md(4)
  if (r4 == 100) {
    svc<(uint32_t)shizu::kernel_object_svc_num::METHOD_EXIT>(0, 0, 0, 0, 0);
  }
  if (r4 == 255) {
    if (md_ok(r0, r1))
      md_table[r0][r1] = {r2, r3}; // 要改修
    svc<(uint32_t)shizu::kernel_object_svc_num::METHOD_EXIT>(0, 0, 0, 0, 0);
  }

  shizu::obj_api::svc_num svc_num = (shizu::obj_api::svc_num)r0;
  switch (svc_num) {
  case shizu::obj_api::svc_num::CREATE_OBJECT: {
    shizu::FOR_KERNEL_OBJECT::create_object(r1, r2, (shizu::method_t)r3);
    break;
  }
  case shizu::obj_api::svc_num::CREATE_THREAD: {
    shizu::FOR_KERNEL_OBJECT::create_thread(r1, r2, (shizu::method_t)r3);
    break;
  }
  case shizu::obj_api::svc_num::CREATE_OBJECT_ONLY: {
    // r1=obj_id。オブジェクトのみ生成 (スレッド無し)。起動は ASYNC_CALL で。
    shizu::FOR_KERNEL_OBJECT::create_object(r1);
    break;
  }
  case shizu::obj_api::svc_num::ASYNC_CALL: {
    // r1=obj_id, r2=entry, r3=arg。空き tid を自動確保して spawn。戻り r1=tid。
    uint32_t tid =
        shizu::FOR_KERNEL_OBJECT::async_call(r1, (shizu::method_t)r2, r3);
    svc<(uint32_t)shizu::kernel_object_svc_num::METHOD_EXIT>(tid, 0, 0, 0, 0);
    break;
  }
  case shizu::obj_api::svc_num::EXPORT_METHOD: {
    // printf("EXPORT_METHOD\n");
    // r1: method_num, r2: entry
    shizu::FOR_KERNEL_OBJECT::export_method(r5, r1, (shizu::method_t)r2);
    break;
  }
  case shizu::obj_api::svc_num::EXIT_METHOD: {
    svc<(uint32_t)shizu::kernel_object_svc_num::METHOD_EXIT>(r1, 1, 0, 0, 0);
    break;
  }
  case shizu::obj_api::svc_num::YIELD: {
    // grant 中 (自分は誰かの grantee) の yield = grantor への早期復帰。
    // SWITCH_THREAD はインターセプトされて対象を無視し grantor へ pop するので、
    // 引数はダミーでよい。round-robin へは入らない (grantor が優先復帰先)。
    if (shizu::grant_active_this_core()) {
      ::svc<(uint32_t)shizu::kernel_object_svc_num::SWITCH_THREAD>(0, 0, 0, 0);
      break;
    }
    // sleep-aware round-robin: 次の runnable スレッドへ切替える。誰も居なければ何も
    // せず自スレッド続行 (= plain yield)。sleep 中 (wake_at>now) はスキップされる。
    const uint32_t self =
        ::svc<(uint32_t)shizu::kernel_object_svc_num::GET_CURRENT_THREAD_ID>(
            0, 0, 0, 0)
            .value;
    sched_pick_next(self, get_core_num(), time_us_64());
    break;
  }
  case shizu::obj_api::svc_num::SLEEP_US: {
    // 自スレッドの wake_at を締切に設定し、締切まで他の runnable スレッドへ譲り続ける。
    // 誰も runnable でなければ tight spin。締切到来 (= scheduler が自スレッドを再び拾える)
    // で続行。wake_at>now の間はどの scheduler も自スレッドを claim しない。
    const uint32_t self =
        ::svc<(uint32_t)shizu::kernel_object_svc_num::GET_CURRENT_THREAD_ID>(
            0, 0, 0, 0)
            .value;
    const uint64_t deadline = time_us_64() + (uint64_t)r1;
    shizu::thread_table[self].wake_at = deadline;
    while ((int64_t)(deadline - time_us_64()) > 0) {
      // grant 中の sleep = grantor への早期復帰 (wake_at は設定済みなので、自分は
      // READY+wake_at で round-robin から外れ、締切後に再 claim されて続きを走る)。
      if (shizu::grant_active_this_core()) {
        ::svc<(uint32_t)shizu::kernel_object_svc_num::SWITCH_THREAD>(0, 0, 0,
                                                                     0);
        continue;
      }
      if (!sched_pick_next(self, get_core_num(), time_us_64()))
        tight_loop_contents(); // 他に runnable が居ない → 締切までスピン
    }
    shizu::thread_table[self].wake_at = 0; // 起床: 以後 plain yield でスキップされない
    break;
  }
  case shizu::obj_api::svc_num::RUN_FOR: {
    // r1=対象 tid, r2=最大実行時間[µs]。GRANT_CPU を発行し、この (grantor の)
    // スレッドは復帰までここで待つ。復帰後、error(r0) と reason(r1) を 16bit ずつ
    // にパックして METHOD_EXIT で返す (一般オブジェクトの戻りは r1 一語のため)。
    result_t<uint32_t> grant_result =
        ::svc<(uint32_t)shizu::kernel_object_svc_num::GRANT_CPU>(r1, r2, 0, 0);
    svc<(uint32_t)shizu::kernel_object_svc_num::METHOD_EXIT>(
        (((uint32_t)grant_result.result & 0xFFFFu) << 16) |
            (grant_result.value & 0xFFFFu),
        0, 0, 0, 0);
    break;
  }
  case shizu::obj_api::svc_num::SET_THREAD_BUDGET: {
    // r1=tid, r2=µs (0=無制限バトン)。scheduler が host するときの時限を変更する。
    // 書き込みは u32 一語 (torn しない)。読む側 (sched_pick_next) は advisory。
    if (r1 < 128)
      shizu::thread_table[r1].grant_budget_us = r2;
    break;
  }
  case shizu::obj_api::svc_num::SET_AFFINITY: {
    // r1=tid, r2=マスク (bit0=core0/bit1=core1)。u32 一語の advisory 書き込みで、
    // 確定は claim (try_claim) の CAS 側 — 反映は対象が次に READY になって
    // scheduler が claim するときから (走行中スライスは回収しない)。空マスクは
    // 誰も claim できない迷子スレッドを作るので弾く。範囲外/不正ビットも無視
    // (SET_THREAD_BUDGET と同じ「黙って無視」規律)。
    if (r1 < 128 && (r2 & shizu::AFFINITY_ALL) != 0 &&
        (r2 & ~shizu::AFFINITY_ALL) == 0)
      shizu::thread_table[r1].affinity = r2;
    break;
  }
  case shizu::obj_api::svc_num::CALL_METHOD: {
    // METHOD_CALL 成功時はメソッドが METHOD_EXIT した後にここへ戻る (r0=0,
    // r1=メソッド戻り値)。失敗 (未 export = UNDECLARED_METHOD / 未生成 =
    // BAD_OBJECT) は pc 張り替え無しで即戻る (r0=call_error)。どちらも
    // METHOD_EXIT の arg2 (エラー) / arg0 (値) に載せ替えて呼び出し元へ透過する
    // — 呼び出し元は r0 の call_error を見て yield → 再試行できる (affinity で
    // コアを移すと初回実行順による export 保証が無いため panic ではなくエラー)。
    result_t<uint32_t> r =
        ::svc<(uint32_t)shizu::kernel_object_svc_num::METHOD_CALL>(r1, r2, r3,
                                                                   0);
    if ((uint32_t)r.result != 0) {
      ++g_call_errors;
      if (get_core_num() == 0) // core1 は printf 禁止 (計装はカウンタのみ)
        printf("[CALL] err=%lu caller_obj=%lu callee_obj=%lu method=%lu\n",
               (unsigned long)r.result, (unsigned long)r5, (unsigned long)r1,
               (unsigned long)r2);
    }
    svc<(uint32_t)shizu::kernel_object_svc_num::METHOD_EXIT>(
        r.value, 0, (uint32_t)r.result, 0, 0);
    break;
  }
  case shizu::obj_api::svc_num::CALL_METHOD_VIA_MD: {
    if (!md_ok(r1, r2)) {
      svc<(uint32_t)shizu::kernel_object_svc_num::METHOD_EXIT>(
          0, 0, (uint32_t)shizu::call_error::BAD_OBJECT, 0, 0);
      break;
    }
    result_t<uint32_t> r =
        ::svc<(uint32_t)shizu::kernel_object_svc_num::METHOD_CALL>(
            md_table[r1][r2].object_id, md_table[r1][r2].method_id, r3, 0);
    if ((uint32_t)r.result != 0)
      ++g_call_errors;
    svc<(uint32_t)shizu::kernel_object_svc_num::METHOD_EXIT>(
        r.value, 0, (uint32_t)r.result, 0, 0);
    break;
  }
  case shizu::obj_api::svc_num::SET_OBJECT_MD: {
    if (md_ok(r1, r2))
      md_table[r1][r2] = {r3, 0}; // 要改修
    break;
  }
  case shizu::obj_api::svc_num::GET_CURRENT_OBJ_ID: {
    svc<(uint32_t)shizu::kernel_object_svc_num::METHOD_EXIT>(r5, 0, 0, 0, 0);
    break;
  }
  case shizu::obj_api::svc_num::SET_OBJ_MEMORY: {
    if (mem_ok(r5, r1))
      obj_mem[r5][r1] = r2;
    break;
  }
  case shizu::obj_api::svc_num::GET_OBJ_MEMORY: {
    svc<(uint32_t)shizu::kernel_object_svc_num::METHOD_EXIT>(
        mem_ok(r5, r1) ? obj_mem[r5][r1] : 0, 0, 0, 0, 0);
    break;
  }
  // ---- ストリーム制御プレーン (include/stream.hpp)
  // --------------------------- 戻り規約: r1(value) に結果を載せる。create/bind
  // は stream::error、open は stream_desc_t* (未登録 0)。データプレーン
  // (push/pop) は SVC を通らない。
  case shizu::obj_api::svc_num::CREATE_STREAM: {
    // r1=id, r2=stream_desc_t*, r3=flags, r5=owner obj。
    uint32_t id = r1;
    stream::stream_desc_t *d = reinterpret_cast<stream::stream_desc_t *>(r2);
    stream::error err = stream::error::OK;
    if (id >= stream::MAX_STREAMS || d == nullptr) {
      err = stream::error::BAD_ID;
    } else if (stream_table[id].desc != nullptr && stream_table[id].desc != d) {
      err = stream::error::ALREADY_BOUND; // 別ディスクリプタが既に占有
    } else {
      stream_table[id].desc = d;
      stream_table[id].owner_obj = r5;
      stream_table[id].producer_obj = NO_OBJ;
      stream_table[id].consumer_obj = NO_OBJ;
    }
    svc<(uint32_t)shizu::kernel_object_svc_num::METHOD_EXIT>((uint32_t)err, 0,
                                                             0, 0, 0);
    break;
  }
  case shizu::obj_api::svc_num::OPEN_STREAM: {
    // r1=id → r1(戻)=stream_desc_t* (未登録は 0)。
    uint32_t id = r1;
    stream::stream_desc_t *d =
        (id < stream::MAX_STREAMS) ? stream_table[id].desc : nullptr;
    svc<(uint32_t)shizu::kernel_object_svc_num::METHOD_EXIT>(
        (uint32_t)(uintptr_t)d, 0, 0, 0, 0);
    break;
  }
  case shizu::obj_api::svc_num::BIND_STREAM: {
    // r1=id, r2=role, r5=caller obj。単一 producer/consumer 強制 (MP_PROD
    // は例外)。
    uint32_t id = r1;
    stream::role rl = (stream::role)r2;
    uint32_t caller = r5;
    stream::error err = stream::error::OK;
    if (id >= stream::MAX_STREAMS || stream_table[id].desc == nullptr) {
      err = stream::error::BAD_ID;
    } else if (rl == stream::role::PRODUCER) {
      bool mp = (stream_table[id].desc->flags & stream::MP_PROD) != 0;
      if (stream_table[id].producer_obj != NO_OBJ && !mp &&
          stream_table[id].producer_obj != caller) {
        err = stream::error::ALREADY_BOUND;
      } else {
        stream_table[id].producer_obj = caller;
      }
    } else { // CONSUMER
      if (stream_table[id].consumer_obj != NO_OBJ &&
          stream_table[id].consumer_obj != caller) {
        err = stream::error::ALREADY_BOUND;
      } else {
        stream_table[id].consumer_obj = caller;
      }
    }
    svc<(uint32_t)shizu::kernel_object_svc_num::METHOD_EXIT>((uint32_t)err, 0,
                                                             0, 0, 0);
    break;
  }
  case shizu::obj_api::svc_num::CONNECT_STREAM: {
    // r1=src id, r2=dst id。src の consumer 端と dst の producer 端をカーネルの
    // DMA ポンプで直結する。以後 src へ push されたレコードはオブジェクトの
    // コピー無しで dst へ流れる。戻り r1=stream::error (他の stream SVC と同規約)。
    stream::error err = connect_streams(r1, r2);
    svc<(uint32_t)shizu::kernel_object_svc_num::METHOD_EXIT>((uint32_t)err, 0,
                                                             0, 0, 0);
    break;
  }
  case shizu::obj_api::svc_num::STREAM_WAIT:
  case shizu::obj_api::svc_num::STREAM_NOTIFY: {
    // ブロッキング (空で SUSPEND → producer が push 後に wake) は claim/wake
    // 機構が 要るので後実装。現状はポーリングフォールバック: 即戻る (consumer
    // は while(pop()) + yield で回す)。
    svc<(uint32_t)shizu::kernel_object_svc_num::METHOD_EXIT>(0, 0, 0, 0, 0);
    break;
  }
  default: {
    printf("undefined obj_svc_num\n"
           "called_obj_svc_num: %lx\n",
           r0);
    break;
  }
  }

  svc<(uint32_t)shizu::kernel_object_svc_num::METHOD_EXIT>(0, 0, 0, 0, 0);
  return 0;
}

template <auto T>
  requires std::invocable<decltype(T), uint32_t, uint32_t, uint32_t, uint32_t,
                          uint32_t, uint32_t, uint32_t, uint32_t>
__attribute__((naked, aligned(4))) void wrapper() {
  asm volatile("push {r4-r7,r12, lr}\n"
               "ldr  r4, 1f\n"
               "blx  r4\n"
               "pop  {r4-r7,r12, pc}\n"
               ".align 2\n"
               "1: .word %c0\n"
               :
               : "i"(reinterpret_cast<uint32_t>(T))
               :);
}

void kernel_object_main() {
  svc<(uint32_t)shizu::kernel_object_svc_num::SET_SVC_HANDLER>(
      (uint32_t)(&shizu::wrapper<kernel_obj_svc_handler>), 0, 0, 0, 0);
  // オブジェクト起動 = create_object(id) + async_call(id, main)。thread 番号は
  // async_call が自動確保する (手番廃止)。
  FOR_KERNEL_OBJECT::create_object((uint32_t)object_ids::IO_CONTROLLER);
  FOR_KERNEL_OBJECT::async_call((uint32_t)object_ids::IO_CONTROLLER,
                                (method_t)IO_CONTROLLER::main);
#if SHIZU_CORE1_KERNEL_POC
  // デュアルコア: core1 を Shizuku カーネルとして起動し、センサ I/O を SENSOR_IO
  // (affinity=core1) スレッドとして走らせる。thread0 がまだ yield/switch する前に
  // 生成するので、core0 が SENSOR_IO(affinity=core1) を拾う窓は無い。
  shizu::core1_kernel_launch();
#endif
#if SHIZU_STREAM_SELFTEST
  // ストリーム API の end-to-end 自己テスト (producer/consumer 対を core0
  // で協調実行)。
  shizu::stream_selftest_launch();
#endif
#if SHIZU_GRANT_SELFTEST
  // 時限実行権移譲 (run_for/GRANT_CPU) の end-to-end 自己テスト。
  shizu::grant_selftest_launch();
#endif
#if SHIZU_SMP_STRESS
  // カーネル SVC 経路の 2 コア同時進入ストレス (SLEEP/SWITCH ピンポン + 跨コア移動)。
  shizu::smp_stress_launch();
#endif
  // スレッド 0 (カーネルオブジェクト) は以降 round-robin のアイドル/スケジューラ心拍。
  // 次の runnable スレッドへ切替え、誰も居なければ (全員 sleep 中/未生成) スピンする。
  // 最初の一手で IO_CONTROLLER(thr1) へ入り、以後は各スレッドの yield/sleep と協調して
  // 回る。sleep 中 (wake_at>now) のスレッドはスキップされる。
  // スレッド 0 (カーネルオブジェクト) は以降このコアの共通スケジューラ idle ループへ。
  // core1 の idle スレッドも同じ scheduler_idle_loop を回すので実装は 1 本 (core1 専用
  // idle 廃止)。使用率はこのループの busy 会計から両コア一様に出る。
  shizu::scheduler_idle_loop(0);

  while (1) {
    const uint32_t current_id = 0;
    uint32_t next_id = (current_id + 1) % 128;

    while (next_id != current_id) {
      ::result_t<uint32_t> state_res =
          ::svc<(uint32_t)shizu::kernel_object_svc_num::GET_THREAD_STATE>(
              next_id, 0, 0, 0);
      if (state_res.value == (uint32_t)shizu::thread_t::state_t::READY) {
        break;
      }
      next_id = (next_id + 1) % 128;
    }
    if (next_id != current_id) {
      ::svc<(uint32_t)shizu::kernel_object_svc_num::SWITCH_THREAD>(next_id, 0,
                                                                   0, 0);
    }
  }

  while (1) {
    printf("kernel_object_main\n");
    printf("svc_calling\n");
    // result_t<uint32_t> result =
    // svc<(uint32_t)shizu::kernel_object_svc_num::METHOD_CALL>(0, 0, 0, 0, 0);

    // printf("svc_called\nreturn: %lx\n", result.result);

    sleep_ms(500);
  }
}
} // namespace shizu