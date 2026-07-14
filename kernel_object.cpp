
#include <cstdio>
#include <cstdlib>
#include <kernel_object.hpp>
#include <map>
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

std::map<uint32_t, std::map<uint32_t, method_descriptor_t>> method_map;
std::map<uint32_t, std::map<uint32_t, uint32_t>> memory_map;

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

// カーネルオブジェクトの SVC ハンドラ。一般オブジェクトが発行した obj_api の
// svc 0x00 はカーネル (svc_cpp_handler の else 枝) からここへトランポリンされる。
//   r0..r3 = 呼び出し元が渡した引数 (r0 = obj_api::svc_num)
//   r5     = 呼び出し元オブジェクト ID,  r6 = 呼び出し元スレッド ID
// メモリ API (SET/GET_OBJ_MEMORY) は memory_map[呼び出し元 obj][slot] に格納するため、
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
    method_map[r0][r1] = {r2, r3}; // 要改修
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
  case shizu::obj_api::svc_num::CALL_METHOD: {
    ::svc<(uint32_t)shizu::kernel_object_svc_num::METHOD_CALL>(r1, r2, r3, 0);
    break;
  }
  case shizu::obj_api::svc_num::CALL_METHOD_VIA_MD: {
    ::svc<(uint32_t)shizu::kernel_object_svc_num::METHOD_CALL>(
        method_map[r1][r2].object_id, method_map[r1][r2].method_id, r3, 0);
    break;
  }
  case shizu::obj_api::svc_num::SET_OBJECT_MD: {

    method_map[r1][r2] = {r3, 0}; // 要改修
    break;
  }
  case shizu::obj_api::svc_num::GET_CURRENT_OBJ_ID: {
    svc<(uint32_t)shizu::kernel_object_svc_num::METHOD_EXIT>(r5, 0, 0, 0, 0);
    break;
  }
  case shizu::obj_api::svc_num::SET_OBJ_MEMORY: {
    memory_map[r5][r1] = r2;
    break;
  }
  case shizu::obj_api::svc_num::GET_OBJ_MEMORY: {
    svc<(uint32_t)shizu::kernel_object_svc_num::METHOD_EXIT>(memory_map[r5][r1],
                                                             0, 0, 0, 0);
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
  method_map = {};
  memory_map = {};
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
  // スレッド 0 (カーネルオブジェクト) は以降 round-robin のアイドル/スケジューラ心拍。
  // 次の runnable スレッドへ切替え、誰も居なければ (全員 sleep 中/未生成) スピンする。
  // 最初の一手で IO_CONTROLLER(thr1) へ入り、以後は各スレッドの yield/sleep と協調して
  // 回る。sleep 中 (wake_at>now) のスレッドはスキップされる。
  while (1) {
    if (!sched_pick_next(0, get_core_num(), time_us_64()))
      tight_loop_contents();
  }

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