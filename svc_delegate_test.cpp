// ===========================================================================
//  svc_delegate_test — svc 委譲 (指定オブジェクトが syscall を捌く) の 2 段ネスト実証
// ===========================================================================
//  構図:
//    T (SVC_DELEGATE_T, 一般オブジェクト)
//      └─ svci<SVC_A>  → カーネルが委譲表を引く → **A のハンドラ**へトランポリン
//           A は「オブジェクト A として」走る (current object が A に張り替わる)
//             └─ svci<SVC_B> → もう一度トランポリン → **B のハンドラ** (= 2 段目)
//                  B は「オブジェクト B として」走り、値を +1 して返す
//             └─ A は B の戻りに +10 して返す
//    → T が受け取る値が (x+1)+10 なら 2 段ネストが往復とも正しく巻き戻っている。
//
//  ネストを担うのは既存の call_stack。委譲ハンドラは「担当オブジェクトとして」走るので、
//  その中で撃った svc も同じ一般オブジェクト経路 (トランポリン) に乗り、EXIT_METHOD の
//  pop で内側から順に戻る。カーネル側に専用のネスト機構は要らない — これを実機で確かめる
//  のがこのテストの目的。
//
//  ★カーネルは一切関与しない: 「発行元が KERNEL_OBJECT か否か」という従来の判断だけ。
//  svc 番号 → 担当オブジェクトのルート表はカーネル**オブジェクト**が持ち、委譲は
//  通常の METHOD_CALL (保護されたサブルーチン呼び出し) で行う。したがって担当ハンドラは
//  **普通の export メソッド**でよく、専用の戻り口も特別な pop 段数も要らない。
//  戻り値を返したいときだけ obj_api の EXIT_METHOD を自分で撃つ (一般オブジェクトの正規 API)。
// ===========================================================================
#include <cstdint>
#include <kernel.hpp>

#if SHIZU_SVC_DELEGATION

#include <log.hpp>
#include <obj_api.hpp>
#include <object_id.hpp>
#include <shizu.hpp>

#include <svc_handler.hpp>

namespace shizu {
namespace {

constexpr uint32_t SVC_A = 120; // kernel_obj_svc_handler の臨時ハック (100/255) を避ける
constexpr uint32_t SVC_B = 121;
constexpr uint32_t SVC_P = 122; // 最小プローブ (ネスト無し)

volatile uint32_t g_p_hits = 0, g_p_caller = 0xFFFF, g_p_num = 0, g_p_arg = 0;

volatile uint32_t g_a_hits = 0, g_b_hits = 0;
volatile uint32_t g_a_caller = 0xFFFF, g_b_caller = 0xFFFF;
volatile uint32_t g_a_num = 0, g_b_num = 0;

// ---- サブハンドラ: kernel_obj_svc_handler と**完全に同じ ABI** ----------------
// r4 = svc 番号 / r5 = 発行元 obj / r6 = 発行元 thread / r7 = 今のネスト数 /
// r0..r3 = 引数。
// in_handler=1 で走るので生のカーネルプリミティブを直接叩ける。普通に return すれば
// カーネルが lr に載せた exit_method が 1 枚 pop で戻す。
uint32_t handler_b(uint32_t r0, uint32_t r1, uint32_t r2, uint32_t r3,
                   uint32_t r4, uint32_t r5, uint32_t r6, uint32_t r7) {
  (void)r0; (void)r2; (void)r3; (void)r6; (void)r7;
  g_b_hits = g_b_hits + 1;
  g_b_caller = r5;   // ★発行元がそのまま見える (METHOD_CALL 版では 0 に化けていた)
  g_b_num = r4;
  if (r1 != 0)
    return obj_api::svc<SVC_A>(0, r1 - 1, 0, 0).value + 1;
  return 0;
}
uint32_t handler_a(uint32_t r0, uint32_t r1, uint32_t r2, uint32_t r3,
                   uint32_t r4, uint32_t r5, uint32_t r6, uint32_t r7) {
  (void)r0; (void)r2; (void)r3; (void)r6; (void)r7;
  g_a_hits = g_a_hits + 1;
  g_a_caller = r5;
  g_a_num = r4;
  if (r1 != 0)
    return obj_api::svc<SVC_B>(0, r1 - 1, 0, 0).value + 1;
  return 0;
}

// ---- 最小プローブ用ハンドラ ------------------------------------------------
// ★入れ子 svc を一切撃たず、引数に定数を足して return するだけ。これで
// 「非カーネルオブジェクトのハンドラが **起動して戻れるか**」という一番手前の問いを、
// ネストの往復から切り離して確かめる。6 段ネストで一気に見ようとすると、1 段目で
// 死んでいるのか往復で死んでいるのか区別できない (実際それで探索範囲を絞れなかった)。
constexpr uint32_t PROBE_ADD = 7;
uint32_t handler_probe(uint32_t r0, uint32_t r1, uint32_t r2, uint32_t r3,
                       uint32_t r4, uint32_t r5, uint32_t r6, uint32_t r7) {
  (void)r0; (void)r2; (void)r3; (void)r6; (void)r7;
  g_p_hits = g_p_hits + 1;
  g_p_caller = r5;
  g_p_num = r4;
  g_p_arg = r1;
  return r1 + PROBE_ADD;
}

void a_main() { while (true) obj_api::yield_us(1000000); }
void b_main() { while (true) obj_api::yield_us(1000000); }

void t_main() {
  obj_api::yield_us(50000);
  // 方針 (どの svc を誰に回すか) は合成側が決める。担当は明示指定。
  obj_api::set_svc_route(SVC_A, SVC_A, (uint32_t)object_ids::SVC_DELEGATE_A,
                         (uintptr_t)&svc_handler_shim<handler_a>);
  obj_api::set_svc_route(SVC_B, SVC_B, (uint32_t)object_ids::SVC_DELEGATE_B,
                         (uintptr_t)&svc_handler_shim<handler_b>);

  // ---- 段階 0: 最小プローブ (1 段、入れ子なし) ------------------------------
  // ここが通れば「委譲の 1 往復」は成立している。通らなければ問題は REDISPATCH_SVC
  // の 1 往復の中にあり、ネストの話は無関係になる。
  obj_api::set_svc_route(SVC_P, SVC_P, (uint32_t)object_ids::SVC_DELEGATE_A,
                         (uintptr_t)&svc_handler_shim<handler_probe>);
  log::printf("[SVCDLG] probe: issuing svc %lu ...\n", (unsigned long)SVC_P);
  {
    constexpr uint32_t PROBE_ARG = 35;
    auto pr = obj_api::svc<SVC_P>(0, PROBE_ARG, 0, 0);
    const bool ok = (pr.value == PROBE_ARG + PROBE_ADD) && (g_p_hits == 1) &&
                    (g_p_caller == (uint32_t)object_ids::SVC_DELEGATE_T) &&
                    (g_p_num == SVC_P) && (g_p_arg == PROBE_ARG);
    log::printf("[SVCDLG] probe: got=%lu want=%lu hits=%lu caller=%lu num=%lu "
                "arg=%lu => %s\n",
                (unsigned long)pr.value, (unsigned long)(PROBE_ARG + PROBE_ADD),
                (unsigned long)g_p_hits, (unsigned long)g_p_caller,
                (unsigned long)g_p_num, (unsigned long)g_p_arg,
                ok ? "PROBE PASS" : "PROBE FAIL");
  }
  // プローブ結果を確実に排出させてから、危ないネスト段へ進む (ここで固まっても
  // 上の 2 行は表に出ている状態にしておく)。
  obj_api::yield_us(200000);

#if SHIZU_SVC_DELEGATE_NEST
  constexpr uint32_t MAX_N = 6;
  uint32_t pass = 0, fail = 0, deepest = 0;
  for (uint32_t n = 1; n <= MAX_N; ++n) {
    auto r = obj_api::svc<SVC_A>(0, n, 0, 0);
    if (r.value == n) { ++pass; deepest = n; }
    else {
      ++fail;
      log::printf("[SVCDLG] depth=%lu MISMATCH got=%lu want=%lu\n",
                  (unsigned long)n, (unsigned long)r.value, (unsigned long)n);
    }
    obj_api::yield_us(1000);
  }
  // ★発行元 identity が保たれているか: 最後の A 呼び出しは B から、B は A から。
  const bool ident_ok = (g_a_caller == (uint32_t)object_ids::SVC_DELEGATE_B) &&
                        (g_b_caller == (uint32_t)object_ids::SVC_DELEGATE_A) &&
                        (g_a_num == SVC_A) && (g_b_num == SVC_B);
  log::printf("[SVCDLG] depth 1..%lu: pass=%lu fail=%lu deepest=%lu "
              "a=%lu b=%lu caller(a=%lu b=%lu) num(a=%lu b=%lu)%s => %s\n",
              (unsigned long)MAX_N, (unsigned long)pass, (unsigned long)fail,
              (unsigned long)deepest, (unsigned long)g_a_hits,
              (unsigned long)g_b_hits, (unsigned long)g_a_caller,
              (unsigned long)g_b_caller, (unsigned long)g_a_num,
              (unsigned long)g_b_num, ident_ok ? " OK" : " BAD",
              (fail == 0 && deepest == MAX_N && ident_ok) ? "NEST+IDENT PASS"
                                                          : "FAIL");
#else
  log::printf("[SVCDLG] nest stage skipped (SHIZU_SVC_DELEGATE_NEST=0)\n");
#endif
  while (true) obj_api::yield_us(2000000);
}

} // namespace

void svc_delegate_test_launch() {
  FOR_KERNEL_OBJECT::create_object((uint32_t)object_ids::SVC_DELEGATE_A);
  FOR_KERNEL_OBJECT::async_call((uint32_t)object_ids::SVC_DELEGATE_A, (method_t)a_main);
  FOR_KERNEL_OBJECT::create_object((uint32_t)object_ids::SVC_DELEGATE_B);
  FOR_KERNEL_OBJECT::async_call((uint32_t)object_ids::SVC_DELEGATE_B, (method_t)b_main);
  FOR_KERNEL_OBJECT::create_object((uint32_t)object_ids::SVC_DELEGATE_T);
  FOR_KERNEL_OBJECT::async_call((uint32_t)object_ids::SVC_DELEGATE_T, (method_t)t_main);
}

} // namespace shizu

#endif // SHIZU_SVC_DELEGATION
