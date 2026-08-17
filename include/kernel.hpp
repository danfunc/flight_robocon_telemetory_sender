
#ifndef SHIZU_KERNEL_HPP
#define SHIZU_KERNEL_HPP
#include <cstdint>
#include <pico.h> // get_core_num (grant_active_this_core)

// デュアルコアトグル: 1 = core1 を Shizuku カーネルとして起動し、センサ I/O を
// SENSOR_IO スレッド (core1 ピン留め) として走らせる。0 = 従来どおり core1 を
// ベアメタル sensor I/O (core1_io_launch) に使う (フォールバック)。
#ifndef SHIZU_CORE1_KERNEL_POC
#define SHIZU_CORE1_KERNEL_POC 1
#endif

// ストリーム自己テスト トグル: 1 = STREAM_TEST オブジェクトの producer/consumer
// スレッド対を作り、共有メモリストリームで create/open/bind/push/pop を実動確認する。
#ifndef SHIZU_STREAM_SELFTEST
#define SHIZU_STREAM_SELFTEST 0
#endif

// 時限実行権移譲 (run_for/GRANT_CPU) の自己テスト トグル: 1 = GRANT_TEST オブジェクトの
// スレッド群で EXPIRED/YIELDED/ネスト/長期限 (SysTick チャンク継ぎ) を実動確認する。
#ifndef SHIZU_GRANT_SELFTEST
#define SHIZU_GRANT_SELFTEST 0
#endif

// SMP ストレス トグル: 1 = SMP_STRESS オブジェクトの worker 群 (core0×2 / core1×2 /
// 両コア移動×1) が SLEEP/SWITCH の SVC を両コアから同時多発させ、カーネル SVC 経路の
// 2 コア安全性を数分単位で炙る。reporter がカウンタを 2s ごとに印字 (全て前進 = 健全)。
#ifndef SHIZU_SMP_STRESS
#define SHIZU_SMP_STRESS 0
#endif

// CPU ホグ負荷試験トグル: 1 = BUSY_LOAD オブジェクトに「yield しない無限ループ」を
// BLE と同じ core1 へ既定 budget (3ms) でピン留めし、凍結ウォッチドッグの取り上げが
// 通常運転の BLE と同居して効くこと (= ホグが BLE を妨げない) を実地確認する。reporter
// が生存カウンタを 2s ごとに印字 (ADVANCING = ホグが 3ms スライスをもらえ系も非凍結)。
// BLE 側の無事は BLE_UART の 1s tx 統計 (ctrl_lat) / ブラストで別途見る。
#ifndef SHIZU_BUSY_LOAD
#define SHIZU_BUSY_LOAD 0
#endif

// RT スケジューラ検証トグル: 1 = RT_SCHED_TEST に「合成周期 victim (代表レートで締切
// ジッタを自己計測) + never-yield hog + reporter」を置き、CPU ホグ同居下でも周期タスクが
// 締切 (ジッタ ~budget) を保つかを device 側だけで測る (センサ実機/ホスト接続不要)。
// reporter が 2s ごとに victim ジッタ表と hog 生存を stdio へ印字。詳細は rt_sched_test.cpp。
#ifndef SHIZU_RT_SCHED_TEST
#define SHIZU_RT_SCHED_TEST 0
#endif

// BLE poll 計装トグル: 1 = BLE_UART の主ループで cyw43_arch_poll() 1 回の所要時間を測り、
// 窓内最大を 1s 統計へ出す + 閾値 (BLE_POLL_WARN_US) 超えは接続/認可/NC 状態つきで即警告。
// 目的: RT を数秒スタールさせる「budget-0 の BLE がコアを握る一塊」が cyw43_arch_poll 内の
// 単発長時間処理か (ble_dbg_poll が止まるか) を実機で確定し、それが接続/ペアリング局面に
// 限るか定常でも起きるかを切り分ける。既定 OFF (計装の printf/時刻読みを常用ビルドに載せない)。
#ifndef SHIZU_BLE_POLL_INSTR
#define SHIZU_BLE_POLL_INSTR 0
#endif

// 診断トグル: 1 = 接続時の 2M PHY プローブ (gap_le_set_phy) を無効化する。接続時に必ず出る
// ~38ms/ときに1s の LONG poll が、この PHY 設定 IOCTL 由来かを切り分ける (OFF で消えれば犯人)。
#ifndef SHIZU_BLE_NO_PHY_PROBE
#define SHIZU_BLE_NO_PHY_PROBE 0
#endif

// SVC 委譲トグル (HANDOFF §7 の「オブジェクトランドの syscall ハンドリングを開放」):
// 1 = svc 番号の範囲ごとに「捌く担当オブジェクト」を登録できるようにする。
// 現行は全 svc が単一の kernel_obj_svc_handler へトランポリンするが、これを
// 「番号 → 担当オブジェクト」の表引きに一般化し、サブシステムが自分の syscall を
// 自分で捌ける (sub-kernel 化) ようにする。
// ★方針はカーネルオブジェクトも感知しない: カーネルは表を機械的に引くだけで、
//   どの範囲を誰に割り当てるかを解釈しない (ログの sink 表と同じ「機構と方針の分離」)。
// ネストは既存の call_stack がそのまま担う — 委譲ハンドラは「担当オブジェクトとして」
// 走るので、その中でさらに委譲された svc を撃つと同じトランポリンがもう一段積まれ、
// EXIT_METHOD の pop で順に戻る。2 段ネストは svc_delegate_test.cpp で実証する。
// ★既定 ON (2026-08-17)。実機検証済み: 最小プローブ (1 段) / 6 段ネスト / 発行元
// identity の保存 が全て PASS し、系も健全 (BEACON 前進 / util 不変 / panic 0)。
// **委譲ハンドラに特権は一切与えない** — 一般オブジェクトのまま走り、戻りは
// delegate_exit_method (obj_api EXIT_METHOD) 経由。以前あった
// `in_handler && to_kernel_primitive` の昇格 (SET_SVC_HANDLER 乗っ取りが可能だった)
// は削除済み。ルート未登録なら表引きが空振りするだけでコストは実質ゼロ。
#ifndef SHIZU_SVC_DELEGATION
#define SHIZU_SVC_DELEGATION 1
#endif


// 委譲の自己テスト (svc_delegate_test.cpp) を起動するか。**既定 OFF** — 委譲機構は
// 既定 ON になったが、テストはオブジェクト 3 個 + スレッド 3 本を常駐させるので
// 製品ビルドには載せない。機構だけを載せた構成との切り分けにも使える
// (無言ハングの原因がテスト側かカーネル経路かを 1 回の書き込みで判別できる)。
// 有効化: cmake -DSHIZU_SVC_DELEGATE_TEST=ON
#ifndef SHIZU_SVC_DELEGATE_TEST
#define SHIZU_SVC_DELEGATE_TEST 0
#endif

// 委譲テストのネスト段 (6 段の相互再帰) を走らせるか。0 = 最小プローブ (1 段、
// 入れ子なし) だけで止める。プローブが通るかどうかを、ネストの往復から切り離して
// 確かめるための段階分け。cmake -DSHIZU_SVC_DELEGATE_NEST=OFF
#ifndef SHIZU_SVC_DELEGATE_NEST
#define SHIZU_SVC_DELEGATE_NEST 1
#endif

// 委譲エントリ数。svc 即値の番号空間は 0..255 なので少数で足りる。
#ifndef SHIZU_SVC_DELEGATE_SLOTS
#define SHIZU_SVC_DELEGATE_SLOTS 8
#endif

// SHIZUKU_USB トグル: 1 = printf (USB CDC) を「待たない」経路にする。自前 stdio
// ドライバがリングへ積んで即 return し、SHIZUKU_USB の排出スレッドが CDC の空き分だけ
// を SDK へ渡す (詳細は SHIZUKU_USB.cpp 冒頭)。
// ★実測 (2026-08-07, BLE 接続/切断 n=3): 「BLE を接続するとフリーズする」の原因は
//   cyw43 ではなく printf で、SDK の pico_stdio_usb が呼び出し元スレッドで
//   1000ms (print_mutex) / 500ms (CDC 満杯ビジー回し) 待つのが正体だった。
//   stdio を非ブロッキングにするだけで単発 poll 最大 1.006s → 0.038s、
//   RT late(max) 1.008s → 0.046s (3/3 再現)。
#ifndef SHIZU_USB_DRIVER
#define SHIZU_USB_DRIVER 0
#endif

// Stage 2 修正トグル (BLE 完全化): 1 = cyw43 の駆動を BLE 主ループから切り離し、
//   ① 独自 host-wake IRQ (WL_HOST_WAKE の GPIO 割り込みに自前ハンドラを相乗り登録)
//   ② その通知で起きる Shizuku の cyw43 サービススレッド (BLE と同じコアにピン留め、
//      budget0)
//   ③ 単一所有ロック (cyw43/btstack に触るのは常に 1 スレッドだけ = ロック順守)
// の 3 点で「チップが上げた host-wake → サービススレッドがロックを取ってドレイン →
// 待っている IOCTL/HCI トランザクションがその場で完了」という鎖を作る。
// Stage 1 (待ち→yield) だけでは、待ちの間ドレインする主体が居らず、待ちは毎回
// タイムアウト満了 (最悪 1s) まで伸びていた。Stage 2 はそのドレイン主体を用意する。
// 副次効果: BLE のアプリ層 (広告保険/ウォッチドッグ/RSSI/NC 確認/TX 補填) が
// cyw43 のドレインと同じスレッドに詰まらなくなる。既定 OFF。
// ★ Stage 2 は Stage 1 を内包する (待ちが yield しないとサービススレッドが走れない)
//   ので、下の SHIZU_CYW43_YIELD_WAIT は既定でこの値に追従する。
#ifndef SHIZU_CYW43_SVC_THREAD
#define SHIZU_CYW43_SVC_THREAD 0
#endif

// Stage 1 修正トグル: 1 = cyw43 の待ち (async_context の wait_for_work_until) を WFE から
// Shizuku の yield へ置き換える。cyw43 IOCTL/SDPCM の応答待ちがコアを WFE で握るのをやめ、
// 待ちの間 Shizuku スケジューラへ制御を返す → cyw43 の応答遅延 (最悪 1s) でも core1 の RT
// タスクが凍結しなくなる (= 待ちを「スケジューラ管轄内」へ戻す)。POLL モード再入で IOCTL
// 自体が完了しない件は Stage 2 (SHIZU_CYW43_SVC_THREAD) で対処。既定 OFF
// (ただし Stage 2 が ON なら自動で ON)。
#ifndef SHIZU_CYW43_YIELD_WAIT
#define SHIZU_CYW43_YIELD_WAIT SHIZU_CYW43_SVC_THREAD
#endif
#if SHIZU_CYW43_SVC_THREAD && !SHIZU_CYW43_YIELD_WAIT
#error "SHIZU_CYW43_SVC_THREAD (Stage 2) requires SHIZU_CYW43_YIELD_WAIT (Stage 1)"
#endif

// Stage 2: cyw43 サービススレッドの巡回間隔 [µs]。host-wake IRQ は Shizuku の sleep 中の
// スレッドを起こせない (IRQ から SVC は撃てない) ので、ドレイン遅延の上限はこの値になる。
// 既定 1000µs = Stage 1 以前の主ループ poll と同じ刻み (退行させないための下限)。
// 大きくすると core の余裕は増えるが host-wake → ドレインの遅延も増える。
#ifndef SHIZU_CYW43_SVC_NAP_US
#define SHIZU_CYW43_SVC_NAP_US 1000
#endif

// Pico 2 (無印) 一時テストビルド用トグル: 1 = BLE (cyw43/btstack/BLE_UART_DRIVER) を
// ビルドから外す。CMake の -DSHIZU_PICO2_TEST=1 が定義する (手で立てるものではない)。
// RP2350 チップは pico2_w と同一なので、カーネル検証 (PSPLIM/MPU/DMA/affinity) は等価。
#ifndef SHIZU_NO_BLE
#define SHIZU_NO_BLE 0
#endif

// BLE ドライバを core1 で走らせる実験トグル: 1 = BLE_UART_DRIVER スレッドを
// AFFINITY_CORE1 + budget0 (バトン組) で生成する。cyw43/btstack/async_context の
// コア縛りは「init を実行したコアと同じコアからしか触れない」なので、init〜poll
// ループ全体が core1 ピン留めスレッドに載れば整合する (CYW43 の GPIO IRQ と
// async_context の owner も core1 になる)。0 = 従来どおり core0。
// 既知のクロスコア面: (i) tx_srcs 排出表への登録は core0 の TELEMETRY スレッドで
// 走る → append+publish 化で対処済み。(ii) RX sink 配送 (BLE スレッドが core1 で
// TELEMETRY のハンドラを実行) は TELEMETRY 内部状態と並行になる — 低頻度
// (ping/コマンド行) の既知リスクとして許容 (実験トグルたる所以)。
#ifndef SHIZU_BLE_ON_CORE1
#define SHIZU_BLE_ON_CORE1 1
#endif

// MPU Step1 プロトタイプトグル (HANDOFF §6): 1 = FLIGHT_CONTROLLER を
// unprivileged (CONTROL.nPRIV=1) で走らせる。既定 0 — W^X/BLE-on-core1 と違い
// このカーネルで「不特権スレッドを実際に走らせる」のはこのセッションが初めてで、
// CONTROL 遷移機構 (svc_cpp_handler の METHOD_CALL/トランポリン, svc_asm_handler.S
// の CTX_RESTORE) 自体が実機未検証のため、デフォルト ON にしない。
// 既知のリスク: FLIGHT_CONTROLLER::main の printf は pico-sdk 内部で
// save_and_disable_interrupts (PRIMASK 経由の CPSID) を使うことがあり、CPS 系命令は
// ARMv8-M では unprivileged 実行時に NOP 化される (フォールトはしないが、意図した
// 割り込み禁止がサイレントに効かなくなる) — このトグル ON のビルドでは
// FLIGHT_CONTROLLER.cpp 側で該当 printf を SHIZU_STEP1_UNPRIV_FLIGHT_CONTROLLER
// でガードして呼ばないようにしてある。region1 (W^X heap) は unprivileged にも
// RW を許可しているため、他オブジェクトのスタック/静的データへのアクセス自体は
// フォールトしない (= このプロトタイプは「特権遷移機構の実地検証」が目的で、
// オブジェクト間の隔離はまだ提供しない。隔離は Step2 の per-subsystem region)。
#ifndef SHIZU_STEP1_UNPRIV_FLIGHT_CONTROLLER
#define SHIZU_STEP1_UNPRIV_FLIGHT_CONTROLLER 0
#endif

// 非特権実行の最小プローブ トグル: 1 = UNPRIV_PROBE オブジェクトを
// CONTROL.nPRIV=1 で走らせる。**グローバル/static も標準ライブラリも一切使わず**、
// 状態は malloc したブロック (= MPU region1 = 非特権 RW) にだけ置き、yield は生の
// svc で撃つ。MPU 設定を変えずに「特権遷移機構だけ」を切り出して検証するためのもの。
// 詳細と背景は unpriv_probe.cpp 冒頭 / docs/SHIZUKU_DESIGN.md §11.2。
// cmake -DSHIZU_UNPRIV_PROBE=ON
#ifndef SHIZU_UNPRIV_PROBE
#define SHIZU_UNPRIV_PROBE 0
#endif

// MPU Step0 (W^X) トグル: 1 = 両コアで MPU を有効化し、(i) XIP flash 全窓 = RO+X、
// (ii) ヒープ先頭 (__end__) 〜 SRAM 終端 = RW+XN を張る。スタック/ヒープからのコード
// 実行と flash への誤ストアを即 fault にする (HANDOFF §6 Step0 の残り半分)。
// PRIVDEFENA=1 なので region 外 (静的データ / ペリフェラル / SIO) は従来どおり
// default map で無傷。実機で問題が出たら 0 で丸ごと外せる。
#ifndef SHIZU_MPU_WX
#define SHIZU_MPU_WX 1
#endif

// スケジューラが budget 付きスレッドへ与える時限の既定値 [µs]。
// 意味論は quantum でなく「凍結ウォッチドッグ」: 全ドライバは正常時これより十分
// 短い周期で yield する (最長の正当ホールドは I2C 100kHz 26B 読み ≈2.4ms) ので、
// 期限切れは「従来なら全系凍結だった異常時」にしか発火しない。
#ifndef SHIZU_DEFAULT_GRANT_BUDGET_US
#define SHIZU_DEFAULT_GRANT_BUDGET_US 3000
#endif

int main(int argc, char const *argv[]);

namespace shizu {
using method_t = uint32_t (*)(uint32_t, uint32_t, uint32_t, uint32_t);

#if SHIZU_SVC_DELEGATION
// svc 委譲の計装: リセットを跨ぐ noinit RAM に「どの活性化が・どの番号を・どちらの
// 経路へ送ったか」の直近 N 件を残す。**固まっても次のブートで読める**ので、無音
// ロックアップでも 1 回焼けば原因が確定する (焼く→固まる→情報ゼロ、を繰り返さない)。
struct svc_trace_t {
  uint32_t magic;   // SVC_TRACE_MAGIC なら有効
  uint32_t n;       // 記録した総件数 (リングは n % SVC_TRACE_SLOTS)
  struct entry_t {
    uint16_t num;      // svc 即値
    uint8_t obj;       // 発行元 object_id
    uint8_t flags;     // bit0=in_handler, bit1=primitive 判定, bit2=トランポリンへ
    uint32_t pc;       // 発行元 pc (addr2line で行番号にできる)
  } e[16];
};
constexpr uint32_t SVC_TRACE_SLOTS = 16;
constexpr uint32_t SVC_TRACE_MAGIC = 0x53564354; // "SVCT"
extern svc_trace_t svc_trace;
void svc_trace_boot_report(); // ブート時に前回の残骸を印字してクリア
void svc_trace_record(uint32_t num, uint32_t obj, uint32_t in_handler,
                      bool primitive, bool trampoline, uint32_t pc);
#endif

void exit(uint32_t return_code);
[[noreturn]] __always_inline void init();
[[noreturn]] __always_inline void set_current_context_as_kernel_init();
[[noreturn]] void init_core1();  // core1 のカーネル入口 (core1_boot.cpp)
void core1_kernel_launch();      // core0 から core1 を起動する (core1_boot.cpp)
void stream_selftest_launch();   // ストリーム自己テストの起動 (stream_selftest.cpp)
void grant_selftest_launch();    // 時限移譲自己テストの起動 (grant_selftest.cpp)
void smp_stress_launch();        // 2 コア SVC ストレスの起動 (smp_stress.cpp)
void busy_load_launch();         // CPU ホグ負荷試験の起動 (busy_load.cpp)
void rt_sched_test_launch();     // RT スケジューラ検証の起動 (rt_sched_test.cpp)
#if SHIZU_UNPRIV_PROBE
void unpriv_probe_launch();      // 非特権実行の最小プローブ (unpriv_probe.cpp)
#endif
#if SHIZU_SVC_DELEGATION
void svc_delegate_test_launch(); // svc 委譲 2 段ネスト実証の起動 (svc_delegate_test.cpp)
#endif
// per-core の例外優先度設定 (SVC 最優先 > SysTick > PendSV 最低)。SHPR は banked な
// ので core0 は cpu_manager::init、core1 は init_core1 がそれぞれ呼ぶ。
void init_exception_priorities();
// MPU Step0 (W^X) を自コアで有効化する (SHIZU_MPU_WX=0 なら no-op)。MPU レジスタは
// per-core banked なので core0 は cpu_manager::init、core1 は init_core1 が呼ぶ。
void mpu_init_this_core();
enum struct grant_end : uint32_t; // 前方宣言 (定義は下の grant セクション)
// grant を 1 段 pop して grantor へ復帰させる (kernel.cpp 内部: PendSV=EXPIRED /
// SWITCH_THREAD インターセプト=YIELDED の共通経路)。
void grant_pop_to_grantor(uint32_t core, grant_end reason);
class cpu_manager;
struct object_t;
struct thread_t;
struct context_t;
struct exception_frame_t;
struct call_frame_hdr_t;

extern object_t object_table[128];
extern thread_t thread_table[128];
// create_thread/async_call の budget_us 引数「thread_table_init の既定を維持」の
// センチネル (0 は「無制限バトン」という正当値なので別値が要る)。
constexpr uint32_t BUDGET_KEEP = 0xFFFFFFFFu;

namespace FOR_KERNEL_OBJECT {
void export_method(uint32_t obj_num, uint32_t method_num, method_t entry);
void exit_method(uint32_t return_code);
// 委譲サブハンドラ (一般オブジェクト) 用の戻り口。obj_api EXIT_METHOD 経由で
// 2 枚巻き戻す。一般オブジェクトへ生のカーネルプリミティブを渡さないための口。
void delegate_exit_method(uint32_t return_code);
void create_object(uint32_t obj_num, uint32_t thread_num, method_t entry);
void create_object(uint32_t obj_num); // オブジェクトのみ生成 (スレッド無し)
// arg は entry の r0 に載る。affinity (0=既定 core0 を維持) と budget_us
// (BUDGET_KEEP=既定を維持, 0=バトン組) は READY 公開**前**に確定する — 別コア行き
// スレッドを「作ってから set_affinity」の隙間レース (途中初期化のまま他コアが
// claim / 既定 core0 で走り出してから移動) なしに生成するための生成時オプション。
void create_thread(uint32_t obj_num, uint32_t thread_num, method_t entry,
                   uint32_t arg = 0, uint32_t affinity = 0,
                   uint32_t budget_us = BUDGET_KEEP);
// 空きスレッドスロット (1..127) を自動確保して entry を非同期起動する。戻り = 割り当てた
// thread_id。オブジェクト起動 = create_object(id) + async_call(id, main) の 2 段で行う。
uint32_t async_call(uint32_t obj_num, method_t entry, uint32_t arg = 0,
                    uint32_t affinity = 0, uint32_t budget_us = BUDGET_KEEP);
} // namespace FOR_KERNEL_OBJECT

} // namespace shizu

extern "C" {
void svc_asm_handler();
void pendsv_asm_handler(); // grant 期限の強制切替入口 (svc_asm_handler.S)
shizu::context_t *get_current_context();
void svc_cpp_handler(shizu::context_t *context);
void pendsv_cpp_handler(shizu::context_t *context);
void systick_grant_handler(); // grant 期限の監視 (tickless ワンショット)
void shizu_panic(const char *fmt, ...); // panic() の差し替え先 (panic_ring.hpp)
}

namespace shizu {

class cpu_manager {
  static uint32_t current_thread_id[2]; // per-core: SIO cpuid (get_core_num) で索引
  static void init();
  struct svc_handler_descriptor {
    method_t entry_point;
    uint32_t handling_object_id;
  } static svc_handler_info;
  friend void shizu::init();
  friend void shizu::init_core1();
  friend shizu::context_t * ::get_current_context();
  friend int ::main(int argc, char const *argv[]);
  friend void ::svc_cpp_handler(shizu::context_t *context);
  friend void ::pendsv_cpp_handler(shizu::context_t *context);
  friend void shizu::grant_pop_to_grantor(uint32_t core, grant_end reason);
  friend void ::shizu_panic(const char *fmt, ...); // panic 記録に thread id を載せる
};

// object_t::thread_table の実体。所属スレッド集合は書き込み専用 (insert/clear のみ、
// 現状どこからも列挙されない) — 元は std::set だったが、insert が SVC トランポリン
// 経由の thread mode で毎回 malloc/木リバランスする「std::map 根絶」で狙った対象と
// 同種の残存箇所だったため、固定 128bit ビットマップに置き換える (malloc ゼロ)。
// 将来の MemManage→kill/再起動 (HANDOFF §6) が「このオブジェクトの全スレッド」を
// 引くのに使う想定で API 形だけ std::set と揃えてある。
struct thread_bitmap_t {
  uint32_t bits[4] = {}; // 128 スレッド分 (4 x 32bit)
  void clear() { bits[0] = bits[1] = bits[2] = bits[3] = 0; }
  void insert(uint32_t tid) { bits[tid >> 5] |= (1u << (tid & 31)); }
  bool test(uint32_t tid) const { return (bits[tid >> 5] >> (tid & 31)) & 1u; }
};

struct object_t {
  uint32_t id;
  uint32_t svc_handler_obj_id;
  uint32_t svc_handler_method_num;
  thread_bitmap_t thread_table;
  method_t method_table[128];
  enum struct state_t {
    UNINITIALIZED = 0,
    KERNEL_OBJECT = 1,
    KERNEL_SPACE_OBJECT = 2,

  } state;
  // MPU Step1 プロトタイプ (HANDOFF §6): true = このオブジェクトとして走行中は
  // CONTROL.nPRIV=1 (unprivileged Thread mode)。既定 false = 従来どおり全オブジェクト
  // 特権 (このセッションまでの挙動と完全互換)。svc_cpp_handler が METHOD_CALL /
  // METHOD_EXIT / トランポリンでの object 遷移のたびに CONTROL を張り替える。
  // SET_OBJECT_UNPRIVILEGED (obj_api::svc_num) で、対象オブジェクトのスレッドが
  // 走り出す**前**に (create_object 直後・async_call 前に) 立てること — 生成時に
  // context->control を確定するので、後から立てても既存スレッドには反映されない。
  bool unprivileged = false;
};

// obj_api::svc_num::EXIT_METHOD の値 (obj_api.hpp は kernel.hpp を include するので
// 循環を避けてここに定数で置く。両者がズレたら delegate_exit_method が壊れる)。
constexpr uint32_t obj_api_EXIT_METHOD_NUM = 5;

enum struct kernel_object_svc_num : uint32_t {
  GET_CURRENT_THREAD_ID = 0,
  METHOD_CALL = 200,
  // 巻き戻し。arg0=戻り値 (呼び出し元の r1 へ), arg1=**追加** pop 段数
  // (落とす枚数 = arg1+1。任意段数を指定してよい), arg2=エラーコード (呼び出し元の
  // r0 へ), arg3=発行側が申告する「今のネスト数」= call_stack の深さ。
  // arg3 は 0 で「申告しない」(旧呼び出し元との互換)。非 0 なら実際の深さと一致
  // しない限り 1 段も落とさず exit_error::DEPTH_MISMATCH を返す。
  METHOD_EXIT = 201,
  SVC_EXIT = 201,
  SET_SVC_HANDLER = 202,
  SWITCH_THREAD = 203,
  // SWITCH_THREAD を「READY→RUNNING の CAS claim + affinity 検査 + 成否返却」に
  // 再定義した別名。旧 SWITCH_THREAD 呼び出し元は r0 を見ないので互換。
  TRY_SWITCH_THREAD = 203,
  GET_OBJECT_ID_FROM_THREAD_ID = 204,
  GET_THREAD_STATE = 205,
  // 時限実行権移譲: arg0=対象 tid, arg1=最大実行時間[µs]。対象を claim して切替え、
  // 期限到来 (SysTick→PendSV 強制切替) か対象の yield/switch (早期復帰) で戻る。
  // 戻り: r0=grant_error, r1=grant_end (EXPIRED/YIELDED)。
  GRANT_CPU = 206,
#if SHIZU_SVC_DELEGATION
  // svc 委譲: 今扱っている syscall を「オブジェクト arg1 のハンドラ arg0」として
  // 実行し直す。カーネルは番号を一切見ない — 誰に回すかを決めるのは発行元
  // (= カーネルオブジェクトのハンドラ) であり、カーネルは機械的に張り替えるだけ。
  // サブハンドラは in_handler=1 で走り、r4/r5/r6 に**元の呼び出し情報**を受け取る
  // (kernel_obj_svc_handler と同じ ABI)。戻りは普通に return → exit_method (1 枚 pop)。
  REDISPATCH_SVC = 208,
#endif
};

// TRY_SWITCH_THREAD (claim) の型付きエラー。error_t<switch_error> で受ける。
// (SMP 基盤: 各コアのスケジューラが READY スレッドを原子的に claim して、2 コアが
//  同一 context_t を走らせないようにする。AMP 相当は affinity 固定という policy。)
enum struct switch_error : uint32_t {
  OK = 0,       // 切替成功 (自スレッド指定の no-op も OK 扱い)
  NOT_READY,    // 対象が READY でない / claim 競り負け (他コアが先に取った)
  BAD_AFFINITY, // 対象がこのコアで走ることを許されていない
};
// METHOD_CALL (メソッド呼び出しプリミティブ) の型付きエラー。error_t<call_error> で
// 受ける。従来は未 export メソッドへの呼び出しで panic 停止していたが、affinity で
// オブジェクトのコアを動かすと「スレッド番号昇順の初回実行順」による export 順の暗黙
// 保証が崩れるため、エラー返却に変更した — 呼び出し元は yield して再試行できる。
enum struct call_error : uint32_t {
  OK = 0,            // 呼び出し成功 (r1 にメソッド戻り値)
  BAD_OBJECT,        // obj/method 番号が範囲外、または対象オブジェクト未生成
  UNDECLARED_METHOD, // 対象メソッドが未 export (export_method 前に呼んだ)
  NO_STACK,          // 呼び出しフレームを積むスレッドスタックが足りない (PSPLIM 手前)
  UNKNOWN_API,       // 未知の obj_api 番号。**黙って捨てず**エラーで返す
                     // (一般オブジェクトからの呼び出しなので panic は禁止 = I-9)
};

// METHOD_EXIT (巻き戻しプリミティブ) の型付きエラー。r0 に返る。
// ★巻き戻しは「何段落とすか」を発行側が自由に決められる (任意段数) が、その段数が
//   発行側の思っている呼び出し文脈と食い違っていたら黙って巻き戻してはいけない。
//   そこで発行側は arg3 に「今のネスト数」(自分が知っている call_stack の深さ) を
//   申告し、カーネルは実際の深さと突き合わせてから落とす。食い違いは巻き戻さずに
//   エラーで返す (panic しない = SVC ABI の規律)。r1 には実際の深さを返すので、
//   発行側は自分の思い違いをその場で診断できる。
enum struct exit_error : uint32_t {
  OK = 0,         // 指定段数の巻き戻し完了
  DEPTH_MISMATCH, // 申告した「今のネスト数」が実際と違う (1 段も落としていない)
  BAD_COUNT,      // 落とす段数が 0、または現在の深さを超えている (同上)
};

// affinity ビットマスク。bit0=core0, bit1=core1。thread_table_init の既定は CORE0
// (core1 が既存スレッドを勝手に claim しないよう安全側に倒す = AMP。core1 で走らせる
//  スレッドだけ明示的に CORE1 を立てる)。
constexpr uint32_t AFFINITY_CORE0 = 0b01;
constexpr uint32_t AFFINITY_CORE1 = 0b10;
constexpr uint32_t AFFINITY_ALL = 0b11;

// CONTROL レジスタの既定値 (bit1=SPSEL=1 固定: 全スレッド PSP 使用、bit0=nPRIV)。
// bit2(FPCA) はハードウェアが管理するので、この定数はキャッシュ専用 — 実際の
// MSR CONTROL は svc_asm_handler.S の CTX_RESTORE が MRS で読んだ現在値の bit0 だけを
// この値で置き換える read-modify-write を行う (FPCA/SPSEL を壊さない)。
constexpr uint32_t CONTROL_PRIV_PSP = 0b10;   // nPRIV=0 (特権)
constexpr uint32_t CONTROL_UNPRIV_PSP = 0b11; // nPRIV=1 (非特権, MPU Step1 プロトタイプ)

// ---- 時限実行権移譲 (GRANT_CPU / run_for) ---------------------------------
// GRANT_CPU の型付きエラー。先頭 3 値は switch_error と値を揃える (claim 共通ヘルパの
// 結果をそのままキャストで写せるように)。
enum struct grant_error : uint32_t {
  OK = 0,       // 移譲成功 (自スレッド指定の no-op も OK)
  NOT_READY,    // 対象が READY でない / claim 競り負け
  BAD_AFFINITY, // 対象がこのコアで走ることを許されていない
  DEPTH,        // grant スタック満杯 (ネスト深さ上限)
};
// grant の終了理由 (r1 で返る)。
enum struct grant_end : uint32_t {
  EXPIRED = 0, // 期限到来による強制復帰 (PendSV)
  YIELDED = 1, // 移譲先の yield/switch/sleep による早期復帰
};
// per-core grant スタック。ネスト許容 (内側 deadline は外側へクランプされるので
// over-time は構造的に不可能)。SVC/SysTick/PendSV は全て自コアのスタックだけを触り、
// 優先度 (SVC > SysTick > PendSV) で直列化されるためロック不要。SVC パス脱 malloc
// 方針に従い固定配列。
struct grant_frame_t {
  uint32_t grantor;  // 復帰先スレッド (WAIT_GRANT で待つ)
  uint64_t deadline; // 期限 (µs, time_us_64 基準)。外側 deadline でクランプ済み
};
struct grant_stack_t {
  static constexpr uint32_t MAX_DEPTH = 8;
  grant_frame_t frames[MAX_DEPTH];
  uint32_t depth;
};
extern grant_stack_t grant_stacks[2]; // per-core (SIO cpuid で索引)

// 自コアで grant がアクティブか (= 自分は誰かの grantee として走っているか)。
// kernel_obj_svc_handler の YIELD/SLEEP_US が「grantor への早期復帰」へ分岐するのに使う。
inline bool grant_active_this_core() {
  return grant_stacks[get_core_num()].depth != 0;
}

// ---- per-core CPU 使用率計測 (スケジューラが実務へ渡した時間の累積) ---------
// 各コアの共通 idle ループ (scheduler_idle_loop) が sched_pick_next 1 回ごとに、
// 「実際にスレッドを走らせて戻ってきた経過時間」を cpu_busy_us へ足す。誰も runnable
// でなくスピンした分は足さない。使用率 = Δcpu_busy_us/Δwall。スケジューラが唯一の
// 会計点なので core0/core1 一様、core1 専用計装が不要 (SENSOR_IO が idle 時に sleep で
// スケジューラへ戻る前提)。単一コアが自分の要素だけ触るのでロック不要。
extern volatile uint64_t cpu_busy_us[2]; // スケジューラが実務へ渡した累積時間 [µs]

// 各コアの共通スケジューラ idle ループ (kernel_object.cpp)。core0=thread0、
// core1=idle スレッドがそれぞれ自分の tid で呼び、以後この 1 実装だけが両コアで回る。
[[noreturn]] void scheduler_idle_loop(uint32_t self_tid);

struct context_t {
  uint32_t r4, r5, r6, r7, r8, r9, r10, r11; // offset 0..28
  exception_frame_t *sp;                      // offset 32 (must stay 32)
  // --- 遅延 FPU コンテキストスイッチ用 (lazy FPU context switch) ---
  // exc_return: スレッドごとにキャッシュした EXC_RETURN。bit4=0 なら拡張(FP)フレーム。
  // fp[16]    : callee-saved な FP レジスタ S16-S31 の退避域。
  // svc_asm_handler.S の .equ CTX_EXC_RETURN(36)/CTX_FP(40) と一致させること。
  uint32_t exc_return; // offset 36
  uint32_t fp[16];     // offset 40 (S16-S31)
  // psplim: このスレッドの PSP 下限 (= スタック底のアドレス)。PSP がここを下回ると
  // v8-M が UsageFault (→ HardFault にエスカレート) を上げ、スタックオーバーフロー
  // (メモリ破壊の最大手) を黙って壊す前に即検出する。スレッド生成時に 1 回だけ設定し、
  // CTX_RESTORE が切替のたびに MSR PSPLIM で復元する (スレッド寿命の間は不変なので
  // CTX_SAVE では触らない)。0 = 制限なし (未設定スレッドの安全既定)。
  // svc_asm_handler.S の .equ CTX_PSPLIM(104) と一致させること。
  uint32_t psplim; // offset 104
  // control: このスレッドが「現在のオブジェクト」として走るときの CONTROL レジスタ
  // 値のキャッシュ (CONTROL_PRIV_PSP / CONTROL_UNPRIV_PSP)。svc_cpp_handler が
  // METHOD_CALL (呼び先オブジェクトの値に更新) / トランポリン (カーネルオブジェクトの
  // 値=常に PRIV に更新) のたびに書き換える。METHOD_EXIT は call_stack のスナップ
  // ショット (= 呼び出し前の値。push は書き換えより前に取るので自動的に正しい) を
  // context_t ごと丸ごと復元するので、戻り側は明示的な書き換え不要。CTX_RESTORE が
  // 例外復帰のたびに (スレッド切替でも同一スレッド復帰でも) この値を適用する。
  // create_thread がスレッド生成時にオブジェクトの unprivileged フラグから初期化する。
  // svc_asm_handler.S の .equ CTX_CONTROL(108) と一致させること。
  uint32_t control; // offset 108
#if SHIZU_SVC_DELEGATION
  // --- svc ハンドラ活性化 (offset 112 以降。svc_asm_handler.S は触らないので
  //     ここから後ろに足す限り .equ オフセットは不変) ---
  // in_handler: この活性化が「svc ハンドラとして」走っているか。1 ならカーネルは
  //   発行元を KERNEL_OBJECT と同等に扱い、生のカーネルプリミティブを直接実行させる
  //   (= ハンドラは METHOD_EXIT を自分で叩けるので、戻るためだけの svc とその
  //    トランポリン枠が要らなくなる)。**METHOD_CALL では伝播させない** — ハンドラから
  //   呼ばれた普通のメソッドまで特権化してしまうため。
  // inv_*: トランポリンが記録する「元の呼び出し」情報。委譲 (REDISPATCH_SVC) が
  //   サブハンドラへ同じ材料を渡すのに使う。context_t ごと call_stack に退避され、
  //   METHOD_EXIT の復元で自動的に巻き戻る。
  uint32_t in_handler;        // offset 112
  uint32_t inv_svc_num;       // offset 116
  uint32_t inv_caller_obj;    // offset 120
  uint32_t inv_caller_thread; // offset 124
#endif
};

struct exception_frame_t {
  uint32_t r0, r1, r2, r3, r12, lr, pc, xPSR;
};

// ---------------------------------------------------------------------------
//  呼び出しフレーム — **スレッド自身のスタック上**に積む (2026-08-13)
// ---------------------------------------------------------------------------
//  旧実装は「context_t 丸ごと + 例外フレーム」= 168B の固定配列をスレッドごとに
//  MAX_DEPTH 分 .bss へ予約していた (20 スレッド × 32 段 = 105KB)。実際のネストは
//  通常 2〜4 段なので大半が遊び、しかも FP 退避域 (64B) を使わないフレームにも
//  払っていた。同一スレッド内の「保護されたサブルーチン呼び出し」の退避先は
//  そのスレッドのスタックであるべき — 実際に積んだ分しか消費せず、溢れは PSPLIM が
//  UsageFault で捕まえる (MAX_DEPTH の panic が要らなくなる)。
//
//  【幾何】スタックは下へ伸びる。SVC 発行時の PSP を X-fs とする
//  (fs = 例外フレーム実サイズ: 基本 32B / FP 拡張 104B。EXC_RETURN bit4 で判別)。
//
//    低位 ←                                                          → 高位
//    [ 呼び先が使う ][ 書き換え用フレーム fs ][ 退避域 total ][ 呼び出し元の生スタック ]
//                    X-total-fs             X-total        X
//                    ↑ context->sp          ↑ 復帰後 PSP
//
//  退避域 [X-total, X) の内訳 (低位→高位):
//    [ call_frame_hdr_t ][ fp[16] (has_fp のみ) ][ **元の例外フレーム fs** ]
//
//  ★元の例外フレームは退避域の一番上 = 元の位置にそのまま残る。1 バイトも動かさない。
//    (「拡張 FP フレームの S0-S15 と乖離するのでフレームを再配置するな」という
//     従来の禁止事項を構造的に踏まない。下へ複製するのは書き換え用の作業コピーだけ。)
//  ★復帰後の PSP は X-total なので、呼び先はそこから下へ伸びる = 退避域は無傷。
//
//  【FP】呼び出し元が FP 活性 (EXC_RETURN bit4=0) のときだけ S16-S31 の枠を予約し
//  has_fp を立てる。復帰時は has_fp が立っているときだけ復元する。呼び先の実行中に
//  FP 状態が変わっても、幾何は退避域に記録した total_bytes/frame_bytes を読み戻して
//  使うので push 時と pop 時で寸法がズレない。呼び出し元が FP 非活性だった場合は
//  基本フレームの EXC_RETURN を復元するので、FP 文脈なしで正しく再開する。
struct call_frame_hdr_t {
  uint32_t prev;             // 一つ外側の call_frame_hdr_t のアドレス (0 = 最外)
  uint32_t total_bytes;      // 退避域の総バイト数 (8B 境界に丸め済み)
  uint32_t frame_bytes;      // 例外フレーム実サイズ (32 / 104)
  uint32_t caller_object_id;
  uint32_t control;
  uint32_t exc_return;
  uint32_t has_fp;           // 1 = 直後の fp[16] が有効
  uint32_t pad;              // 8B アライン維持
  uint32_t r4_r11[8];
#if SHIZU_SVC_DELEGATION
  uint32_t in_handler;
  uint32_t inv_svc_num;
  uint32_t inv_caller_obj;
  uint32_t inv_caller_thread;
#endif
};

// スレッドごとの呼び出しスタック。実体はスレッドスタック上にあり、ここは
// 先頭アドレスと段数だけを持つ (旧: MAX_DEPTH 分の配列ポインタ)。
struct call_stack_t {
  uint32_t top;   // 最内 call_frame_hdr_t のアドレス (0 = 空)
  uint32_t depth; // METHOD_EXIT の「今のネスト数」検算に使う
  bool empty() const { return depth == 0; }
};

struct thread_t {
  // state は claim の CAS 対象なので 32bit 幅を固定する (kernel.cpp の
  // __atomic_compare_exchange_n が (uint32_t*)&state で叩く)。
  enum struct state_t : uint32_t {
    UNINITIALIZED = 0,
    READY,
    RUNNING,
    SUSPENDED,
    // grant 発行中の grantor。READY でないので他コアの claim 対象にならず、復帰は
    // 発行コア上の PendSV (期限) か SWITCH_THREAD インターセプト (早期復帰) のみ。
    WAIT_GRANT,
    // 枠を確保したが初期化がまだ終わっていない過渡状態。READY でないのでどの
    // スケジューラにも拾われない。async_call の空きスキャンが CAS でここへ遷移させ、
    // 2 コアが同じ枠を掴むのを防ぐ (それが無いと create_thread 側の二重初期化
    // 検査に引っかかって **panic = 系ごと停止**していた)。
    RESERVED,
  } state;
  // affinity: 走行を許すコアのビットマスク (bit0=core0, bit1=core1)。既定は全コア。
  // TRY_SWITCH_THREAD の claim がこれを検査する (AMP 相当は「core1 固定」等の policy)。
  uint32_t affinity;
  // grant_budget_us: スケジューラ (sched_pick_next) がこのスレッドへ CPU を渡すときの
  // 時限 [µs]。0 = 無制限 (従来の SWITCH_THREAD バトンパス)。>0 = GRANT_CPU で host
  // される (期限で必ず回収される guest)。不変条件: budget>0 のスレッドは guest として
  // しか走らないので、yield を怠っても系を凍らせられない。バトン (無限移譲) を受け
  // 取れるのは budget 0 のスレッドだけ (thread0 idle / BLE_UART / core1 組)。
  uint32_t grant_budget_us;
  // wake_at: sleep 中の起床時刻 (µs, time_us_64 基準)。スケジューラは wake_at > now の
  // スレッドをスキップする (sleep 中は round-robin から外す)。0 や過去値なら即 runnable。
  uint64_t wake_at;
  context_t *context;
  uint32_t object_id;
  uint32_t thread_id;
  call_stack_t call_stack;
};

void object_table_init();
void thread_table_init();

// ---- カーネル専用データ (kernel.cpp の静的プール、malloc 非使用) -------------
// context_t は thread_id で直接引ける固定 128 スロットプール (存在する最大
// thread_id が 128 未満なのは既に全体の不変条件)。call_stack フレームはスレッド数が
// 実行時可変なのでカーネル専用アリーナ (bump allocator, malloc フォールバック付き)
// から確保する。どちらも create_thread / 起動時ブートストラップ (thread0, core1
// idle) の 3 箇所が共通で使う — 汎用ヒープ (malloc、スレッドスタックが住む場所) と
// 物理的に分離することが目的: 汎用ヒープは .bss/.data の後 (__end__ 以降) にあり
// MPU Step0 の W^X region1 [__end__, SRAM_END) の対象。カーネル専用プールは
// __end__ より前の静的領域 (.bss) に置かれるため、Step1 で不特権オブジェクトへ
// 渡す MPU region (heap 側だけ) には最初から含まれない — リンカ配置の変更なしに
// 「カーネル簿記は不特権から触れない」を得られる (HANDOFF §6 の一方向ドアの解)。
context_t *kernel_context_for(uint32_t thread_id);

} // namespace shizu
#endif // SHIZU_KERNEL_HPP