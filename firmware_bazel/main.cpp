// XNO 側 (このリポジトリ) の bring-up ファームウェア。
//
// BLE UART (ble_uart.cpp) を組み込んだ構成。既存 CMake ビルド (main.cpp,
// kernel.cpp ...) には一切触れていない。
#include "ble_uart.hpp"
#include "bme280.hpp"
#include "bno055.hpp"
#include "flight_controller.hpp"
#include "logger.hpp"
#include "object_ids.hpp"
#include "ota.hpp"
#include "pico/stdlib.h"
#include "shizuku/app_entry.hpp"
#include "shizuku/kernel.hpp"
#include "shizuku/kernel_object.hpp"
#include "shizuku/object_api.hpp"
#include "shizuku/objects/gdb_stub.hpp"
#include "shizuku/objects/peripherals.hpp"
#include "shizuku/objects/usb_cdc.hpp"
#include "telemetry.hpp"
#include <cstdio>

namespace {

// ★GDB で止めて見せるための対象。LED を叩くだけで、何の仕事も持たない。
//   **本業のスレッド (BLE / センサ) を対象にしない**のが肝 — 止めている間
//   通信も計測もそのまま止まるので、「デバッガで止めた」のか「壊れた」のかが
//   区別できなくなる。こいつなら止めても系は何も困らないし、
//   **止まったことが LED で目に見える**。
//  ★2026-08-24: OTA (commit) の実証用に、250ms 固定から **300ms〜1500ms を
//    往復する掃引**へ替えた。狙いは「焼き替わったことが目で分かる」こと ——
//    間隔を別の**一定値**にすると、直前の像を覚えていないと区別できない。
//    速くなって遅くなってを繰り返していれば、一目で「新しい方だ」と分かる。
uintptr_t blink_main(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  using shizuku::object_api;
  shizuku::KERNEL::ARCH::syscall((uintptr_t)object_api::DECLARE_NAME,
                                 (uintptr_t)"blink");
  shizuku::objects::led_request request{0};
  // 点灯/消灯それぞれの長さ (= トグル間隔)。端に着いたら向きを変える。
  constexpr int32_t MIN_MS = 300;
  constexpr int32_t MAX_MS = 2000;
  constexpr int32_t STEP_MS = 100;
  int32_t interval_ms = MIN_MS;
  int32_t step_ms = STEP_MS;
  while (true) {
    request.value ^= 1u;
    shizuku::KERNEL::ARCH::syscall(
        (uintptr_t)object_api::CALL_METHOD, shizuku::objects::LED_OBJECT,
        (uintptr_t)shizuku::objects::led_method::WRITE, (uintptr_t)&request);
    shizuku::KERNEL::ARCH::syscall((uintptr_t)object_api::SLEEP_US,
                                   (uintptr_t)(interval_ms * 1000));
    interval_ms += step_ms;
    if (interval_ms >= MAX_MS) {
      interval_ms = MAX_MS;
      step_ms = -STEP_MS;
    } else if (interval_ms <= MIN_MS) {
      interval_ms = MIN_MS;
      step_ms = STEP_MS;
    }
  }
  return 0;
}

} // namespace

// ★切り分け用のつまみ。false にするとセンサを**登録すらしない** = I2C を
//   一度も触らない。GP6/GP7 (I2C1) へ移した直後から起動後に必ずウェッジする
//   ようになったので、「I2C 由来か、それ以外か」を分けるために置いた。
//   ウェッジが消えれば I2C 経路、消えなければ別の原因。
constexpr bool XNO_ENABLE_SENSORS = true;

void shizuku::app_entry() {
  // ★診断: シリアルが完全に無音だった原因切り分け用。ここが出ない場合は
  //   スレッドモードに到達する前 (main()/kernel_instance.init() 側) で
  //   止まっている。
  shizuku::KERNEL::BOARD::diag_printf("[XNO BOOT] app_entry start\n");
  shizuku::kernel_instance.set_recovery_thread(0);

  // ★★2026-08-24 訂正: ここには以前「start_gdb_stub() は attach まで
  //   boot thread をブロックするので付けない」と書いてあったが、**それは
  //   誤診だった**。start_gdb_stub() は中で stub を別スレッドとして spawn して
  //   即 return する — 呼び出し元は一切待たない。advertise へ到達できなかった
  //   のは別の原因 (Shizuku 側 D48 参照)。
  //   ★実際にあった問題は「待たない」ではなく「**寝ない**」ことの方だった:
  //     未接続でも stub が YIELD で回り続け、スケジューラ 1 周ごとに syscall が
  //     挟まっていた (実測で、協調的に譲るスレッドの周回数が 10.8% 落ちた)。
  //     Shizuku 側 D48 で「未接続なら SLEEP」に直したので、常時起こしておいて
  //     構わない。
  //
  // ★LED (cyw43 経由) を BLE より先に上げる。cyw43_arch_init を呼ぶのは
  //   どちらが先でもよいが、診断ログの読みやすさのため先に確定させておく。
  // ★★2 本目のコアは**ペリフェラル登録より前**に起こす (2026-08-24)。
  //   理由は cyw43 ではなく **flash**: `cyw43_arch_init()` は SDK の
  //   `btstack_cyw43_init()` → `setup_tlv()` の中で bonding 用 TLV の
  //   flash bank を初期化し、必要なら erase/write する。その書き込みは
  //   `flash_safe_execute()` を通り、これは「**もう片方のコアが lockout
  //   victim として登録済み**」を要求する (pico_flash/flash.c:184)。
  //   core1 を後から起こしていると、この時点で未登録なので
  //   `assert(false)` (flash.c:190) で停止する — しかも壊れた状態は flash に
  //   残るため、電源再投入でもアプリの焼き直しでも直らず
  //   `picotool erase --all` でしか戻らない (実測で 1 回踏んだ)。
  //   Shizuku の core1 は起動時に `multicore_lockout_victim_init()` を
  //   呼ぶ (board.cpp) ので、先に起こしておけば前提が満たされる。
  //   ★cyw43 を「初期化したコアからしか触れない」制約とは無関係 —
  //     こちらは core1 を**走らせるだけ**で、cyw43 には触れない。
  if (shizuku::kernel_object_instance.start_secondary_core())
    shizuku::KERNEL::BOARD::diag_printf("[XNO BOOT] secondary core launched\n");

  shizuku::objects::register_peripherals();
  shizuku::KERNEL::BOARD::diag_printf("[XNO BOOT] peripherals registered\n");
  // ★flash_fs は今回外した (2026-08-24)。bonding 永続化 (ble_uart.cpp) を
  //   公式 SDK の pico_flash_bank_instance() の既定オフセット (flash 末尾)
  //   でオフセット上書き無しに使うため、flash_fs が末尾 1MB を占有している
  //   状態と両立できない。ログ用途 (Phase E) は bonding 永続化の決着後に
  //   再検討する。詳細は ble_uart.cpp 冒頭コメント / plan の Phase C.6。

  // ---- 1) 登録 (生成 + export + 自分のストリーム作成)。まだ誰も走らせない
  // ---- ★「作る」と「走らせる」を分けてあるのは、間に**配線**が要るから。
  //   ストリームの席は 1 つずつしか無い (SPSC) ので、番号を配ってから
  //   poll スレッドを起こさないと、走り出した側が繋がっていない路を掴む。
  ble_uart::register_ble_uart();
  if (XNO_ENABLE_SENSORS) {
    bno055::register_bno055();
    bme280::register_bme280();
  }
  telemetry::register_telemetry();
  flight_controller::register_flight_controller();
  logger::register_logger();
  ota::register_ota();
  shizuku::KERNEL::BOARD::diag_printf("[XNO BOOT] objects registered\n");

  // ---- 2) 配線。合成はここでしか書かない ------------------------------------
  //   役はひとつずつに割ってある (UNIX 哲学。ストリームがパイプの役をする):
  //
  //     bno055 ──samples──┐
  //                        ├→ [telemetry] ──lines──┐
  //     bme280 ──samples──┘                         ├→ [logger] ─→ [ble_uart] →
  //     BLE
  //                                                 │
  //     ble_uart ──rx──→ [flight_controller] ──lines┘
  //
  //   ★どのリンクも producer 1 / consumer 1 のまま (D46)。センサは BLE を
  //     知らず、ble_uart は誰が送ってきたかを知らない。合流の方針は logger、
  //     符号化は telemetry、判断は flight_controller の中だけにある。
  {
    using shizuku::object_api;
    const auto call = [](uintptr_t object, uintptr_t m, uintptr_t a) {
      return shizuku::KERNEL::ARCH::syscall((uintptr_t)object_api::CALL_METHOD,
                                            object, m, a);
    };
    // センサ → telemetry
    shizuku::KERNEL::ARCH::syscall_result bno_stream{}, bme_stream{};
    if (XNO_ENABLE_SENSORS) {
      bno_stream =
          call(bno055::OBJECT, (uintptr_t)bno055::method::GET_STREAM, 0);
      bme_stream =
          call(bme280::OBJECT, (uintptr_t)bme280::method::GET_STREAM, 0);
      call(telemetry::OBJECT, (uintptr_t)telemetry::method::ADD_INPUT,
           bno_stream.value);
      call(telemetry::OBJECT, (uintptr_t)telemetry::method::ADD_INPUT,
           bme_stream.value);
    }

    // ble_uart RX → flight_controller
    const auto rx_stream =
        call(ble_uart::OBJECT, (uintptr_t)ble_uart::method::GET_RX_STREAM, 0);
    call(flight_controller::OBJECT,
         (uintptr_t)flight_controller::method::SET_RX_STREAM, rx_stream.value);

    // telemetry / flight_controller → logger。★応答を先に出す (対話の待ち時間が
    //   一番目立つ)。テレメトリは量が多いので後ろ。
    const auto telem_lines =
        call(telemetry::OBJECT, (uintptr_t)telemetry::method::GET_STREAM, 0);
    const auto reply_lines =
        call(flight_controller::OBJECT,
             (uintptr_t)flight_controller::method::GET_STREAM, 0);
    call(logger::OBJECT, (uintptr_t)logger::method::ADD_INPUT,
         logger::pack_input(reply_lines.value, logger::PRIO_CONTROL));
    call(logger::OBJECT, (uintptr_t)logger::method::ADD_INPUT,
         logger::pack_input(telem_lines.value, logger::PRIO_BULK));

    // OTA: ble_uart の専用 characteristic → ota、ota の進捗行 → logger
    const auto ota_lines =
        call(ota::OBJECT, (uintptr_t)ota::method::GET_STREAM, 0);
    call(logger::OBJECT, (uintptr_t)logger::method::ADD_INPUT,
         logger::pack_input(ota_lines.value, logger::PRIO_CONTROL));

    // logger → ble_uart
    const auto out_stream =
        call(logger::OBJECT, (uintptr_t)logger::method::GET_STREAM, 0);
    call(ble_uart::OBJECT, (uintptr_t)ble_uart::method::SET_TX_STREAM,
         out_stream.value);

    // OTA 受信: **ble_uart が producer / ota が consumer** (RX と同じ向き)。
    const auto ota_in =
        call(ble_uart::OBJECT, (uintptr_t)ble_uart::method::GET_OTA_STREAM, 0);
    call(ota::OBJECT, (uintptr_t)ota::method::SET_INPUT_STREAM, ota_in.value);

    shizuku::KERNEL::BOARD::diag_printf(
        "[XNO BOOT] wired: bno=%lu bme=%lu rx=%lu telem=%lu reply=%lu "
        "out=%lu\n",
        (unsigned long)bno_stream.value, (unsigned long)bme_stream.value,
        (unsigned long)rx_stream.value, (unsigned long)telem_lines.value,
        (unsigned long)reply_lines.value, (unsigned long)out_stream.value);
  }

  // ---- 2.5) デバッグ対象 + GDB stub (★起動より前)
  // ------------------------------------------ ★★ここは **start_*
  // より前**でなければならない。ble_uart の poll スレッドは
  //   起動直後に GDB リンクの席を確保しに行くので、番号を渡す前に走らせると
  //   「配線されていない」と判断してそのまま回り続ける (実際それで GDB の
  //   characteristic が繋がらなかった)。
  // ★blink を先に起こしてから、そのスレッドを対象として stub を起こす。
  //   ★LED は cyw43 経由 = **初期化したコアからしか触れない**ので core0 に
  //     固定する (Shizuku 側で実測済みの制約。外すと core1 へ移った瞬間に
  //     pico-sdk の assert でそのスレッドが止まる)。
  {
    using shizuku::object_api;
    const auto sys = [](uintptr_t n, uintptr_t a1 = 0, uintptr_t a2 = 0,
                        uintptr_t a3 = 0) {
      return shizuku::KERNEL::ARCH::syscall(n, a1, a2, a3);
    };
    const auto call = [](uintptr_t object, uintptr_t m, uintptr_t a) {
      return shizuku::KERNEL::ARCH::syscall((uintptr_t)object_api::CALL_METHOD,
                                            object, m, a);
    };
    sys((uintptr_t)object_api::CREATE_OBJECT, xno_object_id::blink,
        (uintptr_t)&blink_main, shizuku::OBJECT_ON_CORE(0));
    const auto blink_thread =
        sys((uintptr_t)object_api::SPAWN, xno_object_id::blink, 0, 0);
    if (blink_thread.error != 0) {
      shizuku::KERNEL::BOARD::diag_printf(
          "[XNO BOOT] could not spawn blink (%lu); gdb stub not started\n",
          (unsigned long)blink_thread.error);
    } else {
      // ★★GDB を **BLE で** 運ぶ。CDC 直ではなくストリーム経由の stub を起こし
      //   (Shizuku 側 start_gdb_stub_over_stream)、その 2 本を ble_uart の
      //   GDB 専用 characteristic に繋ぐ。
      //   ★NUS (テレメトリ) とは別チャネル — RSP は `$...#xx` の枠付きバイト列
      //     なので、CSV 行が混ざると握手ごと壊れる (ble_uart.gatt のコメント)。
      //   ★開くのは**認可済みリンクのときだけ** — GDB は任意のメモリ読み書きと
      //     レジスタ操作そのもの。ble_uart 側で fail-closed にしてある。
      const auto link = shizuku::objects::start_gdb_stub_over_stream(
          (uint32_t)blink_thread.value);
      if (link.ok) {
        call(ble_uart::OBJECT, (uintptr_t)ble_uart::method::SET_GDB_STREAMS,
             (link.to_stub << 16) | link.from_stub);
        shizuku::KERNEL::BOARD::diag_printf(
            "[XNO BOOT] gdb over BLE (streams %lu/%lu), watching blink "
            "(thread %lu)\n",
            (unsigned long)link.to_stub, (unsigned long)link.from_stub,
            (unsigned long)blink_thread.value);
      } else {
        shizuku::KERNEL::BOARD::diag_printf(
            "[XNO BOOT] gdb link streams failed; stub not started\n");
      }
    }
  }

  // ---- 3) 起動 (poll スレッドを起こす) --------------------------------------
  ble_uart::start_ble_uart();
  logger::start_logger();
  ota::start_ota();
  telemetry::start_telemetry();
  flight_controller::start_flight_controller();
  if (XNO_ENABLE_SENSORS) {
    bno055::start_bno055();
    bme280::start_bme280();
  }
  shizuku::KERNEL::BOARD::diag_printf("[XNO BOOT] all threads started\n");

  shizuku::KERNEL::BOARD::diag_printf(
      "[XNO BOOT] flight_robocon xno bring-up ok\n");

  while (true)
    shizuku::KERNEL::ARCH::syscall((uintptr_t)shizuku::object_api::YIELD);
}

int main() {
  shizuku::objects::usb_cdc_init();
  sleep_ms(1000);
  // ★ホストが CDC を開くまでの猶予。診断出力は**未接続なら捨てられる**
  //   (usb_cdc.cpp の diag_out_chars: 「溢れたら捨てる。診断のために本業を
  //   止めない」)。焼いた直後は列挙が終わるまで数秒かかるので、1 秒では
  //   起動バナーがまるごと落ちる — 実際それで「無音になった」と誤診した。
  for (int i = 0; i < 10; ++i) {
    printf("[XNO] waiting for the host to open CDC (%d)\n", i);
    sleep_ms(200);
  }
  shizuku::kernel_instance.init();
  shizuku::kernel_object_instance.init();
  shizuku::kernel_instance.set_object_handler(
      shizuku::KERNEL_OBJECT::handler_entry());
  const auto boot = shizuku::kernel_object_instance.lend_boot_stack();
  shizuku::kernel_instance.bootstrap(shizuku::app_entry, boot.base, boot.bytes);
}
