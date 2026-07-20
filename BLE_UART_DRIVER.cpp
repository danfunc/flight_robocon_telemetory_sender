#include "btstack_config.h"
extern "C" {
#include "ble/att_db.h"
#include "ble/att_server.h"
#include "btstack.h"
#include "gap.h"
#include "hci.h"
#include "pico/stdlib.h" // getchar_timeout_us (USB CDC 経由の confirm 入力)
}
#include "ble_uart.h" // pico_btstack_make_gatt_header() が生成
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ble_tx_stream.hpp> // 優先度付き TX マルチストリーム (frame_t / push_msg)
#include <export_method.hpp>
#include <obj_api.hpp>
#include <hardware/sync.h>   // save_and_disable_interrupts / restore_interrupts
#include <object_headers/BLE_UART_DRIVER.hpp>
#include <stream.hpp> // open / bind (register_tx_stream)
#include <pico/cyw43_arch.h> // cyw43_arch_init / cyw43_arch_poll

namespace shizu {

// ---- セキュリティ設定 ------------------------------------------------------
// LE Secure Connections + Numeric Comparison。Pico は画面を持たないが USB CDC
// (シリアルコンソール)をオペレータ I/O として使い、表示=printf、確認=getchar
// で human-in-the-loop の番号照合を成立させる。固定/転記式の passkey を持たない
// ので焼き込み秘密が無く、再ペアリング時も y/n 一発で済む。
//
// NC 確認待ちのコネクション(なければ INVALID)。poll ループが USB CDC を覗いて
// オペレータの y/n でこれを confirm/decline する。
static hci_con_handle_t nc_pending_handle = HCI_CON_HANDLE_INVALID;

// ---- 接続 / 通知状態 -------------------------------------------------------
static hci_con_handle_t con_handle = HCI_CON_HANDLE_INVALID;
static bool tx_notify_enabled = false;
static bool can_send_requested = false;
// 最新の Connection Interval (単位 1.25ms)。接続時/接続更新時に捕捉する。
static uint16_t conn_interval = 0;

// (B) ケーパビリティビット。A(SM)がリンクを暗号化+passkey 認証で固めた
// 結果をここに反映する。これが立っていない限り命令は sink へ流さない
// (fail-closed)。独自の秘密は持たず、SM の判定を読むだけ。
static bool cmd_authorized = false;

// ---- 接続ウォッチドッグ (修正版で復活) --------------------------------------
// DISCONNECTION_COMPLETE は取りこぼすことがある (センサ障害等で長時間ストール
// した間に HCI イベントが溢れるケースをビーコン計測で実測)。取りこぼすと
// con_handle が幽霊のまま残り、再アドバタイズ経路が二度と走らない=母艦から
// 再接続できない。そこで「Central の生存兆候 (接続確立/認可/CAN_SEND_NOW/
// RSSI イベント/RX 書込)」が一定時間途絶えたらローカル状態を畳み、下の広告保険
// で connectable へ戻す。健全な通信中は notify のたびに CAN_SEND_NOW が、毎秒
// RSSI イベントが来るので誤発火しない。
// ※ 旧版ウォッチドッグが「再接続不能を招いた」ように見えたのは、当時は
//    センサ起因でシステム全体が凍結しており、再アドバタイズ自体が実行されて
//    いなかったため。凍結修正後の現在はこの機構が正しく機能する。
static constexpr uint64_t CONN_IDLE_TIMEOUT_US = 5000000ull;     // 認可+notify 中: 5s
static constexpr uint64_t CONN_PAIRING_TIMEOUT_US = 30000000ull; // それ以外: 30s
static uint64_t last_conn_activity_us = 0;
static inline void note_conn_activity() { last_conn_activity_us = time_us_64(); }

// ---- ウォッチドッグのエスカレーション (BT 電源再投入) -----------------------
// 上の掃除 (gap_disconnect + 広告保険) はコントローラが生きている前提の復旧。
// 実測した重い故障モードでは HCI イベントの流れごと死んでおり (RSSI コマンドの
// 応答すら来ない)、HCI コマンドを積んでも送信されないため掃除では復帰しない。
// 判定: 掃除直後の gap_disconnect / 広告 enable に対して、生きたコントローラは
// 必ず応答イベント (COMMAND_COMPLETE 等 → ble_dbg_evt が進む) を返す。
// WD_RECOVERY_TIMEOUT_US 待ってもイベントがゼロのままなら無応答と断定し、
// BT コアを電源 OFF → FW 再ダウンロード → ON で丸ごと入れ直す
// (ボード再起動と同等の効果を BT だけに適用)。
static bool wd_recovery_pending = false;  // 掃除後、復旧確認中
static uint64_t wd_cleanup_us = 0;        // 掃除した時刻
static uint32_t wd_evt_snapshot = 0;      // 掃除時点のイベント数
static constexpr uint64_t WD_RECOVERY_TIMEOUT_US = 10000000ull; // 10s

// ---- 生存カウンタ (ビーコン診断用。main.cpp の [BEACON] が extern 参照) ------
volatile uint32_t ble_dbg_poll = 0; // poll ループの回転数
volatile uint32_t ble_dbg_evt = 0;  // HCI/SM イベント受信数

// ---- 受信バイトの配送先 (sink) --------------------------------------------
// set_rx_sink で設定。0xFFFF は無効。
static uint32_t rx_sink_obj_id = 0xFFFF;
static uint32_t rx_sink_method_id = 0;

// ---- TX 優先度付きマルチストリーム -----------------------------------------
// 単一 FIFO バイトリングを廃し、Shizuku ストリームを N 本、優先度順に排出する
// (詳細は ble_tx_stream.hpp)。各 producer が frame_t の SPSC ストリームを所有し、
// register_tx_stream で登録する。CAN_SEND_NOW ごとに flush_tx が空でない最高優先度
// ストリームから notify する。SPSC の wr/rd は協調原子性 + __dmb で扱う (btstack
// 呼び出しはロック外)。これにより ping 応答(ctrl)がバルクテレメトリを追い越せる。
namespace bt = shizu::ble_tx;

namespace ble_tx { // ctrl レイテンシ計装の実体 (このファイルは namespace shizu 内。
                   // 宣言は ble_tx_stream.hpp)
volatile uint64_t ctrl_enq_us = 0;
volatile uint32_t ctrl_lat_last_us = 0;
volatile uint32_t ctrl_lat_max_us = 0;
} // namespace ble_tx

// 1 CAN_SEND_NOW で連続排出する bulk PDU の上限。コントローラへ渡した PDU は FIFO
// なので、bulk を無制限に連射すると後着の ctrl フレームが次 CI 以降へ押される。
// bulk をイベントあたり数発に締めることで後着 ctrl の待ちを ~1 CI に抑える (実測で調整)。
#ifndef BLE_TX_BULK_PDUS_PER_EVENT
#define BLE_TX_BULK_PDUS_PER_EVENT 4
#endif

// BLE_UART 自身の内部行 (RSSI / status / 互換 send_byte・send_buf) 用のローカル
// ストリーム。外部 producer (TELEMETRY 等) は自分のストリームを register_tx_stream で
// 登録する。全ストリーム LOSSLESS = 満杯で「メッセージ丸ごと破棄」(部分フレームを
// 絶対に出さない → MORE フレーミングが壊れない)。
static shizu::stream::storage<bt::frame_t, 16, shizu::stream::LOSSLESS> g_tx_sys;

// 排出対象ストリーム表 (未ソート・追記専用)。ローカル sys + register_tx_stream の
// 外部登録。register_tx_stream は**呼び出し元スレッドのコア**で走る (BLE on core1 の
// とき TELEMETRY=core0 から) ため、core1 の poll ループの走査と並行し得る。既存要素を
// シフトしない (index 不変) + tx_src_n を最後に publish、で torn 読みを塞ぐ。優先度は
// 排出側が毎回 argmax で選ぶ (要素数 ≤8 なので走査コストは無視できる)。
struct tx_src_t {
  bt::handle_t h;
  uint32_t prio;
};
static tx_src_t tx_srcs[8];
static volatile uint32_t tx_src_n = 0;
// メッセージ途中のソース index。MORE 中はここを保持し、CAN_SEND_NOW を跨いでも同一
// ストリームを継続排出する (他ストリームの割り込み禁止 = host のバイトパーサ保護)。
static int tx_cur_src = -1;
static uint32_t dropped_msgs = 0; // 容量不足で丸ごと捨てたメッセージ数 (デバッグ用)

// 末尾に完全に書いてから tx_src_n を publish する (旧: 挿入ソート — 既存要素の
// シフトが走査と並行すると torn handle 読みになるため廃止)。
static void tx_src_insert(bt::handle_t h, uint32_t prio) {
  const uint32_t n = tx_src_n;
  if (n >= (sizeof(tx_srcs) / sizeof(tx_srcs[0])))
    return;
  tx_srcs[n] = {h, prio};
  __dmb(); // エントリ本体の書き込みを publish (tx_src_n) より先に完了させる
  tx_src_n = n + 1;
}

// 登録ストリームが全て空か (poll ループの can_send 補填判定用)。
static bool tx_all_empty() {
  for (uint32_t i = 0; i < tx_src_n; ++i)
    if (!tx_srcs[i].h.empty())
      return false;
  return true;
}

// 内部生成の 1 行 (例: "RSSI=-55\n") を sys ストリームへ (丸ごと破棄型)。
static void tx_emit_line(const char *s) {
  if (!bt::push_msg(g_tx_sys.hdl(), (const uint8_t *)s, (uint32_t)strlen(s)))
    dropped_msgs++;
}

// ---- 広告データ ------------------------------------------------------------
// flags + complete local name のみ (31 byte 以内に収める)。
// 128bit UUID を載せたい場合は scan response 側へ。
static uint8_t adv_buffer[31];
static uint8_t adv_len = 0;

static void build_adv_data() {
  uint8_t *p = adv_buffer;
  // Flags
  *p++ = 0x02;
  *p++ = BLUETOOTH_DATA_TYPE_FLAGS;
  *p++ = 0x06;
  // Complete Local Name "Shizuku UART"
  static const char name[] = "Shizuku UART";
  uint8_t name_len = (uint8_t)strlen(name);
  *p++ = name_len + 1;
  *p++ = BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME;
  memcpy(p, name, name_len);
  p += name_len;
  adv_len = (uint8_t)(p - adv_buffer);
}

static btstack_packet_callback_registration_t hci_event_callback_registration;
static btstack_packet_callback_registration_t sm_event_callback_registration;
// CONNECTION_PARAMETER_UPDATE_RESPONSE は l2cap のイベントハンドラにのみ
// 配送される (l2cap_emit_event は l2cap_event_handlers を回すだけ) ため、
// hci とは別に l2cap にも packet_handler を登録する。
static btstack_packet_callback_registration_t l2cap_event_callback_registration;

// ---- 受信処理 --------------------------------------------------------------
static void process_rx(const uint8_t *data, uint16_t len) {
  // (B) 認可されていないコネクションからの命令は捨てる。RX characteristic
  // 自体も ATT 層で AUTHENTICATION_REQUIRED
  // により保護されている(A)ので、ここに到達する
  // 時点で本来は認証済みのはず。二重に fail-closed で確認する。
  if (!cmd_authorized) {
    printf("[BLE_UART] RX dropped (unauthorized, %u byte)\n", len);
    return;
  }
  for (uint16_t i = 0; i < len; i++) {
    uint8_t b = data[i];
    if (rx_sink_obj_id != 0xFFFF) {
      // sink オブジェクトへ 1 バイトずつ配送。
      shizu::obj_api::svc(shizu::obj_api::svc_num::CALL_METHOD, rx_sink_obj_id,
                          rx_sink_method_id, b);
    }
  }
  // デバッグ出力
  printf("[BLE_UART] RX %u byte: ", len);
  for (uint16_t i = 0; i < len; i++)
    printf("%02x ", data[i]);
  printf("\n");
}

// ---- TX 統計 (A/B 計測用: 1 CAN_SEND_NOW あたり何発詰めたか) ----------------
static uint32_t tx_stat_pkts = 0;  // 直近窓の notify 送信数
static uint32_t tx_stat_csn = 0;   // 直近窓の CAN_SEND_NOW イベント数
static uint32_t tx_stat_bytes = 0; // 直近窓の送信バイト数

// ---- TX flush --------------------------------------------------------------
// 1 回の CAN_SEND_NOW で notification を 1 発だけ送ると、CI (15ms) あたり
// 1 パケットに律速され ~130kbps が理論上限になる。コントローラの ACL バッファ
// クレジットがある限り att_server_notify は連続で成功する
// (att_server_can_send_packet → hci の free ACL slots 判定。尽きると
// BTSTACK_ACL_BUFFERS_FULL が返る) ので、「失敗するか送るものが無くなるまで」
// 詰めて 1 CI に複数 LL PDU を載せる。
static void flush_tx() {
  if (con_handle == HCI_CON_HANDLE_INVALID || !tx_notify_enabled)
    return;
  if (!cmd_authorized)
    return;

  // 1 frame ≤244B = ちょうど 1 LL PDU (producer 側で分割済み。★CYW43 wedge 回避の
  // 不可侵規則★)。クレジットが尽きる (att_server_notify が BUFFERS_FULL) か送るものが
  // 無くなるまで連射しつつ、以下 3 つを守る:
  //   ・優先度: メッセージ境界では空でない最高優先度ストリームを選ぶ。
  //   ・MORE/切替禁止: メッセージ途中 (tx_cur_src>=0) は同一ストリームを継続。
  //   ・in-flight 上限: bulk は 1 イベント数発でメッセージ境界打ち切り (後着 ctrl 保護)。
  uint32_t bulk_pdus = 0;
  while (true) {
    int src = tx_cur_src;
    if (src < 0) { // メッセージ境界: 空でない最高優先度を選ぶ (表は未ソート → argmax)
      uint32_t best_prio = 0;
      const uint32_t n = tx_src_n;
      for (uint32_t i = 0; i < n; ++i) {
        if (!tx_srcs[i].h.empty() && (src < 0 || tx_srcs[i].prio > best_prio)) {
          src = (int)i;
          best_prio = tx_srcs[i].prio;
        }
      }
      if (src < 0)
        break; // 全ストリーム空
    }
    // 覗くだけ (rd は進めない)。notify 成功後に drop() で消費確定 = credit 切れで
    // 送れなかったフレームを失わない。
    bt::frame_t f;
    if (!tx_srcs[src].h.peek(&f)) {
      tx_cur_src = -1; // 競合で空になった (通常起きない) → 選び直し
      continue;
    }
    const uint16_t n = f.len & bt::LEN_MASK;
    const bool more = (f.len & bt::MORE) != 0;
    int err = att_server_notify(
        con_handle,
        ATT_CHARACTERISTIC_6E400003_B5A3_F393_E0A9_E50E24DCCA9E_01_VALUE_HANDLE,
        f.data, n);
    if (err != 0)
      break; // クレジット切れ。drop していないので次機会に再送 (tx_cur_src 保持)。
    tx_srcs[src].h.drop();
    ++tx_stat_pkts;
    tx_stat_bytes += n;
    // ctrl レイテンシ計装: ctrl 優先度のフレームを notify に載せた時点で
    // 「push からの device 内滞在時間」を確定する (ping >100ms の切り分け用)。
    if (tx_srcs[src].prio >= bt::PRIO_CTRL && bt::ctrl_enq_us != 0) {
      uint32_t lat = (uint32_t)(time_us_64() - bt::ctrl_enq_us);
      bt::ctrl_enq_us = 0;
      bt::ctrl_lat_last_us = lat;
      if (lat > bt::ctrl_lat_max_us)
        bt::ctrl_lat_max_us = lat;
    }
    tx_cur_src = more ? src : -1; // MORE 中は同一ソース継続、完了で境界へ
    // in-flight 上限: bulk はメッセージ境界でのみ打ち切る (メッセージ分割はしない)。
    if (tx_srcs[src].prio <= bt::PRIO_BULK) {
      if (++bulk_pdus >= BLE_TX_BULK_PDUS_PER_EVENT && !more)
        break;
    }
  }

  // まだ残っていれば次の送信機会を要求。
  if (!tx_all_empty()) {
    can_send_requested = true;
    att_server_request_can_send_now_event(con_handle);
  }
}

// ---- ATT read callback -----------------------------------------------------
// TX は notify 専用、RX も値読み出しは無し。0 を返すだけ。
static uint16_t att_read_callback(hci_con_handle_t connection_handle,
                                  uint16_t att_handle, uint16_t offset,
                                  uint8_t *buffer, uint16_t buffer_size) {
  (void)connection_handle;
  (void)att_handle;
  (void)offset;
  (void)buffer;
  (void)buffer_size;
  return 0;
}

// ---- ATT write callback ----------------------------------------------------
static int att_write_callback(hci_con_handle_t connection_handle,
                              uint16_t att_handle, uint16_t transaction_mode,
                              uint16_t offset, uint8_t *buffer,
                              uint16_t buffer_size) {
  (void)transaction_mode;
  (void)offset;

  // TX characteristic の CCC への書き込み = notify enable/disable
  if (att_handle ==
      ATT_CHARACTERISTIC_6E400003_B5A3_F393_E0A9_E50E24DCCA9E_01_CLIENT_CONFIGURATION_HANDLE) {
    tx_notify_enabled =
        (little_endian_read_16(buffer, 0) ==
         GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION);
    con_handle = connection_handle;
    printf("[BLE_UART] notify %s\n",
           tx_notify_enabled ? "enabled" : "disabled");
    if (tx_notify_enabled && !tx_all_empty() && !can_send_requested) {
      can_send_requested = true;
      att_server_request_can_send_now_event(con_handle);
    }
    return 0;
  }

  // RX characteristic の値への書き込み = Central からの受信データ
  if (att_handle ==
      ATT_CHARACTERISTIC_6E400002_B5A3_F393_E0A9_E50E24DCCA9E_01_VALUE_HANDLE) {
    note_conn_activity(); // Central からの書込 = 生存の兆候
    process_rx(buffer, buffer_size);
    return 0;
  }

  return 0;
}

// Connection Interval を表示する。単位 1.25ms を整数演算で ms へ変換
// (%f に依存しない)。
static void print_conn_interval(const char *tag) {
  uint32_t x100 = (uint32_t)conn_interval * 125u; // 1.25ms = 125/100
  printf("[BLE_UART] %s CI = %u units (%lu.%02lu ms)\n", tag, conn_interval,
         (unsigned long)(x100 / 100), (unsigned long)(x100 % 100));
}

// 接続インターバルを速める要求。min=FORCED_CI, max=2*FORCED_CI, latency=0,
// supervision timeout=400 (4000ms; 制約 timeout > 2*(1+latency)*CI を満たす)。
// ★ Apple/macOS は「接続インターバル最小 15ms」を要求し、これを下回る値を要求すると
//   接続を数秒で強制切断する。以前の 6(=7.5ms) が macOS 側の 2 秒切断の原因だった。
//   12 units = 15.00ms が Apple 準拠の最速。iOS/macOS 相手はこれ以上速くできない。
static constexpr uint16_t FORCED_CI = 12;

// 接続中のリンク RSSI を周期的にサンプリングする間隔 (ms)。poll ループが
// gap_read_rssi を投げ、結果は GAP_EVENT_RSSI_MEASUREMENT で返り、"RSSI=<dBm>"
// 行として notify でホストへ流す。母艦側も自分の RSSI を測るので双方が並ぶ。
static constexpr uint32_t RSSI_PERIOD_MS = 1000;

// ---- CI ネゴシエーションの状態機械 ------------------------------------------
// 実測: (min=12, max=24) で要求すると macOS は max 側の 24 (30ms) を選ぶ。
// そこで 1 回目は min=max=12 (15ms 丁度 = Apple の床) をピン留めして要求する。
// ただし Apple の旧アクセサリガイドラインに「max ≥ min+15ms」の推奨があり、
// min=max が拒否される可能性もゼロではないため、
//   ・L2CAP の CONNECTION_PARAMETER_UPDATE_RESPONSE が reject だった
//   ・または updated CI が 12 にならなかった
// 場合に限り、1 回だけ (12, 24) へフォールバック再要求する。それ以上は追わない
// (無限再要求ループ = macOS との要求合戦は禁止)。
//   0 = 未要求 / 1 = (12,12) 要求済み / 2 = フォールバック (12,24) 要求済み (終端)
static uint8_t ci_nego_stage = 0;

static void request_fast_ci() {
  if (con_handle == HCI_CON_HANDLE_INVALID)
    return;
  ci_nego_stage = 1;
  int r = gap_request_connection_parameter_update(con_handle, FORCED_CI,
                                                  FORCED_CI, 0, 400);
  uint32_t ms100 = (uint32_t)FORCED_CI * 125u; // 単位 1.25ms → ms×100
  printf("[BLE_UART] request CI pinned %lu.%02lu ms -> %d\n",
         (unsigned long)(ms100 / 100), (unsigned long)(ms100 % 100), r);
}

// min=max ピン留めが通らなかったときの一度きりのフォールバック (12, 24)。
static void request_ci_fallback(const char *why) {
  if (con_handle == HCI_CON_HANDLE_INVALID || ci_nego_stage != 1)
    return;
  ci_nego_stage = 2; // 終端: これ以上再要求しない
  int r = gap_request_connection_parameter_update(con_handle, FORCED_CI,
                                                  2 * FORCED_CI, 0, 400);
  printf("[BLE_UART] CI fallback (12,24) [%s] -> %d\n", why, r);
}

// ---- 2M PHY / LE features 診断 ----------------------------------------------
// 実測で `LE features:` も `LE Set PHY command status` も出なかった件の対策:
//  (a) hci_send_cmd はコマンドクレジットが無いと黙って捨てる (log_error のみ、
//      戻り値 ERROR_CODE_COMMAND_DISALLOWED)。HCI ready 直後は btstack 自身の
//      コマンドと競合しやすい → 送信は poll ループへ移し、
//      hci_can_send_command_packet_now() を確認してから送る (成るまで再試行)。
//  (b) 旧 FW のコントローラは未知オペコードに Command Status ではなく
//      Command Complete (Unknown HCI Command) で応える場合がある →
//      COMMAND_COMPLETE 側でも LE Set PHY オペコードを拾う。
//  (c) 最終手段: request_2m_phy から 5 秒以内に PHY_UPDATE_COMPLETE が来なければ
//      「no response (likely unsupported)」を 1 回だけログ (poll ループで監視)。
static bool le_features_pending = false; // 読み出しコマンドの送信待ち
static bool phy_probe_active = false;    // PHY update 応答待ち
static uint64_t phy_req_us = 0;
static constexpr uint64_t PHY_PROBE_TIMEOUT_US = 5000000ull;

// 2M PHY を要求する。CYW43439 の BT FW が 2M PHY 対応かは要実測確認。
// 2M 化で 1 LL PDU の空中時間が半減し、同じ CI (15ms) により多くの PDU を
// 詰められる。gap_le_set_phy はコネクションにキューされ hci_run が送る
// (この時点では捨てられない)。適用可否は HCI_SUBEVENT_LE_PHY_UPDATE_COMPLETE
// / Command Status / 5s タイムアウトの 3 経路のどれかで必ずログに出る。
// tx_phys/rx_phys: bit0=1M, bit1=2M, bit2=Coded。all_phys=0 = 両方向とも希望を指定。
static void request_2m_phy() {
  if (con_handle == HCI_CON_HANDLE_INVALID)
    return;
  uint8_t r = gap_le_set_phy(con_handle, 0, 0x02, 0x02, 0);
  phy_probe_active = true;
  phy_req_us = time_us_64();
  printf("[BLE_UART] request 2M PHY -> %u (watching for PHY update, 5s)\n", r);
}

// ---- HCI / ATT イベント ----------------------------------------------------
static void packet_handler(uint8_t packet_type, uint16_t channel,
                           uint8_t *packet, uint16_t size) {
  (void)channel;
  (void)size;
  if (packet_type != HCI_EVENT_PACKET)
    return;
  ble_dbg_evt = ble_dbg_evt + 1;

  uint8_t event = hci_event_packet_get_type(packet);
  switch (event) {

  case BTSTACK_EVENT_STATE:
    if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
      printf("[BLE_UART] HCI ready, advertising start\n");
      gap_advertisements_enable(1);
      // 2M PHY サポートの白黒付け: ローカル LE features を読む。ここで直接
      // hci_send_cmd すると btstack 自身のコマンドとクレジット競合して黙って
      // 捨てられる (実測でログ不発の原因) ため、poll ループから
      // hci_can_send_command_packet_now() を確認して送る。
      le_features_pending = true;
    }
    break;

  case HCI_EVENT_COMMAND_COMPLETE: {
    uint16_t cc_opcode = hci_event_command_complete_get_command_opcode(packet);
    if (cc_opcode == HCI_OPCODE_HCI_LE_READ_LOCAL_SUPPORTED_FEATURES) {
      // return params: [0]=status, [1..8]=LE features (LSB first)。
      // LE 2M PHY = feature bit 8 = byte1 の bit0。
      const uint8_t *rp = hci_event_command_complete_get_return_parameters(packet);
      printf("[BLE_UART] LE features: status=0x%02x 2M_PHY=%s "
             "(bytes[0..1]=%02x %02x)\n",
             rp[0], (rp[2] & 0x01) ? "supported" : "NOT supported", rp[1],
             rp[2]);
    } else if (cc_opcode == HCI_OPCODE_HCI_LE_SET_PHY) {
      // 旧 FW は未知オペコードに Command Complete (status=0x01 Unknown HCI
      // Command) で応えることがある。ここに来たら 2M PHY 非対応がほぼ確定。
      const uint8_t *rp = hci_event_command_complete_get_return_parameters(packet);
      printf("[BLE_UART] LE Set PHY answered via Command Complete: "
             "status=0x%02x (0x01 = Unknown HCI Command → 2M PHY unsupported)\n",
             rp[0]);
      phy_probe_active = false;
    }
    break;
  }

  case HCI_EVENT_COMMAND_STATUS: {
    // LE Set PHY はまず Command Status が返る (0x00=実行中 → 後で
    // PHY_UPDATE_COMPLETE)。0x11=Unsupported Feature or Parameter 等なら
    // その場で棄却されており、PHY_UPDATE_COMPLETE は永遠に来ない。
    uint16_t opcode = hci_event_command_status_get_command_opcode(packet);
    if (opcode == HCI_OPCODE_HCI_LE_SET_PHY) {
      uint8_t st = hci_event_command_status_get_status(packet);
      printf("[BLE_UART] LE Set PHY command status = 0x%02x%s\n", st,
             st == 0 ? " (pending, wait for PHY update)" : " (REJECTED)");
      if (st != 0)
        phy_probe_active = false; // 棄却 = 応答済み。タイムアウトログは不要
    }
    break;
  }

  case L2CAP_EVENT_CONNECTION_PARAMETER_UPDATE_RESPONSE: {
    // gap_request_connection_parameter_update (L2CAP 経由) への Central の
    // accept(0)/reject(1)。reject ならピン留め (12,12) が蹴られたので 1 回
    // だけ (12,24) へフォールバック。
    uint16_t result =
        l2cap_event_connection_parameter_update_response_get_result(packet);
    printf("[BLE_UART] CI param update response: %s (result=%u)\n",
           result == 0 ? "accepted" : "rejected", result);
    if (result != 0)
      request_ci_fallback("rejected");
    break;
  }

  case HCI_EVENT_LE_META:
    switch (hci_event_le_meta_get_subevent_code(packet)) {
    case HCI_SUBEVENT_LE_CONNECTION_COMPLETE: {
      // status != 0 は「接続確立失敗」(例 0x3E)。このとき handle はゴミなので
      // 取り込むと con_handle が偽の有効値になり、以後 DISCONNECTION も来ない
      // ため再アドバタイズ経路が完全に死ぬ(=再起動するまで再接続不能)。
      uint8_t st = hci_subevent_le_connection_complete_get_status(packet);
      if (st != ERROR_CODE_SUCCESS) {
        printf("[BLE_UART] connection failed (status=0x%02x), re-advertising\n",
               st);
        gap_advertisements_enable(1);
        break;
      }
      hci_con_handle_t h =
          hci_subevent_le_connection_complete_get_connection_handle(packet);
      if (h == con_handle)
        break; // ← 二重配送ガード(下記参照)
      con_handle = h;
      note_conn_activity(); // ウォッチドッグの基準時刻を接続時に仕切り直す
      // 接続時の Connection Interval を捕捉 (ペアリング完了時に表示する)。
      conn_interval =
          hci_subevent_le_connection_complete_get_conn_interval(packet);
      tx_notify_enabled = false;
      can_send_requested = false;
      cmd_authorized = false;
      ci_nego_stage = 0; // 新しい接続: CI ネゴシエーションを仕切り直す
      printf("[BLE_UART] connected, handle=0x%04x (requesting pairing)\n", h);
      sm_request_pairing(con_handle); // ★ 接続直後に Security Request を送る
      break;
    }
    case HCI_SUBEVENT_LE_CONNECTION_UPDATE_COMPLETE:
      // 接続パラメータ更新後の新しい CI を反映。
      conn_interval =
          hci_subevent_le_connection_update_complete_get_conn_interval(packet);
      print_conn_interval("updated");
      // ピン留め要求 (12,12) の実績確認: 12 にならなかったら一度だけ (12,24) へ。
      if (conn_interval != FORCED_CI)
        request_ci_fallback("updated CI != 12");
      break;
    case HCI_SUBEVENT_LE_PHY_UPDATE_COMPLETE: {
      // 2M PHY 要求 (request_2m_phy) の結果。tx/rx: 1=1M, 2=2M, 3=Coded。
      uint8_t st = hci_subevent_le_phy_update_complete_get_status(packet);
      uint8_t txp = hci_subevent_le_phy_update_complete_get_tx_phy(packet);
      // この btstack には rx_phy の getter が無い。HCI 仕様上 tx_phy(event[6])
      // の次のバイトが rx_phy。
      uint8_t rxp = packet[7];
      printf("[BLE_UART] PHY update: status=0x%02x tx=%uM rx=%uM\n", st, txp,
             rxp);
      phy_probe_active = false; // 応答あり → タイムアウト監視を解除
      break;
    }
    default:
      break;
    }
    break;

  case HCI_EVENT_DISCONNECTION_COMPLETE:{
    // reason: 0x13=central(mac)側が切断 / 0x08=supervision timeout /
    // 0x3D=MIC failure / 0x16=こちら(host)発。原因切り分けの決定打になる。
    printf("[BLE_UART] disconnected (reason=0x%02x), re-advertising\n",
           hci_event_disconnection_complete_get_reason(packet));
    con_handle = HCI_CON_HANDLE_INVALID;
    tx_notify_enabled = false;
    can_send_requested = false;
    cmd_authorized = false; // 切断時は必ず locked へ倒す
    ci_nego_stage = 0;
    nc_pending_handle = HCI_CON_HANDLE_INVALID;
    gap_advertisements_enable(1);
    break;
  }
  case ATT_EVENT_CAN_SEND_NOW:
    can_send_requested = false;
    note_conn_activity(); // Central が notify を受けている = 生存の兆候
    ++tx_stat_csn;
    flush_tx();
    break;

  case ATT_EVENT_MTU_EXCHANGE_COMPLETE:
    printf("[BLE_UART] MTU = %u\n",
           att_event_mtu_exchange_complete_get_MTU(packet));
    break;

  case GAP_EVENT_RSSI_MEASUREMENT: {
    // gap_read_rssi の結果。getter は uint8_t を返すが値は dBm の符号付きなので
    // int8_t へ落としてから扱う。"RSSI=<dBm>\n" 行にして notify でホストへ。
    int rssi = (int8_t)gap_event_rssi_measurement_get_rssi(packet);
    note_conn_activity(); // RSSI が測れている = リンク生存の兆候
    char buf[24];
    snprintf(buf, sizeof(buf), "RSSI=%d\n", rssi);
    tx_emit_line(buf);
    if (con_handle != HCI_CON_HANDLE_INVALID && tx_notify_enabled &&
        !can_send_requested) {
      can_send_requested = true;
      att_server_request_can_send_now_event(con_handle);
    }
    printf("[BLE_UART] RSSI = %d dBm\n", rssi);
    break;
  }

  default:
    break;
  }
}

// ---- SM(ペアリング/暗号化)イベント ---------------------------------------
// A の結果を B のケーパビリティビットへ反映する。独自鍵は扱わない。
static void sm_packet_handler(uint8_t packet_type, uint16_t channel,
                              uint8_t *packet, uint16_t size) {
  (void)channel;
  (void)size;
  if (packet_type != HCI_EVENT_PACKET)
    return;
  ble_dbg_evt = ble_dbg_evt + 1;

  switch (hci_event_packet_get_type(packet)) {

  case SM_EVENT_NUMERIC_COMPARISON_REQUEST:
    // NC: 番号を USB CDC に出し、オペレータの照合 confirm を poll
    // ループで待つ。
    nc_pending_handle = sm_event_numeric_comparison_request_get_handle(packet);
    printf("[BLE_UART] === NUMERIC COMPARISON ===\n");
    printf(
        "[BLE_UART] device number : %06lu\n",
        (unsigned long)sm_event_numeric_comparison_request_get_passkey(packet));
    printf(
        "[BLE_UART] スマホ側の表示と一致していれば 'y'、違えば 'n' を入力\n");
    break;

  case SM_EVENT_PASSKEY_DISPLAY_NUMBER:
    // Central が Passkey Entry を選んだ場合のフォールバック。BTstack 生成の
    // ランダム passkey をシリアルへ出し、これをスマホ側へ入力させる。
    printf("[BLE_UART] passkey (enter on phone) = %06lu\n",
           (unsigned long)sm_event_passkey_display_number_get_passkey(packet));
    break;

  case SM_EVENT_PAIRING_COMPLETE:
    if (sm_event_pairing_complete_get_status(packet) == ERROR_CODE_SUCCESS) {
      cmd_authorized = true;
      note_conn_activity(); // 認可直後からウォッチドッグの窓を仕切り直す
      printf("[BLE_UART] pairing complete -> authorized\n");
      print_conn_interval("paired"); // ★ ペアリング直後に CI を表示
      request_fast_ci();             // ★ CI=12 (15ms) を強制要求
      request_2m_phy();              // ★ 2M PHY を要求 (結果は PHY update イベント)
    } else {
      cmd_authorized = false;
      printf("[BLE_UART] pairing failed (status 0x%02x)\n",
             sm_event_pairing_complete_get_status(packet));
    }
    break;

  case SM_EVENT_REENCRYPTION_COMPLETE:
    // bond 済みデバイスの再接続(本構成では RAM db
    // なのでデバイス再起動後は失敗し、 その場合 BTstack peripheral
    // は自動で再ペアリングへ入る)。
    if (sm_event_reencryption_complete_get_status(packet) ==
        ERROR_CODE_SUCCESS) {
      cmd_authorized = true;
      note_conn_activity(); // 再接続(再暗号化)直後も窓を仕切り直す
      printf("[BLE_UART] reencryption complete -> authorized\n");
      print_conn_interval("paired"); // ★ 再接続(再暗号化)直後にも CI を表示
      request_fast_ci();             // ★ CI=12 (15ms) を強制要求
      request_2m_phy();              // ★ 2M PHY を要求 (結果は PHY update イベント)
    } else {
      cmd_authorized = false;
      printf("[BLE_UART] reencryption failed -> stays locked\n");
    }
    break;

  default:
    break;
  }
}

// ===========================================================================
//  エクスポートするメソッド
// ===========================================================================
// 互換用の send_byte 行ステージング。'\n' で 1 メッセージ確定 → sys ストリームへ。
// (現行の飛行構成では TELEMETRY が自前ストリームを使うためこの経路は不使用。
//  HELLO_WORLD ベンチ等の旧 send_byte 呼び出し元のためだけに残す。)
static uint8_t compat_line[512];
static uint32_t compat_len = 0;

// notify が有効で要求未発なら次の送信機会を確保 (send_byte/send_buf 用)。
static void request_send_if_needed() {
  if (con_handle != HCI_CON_HANDLE_INVALID && tx_notify_enabled &&
      !can_send_requested) {
    can_send_requested = true;
    att_server_request_can_send_now_event(con_handle);
  }
}

static void method_send_byte(uint32_t _caller_obj_id,
                             uint32_t _caller_thread_id, uint32_t byte,
                             uint32_t _arg1) {
  (void)_caller_obj_id;
  (void)_caller_thread_id;
  (void)_arg1;
  uint8_t b = (uint8_t)(byte & 0xFF);
  if (compat_len < sizeof(compat_line))
    compat_line[compat_len++] = b;
  if (b == (uint8_t)'\n') {
    if (!bt::push_msg(g_tx_sys.hdl(), compat_line, compat_len))
      dropped_msgs++;
    compat_len = 0;
    request_send_if_needed();
  }
}

// arg0 = ble_tx_buf_t* 。バッファをまとめて 1 メッセージとして sys ストリームへ積む。
static void method_send_buf(uint32_t _caller_obj_id, uint32_t _caller_thread_id,
                            uint32_t ptr, uint32_t _arg1) {
  (void)_caller_obj_id;
  (void)_caller_thread_id;
  (void)_arg1;
  if (ptr == 0)
    return;
  const ble_tx_buf_t *b = (const ble_tx_buf_t *)(uintptr_t)ptr;
  if (b->data == nullptr)
    return;
  if (!bt::push_msg(g_tx_sys.hdl(), b->data, b->len))
    dropped_msgs++;
  request_send_if_needed();
}

// arg0 = uint32_t* out。sys ストリームの空きバイト数 (writable_slots × 244) を書く。
// TELEMETRY のバルク backpressure は自前の bulk ストリームを直接見るのでここは使わない。
static void method_get_tx_free(uint32_t _caller_obj_id,
                               uint32_t _caller_thread_id, uint32_t out_ptr,
                               uint32_t _arg1) {
  (void)_caller_obj_id;
  (void)_caller_thread_id;
  (void)_arg1;
  if (out_ptr == 0)
    return;
  *(uint32_t *)(uintptr_t)out_ptr = g_tx_sys.hdl().writable_slots() * bt::FRAME_MAX;
}

// arg0 = (stream_id << 16) | priority。外部 producer の TX ストリームを排出表へ登録。
static void method_register_tx_stream(uint32_t _caller_obj_id,
                                      uint32_t _caller_thread_id, uint32_t arg,
                                      uint32_t _arg1) {
  (void)_caller_obj_id;
  (void)_caller_thread_id;
  (void)_arg1;
  uint32_t id = arg >> 16;
  uint32_t prio = arg & 0xFFFFu;
  bt::handle_t h = shizu::stream::open<bt::frame_t>(id);
  if (!h.valid()) {
    printf("[BLE_UART] register_tx_stream: id=%lu not found\n",
           (unsigned long)id);
    return;
  }
  shizu::stream::bind(id, shizu::stream::role::CONSUMER);
  tx_src_insert(h, prio);
  printf("[BLE_UART] tx stream registered: id=%lu prio=%lu\n",
         (unsigned long)id, (unsigned long)prio);
}

static void method_set_rx_sink(uint32_t _caller_obj_id,
                               uint32_t _caller_thread_id, uint32_t packed,
                               uint32_t _arg1) {
  (void)_caller_obj_id;
  (void)_caller_thread_id;
  (void)_arg1;
  rx_sink_obj_id = (packed >> 16) & 0xFFFF;
  rx_sink_method_id = packed & 0xFFFF;
  printf("[BLE_UART] rx_sink -> obj=%lu method=%lu\n",
         (unsigned long)rx_sink_obj_id, (unsigned long)rx_sink_method_id);
}

// ===========================================================================
//  BT スタック一式の起動/再起動
// ===========================================================================
// 初回起動と「チップリセット後の再起動」で共用する。cyw43_arch_init() が
// btstack コア (hci_init / run loop / TLV) まで初期化するので、その上の
// プロトコル層 (L2CAP/SM/ATT) と GAP 設定をここで毎回積み直す。
// 再起動時は呼び出し前に bt_stack_teardown() で各層を deinit しておくこと
// (sm_init 等は再初期化ガード付きのため、deinit しないと no-op になり、
// 新しい hci にハンドラが再登録されず SM が沈黙する)。
static void bt_stack_bringup() {
  // CYW43 チップ起動 (WiFi/BT ファームウェアのロードを含む)
  cyw43_arch_init();
  l2cap_init();
  sm_init();

  // --- A: SM セキュリティ設定 ---
  // LE Secure Connections + Numeric Comparison。Pico を DISPLAY_YES_NO 役にし、
  // 番号は USB CDC へ printf、confirm は USB CDC からの getchar で取る。
  // NC は LESC 専用方式なので btstack_config.h の ENABLE_LE_SECURE_CONNECTIONS が
  // 必須 (無いと Legacy へ落ち、MITM 必須のため Passkey Entry になる)。
  // DISPLAY_YES_NO + SC + MITM の両側で、ほぼ全ての Central と NC になる。
  // ※ Central が KeyboardOnly 等で NC 非対応のときのみ下の Passkey 分岐に倒れる。
  sm_set_io_capabilities(IO_CAPABILITY_DISPLAY_YES_NO);
  sm_set_authentication_requirements(SM_AUTHREQ_SECURE_CONNECTION |
                                     SM_AUTHREQ_MITM_PROTECTION |
                                     SM_AUTHREQ_BONDING);

  // ATT サーバ (生成済み profile_data を渡す)
  att_server_init(profile_data, att_read_callback, att_write_callback);

  // HCI / ATT / SM / L2CAP イベント登録
  hci_event_callback_registration.callback = &packet_handler;
  hci_add_event_handler(&hci_event_callback_registration);
  att_server_register_packet_handler(packet_handler);
  sm_event_callback_registration.callback = &sm_packet_handler;
  sm_add_event_handler(&sm_event_callback_registration);
  // CI パラメータ更新の accept/reject (L2CAP signaling 応答) を受けるため。
  l2cap_event_callback_registration.callback = &packet_handler;
  l2cap_add_event_handler(&l2cap_event_callback_registration);

  // 3) 広告設定
  build_adv_data();
  bd_addr_t null_addr;
  memset(null_addr, 0, sizeof(null_addr));
  uint16_t adv_int_min = 0x0030; // 30ms
  uint16_t adv_int_max = 0x0060;
  gap_advertisements_set_params(adv_int_min, adv_int_max, 0 /*ADV_IND*/, 0,
                                null_addr, 0x07, 0x00);
  gap_advertisements_set_data(adv_len, adv_buffer);
  // gap_advertisements_enable は HCI_STATE_WORKING で行う。

  // 電源 ON (HCI_STATE_WORKING 到達で packet_handler が広告を開始する)
  hci_power_control(HCI_POWER_ON);
}

// BT スタック一式の解体 → チップ電源断。上位層の明示 deinit が重要
// (bt_stack_bringup のコメント参照)。cyw43_arch_deinit() は内部で
// hci_power_control(OFF) + hci_close + run loop/メモリ解体まで行い、
// チップの電源も落とす (次の cyw43_arch_init で FW が再ロードされる)。
static void bt_stack_teardown() {
  att_server_deinit();
  sm_deinit();
  l2cap_deinit();
  cyw43_arch_deinit();
}

// コントローラ完全無応答 (HCI 電源再投入でも FW 再ダウンロードが通らない
// ケースを実測) からの最終復旧手段: CYW43 チップごとリセットして全部作り直す。
// ~2s ブロックする (協調スケジューラなので全スレッド停止) が、BT が死んでいる
// 時点でテレメトリは既に止まっており、代償より復旧を優先する。
static void bt_full_chip_restart() {
  printf("[BLE_UART] full CYW43 chip reset...\n");
  bt_stack_teardown();
  obj_api::yield_us(100000); // 100ms: チップ電源断を落ち着かせつつ他スレッドへ譲る
  bt_stack_bringup();
  printf("[BLE_UART] chip reset done, waiting for HCI ready\n");
}

// ===========================================================================
//  オブジェクトエントリ
// ===========================================================================
void BLE_UART_DRIVER::init() {
  printf("[BLE_UART] init\n");

  // 1) メソッドを先にエクスポート (BT が立ち上がる前でも積める)
  export_method<method_send_byte>(BLE_UART_DRIVER::METHOD_IDs::send_byte);
  export_method<method_send_buf>(BLE_UART_DRIVER::METHOD_IDs::send_buf);
  export_method<method_set_rx_sink>(BLE_UART_DRIVER::METHOD_IDs::set_rx_sink);
  export_method<method_get_tx_free>(BLE_UART_DRIVER::METHOD_IDs::get_tx_free);
  export_method<method_register_tx_stream>(
      BLE_UART_DRIVER::METHOD_IDs::register_tx_stream);

  // BLE_UART 自身の内部行 (RSSI/status/互換 send) 用の sys ストリームを排出表へ。
  // 外部ストリーム (TELEMETRY の bulk/ctrl) は register_tx_stream で後から加わる。
  tx_src_insert(g_tx_sys.hdl(), bt::PRIO_SYS);

  // 2) BT スタック起動 (CYW43 チップ + btstack 全層 + 広告設定 + 電源 ON)
  bt_stack_bringup();

  // 3) poll ループ
  absolute_time_t next_rssi = make_timeout_time_ms(RSSI_PERIOD_MS);
  absolute_time_t next_adv_ensure = make_timeout_time_ms(1000);
  absolute_time_t next_tx_stat = make_timeout_time_ms(1000);
  while (true) {
    ble_dbg_poll = ble_dbg_poll + 1;
    cyw43_arch_poll();

    // LE features 読み出し (HCI ready 時に予約)。クレジットが空くのを待って
    // から送る (hci_send_cmd はクレジット無しだと黙って捨てるため)。
    if (le_features_pending && hci_can_send_command_packet_now()) {
      le_features_pending = false;
      uint8_t st = hci_send_cmd(&hci_le_read_local_supported_features);
      printf("[BLE_UART] LE features read sent -> %u\n", st);
      if (st != 0)
        le_features_pending = true; // 万一失敗したら次周で再試行
    }

    // 2M PHY 応答ウォッチ: 要求から 5s、Command Status / PHY update のどちらも
    // 来なければ 1 回だけ結論を出す (コントローラ FW 非対応の可能性が濃厚)。
    if (phy_probe_active && time_us_64() - phy_req_us > PHY_PROBE_TIMEOUT_US) {
      phy_probe_active = false;
      printf("[BLE_UART] 2M PHY: no response in 5s (likely unsupported by "
             "controller FW)\n");
    }

    // TX 統計 (A/B 計測用): 1 秒窓の notify 数 / CAN_SEND_NOW 数 / バイト数。
    // pkts/csn が 1.0 を超えていれば 1 CI に複数発詰められている証拠。
    if (time_reached(next_tx_stat)) {
      if (tx_stat_pkts != 0 || tx_stat_csn != 0) {
        printf("[BLE_UART] tx 1s: %lu pkts / %lu csn / %lu B\n",
               (unsigned long)tx_stat_pkts, (unsigned long)tx_stat_csn,
               (unsigned long)tx_stat_bytes);
        tx_stat_pkts = 0;
        tx_stat_csn = 0;
        tx_stat_bytes = 0;
      }
      // ctrl (ping 応答等) の device 内滞在時間。RTT>100ms の切り分け: この値が
      // 小さい (≲CI=15ms) のに GUI の RTT が跳ねるなら犯人は host 側で確定。
      if (bt::ctrl_lat_max_us != 0) {
        printf("[BLE_UART] ctrl_lat 1s: last=%luus max=%luus\n",
               (unsigned long)bt::ctrl_lat_last_us,
               (unsigned long)bt::ctrl_lat_max_us);
        bt::ctrl_lat_max_us = 0;
      }
      next_tx_stat = make_timeout_time_ms(1000);
    }

    // RSSI を周期サンプリング。認可済み (notify が流れる状態) のときだけ投げ、
    // 結果は GAP_EVENT_RSSI_MEASUREMENT で受けてホストへ送る。
    if (con_handle != HCI_CON_HANDLE_INVALID && tx_notify_enabled &&
        cmd_authorized && time_reached(next_rssi)) {
      gap_read_rssi(con_handle);
      next_rssi = make_timeout_time_ms(RSSI_PERIOD_MS);
    }

    // NC 確認待ちなら USB CDC を覗いて y/n を処理(非ブロッキング)。
    // ここで確認することで NC が自動 confirm にならず MITM 防御が成立する。
    if (nc_pending_handle != HCI_CON_HANDLE_INVALID) {
      int c = getchar_timeout_us(0);
      if (c == 'y' || c == 'Y') {
        printf("[BLE_UART] numeric comparison confirmed\n");
        sm_numeric_comparison_confirm(nc_pending_handle);
        nc_pending_handle = HCI_CON_HANDLE_INVALID;
      } else if (c == 'n' || c == 'N') {
        printf("[BLE_UART] numeric comparison declined\n");
        sm_bonding_decline(nc_pending_handle);
        nc_pending_handle = HCI_CON_HANDLE_INVALID;
      }
    }

    // notify が有効でストリームに残りがあるのに要求が出ていなければ補填 (producer の
    // push は SVC を通らないので、この poll がアイドル→非空の cold start を拾う)。
    if (con_handle != HCI_CON_HANDLE_INVALID && tx_notify_enabled &&
        !tx_all_empty() && !can_send_requested) {
      can_send_requested = true;
      att_server_request_can_send_now_event(con_handle);
    }

    // 接続ウォッチドッグ: 生存兆候が途絶えたらローカル状態を畳む (冒頭コメント
    // 参照)。通信中 (認可+notify) は CAN_SEND_NOW/RSSI が毎秒来るので 5s、
    // ペアリング途中など人手が絡む段階は 30s の猶予にする。実際の再アドバタイズ
    // は下の広告保険が「未接続」を見て行う。
    if (con_handle != HCI_CON_HANDLE_INVALID) {
      uint64_t idle = time_us_64() - last_conn_activity_us;
      uint64_t limit = (cmd_authorized && tx_notify_enabled)
                           ? CONN_IDLE_TIMEOUT_US
                           : CONN_PAIRING_TIMEOUT_US;
      if (idle > limit) {
        printf("[BLE_UART] link idle %lums — force cleanup & re-advertise\n",
               (unsigned long)(idle / 1000));
        gap_disconnect(con_handle); // リンクがまだ生きていれば正規に切る (死んでいれば無害)
        con_handle = HCI_CON_HANDLE_INVALID;
        tx_notify_enabled = false;
        can_send_requested = false;
        cmd_authorized = false;
        nc_pending_handle = HCI_CON_HANDLE_INVALID;
        // 掃除で戻れたかの確認を開始: この gap_disconnect / 直後の広告 enable に
        // 対する応答イベントが来なければコントローラごと無応答 → 電源再投入へ。
        wd_recovery_pending = true;
        wd_cleanup_us = time_us_64();
        wd_evt_snapshot = ble_dbg_evt;
      }
    }

    // ウォッチドッグのエスカレーション: 掃除後 WD_RECOVERY_TIMEOUT_US 経っても
    // HCI イベントが 1 つも来ない = コントローラ/トランスポート無応答。BT を
    // 電源から入れ直す (bt_loaded を倒すことで FW 再ダウンロード=コア完全リセット)。
    if (wd_recovery_pending) {
      if (ble_dbg_evt != wd_evt_snapshot) {
        wd_recovery_pending = false; // 応答あり = コントローラ生存、掃除で十分
      } else if (time_us_64() - wd_cleanup_us > WD_RECOVERY_TIMEOUT_US) {
        wd_recovery_pending = false;
        printf("[BLE_UART] controller unresponsive — resetting CYW43\n");
        // hci_power_control の OFF→ON (+BT FW 再ダウンロード) では蘇生しない
        // ことを実測済み。チップごとリセットして btstack 全層を作り直す。
        bt_full_chip_restart();
      }
    }

    // 広告保険: 未接続なのに広告が止まっている事態 (切断イベント取りこぼし後や
    // enable 失敗) を避けるため、切断中は 1s ごとに広告を張り直す (enable は冪等)。
    if (con_handle == HCI_CON_HANDLE_INVALID && time_reached(next_adv_ensure)) {
      gap_advertisements_enable(1);
      next_adv_ensure = make_timeout_time_ms(1000);
    }
    obj_api::yield_us(1000);
  }
}

} // namespace shizu