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
#include <export_method.hpp>
#include <obj_api.hpp>
#include <hardware/sync.h>   // save_and_disable_interrupts / restore_interrupts
#include <object_headers/BLE_UART_DRIVER.hpp>
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

// ---- TX リングバッファ -----------------------------------------------------
// send_byte が行を組み立て、'\n' で 1 行まるごとリングへコミットする。
// CAN_SEND_NOW で flush_tx が notify として吐き出す。
//
// ロック方針: producer (送信オブジェクトのスレッド) と consumer (poll
// スレッドの flush_tx) が tx_head / tx_tail を触る。協調スケジューリングでは
// 関数の途中で切り替わらないが、将来のプリエンプティブ化や IRQ に備えて
// インデックス更新だけを save_and_disable_interrupts でガードする (btstack
// 呼び出しはロック外)。
static constexpr uint32_t TX_RING_SIZE = 2048; // 2 の冪
static constexpr uint32_t TX_RING_MASK = TX_RING_SIZE - 1;
static uint8_t tx_ring[TX_RING_SIZE];
static volatile uint32_t tx_head = 0; // 書き込み位置 (producer)
static volatile uint32_t tx_tail = 0; // 読み出し位置 (consumer)

static inline uint32_t tx_count() { return (tx_head - tx_tail) & TX_RING_MASK; }
static inline bool tx_empty() { return tx_head == tx_tail; }
// 使用可能容量 (満杯判定のため 1 バイト空ける)。
static inline uint32_t tx_free() { return TX_RING_SIZE - 1 - tx_count(); }

// ---- 行ステージング --------------------------------------------------------
// '\n' が来るまでここに溜め、行が確定したら原子的にリングへ移す。リングに
// 入りきらない行は「丸ごと」捨てる (部分行を絶対に出さない = 表示が崩れない)。
// バッチ・バイナリフレーム (TELEMETRY_SENDER) は最悪スタッフィングで ~900B に
// なりうる。540 だと溢れて行ごと破棄され 1 バッチ丸ごと失う。リング(2048)に収まる
// 範囲で 1024 まで拡張する (>512 のフレームは複数 notify に跨るが、クライアントは
// 0x0A まで連結して復元するので問題ない)。
static constexpr uint32_t TX_LINE_MAX = 1024;
static uint8_t line_buf[TX_LINE_MAX];
static uint32_t line_len = 0;
static bool line_overflow = false; // 行が TX_LINE_MAX を超えた
static uint32_t dropped_lines = 0; // 容量不足で捨てた行数 (デバッグ用)

// 確定した 1 行をロック下でリングへコミット。空きが無ければ丸ごと破棄。
static void tx_commit_line() {
  uint32_t s = save_and_disable_interrupts();
  if (!line_overflow && tx_free() >= line_len) {
    for (uint32_t i = 0; i < line_len; ++i) {
      tx_ring[tx_head] = line_buf[i];
      tx_head = (tx_head + 1) & TX_RING_MASK;
    }
  } else {
    dropped_lines++;
  }
  restore_interrupts(s);
  line_len = 0;
  line_overflow = false;
}

// 内部生成の 1 行 (例: "RSSI=-55\n") をロック下でリングへ原子的に積む。
// send_byte の行ステージング (line_buf) とは独立で、空きが無ければ丸ごと破棄。
static void tx_emit_line(const char *s) {
  uint32_t len = (uint32_t)strlen(s);
  uint32_t st = save_and_disable_interrupts();
  if (tx_free() >= len) {
    for (uint32_t i = 0; i < len; ++i) {
      tx_ring[tx_head] = (uint8_t)s[i];
      tx_head = (tx_head + 1) & TX_RING_MASK;
    }
  } else {
    dropped_lines++;
  }
  restore_interrupts(st);
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

// ---- TX flush --------------------------------------------------------------
static void flush_tx() {

    /*printf("[BLE_UART] notify handle=0x%04x mtu=%u\n",
       ATT_CHARACTERISTIC_6E400003_B5A3_F393_E0A9_E50E24DCCA9E_01_VALUE_HANDLE,
       att_server_get_mtu(con_handle));*/
  if (con_handle == HCI_CON_HANDLE_INVALID || !tx_notify_enabled)
    return;
  if (!cmd_authorized)
    return;
  if (tx_empty())
    return;

  uint16_t mtu = att_server_get_mtu(con_handle);
  uint16_t max_payload = (mtu > 3) ? (mtu - 3) : 20; // ATT_NOTIFY = MTU-3

  // MTU をなるべく使い切る (現構成は HCI_ACL_PAYLOAD_SIZE=259 → ATT MTU ~247、
  // notify ペイロード ~244B。CYW43 はこれ以上=LL 251B 超の ACL 分割で固まる
  // 既知バグがあるため、btstack_config.h 側で意図的に制限している)。chunk の
  // 512B は上限保険で、実際の 1 notify は max_payload (=MTU-3) で切られる。
  uint8_t chunk[512];
  if (max_payload > sizeof(chunk))
    max_payload = sizeof(chunk);

  // ロック下で「覗くだけ」(tail は進めない)。成功してから消費する方式にし、
  // エラー時の tail 巻き戻し (producer と競合しうる) を排除する。
  uint16_t n = 0;
  {
    uint32_t s = save_and_disable_interrupts();
    uint32_t t = tx_tail;
    while (n < max_payload && t != tx_head) {
      chunk[n++] = tx_ring[t];
      t = (t + 1) & TX_RING_MASK;
    }
    restore_interrupts(s);
  }
  if (n == 0)
    return;

  // btstack 呼び出しはロック外で行う。
  int err = att_server_notify(
      con_handle,
      ATT_CHARACTERISTIC_6E400003_B5A3_F393_E0A9_E50E24DCCA9E_01_VALUE_HANDLE,
      chunk, n);
  if (err == 0) {
    // 送信成功した分だけ tail を進めて消費を確定する。
    uint32_t s = save_and_disable_interrupts();
    tx_tail = (tx_tail + n) & TX_RING_MASK;
    restore_interrupts(s);
  }
  // err != 0 のときは tail を進めないので、次の CAN_SEND_NOW で再送される。

  // まだ残っていれば次の送信機会を要求。
  if (!tx_empty()) {
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
    if (tx_notify_enabled && !tx_empty() && !can_send_requested) {
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

static void request_fast_ci() {
  if (con_handle == HCI_CON_HANDLE_INVALID)
    return;
  int r = gap_request_connection_parameter_update(con_handle, FORCED_CI,
                                                  2*FORCED_CI, 0, 400);
  uint32_t ms100 = (uint32_t)FORCED_CI * 125u; // 単位 1.25ms → ms×100
  printf("[BLE_UART] request CI min=%lu.%02lu ms -> %d\n",
         (unsigned long)(ms100 / 100), (unsigned long)(ms100 % 100), r);
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
    }
    break;

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
      printf("[BLE_UART] connected, handle=0x%04x (requesting pairing)\n", h);
      sm_request_pairing(con_handle); // ★ 接続直後に Security Request を送る
      break;
    }
    case HCI_SUBEVENT_LE_CONNECTION_UPDATE_COMPLETE:
      // 接続パラメータ更新後の新しい CI を反映。
      conn_interval =
          hci_subevent_le_connection_update_complete_get_conn_interval(packet);
      print_conn_interval("updated");
      break;
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
    nc_pending_handle = HCI_CON_HANDLE_INVALID;
    gap_advertisements_enable(1);
    break;
  }
  case ATT_EVENT_CAN_SEND_NOW:
    can_send_requested = false;
    note_conn_activity(); // Central が notify を受けている = 生存の兆候
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
// 1 バイトを行ステージングへ積む。'\n' で 1 行確定 → リングへコミット。
// send_byte / send_buf 共通の本体。協調スケジューリングでは 1 行分の連続呼び出しの
// 途中でスレッドが切り替わらないので、line_buf は実質その行専有になる。
static void tx_stage_byte(uint8_t b) {
  if (line_len < TX_LINE_MAX)
    line_buf[line_len++] = b;
  else
    line_overflow = true; // 長すぎる行はコミット時に丸ごと破棄する

  if (b == (uint8_t)'\n') {
    tx_commit_line();
    // 通知が有効で送信要求がまだなら、次の送信機会を確保。
    if (con_handle != HCI_CON_HANDLE_INVALID && tx_notify_enabled &&
        !can_send_requested) {
      can_send_requested = true;
      att_server_request_can_send_now_event(con_handle);
    }
  }
}

static void method_send_byte(uint32_t _caller_obj_id,
                             uint32_t _caller_thread_id, uint32_t byte,
                             uint32_t _arg1) {
  (void)_caller_obj_id;
  (void)_caller_thread_id;
  (void)_arg1;
  tx_stage_byte((uint8_t)(byte & 0xFF));
}

// arg0 = ble_tx_buf_t* 。バッファをまとめて積む(1 バイトずつ call_method しない)。
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
  for (uint32_t i = 0; i < b->len; ++i)
    tx_stage_byte(b->data[i]);
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

  // HCI / ATT / SM イベント登録
  hci_event_callback_registration.callback = &packet_handler;
  hci_add_event_handler(&hci_event_callback_registration);
  att_server_register_packet_handler(packet_handler);
  sm_event_callback_registration.callback = &sm_packet_handler;
  sm_add_event_handler(&sm_event_callback_registration);

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

  // 2) BT スタック起動 (CYW43 チップ + btstack 全層 + 広告設定 + 電源 ON)
  bt_stack_bringup();

  // 3) poll ループ
  absolute_time_t next_rssi = make_timeout_time_ms(RSSI_PERIOD_MS);
  absolute_time_t next_adv_ensure = make_timeout_time_ms(1000);
  while (true) {
    ble_dbg_poll = ble_dbg_poll + 1;
    cyw43_arch_poll();

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

    // notify が有効でリングに残りがあるのに要求が出ていなければ補填。
    if (con_handle != HCI_CON_HANDLE_INVALID && tx_notify_enabled &&
        !tx_empty() && !can_send_requested) {
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