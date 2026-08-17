#include <call_method.hpp>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <export_method.hpp>
#include <obj_api.hpp>
#include <object_headers/BLE_UART_DRIVER.hpp>
#include <object_headers/HELLO_WORLD.hpp>
#include <object_id.hpp>
#include <pico/time.h> // time_us_64 (プロセッサ時間 = 起動からの経過 us)
#include <log.hpp>

namespace shizu {

// ---- RTC 同調状態 ----------------------------------------------------------
// wall_us = time_us_64() + clock_offset_us。ホストから "T<epoch_us>" を受けた
// 瞬間に offset = epoch_us - uptime で確定する。
static volatile int64_t clock_offset_us = 0;
static volatile bool clock_synced = false;

// ---- 計測カウンタ / ブラスト状態 -------------------------------------------
static volatile uint32_t rx_total = 0; // 受信バイト総数 (アップリンク計測用)
static volatile bool blast_active = false; // ダウンリンク・ブラスト中か
static volatile uint64_t blast_end_us = 0; // ブラスト終了時刻
static volatile uint32_t blast_seq = 0;    // 送出した D 行の連番
static volatile uint32_t blast_bytes = 0;  // 送出した総バイト数

// ブラスト 1 行のペイロード長 (D<seq> + パディング + CRLF)。
// MTU=527 なら 1 notify で ~524B 送れるので、それに収まる大きめの値にして
// 30ms あたりの転送量 (スループット) を上げる。ドライバの TX_LINE_MAX(540) と
// flush_tx の chunk(512) 以下に収めること。
static constexpr int BLAST_LINE_LEN = 480;

// 通常 (Hello, World) の送出周期。1 接続イベントあたり実質 1 notify しか
// 確実に流れないため、実効 CI (~30ms) より短い周期で送ると TX リングが溢れて
// 行ドロップ (パケロス) しやすい。ロスを避けるためここを 30ms に合わせる。
static constexpr uint64_t HELLO_PERIOD_US = 15000;

// RX 行アセンブリ用バッファ。
static char rxline[64];
static uint32_t rxlen = 0;

// 1 文字ずつ BLE_UART_DRIVER の TX リングへ積む。
// 購読前は内部リングで捨てられるだけなので、いつ呼んでも安全。
static void ble_send(const char *s, int len) {
  for (int i = 0; i < len; ++i) {
    call_method(object_ids::BLE_UART_DRIVER,
                BLE_UART_DRIVER::METHOD_IDs::send_byte,
                (uint32_t)(uint8_t)s[i]);
  }
}

// 確定した 1 行コマンドを処理する。
static void handle_line() {
  rxline[rxlen] = '\0';
  if (rxlen == 0)
    return;

  switch (rxline[0]) {

  case 'T': { // 時刻同期: "T<10進エポックマイクロ秒>"
    uint64_t v = 0;
    bool ok = false;
    for (uint32_t i = 1; i < rxlen; ++i) {
      if (rxline[i] < '0' || rxline[i] > '9')
        break;
      v = v * 10u + (uint64_t)(rxline[i] - '0');
      ok = true;
    }
    if (ok) {
      clock_offset_us = (int64_t)v - (int64_t)time_us_64();
      clock_synced = true;
      int64_t wall = (int64_t)time_us_64() + clock_offset_us;
      log::printf("[HELLO_WORLD] time synced, wall=%lu s\n",
             (unsigned long)(wall / 1000000));
    }
    break;
  }

  case 'P': { // ping: 受け取った行をそのままエコーして返す
    ble_send(rxline, (int)rxlen);
    ble_send("\r\n", 2);
    break;
  }

  case 'B': { // ダウンリンク・ブラスト開始: "B<秒>" (既定 3、最大 30)
    uint32_t secs = 0;
    bool any = false;
    for (uint32_t i = 1; i < rxlen; ++i) {
      if (rxline[i] < '0' || rxline[i] > '9')
        break;
      secs = secs * 10u + (uint32_t)(rxline[i] - '0');
      any = true;
    }
    if (!any)
      secs = 3;
    if (secs > 30)
      secs = 30;
    blast_seq = 0;
    blast_bytes = 0;
    blast_end_us = time_us_64() + (uint64_t)secs * 1000000ull;
    blast_active = true;
    log::printf("[HELLO_WORLD] blast start %lu s\n", (unsigned long)secs);
    break;
  }

  case 'S': { // 統計返信: アップリンク受信バイト数
    char tmp[48];
    int n =
        snprintf(tmp, sizeof(tmp), "STATS rx=%lu\r\n", (unsigned long)rx_total);
    ble_send(tmp, n);
    break;
  }

  default:
    break; // それ以外の行 (アップリンク試験データ U... 等) は受信計測のみ
  }
}

// ---- RX 受信 (rx_sink から 1 バイトずつ) -----------------------------------
// 協調スケジューリングなので途中で yield せず、状態更新は原子的。
static void rx_byte_method(uint32_t _caller_obj_id, uint32_t _caller_thread_id,
                           uint32_t byte, uint32_t _arg1) {
  (void)_caller_obj_id;
  (void)_caller_thread_id;
  (void)_arg1;
  char c = (char)(byte & 0xFF);

  rx_total++; // アップリンク総バイト数 (CR/LF 含む raw)

  if (c == '\r')
    return; // CR は無視 (CRLF 対応)

  if (c == '\n') {
    handle_line();
    rxlen = 0;
    return;
  }

  if (rxlen < sizeof(rxline) - 1)
    rxline[rxlen++] = c;
  else
    rxlen = 0; // オーバーフローは行を捨ててリセット
}

// ブラスト 1 行を送出 (時間切れなら BEND を出して終了)。
static void blast_step(char *line, int cap) {
  if (time_us_64() >= blast_end_us) {
    int n = snprintf(line, cap, "BEND seq=%lu bytes=%lu\r\n",
                     (unsigned long)blast_seq, (unsigned long)blast_bytes);
    ble_send(line, n);
    blast_active = false;
    log::printf("[HELLO_WORLD] blast end seq=%lu bytes=%lu\n",
           (unsigned long)blast_seq, (unsigned long)blast_bytes);
    return;
  }
  // "D<seq> " + 'x' パディング + CRLF を BLAST_LINE_LEN まで埋める。
  int n = snprintf(line, cap, "D%lu ", (unsigned long)blast_seq);
  while (n < BLAST_LINE_LEN - 2 && n < cap - 2)
    line[n++] = 'x';
  line[n++] = '\r';
  line[n++] = '\n';
  ble_send(line, n);
  blast_seq++;
  blast_bytes += (uint32_t)n;
}

void HELLO_WORLD::main() {
  log::printf("HELLO_WORLD::main\n");

  // RX 経由のコマンドを受け取れるよう、自分の rx_byte を
  // BLE_UART の rx_sink に登録する。
  export_method<rx_byte_method>(HELLO_WORLD::METHOD_IDs::rx_byte);
  uint32_t sink = ((uint32_t)object_ids::HELLO_WORLD << 16) |
                  HELLO_WORLD::METHOD_IDs::rx_byte;
  call_method(object_ids::BLE_UART_DRIVER,
              BLE_UART_DRIVER::METHOD_IDs::set_rx_sink, sink);

  char line[BLAST_LINE_LEN + 16];
  while (true) {
    // ブラスト中も 1 接続イベントに収まるよう実効 CI (~30ms) ペースで送る。
    // これより速く送ると TX リングが溢れて行ドロップ (パケロス) するため、
    // ロスの出ない安定スループットを測る目的でここを 30ms に合わせる。
    if (blast_active) {
      blast_step(line, (int)sizeof(line));
      obj_api::yield_us(HELLO_PERIOD_US);
      continue;
    }

    // 通常時: "Hello, World" + プロセッサ時間 (+ 同期済みなら wall)。
    uint64_t now_us = time_us_64();
    uint32_t up_sec = (uint32_t)(now_us / 1000000ull);
    uint32_t up_us = (uint32_t)(now_us % 1000000ull);

    int len;
    if (clock_synced) {
      int64_t wall_us = (int64_t)now_us + clock_offset_us;
      uint32_t w_sec = (uint32_t)(wall_us / 1000000);
      uint32_t w_us = (uint32_t)(wall_us % 1000000);
      len = snprintf(line, sizeof(line),
                     "Hello, World  up=%lu.%06lu s  wall=%lu.%06lu\r\n",
                     (unsigned long)up_sec, (unsigned long)up_us,
                     (unsigned long)w_sec, (unsigned long)w_us);
    } else {
      len = snprintf(line, sizeof(line),
                     "Hello, World  up=%lu.%06lu s  wall=unsynced\r\n",
                     (unsigned long)up_sec, (unsigned long)up_us);
    }
    if (len < 0)
      len = 0;
    if (len > (int)sizeof(line))
      len = sizeof(line);

    ble_send(line, len);

    obj_api::yield_us(HELLO_PERIOD_US);
  }
}

} // namespace shizu
