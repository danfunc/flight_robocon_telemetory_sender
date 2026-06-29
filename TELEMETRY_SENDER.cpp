// ===========================================================================
//  TELEMETRY_SENDER — センサ融合 + BLE テレメトリ送信オブジェクト
// ===========================================================================
//  BME280_DRIVER / BNO055_DRIVER (Shizuku オブジェクト) を read_latest
//  メソッドで 購読し、相補フィルタで高度/上下速度を融合、altitude_fusion_wifi.c
//  の状態判定と 高度保持 PID を移植して 1 行 CSV にまとめ、BLE_UART_DRIVER
//  経由で母艦へ送る。
//
//  元実装が Wi-Fi UDP だった送信経路を BLE UART (NUS)
//  へ置き換えたもの。受信側は HELLO_WORLD と同じ rx_sink
//  機構でコマンドを受ける。
// ===========================================================================
#include <call_method.hpp>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <export_method.hpp>
#include <obj_api.hpp>
#include <object_headers/BLE_UART_DRIVER.hpp>
#include <object_headers/BME280_DRIVER.hpp>
#include <object_headers/BNO055_DRIVER.hpp>
#include <object_headers/TELEMETRY_SENDER.hpp>
#include <object_id.hpp>
#include <pico/time.h>
#include <soft_math.hpp> // FPU を踏まない sqrtf/fabsf (kernel が FP 例外フレーム非対応)

namespace shizu {

// ===========================================================================
//  制御パラメータ (altitude_fusion_wifi.c より移植)
// ===========================================================================
static constexpr float ALPHA_H = 0.80f; // 高度相補フィルタ係数 (慣性側の重み)
static constexpr float ALPHA_V = 0.75f; // 速度相補フィルタ係数
static constexpr float A_Z_CLIP = 5.0f; // 世界鉛直加速度のクリップ [m/s^2]

// 上昇/下降ヒステリシス閾値 [m/s]
static constexpr float V_ENTER_ASC = +0.30f, V_LEAVE_ASC = +0.10f;
static constexpr float V_ENTER_DESC = -0.30f, V_LEAVE_DESC = -0.10f;

// 高度保持 PID
static constexpr float H_TARGET_M = 1.00f, H_DEADBAND_M = 0.10f;
static constexpr float KP_ALT = 30.0f, KI_ALT = 1.0f, KD_ALT = 5.0f;
static constexpr float SERVO_LIMIT_DEG = 30.0f, I_TERM_LIMIT = 15.0f;
static constexpr float ELEV_UP_DEG = +5.0f, ELEV_DOWN_DEG = -5.0f;

// サンプル周期 (R コマンドで変更可)。既定 20ms = 50Hz。バッチ送信なので、この
// レートで生成したサンプルは BATCH_FLUSH_US ごとにまとめて 1
// フレームで送られる。
static volatile uint32_t telemetry_period_us = 10000;

// 送信フォーマット。true=バイナリフレーム(既定), false=CSV テキスト。F
// コマンドで切替。 バイナリは ~44B/フレームで、CSV(~120B) の約 1/3。BLE
// の実効上限 ~140kbps に対し 余裕を持って高レート送信できる。
static volatile bool telemetry_binary = true;

enum VState { ST_LEVEL = 0, ST_ASC = 1, ST_DESC = 2 };
enum Elev { EL_NEUTRAL = 0, EL_UP = 1, EL_DOWN = 2 };

// ===========================================================================
//  BLE 送信ヘルパー
// ===========================================================================
static void ble_send(const char *s, int len) {
  for (int i = 0; i < len; ++i) {
    call_method(object_ids::BLE_UART_DRIVER,
                BLE_UART_DRIVER::METHOD_IDs::send_byte,
                (uint32_t)(uint8_t)s[i]);
  }
}

// float を四捨五入して整数スケールに落とす (%f を使わず送るため)。
static inline int32_t scaled(float x, float scale) {
  float v = x * scale;
  return (int32_t)(v >= 0 ? v + 0.5f : v - 0.5f);
}

// ===========================================================================
//  RTC 同調 / 計測カウンタ (HELLO_WORLD と同じ枠組み)
// ===========================================================================
static volatile int64_t clock_offset_us = 0;
static volatile bool clock_synced = false;
static volatile uint32_t rx_total = 0;

// ---- ダウンリンク・ブラスト (スループット試験用) ---------------------------
static volatile bool blast_active = false;
static volatile uint64_t blast_end_us = 0;
static volatile uint32_t blast_seq = 0, blast_bytes = 0;
static constexpr int BLAST_LINE_LEN = 480;

// ---- 較正オフセット保存/復元のダウンリンク待ち -----------------------------
// CS/CL コマンドは BNO スレッドに非同期で処理されるので、完了を main ループで
// ポーリングして結果(CDUMP/CLOAD)をダウンリンクする。
static bool calib_pending = false;     // 結果待ち中
static bool calib_pending_save = false; // true=save(ダンプ返信), false=load(ack)

// ---- RX 行アセンブリ -------------------------------------------------------
static char rxline[80];
static uint32_t rxlen = 0;

// ===========================================================================
//  PID
// ===========================================================================
struct PID {
  float integral = 0.f, prev_err = 0.f;
  bool first = true;
  float step(float err, float dt) {
    float p = KP_ALT * err;
    integral += err * dt;
    if (integral > I_TERM_LIMIT)
      integral = I_TERM_LIMIT;
    if (integral < -I_TERM_LIMIT)
      integral = -I_TERM_LIMIT;
    float i = KI_ALT * integral;
    float d = first ? 0.f : KD_ALT * (err - prev_err) / dt;
    prev_err = err;
    first = false;
    float out = p + i + d;
    if (out > SERVO_LIMIT_DEG)
      out = SERVO_LIMIT_DEG;
    if (out < -SERVO_LIMIT_DEG)
      out = -SERVO_LIMIT_DEG;
    return out;
  }
};

// 傾き補正済みの世界鉛直加速度 (重力ベクトルへの線形加速度の射影)。
static float world_z_accel(const bno055_sample_t &b) {
  float gn = soft_math::sqrtf(b.gx * b.gx + b.gy * b.gy + b.gz * b.gz);
  if (gn < 1.0f)
    return 0.0f;
  return (b.lax * b.gx + b.lay * b.gy + b.laz * b.gz) / gn;
}

static VState next_state(VState cur, float v) {
  switch (cur) {
  case ST_LEVEL:
    if (v > V_ENTER_ASC)
      return ST_ASC;
    if (v < V_ENTER_DESC)
      return ST_DESC;
    return ST_LEVEL;
  case ST_ASC:
    return (v < V_LEAVE_ASC) ? ST_LEVEL : ST_ASC;
  case ST_DESC:
    return (v > V_LEAVE_DESC) ? ST_LEVEL : ST_DESC;
  }
  return ST_LEVEL;
}

// ===========================================================================
//  受信コマンド処理
// ===========================================================================
static void handle_line() {
  rxline[rxlen] = '\0';
  if (rxlen == 0)
    return;

  switch (rxline[0]) {
  case 'T': { // 時刻同期: "T<epoch_us>"
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
      printf("[TELEMETRY] time synced\n");
    }
    break;
  }
  case 'P': { // ping エコー
    ble_send(rxline, (int)rxlen);
    ble_send("\r\n", 2);
    break;
  }
  case 'R': { // 送信周期変更: "R<ms>" (10..1000ms)
    uint32_t ms = 0;
    bool any = false;
    for (uint32_t i = 1; i < rxlen; ++i) {
      if (rxline[i] < '0' || rxline[i] > '9')
        break;
      ms = ms * 10u + (uint32_t)(rxline[i] - '0');
      any = true;
    }
    if (any) {
      if (ms < 10)
        ms = 10;
      if (ms > 1000)
        ms = 1000;
      telemetry_period_us = ms * 1000u;
      printf("[TELEMETRY] period -> %lu ms\n", (unsigned long)ms);
    }
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
    printf("[TELEMETRY] blast start %lu s\n", (unsigned long)secs);
    break;
  }
  case 'S': { // 統計返信
    char tmp[48];
    int n =
        snprintf(tmp, sizeof(tmp), "STATS rx=%lu\r\n", (unsigned long)rx_total);
    ble_send(tmp, n);
    break;
  }
  case 'F': { // 送信フォーマット: F1=バイナリ(既定), F0=CSV テキスト
    telemetry_binary = !(rxlen >= 2 && rxline[1] == '0');
    printf("[TELEMETRY] format -> %s\n", telemetry_binary ? "binary" : "CSV");
    break;
  }
  case 'X': { // BNO055 読みモード: X1=2B 個別読み, X0=26B ブロック読み(既定)
    uint32_t mode = (rxlen >= 2 && rxline[1] == '1') ? 1 : 0;
    call_method(object_ids::BNO055_DRIVER,
                BNO055_DRIVER::METHOD_IDs::set_read_mode, mode);
    printf("[TELEMETRY] BNO read mode -> %s\n", mode ? "split2B" : "block");
    break;
  }
  case 'Y': { // 0xFFFF 破損破棄: Y1=有効(既定), Y0=素通し
    uint32_t on = (rxlen >= 2 && rxline[1] == '0') ? 0 : 1;
    call_method(object_ids::BNO055_DRIVER,
                BNO055_DRIVER::METHOD_IDs::set_ffff_reject, on);
    printf("[TELEMETRY] 0xFFFF reject -> %s\n", on ? "on" : "off");
    break;
  }
  case 'C': { // 較正オフセット: "CS"=保存(吸出), "CL<44hex>"=復元(書込)
    if (rxlen >= 2 && rxline[1] == 'S') {
      call_method(object_ids::BNO055_DRIVER,
                  BNO055_DRIVER::METHOD_IDs::calib_save, 0);
      calib_pending = true;
      calib_pending_save = true;
      printf("[TELEMETRY] calib save requested\n");
    } else if (rxlen >= 2 && rxline[1] == 'L') {
      // "CL" + 44 hex (= 22 バイト) を期待。
      static uint8_t prof[BNO055_CALIB_PROFILE_LEN];
      bool ok = (rxlen >= 2 + 2 * BNO055_CALIB_PROFILE_LEN);
      for (int i = 0; ok && i < BNO055_CALIB_PROFILE_LEN; ++i) {
        auto hexval = [](char c) -> int {
          if (c >= '0' && c <= '9')
            return c - '0';
          if (c >= 'a' && c <= 'f')
            return c - 'a' + 10;
          if (c >= 'A' && c <= 'F')
            return c - 'A' + 10;
          return -1;
        };
        int hi = hexval(rxline[2 + 2 * i]);
        int lo = hexval(rxline[2 + 2 * i + 1]);
        if (hi < 0 || lo < 0)
          ok = false;
        else
          prof[i] = (uint8_t)((hi << 4) | lo);
      }
      if (ok) {
        call_method(object_ids::BNO055_DRIVER,
                    BNO055_DRIVER::METHOD_IDs::calib_load,
                    (uint32_t)(uintptr_t)prof);
        calib_pending = true;
        calib_pending_save = false;
        printf("[TELEMETRY] calib load requested\n");
      } else {
        ble_send("CLOAD err=parse\r\n", 16);
      }
    }
    break;
  }
  default:
    break;
  }
}

static void rx_byte_method(uint32_t, uint32_t, uint32_t byte, uint32_t) {
  char c = (char)(byte & 0xFF);
  rx_total = rx_total + 1;
  if (c == '\r')
    return;
  if (c == '\n') {
    handle_line();
    rxlen = 0;
    return;
  }
  if (rxlen < sizeof(rxline) - 1)
    rxline[rxlen++] = c;
  else
    rxlen = 0;
}

// ブラスト 1 行を送出 (HELLO_WORLD と同じ。時間切れで BEND)。
static void blast_step(char *line, int cap) {
  if (time_us_64() >= blast_end_us) {
    int n = snprintf(line, cap, "BEND seq=%lu bytes=%lu\r\n",
                     (unsigned long)blast_seq, (unsigned long)blast_bytes);
    ble_send(line, n);
    blast_active = false;
    return;
  }
  int n = snprintf(line, cap, "D%lu ", (unsigned long)blast_seq);
  while (n < BLAST_LINE_LEN - 2 && n < cap - 2)
    line[n++] = 'x';
  line[n++] = '\r';
  line[n++] = '\n';
  ble_send(line, n);
  blast_seq = blast_seq + 1;
  blast_bytes = blast_bytes + (uint32_t)n;
}

// ===========================================================================
//  バイナリテレメトリ フレーム (バッファード・バッチ送信)
// ===========================================================================
//  BLE は「1 接続イベントあたり実質 1 notify」が上限なので、サンプルを 1 個ずつ
//  送ると ~140kbps の事象数 (~33/s) に律速される。そこでセンサ融合を高レートで
//  回してサンプルを内部リングに溜め、複数サンプルを 1 フレームにまとめて送る
//  (バッチ)。1 notify
//  に多数サンプルを詰めることで、イベント数の壁を越えて高レート
//  テレメトリをロスなく届けられる。
//
//  ワイヤ表現 (1 バッチフレーム):
//    [0x02 マーカー][0x0A バイトスタッフィング本体][0x0A 終端]
//    本体 = batch_header_t(17B) + count × batch_sample_t(35B) +
//    CRC16-CCITT(2B,LE)
//
//  ・BLE_UART は '\n'(0x0A) で 1 行を原子的にコミットするので、本体から 0x0A を
//    除けば既存の send_byte 経路でフレームが丸ごと届く/落ちる (部分無し)。
//  ・マーカー 0x02 は印字可能 ASCII の制御テキスト行 (RSSI=, P, BEND, STATS) と
//    衝突しないので同一ストリームに混在できる。
//  ・各サンプルの時刻は header.t0_up_ms + sample.d_ms
//  (起動後時間)。さらに時刻同期
//    済みなら header に t0 の壁時計 (Unix epoch) を載せ、絶対時刻も復元できる。

// CRC16-CCITT (poly 0x1021, init 0xFFFF)。
static uint16_t crc16_ccitt(const uint8_t *p, int n) {
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < n; i++) {
    crc ^= (uint16_t)p[i] << 8;
    for (int b = 0; b < 8; b++)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                           : (uint16_t)(crc << 1);
  }
  return crc;
}

// バッチ先頭ヘッダ (バッチ内で一定の情報)。
struct __attribute__((packed)) batch_header_t {
  uint8_t ver;         // =2 (バッチ形式)
  uint8_t count;       // 収録サンプル数
  uint8_t flags;       // bit0: 壁時計同期済み (t0_wall_* が有効)
  uint32_t seq0;       // 先頭サンプルの連番 (以降 seq0+i)
  uint32_t t0_up_ms;   // 先頭サンプルの起動後時間 [ms]
  uint32_t t0_wall_s;  // 先頭サンプルの Unix epoch 秒 (未同期なら 0)
  uint16_t t0_wall_ms; // その小数部 [ms] (未同期なら 0)
};
static_assert(sizeof(batch_header_t) == 17, "batch_header_t must be 17 bytes");

// 1 サンプルの記録 (時刻は t0 からの差分 d_ms)。
struct __attribute__((packed)) batch_sample_t {
  uint16_t d_ms;         // t0_up_ms からの経過 [ms]
  int16_t temp_cC;       // 温度 [1/100 ℃]
  uint32_t press_Pa;     // 気圧 [Pa]
  int32_t alt_baro_mm;   // 気圧高度 [mm]
  int32_t alt_fused_mm;  // 融合高度 [mm]
  int32_t vel_mm_s;      // 上下速度 [mm/s]
  int32_t az_mm_s2;      // 鉛直加速度 [mm/s^2]
  uint16_t heading_cdeg; // 方位 [1/100 deg]
  int16_t roll_cdeg;     // ロール [1/100 deg]
  int16_t pitch_cdeg;    // ピッチ [1/100 deg]
  uint8_t calib;         // 較正 (SYS<<6|GYR<<4|ACC<<2|MAG)
  uint8_t vstate;        // 上下状態
  uint8_t elev;          // エレベータ
  int16_t servo_cdeg;    // サーボ指令 [1/100 deg]
};
// クライアント (struct.unpack '<HhIiiiiHhhBBBh') と一致させる。
static_assert(sizeof(batch_sample_t) == 35, "batch_sample_t must be 35 bytes");

// 1 バッチに詰める最大サンプル数。スタッフィング後でも BLE 側の
// TX_LINE_MAX(1024) に収まり、かつ概ね 1 notify(~512B) に乗るサイズに抑える。
static constexpr int BATCH_MAX = 12;
// バッチを送出する最大間隔。これを超えたら満杯でなくても flush する。
static constexpr uint64_t BATCH_FLUSH_US = 50000; // 50ms

static constexpr uint8_t FRAME_MARKER_BATCH =
    0x02;                                  // バッチフレーム先頭マーカー
static constexpr uint8_t FRAME_ESC = 0x1B; // 0x0A 回避用エスケープ

// バッチ蓄積バッファ。
static batch_sample_t batch_buf[BATCH_MAX];
static int batch_count = 0;
static uint32_t batch_seq0 = 0;
static uint32_t batch_t0_up_ms = 0;
static int64_t batch_t0_wall_us = 0;
static bool batch_t0_synced = false;

// 本体を 0x0A バイトスタッフィングし、マーカー + 終端 '\n' で括って送る。
static void send_framed(uint8_t marker, const uint8_t *payload, int len) {
  // 最悪 (全バイトがスタッフ対象) でも収まるサイズ。
  uint8_t out[2 * (sizeof(batch_header_t) + BATCH_MAX * sizeof(batch_sample_t) +
                   2) +
              4];
  int o = 0;
  out[o++] = marker;
  for (int i = 0; i < len; i++) {
    uint8_t b = payload[i];
    if (b == 0x0A) {
      out[o++] = FRAME_ESC;
      out[o++] = 0x01;
    } else if (b == FRAME_ESC) {
      out[o++] = FRAME_ESC;
      out[o++] = 0x02;
    } else {
      out[o++] = b;
    }
  }
  out[o++] = 0x0A; // フレーム確定 (BLE 側でコミット)
  ble_send((const char *)out, o);
}

// 蓄積したバッチを 1 フレームにまとめて送り、バッファを空にする。
static void flush_batch() {
  if (batch_count == 0)
    return;
  uint8_t body[sizeof(batch_header_t) + BATCH_MAX * sizeof(batch_sample_t) + 2];
  batch_header_t hdr;
  hdr.ver = 2;
  hdr.count = (uint8_t)batch_count;
  hdr.flags = batch_t0_synced ? 0x01 : 0x00;
  hdr.seq0 = batch_seq0;
  hdr.t0_up_ms = batch_t0_up_ms;
  if (batch_t0_synced) {
    hdr.t0_wall_s = (uint32_t)(batch_t0_wall_us / 1000000);
    hdr.t0_wall_ms = (uint16_t)((batch_t0_wall_us / 1000) % 1000);
  } else {
    hdr.t0_wall_s = 0;
    hdr.t0_wall_ms = 0;
  }
  int n = 0;
  memcpy(body + n, &hdr, sizeof(hdr));
  n += sizeof(hdr);
  memcpy(body + n, batch_buf, batch_count * sizeof(batch_sample_t));
  n += batch_count * (int)sizeof(batch_sample_t);
  uint16_t crc = crc16_ccitt(body, n);
  body[n++] = (uint8_t)(crc & 0xFF); // CRC は LE
  body[n++] = (uint8_t)(crc >> 8);
  send_framed(FRAME_MARKER_BATCH, body, n);
  batch_count = 0;
}

// 計算済みサンプルをバッチへ 1 件積む。満杯なら先に flush する。
static void batch_push(uint32_t seq, uint32_t up_ms, const bme280_sample_t &bme,
                       const bno055_sample_t &bno, float h_est, float v_est,
                       float a_z, int vstate, int elev, float servo) {
  if (batch_count == 0) {
    // バッチ先頭: 基準時刻を確定する。
    batch_seq0 = seq;
    batch_t0_up_ms = up_ms;
    batch_t0_synced = clock_synced;
    batch_t0_wall_us = (int64_t)time_us_64() + clock_offset_us;
  }
  batch_sample_t &s = batch_buf[batch_count];
  uint32_t d = up_ms - batch_t0_up_ms;
  s.d_ms = (uint16_t)(d > 0xFFFF ? 0xFFFF : d);
  s.temp_cC = (int16_t)scaled(bme.temp_c, 100.f);
  s.press_Pa = (uint32_t)scaled(bme.press_hpa, 100.f);
  s.alt_baro_mm = scaled(bme.alt_m, 1000.f);
  s.alt_fused_mm = scaled(h_est, 1000.f);
  s.vel_mm_s = scaled(v_est, 1000.f);
  s.az_mm_s2 = scaled(a_z, 1000.f);
  s.heading_cdeg = (uint16_t)scaled(bno.heading, 100.f);
  s.roll_cdeg = (int16_t)scaled(bno.roll, 100.f);
  s.pitch_cdeg = (int16_t)scaled(bno.pitch, 100.f);
  s.calib = bno.calib;
  s.vstate = (uint8_t)vstate;
  s.elev = (uint8_t)elev;
  s.servo_cdeg = (int16_t)scaled(servo, 100.f);
  batch_count++;
  if (batch_count >= BATCH_MAX)
    flush_batch();
}

// ===========================================================================
//  オブジェクトエントリ
// ===========================================================================
void TELEMETRY_SENDER::main() {
  printf("[TELEMETRY] main\n");

  // RX コマンドを受け取れるよう rx_sink を自分へ向ける。
  export_method<rx_byte_method>(TELEMETRY_SENDER::METHOD_IDs::rx_byte);
  uint32_t sink = ((uint32_t)object_ids::TELEMETRY_SENDER << 16) |
                  TELEMETRY_SENDER::METHOD_IDs::rx_byte;
  call_method(object_ids::BLE_UART_DRIVER,
              BLE_UART_DRIVER::METHOD_IDs::set_rx_sink, sink);

  // 融合状態
  float h_est = 0.f, v_est = 0.f;
  float h_baro_prev = 0.f;
  uint32_t last_bme_seq = 0;
  uint64_t last_baro_us = time_us_64();
  VState vstate = ST_LEVEL;
  PID pid;

  uint32_t seq = 0;
  uint64_t prev_us = time_us_64();
  uint64_t last_flush_us = prev_us;
  uint64_t next_us = prev_us; // 送信周期の絶対グリッド (処理時間を吸収して真の周期)
  char line[BLAST_LINE_LEN + 16];

  while (true) {
    // スループット試験中はテレメトリを止めてブラストに専念する。
    if (blast_active) {
      flush_batch(); // 溜まっていれば吐き出してからブラストへ
      blast_step(line, (int)sizeof(line));
      obj_api::yield_us(30000);
      next_us = time_us_64(); // ブラスト後はグリッドを取り直す
      continue;
    }

    // 較正 save/load の完了をポーリングして結果をダウンリンク。
    if (calib_pending) {
      bno055_calib_xfer_t cx = {};
      call_method(object_ids::BNO055_DRIVER,
                  BNO055_DRIVER::METHOD_IDs::calib_get,
                  (uint32_t)(uintptr_t)&cx);
      if (cx.done) {
        char cl[80];
        int n;
        if (calib_pending_save) {
          n = snprintf(cl, sizeof(cl), "CDUMP ");
          for (int i = 0; i < BNO055_CALIB_PROFILE_LEN; ++i)
            n += snprintf(cl + n, sizeof(cl) - n, "%02X", cx.data[i]);
          n += snprintf(cl + n, sizeof(cl) - n, " ok=%d\r\n", cx.ok);
        } else {
          n = snprintf(cl, sizeof(cl), "CLOAD ok=%d\r\n", cx.ok);
        }
        ble_send(cl, n);
        calib_pending = false;
      }
    }

    uint64_t now_us = time_us_64();
    float dt = (float)(now_us - prev_us) / 1e6f;
    prev_us = now_us;
    if (dt <= 0.f || dt > 0.5f)
      dt = 0.02f; // 異常 dt のガード

    // --- センサ購読 (ポインタ渡しでスナップショットを受け取る) ---
    bme280_sample_t bme = {};
    bno055_sample_t bno = {};
    call_method(object_ids::BME280_DRIVER,
                BME280_DRIVER::METHOD_IDs::read_latest,
                (uint32_t)(uintptr_t)&bme);
    call_method(object_ids::BNO055_DRIVER,
                BNO055_DRIVER::METHOD_IDs::read_latest,
                (uint32_t)(uintptr_t)&bno);

    // --- 慣性による高度/速度の予測 (傾き補正済み鉛直加速度を積分) ---
    float a_z = bno.valid ? world_z_accel(bno) : 0.f;
    if (a_z > A_Z_CLIP)
      a_z = A_Z_CLIP;
    if (a_z < -A_Z_CLIP)
      a_z = -A_Z_CLIP;
    h_est += v_est * dt + 0.5f * a_z * dt * dt;
    v_est += a_z * dt;

    // --- 気圧高度で相補補正 (BME のサンプルが更新されたときだけ) ---
    if (bme.valid && bme.seq != last_bme_seq) {
      float baro_dt = (float)(now_us - last_baro_us) / 1e6f;
      if (baro_dt <= 0.f)
        baro_dt = 0.02f;
      last_baro_us = now_us;
      last_bme_seq = bme.seq;
      h_est = ALPHA_H * h_est + (1.0f - ALPHA_H) * bme.alt_m;
      float v_baro = (bme.alt_m - h_baro_prev) / baro_dt;
      v_est = ALPHA_V * v_est + (1.0f - ALPHA_V) * v_baro;
      h_baro_prev = bme.alt_m;
    }

    vstate = next_state(vstate, v_est);

    // --- 高度保持 PID (監視用に算出。サーボ駆動は本オブジェクトの範囲外) ---
    float h_err = H_TARGET_M - h_est;
    float h_err_f = (soft_math::fabsf(h_err) < H_DEADBAND_M) ? 0.f : h_err;
    float servo = pid.step(h_err_f, dt);
    Elev elev = (servo > ELEV_UP_DEG)     ? EL_UP
                : (servo < ELEV_DOWN_DEG) ? EL_DOWN
                                          : EL_NEUTRAL;

    // --- サンプルを送信 (既定: バイナリ・バッチ、F0: CSV 即時) ---
    uint32_t up_ms = (uint32_t)(now_us / 1000ull);
    if (telemetry_binary) {
      // バッチに積む。満杯なら batch_push 内で flush、そうでなくても一定間隔で
      // flush して低レート時の遅延を抑える。
      batch_push(seq, up_ms, bme, bno, h_est, v_est, a_z, (int)vstate,
                 (int)elev, servo);
      // batch_push が満杯で flush 済み (batch_count==0) か、一定間隔超過なら
      // flush。
      if (batch_count == 0 || now_us - last_flush_us >= BATCH_FLUSH_US) {
        flush_batch();
        last_flush_us = now_us;
      }
    } else {
      flush_batch(); // 形式切替直後に溜まっていれば吐き出す
      int len = snprintf(
          line, sizeof(line),
          "PICO,%lu,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%u,%d,%d,%ld\r\n",
          (unsigned long)seq, (unsigned long)up_ms,
          (long)scaled(bme.temp_c, 100.f), (long)scaled(bme.press_hpa, 100.f),
          (long)scaled(bme.alt_m, 1000.f), (long)scaled(h_est, 1000.f),
          (long)scaled(v_est, 1000.f), (long)scaled(a_z, 1000.f),
          (long)scaled(bno.heading, 100.f), (long)scaled(bno.roll, 100.f),
          (long)scaled(bno.pitch, 100.f), (unsigned)bno.calib, (int)vstate,
          (int)elev, (long)scaled(servo, 100.f));
      if (len < 0)
        len = 0;
      if (len > (int)sizeof(line))
        len = sizeof(line);
      ble_send(line, len);
    }

    seq++;
    next_us += telemetry_period_us; // 絶対グリッドを1周期進める
    uint64_t end_us = time_us_64();
    if ((int64_t)(next_us - end_us) < 0)
      next_us = end_us; // 処理が周期を超えた → 取りこぼし再同期
    else
      obj_api::yield_us(next_us - end_us); // 残り時間だけ待って真の周期を保つ
  }
}

} // namespace shizu
