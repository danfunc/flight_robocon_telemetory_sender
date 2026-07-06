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
#include <object_headers/FLIGHT_CONTROLLER.hpp>
#include <object_headers/TELEMETRY_SENDER.hpp>
#include <object_id.hpp>
#include <pico/time.h>
#include <cmath> // 標準 libm (kernel が FP コンテキスト退避に対応したため soft_math は退役)

namespace shizu {

// ===========================================================================
//  制御パラメータ (altitude_fusion_wifi.c より移植)
// ===========================================================================
// 相補フィルタ係数 (慣性=加速度積分側の重み)。1.0 に近いほど気圧を信用せず、
// 加速度積分を優先する。気圧高度は室内気流/ドア開閉/プロペラ後流で数十cmの
// ノイズが乗りやすく、その差分で作る v_baro は特に暴れるため、既定より慣性寄りへ
// 振って「気圧(差分)の信用度」を落としている。ただし 1.0 にすると加速度バイアスの
// ドリフトを気圧が抑えられず発散するので、緩く気圧で引き戻す余地は残す。
static constexpr float ALPHA_H = 0.90f; // 高度: 気圧の反映を 20%→10% に低減
static constexpr float ALPHA_V = 0.93f; // 速度: 気圧差分速度の反映を 25%→7% に低減
static constexpr float A_Z_CLIP = 5.0f; // 世界鉛直加速度のクリップ [m/s^2]

// 水平速度 (vx,vy) は GPS 等の位置基準が無く、加速度の積分だけでは
// ドリフトが止まらない。そこでリーク積分で有界化する: 静止時は 0 へ緩く戻り、
// 巡航時はやや低め (参考値) に出る。加速度バイアス a_b に対する定常オフセットは
// a_b×τ (例: 0.05 m/s² × 2s = 0.1 m/s)。τ を小さくするほど「微小に減衰し続ける」
// 側に倒れてドリフト残差が減り、代わりに巡航中の読みが下振れする。位置基準が
// 無い以上ドリフト抑制を優先し 3.0 → 2.0 に短縮。
static constexpr float HVEL_LEAK_TAU = 2.0f; // 水平速度リーク時定数 [s]

// テレメトリの elev 方向ヒントを FLIGHT_CONTROLLER のエレベータ指令から導く閾値 [deg]。
// (飛行制御の本体は FLIGHT_CONTROLLER に分離済み。ここは融合+通信のみ担う。)
static constexpr float ELEV_UP_DEG = +5.0f, ELEV_DOWN_DEG = -5.0f;

// サンプル周期 (R コマンドで変更可)。既定 20ms = 50Hz。バッチ送信なので、この
// レートで生成したサンプルは BATCH_FLUSH_US ごとにまとめて 1
// フレームで送られる。
static volatile uint32_t telemetry_period_us = 10000;

// 送信フォーマット。true=バイナリフレーム(既定), false=CSV テキスト。F
// コマンドで切替。 バイナリは ~44B/フレームで、CSV(~120B) の約 1/3。BLE
// の実効上限 ~140kbps に対し 余裕を持って高レート送信できる。
static volatile bool telemetry_binary = true;

enum Elev { EL_NEUTRAL = 0, EL_UP = 1, EL_DOWN = 2 };

// ===========================================================================
//  BLE 送信ヘルパー
// ===========================================================================
static void ble_send(const char *s, int len) {
  if (len <= 0)
    return;
  // バッファのアドレス+長さを 1 個のポインタで渡して一括送信(1 バイトずつ
  // call_method する代わり)。記述子 b はこの同期呼び出しの間だけ有効ならよい。
  ble_tx_buf_t b = {(const uint8_t *)s, (uint32_t)len};
  call_method(object_ids::BLE_UART_DRIVER, BLE_UART_DRIVER::METHOD_IDs::send_buf,
              (uint32_t)(uintptr_t)&b);
}

// スループット試験中はセンサのサンプリングを止め、I2C/融合に食われる時間を帯域へ
// 回す(協調スケジューラなので他スレッドの停止はできないが、無駄な仕事は消せる)。
static void set_sensors_paused(bool paused) {
  uint32_t v = paused ? 1u : 0u;
  call_method(object_ids::BNO055_DRIVER, BNO055_DRIVER::METHOD_IDs::set_paused, v);
  call_method(object_ids::BME280_DRIVER, BME280_DRIVER::METHOD_IDs::set_paused, v);
}

// float を四捨五入して整数スケールに落とす (%f を使わず送るため)。
static inline int32_t scaled(float x, float scale) {
  float v = x * scale;
  return (int32_t)(v >= 0 ? v + 0.5f : v - 0.5f);
}

// int16 に飽和させて詰める (加速度成分など ±32.767 を超えうる値の保護)。
static inline int16_t scaled16(float x, float scale) {
  int32_t v = scaled(x, scale);
  if (v > 32767)
    v = 32767;
  if (v < -32768)
    v = -32768;
  return (int16_t)v;
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

// ---- 較正オフセット保存/復元 (push 受信) ------------------------------------
// CS/CL を BNO へ要求し、完了は on_calib_result で push 受信する(ポーリングしない)。
// ダウンリンク自体は BLE バイト生成を単一スレッドに保つため送信ループ側で行う。
static bool calib_pending_save = false;      // true=save(CDUMP), false=load(CLOAD)
static volatile bool calib_dl_ready = false; // 結果を受領しダウンリンク待ち
static volatile uint8_t calib_dl_ok = 0;
static uint8_t calib_dl_data[BNO055_CALIB_PROFILE_LEN];

// ---- RX 行アセンブリ -------------------------------------------------------
static char rxline[80];
static uint32_t rxlen = 0;

// 傾き補正済みの世界鉛直加速度 (重力ベクトルへの線形加速度の射影)。
static float world_z_accel(const bno055_sample_t &b) {
  float gn = sqrtf(b.gx * b.gx + b.gy * b.gy + b.gz * b.gz);
  if (gn < 1.0f)
    return 0.0f;
  return (b.lax * b.gx + b.lay * b.gy + b.laz * b.gz) / gn;
}

// 機体系の線形加速度 (lax,lay,laz) を世界系の水平 北/東 成分へ回す。euler
// (ψ=heading, θ=pitch, φ=roll) から作った body→NED DCM の上 2 行を使う。
// 速度の大きさ算出が目的なので、絶対方位の厳密さより「非回転な一貫した水平系」で
// 積分できることを優先している (ヨー回転を打ち消す ψ 回転が本質)。
static void world_horiz_accel(const bno055_sample_t &b, float &a_n, float &a_e) {
  const float d2r = 0.017453292519943f;
  float ps = b.heading * d2r, th = b.pitch * d2r, ph = b.roll * d2r;
  float sp = sinf(ps), cp = cosf(ps);
  float st = sinf(th), ct = cosf(th);
  float sr = sinf(ph), cr = cosf(ph);
  float ax = b.lax, ay = b.lay, az = b.laz;
  a_n = ax * (ct * cp) + ay * (sr * st * cp - cr * sp) + az * (cr * st * cp + sr * sp);
  a_e = ax * (ct * sp) + ay * (sr * st * sp + cr * cp) + az * (cr * st * sp - sr * cp);
}

// FLIGHT_CONTROLLER へコマンドを 1 個転送する (arm/目標設定)。
static void fc_cmd(uint8_t kind, float value) {
  flight_cmd_t c;
  c.kind = kind;
  c.value = value;
  call_method(object_ids::FLIGHT_CONTROLLER,
              FLIGHT_CONTROLLER::METHOD_IDs::set_command,
              (uint32_t)(uintptr_t)&c);
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
    set_sensors_paused(true); // 試験中はセンサを止めて帯域に振る
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
      calib_pending_save = true; // 結果は on_calib_result に push される
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
        calib_pending_save = false; // 結果は on_calib_result に push される
        printf("[TELEMETRY] calib load requested\n");
      } else {
        ble_send("CLOAD err=parse\r\n", 16);
      }
    }
    break;
  }
  case 'A': { // 制御 arm/disarm: "A1"=arm, "A0"=disarm
    bool arm = !(rxlen >= 2 && rxline[1] == '0');
    fc_cmd(arm ? 1u : 0u, 0.f);
    printf("[TELEMETRY] flight %s\n", arm ? "arm" : "disarm");
    break;
  }
  case 'G': { // 制御目標: "G<sub><signed int>"
    // sub= h:高度[cm] p:ピッチ[cdeg] y:方位[cdeg] t:スロットルトリム[permil]
    if (rxlen >= 3) {
      char sub = rxline[1];
      int sign = 1;
      uint32_t i = 2;
      if (rxline[i] == '-') {
        sign = -1;
        ++i;
      }
      long v = 0;
      bool any = false;
      for (; i < rxlen; ++i) {
        if (rxline[i] < '0' || rxline[i] > '9')
          break;
        v = v * 10 + (rxline[i] - '0');
        any = true;
      }
      if (any) {
        float fv = (float)(sign * v);
        switch (sub) {
        case 'h': fc_cmd(2, fv / 100.f); break;    // cm -> m
        case 'p': fc_cmd(3, fv / 100.f); break;    // cdeg -> deg
        case 'y': fc_cmd(4, fv / 100.f); break;    // cdeg -> deg
        case 't': fc_cmd(5, fv / 1000.f); break;   // permil -> [0..1]
        default: break;
        }
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
    set_sensors_paused(false); // 試験終了 → センサ再開
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
  int32_t vel_mm_s;      // 上下速度 (vz) [mm/s]
  int32_t speed_mm_s;    // 対地速度 sqrt(vx^2+vy^2+vz^2) [mm/s]
  int32_t az_mm_s2;      // 融合に使う世界鉛直加速度 [mm/s^2]
  int16_t lax_mm_s2;     // 線形加速度 X (重力除去済) [mm/s^2]
  int16_t lay_mm_s2;     // 線形加速度 Y [mm/s^2]
  int16_t laz_mm_s2;     // 線形加速度 Z [mm/s^2]
  int16_t gx_mm_s2;      // 重力ベクトル X [mm/s^2]
  int16_t gy_mm_s2;      // 重力ベクトル Y [mm/s^2]
  int16_t gz_mm_s2;      // 重力ベクトル Z [mm/s^2]
  uint16_t heading_cdeg; // 方位 [1/100 deg]
  int16_t roll_cdeg;     // ロール [1/100 deg]
  int16_t pitch_cdeg;    // ピッチ [1/100 deg]
  uint8_t calib;         // 較正 (SYS<<6|GYR<<4|ACC<<2|MAG)
  uint8_t vstate;        // 上下状態 (FLIGHT_CONTROLLER 由来)
  uint8_t elev;          // エレベータ方向ヒント (0 中立/1 上げ/2 下げ)
  int16_t servo_cdeg;    // エレベータ指令 [1/100 deg] (FLIGHT_CONTROLLER 由来)
  int16_t rudder_cdeg;   // ラダー指令 [1/100 deg]
  uint8_t thr_pct;       // スロットル指令 [0..100] (= throttle*100)
};
// クライアント (struct.unpack '<HhIiiiiihhhhhhHhhBBBhhB') と一致させる。
static_assert(sizeof(batch_sample_t) == 54, "batch_sample_t must be 54 bytes");

// 1 バッチに詰める最大サンプル数。CYW43 は notify 244B (LL 251B = DLE 上限に
// 1 パケットで収まるサイズ) までが安全圏 (btstack_config.h の
// HCI_ACL_PAYLOAD_SIZE 参照。512B notify は切断/再接続で BT コアが固まる既知
// バグを踏む)。17+4*54+2=235B ≤ 244B で、スタッフィング少なめなら 1 フレーム
// = 1 notify に乗る。最悪 2 倍スタッフでも 2*235+2=472 < TX_LINE_MAX(1024)。
static constexpr int BATCH_MAX = 4;
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
                       float speed, float a_z, const control_out_t &ctrl) {
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
  s.speed_mm_s = scaled(speed, 1000.f);
  s.az_mm_s2 = scaled(a_z, 1000.f);
  s.lax_mm_s2 = scaled16(bno.lax, 1000.f);
  s.lay_mm_s2 = scaled16(bno.lay, 1000.f);
  s.laz_mm_s2 = scaled16(bno.laz, 1000.f);
  s.gx_mm_s2 = scaled16(bno.gx, 1000.f);
  s.gy_mm_s2 = scaled16(bno.gy, 1000.f);
  s.gz_mm_s2 = scaled16(bno.gz, 1000.f);
  s.heading_cdeg = (uint16_t)scaled(bno.heading, 100.f);
  s.roll_cdeg = (int16_t)scaled(bno.roll, 100.f);
  s.pitch_cdeg = (int16_t)scaled(bno.pitch, 100.f);
  s.calib = bno.calib;
  s.vstate = ctrl.vstate;
  s.elev = (uint8_t)((ctrl.elevator > ELEV_UP_DEG)     ? EL_UP
                     : (ctrl.elevator < ELEV_DOWN_DEG) ? EL_DOWN
                                                       : EL_NEUTRAL);
  s.servo_cdeg = (int16_t)scaled(ctrl.elevator, 100.f);
  s.rudder_cdeg = (int16_t)scaled(ctrl.rudder, 100.f);
  float thr = ctrl.throttle < 0.f ? 0.f : (ctrl.throttle > 1.f ? 1.f : ctrl.throttle);
  s.thr_pct = (uint8_t)scaled(thr, 100.f);
  batch_count++;
  if (batch_count >= BATCH_MAX)
    flush_batch();
}

// ===========================================================================
//  融合状態 (push ハンドラが更新、送信ループが読む)
// ---------------------------------------------------------------------------
//  協調スケジューラ単一コアなので、push ハンドラは yield せず原子的に走る。送信
//  ループも状態の読み出し〜パケット化の間 yield しないため、torn read は起きない。
// ===========================================================================
static float g_h_est = 0.f, g_v_est = 0.f, g_h_baro_prev = 0.f;
static float g_last_a_z = 0.f;
static float g_vx_est = 0.f, g_vy_est = 0.f; // 世界系 水平速度(北/東) [m/s] リーク積分
static uint64_t g_last_bno_us = 0, g_last_baro_us = 0;
static uint32_t g_last_bme_seq = 0;
static bno055_sample_t g_bno = {}; // 直近の BNO サンプル(送信スナップ用)
static bme280_sample_t g_bme = {}; // 直近の BME サンプル

// BNO055 から新サンプルが push される。慣性鉛直加速度を 100Hz フルレートで積分。
static void handle_bno_push(uint32_t, uint32_t, uint32_t ptr, uint32_t) {
  if (ptr == 0)
    return;
  memcpy(&g_bno, (const void *)(uintptr_t)ptr, sizeof(g_bno));
  if (!g_bno.valid)
    return;
  uint64_t now = time_us_64();
  float dt = (float)(now - g_last_bno_us) / 1e6f;
  g_last_bno_us = now;
  if (dt <= 0.f || dt > 0.5f)
    dt = 0.01f; // 異常 dt のガード(初回含む)
  float a_z = world_z_accel(g_bno);
  if (a_z > A_Z_CLIP)
    a_z = A_Z_CLIP;
  if (a_z < -A_Z_CLIP)
    a_z = -A_Z_CLIP;
  g_h_est += g_v_est * dt + 0.5f * a_z * dt * dt;
  g_v_est += a_z * dt;
  g_last_a_z = a_z;

  // 水平速度: 世界系の北/東加速度を積分し、位置基準が無いぶんリークで有界化する。
  float a_n, a_e;
  world_horiz_accel(g_bno, a_n, a_e);
  float leak = 1.0f - dt / HVEL_LEAK_TAU;
  if (leak < 0.0f)
    leak = 0.0f;
  g_vx_est = (g_vx_est + a_n * dt) * leak;
  g_vy_est = (g_vy_est + a_e * dt) * leak;
}

// BME280 から新サンプルが push される。気圧高度で相補補正(BME の実レート ~21Hz)。
static void handle_bme_push(uint32_t, uint32_t, uint32_t ptr, uint32_t) {
  if (ptr == 0)
    return;
  memcpy(&g_bme, (const void *)(uintptr_t)ptr, sizeof(g_bme));
  if (!g_bme.valid || g_bme.seq == g_last_bme_seq)
    return;
  uint64_t now = time_us_64();
  float baro_dt = (float)(now - g_last_baro_us) / 1e6f;
  if (baro_dt <= 0.f)
    baro_dt = 0.02f;
  g_last_baro_us = now;
  g_last_bme_seq = g_bme.seq;
  g_h_est = ALPHA_H * g_h_est + (1.0f - ALPHA_H) * g_bme.alt_m;
  float v_baro = (g_bme.alt_m - g_h_baro_prev) / baro_dt;
  g_v_est = ALPHA_V * g_v_est + (1.0f - ALPHA_V) * v_baro;
  g_h_baro_prev = g_bme.alt_m;
}

// BNO055 から較正 save/load 完了が push される。データを受け取りフラグを立てる
// (ダウンリンクは送信ループが行う)。
static void handle_calib_push(uint32_t, uint32_t, uint32_t ptr, uint32_t) {
  if (ptr == 0)
    return;
  bno055_calib_xfer_t x;
  memcpy(&x, (const void *)(uintptr_t)ptr, sizeof(x));
  calib_dl_ok = x.ok;
  memcpy(calib_dl_data, x.data, BNO055_CALIB_PROFILE_LEN);
  calib_dl_ready = true;
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

  // センサ/較正の push を受け取るハンドラを公開し、各ドライバへ sink を登録する
  // (以後 read_latest のポーリングは行わず、push されたサンプルで融合する)。
  export_method<handle_bno_push>(TELEMETRY_SENDER::METHOD_IDs::on_bno_sample);
  export_method<handle_bme_push>(TELEMETRY_SENDER::METHOD_IDs::on_bme_sample);
  export_method<handle_calib_push>(TELEMETRY_SENDER::METHOD_IDs::on_calib_result);
  auto pack = [](TELEMETRY_SENDER::METHOD_IDs m) {
    return ((uint32_t)object_ids::TELEMETRY_SENDER << 16) | (uint32_t)m;
  };
  call_method(object_ids::BNO055_DRIVER, BNO055_DRIVER::METHOD_IDs::set_sample_sink,
              pack(TELEMETRY_SENDER::METHOD_IDs::on_bno_sample));
  call_method(object_ids::BNO055_DRIVER, BNO055_DRIVER::METHOD_IDs::set_calib_sink,
              pack(TELEMETRY_SENDER::METHOD_IDs::on_calib_result));
  call_method(object_ids::BME280_DRIVER, BME280_DRIVER::METHOD_IDs::set_sample_sink,
              pack(TELEMETRY_SENDER::METHOD_IDs::on_bme_sample));

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

    // 較正 save/load 完了が push されていればダウンリンク(BLE 生成は本スレッドのみ)。
    if (calib_dl_ready) {
      calib_dl_ready = false;
      char cl[80];
      int n;
      if (calib_pending_save) {
        n = snprintf(cl, sizeof(cl), "CDUMP ");
        for (int i = 0; i < BNO055_CALIB_PROFILE_LEN; ++i)
          n += snprintf(cl + n, sizeof(cl) - n, "%02X", calib_dl_data[i]);
        n += snprintf(cl + n, sizeof(cl) - n, " ok=%d\r\n", calib_dl_ok);
      } else {
        n = snprintf(cl, sizeof(cl), "CLOAD ok=%d\r\n", calib_dl_ok);
      }
      ble_send(cl, n);
    }

    uint64_t now_us = time_us_64();
    float dt = (float)(now_us - prev_us) / 1e6f;
    prev_us = now_us;
    if (dt <= 0.f || dt > 0.5f)
      dt = 0.04f; // 制御/送信周期相当の dt。異常値ガード。

    // 融合状態は push ハンドラ(on_bno_sample / on_bme_sample)が随時更新済み。ここ
    // ではスナップショットするだけ(プルもポーリングもしない)。
    float h_est = g_h_est, v_est = g_v_est, a_z = g_last_a_z;
    float vx = g_vx_est, vy = g_vy_est;
    float speed = sqrtf(vx * vx + vy * vy + v_est * v_est);

    // --- 飛行制御は FLIGHT_CONTROLLER に分離。融合状態を push し、最新指令を読む。
    // call_method は yield せず同期実行されるので、スタック上の fst/ctrl は有効。---
    flight_state_t fst;
    fst.dt = dt;
    fst.pitch = g_bno.pitch;
    fst.roll = g_bno.roll;
    fst.heading = g_bno.heading;
    fst.alt = h_est;
    fst.vel = v_est;
    fst.valid = g_bno.valid;
    call_method(object_ids::FLIGHT_CONTROLLER,
                FLIGHT_CONTROLLER::METHOD_IDs::on_state,
                (uint32_t)(uintptr_t)&fst);
    control_out_t ctrl = {};
    call_method(object_ids::FLIGHT_CONTROLLER,
                FLIGHT_CONTROLLER::METHOD_IDs::read_control,
                (uint32_t)(uintptr_t)&ctrl);
    int elevdir = (ctrl.elevator > ELEV_UP_DEG)     ? EL_UP
                  : (ctrl.elevator < ELEV_DOWN_DEG) ? EL_DOWN
                                                    : EL_NEUTRAL;
    float thr = ctrl.throttle < 0.f ? 0.f : (ctrl.throttle > 1.f ? 1.f : ctrl.throttle);

    // --- サンプルを送信 (既定: バイナリ・バッチ、F0: CSV 即時) ---
    uint32_t up_ms = (uint32_t)(now_us / 1000ull);
    if (telemetry_binary) {
      // バッチに積む。満杯なら batch_push 内で flush、そうでなくても一定間隔で
      // flush して低レート時の遅延を抑える。
      batch_push(seq, up_ms, g_bme, g_bno, h_est, v_est, speed, a_z, ctrl);
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
          "PICO,%lu,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,"
          "%ld,%ld,%u,%d,%d,%ld,%ld,%d\r\n",
          (unsigned long)seq, (unsigned long)up_ms,
          (long)scaled(g_bme.temp_c, 100.f), (long)scaled(g_bme.press_hpa, 100.f),
          (long)scaled(g_bme.alt_m, 1000.f), (long)scaled(h_est, 1000.f),
          (long)scaled(v_est, 1000.f), (long)scaled(speed, 1000.f),
          (long)scaled(a_z, 1000.f),
          (long)scaled(g_bno.lax, 1000.f), (long)scaled(g_bno.lay, 1000.f),
          (long)scaled(g_bno.laz, 1000.f), (long)scaled(g_bno.gx, 1000.f),
          (long)scaled(g_bno.gy, 1000.f), (long)scaled(g_bno.gz, 1000.f),
          (long)scaled(g_bno.heading, 100.f), (long)scaled(g_bno.roll, 100.f),
          (long)scaled(g_bno.pitch, 100.f), (unsigned)g_bno.calib,
          (int)ctrl.vstate, (int)elevdir, (long)scaled(ctrl.elevator, 100.f),
          (long)scaled(ctrl.rudder, 100.f), (int)scaled(thr, 100.f));
      if (len < 0)
        len = 0;
      if (len > (int)sizeof(line))
        len = sizeof(line);
      ble_send(line, len);
    }

    seq++;
    next_us += telemetry_period_us; // 絶対グリッドを1周期進める
    uint64_t end_us = time_us_64();
    if ((int64_t)(next_us - end_us) < 0) {
      next_us = end_us; // 処理が周期を超えた → 取りこぼし再同期
      obj_api::yield(); // 超過が続いても必ず 1 回は譲る (無 yield 凍結の防止)
    } else
      obj_api::yield_us(next_us - end_us); // 残り時間だけ待って真の周期を保つ
  }
}

} // namespace shizu
