// ===========================================================================
//  BNO055_DRIVER — 9 軸 IMU の Shizuku オブジェクト (core0 側: リング排出係)
// ===========================================================================
//  step2 (docs/sensor_stream_protocol.md §7) で I2C アクセスは core1
//  (core1_io.cpp) へ移った。本オブジェクトは I2C を一切触らず、
//    1) コア間データリングの唯一の consumer として 100Hz でレコードを排出し、
//       EUL/LIA/GRV の 3 レコード (同一 t_us) を bno055_sample_t へ再構成
//       (float 換算 /16, /100 はここで行う)
//    2) BARO/GROUND レコードを BME280 モジュールへ配る (bme280_on_baro/_ground)
//    3) コマンド (pause/read_mode/ffff_reject) をコマンドリングへ、較正
//       プロファイル save/load をサイドバンドへ転送する
//  外部インタフェース (メソッド ID / セマンティクス) は従来と完全互換。
//  TELEMETRY_SENDER / FLIGHT_CONTROLLER は無変更で動く。
// ===========================================================================
#include <core_ring.hpp>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <export_method.hpp>
#include <obj_api.hpp>
#include <object_headers/BNO055_DRIVER.hpp>
#include <pico/time.h>

namespace shizu {

static constexpr int CALIB_PROFILE_LEN = BNO055_CALIB_PROFILE_LEN; // 22

// ---- 最新サンプル ----------------------------------------------------------
static bno055_sample_t latest = {};

// ---- 較正 save/load の core0 側ミラー (calib_get が返す) ---------------------
static volatile bool g_calib_done = false;
static volatile bool g_calib_ok = false;
static uint8_t g_calib_buf[CALIB_PROFILE_LEN];

// push 先 (sink)。set_sample_sink / set_calib_sink で (obj<<16)|method を登録。
static uint32_t sample_sink_obj = 0xFFFF, sample_sink_method = 0;
static uint32_t calib_sink_obj = 0xFFFF, calib_sink_method = 0;

// ---- リング排出の状態 --------------------------------------------------------
// EUL/LIA/GRV は core1 が同一 t_us で 3 連続 push する。t_us で組にして 9 値
// 揃ったらサンプル確定 (途中で t が変わったら不完全組は捨てて数える)。
static int16_t pend[9];
static uint32_t pend_t = 0;
static uint8_t pend_mask = 0; // bit0=EUL bit1=LIA bit2=GRV
static uint32_t g_drop_count = 0;       // リング上書きで失われたレコード数
static uint32_t g_incomplete_count = 0; // 3 点組が揃わなかった回数
static uint8_t g_last_calib = 0;        // STATUS レコード由来
static uint8_t g_health = 0;            // STATUS: HEALTH_* ビット
static uint64_t g_last_motion_us = 0;   // 鮮度監視 (500ms 無音で valid=false)

// core0 → core1 コマンド送信 (協調スケジューラなので push は実質直列)。
static void send_cmd(uint8_t op, uint8_t arg) {
  core_ring::cmd_rec_t c = {};
  c.op = op;
  c.arg = arg;
  core_ring::g_cmd_ring.push(c);
}

// ===========================================================================
//  公開メソッド (ID/セマンティクスは従来どおり)
// ===========================================================================
static void method_read_latest(uint32_t _caller_obj_id,
                               uint32_t _caller_thread_id, uint32_t out_ptr,
                               uint32_t _arg1) {
  (void)_caller_obj_id;
  (void)_caller_thread_id;
  (void)_arg1;
  if (out_ptr == 0)
    return;
  memcpy((void *)(uintptr_t)out_ptr, (const void *)&latest, sizeof(latest));
}

// arg0: 0=ブロック読み / 非0=2B 個別読み (core1 へ転送)。
static void method_set_read_mode(uint32_t, uint32_t, uint32_t mode, uint32_t) {
  send_cmd(core_ring::CMD_SET_READ_MODE, mode != 0);
}

// arg0: 0=0xFFFF 破損を素通し / 非0=検出して据え置き (既定。core1 へ転送)。
static void method_set_ffff_reject(uint32_t, uint32_t, uint32_t on, uint32_t) {
  send_cmd(core_ring::CMD_SET_FFFF_REJECT, on != 0);
}

// arg0 = (obj<<16)|method。新サンプル push 先を登録。
static void method_set_sample_sink(uint32_t, uint32_t, uint32_t packed,
                                   uint32_t) {
  printf("[BNO055]set_sample_sink called\n");
  sample_sink_obj = (packed >> 16) & 0xFFFF;
  sample_sink_method = packed & 0xFFFF;
}

// arg0 = (obj<<16)|method。較正 save/load 完了の push 先を登録。
static void method_set_calib_sink(uint32_t, uint32_t, uint32_t packed,
                                  uint32_t) {
  printf("[BNO055]set_calib_sink called\n");
  calib_sink_obj = (packed >> 16) & 0xFFFF;
  calib_sink_method = packed & 0xFFFF;
}

// arg0: 非0=サンプリング一時停止 / 0=再開 (core1 へ転送)。
static void method_set_paused(uint32_t, uint32_t, uint32_t on, uint32_t) {
  send_cmd(core_ring::CMD_SET_PAUSED_BNO, on != 0);
}

// arg0: N=連続失敗 N 回目で 20Hz 退避 (1=毎回=旧, 既定5)。core1 へ転送。
static void method_set_fail_backoff(uint32_t, uint32_t, uint32_t n, uint32_t) {
  send_cmd(core_ring::CMD_SET_FAIL_BACKOFF, (uint8_t)(n & 0xFF));
}

// 較正オフセットの吸い出し要求 (実処理は core1。完了はサイドバンド ack で届く)。
static void method_calib_save(uint32_t, uint32_t, uint32_t, uint32_t) {
  g_calib_done = false;
  g_calib_ok = false;
  core_ring::g_calib_xfer.op = 1;
  __dmb(); // op を書いてから req 発行
  core_ring::g_calib_xfer.req_seq = core_ring::g_calib_xfer.req_seq + 1;
}

// arg0: uint8_t[22] への ptr。オフセットを取り込み core1 へ書き戻し要求。
static void method_calib_load(uint32_t, uint32_t, uint32_t in_ptr, uint32_t) {
  if (in_ptr == 0)
    return;
  memcpy(g_calib_buf, (const void *)(uintptr_t)in_ptr, CALIB_PROFILE_LEN);
  memcpy(core_ring::g_calib_xfer.data, g_calib_buf, CALIB_PROFILE_LEN);
  g_calib_done = false;
  g_calib_ok = false;
  core_ring::g_calib_xfer.op = 2;
  __dmb(); // data/op を書いてから req 発行
  core_ring::g_calib_xfer.req_seq = core_ring::g_calib_xfer.req_seq + 1;
}

// arg0: bno055_calib_xfer_t*。直近 save/load の done/ok とダンプを返す。
static void method_calib_get(uint32_t, uint32_t, uint32_t out_ptr, uint32_t) {
  if (out_ptr == 0)
    return;
  bno055_calib_xfer_t x;
  x.done = g_calib_done ? 1 : 0;
  x.ok = g_calib_ok ? 1 : 0;
  memcpy(x.data, g_calib_buf, CALIB_PROFILE_LEN);
  memcpy((void *)(uintptr_t)out_ptr, &x, sizeof(x));
}

// ===========================================================================
//  レコードディスパッチ
// ===========================================================================
// EUL/LIA/GRV の 3 点組が揃ったのでサンプル確定 (float 換算はここでだけ行う)。
static void commit_motion_sample() {
  bno055_sample_t s;
  s.seq = latest.seq + 1;
  s.heading = pend[0] / 16.0f; // 1/16 deg (デッドバンドは core1 で適用済み)
  s.roll = pend[1] / 16.0f;
  s.pitch = pend[2] / 16.0f;
  s.lax = pend[3] / 100.0f; // 1/100 m/s^2
  s.lay = pend[4] / 100.0f;
  s.laz = pend[5] / 100.0f;
  s.gx = pend[6] / 100.0f;
  s.gy = pend[7] / 100.0f;
  s.gz = pend[8] / 100.0f;
  s.calib = g_last_calib;
  s.valid = true;
  latest = s;
  g_last_motion_us = time_us_64();
  if (sample_sink_obj != 0xFFFF)
    obj_api::svc(obj_api::svc_num::CALL_METHOD, sample_sink_obj,
                 sample_sink_method, (uint32_t)(uintptr_t)&latest);
}

// モーション 3 点組の部分レコードを取り込む。idx: 0=EUL 1=LIA 2=GRV。
static void accept_motion_part(int idx, const core_ring::record_t &rec) {
  if (pend_mask != 0 && rec.t_us != pend_t) {
    ++g_incomplete_count; // 組の途中で新しい t が来た → 前の組は不成立
    pend_mask = 0;
  }
  pend_t = rec.t_us;
  memcpy(&pend[idx * 3], rec.payload, 6);
  pend_mask |= (uint8_t)(1u << idx);
  if (pend_mask == 0x7) {
    pend_mask = 0;
    commit_motion_sample();
  }
}

static void dispatch_record(const core_ring::record_t &rec) {
  switch (rec.ch_id) {
  case core_ring::CH_EUL:
    accept_motion_part(0, rec);
    break;
  case core_ring::CH_LIA:
    accept_motion_part(1, rec);
    break;
  case core_ring::CH_GRV:
    accept_motion_part(2, rec);
    break;
  case core_ring::CH_BARO: {
    uint32_t pa;
    int16_t cc;
    memcpy(&pa, &rec.payload[0], 4);
    memcpy(&cc, &rec.payload[4], 2);
    bme280_on_baro(pa, cc, rec.t_us);
    break;
  }
  case core_ring::CH_GROUND: {
    uint32_t pa;
    int16_t cc;
    memcpy(&pa, &rec.payload[0], 4);
    memcpy(&cc, &rec.payload[4], 2);
    bme280_on_ground(pa, cc);
    break;
  }
  case core_ring::CH_STATUS: {
    g_last_calib = rec.payload[0];
    g_health = rec.payload[1];
    (void)g_health; // (現状は保持のみ。将来 STATUS のダウンリンクに使う)
    // I2C 失敗の毎秒デルタを可視化 (レート低下の裏取り: reads 低下と同期して
    // i2c_fail が増えていれば「失敗→バックオフ」が犯人と確定できる)。
    uint16_t fail;
    memcpy(&fail, &rec.payload[2], 2);
    static uint16_t prev_fail = 0;
    uint16_t dfail = (uint16_t)(fail - prev_fail); // uint16 でラップ安全
    prev_fail = fail;
    printf("[BNO055] status: i2c_fail=+%u (%u tot) recover=%u reinit=%u calib=0x%02X\n",
           (unsigned)dfail, (unsigned)fail, (unsigned)rec.payload[4],
           (unsigned)rec.payload[5], (unsigned)g_last_calib);
    break;
  }
  case core_ring::CH_DIAG: { // 0xFFFF 破損率 A/B (X0=block / X1=split で切替え)
    uint16_t reads, ffff;
    memcpy(&reads, &rec.payload[2], 2);
    memcpy(&ffff, &rec.payload[4], 2);
    unsigned pm = reads ? (unsigned)((uint32_t)ffff * 1000u / reads) : 0;
    printf("[BNO055] 0xFFFF diag: mode=%s reject=%u reads=%u ffff=%u (%u.%u%%)\n",
           rec.payload[0] ? "split" : "block", (unsigned)rec.payload[1],
           (unsigned)reads, (unsigned)ffff, pm / 10, pm % 10);
    break;
  }
  default:
    break;
  }
}

// ===========================================================================
//  オブジェクトエントリ (データリングの唯一の consumer スレッド)
// ===========================================================================
void BNO055_DRIVER::init() {
  printf("[BNO055] init (core1 stream consumer)\n");
  export_method<method_read_latest>(BNO055_DRIVER::METHOD_IDs::read_latest);
  export_method<method_set_read_mode>(BNO055_DRIVER::METHOD_IDs::set_read_mode);
  export_method<method_set_ffff_reject>(
      BNO055_DRIVER::METHOD_IDs::set_ffff_reject);
  export_method<method_set_sample_sink>(
      BNO055_DRIVER::METHOD_IDs::set_sample_sink);
  export_method<method_set_calib_sink>(
      BNO055_DRIVER::METHOD_IDs::set_calib_sink);
  export_method<method_set_paused>(BNO055_DRIVER::METHOD_IDs::set_paused);
  export_method<method_set_fail_backoff>(
      BNO055_DRIVER::METHOD_IDs::set_fail_backoff);
  export_method<method_calib_save>(BNO055_DRIVER::METHOD_IDs::calib_save);
  export_method<method_calib_load>(BNO055_DRIVER::METHOD_IDs::calib_load);
  export_method<method_calib_get>(BNO055_DRIVER::METHOD_IDs::calib_get);

  uint32_t last_ack = core_ring::g_calib_xfer.ack_seq;
  uint32_t last_drop_report = 0;

  // 排出周期は最高レートのチャンネル (NDOF 100Hz) に合わせた絶対グリッド。
  // 1 tick あたり定常 ~4 レコード (EUL/LIA/GRV + BARO/STATUS が時々)。
  uint64_t next = time_us_64();
  while (true) {
    next += 10000; // 100Hz の絶対グリッド

    // ---- データリング排出 (1 tick の上限 64 件: 遅延回復中も yield を守る) ----
    core_ring::record_t rec;
    for (int budget = 64;
         budget > 0 && core_ring::g_data_stream.hdl().pop(&rec, &g_drop_count);
         --budget)
      dispatch_record(rec);
    if (g_drop_count != last_drop_report) {
      printf("[BNO055] ring overrun: %lu records dropped (total)\n",
             (unsigned long)g_drop_count);
      last_drop_report = g_drop_count;
    }

    // ---- 較正サイドバンドの完了 (ack) 監視 ----
    uint32_t ack = core_ring::g_calib_xfer.ack_seq;
    if (ack != last_ack) {
      last_ack = ack;
      __dmb(); // ack 観測 → data/ok 読みの順序を保証
      g_calib_ok = (core_ring::g_calib_xfer.ok != 0);
      memcpy(g_calib_buf, core_ring::g_calib_xfer.data, CALIB_PROFILE_LEN);
      g_calib_done = true;
      // 結果を sink へ push (従来 handle_calib_cmd がやっていたのと同じ)。
      if (calib_sink_obj != 0xFFFF) {
        bno055_calib_xfer_t x;
        x.done = 1;
        x.ok = g_calib_ok ? 1 : 0;
        memcpy(x.data, g_calib_buf, CALIB_PROFILE_LEN);
        obj_api::svc(obj_api::svc_num::CALL_METHOD, calib_sink_obj,
                     calib_sink_method, (uint32_t)(uintptr_t)&x);
      }
    }

    // ---- 鮮度監視: 500ms モーションが来なければ valid=false (I2C 断相当) ----
    uint64_t now = time_us_64();
    if (latest.valid && now - g_last_motion_us > 500000)
      latest.valid = false;

    if ((int64_t)(next - now) < 0) {
      next = now;
      obj_api::yield(); // 周期超過でも必ず 1 回は譲る (無 yield 凍結の防止)
    } else
      obj_api::yield_until_us(next); // 締切に張り付いて true 100Hz(±数µs)
  }
}

} // namespace shizu
