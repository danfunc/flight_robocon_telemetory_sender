// ===========================================================================
//  BNO055_DRIVER — 9 軸 IMU の Shizuku オブジェクト (core0 側: リング排出係)
// ===========================================================================
//  step2 (docs/sensor_stream_protocol.md §7) で I2C アクセスは core1
//  (core1_io.cpp) へ移った。本オブジェクトは I2C を一切触らず、
//    1) コア間データストリームの唯一の consumer として 100Hz でレコードを排出し、
//       EUL/LIA/GRV の 3 レコード (同一 t_us) を bno055_sample_t へ再構成して
//       ID_BNO_SAMPLE へ流す (float 換算 /16, /100 はここで行う)
//    2) BARO/GROUND レコードを ID_BARO_RAW へ流す (consumer は BME280)
//    3) コマンド (pause/read_mode/ffff_reject) を g_cmd_stream へ、較正プロファイル
//       save/load を g_calib_req へ転送し、結果を g_calib_resp で受けて
//       ID_CALIB_RESULT へ流す
//  データの配りはすべて Shizuku ストリーム。旧 set_sample_sink / set_calib_sink の
//  CALL_METHOD push は廃止した — sink 方式は consumer のコードを producer のスレッド
//  とスタックで同期実行しており、「call_method は yield しないから原子的」という
//  不変条件に全員がぶら下がっていた。ストリームなら各 consumer が自分のスレッドで
//  自分の周期に drain するだけで済む。
//  read_latest / set_paused など一発の RPC は従来どおり call_method のまま。
// ===========================================================================
#include <core_ring.hpp>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <driver_streams.hpp>
#include <export_method.hpp>
#include <obj_api.hpp>
#include <object_headers/BNO055_DRIVER.hpp>
#include <pico/time.h>

namespace shizu {

static constexpr int CALIB_PROFILE_LEN = BNO055_CALIB_PROFILE_LEN; // 22
static_assert(CALIB_PROFILE_LEN == (int)core_ring::CALIB_PROFILE_LEN,
              "calib profile length must agree with the core1 stream record");

// ---- 最新サンプル ----------------------------------------------------------
static bno055_sample_t latest = {};

// ---- 較正 save/load の core0 側ミラー (calib_get が返す) ---------------------
static volatile bool g_calib_done = false;
static volatile bool g_calib_ok = false;
static uint8_t g_calib_buf[CALIB_PROFILE_LEN];

// ---- 出力ストリーム (カーネル登録型。consumer は open_wait で繋ぐ) ------------
// モーションは LOSSY: consumer が遅れたら古い姿勢より新しい姿勢が要る (鮮度優先)。
// baro / calib は LOSSLESS: GROUND (高度ゼロ点) や較正結果は落とすと状態が食い違う。
static stream::storage<bno055_sample_t, 16> g_motion_tx;
static stream::storage<drv_stream::baro_raw_t, 16, stream::LOSSLESS> g_baro_tx;
static stream::storage<bno055_calib_xfer_t, 4, stream::LOSSLESS> g_calib_tx;
static uint32_t g_cmd_drop = 0, g_baro_drop = 0, g_calib_drop = 0;

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

// core0 → core1 コマンド送信。producer は BNO055 と BME280 の 2 スレッドなので
// MP_PROD 版 (push_mp) で CAS 直列化する — 旧実装は「協調型単一コアだから push は
// 実質直列」に寄りかかっており、デュアルコア化でその前提が消えている。
static void send_cmd(uint8_t op, uint8_t arg) {
  core_ring::cmd_rec_t c = {};
  c.op = op;
  c.arg = arg;
  if (!core_ring::g_cmd_stream.hdl().push_mp(c))
    ++g_cmd_drop; // LOSSLESS 満杯 (深さ 16)。設定コマンドは低頻度なので実質起きない
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

// arg0: 非0=サンプリング一時停止 / 0=再開 (core1 へ転送)。
static void method_set_paused(uint32_t, uint32_t, uint32_t on, uint32_t) {
  send_cmd(core_ring::CMD_SET_PAUSED_BNO, on != 0);
}

// arg0: N=連続失敗 N 回目で 20Hz 退避 (1=毎回=旧, 既定5)。core1 へ転送。
static void method_set_fail_backoff(uint32_t, uint32_t, uint32_t n, uint32_t) {
  send_cmd(core_ring::CMD_SET_FAIL_BACKOFF, (uint8_t)(n & 0xFF));
}

// 較正オフセットの吸い出し要求 (実処理は core1。結果は g_calib_resp で届く)。
static void method_calib_save(uint32_t, uint32_t, uint32_t, uint32_t) {
  g_calib_done = false;
  g_calib_ok = false;
  core_ring::calib_req_t rq = {};
  rq.op = 1; // save
  core_ring::g_calib_req.hdl().push(rq);
}

// arg0: uint8_t[22] への ptr。オフセットを取り込み core1 へ書き戻し要求。
static void method_calib_load(uint32_t, uint32_t, uint32_t in_ptr, uint32_t) {
  if (in_ptr == 0)
    return;
  memcpy(g_calib_buf, (const void *)(uintptr_t)in_ptr, CALIB_PROFILE_LEN);
  g_calib_done = false;
  g_calib_ok = false;
  core_ring::calib_req_t rq = {};
  rq.op = 2; // load
  memcpy(rq.data, g_calib_buf, CALIB_PROFILE_LEN);
  core_ring::g_calib_req.hdl().push(rq);
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
  s.t_us = pend_t; // core1 が付けた標本時刻 (consumer の積分 dt はこれの差分)
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
  // LOSSY なので満杯でも黙って最古を上書きする (戻り値は常に true)。
  g_motion_tx.hdl().push(latest);
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
  // BARO (定常) と GROUND (高度ゼロ点) は 1 本のストリームへ載せる。別ストリームに
  // 割ると「ground が baro を追い越す」順序逆転が起き得るため (is_ground で区別)。
  case core_ring::CH_BARO:
  case core_ring::CH_GROUND: {
    drv_stream::baro_raw_t b = {};
    int16_t cc;
    memcpy(&b.press_pa, &rec.payload[0], 4);
    memcpy(&cc, &rec.payload[4], 2);
    b.temp_cc = cc;
    b.t_us = rec.t_us;
    b.is_ground = (rec.ch_id == core_ring::CH_GROUND) ? 1 : 0;
    if (!g_baro_tx.hdl().push(b))
      ++g_baro_drop; // LOSSLESS 満杯 = BME280 が長時間 drain していない
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
  export_method<method_set_paused>(BNO055_DRIVER::METHOD_IDs::set_paused);
  export_method<method_set_fail_backoff>(
      BNO055_DRIVER::METHOD_IDs::set_fail_backoff);
  export_method<method_calib_save>(BNO055_DRIVER::METHOD_IDs::calib_save);
  export_method<method_calib_load>(BNO055_DRIVER::METHOD_IDs::calib_load);
  export_method<method_calib_get>(BNO055_DRIVER::METHOD_IDs::calib_get);

  // 出力ストリームを登録する。consumer (TELEMETRY / BME280) は open_wait で繋ぐので
  // 起動順は問わない (BME280 は本オブジェクトより先に起動するが ID_BARO_RAW の
  // consumer、という順序でも成立する)。
  drv_stream::publish(drv_stream::ID_BNO_SAMPLE, &g_motion_tx.desc, "BNO055");
  drv_stream::publish(drv_stream::ID_BARO_RAW, &g_baro_tx.desc, "BNO055");
  drv_stream::publish(drv_stream::ID_CALIB_RESULT, &g_calib_tx.desc, "BNO055");

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
      // baro/calib/cmd は LOSSLESS なので drop は「consumer が排出していない」の証拠。
      // データストリームの overrun とは意味が違うので分けて出す。
      printf("[BNO055] stream drops: data=%lu baro=%lu calib=%lu cmd=%lu\n",
             (unsigned long)g_drop_count, (unsigned long)g_baro_drop,
             (unsigned long)g_calib_drop, (unsigned long)g_cmd_drop);
      last_drop_report = g_drop_count;
    }

    // ---- 較正結果 (core1 → g_calib_resp) の受領 ----
    // 自分のミラー (calib_get が返す) を更新してから、ダウンリンク用に
    // ID_CALIB_RESULT へ流す (旧 calib_sink への CALL_METHOD)。
    core_ring::calib_resp_t rs;
    while (core_ring::g_calib_resp.hdl().pop(&rs)) {
      g_calib_ok = (rs.ok != 0);
      memcpy(g_calib_buf, rs.data, CALIB_PROFILE_LEN);
      g_calib_done = true;
      bno055_calib_xfer_t x;
      x.done = 1;
      x.ok = g_calib_ok ? 1 : 0;
      memcpy(x.data, g_calib_buf, CALIB_PROFILE_LEN);
      if (!g_calib_tx.hdl().push(x))
        ++g_calib_drop;
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
