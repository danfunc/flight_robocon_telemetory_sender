// ===========================================================================
//  ota — BLE で受け取ってステージング領域へ置く
// ===========================================================================
//  設計の理由は ota.hpp 冒頭。
#include "ota.hpp"
#include "shizuku/kernel.hpp"
#include "shizuku/object_api.hpp"
#include "shizuku/stream.hpp"
extern "C" {
#include "boot/picoboot_constants.h"
#include "hardware/flash.h"
#include "hardware/regs/addressmap.h"
#include "hardware/sync.h"
#include "pico/bootrom.h"
#include "pico/flash.h"
}
#include <cstdint>
#include <cstdio>
#include <cstring>

// ★commit のコピー先。**リンカに聞く** —— 0 番地決め打ちにしないのは
//   ota.hpp 冒頭の理由 (自分がいる場所がそのまま答え)。
extern "C" {
extern uint8_t __flash_binary_start[];
extern uint8_t __flash_binary_end[];
}

namespace ota {
namespace {

using ARCH = shizuku::KERNEL::ARCH;
using BOARD = shizuku::KERNEL::BOARD;

struct api_result {
  uintptr_t error;
  uintptr_t value;
};

api_result api(shizuku::object_api number, uintptr_t a1 = 0, uintptr_t a2 = 0,
               uintptr_t a3 = 0) {
  const auto result = ARCH::syscall((uintptr_t)number, a1, a2, a3);
  return {result.error, result.value};
}

uintptr_t export_method(method m, uintptr_t entry) {
  return api(shizuku::object_api::EXPORT_METHOD, (uintptr_t)m, entry).error;
}

// ---- 出口: 進捗・結果の行 ----------------------------------------------------
shizuku::stream::storage<frame_t, 8> g_out;
uintptr_t g_out_id = 0;

// ---- 入口: ble_uart の OTA characteristic から来る生バイト -------------------
uintptr_t g_in_id = xno::NO_STREAM;
shizuku::stream::handle<frame_t> g_in;

void say(const char *text) {
  frame_t f{};
  uint32_t n = (uint32_t)strlen(text);
  if (n > sizeof(f.data))
    n = sizeof(f.data);
  f.len = (uint16_t)n;
  memcpy(f.data, text, n);
  g_out.hdl().push(f);
  BOARD::diag_printf("[OTA] %s", text);
}

// ---- CRC32 (IEEE, 反転あり)。表は持たない (4 ビットずつ、16 語の表で十分速い) --
const uint32_t CRC_NIBBLE[16] = {
    0x00000000, 0x1DB71064, 0x3B6E20C8, 0x26D930AC, 0x76DC4190, 0x6B6B51F4,
    0x4DB26158, 0x5005713C, 0xEDB88320, 0xF00F9344, 0xD6D6A3E8, 0xCB61B38C,
    0x9B64C2B0, 0x86D3D2D4, 0xA00AE278, 0xBDBDF21C};

uint32_t crc32_update(uint32_t crc, const uint8_t *data, uint32_t length) {
  for (uint32_t i = 0; i < length; ++i) {
    crc ^= data[i];
    crc = (crc >> 4) ^ CRC_NIBBLE[crc & 0x0F];
    crc = (crc >> 4) ^ CRC_NIBBLE[crc & 0x0F];
  }
  return crc;
}

// ---- flash 書き込み ----------------------------------------------------------
// ★`flash_safe_execute` は「もう片方のコアが lockout victim 登録済み」を要求する
//   (pico_flash/flash.c:184)。**core1 が起きていないと assert で停止し、しかも
//   壊れた状態が flash に残って picotool erase --all でしか戻らない**
//   (2026-08-24 に別件で踏んだ)。合成側は start_secondary_core() を
//   peripherals 登録より前に呼んでいるので、ここへ来る時点で前提は満たされている。
// ★**1 セクタ (4KB) を 1 回のロックにまとめる**。以前はページ (256B) ごとに
//   `flash_safe_execute` を呼んでいて、396KB の転送で **約 1650 回**も両コアを
//   止めていた。1 回あたりは短くても、ロックの handshake と IRQ 禁止が
//   その回数だけ BLE に割り込むので、長い転送でリンクが保たなかった。
//   消去の粒度がセクタである以上、セクタ単位が自然なまとまりで、
//   ロック回数は 97 回 (17 分の 1) に落ちる。
//   ★★これ以上大きくまとめない (例: 64KB)。16 セクタ消去 + 256 ページ書き込みで
//     **1 回の停止が約 1 秒**になり、今度は BLE の supervision timeout や
//     write の応答待ちに引っかかる。「止める回数を減らす」のが目的であって
//     「1 回を長くする」ことではない。
struct sector_op {
  uint32_t offset;      // flash 先頭からのオフセット (セクタ境界)
  const uint8_t *data;  // FLASH_SECTOR_SIZE バイト
};

void sector_mutate(void *param) {
  const auto *op = (const sector_op *)param;
  ::flash_range_erase(op->offset, FLASH_SECTOR_SIZE);
  ::flash_range_program(op->offset, op->data, FLASH_SECTOR_SIZE);
}

// ---- 受信の状態 --------------------------------------------------------------
// ★ページ単位 (256B) に貯めてから焼く。BLE の 1 write は最大 244B なので
//   そのまま焼くとページ境界を跨いで効率も正しさも落ちる。
enum struct state : uint32_t { IDLE, HEADER, DATA, DONE, FAILED };

state g_state = state::IDLE;
uint8_t g_header[12];
uint32_t g_header_len = 0;
uint32_t g_total = 0;
uint32_t g_expect_crc = 0;
uint32_t g_received = 0;
uint32_t g_crc = 0xFFFFFFFFu;
uint8_t g_sector[FLASH_SECTOR_SIZE]; // 1 セクタぶん貯めてから 1 回で焼く
uint32_t g_sector_len = 0;
uint32_t g_last_report = 0;
uint32_t g_writes = 0;       // OTA characteristic への write を何回受けたか
uint32_t g_write_bytes = 0;  // その合計バイト
uint32_t g_last_write_report = 0;

uint32_t read_le32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

void reset_transfer() {
  g_state = state::IDLE;
  g_header_len = 0;
  g_total = 0;
  g_expect_crc = 0;
  g_received = 0;
  g_crc = 0xFFFFFFFFu;
  g_sector_len = 0;
  g_last_report = 0;
}

// 貯まった 1 セクタを焼く。★消去と書き込みを**同じロックの中**でやる
//   (別々に呼ぶとロック回数が倍になり、間に他が割り込む窓もできる)。
bool commit_sector() {
  if (g_sector_len == 0)
    return true;
  // 端数は 0xFF で埋める (消去後と同じ値にしておく)。
  if (g_sector_len < FLASH_SECTOR_SIZE)
    memset(g_sector + g_sector_len, 0xFF, FLASH_SECTOR_SIZE - g_sector_len);
  const uint32_t at = g_received - g_sector_len; // このセクタの先頭バイト位置
  const uint32_t offset =
      STAGING_OFFSET + (at / FLASH_SECTOR_SIZE) * FLASH_SECTOR_SIZE;
  sector_op op{offset, g_sector};
  if (::flash_safe_execute(sector_mutate, &op, 5000) != PICO_OK) {
    say("flash write failed\n");
    g_state = state::FAILED;
    return false;
  }
  g_sector_len = 0;
  return true;
}

// ---- commit: ステージング → 本体 ---------------------------------------------
// ★ここから先は**戻ってこられない**。自分がいる領域を消すので、消し始めた
//   あとに flash の命令を 1 つでも実行したら死ぬ。守るべきは 3 つ:
//
//   1. **コピーのループも、そこから呼ぶものも全部 RAM に居ること**。
//      `__no_inline_not_in_flash_func` で .time_critical (= RAM) に置く。
//      SDK の flash_range_erase / flash_range_program は元から RAM 常駐。
//      ★**memcpy は RAM 常駐ではない**。素の代入ループが memcpy 呼び出しへ
//        畳まれると、その瞬間に flash へ飛んで死ぬので、volatile ポインタで
//        書いて畳ませない。ROM の reboot も、ここで探すのではなく
//        **呼ぶ前に引いておいて**関数ポインタで渡す。
//
//   2. 読み元 (ステージング) は XIP 経由で読む。erase/program は終わりに
//      XIP を張り直すので、セクタごとに **「読む → 消す → 書く」** の順なら
//      読みは必ず XIP が生きている瞬間に来る。逆順にはしない。
//
//   3. 焼き終わったら return しない。戻り先 (flash_safe_execute の後半) は
//      もう別の像なので、その場で ROM の reboot を呼ぶ。
//
// ★止める回数の話 (ステージング側と同じ) はここでは効かない。commit 中は
//   BLE も何も動いていないし、動かす必要もない。セクタ単位なのは消去の
//   粒度がセクタだから、というだけ。
using reboot_fn = int (*)(uint32_t, uint32_t, uint32_t, uint32_t);

struct commit_op {
  uint32_t src_offset;    // ステージングの先頭 (flash オフセット)
  uint32_t dst_offset;    // 本体の先頭 (flash オフセット)
  uint32_t copy_sectors;  // 新しい像が占めるセクタ数
  uint32_t erase_sectors; // 消すセクタ数 (>= copy_sectors。旧像の残りぶん)
  uint8_t *buffer;        // FLASH_SECTOR_SIZE バイトの RAM
  reboot_fn reboot;       // ROM の reboot (呼ぶ前に引いてある)
};

void __no_inline_not_in_flash_func(commit_blast)(void *param) {
  // ★引数も 1 語ずつ読む (構造体まるごとのコピーは memcpy に化けうる)。
  const volatile commit_op *op = (const volatile commit_op *)param;
  const uint32_t src_offset = op->src_offset;
  const uint32_t dst_offset = op->dst_offset;
  const uint32_t copy_sectors = op->copy_sectors;
  const uint32_t erase_sectors = op->erase_sectors;
  volatile uint32_t *const buffer = (volatile uint32_t *)op->buffer;
  const reboot_fn reboot = op->reboot;

  for (uint32_t s = 0; s < erase_sectors; ++s) {
    const uint32_t at = s * FLASH_SECTOR_SIZE;
    if (s < copy_sectors) {
      const volatile uint32_t *src =
          (const volatile uint32_t *)(XIP_BASE + src_offset + at);
      for (uint32_t i = 0; i < FLASH_SECTOR_SIZE / sizeof(uint32_t); ++i)
        buffer[i] = src[i];
    }
    ::flash_range_erase(dst_offset + at, FLASH_SECTOR_SIZE);
    if (s < copy_sectors)
      ::flash_range_program(dst_offset + at, (const uint8_t *)buffer,
                            FLASH_SECTOR_SIZE);
  }

  reboot(REBOOT2_FLAG_REBOOT_TYPE_NORMAL | REBOOT2_FLAG_NO_RETURN_ON_SUCCESS,
         10, 0, 0);
  while (true)
    __asm volatile("wfi");
}

// ステージングを **flash から読み直して** CRC を確かめる。
// ★受信直後にも CRC は見ているが、あれは「受け取ったバイト列」の検証で、
//   flash に本当にその通り書けたかは別の話。本体を消す前にもう一度、
//   今度は焼いた結果そのもので確かめる。
bool staged_matches(uint32_t total, uint32_t want) {
  const uint8_t *staged = (const uint8_t *)(XIP_BASE + STAGING_OFFSET);
  uint32_t crc = 0xFFFFFFFFu;
  for (uint32_t done = 0; done < total;) {
    uint32_t n = total - done;
    if (n > 32 * 1024)
      n = 32 * 1024; // ★長い計算なので刻んで譲る (他スレッドの締切を潰さない)
    crc = crc32_update(crc, staged + done, n);
    done += n;
    api(shizuku::object_api::YIELD);
  }
  return (crc ^ 0xFFFFFFFFu) == want;
}

void reject_commit(const char *why) {
  char line[96];
  if (snprintf(line, sizeof(line), "commit rejected: %s\n", why) > 0)
    say(line);
  g_state = state::FAILED;
}

uint32_t sectors_for(uint32_t bytes) {
  return (bytes + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE;
}

void begin_commit() {
  const uint32_t total = read_le32(g_header + 4);
  const uint32_t want = read_le32(g_header + 8);
  const uint32_t dst_offset =
      (uint32_t)((uintptr_t)__flash_binary_start - XIP_BASE);
  const uint32_t old_bytes =
      (uint32_t)((uintptr_t)__flash_binary_end - (uintptr_t)__flash_binary_start);

  if (total == 0 || total > STAGING_BYTES) {
    reject_commit("size");
    return;
  }
  // ★消す範囲がステージングに掛かったら、コピー元を自分で消すことになる。
  const uint32_t erase_sectors =
      sectors_for(total) > sectors_for(old_bytes) ? sectors_for(total)
                                                  : sectors_for(old_bytes);
  if (dst_offset + erase_sectors * FLASH_SECTOR_SIZE > STAGING_OFFSET) {
    reject_commit("would erase the staging area");
    return;
  }
  // ★「.uf2 を送ってしまった」を**焼く前に**捕まえる。生イメージなら先頭は
  //   ベクタテーブルで、[0] が SRAM の初期 SP、[1] が XIP の reset ハンドラ。
  //   uf2 の先頭は 'UF2\n' なので、この 1 行で弾ける。
  const uint32_t *head = (const uint32_t *)(XIP_BASE + STAGING_OFFSET);
  if ((head[0] & 0xFF000000u) != SRAM_BASE ||
      (head[1] & 0xFF000000u) != XIP_BASE) {
    reject_commit("staged image does not start with a vector table");
    return;
  }
  if (!staged_matches(total, want)) {
    reject_commit("staged CRC mismatch (本体は無傷)");
    return;
  }

  char line[128];
  if (snprintf(line, sizeof(line),
               "commit: %lu bytes -> 0x%lx (%lu sectors), no return\n",
               (unsigned long)total, (unsigned long)dst_offset,
               (unsigned long)erase_sectors) > 0)
    say(line);
  // ★この行が実際に BLE から出るまで待つ (say → logger → notify は非同期)。
  //   出ないまま消し始めると、ホストからは黙って切れたようにしか見えない。
  api(shizuku::object_api::SLEEP_US, 1000000);

  commit_op op{STAGING_OFFSET,
               dst_offset,
               sectors_for(total),
               erase_sectors,
               g_sector,
               (reboot_fn)rom_func_lookup_inline(ROM_FUNC_REBOOT)};
  if (op.reboot == nullptr) {
    reject_commit("no ROM reboot");
    return;
  }
  const int rc = ::flash_safe_execute(commit_blast, &op, 10000);
  // ★ここへ戻ってくるのは「ロックが取れなかった」ときだけ。取れていたら
  //   commit_blast の中で再起動しているので戻らない。
  if (snprintf(line, sizeof(line), "commit could not start (rc=%d)\n", rc) > 0)
    say(line);
  g_state = state::FAILED;
}

void begin_transfer() {
  g_total = read_le32(g_header + 4);
  g_expect_crc = read_le32(g_header + 8);
  if (g_total == 0 || g_total > STAGING_BYTES) {
    char line[96];
    if (snprintf(line, sizeof(line), "reject: size %lu (max %lu)\n",
                 (unsigned long)g_total, (unsigned long)STAGING_BYTES) > 0)
      say(line);
    g_state = state::FAILED;
    return;
  }
  char line[96];
  if (snprintf(line, sizeof(line), "begin: %lu bytes crc=%08lx\n",
               (unsigned long)g_total, (unsigned long)g_expect_crc) > 0)
    say(line);
  g_state = state::DATA;
}

void feed(const uint8_t *data, uint32_t length) {
  for (uint32_t i = 0; i < length; ++i) {
    if (g_state == state::IDLE || g_state == state::HEADER) {
      // マジックを探す。途中から繋いだ相手でも拾えるように、先頭 4 文字が
      // 合うまでは読み捨てる。★4 文字目がコマンド: 'U' = upload, 'C' = commit。
      const uint8_t prefix[3] = {'X', 'N', 'O'};
      const bool matches = g_header_len < 3   ? data[i] == prefix[g_header_len]
                           : g_header_len == 3 ? (data[i] == 'U' || data[i] == 'C')
                                               : true;
      if (!matches) {
        g_header_len = 0;
        continue;
      }
      g_header[g_header_len++] = data[i];
      g_state = state::HEADER;
      if (g_header_len == sizeof(g_header)) {
        g_header_len = 0;
        if (g_header[3] == 'C')
          begin_commit();  // ★成功すれば戻ってこない (再起動する)
        else
          begin_transfer();
      }
      continue;
    }
    if (g_state != state::DATA)
      return; // DONE / FAILED のあいだは捨てる (次のヘッダは reset 後に受ける)

    g_sector[g_sector_len++] = data[i];
    g_crc = crc32_update(g_crc, &data[i], 1);
    ++g_received;
    if (g_sector_len == FLASH_SECTOR_SIZE && !commit_sector())
      return;

    if (g_received >= g_total) {
      if (!commit_sector())
        return;
      const uint32_t crc = g_crc ^ 0xFFFFFFFFu;
      char line[112];
      if (crc == g_expect_crc) {
        g_state = state::DONE;
        if (snprintf(line, sizeof(line),
                     "done: %lu bytes crc=%08lx OK (staged at 0x%lx)\n",
                     (unsigned long)g_received, (unsigned long)crc,
                     (unsigned long)STAGING_OFFSET) > 0)
          say(line);
      } else {
        g_state = state::FAILED;
        if (snprintf(line, sizeof(line),
                     "CRC MISMATCH got=%08lx want=%08lx — 本体は無傷\n",
                     (unsigned long)crc, (unsigned long)g_expect_crc) > 0)
          say(line);
      }
      return;
    }
  }
}

// ---- エクスポートするメソッド --------------------------------------------------
uintptr_t method_set_input_stream(uintptr_t argument, uintptr_t, uintptr_t,
                                  uintptr_t) {
  g_in_id = argument;
  return 1;
}

uintptr_t method_get_stream(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  return g_out_id;
}

// ---- 受信ループ ---------------------------------------------------------------
constexpr uint32_t PERIOD_US = 2000;

uintptr_t poll_loop(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  api(shizuku::object_api::STREAM_BIND, g_out_id,
      (uintptr_t)shizuku::stream::role::PRODUCER);
  if (g_in_id != xno::NO_STREAM) {
    const auto opened = api(shizuku::object_api::STREAM_OPEN, g_in_id);
    if (opened.error == 0 && opened.value != 0) {
      g_in = shizuku::stream::handle<frame_t>(
          (shizuku::stream::descriptor *)opened.value);
      api(shizuku::object_api::STREAM_BIND, g_in_id,
          (uintptr_t)shizuku::stream::role::CONSUMER);
    }
  }
  BOARD::diag_printf("[OTA] ready (staging 0x%lx, %lu KB, in stream %lu)\n",
                     (unsigned long)STAGING_OFFSET,
                     (unsigned long)(STAGING_BYTES / 1024),
                     (unsigned long)g_in_id);

  uint64_t next = BOARD::time_us() + PERIOD_US;
  while (true) {
    const int64_t remaining = (int64_t)(next - BOARD::time_us());
    if (remaining > 0)
      api(shizuku::object_api::SLEEP_US, (uintptr_t)remaining);
    next += PERIOD_US;

    if (!g_in.valid())
      continue;
    frame_t f{};
    uint32_t lost = 0;
    while (g_in.pop(&f, &lost)) {
      if (lost != 0) {
        // ★取りこぼしたら CRC は必ず外れる。黙って続けない。
        say("input overrun — 転送をやり直すこと\n");
        reset_transfer();
        g_state = state::FAILED;
        continue;
      }
      // ★診断: 「ヘッダは来たがデータが来ない」を切り分けるため、write が
      //   届いていること自体を数える (2026-08-24 の実測でここが要った)。
      ++g_writes;
      g_write_bytes += f.len;
      feed(f.data, f.len);
      // 進捗は 64KB ごと (行が多すぎると転送そのものを圧迫する)。
      if (g_state == state::DATA && g_received - g_last_report >= 8 * 1024) {
        g_last_report = g_received;
        char line[80];
        if (snprintf(line, sizeof(line), "%lu / %lu bytes\n",
                     (unsigned long)g_received, (unsigned long)g_total) > 0)
          say(line);
      }
    }
    // 受信そのものの生存を 2 秒ごとに出す (状態に関わらず)。
    if (g_writes != g_last_write_report) {
      static uint64_t next_note = 0;
      const uint64_t now = BOARD::time_us();
      if (now >= next_note) {
        next_note = now + 2000000ull;
        g_last_write_report = g_writes;
        char line[96];
        if (snprintf(line, sizeof(line), "rx %lu writes / %lu bytes (state %lu)\n",
                     (unsigned long)g_writes, (unsigned long)g_write_bytes,
                     (unsigned long)g_state) > 0)
          say(line);
      }
    }
    if (g_state == state::FAILED || g_state == state::DONE) {
      // 次の転送を受けられるように、ヘッダ待ちへ戻す (結果は既に出してある)。
      const bool ok = g_state == state::DONE;
      reset_transfer();
      (void)ok;
    }
  }
  return 0;
}

uintptr_t ota_main(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  uintptr_t failures = api(shizuku::object_api::DECLARE_NAME, (uintptr_t) "ota").error;
  failures += export_method(method::SET_INPUT_STREAM,
                            (uintptr_t)&method_set_input_stream);
  failures += export_method(method::GET_STREAM, (uintptr_t)&method_get_stream);
  failures += export_method(method::POLL, (uintptr_t)&poll_loop);

  g_out.init();
  const auto created =
      api(shizuku::object_api::STREAM_CREATE, (uintptr_t)&g_out.desc);
  failures += created.error;
  g_out_id = created.value;
  return failures;
}

} // namespace

uint32_t register_ota() {
  const auto created = api(shizuku::object_api::CREATE_OBJECT, OBJECT,
                           (uintptr_t)&ota_main, shizuku::OBJECT_PRIVILEGED);
  const auto started = api(shizuku::object_api::CALL_METHOD, OBJECT, 0, 0);
  if (created.error != 0 || started.error != 0 || started.value != 0) {
    BOARD::diag_printf("[OTA] FAILED: create=%lu call=%lu exports_failed=%lu\n",
                       (unsigned long)created.error,
                       (unsigned long)started.error,
                       (unsigned long)started.value);
    return 1;
  }
  BOARD::diag_printf("[OTA] registered (object %lu, stream %lu)\n",
                     (unsigned long)OBJECT, (unsigned long)g_out_id);
  return 0;
}

uint32_t start_ota() {
  const auto spawned =
      api(shizuku::object_api::SPAWN, OBJECT, (uintptr_t)method::POLL, 0);
  if (spawned.error != 0) {
    BOARD::diag_printf("[OTA] could not spawn (%lu)\n",
                       (unsigned long)spawned.error);
    return 1;
  }
  BOARD::diag_printf("[OTA] thread %lu started\n", (unsigned long)spawned.value);
  return 0;
}

} // namespace ota
