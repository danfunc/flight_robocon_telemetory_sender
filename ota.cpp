// ===========================================================================
//  ota — BLE で受け取ってステージング領域へ置く
// ===========================================================================
//  設計の理由は ota.hpp 冒頭。
#include "ota.hpp"
#include "blink.hpp"
#include "shizuku/objects/ble_uart.hpp"
#include "inflate.hpp"
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
// ★★消去と書き込みを**分けた** (2026-08-24, 速度改善)。分ける前は
//   「1 セクタぶん貯める → 消して書く」を 1 回のロックでやっていて、
//   これが **転送時間の支配項**だった:
//
//     4KB セクタ消去 (20h)  ≈ 55ms  ×97 回 = 5.3s   ← ほぼ全部これ
//     4KB ぶんの書き込み    ≈ 11ms  ×97 回 = 1.1s
//
//   消去は「1 バイトあたり」ではなく「1 コマンドあたり」が重い。同じ 64KB を
//   消すのに 4KB×16 回だと 16 回ぶんの待ちが要るが、**64KB ブロック消去 (D8h)
//   なら 1 回**で済む (ROM の flash_range_erase は、範囲がブロックに揃って
//   いれば自動で D8h を使う)。so:
//
//     消去は 64KB ブロック単位で**手前に 1 回だけ**、書き込みは 4KB ごと。
//
//   ★消去を「全部先に」ではなく**必要になった時に 1 ブロックずつ**にしてある。
//     全部先にやると転送開始前に 1.4s 黙ることになり、ホスト側に「まだ送るな」
//     の握手を足す羽目になる。1 ブロックずつなら 1 回の停止が 200ms 程度で
//     収まり、その間はリンク層が勝手に詰まってくれる (握手が要らない)。
struct erase_op {
  uint32_t offset; // ブロック境界
  uint32_t bytes;
};

struct program_op {
  uint32_t offset;     // ページ境界
  const uint8_t *data;
  uint32_t bytes;
};

void erase_range(void *param) {
  const auto *op = (const erase_op *)param;
  ::flash_range_erase(op->offset, op->bytes);
}

void program_range(void *param) {
  const auto *op = (const program_op *)param;
  ::flash_range_program(op->offset, op->data, op->bytes);
}

// ---- 受信の状態 --------------------------------------------------------------
// ★ページ単位 (256B) に貯めてから焼く。BLE の 1 write は最大 244B なので
//   そのまま焼くとページ境界を跨いで効率も正しさも落ちる。
// ZLEN/ZDATA は圧縮アップロード ('XNOZ') 用。★1 チャンク = 独立した raw
// deflate ストリームで、展開すると**ちょうど 1 セクタ** (最後だけ端数) に
// なるようにホストが切ってある (理由は inflate.hpp 冒頭)。
enum struct state : uint32_t { IDLE, HEADER, DATA, ZLEN, ZDATA, DONE, FAILED };

// 圧縮チャンクの上限。非圧縮ブロックに落ちると 4096 + deflate の枠ぶん
// 増えるので、素の 4096 では足りない。
constexpr uint32_t ZCHUNK_MAX = FLASH_SECTOR_SIZE + 256;

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
// ★ステージング領域のうち、どこまで消し終わっているか (先頭からのバイト数)。
//   0 で始まり、64KB ブロックを消すたびに伸びる。
uint32_t g_erased = 0;
// 内訳を測る。「遅い」の原因を消去と書き込みとリンクに分けないと直せない。
uint64_t g_erase_us = 0;
uint64_t g_program_us = 0;
uint32_t g_erase_count = 0;
uint64_t g_start_us = 0;
// ---- 圧縮アップロード ----
bool g_compressed = false;
uint8_t g_zbuf[ZCHUNK_MAX];        // 圧縮された 1 チャンク
uint8_t g_raw[FLASH_SECTOR_SIZE];  // その展開先
uint32_t g_zlen = 0;               // このチャンクの圧縮長
uint32_t g_zgot = 0;               // 受け取った圧縮バイト
uint8_t g_zlen_bytes[2];
uint32_t g_zlen_got = 0;
uint64_t g_inflate_us = 0;
// ★展開の作業領域 (ハフマンの高速表で 3.3KB)。スレッドのスタックに積むと
//   危ないので、ここに 1 つだけ持つ (使うのは ota スレッドだけ)。
tiny_inflate::state g_inflate_state;
// ★中断された転送を放置しない。途中で切れると「残りを待つ」状態のまま
//   居座り、**次の転送のヘッダをその続きとして食う** (2026-08-24 に実測で
//   踏んだ: 前の実験の CRC を want= に出して MISMATCH になった)。
//   バイトが途絶えたら捨てて IDLE へ戻す。
uint64_t g_last_byte_us = 0;
constexpr uint64_t IDLE_TIMEOUT_US = 5000000ull;

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
  g_erased = 0; // ★やり直しでは消し直す (前の中身が残っている)
  g_erase_us = 0;
  g_program_us = 0;
  g_erase_count = 0;
  g_compressed = false;
  g_zlen = 0;
  g_zgot = 0;
  g_zlen_got = 0;
  g_inflate_us = 0;
}

// 64KB ブロックを 1 つ消す。
// ★★消去は**転送を始める前に済ませる** (2026-08-24 に方針変更)。理由は速度
//   ではなく**安全**:
//     `flash_safe_execute` は両コアを止めて IRQ を切る。64KB ブロック消去は
//     1 回およそ 105ms で、その間 CYW43 (BLE) を相手にする SPI/PIO の面倒を
//     誰も見られない。転送を流しながらこれをやると `[CYW43] Bus error` /
//     `do_ioctl timeout` が出て、最後にはチップごと固まる (実機で踏んだ:
//     電源を抜くまで戻らなくなった)。**リンクが暇なうちに消しておけば**、
//     転送中に止めるのは書き込みの ~9ms だけで済む。
//   ★1 ブロックずつ別々の flash_safe_execute にして、間で譲る。1 回の停止を
//     短く保つのが目的で、まとめて 740ms 止めるのでは意味が無い。
bool erase_block(uint32_t at) {
  erase_op op{STAGING_OFFSET + at, FLASH_BLOCK_SIZE};
  const uint64_t t0 = BOARD::time_us();
  if (::flash_safe_execute(erase_range, &op, 5000) != PICO_OK) {
    say("flash erase failed\n");
    g_state = state::FAILED;
    return false;
  }
  g_erase_us += BOARD::time_us() - t0;
  ++g_erase_count;
  return true;
}

// 受け取る予定の範囲をまとめて消す。終わったら "ready" を出す —
// ★ホストはこの行を見てから流し始める。消している間に流されると、
//   結局「消しながら受ける」ことになって上の危険が戻ってくる。
bool erase_staging(uint32_t bytes) {
  const uint32_t blocks = (bytes + FLASH_BLOCK_SIZE - 1) / FLASH_BLOCK_SIZE;
  for (uint32_t b = 0; b < blocks; ++b) {
    if (!erase_block(b * FLASH_BLOCK_SIZE))
      return false;
    api(shizuku::object_api::YIELD);
  }
  g_erased = blocks * FLASH_BLOCK_SIZE;
  return true;
}

// 貯まった 1 セクタを焼く。★消去はここではやらない — 転送を始める前に
//   erase_staging() が済ませてある (理由は erase_block のコメント)。
//   ここで止まるのは 1 回およそ 9ms。
bool commit_sector() {
  if (g_sector_len == 0)
    return true;
  // 端数は 0xFF で埋める (消去後と同じ値にしておく)。
  if (g_sector_len < FLASH_SECTOR_SIZE)
    memset(g_sector + g_sector_len, 0xFF, FLASH_SECTOR_SIZE - g_sector_len);
  const uint32_t at = g_received - g_sector_len; // このセクタの先頭バイト位置
  const uint32_t offset =
      STAGING_OFFSET + (at / FLASH_SECTOR_SIZE) * FLASH_SECTOR_SIZE;
  program_op op{offset, g_sector, FLASH_SECTOR_SIZE};
  const uint64_t t0 = BOARD::time_us();
  if (::flash_safe_execute(program_range, &op, 5000) != PICO_OK) {
    say("flash write failed\n");
    g_state = state::FAILED;
    return false;
  }
  g_program_us += BOARD::time_us() - t0;
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
  uint32_t src_offset;   // ステージングの先頭 (flash オフセット)
  uint32_t dst_offset;   // 本体の先頭 (flash オフセット)
  uint32_t copy_sectors; // 新しい像が占めるセクタ数
  uint32_t erase_blocks; // 消す 64KB ブロック数 (旧像の残りも含めて消す)
  uint8_t *buffer;       // FLASH_SECTOR_SIZE バイトの RAM
  reboot_fn reboot;      // ROM の reboot (呼ぶ前に引いてある)
};

void __no_inline_not_in_flash_func(commit_blast)(void *param) {
  // ★引数も 1 語ずつ読む (構造体まるごとのコピーは memcpy に化けうる)。
  const volatile commit_op *op = (const volatile commit_op *)param;
  const uint32_t src_offset = op->src_offset;
  const uint32_t dst_offset = op->dst_offset;
  const uint32_t copy_sectors = op->copy_sectors;
  const uint32_t erase_blocks = op->erase_blocks;
  volatile uint32_t *const buffer = (volatile uint32_t *)op->buffer;
  const reboot_fn reboot = op->reboot;

  // ★消去は **64KB ブロック (D8h) 単位**。4KB セクタ消去 (20h) を 98 回
  //   やると 5.4 秒かかるが、64KB ブロック 7 回なら 0.7 秒で済む。消去は
  //   「バイトあたり」ではなく「コマンドあたり」が重いため。ROM の
  //   flash_range_erase は範囲がブロックに揃っていれば自動で D8h を使う。
  constexpr uint32_t SECTORS_PER_BLOCK = FLASH_BLOCK_SIZE / FLASH_SECTOR_SIZE;
  for (uint32_t b = 0; b < erase_blocks; ++b) {
    const uint32_t base = b * FLASH_BLOCK_SIZE;
    ::flash_range_erase(dst_offset + base, FLASH_BLOCK_SIZE);
    for (uint32_t s = 0; s < SECTORS_PER_BLOCK; ++s) {
      const uint32_t sector = b * SECTORS_PER_BLOCK + s;
      if (sector >= copy_sectors)
        break; // この先は消すだけ (旧像の残り)
      const uint32_t at = base + s * FLASH_SECTOR_SIZE;
      const volatile uint32_t *src =
          (const volatile uint32_t *)(XIP_BASE + src_offset + at);
      for (uint32_t i = 0; i < FLASH_SECTOR_SIZE / sizeof(uint32_t); ++i)
        buffer[i] = src[i];
      ::flash_range_program(dst_offset + at, (const uint8_t *)buffer,
                            FLASH_SECTOR_SIZE);
    }
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
  api(shizuku::object_api::CALL_METHOD, blink::OBJECT,
      (uintptr_t)blink::method::SET_PATTERN,
      (uintptr_t)blink::pattern_id::DYNAMIC_SWEEP);
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
  //   消去はブロック単位なので、判定もブロックに切り上げた範囲でやる。
  const uint32_t span = total > old_bytes ? total : old_bytes;
  const uint32_t erase_blocks =
      (span + FLASH_BLOCK_SIZE - 1) / FLASH_BLOCK_SIZE;
  if (dst_offset + erase_blocks * FLASH_BLOCK_SIZE > STAGING_OFFSET) {
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
               "commit: %lu bytes -> 0x%lx (%lu blocks), no return\n",
               (unsigned long)total, (unsigned long)dst_offset,
               (unsigned long)erase_blocks) > 0)
    say(line);
  // ★この行が実際に BLE から出るまで待つ (say → logger → notify は非同期)。
  //   出ないまま消し始めると、ホストからは黙って切れたようにしか見えない。
  api(shizuku::object_api::SLEEP_US, 400000);

  // ★★焼く前に BLE を切る。commit は両コアを 1.7 秒止めるので、繋いだまま
  //   だと CYW43 の面倒を誰も見られない時間がそれだけ続く。転送中の
  //   105ms の消去でさえ `[CYW43] Bus error` を出してチップごと固めた
  //   (2026-08-24 実機、電源を抜くまで復帰せず) —— 1.7 秒はその 16 倍。
  //   ★切るのを頼むだけ。実際に gap_disconnect を呼ぶのは ble_uart の
  //     poll ループ (呼び出し元スレッドから btstack を触ると壊れる)。
  api(shizuku::object_api::CALL_METHOD, xno_object_id::ble_uart,
      (uintptr_t)shizuku::objects::ble_uart::method::REQUEST_DISCONNECT, 0);
  api(shizuku::object_api::SLEEP_US, 300000);

  commit_op op{STAGING_OFFSET,
               dst_offset,
               sectors_for(total),
               erase_blocks,
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

void begin_transfer(bool compressed) {
  g_compressed = compressed;
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
  if (snprintf(line, sizeof(line), "begin: %lu bytes crc=%08lx%s\n",
               (unsigned long)g_total, (unsigned long)g_expect_crc,
               compressed ? " (deflate, 4KB chunks)" : "") > 0)
    say(line);
  g_start_us = BOARD::time_us();
  g_zlen_got = 0;
  g_zgot = 0;
  // ★流れて来る前に消しておく。ここで数百 ms 黙るが、リンクは暇なので安全。
  if (!erase_staging(g_total))
    return;
  g_state = compressed ? state::ZLEN : state::DATA;
  // ★OTA 転送中は LED を高速ストロボ (Pattern 4) にして受信中であることを視覚化
  api(shizuku::object_api::CALL_METHOD, blink::OBJECT,
      (uintptr_t)blink::method::SET_PATTERN,
      (uintptr_t)blink::pattern_id::FAST_STROBE);
  say("ready\n"); // ホストはこれを見てから流す
}

// 転送の締め: CRC 判定と内訳の報告。★「受け取ったバイト列」の検証であって、
//   flash に本当にそう書けたかは commit 側でもう一度確かめる。
void finish_transfer() {
  const uint32_t crc = g_crc ^ 0xFFFFFFFFu;
  char line[128];
  if (crc != g_expect_crc) {
    g_state = state::FAILED;
    api(shizuku::object_api::CALL_METHOD, blink::OBJECT,
        (uintptr_t)blink::method::SET_PATTERN,
        (uintptr_t)blink::pattern_id::DYNAMIC_SWEEP);
    if (snprintf(line, sizeof(line),
                 "CRC MISMATCH got=%08lx want=%08lx — 本体は無傷\n",
                 (unsigned long)crc, (unsigned long)g_expect_crc) > 0)
      say(line);
    return;
  }
  g_state = state::DONE;
  // ★ステージング完了時はトリプルフラッシュ (Pattern 2) で検証合格を通知
  api(shizuku::object_api::CALL_METHOD, blink::OBJECT,
      (uintptr_t)blink::method::SET_PATTERN,
      (uintptr_t)blink::pattern_id::TRIPLE_BEACON);
  if (snprintf(line, sizeof(line),
               "done: %lu bytes crc=%08lx OK (staged at 0x%lx)\n",
               (unsigned long)g_received, (unsigned long)crc,
               (unsigned long)STAGING_OFFSET) > 0)
    say(line);
  // ★内訳を出す。「遅い」を消去・書き込み・展開・リンクに割らないと直せない。
  const uint32_t total_ms = (uint32_t)((BOARD::time_us() - g_start_us) / 1000);
  const uint32_t erase_ms = (uint32_t)(g_erase_us / 1000);
  const uint32_t program_ms = (uint32_t)(g_program_us / 1000);
  const uint32_t inflate_ms = (uint32_t)(g_inflate_us / 1000);
  if (snprintf(line, sizeof(line),
               "time: %lums = erase %lu (%lu blk) + program %lu + inflate %lu "
               "+ link %lu\n",
               (unsigned long)total_ms, (unsigned long)erase_ms,
               (unsigned long)g_erase_count, (unsigned long)program_ms,
               (unsigned long)inflate_ms,
               (unsigned long)(total_ms - erase_ms - program_ms - inflate_ms)) > 0)
    say(line);
}

// **展開後の**バイト列を取り込む。戻り値 = 実際に取り込んだバイト数。
// ★圧縮でも非圧縮でも、ここから先の扱いは完全に同じ — セクタに貯めて、
//   埋まったら焼いて、CRC を回す。圧縮の有無はここより手前で吸収してある。
uint32_t absorb(const uint8_t *data, uint32_t length) {
  uint32_t taken = 0;
  while (taken < length && g_state != state::DONE && g_state != state::FAILED) {
    uint32_t take = length - taken;
    const uint32_t room = FLASH_SECTOR_SIZE - g_sector_len;
    if (take > room)
      take = room;
    const uint32_t remaining = g_total - g_received;
    if (take > remaining)
      take = remaining;
    if (take == 0)
      break;
    memcpy(g_sector + g_sector_len, data + taken, take);
    g_crc = crc32_update(g_crc, data + taken, take);
    g_sector_len += take;
    g_received += take;
    taken += take;
    if (g_sector_len == FLASH_SECTOR_SIZE && !commit_sector())
      return taken;
    if (g_received >= g_total) {
      if (!commit_sector())
        return taken;
      finish_transfer();
      return taken;
    }
  }
  return taken;
}

// 1 チャンクぶんの圧縮データが揃った。展開して absorb へ渡す。
void inflate_chunk() {
  const uint32_t remaining = g_total - g_received;
  const uint32_t expect =
      remaining < FLASH_SECTOR_SIZE ? remaining : FLASH_SECTOR_SIZE;
  const uint64_t t0 = BOARD::time_us();
  const int32_t produced =
      tiny_inflate::run(g_inflate_state, g_zbuf, g_zlen, g_raw, sizeof(g_raw));
  g_inflate_us += BOARD::time_us() - t0;
  if (produced < 0 || (uint32_t)produced != expect) {
    char line[96];
    if (snprintf(line, sizeof(line),
                 "inflate failed at %lu (rc=%ld want=%lu) — 本体は無傷\n",
                 (unsigned long)g_received, (long)produced,
                 (unsigned long)expect) > 0)
      say(line);
    g_state = state::FAILED;
    return;
  }
  absorb(g_raw, (uint32_t)produced);
  if (g_state == state::ZDATA) {
    g_state = state::ZLEN; // 次のチャンクの長さへ
    g_zlen_got = 0;
    g_zgot = 0;
  }
}

void feed(const uint8_t *data, uint32_t length) {
  g_last_byte_us = BOARD::time_us();
  uint32_t i = 0;
  while (i < length) {
    switch (g_state) {
    case state::IDLE:
    case state::HEADER: {
      // マジックを探す。途中から繋いだ相手でも拾えるように、先頭 4 文字が
      // 合うまでは読み捨てる。★4 文字目がコマンド:
      //   'U' = 生のまま送る / 'Z' = 圧縮して送る / 'C' = commit。
      const uint8_t prefix[3] = {'X', 'N', 'O'};
      const uint8_t byte = data[i];
      const bool matches =
          g_header_len < 3    ? byte == prefix[g_header_len]
          : g_header_len == 3 ? (byte == 'U' || byte == 'Z' || byte == 'C')
                              : true;
      ++i;
      if (!matches) {
        g_header_len = 0;
        continue;
      }
      g_header[g_header_len++] = byte;
      g_state = state::HEADER;
      if (g_header_len == sizeof(g_header)) {
        g_header_len = 0;
        if (g_header[3] == 'C')
          begin_commit(); // ★成功すれば戻ってこない (再起動する)
        else
          begin_transfer(g_header[3] == 'Z');
      }
      break;
    }
    case state::DATA:
      i += absorb(data + i, length - i);
      break;
    case state::ZLEN: {
      // チャンクの圧縮長 (uint16 le)
      while (i < length && g_zlen_got < 2)
        g_zlen_bytes[g_zlen_got++] = data[i++];
      if (g_zlen_got < 2)
        break;
      g_zlen = (uint32_t)g_zlen_bytes[0] | ((uint32_t)g_zlen_bytes[1] << 8);
      g_zlen_got = 0;
      g_zgot = 0;
      if (g_zlen == 0 || g_zlen > ZCHUNK_MAX) {
        char line[96];
        if (snprintf(line, sizeof(line), "bad chunk length %lu — 本体は無傷\n",
                     (unsigned long)g_zlen) > 0)
          say(line);
        g_state = state::FAILED;
        break;
      }
      g_state = state::ZDATA;
      break;
    }
    case state::ZDATA: {
      uint32_t take = length - i;
      const uint32_t room = g_zlen - g_zgot;
      if (take > room)
        take = room;
      memcpy(g_zbuf + g_zgot, data + i, take);
      g_zgot += take;
      i += take;
      if (g_zgot == g_zlen)
        inflate_chunk();
      break;
    }
    default:
      return; // DONE / FAILED のあいだは捨てる (次は reset 後に受ける)
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
    // ★受信が途絶えたまま残っている転送は捨てる。ここを入れないと、
    //   中断のあとの 1 回目が必ず化ける (上の g_last_byte_us のコメント)。
    if (g_state != state::IDLE && g_state != state::DONE &&
        g_state != state::FAILED &&
        BOARD::time_us() - g_last_byte_us > IDLE_TIMEOUT_US) {
      char line[96];
      if (snprintf(line, sizeof(line),
                   "timed out at %lu / %lu bytes — 捨てて待ち受けに戻る\n",
                   (unsigned long)g_received, (unsigned long)g_total) > 0)
        say(line);
      reset_transfer();
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
