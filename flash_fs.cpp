#include "flash_fs.hpp"
#include "shizuku/kernel.hpp"
extern "C" {
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/flash.h"
}
#include <cstdio>
#include <cstring>

namespace xno::fs {
namespace {

constexpr uint32_t FS_MAGIC = 0x584E4F46;  // 'XNOF' (file_entry)
constexpr uint32_t DIR_MAGIC = 0x584E4F44; // 'XNOD' (dir_header)
constexpr uintptr_t DATA_OFFSET = FS_FLASH_OFFSET + (DIR_BANKS * SECTOR_SIZE);
constexpr size_t DATA_SIZE = FS_FLASH_SIZE - (DIR_BANKS * SECTOR_SIZE);

// CRC32 計算
const uint32_t CRC_NIBBLE[16] = {
    0x00000000, 0x1DB71064, 0x3B6E20C8, 0x26D930AC, 0x76DC4190, 0x6B6B51F4,
    0x4DB26158, 0x5005713C, 0xEDB88320, 0xF00F9344, 0xD6D6A3E8, 0xCB61B38C,
    0x9B64C2B0, 0x86D3D2D4, 0xA00AE278, 0xBDBDF21C};

uint32_t calc_crc32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    crc = (crc >> 4) ^ CRC_NIBBLE[crc & 0x0F];
    crc = (crc >> 4) ^ CRC_NIBBLE[crc & 0x0F];
  }
  return ~crc;
}

// RAM 上のディレクトリキャッシュ (4KB, 256B アライン)
alignas(256) file_entry g_dir_cache[MAX_FILES];
alignas(256) uint8_t g_sector_buf[SECTOR_SIZE]; // スタック枯渇防止用の作業バッファ
uint32_t g_sequence = 0;
int g_active_bank = -1; // 0 (Bank A) or 1 (Bank B)
bool g_initialized = false;

// Flash 安全書き込みヘルパー
// ★重要: flash_safe_execute 中は XIP (Flash) が無効化されるため、
//   コールバック内で memset/memcpy などの libc 関数を呼ぶとクラッシュする。
//   データの準備 (ページパディング等) は必ず flash_safe_execute の外側で行う。
struct flash_erase_op {
  uint32_t offset;
  uint32_t size;
};

struct flash_prog_op {
  uint32_t offset;
  const uint8_t *data;
  uint32_t size;
};

void flash_erase_cb(void *param) {
  const auto *op = (const flash_erase_op *)param;
  ::flash_range_erase(op->offset, op->size);
}

void flash_prog_cb(void *param) {
  const auto *op = (const flash_prog_op *)param;
  ::flash_range_program(op->offset, op->data, op->size);
}

bool safe_flash_erase_sector(uintptr_t offset) {
  flash_erase_op op{(uint32_t)offset, (uint32_t)SECTOR_SIZE};
  return ::flash_safe_execute(flash_erase_cb, &op, 5000) == PICO_OK;
}

bool safe_flash_program_pages(uintptr_t offset, const uint8_t *data, size_t size) {
  if (size == 0) return true;
  flash_prog_op op{(uint32_t)offset, data, (uint32_t)size};
  return ::flash_safe_execute(flash_prog_cb, &op, 5000) == PICO_OK;
}

// バンクの妥当性検査
bool check_bank(int bank, dir_header &out_hdr, file_entry *out_entries) {
  const uint8_t *bank_ptr =
      (const uint8_t *)(XIP_BASE + FS_FLASH_OFFSET + (bank * SECTOR_SIZE));
  const auto *hdr = (const dir_header *)bank_ptr;
  if (hdr->magic != DIR_MAGIC) {
    return false;
  }
  const auto *entries = (const file_entry *)(bank_ptr + sizeof(dir_header));
  const uint32_t calculated_crc =
      calc_crc32((const uint8_t *)entries, sizeof(file_entry) * MAX_FILES);
  if (calculated_crc != hdr->crc32) {
    return false;
  }
  out_hdr = *hdr;
  if (out_entries != nullptr) {
    memcpy(out_entries, entries, sizeof(file_entry) * MAX_FILES);
  }
  return true;
}

// ディレクトリテーブルを A/B ピンポンで裏側バンクへフラッシュ (電源断保護)
bool sync_dir_to_flash() {
  const int target_bank = (g_active_bank == 0) ? 1 : 0;
  const uint32_t next_seq = g_sequence + 1;

  uint32_t count = 0;
  for (size_t i = 0; i < MAX_FILES; ++i) {
    if (g_dir_cache[i].magic == FS_MAGIC && (g_dir_cache[i].flags & 1) != 0) {
      count++;
    }
  }

  dir_header hdr{};
  hdr.magic = DIR_MAGIC;
  hdr.sequence = next_seq;
  hdr.file_count = count;
  hdr.crc32 = calc_crc32((const uint8_t *)g_dir_cache, sizeof(g_dir_cache));

  memset(g_sector_buf, 0xFF, sizeof(g_sector_buf));
  memcpy(g_sector_buf, &hdr, sizeof(hdr));
  memcpy(g_sector_buf + sizeof(hdr), g_dir_cache, sizeof(g_dir_cache));

  const uintptr_t target_offset =
      FS_FLASH_OFFSET + (target_bank * SECTOR_SIZE);
  if (!safe_flash_erase_sector(target_offset)) {
    return false;
  }
  if (!safe_flash_program_pages(target_offset, g_sector_buf, SECTOR_SIZE)) {
    return false;
  }

  // ベリファイ
  dir_header vhdr{};
  if (!check_bank(target_bank, vhdr, nullptr)) {
    return false;
  }

  g_active_bank = target_bank;
  g_sequence = next_seq;
  return true;
}

int find_file_index(const char *path) {
  for (size_t i = 0; i < MAX_FILES; ++i) {
    if (g_dir_cache[i].magic == FS_MAGIC && (g_dir_cache[i].flags & 1) != 0) {
      if (strncmp(g_dir_cache[i].name, path, MAX_FILENAME_LEN) == 0) {
        return (int)i;
      }
    }
  }
  return -1;
}

int find_free_entry() {
  for (size_t i = 0; i < MAX_FILES; ++i) {
    if (g_dir_cache[i].magic != FS_MAGIC || (g_dir_cache[i].flags & 1) == 0) {
      return (int)i;
    }
  }
  return -1;
}

// 空き領域のオフセットを計算 (4KB セクタアライメント)
uint32_t allocate_data_offset(size_t size) {
  uint32_t needed_sectors =
      ((uint32_t)size + SECTOR_SIZE - 1) / SECTOR_SIZE;
  if (needed_sectors == 0)
    needed_sectors = 1;

  // 使用中の最大オフセットを探索 (データ領域は DIR_BANKS * 4KB = 8KB 以降)
  uint32_t max_used_offset = (uint32_t)(DIR_BANKS * SECTOR_SIZE);
  for (size_t i = 0; i < MAX_FILES; ++i) {
    if (g_dir_cache[i].magic == FS_MAGIC && (g_dir_cache[i].flags & 1) != 0) {
      uint32_t f_end =
          g_dir_cache[i].flash_offset +
          (((g_dir_cache[i].size + SECTOR_SIZE - 1) / SECTOR_SIZE) *
           SECTOR_SIZE);
      if (f_end > max_used_offset) {
        max_used_offset = f_end;
      }
    }
  }

  if (max_used_offset + (needed_sectors * SECTOR_SIZE) <= FS_FLASH_SIZE) {
    return max_used_offset;
  }
  return 0xFFFFFFFFu; // 空きなし
}

} // namespace

bool init() {
  dir_header hdr0{}, hdr1{};
  const bool v0 = check_bank(0, hdr0, nullptr);
  const bool v1 = check_bank(1, hdr1, nullptr);

  if (v0 && v1) {
    // 両バンク有効: 世代番号 (sequence) の新しい方を採用
    if (hdr1.sequence > hdr0.sequence) {
      g_active_bank = 1;
      g_sequence = hdr1.sequence;
      check_bank(1, hdr1, g_dir_cache);
    } else {
      g_active_bank = 0;
      g_sequence = hdr0.sequence;
      check_bank(0, hdr0, g_dir_cache);
    }
  } else if (v0) {
    g_active_bank = 0;
    g_sequence = hdr0.sequence;
    check_bank(0, hdr0, g_dir_cache);
  } else if (v1) {
    g_active_bank = 1;
    g_sequence = hdr1.sequence;
    check_bank(1, hdr1, g_dir_cache);
  } else {
    // レガシー形式 (Bank 0 ヘッダなし直接エントリ) の移行互換
    const auto *legacy_entries =
        (const file_entry *)(XIP_BASE + FS_FLASH_OFFSET);
    bool has_legacy = false;
    for (size_t i = 0; i < MAX_FILES; ++i) {
      if (legacy_entries[i].magic == FS_MAGIC) {
        has_legacy = true;
        break;
      }
    }
    if (has_legacy) {
      memcpy(g_dir_cache, legacy_entries, sizeof(g_dir_cache));
      g_sequence = 1;
      g_active_bank = 0;
      sync_dir_to_flash(); // A/B 構造へアップグレード
    } else {
      memset(g_dir_cache, 0, sizeof(g_dir_cache));
      g_sequence = 0;
      g_active_bank = -1;
      g_initialized = true;
      shizuku::KERNEL::BOARD::diag_printf(
          "[FLASH FS] partition ready (empty)\n");
      return true;
    }
  }

  g_initialized = true;
  shizuku::KERNEL::BOARD::diag_printf(
      "[FLASH FS] mounted A/B partition (active: Bank %d, seq: %lu, used: %lu "
      "KB)\n",
      g_active_bank, (unsigned long)g_sequence,
      (unsigned long)(used_space() / 1024));
  return true;
}

bool format() {
  memset(g_dir_cache, 0, sizeof(g_dir_cache));
  g_sequence = 0;
  g_active_bank = 0;
  if (!sync_dir_to_flash()) {
    shizuku::KERNEL::BOARD::diag_printf("[FLASH FS] format failed\n");
    return false;
  }
  // Bank 1 も初期化消去
  safe_flash_erase_sector(FS_FLASH_OFFSET + SECTOR_SIZE);
  g_initialized = true;
  shizuku::KERNEL::BOARD::diag_printf(
      "[FLASH FS] format completed successfully\n");
  return true;
}

size_t list_files(stat_t *out_list, size_t max_entries) {
  if (!g_initialized)
    init();
  size_t count = 0;
  for (size_t i = 0; i < MAX_FILES && count < max_entries; ++i) {
    if (g_dir_cache[i].magic == FS_MAGIC && (g_dir_cache[i].flags & 1) != 0) {
      stat_t &st = out_list[count++];
      strncpy(st.name, g_dir_cache[i].name, MAX_FILENAME_LEN - 1);
      st.name[MAX_FILENAME_LEN - 1] = '\0';
      st.size = g_dir_cache[i].size;
      st.crc32 = g_dir_cache[i].crc32;
      st.xip_address =
          XIP_BASE + FS_FLASH_OFFSET + g_dir_cache[i].flash_offset;
      st.is_valid = true;
    }
  }
  return count;
}

bool stat(const char *path, stat_t *out_stat) {
  if (!g_initialized)
    init();
  int idx = find_file_index(path);
  if (idx < 0)
    return false;

  if (out_stat != nullptr) {
    strncpy(out_stat->name, g_dir_cache[idx].name, MAX_FILENAME_LEN - 1);
    out_stat->name[MAX_FILENAME_LEN - 1] = '\0';
    out_stat->size = g_dir_cache[idx].size;
    out_stat->crc32 = g_dir_cache[idx].crc32;
    out_stat->xip_address =
        XIP_BASE + FS_FLASH_OFFSET + g_dir_cache[idx].flash_offset;
    out_stat->is_valid = true;
  }
  return true;
}

bool write_file(const char *path, const uint8_t *data, size_t size,
                uint32_t flags) {
  if (!g_initialized)
    init();
  if (path == nullptr || (data == nullptr && size > 0))
    return false;

  // 既存ファイルの確認
  int idx = find_file_index(path);
  uint32_t f_offset = 0;

  if (idx >= 0) {
    // 既存エントリの再利用
    f_offset = g_dir_cache[idx].flash_offset;
  } else {
    // 新規エントリの確保
    idx = find_free_entry();
    if (idx < 0) {
      shizuku::KERNEL::BOARD::diag_printf("[FLASH FS] no free file slots\n");
      return false;
    }
    f_offset = allocate_data_offset(size);
    if (f_offset == 0xFFFFFFFFu) {
      shizuku::KERNEL::BOARD::diag_printf("[FLASH FS] no free space\n");
      return false;
    }
  }

  // セクタ単位で消去と書き込みを実行
  // ★消去は 1 セクタに 1 回、書き込みはページアライン済みサイズで 1 回
  size_t total_sectors =
      (size + SECTOR_SIZE - 1) / SECTOR_SIZE;
  if (total_sectors == 0)
    total_sectors = 1;

  for (size_t s = 0; s < total_sectors; ++s) {
    size_t sector_start = s * SECTOR_SIZE;
    uintptr_t sector_offset = FS_FLASH_OFFSET + f_offset + sector_start;

    // 1. セクタ消去
    if (!safe_flash_erase_sector(sector_offset)) {
      return false;
    }

    // 2. データ書き込み (ページ境界にアライン)
    if (sector_start < size) {
      size_t chunk = size - sector_start;
      if (chunk > SECTOR_SIZE)
        chunk = SECTOR_SIZE;

      memset(g_sector_buf, 0xFF, sizeof(g_sector_buf));
      memcpy(g_sector_buf, data + sector_start, chunk);

      size_t prog_bytes = ((chunk + PAGE_SIZE - 1) / PAGE_SIZE) * PAGE_SIZE;
      if (!safe_flash_program_pages(sector_offset, g_sector_buf, prog_bytes)) {
        return false;
      }
    }
  }

  // メタデータの更新
  file_entry &entry = g_dir_cache[idx];
  entry.magic = FS_MAGIC;
  strncpy(entry.name, path, MAX_FILENAME_LEN - 1);
  entry.name[MAX_FILENAME_LEN - 1] = '\0';
  entry.size = (uint32_t)size;
  entry.crc32 = calc_crc32(data, size);
  entry.flash_offset = f_offset;
  entry.flags = flags | 1;

  return sync_dir_to_flash();
}

bool read_file(const char *path, uint8_t *out_buf, size_t max_size,
               size_t *out_actual_size) {
  if (!g_initialized)
    init();
  int idx = find_file_index(path);
  if (idx < 0)
    return false;

  size_t to_copy = g_dir_cache[idx].size;
  if (to_copy > max_size)
    to_copy = max_size;

  const uint8_t *src =
      (const uint8_t *)(XIP_BASE + FS_FLASH_OFFSET +
                        g_dir_cache[idx].flash_offset);
  memcpy(out_buf, src, to_copy);
  if (out_actual_size != nullptr) {
    *out_actual_size = g_dir_cache[idx].size;
  }
  return true;
}

const uint8_t *get_xip_ptr(const char *path, size_t *out_size) {
  if (!g_initialized)
    init();
  int idx = find_file_index(path);
  if (idx < 0)
    return nullptr;

  if (out_size != nullptr) {
    *out_size = g_dir_cache[idx].size;
  }
  return (const uint8_t *)(XIP_BASE + FS_FLASH_OFFSET +
                           g_dir_cache[idx].flash_offset);
}

bool remove_file(const char *path) {
  if (!g_initialized)
    init();
  int idx = find_file_index(path);
  if (idx < 0)
    return false;

  g_dir_cache[idx].flags = 0; // 無効化
  g_dir_cache[idx].magic = 0;
  return sync_dir_to_flash();
}

size_t used_space() {
  if (!g_initialized)
    init();
  size_t used = DIR_BANKS * SECTOR_SIZE;
  for (size_t i = 0; i < MAX_FILES; ++i) {
    if (g_dir_cache[i].magic == FS_MAGIC && (g_dir_cache[i].flags & 1) != 0) {
      used += ((g_dir_cache[i].size + SECTOR_SIZE - 1) / SECTOR_SIZE) *
              SECTOR_SIZE;
    }
  }
  return used;
}

size_t free_space() {
  size_t u = used_space();
  return (u < FS_FLASH_SIZE) ? (FS_FLASH_SIZE - u) : 0;
}

} // namespace xno::fs
