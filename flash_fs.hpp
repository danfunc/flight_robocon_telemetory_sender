#ifndef FLIGHT_ROBOCON_FLASH_FS_HPP
#define FLIGHT_ROBOCON_FLASH_FS_HPP

#include <cstddef>
#include <cstdint>

namespace xno::fs {

// Flash FS パーティション設定 (RP2350 4MB Flash)
// 0x00200000 〜 0x003F8000 (約 2MB, 504 セクタ)
constexpr uintptr_t FS_FLASH_OFFSET = 0x00200000;
constexpr size_t FS_FLASH_SIZE = 0x001F8000; // 2,064,384 bytes (504 * 4KB)
constexpr size_t SECTOR_SIZE = 4096;
constexpr size_t PAGE_SIZE = 256;
constexpr size_t MAX_FILENAME_LEN = 32;
constexpr size_t MAX_FILES = 64;

constexpr size_t DIR_BANKS = 2; // Bank A: 0x00200000, Bank B: 0x00201000 (A/B ピンポンで電源断保護)

// ディレクトリヘッダ (A/B ピンポン用)
struct dir_header {
  uint32_t magic;                    // 'XNOD' (0x584E4F44)
  uint32_t sequence;                 // 単調増加世代番号 (大きい方が最新)
  uint32_t file_count;               // 有効ファイル数
  uint32_t crc32;                    // entries[MAX_FILES] の CRC32
};

// ファイルメタデータ
struct file_entry {
  uint32_t magic;                    // 'XNOF' (0x584E4F46)
  char name[MAX_FILENAME_LEN];        // 例: "/bin/blink.bin"
  uint32_t size;                     // ファイルサイズ (バイト)
  uint32_t crc32;                    // CRC32 チェックサム
  uint32_t flash_offset;             // パーティション先頭からの相対オフセット (4KB アライン)
  uint32_t flags;                    // 1: 有効, 2: 実行可能 (Executable)
  uint32_t reserved[2];
};

struct stat_t {
  char name[MAX_FILENAME_LEN];
  size_t size;
  uint32_t crc32;
  uintptr_t xip_address;             // XIP 直読可能アドレス (0x10000000 + ...)
  bool is_valid;
};

// 初期化・マウント (未初期化の場合は自動フォーマット)
bool init();

// 強制フォーマット (全エントリ消去)
bool format();

// ファイル一覧取得 (見つかったファイル数を返す)
size_t list_files(stat_t *out_list, size_t max_entries);

// ファイル情報取得
bool stat(const char *path, stat_t *out_stat);

// ファイルの作成・保存 (既存ファイルは上書き)
bool write_file(const char *path, const uint8_t *data, size_t size, uint32_t flags = 1);

// ファイルの読み出し
bool read_file(const char *path, uint8_t *out_buf, size_t max_size, size_t *out_actual_size = nullptr);

// ファイルの XIP 直読アドレスを取得 (Flash 上の実行用ポインタ)
const uint8_t *get_xip_ptr(const char *path, size_t *out_size = nullptr);

// ファイルの削除
bool remove_file(const char *path);

// 空き容量 (バイト)
size_t free_space();

// 使用容量 (バイト)
size_t used_space();

} // namespace xno::fs

#endif // FLIGHT_ROBOCON_FLASH_FS_HPP
