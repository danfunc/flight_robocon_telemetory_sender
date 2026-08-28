#ifndef FLIGHT_ROBOCON_SHIZUKU_LOADER_HPP
#define FLIGHT_ROBOCON_SHIZUKU_LOADER_HPP

#include <cstddef>
#include <cstdint>

namespace xno::loader {

struct loaded_info {
  uint32_t object_id;
  uint32_t thread_id;
  char name[32];
  char path[32];
  uintptr_t code_addr;
  size_t size;
  bool is_xip;
  bool is_active;
};

// ローダーサブシステムの初期化
void init();

// Flash FS からバイナリをロードし、オブジェクト登録 ＆ スレッド起動を行う
// core_id: 0 (Core 0, CYW43/LED等), 1 (Core 1)
// use_xip: true の場合 Flash XIP 上で直接実行、false の場合 SRAM へコピーして実行
bool load_object(const char *path, uint32_t core_id = 0, bool use_xip = false, loaded_info *out_info = nullptr);

// オブジェクトの停止・アンロード・リソース解放
bool unload_object(uint32_t obj_id);

// 名前でアンロード
bool unload_object_by_name(const char *name);

// 既存オブジェクトを安全停止し、新しい Flash FS モジュール (ELF/BIN) でホットスワップ実行
bool hot_swap(const char *target_name, const char *new_path, uint32_t core_id = 0, loaded_info *out_info = nullptr);

// 実行中の動的オブジェクト一覧取得
size_t list_loaded(loaded_info *out_list, size_t max_entries);

// 起動時自動ロード (/etc/autorun または既定のバイナリ)
void run_autorun();

} // namespace xno::loader

#endif // FLIGHT_ROBOCON_SHIZUKU_LOADER_HPP
