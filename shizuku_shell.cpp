// ===========================================================================
//  shizuku_shell.cpp — Shizuku OS 対話型管理シェル (CLI)
// ===========================================================================
#include "flash_fs.hpp"
#include "shizuku/object_api.hpp"
#include "shizuku/objects/usb_cdc.hpp"
#include "shizuku_loader.hpp"
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include "pico/stdlib.h"
#include "hardware/sync.h"

extern "C" void rom_reset_usb_boot(uint32_t usb_activity_gpio_pin_mask,
                                   uint32_t disable_interface_mask);

namespace xno::shell {
namespace {

struct api_result {
  uintptr_t error;
  uintptr_t value;
};

__attribute__((always_inline)) static inline api_result
api(shizuku::object_api number, uintptr_t a1 = 0, uintptr_t a2 = 0,
    uintptr_t a3 = 0, uintptr_t a4 = 0) {
  register uintptr_t r0 asm("r0") = (uintptr_t)number;
  register uintptr_t r1 asm("r1") = a1;
  register uintptr_t r2 asm("r2") = a2;
  register uintptr_t r3 asm("r3") = a3;
  register uintptr_t r12 asm("r12") = a4;
  asm volatile("svc 0"
               : "+r"(r0), "+r"(r1)
               : "r"(r2), "r"(r3), "r"(r12)
               : "memory");
  return {r0, r1};
}

// I/O 出力ヘルパー (Pico SDK stdio 経由で完全一元化)
void cdc_print(const char *str) {
  printf("%s", str);
  fflush(stdout);
}

void cdc_printf(const char *fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  cdc_print(buf);
}

// 1文字入力 (非ブロッキング/タイムアウト付き: stdio 経由)
int get_char_nonblock(uint32_t timeout_us = 1000) {
  int c = getchar_timeout_us(timeout_us);
  return (c == PICO_ERROR_TIMEOUT || c == PICO_ERROR_NO_DATA) ? -1 : c;
}

// コマンドハンドラ群
void cmd_help() {
  cdc_print("\n=== Shizuku Interactive Shell ===\n");
  cdc_print("ファイル管理:\n");
  cdc_print("  ls                    Flash FS 内のファイル一覧\n");
  cdc_print("  cat <file>            ファイル内容の表示 (Hex / ASCII)\n");
  cdc_print("  rm <file>             ファイル削除\n");
  cdc_print("  upload <path> <size>  ファイルを Flash FS へアップロード\n");
  cdc_print("  format                Flash FS を初期化\n\n");
  cdc_print("プロセス / オブジェクト管理:\n");
  cdc_print("  ps                    実行中スレッド / 動的オブジェクト一覧\n");
  cdc_print("  load <path> [core]    Flash FS からバイナリをロード・起動\n");
  cdc_print("  swap <tgt> <path> [c] 動的オブジェクトを停止し新モジュールに差替\n");
  cdc_print("  unload <id|name>      オブジェクトの停止・解放\n");
  cdc_print("  call <id> <m> [arg]   オブジェクトメソッド実行\n\n");
  cdc_print("システム:\n");
  cdc_print("  mem                   メモリ・Flash FS 使用量表示\n");
  cdc_print("  reboot                システム再起動\n");
  cdc_print("  help / ?              ヘルプ表示\n\n");
}

void cmd_ls() {
  fs::stat_t entries[16];
  size_t count = fs::list_files(entries, 16);
  cdc_print("\nNAME                     SIZE(B)    CRC32        XIP_ADDR    \n");
  cdc_print("----------------------------------------------------------------\n");
  for (size_t i = 0; i < count; ++i) {
    cdc_printf("%-24s %-10lu 0x%08lx   0x%08lx\n",
               entries[i].name,
               (unsigned long)entries[i].size,
               (unsigned long)entries[i].crc32,
               (unsigned long)entries[i].xip_address);
  }
  cdc_printf("\n合計: %lu 件 (使用中: %lu KB / 空き: %lu KB)\n\n",
             (unsigned long)count,
             (unsigned long)(fs::used_space() / 1024),
             (unsigned long)(fs::free_space() / 1024));
}

void cmd_cat(const char *path) {
  uint8_t buf[256];
  size_t actual = 0;
  if (!fs::read_file(path, buf, sizeof(buf), &actual)) {
    cdc_printf("エラー: ファイルを開けません: %s\n", path);
    return;
  }
  cdc_printf("\n=== %s (%lu bytes, 表示最大 256 bytes) ===\n", path, (unsigned long)actual);
  for (size_t i = 0; i < actual; i += 16) {
    cdc_printf("%04lx: ", (unsigned long)i);
    for (size_t j = 0; j < 16; ++j) {
      if (i + j < actual) cdc_printf("%02x ", buf[i + j]);
      else cdc_print("   ");
    }
    cdc_print(" |");
    for (size_t j = 0; j < 16; ++j) {
      if (i + j < actual) {
        char ch = (char)buf[i + j];
        cdc_printf("%c", (ch >= 0x20 && ch < 0x7F) ? ch : '.');
      }
    }
    cdc_print("|\n");
  }
  cdc_print("\n");
}

void cmd_rm(const char *path) {
  if (fs::remove_file(path)) {
    cdc_printf("削除成功: %s\n", path);
  } else {
    cdc_printf("エラー: 削除に失敗しました: %s\n", path);
  }
}

void cmd_format() {
  cdc_print("Flash FS をフォーマット中...\n");
  if (fs::format()) {
    cdc_print("フォーマット完了。\n");
  } else {
    cdc_print("エラー: フォーマットに失敗しました\n");
  }
}

void cmd_ps() {
  cdc_print("\n[実行中スレッド一覧]\n");
  cdc_print("TID  NAME               STATUS    \n");
  cdc_print("------------------------------------\n");
  for (uint32_t tid = 0; tid < 16; ++tid) {
    char name_buf[32] = "(unknown)";
    switch (tid) {
      case 0: case 1: strcpy(name_buf, "root"); break;
      case 2: strcpy(name_buf, "blink"); break;
      case 3: case 4: strcpy(name_buf, "gdbagent"); break;
      case 5: strcpy(name_buf, "gdbserver"); break;
      case 6: strcpy(name_buf, "ble_uart"); break;
      case 7: strcpy(name_buf, "logger"); break;
      case 8: strcpy(name_buf, "ota"); break;
      case 9: strcpy(name_buf, "telemetry"); break;
      case 10: strcpy(name_buf, "flight_controller"); break;
      case 11: strcpy(name_buf, "bno055"); break;
      case 12: strcpy(name_buf, "bme280"); break;
      case 13: strcpy(name_buf, "shell"); break;
      default: strcpy(name_buf, "user_thread"); break;
    }
    cdc_printf("%-4lu %-18s RUNNING   \n", (unsigned long)tid, name_buf);
  }

  cdc_print("\n[ロード済み動的オブジェクト一覧]\n");
  cdc_print("OBJ  TID  NAME           PATH               MODE      \n");
  cdc_print("------------------------------------------------------------\n");
  loader::loaded_info loaded[8];
  size_t dyn_count = loader::list_loaded(loaded, 8);
  for (size_t i = 0; i < dyn_count; ++i) {
    if (loaded[i].is_active) {
      cdc_printf("%-4lu %-4lu %-14s %-18s %-10s\n",
                 (unsigned long)loaded[i].object_id,
                 (unsigned long)loaded[i].thread_id,
                 loaded[i].name,
                 loaded[i].path,
                 loaded[i].is_xip ? "XIP" : "RAM");
    }
  }
  if (dyn_count == 0) {
    cdc_print("(動的オブジェクトはありません)\n");
  }
  cdc_print("\n");
}

void cmd_load(const char *path, uint32_t core) {
  cdc_printf("ロード中: %s (Core %lu)...\n", path, (unsigned long)core);
  loader::loaded_info info{};
  if (loader::load_object(path, core, true /* XIP */, &info)) {
    cdc_printf("ロード成功: %s -> Object ID: %lu, Thread ID: %lu\n",
               info.name, (unsigned long)info.object_id, (unsigned long)info.thread_id);
  } else {
    cdc_printf("エラー: ロードに失敗しました: %s\n", path);
  }
}

void cmd_swap(const char *target, const char *new_path, uint32_t core) {
  cdc_printf("ホットスワップ中: '%s' -> '%s' (Core %lu)...\n", target, new_path, (unsigned long)core);
  loader::loaded_info info{};
  if (loader::hot_swap(target, new_path, core, &info)) {
    cdc_printf("ホットスワップ成功: '%s' に切り替え完了 (Obj: %lu)\n",
               new_path, (unsigned long)info.object_id);
  } else {
    cdc_printf("エラー: ホットスワップに失敗しました: '%s'\n", target);
  }
}

void cmd_unload(const char *arg) {
  uint32_t obj_id = 0;
  if (sscanf(arg, "%lu", (unsigned long *)&obj_id) == 1 && obj_id >= 20) {
    if (loader::unload_object(obj_id)) {
      cdc_printf("アンロード成功 (Object %lu)\n", (unsigned long)obj_id);
      return;
    }
  }
  if (loader::unload_object_by_name(arg)) {
    cdc_printf("アンロード成功 (%s)\n", arg);
  } else {
    cdc_printf("エラー: オブジェクト '%s' が見つかりません\n", arg);
  }
}

void cmd_call(const char *arg) {
  uint32_t obj_id = 0, method = 0, param = 0;
  int n = sscanf(arg, "%lu %lu %lu", (unsigned long *)&obj_id,
                 (unsigned long *)&method, (unsigned long *)&param);
  if (n >= 2) {
    const auto res = api(shizuku::object_api::CALL_METHOD, obj_id, method, param);
    cdc_printf("CALL結果: error=%lu, value=%lu (0x%lx)\n",
               (unsigned long)res.error, (unsigned long)res.value, (unsigned long)res.value);
  } else {
    cdc_print("使用法: call <obj_id> <method_id> [param]\n");
  }
}

void cmd_mem() {
  cdc_print("\n=== システムメモリ・Flash FS 使用量 ===\n");
  cdc_printf("Flash FS: %lu KB 使用中 / %lu KB 空き (総容量: %lu KB)\n",
             (unsigned long)(fs::used_space() / 1024),
             (unsigned long)(fs::free_space() / 1024),
             (unsigned long)(fs::FS_FLASH_SIZE / 1024));
  cdc_print("\n");
}

void cmd_reboot() {
  cdc_print("システムを再起動します...\n");
  api(shizuku::object_api::SLEEP_US, 100000);
  ::rom_reset_usb_boot(0, 0);
}

void cmd_upload(const char *path, size_t size) {
  if (size == 0 || size > 65536) {
    cdc_printf("エラー: 無効なファイルサイズです (%lu bytes)\n", (unsigned long)size);
    return;
  }
  static uint8_t upload_buf[65536];

  // 1. コマンド行末の残留改行 (\r, \n) をドレイン
  int drain = -1;
  while ((drain = get_char_nonblock(0)) >= 0) {
    if (drain != '\r' && drain != '\n' && drain != ' ') {
      upload_buf[0] = (uint8_t)drain;
      break;
    }
  }

  size_t received = (drain >= 0 && drain != '\r' && drain != '\n' && drain != ' ') ? 1 : 0;
  cdc_printf("READY %lu\n", (unsigned long)size);

  uint32_t idle_count = 0;
  while (received < size) {
    int c = get_char_nonblock(1000);
    if (c < 0) {
      api(shizuku::object_api::SLEEP_US, 1000); // 1ms
      if (++idle_count > 10000) {               // 10秒タイムアウト
        cdc_printf("\nエラー: アップロードがタイムアウトしました (%lu / %lu bytes)\n",
                   (unsigned long)received, (unsigned long)size);
        return;
      }
      continue;
    }
    upload_buf[received++] = (uint8_t)c;
    idle_count = 0;
  }

  if (fs::write_file(path, upload_buf, size)) {
    cdc_printf("UPLOAD_OK %s (%lu bytes saved)\n", path, (unsigned long)size);
  } else {
    cdc_print("エラー: Flash FS への保存に失敗しました\n");
  }
}

void parse_and_execute(char *line) {
  while (*line != '\0' && (unsigned char)*line <= ' ') ++line;
  char *end = line + strlen(line);
  while (end > line && (unsigned char)*(end - 1) <= ' ') *(--end) = '\0';
  if (*line == '\0') return;

  char *cmd = line;
  char *arg = strchr(line, ' ');
  if (arg != nullptr) {
    *arg++ = '\0';
    while (*arg != '\0' && (unsigned char)*arg <= ' ') ++arg;
  }

  if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
    cmd_help();
  } else if (strcmp(cmd, "ls") == 0) {
    cmd_ls();
  } else if (strcmp(cmd, "cat") == 0 && arg) {
    cmd_cat(arg);
  } else if (strcmp(cmd, "rm") == 0 && arg) {
    cmd_rm(arg);
  } else if (strcmp(cmd, "format") == 0) {
    cmd_format();
  } else if (strcmp(cmd, "ps") == 0 || strcmp(cmd, "threads") == 0) {
    cmd_ps();
  } else if (strcmp(cmd, "load") == 0 || strcmp(cmd, "run") == 0) {
    if (arg) {
      char path[32];
      uint32_t core = 0;
      if (sscanf(arg, "%31s %lu", path, (unsigned long *)&core) >= 1) {
        cmd_load(path, core);
      }
    } else {
      cdc_print("使用法: load <path> [core]\n");
    }
  } else if (strcmp(cmd, "swap") == 0 || strcmp(cmd, "hotswap") == 0) {
    if (arg) {
      char target[32], path[32];
      uint32_t core = 0;
      if (sscanf(arg, "%31s %31s %lu", target, path, (unsigned long *)&core) >= 2) {
        cmd_swap(target, path, core);
      } else {
        cdc_print("使用法: swap <target_name> <new_path> [core]\n");
      }
    } else {
      cdc_print("使用法: swap <target_name> <new_path> [core]\n");
    }
  } else if (strcmp(cmd, "unload") == 0 && arg) {
    cmd_unload(arg);
  } else if (strcmp(cmd, "call") == 0 && arg) {
    cmd_call(arg);
  } else if (strcmp(cmd, "mem") == 0 || strcmp(cmd, "free") == 0) {
    cmd_mem();
  } else if (strcmp(cmd, "reboot") == 0) {
    cmd_reboot();
  } else if (strcmp(cmd, "upload") == 0 && arg) {
    char path[32];
    size_t size = 0;
    if (sscanf(arg, "%31s %zu", path, &size) == 2) {
      cmd_upload(path, size);
    } else {
      cdc_print("使用法: upload <path> <size_in_bytes>\n");
    }
  } else {
    cdc_printf("不明なコマンド: '%s' ('help' でコマンド一覧を表示)\n", cmd);
  }
}

// ---- シェルメインループ (Core 0 固定) ------------------------------------------
uintptr_t shell_loop(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  api(shizuku::object_api::DECLARE_NAME, (uintptr_t)"shell");

  char line_buf[128];
  size_t line_len = 0;

  cdc_print("\nshizuku> ");

  uint32_t idle_ticks = 0;

  while (true) {
    int c = get_char_nonblock(0);
    if (c < 0) {
      api(shizuku::object_api::SLEEP_US, 2000); // 2ms
      // 500ms アイドルで未完了行をリセット (ゴミ文字蓄積防止)
      if (++idle_ticks > 250 && line_len > 0) {
        line_len = 0;
      }
      continue;
    }
    idle_ticks = 0;

    while (c >= 0) {
      if (c == '\r' || c == '\n') {
        if (line_len > 0) {
          cdc_print("\n");
          line_buf[line_len] = '\0';
          parse_and_execute(line_buf);
          line_len = 0;
          cdc_print("shizuku> ");
        }
      } else if (c == 0x08 || c == 0x7F) { // Backspace / Delete
        if (line_len > 0) --line_len;
      } else if (c >= 0x20 && c < 0x7F) {
        if (line_len + 1 < sizeof(line_buf)) {
          line_buf[line_len++] = (char)c;
        }
      }
      c = get_char_nonblock(0);
    }
  }
  return 0;
}

uintptr_t shell_main(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  api(shizuku::object_api::DECLARE_NAME, (uintptr_t)"shell");
  api(shizuku::object_api::EXPORT_METHOD, 1 /* POLL */, (uintptr_t)&shell_loop);
  return 0;
}

} // namespace

uint32_t register_shell(uintptr_t obj_id) {
  const auto created =
      api(shizuku::object_api::CREATE_OBJECT, obj_id, (uintptr_t)&shell_main,
          shizuku::OBJECT_PRIVILEGED | shizuku::OBJECT_ON_CORE(0));
  const auto exported = api(shizuku::object_api::CALL_METHOD, obj_id, 0, 0);
  return (created.error == 0 && exported.error == 0) ? 0 : 1;
}

uint32_t start_shell(uintptr_t obj_id) {
  const auto spawned = api(shizuku::object_api::SPAWN, obj_id, 1 /* POLL */, 0);
  return (spawned.error == 0) ? 0 : 1;
}

} // namespace xno::shell
