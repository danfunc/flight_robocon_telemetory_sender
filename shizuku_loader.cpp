#include "shizuku_loader.hpp"
#include "flash_fs.hpp"
#include "object_ids.hpp"
#include "shizuku/kernel.hpp"
#include "shizuku/object_api.hpp"
#include <cstdio>
#include <cstring>

namespace xno::loader {
namespace {

constexpr size_t MAX_LOADED_OBJECTS = 8;
constexpr uint32_t DYN_OBJ_START = (uint32_t)xno_object_id::dyn_obj_start;
constexpr uint32_t DYN_OBJ_END = (uint32_t)xno_object_id::dyn_obj_end;

// 32-bit ARM ELF structures
struct elf32_ehdr {
  uint8_t e_ident[16];   // 0x7F, 'E', 'L', 'F', ...
  uint16_t e_type;       // 2 = ET_EXEC, 3 = ET_DYN
  uint16_t e_machine;    // 40 = EM_ARM
  uint32_t e_version;
  uint32_t e_entry;      // Entry point virtual address / offset
  uint32_t e_phoff;      // Program header table file offset
  uint32_t e_shoff;      // Section header table file offset
  uint32_t e_flags;
  uint16_t e_ehsize;
  uint16_t e_phentsize;
  uint16_t e_phnum;
  uint16_t e_shentsize;
  uint16_t e_shnum;
  uint16_t e_shstrndx;
};

struct elf32_phdr {
  uint32_t p_type;       // 1 = PT_LOAD
  uint32_t p_offset;     // Segment file offset
  uint32_t p_vaddr;      // Virtual address
  uint32_t p_paddr;
  uint32_t p_filesz;
  uint32_t p_memsz;
  uint32_t p_flags;
  uint32_t p_align;
};

bool is_elf(const uint8_t *data, size_t size) {
  if (size < sizeof(elf32_ehdr))
    return false;
  return (data[0] == 0x7F && data[1] == 'E' && data[2] == 'L' &&
          data[3] == 'F');
}

uintptr_t get_entry_offset(const uint8_t *data, size_t size) {
  if (!is_elf(data, size)) {
    return 0; // Flat PIC binary starts at offset 0
  }
  const auto *ehdr = (const elf32_ehdr *)data;
  if (ehdr->e_phoff > 0 && ehdr->e_phnum > 0 &&
      (ehdr->e_phoff + sizeof(elf32_phdr) <= size)) {
    const auto *phdrs = (const elf32_phdr *)(data + ehdr->e_phoff);
    for (uint16_t i = 0; i < ehdr->e_phnum; ++i) {
      if (phdrs[i].p_type == 1 /* PT_LOAD */) {
        if (ehdr->e_entry >= phdrs[i].p_vaddr &&
            ehdr->e_entry < phdrs[i].p_vaddr + phdrs[i].p_filesz) {
          return phdrs[i].p_offset + (ehdr->e_entry - phdrs[i].p_vaddr);
        }
      }
    }
  }
  return ehdr->e_entry;
}

struct api_result {
  uintptr_t error;
  uintptr_t value;
};

static inline api_result api(shizuku::object_api number, uintptr_t a1 = 0,
                             uintptr_t a2 = 0, uintptr_t a3 = 0,
                             uintptr_t a4 = 0) {
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

loaded_info g_loaded_objects[MAX_LOADED_OBJECTS];
uint32_t g_next_obj_id = DYN_OBJ_START;

// 静的 RAM バッファプール (動的オブジェクト用ヒープ)
alignas(16) uint8_t g_obj_ram_pool[MAX_LOADED_OBJECTS][2048];
bool g_pool_allocated[MAX_LOADED_OBJECTS] = {false};

int allocate_ram_slot() {
  for (size_t i = 0; i < MAX_LOADED_OBJECTS; ++i) {
    if (!g_pool_allocated[i]) {
      g_pool_allocated[i] = true;
      return (int)i;
    }
  }
  return -1;
}

void free_ram_slot(int slot) {
  if (slot >= 0 && slot < (int)MAX_LOADED_OBJECTS) {
    g_pool_allocated[slot] = false;
  }
}

int find_loaded_slot(uint32_t obj_id) {
  for (size_t i = 0; i < MAX_LOADED_OBJECTS; ++i) {
    if (g_loaded_objects[i].is_active &&
        g_loaded_objects[i].object_id == obj_id) {
      return (int)i;
    }
  }
  return -1;
}

int find_free_loaded_slot() {
  for (size_t i = 0; i < MAX_LOADED_OBJECTS; ++i) {
    if (!g_loaded_objects[i].is_active) {
      return (int)i;
    }
  }
  return -1;
}

// ファイルパスからベースネーム (例: "/bin/blink.bin" -> "blink") を抽出
void extract_name(const char *path, char *out_name, size_t max_len) {
  const char *slash = strrchr(path, '/');
  const char *base = slash ? (slash + 1) : path;
  size_t i = 0;
  while (base[i] != '\0' && base[i] != '.' && i + 1 < max_len) {
    out_name[i] = base[i];
    ++i;
  }
  out_name[i] = '\0';
}

} // namespace

void init() {
  memset(g_loaded_objects, 0, sizeof(g_loaded_objects));
  memset(g_pool_allocated, 0, sizeof(g_pool_allocated));
  shizuku::KERNEL::BOARD::diag_printf(
      "[LOADER] initialized dynamic object loader (ELF & PIC support)\n");
}

bool load_object(const char *path, uint32_t core_id, bool use_xip,
                 loaded_info *out_info) {
  // 1. 静的 blink を RAM 上で確実に停止 (LED ハードウェアを動的モジュールへ譲渡)
  api(shizuku::object_api::CALL_METHOD, xno_object_id::blink, 3 /* SET_ENABLED */, 0);
  api(shizuku::object_api::CALL_METHOD, xno_object_id::blink, 5 /* STOP */, 0);

  // 2. 既存の動的オブジェクトをすべて停止・アンロード (LED 競合防止)
  for (size_t i = 0; i < MAX_LOADED_OBJECTS; ++i) {
    if (g_loaded_objects[i].is_active) {
      unload_object(g_loaded_objects[i].object_id);
    }
  }

  fs::stat_t st{};
  if (!fs::stat(path, &st)) {
    shizuku::KERNEL::BOARD::diag_printf("[LOADER] file not found: %s\n", path);
    return false;
  }

  int slot = find_free_loaded_slot();
  if (slot < 0) {
    shizuku::KERNEL::BOARD::diag_printf(
        "[LOADER] no free object tracking slots\n");
    return false;
  }

  uintptr_t code_ptr = 0;
  int ram_slot = -1;

  if (use_xip) {
    code_ptr = st.xip_address;
  } else {
    ram_slot = allocate_ram_slot();
    if (ram_slot < 0) {
      shizuku::KERNEL::BOARD::diag_printf(
          "[LOADER] RAM pool full (max %zu objects)\n", MAX_LOADED_OBJECTS);
      return false;
    }
    if (st.size > sizeof(g_obj_ram_pool[0])) {
      shizuku::KERNEL::BOARD::diag_printf(
          "[LOADER] binary too large: %zu bytes (max %zu)\n", st.size,
          sizeof(g_obj_ram_pool[0]));
      free_ram_slot(ram_slot);
      return false;
    }
    uint8_t *dest = g_obj_ram_pool[ram_slot];
    size_t actual = 0;
    if (!fs::read_file(path, dest, sizeof(g_obj_ram_pool[0]), &actual)) {
      shizuku::KERNEL::BOARD::diag_printf(
          "[LOADER] failed to read file to RAM: %s\n", path);
      free_ram_slot(ram_slot);
      return false;
    }
    code_ptr = (uintptr_t)dest;
  }

  // ELF ヘッダまたはフラット PIC バイナリからエントリポイントオフセットを計算
  uintptr_t entry_offset =
      get_entry_offset((const uint8_t *)code_ptr, st.size);
  uintptr_t entry_thumb = (code_ptr + entry_offset) | 1u;

  // オブジェクト ID: 単調増加カウンタでユニークに割り当て (ID 衝突防止)
  uint32_t obj_id = g_next_obj_id++;
  if (g_next_obj_id >= DYN_OBJ_START + 64) {
    g_next_obj_id = DYN_OBJ_START;
  }

  // 1. オブジェクト生成 (CREATE_OBJECT) - 再作成許可フラグ (0x80000000) 付き
  uint32_t core_flag = (core_id == 0)
                           ? (uint32_t)shizuku::OBJECT_ON_CORE(0)
                           : (uint32_t)shizuku::OBJECT_ON_CORE(1);
  uint32_t create_flags =
      (uint32_t)shizuku::OBJECT_PRIVILEGED | core_flag;

  const auto created =
      api(shizuku::object_api::CREATE_OBJECT, obj_id, entry_thumb, create_flags);
  if (created.error != 0) {
    shizuku::KERNEL::BOARD::diag_printf(
        "[LOADER] CREATE_OBJECT failed (obj_id=%lu, error=%lu)\n",
        (unsigned long)obj_id, (unsigned long)created.error);
    if (ram_slot >= 0)
      free_ram_slot(ram_slot);
    return false;
  }

  // 2. オブジェクト初期化 (CALL_METHOD 0: エントリポイントを実行してメソッドをエクスポート & blink 停止)
  const auto init_res = api(shizuku::object_api::CALL_METHOD, obj_id, 0, 0);
  if (init_res.error != 0) {
    shizuku::KERNEL::BOARD::diag_printf(
        "[LOADER] CALL_METHOD(0) init failed (obj_id=%lu, error=%lu)\n",
        (unsigned long)obj_id, (unsigned long)init_res.error);
  }

  // 3. オブジェクトのポーリングスレッド起動 (SPAWN メソッド 1: POLL)
  const auto spawned = api(shizuku::object_api::SPAWN, obj_id, 1 /* POLL */, 0);
  if (spawned.error != 0) {
    shizuku::KERNEL::BOARD::diag_printf(
        "[LOADER] SPAWN failed (obj_id=%lu, error=%lu)\n",
        (unsigned long)obj_id, (unsigned long)spawned.error);
    if (ram_slot >= 0)
      free_ram_slot(ram_slot);
    return false;
  }

  uint32_t thread_id = (uint32_t)spawned.value;

  // トラッキング情報記録
  loaded_info &info = g_loaded_objects[slot];
  info.object_id = obj_id;
  info.thread_id = thread_id;
  extract_name(path, info.name, sizeof(info.name));
  strncpy(info.path, path, sizeof(info.path) - 1);
  info.code_addr = code_ptr;
  info.size = st.size;
  info.is_xip = use_xip;
  info.is_active = true;

  shizuku::KERNEL::BOARD::diag_printf(
      "[LOADER] successfully loaded '%s' -> object %lu, thread %lu (entry: "
      "0x%08lx, %s, %s)\n",
      info.name, (unsigned long)obj_id, (unsigned long)thread_id,
      (unsigned long)entry_thumb, use_xip ? "XIP" : "RAM",
      is_elf((const uint8_t *)code_ptr, st.size) ? "ELF" : "PIC_BIN");

  if (out_info != nullptr) {
    *out_info = info;
  }
  return true;
}

bool unload_object(uint32_t obj_id) {
  int slot = find_loaded_slot(obj_id);
  if (slot < 0)
    return false;

  loaded_info &info = g_loaded_objects[slot];
  // メソッド 5 (STOP) を呼び出して安全に停止
  api(shizuku::object_api::CALL_METHOD, obj_id, 5 /* STOP */, 0);

  // RAM スロットの解放
  for (size_t i = 0; i < MAX_LOADED_OBJECTS; ++i) {
    if ((uintptr_t)g_obj_ram_pool[i] == info.code_addr) {
      free_ram_slot((int)i);
      break;
    }
  }

  shizuku::KERNEL::BOARD::diag_printf("[LOADER] unloaded object %lu (%s)\n",
                                      (unsigned long)obj_id, info.name);
  info.is_active = false;

  // すべての動的オブジェクトがアンロードされた場合、組み込み blink を復帰
  bool any_active = false;
  for (size_t i = 0; i < MAX_LOADED_OBJECTS; ++i) {
    if (g_loaded_objects[i].is_active) {
      any_active = true;
      break;
    }
  }
  if (!any_active) {
    api(shizuku::object_api::CALL_METHOD, xno_object_id::blink, 3 /* SET_ENABLED */, 1);
  }

  return true;
}

bool unload_object_by_name(const char *name) {
  for (size_t i = 0; i < MAX_LOADED_OBJECTS; ++i) {
    if (g_loaded_objects[i].is_active &&
        strcmp(g_loaded_objects[i].name, name) == 0) {
      return unload_object(g_loaded_objects[i].object_id);
    }
  }
  return false;
}

bool hot_swap(const char *target_name, const char *new_path, uint32_t core_id,
              loaded_info *out_info) {
  shizuku::KERNEL::BOARD::diag_printf(
      "[LOADER] hot-swapping '%s' -> '%s'...\n", target_name, new_path);

  // 1. 静的 blink を RAM 上で確実に停止 (LED ハードウェアを動的モジュールへ譲渡)
  api(shizuku::object_api::CALL_METHOD, xno_object_id::blink, 3 /* SET_ENABLED */, 0);
  api(shizuku::object_api::CALL_METHOD, xno_object_id::blink, 5 /* STOP */, 0);

  // 2. 既存の動的オブジェクトをアンロード
  unload_object_by_name(target_name);

  // 3. 新しいモジュールを Flash FS から XIP ロード & 起動
  return load_object(new_path, core_id, true, out_info);
}

size_t list_loaded(loaded_info *out_list, size_t max_entries) {
  size_t count = 0;
  for (size_t i = 0; i < MAX_LOADED_OBJECTS && count < max_entries; ++i) {
    if (g_loaded_objects[i].is_active) {
      out_list[count++] = g_loaded_objects[i];
    }
  }
  return count;
}

void run_autorun() {
  fs::stat_t st{};
  if (fs::stat("/bin/autorun.bin", &st) || fs::stat("/bin/autorun.elf", &st)) {
    const char *path =
        fs::stat("/bin/autorun.bin", &st) ? "/bin/autorun.bin" : "/bin/autorun.elf";
    shizuku::KERNEL::BOARD::diag_printf(
        "[LOADER] autorun: launching %s from Flash FS...\n", path);
    load_object(path, 0, true);
  }
}

} // namespace xno::loader
