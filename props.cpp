// ===========================================================================
//  props — 実装 (flashfs オブジェクトへの薄い口)
// ===========================================================================
//  設計の理由は props.hpp 冒頭。
#include "props.hpp"
#include "shizuku/kernel.hpp"
#include "shizuku/object_api.hpp"
#include "shizuku/objects/flash_fs.hpp"
#include <cstring>

namespace xno::props {
namespace {

using shizuku::object_api;
using KERNEL = shizuku::KERNEL;
using shizuku::objects::FLASH_FS_OBJECT;
using shizuku::objects::flash_fs_method;

uintptr_t call(flash_fs_method method, uintptr_t argument) {
  const auto result =
      KERNEL::ARCH::syscall((uintptr_t)object_api::CALL_METHOD, FLASH_FS_OBJECT,
                            (uintptr_t)method, argument);
  // ★エラーは value ではなく error に載る。value をそのまま返すと
  //   「呼べなかった (未生成)」と「呼べたが 0 だった」が区別できない。
  return result.error == 0 ? result.value : 0;
}

// 目録に載る名前は媒体側で長さが固定されている。溢れる名前を黙って切ると、
// 別々のプロパティが同じ名前になって上書きし合うので、**弾く**。
bool name_fits(const char *name) {
  if (name == nullptr)
    return false;
  // ★strnlen は newlib-nano の既定では出てこないので自分で数える。
  for (uint32_t index = 0; index < MAX_NAME_BYTES; ++index)
    if (name[index] == '\0')
      return true;
  return false;
}

} // namespace

bool available() {
  // STATUS は副作用が無く、未生成なら CALL_METHOD が error を返す。
  shizuku::objects::flash_status status{};
  const auto result = KERNEL::ARCH::syscall(
      (uintptr_t)object_api::CALL_METHOD, FLASH_FS_OBJECT,
      (uintptr_t)flash_fs_method::STATUS, (uintptr_t)&status);
  return result.error == 0 && status.region_bytes != 0;
}

bool load(const char *name, uint32_t magic, uint16_t version, void *out,
          uint32_t bytes) {
  if (!name_fits(name) || out == nullptr || bytes < sizeof(header))
    return false;
  shizuku::objects::flash_lookup lookup{name, 0, 0};
  call(flash_fs_method::LOOKUP, (uintptr_t)&lookup);
  if (lookup.address == 0)
    return false;
  // ★写す前に検査する。載っているバイト数がこちらの構造体と違うなら、それは
  //   別の形なので、読んだ時点で意味が違う。
  if (lookup.bytes != bytes)
    return false;
  // ★XIP なのでそのまま読める (中継バッファは要らない)。
  const header *head = (const header *)lookup.address;
  if (head->magic != magic || head->version != version ||
      head->bytes != (uint16_t)bytes)
    return false;
  memcpy(out, (const void *)lookup.address, bytes);
  return true;
}

bool store(const char *name, const void *data, uint32_t bytes) {
  if (!name_fits(name) || data == nullptr || bytes < sizeof(header))
    return false;
  // 呼ぶ側が頭を埋め忘れていないかを見る (埋め忘れると次回の load が必ず
  // 失敗し、しかも「保存できていない」ようにしか見えない)。
  const header *head = (const header *)data;
  if (head->bytes != (uint16_t)bytes)
    return false;
  shizuku::objects::flash_store request{name, data, bytes, 0};
  call(flash_fs_method::STORE, (uintptr_t)&request);
  return request.address != 0;
}

bool remove(const char *name) {
  if (!name_fits(name))
    return false;
  shizuku::objects::flash_lookup lookup{name, 0, 0};
  return call(flash_fs_method::REMOVE, (uintptr_t)&lookup) != 0;
}

} // namespace xno::props
