// ===========================================================================
//  fw_version — 実装。理由は fw_version.hpp 冒頭。
// ===========================================================================
#include "fw_version.hpp"

extern "C" {
extern const uint8_t __flash_binary_start[];
extern const uint8_t __flash_binary_end[];
}

namespace xno {
namespace {

// ★半バイト表。256 語の表を持つより小さく、速度もこの用途では十分
//   (焼けたかの確認は起動時と、たまの問い合わせだけ)。
constexpr uint32_t CRC_NIBBLE[16] = {
    0x00000000, 0x1DB71064, 0x3B6E20C8, 0x26D930AC, 0x76DC4190, 0x6B6B51F4,
    0x4DB26158, 0x5005713C, 0xEDB88320, 0xF00F9344, 0xD6D6A3E8, 0xCB61B38C,
    0x9B64C2B0, 0x86D3D2D4, 0xA00AE278, 0xBDBDF21C};

fw_id g_id{};
bool g_done = false;

} // namespace

const fw_id &firmware_id() {
  if (g_done)
    return g_id;
  const uint32_t size =
      (uint32_t)((uintptr_t)__flash_binary_end - (uintptr_t)__flash_binary_start);
  const uint8_t *p = __flash_binary_start;
  uint32_t crc = 0xFFFFFFFFu;
  for (uint32_t i = 0; i < size; ++i) {
    crc ^= p[i];
    crc = (crc >> 4) ^ CRC_NIBBLE[crc & 0x0F];
    crc = (crc >> 4) ^ CRC_NIBBLE[crc & 0x0F];
  }
  g_id.bytes = size;
  g_id.crc32 = crc ^ 0xFFFFFFFFu;
  // ★控えを埋めてから旗を立てる。逆だと、途中の値を他コアが完成品として読む。
  g_done = true;
  return g_id;
}

} // namespace xno
