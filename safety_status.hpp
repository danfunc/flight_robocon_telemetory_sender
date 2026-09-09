#ifndef XNO_SAFETY_STATUS_HPP
#define XNO_SAFETY_STATUS_HPP

#include <cstdint>
#include <limits>
#include <string_view>

// Shared wire contract and pure display policy; no hardware or shared state.
namespace xno::safety {
enum class kind : uint8_t { safe, safe2 };
struct update {
  uint64_t received_us = 0; // Pico receive time, not XIAO's clock
  uint32_t gear_us = 0;    // Diagnostic only: XIAO's adjudicated mode wins.
  kind type = kind::safe;
  bool mode_auto = false;
  bool unhealthy = true;
};

inline bool unsigned_field(std::string_view s, uint64_t limit,
                           uint64_t &value, unsigned base = 10) {
  if (s.empty()) return false;
  value = 0;
  for (char ch : s) {
    unsigned d;
    if (ch >= '0' && ch <= '9') d = ch - '0';
    else if (ch >= 'A' && ch <= 'F') d = ch - 'A' + 10;
    else if (ch >= 'a' && ch <= 'f') d = ch - 'a' + 10;
    else return false;
    if (d >= base || d > limit || value > (limit - d) / base)
      return false;
    value = value * base + d;
  }
  return true;
}

// Exact field count, bounded numbers, no allocation, no UART/BLE response.
// Unknown SAFE variants and malformed records do not refresh freshness.
inline bool parse(std::string_view line, uint64_t now, update &out) {
  std::string_view fields[7];
  unsigned count = 0;
  while (true) {
    if (count == 7) return false;
    const auto comma = line.find(',');
    fields[count++] = line.substr(0, comma);
    if (comma == std::string_view::npos) break;
    line.remove_prefix(comma + 1);
  }
  constexpr auto U32 = std::numeric_limits<uint32_t>::max();
  constexpr auto U64 = std::numeric_limits<uint64_t>::max();
  update parsed{};
  parsed.received_us = now;
  uint64_t ignored, mode, failsafe, link, gear;
  if (fields[0] == "SAFE" && count == 7) {
    if (!unsigned_field(fields[1], U32, ignored) ||
        !unsigned_field(fields[2], U64, ignored) ||
        !unsigned_field(fields[3], 1, failsafe) ||
        !unsigned_field(fields[4], 1, mode) ||
        !unsigned_field(fields[5], 1, link) ||
        fields[6].size() != 6 || fields[6].substr(0, 2) != "0x" ||
        !unsigned_field(fields[6].substr(2), 0xffff, ignored, 16))
      return false;
    parsed.type = kind::safe;
    parsed.mode_auto = mode != 0;
    parsed.unhealthy = failsafe != 0 || link == 0;
  } else if (fields[0] == "SAFE2" && count == 6) {
    if (!unsigned_field(fields[1], U32, gear) || fields[2].empty() ||
        (fields[3] != "AUTO" && fields[3] != "MANUAL") ||
        !unsigned_field(fields[4], U32, ignored) ||
        !unsigned_field(fields[5], U32, ignored))
      return false;
    parsed.type = kind::safe2;
    parsed.gear_us = static_cast<uint32_t>(gear);
    parsed.mode_auto = fields[3] == "AUTO";
    // New/unknown faults fail visibly too. PILOT_MANUAL is a normal mode.
    parsed.unhealthy = fields[2] != "NONE" &&
                       !(fields[2] == "PILOT_MANUAL" && !parsed.mode_auto);
  } else {
    return false;
  }
  out = parsed;
  return true;
}

enum class indication { red, green, flashing_red };
constexpr uint64_t STALE_US = 1000000;
constexpr uint64_t FLASH_HALF_PERIOD_US = 250000; // 2 Hz, 50% duty
constexpr uint8_t BRIGHTNESS = 255; // One colour channel on one RGB LED.

class status {
 public:
  void accept(const update &u) {
    if (u.type == kind::safe) { safe_ = u; have_safe_ = true; }
    else { safe2_ = u; have_safe2_ = true; }
  }
  indication display(uint64_t now) const {
    if (!have_safe_ || !have_safe2_ ||
        now - safe_.received_us >= STALE_US ||
        now - safe2_.received_us >= STALE_US ||
        safe_.unhealthy || safe2_.unhealthy ||
        safe_.mode_auto != safe2_.mode_auto)
      return indication::flashing_red;
    return safe2_.mode_auto ? indication::green : indication::red;
  }
 private:
  update safe_{}, safe2_{};
  bool have_safe_ = false, have_safe2_ = false;
};

inline uint32_t pixel_grb(indication value, uint64_t now) {
  if (value == indication::green) return uint32_t{BRIGHTNESS} << 16;
  if (value == indication::flashing_red &&
      (now / FLASH_HALF_PERIOD_US) % 2 != 0) return 0;
  return uint32_t{BRIGHTNESS} << 8;
}
} // namespace xno::safety
#endif
