#include "safety_status.hpp"
#include <cstdio>
#include <cstdlib>

using namespace xno::safety;
static unsigned checks = 0;
static void check(bool condition, const char *message) {
  ++checks;
  if (!condition) {
    std::fprintf(stderr, "FAIL: %s\n", message);
    std::exit(1);
  }
}
static void receive(status &s, const char *line, uint64_t now) {
  update u{};
  check(parse(line, now, u), line);
  s.accept(u);
}
int main() {
  status s;
  check(s.display(0) == indication::flashing_red, "startup is visible fault");
  receive(s, "SAFE,1,10,0,1,1,0x00FF", 100);
  check(s.display(100) == indication::flashing_red, "SAFE alone cannot turn green");
  receive(s, "SAFE2,1000,NONE,AUTO,0,17", 200);
  check(s.display(200) == indication::green, "adjudicated AUTO wins over gear");
  check(s.display(1000099) == indication::green, "just before SAFE timeout");
  check(s.display(1000100) == indication::flashing_red, "SAFE timeout at exactly 1s");
  receive(s, "SAFE,2,1010,0,1,1,0x0000", 1000150);
  check(s.display(1000200) == indication::flashing_red, "SAFE2 timeout at exactly 1s");
  receive(s, "SAFE2,2000,NONE,AUTO,0,18", 1000201);
  check(s.display(1000201) == indication::green, "valid data recovers");
  receive(s, "SAFE,3,1011,0,0,1,0x00FF", 1000300);
  check(s.display(1000300) == indication::flashing_red, "mode disagreement cannot stay green");
  receive(s, "SAFE2,2000,PILOT_MANUAL,MANUAL,0,19", 1000400);
  check(s.display(1000400) == indication::red, "manual wins even with high gear");
  receive(s, "SAFE,4,1012,1,0,0,0x0000", 1000500);
  check(s.display(1000500) == indication::flashing_red, "link loss immediately visible");
  receive(s, "SAFE,5,1013,0,0,1,0x00FF", 1000600);
  receive(s, "SAFE2,0,LINK_LOST,MANUAL,1,20", 1000600);
  check(s.display(1000600) == indication::flashing_red, "SAFE2 link loss overrides SAFE");
  receive(s, "SAFE2,1500,BAD_PULSE,MANUAL,1,21", 1000700);
  check(s.display(1000700) == indication::flashing_red, "bad pulse flashes");
  receive(s, "SAFE2,1800,SW_INHIBIT,MANUAL,1,22", 1000800);
  check(s.display(1000800) == indication::flashing_red, "software inhibit flashes");
  receive(s, "SAFE2,1800,FUTURE_FAULT,MANUAL,1,23", 1000900);
  check(s.display(1000900) == indication::flashing_red, "unknown fault cannot show normal mode");
  receive(s, "SAFE2,1800,NONE,MANUAL,1,24", 1001000);
  receive(s, "SAFE,6,1014,1,0,1,0x00FF", 1001000);
  check(s.display(1001000) == indication::flashing_red, "failsafe with link still up flashes");

  const char *bad[] = {
      "SAFE", "SAFE_ALERT,future", "SAFE2,0,LINK_LOST",
      "SAFE,1,2,0,1,1,0xFFFF,extra", "SAFE,1,2,0,1,1,0xFFFF,",
      "SAFE,1,2,0,2,1,0xFFFF", "SAFE,1,2,0,1,2,0xFFFF",
      "SAFE,1,2,2,1,1,0xFFFF", "SAFE,-1,2,0,1,1,0xFFFF",
      "SAFE,4294967296,2,0,1,1,0xFFFF",
      "SAFE,1,18446744073709551616,0,1,1,0xFFFF",
      "SAFE,1,2,0,1,1,0xFGFF", "SAFE,1,2,0,1,1,FFFF",
      "SAFE2,2000,,AUTO,0,1", "SAFE2,2000,NONE,auto,0,1",
      "SAFE2,2000,NONE,AUTO,0,1x", "SAFE2,,NONE,AUTO,0,1",
      "SAFE2,-1,NONE,AUTO,0,1", "SAFE2,+1,NONE,AUTO,0,1",
      "SAFE2,4294967296,NONE,AUTO,0,1", "SAFE2,2000,NONE,AUTO,0,1,extra",
      "SAFE2,2000,NONE,AUTO,0,", "SAFE2,2000,NONE,AUTO,0,1\n",
      "SAFE2,2000,NONE,AUTO,4294967296,1", "SAFE2,2000,NONE,AUTO,0,4294967296",
  };
  for (const auto *line : bad) {
    update u{};
    u.received_us = 42;
    check(!parse(line, 9999999, u), line);
    check(u.received_us == 42, "invalid record leaves freshness unchanged");
  }
  update u{};
  check(parse("SAFE,4294967295,18446744073709551615,0,1,1,0xFFFF", 0, u),
        "maximum numeric fields accepted");
  check(parse("SAFE2,4294967295,NONE,AUTO,4294967295,4294967295", 0, u),
        "maximum SAFE2 fields accepted");
  check(pixel_grb(indication::red, 0) == 0x00ff00, "red GRB byte order");
  check(pixel_grb(indication::green, 0) == 0xff0000, "green GRB byte order");
  check(pixel_grb(indication::flashing_red, 249999) == 0x00ff00, "blink on");
  check(pixel_grb(indication::flashing_red, 250000) == 0, "blink off boundary");
  check(pixel_grb(indication::flashing_red, 500000) == 0x00ff00, "blink restart");
  std::printf("PASS: %u checks (production parser and display policy)\n", checks);
}
