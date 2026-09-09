#include "neopixel.hpp"
#include "safety_status.hpp"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "shizuku/kernel.hpp"
#include "shizuku/object_api.hpp"
#include "shizuku/stream.hpp"
#include "tx_frame.hpp"
#include "ws2812.pio.h"

namespace xno::neopixel {
namespace {
using BOARD = shizuku::KERNEL::BOARD;
struct api_result { uintptr_t error; uintptr_t value; };
static inline api_result api(shizuku::object_api number, uintptr_t a1 = 0,
                             uintptr_t a2 = 0, uintptr_t a3 = 0) {
  register uintptr_t r0 asm("r0") = (uintptr_t)number;
  register uintptr_t r1 asm("r1") = a1;
  register uintptr_t r2 asm("r2") = a2;
  register uintptr_t r3 asm("r3") = a3;
  register uintptr_t r12 asm("r12") = 0;
  asm volatile("svc 0" : "+r"(r0), "+r"(r1)
               : "r"(r2), "r"(r3), "r"(r12) : "memory");
  return {r0, r1};
}

// Set only during wiring, before SPAWN. Runtime state belongs to poll_loop.
uintptr_t g_input_id = xno::NO_STREAM;
uintptr_t set_input(uintptr_t id, uintptr_t, uintptr_t, uintptr_t) {
  g_input_id = id;
  return 0;
}

uintptr_t poll_loop(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  PIO pio;
  uint sm, offset;
  if (!pio_claim_free_sm_and_add_program_for_gpio_range(
          &ws2812_program, &pio, &sm, &offset, GPIO_PIN, 1, true)) {
    BOARD::diag_printf("[NEOPIXEL] PIO allocation failed; indicator unavailable\n");
    return 1;
  }
  pio_gpio_init(pio, GPIO_PIN);
  pio_sm_set_consecutive_pindirs(pio, sm, GPIO_PIN, 1, true);
  pio_sm_config config = ws2812_program_get_default_config(offset);
  sm_config_set_sideset_pins(&config, GPIO_PIN);
  sm_config_set_out_shift(&config, false, true, 24); // GRB, MSB first
  sm_config_set_fifo_join(&config, PIO_FIFO_JOIN_TX);
  constexpr uint32_t BIT_RATE = 800000;
  constexpr uint32_t CYCLES = ws2812_T1 + ws2812_T2 + ws2812_T3;
  sm_config_set_clkdiv(&config, float(clock_get_hz(clk_sys)) / (BIT_RATE * CYCLES));
  const int init = pio_sm_init(pio, sm, offset, &config);
  if (init != 0) {
    BOARD::diag_printf("[NEOPIXEL] pio_sm_init failed: %d\n", init);
    pio_remove_program_and_unclaim_sm(&ws2812_program, pio, sm, offset);
    return 1;
  }
  pio_sm_set_enabled(pio, sm, true); // Empty FIFO stalls OUT with side-set low.
  BOARD::diag_printf("[NEOPIXEL] GP%u PIO%u SM%u, 800000 bit/s, RGB x1\n",
                     GPIO_PIN, pio_get_index(pio), sm);

  shizuku::stream::handle<safety::update> input;
  if (g_input_id != xno::NO_STREAM) {
    const auto opened = api(shizuku::object_api::STREAM_OPEN, g_input_id);
    if (opened.error == 0 && opened.value != 0) {
      const auto bound = api(shizuku::object_api::STREAM_BIND, g_input_id,
                            (uintptr_t)shizuku::stream::role::CONSUMER);
      if (bound.error == 0)
        input = shizuku::stream::handle<safety::update>(
            (shizuku::stream::descriptor *)opened.value);
      else
        BOARD::diag_printf("[NEOPIXEL] input bind failed: %lu\n",
                           (unsigned long)bound.error);
    } else {
      BOARD::diag_printf("[NEOPIXEL] input open failed: %lu\n",
                         (unsigned long)opened.error);
    }
  } else {
    BOARD::diag_printf("[NEOPIXEL] input stream not configured\n");
  }

  safety::status state;
  constexpr uint64_t PERIOD_US = 20000;
  // Never let late catch-up iterations enqueue adjacent pixels without a reset.
  constexpr uint64_t MIN_FRAME_US = 1000; // 30 us data + >=970 us low
  uint64_t last_frame = BOARD::time_us();
  uint64_t next = last_frame + PERIOD_US;
  bool fifo_warning = false;
  while (true) {
    const int64_t remaining = (int64_t)(next - BOARD::time_us());
    api(shizuku::object_api::SLEEP_US,
        (uintptr_t)(remaining > 0 ? remaining : 0));
    next += PERIOD_US;
    safety::update u{};
    while (input.valid()) {
      uint32_t lost = 0;
      const bool received = input.pop(&u, &lost);
      if (lost != 0) state = {}; // Require both records after an overrun.
      if (!received) break;
      state.accept(u);
    }
    const uint64_t now = BOARD::time_us();
    if (now - last_frame < MIN_FRAME_US) continue;
    if (pio_sm_is_tx_fifo_full(pio, sm)) {
      if (!fifo_warning)
        BOARD::diag_printf("[NEOPIXEL] TX FIFO full; indicator may be stale\n");
      fifo_warning = true;
      continue; // Never block the RTOS on a broken PIO.
    }
    fifo_warning = false;
    pio_sm_put(pio, sm, safety::pixel_grb(state.display(now), now) << 8);
    last_frame = now;
  }
}

uintptr_t neopixel_main(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  uintptr_t failures = api(shizuku::object_api::DECLARE_NAME,
                           (uintptr_t)"neopixel").error;
  failures += api(shizuku::object_api::EXPORT_METHOD,
                  (uintptr_t)method::POLL, (uintptr_t)&poll_loop).error;
  failures += api(shizuku::object_api::EXPORT_METHOD,
                  (uintptr_t)method::SET_INPUT_STREAM, (uintptr_t)&set_input).error;
  return failures;
}
} // namespace

uint32_t register_neopixel(uintptr_t obj_id) {
  const auto created = api(shizuku::object_api::CREATE_OBJECT, obj_id,
                           (uintptr_t)&neopixel_main, shizuku::OBJECT_ON_CORE(0));
  if (created.error != 0) return 1;
  const auto exported = api(shizuku::object_api::CALL_METHOD, obj_id, 0, 0);
  return exported.error != 0 || exported.value != 0 ? 1 : 0;
}
uint32_t start_neopixel(uintptr_t obj_id) {
  return api(shizuku::object_api::SPAWN, obj_id,
             (uintptr_t)method::POLL, 0).error != 0 ? 1 : 0;
}
} // namespace xno::neopixel
