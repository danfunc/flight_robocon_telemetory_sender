#ifndef SHIZU_I2C_BUS_HPP
#define SHIZU_I2C_BUS_HPP
// ===========================================================================
//  共有 I2C バスヘルパー (BME280_DRIVER / BNO055_DRIVER 兼用)
// ===========================================================================
//  どちらのセンサも i2c0 (SDA=GP4, SCL=GP5) にぶら下がる。Shizuku は協調型
//  スケジューリングなので、1 回の I2C トランザクション中 (write/read) に yield
//  しない限り別スレッドへ切り替わらず、トランザクションは原子的になる。よって
//  別オブジェクト (別スレッド) が同じバスを触っても、各自が「自分のスレッドだけ」で
//  I2C を叩く限り衝突しない。init() は冪等なので両ドライバが各々呼んでよい。
// ===========================================================================
#include <cstddef>
#include <cstdint>
#include <hardware/gpio.h>
#include <hardware/i2c.h>
#include <pico/time.h> // busy_wait_us (バス救出のビットバンギング用)

namespace shizu {
namespace i2c_bus {

static constexpr uint SDA_PIN = 4;
static constexpr uint SCL_PIN = 5;
static constexpr uint FREQ_HZ = 100000; // 100kHz Standard-mode (BNO055 は 400kHz
                                        // でバースト末尾が化けるため低速で運用)
// 1 トランザクションの上限。100kHz では 26B 読みが ~2.4ms かかるので、2ms 等にすると
// calib(BME 0x88×26B)/motion(BNO 0x1A×26B) がタイムアウトして失敗する。4x 余裕。
static constexpr uint TIMEOUT_US = 10000;

// i2c0 はマクロ (ポインタ) なので関数で包んで使う。
static inline i2c_inst_t *port() { return i2c0; }

// I2C0 を初期化する。二重に呼んでも害は無い (再初期化されるだけ)。
static inline void init() {
  i2c_init(port(), FREQ_HZ);
  gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(SDA_PIN);
  gpio_pull_up(SCL_PIN);
}

// 1 レジスタ書き込み。戻り値 < 0 で失敗 (タイムアウト/NACK)。
static inline int write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
  uint8_t buf[2] = {reg, val};
  return i2c_write_timeout_us(port(), addr, buf, 2, false, TIMEOUT_US);
}

// reg からの連続読み出し。戻り値 < 0 で失敗。
static inline int read_regs(uint8_t addr, uint8_t reg, uint8_t *buf,
                            size_t len) {
  int ret = i2c_write_timeout_us(port(), addr, &reg, 1, true, TIMEOUT_US);
  if (ret < 0)
    return ret;
  return i2c_read_timeout_us(port(), addr, buf, len, false, TIMEOUT_US);
}

// バス救出 (I2C-bus spec の bus clear 手順)。スレーブが転送途中で固まって
// SDA を掴んだままになると (BNO055 はクロックストレッチ起因のロックアップを
// 起こすことで知られる)、以後の全トランザクションがタイムアウトし続ける。
// ピンを GPIO に戻して SCL を手動で 9 クロック叩き、スレーブに残りビットを
// 吐かせて SDA を解放させ、STOP 条件を打ってから I2C を再初期化する。
// 全体で ~100µs のビジーウェイトのみ (yield 不要、失敗側スレッドから呼んでよい)。
static inline void recover() {
  // open-drain 相当の駆動: 出力ラッチを Low に固定し、方向だけを
  // OUT(=Low 駆動) / IN(=解放、pull-up で High) と切り替える。
  gpio_set_function(SDA_PIN, GPIO_FUNC_SIO);
  gpio_set_function(SCL_PIN, GPIO_FUNC_SIO);
  gpio_put(SDA_PIN, 0);
  gpio_put(SCL_PIN, 0);
  gpio_set_dir(SDA_PIN, GPIO_IN);
  gpio_set_dir(SCL_PIN, GPIO_IN);
  gpio_pull_up(SDA_PIN);
  gpio_pull_up(SCL_PIN);
  busy_wait_us(5);
  for (int i = 0; i < 9 && !gpio_get(SDA_PIN); ++i) {
    gpio_set_dir(SCL_PIN, GPIO_OUT); // SCL Low
    busy_wait_us(5);                 // 100kHz 相当
    gpio_set_dir(SCL_PIN, GPIO_IN);  // SCL High (pull-up)
    busy_wait_us(5);
  }
  // STOP 条件: SCL High のまま SDA を Low → High。
  gpio_set_dir(SDA_PIN, GPIO_OUT);
  busy_wait_us(5);
  gpio_set_dir(SDA_PIN, GPIO_IN);
  busy_wait_us(5);
  init(); // ピンを I2C 機能へ戻し、コントローラも再初期化 (冪等)
}

} // namespace i2c_bus
} // namespace shizu

#endif // SHIZU_I2C_BUS_HPP
