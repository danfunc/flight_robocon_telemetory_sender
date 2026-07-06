#include <call_method.hpp>
#include <cstdint>
#include <hardware/pwm.h>
#include <obj_api.hpp>
#include <object_headers/BLE_UART_DRIVER.hpp>
#include <object_headers/BME280_DRIVER.hpp>
#include <object_headers/BNO055_DRIVER.hpp>
#include <object_headers/CYW43_BL_DRIVER.hpp>
#include <object_headers/FLIGHT_CONTROLLER.hpp>
#include <object_headers/HELLO_WORLD.hpp>
#include <object_headers/IO_CONTROLLER.hpp>
#include <object_headers/LINE_SENSOR_DRIVER.hpp>
#include <object_headers/TELEMETRY_SENDER.hpp>
#include <object_headers/WS2812_DRIVER.hpp>
#include <object_id.hpp>
#include <pico/stdlib.h>
#include <stdio.h>

void shizu::IO_CONTROLLER::main() {
  printf("IO_CONTROLLER::init\n");
  obj_api::create_object((uint32_t)object_ids::WS2812_DRIVER, 10,
                         (uintptr_t)WS2812_DRIVER::init);

  // CYW43 の無線は排他利用のため、CYW43_BL_DRIVER (プロコン HID) と
  // BLE_UART_DRIVER は同時起動できない。ここでは BLE UART を起動する。
  // プロコン制御に戻す場合は下のオブジェクト生成を入れ替える。
  // obj_api::create_object((uint32_t)object_ids::CYW43_BL_DRIVER, 9,
  //                        (uintptr_t)CYW43_BL_DRIVER::init);
  obj_api::create_object((uint32_t)object_ids::BLE_UART_DRIVER, 11,
                         (uintptr_t)BLE_UART_DRIVER::init);

  // 飛行テレメトリ構成:
  //   BME280_DRIVER / BNO055_DRIVER … I2C センサを各々 Shizuku オブジェクト化し、
  //                                    50Hz で読み続けて最新値をキャッシュする。
  //   TELEMETRY_SENDER … 上記 2 つを read_latest で購読し、相補フィルタで融合して
  //                       BLE_UART 経由で母艦 (mac) へ CSV テレメトリを送る。
  // ※ BLE_UART を先に立ててから TELEMETRY (set_rx_sink / send_byte を叩く) を生成。
  obj_api::create_object((uint32_t)object_ids::BME280_DRIVER, 13,
                         (uintptr_t)BME280_DRIVER::init);
  obj_api::create_object((uint32_t)object_ids::BNO055_DRIVER, 14,
                         (uintptr_t)BNO055_DRIVER::init);
  // 飛行制御は TELEMETRY より先に走らせ、on_state/read_control の export を
  // 済ませてから TELEMETRY の最初の call_method を受けられるようにする。
  // ※ 初回実行順は create の順ではなく「スレッド番号の昇順」(YIELD が昇順スキャン)
  //    なので、FLIGHT_CONTROLLER に小さい番号を割り当てることが本質。
  obj_api::create_object((uint32_t)object_ids::FLIGHT_CONTROLLER, 15,
                         (uintptr_t)FLIGHT_CONTROLLER::main);
  obj_api::create_object((uint32_t)object_ids::TELEMETRY_SENDER, 16,
                         (uintptr_t)TELEMETRY_SENDER::main);

  // BLE スループット/RSSI ベンチマーク用の HELLO_WORLD を使いたい場合は、上の
  // TELEMETRY_SENDER の生成をコメントアウトし、下を有効化する (BLE 送信路の
  // 取り合いになるため同時起動はしない)。
  // obj_api::create_object((uint32_t)object_ids::HELLO_WORLD, 12,
  //                        (uintptr_t)HELLO_WORLD::main);

  // IO_CONTROLLER 自身は以降ハートビートを刻むだけ (BLE 送信は TELEMETRY が担う)。
  while (true) {
    obj_api::yield_us(1000000);
  }
}
