# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work =
set(PICO_BOARD pico2_w)
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.20)
set(CMAKE_CXX_STANDARD 20)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
project(shizupico C CXX ASM)
pico_sdk_init()
add_executable(main main.cpp kernel.cpp kernel_object.cpp svc_asm_handler.S obj_api.cpp core1_boot.cpp stream_selftest.cpp)
target_sources(main PRIVATE IO_CONTROLLER.cpp WS2812_DRIVER.cpp BLE_UART_DRIVER.cpp HELLO_WORLD.cpp BME280_DRIVER.cpp BNO055_DRIVER.cpp TELEMETRY_SENDER.cpp FLIGHT_CONTROLLER.cpp core1_io.cpp)
target_link_libraries(main pico_stdlib pico_stdio pico_multicore hardware_exception hardware_irq hardware_pwm hardware_gpio hardware_pio hardware_i2c pico_btstack_classic pico_btstack_ble pico_cyw43_arch_poll pico_btstack_cyw43 pico_lwip_nosys)
target_include_directories(main PRIVATE include)
pico_generate_pio_header(main ${CMAKE_CURRENT_SOURCE_DIR}/ws2812.pio)
# ble_uart.gatt から ble_uart.h (profile_data / ハンドルマクロ) を生成する。
pico_btstack_make_gatt_header(main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/ble_uart.gatt)
pico_add_extra_outputs(main)
pico_enable_stdio_usb(main 1)
# softfp: 呼び出し規約は soft のまま (pico-sdk/libm と ABI 互換)、FP 命令は有効。
# これで __ARM_FP が定義され、svc_asm_handler.S の S16-S31 退避ブロックが有効になる。
target_compile_options(main PUBLIC -mfloat-abi=softfp -Wall)


