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

# ---- 一時対応: Pico 2 (無印, CYW43 なし) でのカーネル検証ビルド ---------------
# pico2_w が壊れている間の代替。RP2350 チップは同一なので PSPLIM / MPU W^X /
# DMA ポンプ / affinity 移動の検証は等価にできる (BLE 経路だけ検証不能)。
# 使い方 (通常の build/ とは別ディレクトリに):
#   cmake -S . -B build_pico2 -DSHIZU_PICO2_TEST=1
#   cmake --build build_pico2        # → build_pico2/main.uf2
# BLE (cyw43/btstack/BLE_UART_DRIVER) はビルドから外れ、SHIZU_NO_BLE=1 が定義される。
option(SHIZU_PICO2_TEST "Plain Pico 2 (no CYW43/BLE) temporary test build" OFF)
if(SHIZU_PICO2_TEST)
    set(PICO_BOARD pico2)
endif()

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
project(shizupico C CXX ASM)
# 診断用: CYW43 IOCTL タイムアウト (既定 1s = cyw43_configport.h) を上書きして、
# BLE 接続時の「1s LONG poll」がこの定数に追従するか確認する。pico_sdk_init より前に
# add_compile_definitions すると SDK ライブラリ (pico_cyw43_driver) にも -D が効く。
# cmake -DSHIZU_IOCTL_TIMEOUT_US=200000 で 200ms 等に。
if(DEFINED SHIZU_IOCTL_TIMEOUT_US)
    add_compile_definitions(CYW43_IOCTL_TIMEOUT_US=${SHIZU_IOCTL_TIMEOUT_US})
endif()
# 診断用: stdio (USB CDC) の詰まりを短縮する。printf は
#  ・pico_stdio の print_mutex を PICO_STDIO_DEADLOCK_TIMEOUT_MS (既定 1000ms) まで待ち
#  ・pico_stdio_usb の out_chars が CDC バッファ満杯で PICO_STDIO_USB_STDOUT_TIMEOUT_US
#    (既定 500ms) まで tud_task() をビジー回し (yield しない)
# ので、接続時の printf バーストが「cyw43 と無関係な秒級ストール」を作り得る。
# cmake -DSHIZU_STDIO_NOBLOCK=ON で両方を実質ゼロにして切り分ける。
if(SHIZU_STDIO_NOBLOCK)
    add_compile_definitions(PICO_STDIO_DEADLOCK_TIMEOUT_MS=2)
    add_compile_definitions(PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000)
endif()
# USB のデバイス名を固有にする (ホスト側ツールがポートを名前で特定できるように)。
# pico_stdio_usb の descriptor は USBD_MANUFACTURER / USBD_PRODUCT を #ifndef ガードで
# 持っているので、pico_sdk_init より前に定義すれば SDK ライブラリ側にも効く。
# ★VID/PID (0x2E8A:0x0009) は変えないこと — picotool が reset-via-baud で
#   デバイスを探すときに使っており、変えると `picotool load -f` のフラッシュ手順が壊れる。
# ---- オーバークロック (cmake -DSHIZU_SYS_CLK_KHZ=300000) ---------------------
# ★フラッシュ (QMI) 側は **PICO_FLASH_SPI_CLKDIV では制御できない**。SDK 2.2 の
#   RP2350 既定ビルドには boot2 が入っておらず (main.elf に `.boot2` が無い)、
#   QMI の M0_TIMING はブート ROM が設定した固定値 (div=3 / rx=2) で走るため。
#   分周比が固定 = フラッシュ速度が clk_sys に比例して動く、という別種の
#   「固定分周の連鎖」になっている。制御は実行時 (flash_clock.cpp) で行う。
#   下の SHIZU_FLASH_TARGET_KHZ を参照。
# ★2026-08-17: 300MHz を**既定**にした (それまでは -DSHIZU_SYS_CLK_KHZ=300000 の明示指定)。
#   根拠: 実機で core0 util 6.9%→5.7%、evt=68 / panic 0 / 異常行 0 を反復確認。かつ
#   clk_sys に依存する固定分周 (フラッシュ QMI / CYW43 PIO) をすべて導出化したので、
#   「クロックを変えると黙って壊れる隠れた前提」が無くなった。
#   0 を渡すとクロックを一切触らない (SDK 既定の 150MHz で走る)。
set(SHIZU_SYS_CLK_KHZ 300000 CACHE STRING "System clock (kHz); 0 = leave SDK default (150MHz)")
if(SHIZU_SYS_CLK_KHZ GREATER 0)
    # ★CYW43 の PIO SPI 分周も clk_sys から導出する。CYW43_PIO_CLOCK_DIV_INT の既定は
    #   **2 固定** (clk_sys から動的に算出していない) ので、sys_clk を 2 倍にすると
    #   SPI も 2 倍速になりチップが応答しなくなる (実測 2026-08-17: BLE の evt が
    #   68 → 1 で停止。ble_poll は前進するので「動いている」と誤読しやすい)。
    #   フラッシュとまったく同じ「固定分周の連鎖」なので、同じ直し方をする:
    #   **目標 SPI クロックを宣言して分周比を計算する**。4 を直書きすると
    #   「300MHz のときだけ正しい値」が残り、次にクロックを変えた人が同じ罠を踏む。
    #   既知良は 150MHz/div2 と 300MHz/div4 = どちらも SPI 75MHz なので目標は 75MHz。
    set(SHIZU_CYW43_SPI_TARGET_KHZ 75000 CACHE STRING "CYW43 PIO SPI clock target (kHz)")
    math(EXPR SHIZU_CYW43_PIO_DIV
         "(${SHIZU_SYS_CLK_KHZ} + ${SHIZU_CYW43_SPI_TARGET_KHZ} - 1) / ${SHIZU_CYW43_SPI_TARGET_KHZ}")
    if(SHIZU_CYW43_PIO_DIV LESS 1)
        set(SHIZU_CYW43_PIO_DIV 1)
    endif()
    message(STATUS "CYW43 PIO SPI: clk_sys ${SHIZU_SYS_CLK_KHZ} kHz / div ${SHIZU_CYW43_PIO_DIV} (target ${SHIZU_CYW43_SPI_TARGET_KHZ} kHz)")
    add_compile_definitions(CYW43_PIO_CLOCK_DIV_INT=${SHIZU_CYW43_PIO_DIV})
endif()
add_compile_definitions(USBD_MANUFACTURER="Shizuku")
add_compile_definitions(USBD_PRODUCT="Shizuku USB")
pico_sdk_init()
add_executable(main main.cpp kernel.cpp kernel_object.cpp svc_asm_handler.S obj_api.cpp core1_boot.cpp stream_selftest.cpp grant_selftest.cpp smp_stress.cpp busy_load.cpp rt_sched_test.cpp svc_delegate_test.cpp unpriv_probe.cpp)
target_sources(main PRIVATE IO_CONTROLLER.cpp WS2812_DRIVER.cpp HELLO_WORLD.cpp BME280_DRIVER.cpp BNO055_DRIVER.cpp TELEMETRY_SENDER.cpp FLIGHT_CONTROLLER.cpp core1_io.cpp SHIZUKU_USB.cpp log.cpp flash_clock.cpp)
target_link_libraries(main pico_stdlib pico_stdio pico_multicore hardware_exception hardware_irq hardware_pwm hardware_gpio hardware_pio hardware_i2c hardware_dma hardware_adc)
# ---- フラッシュ (XIP QSPI) の目標クロック -----------------------------------
# 分周比ではなく **目標周波数** を宣言する。分周比 = ceil(clk_sys / target) を実行時に
# 導出するので、clk_sys を変えてもこの値の意味は変わらない (flash_clock.cpp)。
#   150MHz sys → ceil(150/100)=2 → フラッシュ 75MHz
#   300MHz sys → ceil(300/100)=3 → フラッシュ 100MHz
# ★上げる前に必ず cmake -DSHIZU_XIP_BENCH=ON のスイープを回すこと。速すぎる設定は
#   落ちるのではなく**静かにデータを読み違える**。W25Q32 クラスの定格は 133MHz。
# ★300MHz が「偶然でなく最適点」である理由: フラッシュ目標 100MHz にちょうど
#   割り切れる clk_sys は 300 (÷3) と 400 (÷4) だけ。400MHz は 1.20V の天井の外
#   (実測 396MHz でハング)。324/348MHz へ上げると分周が 4 に跳んで**フラッシュが
#   81/87MHz へ下がる** = 上げ損。よって 300MHz / flash 100MHz が釣り合いの点。
set(SHIZU_FLASH_TARGET_KHZ 100000 CACHE STRING "XIP flash clock target (kHz)")
# ★RXDELAY も段数を直書きしない。レジスタ単位が clk_sys の**半サイクル**なので、
#   固定値は「そのクロックのときだけ正しい値」になる (フラッシュ分周・CYW43 分周と
#   同じ罠)。**遅延の物理量 [ps] を宣言して段数は実行時に導出**する:
#     300MHz → 半サイクル 1.67ns → 5000ps = 3 段 (窓 1..6 の中心 3.5 の内側)
#     150MHz → 半サイクル 3.33ns → 5000ps = 2 段 (窓 0..3 の中心 1.5 の内側)
#   ★段数固定 3 のままだと 150MHz では窓 0..3 の**上端**に座ることになっていた。
#   実測窓を ns に直すと上限はどのクロックでも ~11ns、下限だけがフラッシュ速度に
#   応じて上がる (100MHz で ≥1.7ns / 150MHz で ≥3.3ns)。5ns は全ケースで内側。
set(SHIZU_FLASH_RXDELAY_PS 5000 CACHE STRING "QMI read sample delay (ps)")
# ---- clk_peri (SPI / UART の元クロック) --------------------------------------
# ★`set_sys_clock_khz` は clk_peri を **PLL_USB (48MHz)** へ張り替える (SDK の既定選択。
#   PICO_CLOCK_ADJUST_PERI_CLOCK_WITH_SYS_CLOCK=1 だと clk_sys 直結になるが、300MHz は
#   RP2350 の定格 sys クロック 150MHz の 2 倍なので**使わない**)。
#   48MHz だと SPI の上限が clk_peri/2 = **24MHz** になる (spi_set_baudrate の prescale が
#   最小 2 のため)。UART は 48MHz でも 3Mbaud 出るので困らないが、SPI は足りない場合がある。
# ★RP2350 の clk_peri には**分周器がある** (CLOCKS_CLK_PERI_DIV_INT、2bit で 1..3、0=4)。
#   なので PLL_SYS から分周して「CPU 300MHz のまま clk_peri 150MHz」にできる。
#   150MHz = RP2350 の定格 sys クロックと同値なので周辺としても定格内。SPI 上限 75MHz。
#   分周比は clk_sys から導出する (ここも固定値を書かない)。
set(SHIZU_PERI_TARGET_KHZ 150000 CACHE STRING "clk_peri target (kHz); 0 = leave SDK default (48MHz)")
# コア電圧 [mV]。実測の天井は 1.20V=372MHz / 1.30V=432MHz なので 300MHz は 1.20V で
# 24% の余裕がある。上げても +16% の天井しか買えず、恒久的に消費電力と素子ストレスを
# 払うだけ。熱も律速していない (全負荷 120s で 40.2℃ 飽和) ので既定は 1200。
set(SHIZU_VREG_MV 1200 CACHE STRING "core voltage (mV): 1100/1150/1200/1250/1300")
math(EXPR SHIZU_VREG_ENUM "13 + (${SHIZU_VREG_MV} - 1200) / 50")  # VREG_VOLTAGE_1_20 = 13
target_compile_definitions(main PRIVATE
    SHIZU_FLASH_TARGET_KHZ=${SHIZU_FLASH_TARGET_KHZ}
    SHIZU_FLASH_RXDELAY_PS=${SHIZU_FLASH_RXDELAY_PS}
    SHIZU_VREG_MV=${SHIZU_VREG_MV}
    SHIZU_VREG_ENUM=${SHIZU_VREG_ENUM}
    SHIZU_PERI_TARGET_KHZ=${SHIZU_PERI_TARGET_KHZ})
message(STATUS "core voltage: ${SHIZU_VREG_MV} mV (vreg_voltage enum ${SHIZU_VREG_ENUM})")
# XIP (フラッシュ QSPI) 速度スイープ。cmake -DSHIZU_XIP_BENCH=ON。既定 OFF —
# 命令フェッチの的として 64KB ぶんのダミー関数を .text に埋めるので、常時入れない。
if(SHIZU_XIP_BENCH)
    target_sources(main PRIVATE xip_bench.cpp)
    target_link_libraries(main hardware_xip_cache hardware_timer)
    target_compile_definitions(main PRIVATE SHIZU_XIP_BENCH=1)
endif()
# フラッシュ操作の後に QMI タイミングが ROM 既定へ戻るかの実測。書き込みはしない
# (flash_do_cmd が flash_range_erase と同じ XIP 再入経路を通るため)。
# cmake -DSHIZU_FLASH_DRIFT_PROBE=ON。既定 OFF。
# CPU クロックの上限探索。cmake -DSHIZU_OC_PROBE=ON。既定 OFF。
# ★vreg を 1.30V まで上げて 300→600MHz を階段状に試す診断ビルド。常用しない。
#   固まってもウォッチドッグ + noinit フラグで自動復帰する (oc_probe.cpp 参照)。
if(SHIZU_OC_PROBE)
    target_sources(main PRIVATE oc_probe.cpp)
    target_link_libraries(main hardware_watchdog hardware_vreg)
    # 探索電圧 [mV]。1200 (出荷値) / 1250 / 1300 (SDK の無条件上限)。
    # ★出荷電圧での天井を必ず測ること。1.30V の天井は常用余裕の根拠にならない。
    set(SHIZU_OC_PROBE_MV 1300 CACHE STRING "vreg during OC probe (mV): 1200/1250/1300")
    target_compile_definitions(main PRIVATE SHIZU_OC_PROBE=1
        SHIZU_OC_PROBE_MV=${SHIZU_OC_PROBE_MV})
endif()
if(SHIZU_FLASH_DRIFT_PROBE)
    target_link_libraries(main hardware_flash pico_flash)
    target_compile_definitions(main PRIVATE SHIZU_FLASH_DRIFT_PROBE=1)
endif()
if(SHIZU_BUSY_LOAD)
    # CPU ホグ負荷試験を有効化 (kernel.hpp の既定 0 を上書き)。cmake -DSHIZU_BUSY_LOAD=ON。
    target_compile_definitions(main PRIVATE SHIZU_BUSY_LOAD=1)
endif()
# 非特権実行の最小プローブ。cmake -DSHIZU_UNPRIV_PROBE=ON
if(SHIZU_UNPRIV_PROBE)
    target_compile_definitions(main PRIVATE SHIZU_UNPRIV_PROBE=1)
endif()
if(SHIZU_SYS_CLK_KHZ GREATER 0)
    target_compile_definitions(main PRIVATE SHIZU_SYS_CLK_KHZ=${SHIZU_SYS_CLK_KHZ})
endif()
if(SHIZU_RT_SCHED_TEST)
    # RT スケジューラ検証を有効化。cmake -DSHIZU_RT_SCHED_TEST=ON。
    target_compile_definitions(main PRIVATE SHIZU_RT_SCHED_TEST=1)
endif()
if(SHIZU_BLE_POLL_INSTR)
    # BLE poll 計装を有効化 (cyw43_arch_poll 1 回の所要時間)。cmake -DSHIZU_BLE_POLL_INSTR=ON。
    target_compile_definitions(main PRIVATE SHIZU_BLE_POLL_INSTR=1)
endif()
if(SHIZU_BLE_NO_PHY_PROBE)
    # 診断: 2M PHY プローブを無効化。cmake -DSHIZU_BLE_NO_PHY_PROBE=ON。
    target_compile_definitions(main PRIVATE SHIZU_BLE_NO_PHY_PROBE=1)
endif()
if(SHIZU_CYW43_YIELD_WAIT)
    # Stage1: cyw43 の待ちを WFE→Shizuku yield へ。cmake -DSHIZU_CYW43_YIELD_WAIT=ON。
    target_compile_definitions(main PRIVATE SHIZU_CYW43_YIELD_WAIT=1)
endif()
# ★既定 ON。C++ 側 (SHIZUKU_USB のリング/オブジェクト) と下の 2 つの SDK クランプは
#   **必ず一組で**効かせる必要がある — 片方だけだと「リングはあるが print_mutex で
#   1000ms スピン (実測 1.015s)」または「出力が落ちる」の半端な状態になる。だから
#   include/kernel.hpp の #ifndef 既定は 0 のままにし、有効化はこの 1 箇所に集約する。
#   切り戻しは cmake -DSHIZU_USB_DRIVER=OFF。
option(SHIZU_USB_DRIVER "Non-blocking USB CDC stdio (SHIZUKU_USB object)" ON)
# svc 委譲 (番号範囲 → 担当オブジェクトが自分の syscall を捌く)。
# ★既定 ON (2026-08-17、実機検証済み)。委譲ハンドラに特権は与えない。
#   切り戻しは cmake -DSHIZU_SVC_DELEGATION=OFF。
option(SHIZU_SVC_DELEGATION "svc delegation (subsystem handles its own syscalls)" ON)
if(SHIZU_SVC_DELEGATION)
    target_compile_definitions(main PRIVATE SHIZU_SVC_DELEGATION=1)
    # 自己テスト (オブジェクト 3 個 + スレッド 3 本を常駐) は既定 OFF。
    #   cmake -DSHIZU_SVC_DELEGATE_TEST=ON            … 最小プローブ + 6 段ネスト
    #   cmake -DSHIZU_SVC_DELEGATE_TEST=ON -DSHIZU_SVC_DELEGATE_NEST=OFF … プローブのみ
    if(SHIZU_SVC_DELEGATE_TEST)
        target_compile_definitions(main PRIVATE SHIZU_SVC_DELEGATE_TEST=1)
        if(DEFINED SHIZU_SVC_DELEGATE_NEST AND NOT SHIZU_SVC_DELEGATE_NEST)
            target_compile_definitions(main PRIVATE SHIZU_SVC_DELEGATE_NEST=0)
        endif()
    endif()
endif()
if(SHIZU_USB_DRIVER)
    # printf を待たせない USB CDC 経路 (SHIZUKU_USB)。
    # 実測でこれが「BLE 接続時フリーズ」の本命の直し方 (SHIZUKU_USB.cpp 冒頭参照)。
    target_compile_definitions(main PRIVATE SHIZU_USB_DRIVER=1)
    # ★リングだけでは直らない。printf の待ちは 2 段あり、リングが潰せるのは内側だけ:
    #   (外) pico_stdio の print_mutex … mutex_enter_block_until = **WFE スピン**で
    #        PICO_STDIO_DEADLOCK_TIMEOUT_MS (既定 1000ms) までコアを握る。Shizuku の
    #        yield を通らないのでスケジューラからは手が出せない。実測の 1.0s/2.0s はこれ。
    #   (内) pico_stdio_usb の out_chars … CDC 満杯で 500ms ビジー回し ← リングで解決済み
    # printf は linker --wrap で pico_stdio に固定されており外側を差し替える口が無いので、
    # 外側は待ち時間そのものを詰めて潰す。**リングと組にして初めて安全**: リングのおかげで
    # mutex 保持は memcpy 数µs になり、2ms のタイムアウトはまず発火しない = 出力を落とさない。
    # (タイムアウト単独だと CDC 満杯のたびに出力が消える。SHIZU_STDIO_NOBLOCK がその形)
    add_compile_definitions(PICO_STDIO_DEADLOCK_TIMEOUT_MS=2)
    add_compile_definitions(PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000)
endif()
# ---- マルチ CDC: SHIZUKU_USB が USB デバイススタックを所有する ----------------
# pico_stdio_usb を外し、tinyusb_device を直接リンクして自前の tusb_config.h /
# descriptor を使う (SDK の tusb_config.h は CFG_TUD_CDC を #ifndef ガード無しで
# 1 に固定しているので、これ以外に本数を増やす手が無い)。
# ★reset_interface.c は必ず一緒にビルドすること — 落とすと picotool の
#   reset-via-vendor が死に、`picotool load -f` が使えなくなって BOOTSEL 常用になる。
# cmake -DSHIZU_USB_MULTI_CDC=ON [-DSHIZU_USB_CDC_COUNT=N] (N は最大 7)。既定 OFF。
option(SHIZU_USB_MULTI_CDC "SHIZUKU_USB owns the USB device stack (N x CDC)" OFF)
if(SHIZU_USB_MULTI_CDC)
    target_sources(main PRIVATE shizuku_usb_descriptors.c
        $ENV{PICO_SDK_PATH}/src/rp2_common/pico_stdio_usb/reset_interface.c)
    target_link_libraries(main tinyusb_device tinyusb_board pico_unique_id)
    # ★自前 tusb_config.h は **この構成でだけ** 見えるようにする (include/usb_multicdc)。
    #   include/ に置くと既定ビルド (pico_stdio_usb 経路) でも SDK 付属の
    #   tusb_config.h を先に食ってしまい、そちらが定義している reset インタフェース
    #   関連マクロが落ちて reset_interface.c がコンパイルエラーになる
    #   (PICO_STDIO_USB_RESET_BOOTSEL_INTERFACE_DISABLE_MASK undeclared)。
    #   既定ビルドは SDK の tusb_config.h = CDC 1 本のまま触らない。
    target_include_directories(main BEFORE PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/include/usb_multicdc)
    # pico_stdio_usb ライブラリはリンクしないが、reset インタフェースのヘッダだけ要る。
    target_include_directories(main PRIVATE
        $ENV{PICO_SDK_PATH}/src/rp2_common/pico_stdio_usb/include)
    target_compile_definitions(main PRIVATE SHIZU_USB_MULTI_CDC=1)
    if(DEFINED SHIZU_USB_CDC_COUNT)
        target_compile_definitions(main PRIVATE SHIZU_USB_CDC_COUNT=${SHIZU_USB_CDC_COUNT})
    endif()
endif()
if(SHIZU_CYW43_SVC_THREAD)
    # Stage2: 独自 host-wake IRQ + cyw43 サービススレッド (Stage1 を内包)。
    # cmake -DSHIZU_CYW43_SVC_THREAD=ON。巡回間隔は -DSHIZU_CYW43_SVC_NAP_US=<µs>。
    target_compile_definitions(main PRIVATE SHIZU_CYW43_SVC_THREAD=1)
    if(DEFINED SHIZU_CYW43_SVC_NAP_US)
        target_compile_definitions(main PRIVATE SHIZU_CYW43_SVC_NAP_US=${SHIZU_CYW43_SVC_NAP_US})
    endif()
endif()
if(SHIZU_PICO2_TEST)
    # BLE 無し: ドライバ本体を外し、コード側の #if ガード (IO_CONTROLLER / main の
    # ble_dbg スタブ) を有効化する。
    target_compile_definitions(main PRIVATE SHIZU_NO_BLE=1)
else()
    target_sources(main PRIVATE BLE_UART_DRIVER.cpp)
    target_link_libraries(main pico_btstack_classic pico_btstack_ble pico_cyw43_arch_poll pico_btstack_cyw43 pico_lwip_nosys)
    # ble_uart.gatt から ble_uart.h (profile_data / ハンドルマクロ) を生成する。
    pico_btstack_make_gatt_header(main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/ble_uart.gatt)
endif()
target_include_directories(main PRIVATE include)
pico_generate_pio_header(main ${CMAKE_CURRENT_SOURCE_DIR}/ws2812.pio)
pico_add_extra_outputs(main)
if(NOT SHIZU_USB_MULTI_CDC)
    pico_enable_stdio_usb(main 1)
endif()
# softfp: 呼び出し規約は soft のまま (pico-sdk/libm と ABI 互換)、FP 命令は有効。
# これで __ARM_FP が定義され、svc_asm_handler.S の S16-S31 退避ブロックが有効になる。
target_compile_options(main PUBLIC -mfloat-abi=softfp -Wall)
# panic() を stdio 経由の SDK 実装から shizu_panic (kernel.cpp) へ差し替える。
# ロックを取らず per-core の noinit RAM リングへ記録 → core1/例外文脈の panic でも
# メッセージが失われない (boot 時とビーコンが印字する)。include/panic_ring.hpp 参照。
target_compile_definitions(main PRIVATE PICO_PANIC_FUNCTION=shizu_panic)


