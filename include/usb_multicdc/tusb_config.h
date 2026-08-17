#ifndef SHIZU_TUSB_CONFIG_H
#define SHIZU_TUSB_CONFIG_H

// ===========================================================================
//  tinyusb 設定 — SHIZUKU_USB が USB デバイススタックを所有するときの構成
// ===========================================================================
//  pico_stdio_usb 付属の tusb_config.h は `#define CFG_TUD_CDC (1)` を
//  **#ifndef ガード無し**で持っているため、外から本数を増やせない。ただしその
//  ブロック全体が `#if !defined(LIB_TINYUSB_DEVICE)` で囲まれているので、
//  tinyusb_device を直接リンクすれば SDK 側の設定ごと無効化され、この
//  ファイルが使われる (= 逃げ道は SDK が用意している)。
//
//  ★VID/PID は変えないこと。picotool が reset-via-vendor でデバイスを探すのに
//    使っており、変えると `picotool load -f` が壊れて BOOTSEL 常用になる。
//    reset ベンダインタフェース自体も必ず載せる (SDK の reset_interface.c を
//    そのままビルドに加える)。
// ===========================================================================

#include "pico.h"

#define CFG_TUSB_MCU              OPT_MCU_RP2040 // RP2350 も同じ dcd を使う
#define CFG_TUSB_OS               OPT_OS_PICO
#define CFG_TUSB_RHPORT0_MODE     OPT_MODE_DEVICE

// CDC 本数。CDC-ACM 1 本 = エンドポイント番号 2 つ消費、RP2350 は 1..15 が使えるので
// 理論上限は 7 本。DPRAM (4KB) は 1 本 192B なので律速にならない。
// 既定 1 本: CDC0 = コンソール (生 printf / SHIZUKU_USB のリング排出とも同じ 1 本)。
// 増やすときは cmake -DSHIZU_USB_MULTI_CDC=ON -DSHIZU_USB_CDC_COUNT=N。
#ifndef SHIZU_USB_CDC_COUNT
#define SHIZU_USB_CDC_COUNT 1
#endif
#if SHIZU_USB_CDC_COUNT > 7
#error "CDC は最大 7 本 (エンドポイント番号 1..15 を 1 本あたり 2 つ消費)"
#endif

#define CFG_TUD_CDC               SHIZU_USB_CDC_COUNT
#define CFG_TUD_CDC_RX_BUFSIZE    256
#define CFG_TUD_CDC_TX_BUFSIZE    256
#define CFG_TUD_CDC_EP_BUFSIZE    64

#define CFG_TUD_MSC               0
#define CFG_TUD_HID               0
#define CFG_TUD_MIDI              0

// picotool の reset-via-vendor 用。**外すとフラッシュ手順が壊れる**。
#define CFG_TUD_VENDOR            0
#define PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE 1
#define PICO_STDIO_USB_RESET_INTERFACE_SUPPORT_RESET_TO_BOOTSEL 1

#define CFG_TUD_ENDPOINT0_SIZE    64

#endif // SHIZU_TUSB_CONFIG_H
