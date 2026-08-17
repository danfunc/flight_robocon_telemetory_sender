// ===========================================================================
//  shizuku_usb_descriptors — SHIZUKU_USB が所有する USB デバイス記述子
// ===========================================================================
//  pico_stdio_usb の記述子は CDC 1 本 + reset で固定 (static const 配列) なので、
//  複数 CDC を出すには自前で持つしかない。ここは SDK の stdio_usb_descriptors.c を
//  下敷きに、CDC を SHIZU_USB_CDC_COUNT 本並べた版。
//
//  インタフェース割り当て:
//    CDC n : インタフェース 2n (通信) / 2n+1 (データ)   ← CDC は 2 本使う
//    reset : 最後のインタフェース (ベンダ固有)
//  エンドポイント割り当て (CDC 1 本につき番号 2 つ):
//    CDC n : 0x81+2n (notif IN) / 0x02+2n (bulk OUT) / 0x82+2n (bulk IN)
//
//  ★VID/PID は Raspberry Pi の既定のまま。picotool が reset-via-vendor で
//    デバイスを探すのに使うので、変えると `picotool load -f` が壊れる。
//  ★bcdUSB は 0x0200 固定 (MS OS 2.0 記述子を出さないため。0x0210 にすると
//    Windows がデバイスを認識しなくなる — SDK 側コメントの警告どおり)。
// ===========================================================================
#include "tusb.h"
#include "pico/unique_id.h"
#include "pico/stdio_usb/reset_interface.h"
#include <string.h>

#ifndef USBD_VID
#define USBD_VID (0x2E8A) // Raspberry Pi
#endif
#ifndef USBD_PID
#define USBD_PID (0x0009) // Raspberry Pi Pico SDK CDC
#endif
#ifndef USBD_MANUFACTURER
#define USBD_MANUFACTURER "Shizuku"
#endif
#ifndef USBD_PRODUCT
#define USBD_PRODUCT "Shizuku USB"
#endif

#define CDC_N                 CFG_TUD_CDC
#define USBD_ITF_RPI_RESET    (CDC_N * 2)
#define USBD_ITF_MAX          (CDC_N * 2 + 1)

#define TUD_RPI_RESET_DESC_LEN 9
#define USBD_DESC_LEN (TUD_CONFIG_DESC_LEN + CDC_N * TUD_CDC_DESC_LEN + \
                       TUD_RPI_RESET_DESC_LEN)

#define USBD_MAX_POWER_MA (250)

// 文字列インデックス: 0=lang, 1=manuf, 2=product, 3=serial, 4..=CDC 名, 最後=reset
#define USBD_STR_MANUF     (0x01)
#define USBD_STR_PRODUCT   (0x02)
#define USBD_STR_SERIAL    (0x03)
#define USBD_STR_CDC0      (0x04)
#define USBD_STR_RPI_RESET (USBD_STR_CDC0 + CDC_N)

static const tusb_desc_device_t usbd_desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USBD_VID,
    .idProduct = USBD_PID,
    .bcdDevice = 0x0100,
    .iManufacturer = USBD_STR_MANUF,
    .iProduct = USBD_STR_PRODUCT,
    .iSerialNumber = USBD_STR_SERIAL,
    .bNumConfigurations = 1,
};

#define TUD_RPI_RESET_DESCRIPTOR(_itfnum, _stridx)                             \
  9, TUSB_DESC_INTERFACE, _itfnum, 0, 0, TUSB_CLASS_VENDOR_SPECIFIC,           \
      RESET_INTERFACE_SUBCLASS, RESET_INTERFACE_PROTOCOL, _stridx,

// CDC 1 本ぶんの記述子。n = インスタンス番号。
#define CDC_DESC(n)                                                            \
  TUD_CDC_DESCRIPTOR(2 * (n), USBD_STR_CDC0 + (n), 0x81 + 2 * (n), 8,          \
                     0x02 + 2 * (n), 0x82 + 2 * (n), 64)

static const uint8_t usbd_desc_cfg[USBD_DESC_LEN] = {
    TUD_CONFIG_DESCRIPTOR(1, USBD_ITF_MAX, 0, USBD_DESC_LEN, 0,
                          USBD_MAX_POWER_MA),
    CDC_DESC(0),
#if CFG_TUD_CDC > 1
    CDC_DESC(1),
#endif
#if CFG_TUD_CDC > 2
    CDC_DESC(2),
#endif
#if CFG_TUD_CDC > 3
    CDC_DESC(3),
#endif
#if CFG_TUD_CDC > 4
    CDC_DESC(4),
#endif
#if CFG_TUD_CDC > 5
    CDC_DESC(5),
#endif
#if CFG_TUD_CDC > 6
    CDC_DESC(6),
#endif
    TUD_RPI_RESET_DESCRIPTOR(USBD_ITF_RPI_RESET, USBD_STR_RPI_RESET)};

static char usbd_serial_str[PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2 + 1];

static const char *const usbd_desc_str[] = {
    [USBD_STR_MANUF] = USBD_MANUFACTURER,
    [USBD_STR_PRODUCT] = USBD_PRODUCT,
    [USBD_STR_SERIAL] = usbd_serial_str,
    [USBD_STR_CDC0 + 0] = "Shizuku console",  // CDC0 = 生 printf
#if CFG_TUD_CDC > 1
    [USBD_STR_CDC0 + 1] = "Shizuku channel",  // CDC1 = Shizuku 用
#endif
#if CFG_TUD_CDC > 2
    [USBD_STR_CDC0 + 2] = "Shizuku channel 2",
#endif
#if CFG_TUD_CDC > 3
    [USBD_STR_CDC0 + 3] = "Shizuku channel 3",
#endif
#if CFG_TUD_CDC > 4
    [USBD_STR_CDC0 + 4] = "Shizuku channel 4",
#endif
#if CFG_TUD_CDC > 5
    [USBD_STR_CDC0 + 5] = "Shizuku channel 5",
#endif
#if CFG_TUD_CDC > 6
    [USBD_STR_CDC0 + 6] = "Shizuku channel 6",
#endif
    [USBD_STR_RPI_RESET] = "Reset",
};

const uint8_t *tud_descriptor_device_cb(void) {
  return (const uint8_t *)&usbd_desc_device;
}

const uint8_t *tud_descriptor_configuration_cb(uint8_t index) {
  (void)index;
  return usbd_desc_cfg;
}

const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
  (void)langid;
#define DESC_STR_MAX 32
  static uint16_t desc_str[DESC_STR_MAX];

  if (!usbd_serial_str[0])
    pico_get_unique_board_id_string(usbd_serial_str, sizeof(usbd_serial_str));

  uint8_t len;
  if (index == 0) {
    desc_str[1] = 0x0409; // English
    len = 1;
  } else {
    if (index >= sizeof(usbd_desc_str) / sizeof(usbd_desc_str[0]))
      return NULL;
    const char *str = usbd_desc_str[index];
    if (!str)
      return NULL;
    for (len = 0; len < DESC_STR_MAX - 1 && str[len]; ++len)
      desc_str[1 + len] = str[len];
  }
  desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * len + 2));
  return desc_str;
}
