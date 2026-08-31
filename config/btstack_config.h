#ifndef BTSTACK_CONFIG_H
#define BTSTACK_CONFIG_H

// BLE UART (Nordic UART Service)
#define ENABLE_PRINTF_HEXDUMP
#define ENABLE_LE_PERIPHERAL
#define ENABLE_LE_DATA_LENGTH_EXTENSION

// ★ 安全対策: LE Secure Connections + Numeric Comparison (NC) を有効化
//   ペアリング時に CDC での 'y' 承認を必須とし、不正接続を防止する。
#define ENABLE_LE_SECURE_CONNECTIONS
#define MAX_NR_GATT_CLIENTS 0
// ★枠が 1 つしかないことの意味: **新しいペアリングが成立すると、既存の鍵が
//   押し出されて消える**。つまり第三者にペアリングを通されると、機体を
//   奪われるだけでなく正規の母艦が締め出されて BLE OTA (唯一の遠隔書き込み
//   手段) も失われる。これがペアリング施錠 (ble_uart.cpp の
//   `pairing_locked_now` / `sm_set_accepted_stk_generation_methods(0)`) を
//   入れた直接の理由。枠を増やしても「増えた枠が埋まったら同じこと」なので、
//   枠数ではなく**受け入れるかどうか**の側で塞いでいる。
#define MAX_NR_LE_DEVICE_DB_ENTRIES 1
#define NVM_NUM_DEVICE_DB_ENTRIES 1

// ★ CYW43 の実用上限 (247+4=251B)
#define HCI_ACL_PAYLOAD_SIZE (247 + 4)
#define HCI_OUTGOING_PRE_BUFFER_SIZE 64
#define HCI_ACL_CHUNK_SIZE_ALIGNMENT 64

#define MAX_NR_HCI_CONNECTIONS 1
#define MAX_NR_L2CAP_SERVICES 2
#define MAX_NR_L2CAP_CHANNELS 2

#define HAVE_MALLOC
#define HAVE_ASSERT
#define HAVE_EMBEDDED_TIME_MS

#endif // BTSTACK_CONFIG_H
