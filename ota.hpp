#ifndef FLIGHT_ROBOCON_OTA_HPP
#define FLIGHT_ROBOCON_OTA_HPP
#include <cstdint>
#include "object_ids.hpp"
#include "tx_frame.hpp"

// ===========================================================================
//  ota — 新しいファームウェアを BLE で受け取り、ステージング領域へ置く
// ===========================================================================
//  ★段は 2 つある。**受け取って置く**のと、**本体へ移す (commit)** の。
//    片方だけで完結させない: 受信中に落ちても本体が無傷なのは、この 2 段に
//    分けてあるからで、commit は「全部揃って CRC が合った」あとにしか走らない。
//
//    段 1 (upload): 受け取って、ステージングへ置いて、検証する。
//    段 2 (commit): flash から読み直して再検証 → 本体領域を消して書く → 再起動。
//
//  ★commit の**コピー先はリンカに聞く** (`__flash_binary_start`)。
//    「0 番地決め打ち」にしないのは、パーティションを切った像でも、
//    ブートローダの下に置いた像でも、自分がいる場所がそのまま答えになるから。
//    ボードごとに違うのは A/B パーティションを使うかどうかで
//    ([[ota-strategy-is-board-specific]])、ここは**使わない**方 —— つまり
//    「自分がいる所を上書きする」だけなので RP2040 でも同じコードで通る。
//
//  ★安全性の考え方 (ユーザー判断 2026-08-24):
//    A/B パーティションは「転送が失敗しても戻れる」保険だが、その保険料
//    (パーティション運用の複雑さ) が高いので採らない。代わりに
//    **全部受け取って CRC が一致するまで本体には一切触れない**。
//    通信が途中で切れただけなら本体は無傷でやり直せる。本当に危ないのは
//    「commit のコピー中に電源が落ちる」瞬間だけで、そこは BOOTSEL で
//    焼き直す、と割り切る。
//
//  ★プロトコル (ホスト → デバイス、OTA 専用 characteristic 上の生バイト列):
//      [0] 'X' 'N' 'O' 'U'      マジック
//      [4] uint32 le  total     イメージ長 [byte]
//      [8] uint32 le  crc32     イメージ全体の CRC32 (IEEE, 反転あり)
//      [12..] 生データ (順番どおり。欠落・並べ替えは想定しない —
//             BLE の write は順序保証があり、落ちたら CRC で落ちる)
//    受信の進捗と結果は自分の行ストリームへ出す (logger 経由で BLE へ)。
//
//  ★commit コマンド (同じ characteristic 上、upload と同じ 12 バイト長):
//      [0] 'X' 'N' 'O' 'C'      マジック (4 文字目だけが upload と違う)
//      [4] uint32 le  total     ステージ済みイメージ長 [byte]
//      [8] uint32 le  crc32     そのイメージの CRC32
//    total/crc をホストから改めて貰うのは、**転送と commit を別の接続で
//    やれる**ようにするため (受信側の RAM 上の状態に依存しない)。
//    受け取ったら flash から読み直して CRC を確かめ、一致したときだけ焼く。
namespace ota {

constexpr uintptr_t OBJECT = xno_object_id::ota;

// ステージング領域。★アプリ本体 (~458KB) とも flash_fs の予定地 (末尾 1MB)
//   とも bonding bank (末尾 12KB) とも重ならない位置を選んである。
constexpr uint32_t STAGING_OFFSET = 0x180000; // flash 先頭からのオフセット
constexpr uint32_t STAGING_BYTES = 512 * 1024;

enum struct method : uintptr_t {
  MAIN = 0,
  // a0 = ble_uart の OTA 受信ストリーム番号。
  SET_INPUT_STREAM = 1,
  // 戻り値 = 進捗・結果を出す行ストリームの番号 (logger が購読する)。
  GET_STREAM = 2,
  POLL = 3,
};

using frame_t = xno::tx_frame;

uint32_t register_ota();
uint32_t start_ota();

} // namespace ota
#endif // FLIGHT_ROBOCON_OTA_HPP
