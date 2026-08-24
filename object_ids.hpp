#ifndef FLIGHT_ROBOCON_OBJECT_IDS_HPP
#define FLIGHT_ROBOCON_OBJECT_IDS_HPP
#include "shizuku/object_ids.hpp"

// ===========================================================================
//  このリポジトリ (Bazel/Shizuku 側) が新設するオブジェクトの番号
// ===========================================================================
//  ★D28 (番号を振る規則は 1 つに保つ) の踏襲: Shizuku 自身の objects.list には
//    加えない(別リポジトリの genrule を二重に持つ危険は
//    ancient-cuddling-adleman.md Phase A §3 で却下済み)。代わりに Shizuku が
//    使い切った次の空き番号 (shizuku::object_id::count) を土台にして、この
//    ファイル 1 箇所だけで連番を割り当てる。新しいオブジェクトを足すときは
//    必ずここに追記すること (各 .hpp が勝手に `object_id::count` を直接使うと
//    2 つ目以降で衝突する)。
namespace xno_object_id {

constexpr uintptr_t base = shizuku::object_id::count;

constexpr uintptr_t ble_uart = base + 0;
constexpr uintptr_t bno055 = base + 1;
constexpr uintptr_t bme280 = base + 2;
// センサ群の合流点 (D46 のハブ) 兼コマンド解釈。詳細は flight_controller.hpp。
constexpr uintptr_t flight_controller = base + 3;
// ★GDB で「止まったことが目で見える」ための対象 (firmware_bazel/main.cpp)。
//   LED を一定周期で叩くだけで、何の仕事も持たない — 止めても系に影響が無い
//   相手を対象にしておくのが肝 (本業のスレッドを止めると、止めている間の
//   計測や通信がそのまま乱れる)。
constexpr uintptr_t blink = base + 4;
// ★役をひとつずつに割る (UNIX 哲学。ストリームがパイプの役をする)。
//   telemetry: サンプル → テレメトリ行 (符号化だけ)
//   logger   : 行の合流点。複数の行ソースを 1 本へ束ねる (syslogd と同じ役)。
//              ★flash へのログ (Phase E) はここに tee を足すだけで済む —
//                「出ていく行が全部通る 1 箇所」だから
constexpr uintptr_t telemetry = base + 5;
constexpr uintptr_t logger = base + 6;
// ファームウェアを BLE で受け取ってステージングする (ota.hpp 参照)。
constexpr uintptr_t ota = base + 7;

} // namespace xno_object_id
#endif // FLIGHT_ROBOCON_OBJECT_IDS_HPP
