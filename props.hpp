#ifndef FLIGHT_ROBOCON_PROPS_HPP
#define FLIGHT_ROBOCON_PROPS_HPP
#include <cstdint>

// ===========================================================================
//  props — オブジェクトの「プロパティ」を flash FS の上に置く薄い層
// ===========================================================================
//  ★実体は Shizuku の flashfs オブジェクト。ここがやるのは 2 つだけ:
//    (1) メソッド呼び出し (LOOKUP / STORE) の定型を隠す
//    (2) **版と大きさを検査する** — 目録に載っている名前が、こちらの想像した
//        構造体と同じ形だとは限らない (ファーム更新で struct が変わる)。
//        検査を各オブジェクトに書かせると必ずどこかで抜ける。
//
//  ★読みは**写さない**。flashfs の LOOKUP は XIP アドレスを返すので、そこを
//    そのまま構造体として読める。load() が呼び出し側のバッファへ写すのは
//    「呼び出し側が後で書き換えたい」からで、媒体の都合ではない。
//
//  ★★書きは**周期スレッドから呼ばないこと**。1 セクタの消去に約 33ms かかり、
//    その間 XIP が止まる = **系全体が止まる**。設定を保存してよいのはシェルや
//    OTA のような、止まってよい経路だけ。この層はそれを強制できないので、
//    ここに書いておく以上のことはできない。
namespace xno::props {

// 名前の長さは媒体側 (FLASH_NAME_BYTES = 24) が決める。'\0' 込みで収まること。
constexpr uint32_t MAX_NAME_BYTES = 24;

// 全プロパティに共通の頭。★形が変わったら version を上げる。上げ忘れると
//   古い版を新しい構造体として読むので、ここだけは省略できない。
struct header {
  uint32_t magic;   // オブジェクトごとの識別子
  uint16_t version; // 構造体の版
  uint16_t bytes;   // ヘッダを含む全体の大きさ
};

// 引く。見つかって magic/version/bytes が一致したときだけ out へ写して true。
// ★見つからない・古い・大きさが違う、を区別しない — 呼ぶ側にできることは
//   どれも同じ (既定値で始める) なので、区別させると分岐だけが増える。
bool load(const char *name, uint32_t magic, uint16_t version, void *out,
          uint32_t bytes);

// 置く。ヘッダは呼ぶ側が埋めておくこと (magic/version は型の性質なので、
// ここで勝手に決めると型と値の対応が 2 箇所に分かれる)。
// ★同じ名前へ置き直すと**新しい場所**へ置かれ、古い領域は戻らない (flashfs の
//   性質: 配ったアドレスは動かせないので詰め直さない)。設定の保存を高頻度で
//   呼ぶと領域を食い潰す。
bool store(const char *name, const void *data, uint32_t bytes);

// 消す (名前を空けるだけ。領域は FORMAT まで戻らない)。
bool remove(const char *name);

// flashfs が使える状態か (register_flash_fs() が成功しているか)。
bool available();

} // namespace xno::props
#endif // FLIGHT_ROBOCON_PROPS_HPP
