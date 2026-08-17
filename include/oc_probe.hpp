#pragma once
// CPU クロックの上限探索 (cmake -DSHIZU_OC_PROBE=ON、既定 OFF)。
// 1 回のブートで 300→600MHz を順に試し、各段でフラッシュ読み・演算・ダイ温度を検証して
// 印字する。固まってもウォッチドッグ + noinit フラグで自動復帰し、次のブートは
// 登り直さないので BOOTSEL 押しが要らない。詳細は oc_probe.cpp 冒頭。
namespace shizu {
void oc_probe_run();
}
