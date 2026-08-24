# OTA (BLE でファーム転送) — 現状と次の一手

最終更新: 2026-08-24 / 実機: pico2_w (RP2350)

## いまできること

**BLE だけでファームウェアを送り、ステージングへ置いて検証する** ところまでは
**実機で完走**している。

```
transfer done: 396644 bytes in 50.8s (7.6 kB/s)
device: done: 396632 bytes crc=e406c5a0 OK (staged at 0x180000)
RESULT: staged OK
```

**commit (ステージング → 本体へコピーして再起動) も 2026-08-24 に実装し、
実機で 2 回通した。** USB を一切挿さずに (AC アダプタ給電のまま) 焼き替わる。

```
device: done: 397552 bytes crc=26797164 OK (staged at 0x180000)
RESULT: staged OK
device: commit: 397552 bytes -> 0x0 (98 sectors), no return
RESULT: committed and rebooted (found again after 1 scan(s))
```

★焼き替わった証拠は「再起動した」ではなく **新しい像の中身が動いていること**
で取った。GDB を BLE 経由で attach し、この版で入れた blink の掃引変数を
実機のレジスタから読んで 1400 → 1300 → 1200 と動いているのを見ている
(「GDB を BLE で使う」の節)。

## 使い方

```sh
# ワンライナー (これで足りる)。転送 約 50 秒 + commit 約 7 秒。
bazel build //firmware_bazel:xno_bringup && python3 tools/ota_send.py --commit

# 明示的に渡す場合。**.elf / .uf2 / .bin のどれでもよい** (中身で見分けて
# 生イメージに直してから送る)。省略すると bazel の既定の出力を使う。
python3 tools/ota_send.py bazel-bin/firmware_bazel/xno_bringup --commit
python3 tools/ota_send.py build/main.uf2 --commit

python3 tools/ota_send.py <image>              # 送るだけ (ステージング)
python3 tools/ota_send.py <image> --commit-only  # 既に置いてある像を移す
```

VS Code からは タスク **OTA Deploy (BLE)** か、ステータスバーの
**$(radio-tower) OTA** ボタン (`.vscode/` は `tools/gen_vscode.py` が生成)。

★`.uf2` 経由だけは末尾に最大 255 バイトの 0 詰めが付く (ブロックが 256B 固定の
ため)。中身は同じだが**長さと CRC は変わる**ので、`--commit-only` で後から
commit するときは転送時と同じ形式で渡すこと。

前提: BLE がペアリング済みであること。**認可されていないリンクからは受け取らない**
(fail-closed)。macOS 側で "Shizuku UART" を Forget した後は再ペアリングが要る。

★`--commit-only` が成り立つのは、デバイスが RAM 上の受信状態ではなく
**flash から読み直して** CRC を確かめるため。転送のあとで切れてしまっても、
置いた像はそこに残っているので、繋ぎ直して commit だけやればよい。

## 構成

| ファイル | 役割 |
|---|---|
| `ota.hpp` / `ota.cpp` | 段 1: 受信 → ステージング (`0x180000`, 512KB) → CRC32 検証 / 段 2: commit (再検証 → 本体を消して書く → 再起動) |
| `ble_uart.gatt` | OTA 専用 characteristic `6E402002-...` (write)。NUS とも GDB とも分ける |
| `ble_uart.cpp` | OTA write → ストリームへ流すだけ (中身は解釈しない) |
| `tools/ota_send.py` | ホスト側。.elf/.uf2/.bin を生イメージへ直す + ヘッダ + CRC32 + 進捗 + commit |
| `tools/gdb_ble_bridge.py` | GDB (TCP) ↔ BLE の橋。RSP は解釈せず運ぶだけ |
| `tools/gen_vscode.py` | `.vscode/` 生成。**launch.json/tasks.json は手編集しない** |

### プロトコル
```
[0]  'X','N','O','U'      マジック
[4]  uint32 le  total     イメージ長
[8]  uint32 le  crc32     イメージ全体の CRC32 (zlib と同じ)
[12..] 生データ (順番どおり)
```
commit コマンドは同じ characteristic 上の 12 バイト (4 文字目だけ違う):
```
[0]  'X','N','O','C'      マジック
[4]  uint32 le  total     ステージ済みイメージ長
[8]  uint32 le  crc32     その CRC32
```
進捗・結果はデバイスが NUS の notify に行で出す (logger 経由)。

### flash レイアウト
| 領域 | 範囲 |
|---|---|
| アプリ本体 | `0x000000` 〜 約 `0x070000` (387KB) |
| **ステージング** | `0x180000` 〜 `0x200000` (512KB) |
| flash_fs (将来) | 末尾 1MB |
| bonding bank | `0x3FD000` 〜 `0x400000` (SDK 既定) |

## 踏んだ罠 (再発させない)

1. **`flash_safe_execute` は「自分ではない方のコア」が lockout victim 登録済みを要求する**
   (`pico_flash/flash.c:184`)。Shizuku は `if (core != 0)` で core1 しか登録して
   おらず、**core1 で走る OTA スレッドから flash が書けなかった**
   (`[PANIC] core=1 thread=8`)。→ `board.cpp` で**両コア**が呼ぶよう修正済み。
   bonding 書き込みが動いていたのは、たまたま core0 の BLE スレッドだったから。

2. **`response=False` は流量制御が無い**。CoreBluetooth のキューに 396KB が
   一気に積まれ、デバイスの受信リングが即溢れて切断した (「0.0 秒で 396KB 送信」)。
   → `write_gatt_char(..., response=True)` にする。1 write = 1 往復 ≒ 1 CI (15ms)。

3. **ページ (256B) ごとに flash を叩くとリンクが保たない**。396KB で**約 1650 回**
   両コアを止めることになり、ロックの handshake と IRQ 禁止がその回数だけ BLE に
   割り込む。→ **1 セクタ (4KB) を 1 回のロックにまとめる** (消去 + 16 ページ書き込みを
   同じロック内で)。ロック回数 97 回、1 回の停止 約 66ms。
   ★**これ以上大きくまとめない**。64KB だと 1 回の停止が約 1 秒になり、今度は
   supervision timeout や write の応答待ちに当たる。目的は「止める回数を減らす」
   ことであって「1 回を長くする」ことではない。

## commit の作り (2026-08-24 実装)

**「自分がいる領域を消す」** のが難所。守っているのは 4 点:

1. **コピーのループも、そこから呼ぶものも全部 RAM に居る** (`commit_blast` は
   `__no_inline_not_in_flash_func`)。SDK の `flash_range_erase` /
   `flash_range_program` は元から RAM 常駐なので、そのまま呼べる。
   ★**`memcpy` は RAM 常駐ではない**。素の代入ループが memcpy 呼び出しへ
   畳まれると、その瞬間に flash へ飛んで死ぬ。だから `volatile` ポインタで
   書いて畳ませない。ROM の `reboot` も、その場で引くのではなく
   **呼ぶ前に引いた関数ポインタを渡す**。
   → 逆アセンブルで裏を取ること (下の「焼く前の確認」)。
2. 読み元 (ステージング) は XIP 経由で読む。erase/program は終わりに XIP を
   張り直すので、セクタごとに **「読む → 消す → 書く」** の順なら読みは必ず
   XIP が生きている瞬間に来る。**逆順にしない**。
3. **return しない**。戻り先 (`flash_safe_execute` の後半) はもう別の像なので、
   焼き終わったらその場で ROM の `reboot` を呼ぶ。
   → だから `flash_safe_execute` から戻ってきたら「ロックが取れなかった」の意味。
4. コピー先は **リンカに聞く** (`__flash_binary_start`)。0 番地決め打ちに
   しないのは、パーティションを切った像でも自分がいる場所がそのまま答えだから。

焼く前に弾くもの (どれも**本体は無傷**で返る):

| 弾くもの | 見かた |
|---|---|
| サイズ | `0 < total <= 512KB` |
| ステージングを自分で消す配置 | `dst + 消す範囲 <= 0x180000` |
| **.uf2 を送ってしまった** | 先頭が ベクタテーブルか (`[0]` が SRAM、`[1]` が XIP)。uf2 の先頭は `UF2\n` なので一発で落ちる |
| flash に書けていない | **flash から読み直して** CRC32 再計算。受信直後の CRC は「受け取ったバイト列」の検証であって、flash に本当にその通り書けたかは別の話 |

## commit を最初に動かすまで

**commit が入った像を、まず一度 USB で焼く必要がある** (済: 2026-08-24)。
commit を持たない像に `XNOC` を送っても無視される —— 4 文字目が `U` でないと
マジックとして拾わないので、**古い像に当てても何も起きない**
(ホスト側は「no commit acknowledgement」と出て、本体は無傷のまま)。

```sh
bazel build //firmware_bazel:xno_bringup
/opt/picotool/bin/picotool load -f -x bazel-bin/firmware_bazel/xno_bringup.uf2
```

2 回目以降は BLE だけで完結する。

### 焼く前の確認 (実機に触る前に必ず)

`commit_blast` から flash へ飛ぶ分岐が 1 つも無いことを、逆アセンブルで見る。
`bl`/`blx` の飛び先が全部 `0x2000xxxx` (RAM) か関数ポインタであること:

```sh
ELF=bazel-bin/firmware_bazel/xno_bringup
read -r ADDR SIZE _ < <(arm-none-eabi-nm -S "$ELF" | grep commit_blast)
echo "commit_blast @ 0x$ADDR ($((0x$SIZE)) bytes)"   # 0x2000xxxx なら RAM
arm-none-eabi-objdump -d --start-address=0x$ADDR \
  --stop-address=$((0x$ADDR + 0x$SIZE)) "$ELF"
```

2026-08-24 の確認: `commit_blast` = `0x20000110` (.data 経由で RAM)、
コピーは素の `ldr`/`str` ループ (memcpy 呼び出しなし)、
`bl 200005c0 <flash_range_erase>` / `bl 200006d0 <flash_range_program>` は
どちらも RAM 直行 (veneer 経由でない)、最後は `blx sl` (ROM reboot) → `wfi`。

### 残っているリスク (承知のうえで進める)

- **コピー中の電源断** = BOOTSEL で焼き直し。約 98 セクタ × 66ms ≒ **6.5 秒**
  のあいだ、本体領域は一貫していない。これは A/B を採らないと決めた時点での
  既定路線 (ota.hpp 冒頭)。
- **その 6.5 秒、core0 は `multicore_lockout` のハンドラで止まっている**。
  ステージング側 (1 回 66ms × 97 回) と機構は同じだが、**1 回が長い**。
  もし core0 が優先度の高い割り込みを取ると、飛び先の flash はもう別の像
  なので暴走しうる。flash を書いているのは core1 側なので焼き上がりには
  影響しないはずだが、未実測。
- BLE は commit 開始の時点で切れる。ホスト側は「`commit:` の行が出て、
  そのあと黙って切れる」を成功の合図にしている (`tools/ota_send.py`)。

## GDB を BLE で使う (2026-08-24 実装・実機で通した)

OTA と同じ 1 本の BLE リンクで、**プローブ無しでデバッガが刺さる**。
ホスト側の橋が無かったので `tools/gdb_ble_bridge.py` を書いた
(GATT の characteristic は前から確保してあった)。

```sh
python3 tools/gdb_ble_bridge.py            # 127.0.0.1:3333 で待つ
# 別の端末で
arm-none-eabi-gdb bazel-bin/firmware_bazel/xno_bringup \
  -ex 'directory bazel-flight_robocon_telemetory_sender' \
  -ex 'target remote :3333'
```

VS Code からは **F5 → "GDB over BLE (attach)"**。橋は preLaunchTask
(`GDB BLE bridge`) が起こす。

- **止まるのは対象のスレッドだけ**。SWD の halting debug と違い DebugMonitor
  なので、BLE もテレメトリも走り続ける —— **走り続けないと RSP が運べない**
  ので、これは都合ではなく前提。対象は `main.cpp` が stub に渡したスレッド
  (いまは blink)。
- ★`directory <execroot>` が要る。bazel の DWARF は execroot からの相対パス
  (`external/shizuku+/...`, `firmware_bazel/main.cpp`) なので、教えないと
  「No such file or directory」になる。execroot は
  `bazel-flight_robocon_telemetory_sender` シンボリックリンク。
- ★`target extended-remote` は使わない。stub が喋るのは素の remote だけ。
  launch.json は `overrideAttachCommands` で明示している。
- 認可されていないリンクからの RSP はデバイス側で捨てる (fail-closed)。
  GDB は任意のメモリ読み書きとレジスタ操作そのものなので、ペアリング必須。
- ★橋とテレメトリ/OTA は **同じ 1 接続を取り合う**。橋を上げている間は
  `ota_send.py` は繋がらない。先に橋を落とすこと。

実測 (2026-08-24, pico2_w, AC 給電・USB 未接続):

```
Breakpoint 1, (anonymous namespace)::blink_main () at firmware_bazel/main.cpp:46
interval_ms(lr)=1400  step(r4)=-100
interval_ms(lr)=1300  step(r4)=-100
interval_ms(lr)=1200  step(r4)=-100
```

ブレークポイント (FPB) も効く。上は OTA で入れた blink の掃引が実機で
動いていることを、LED を目で見るのではなく**レジスタの値で**確かめたもの。

## 既知の未解決

- 転送を中断すると、その後デバイスが advertise を再開しないことがある
  (USB CDC は生きている)。要調査。いったん再起動すれば戻る。
- スループット 7.6 kB/s は BLE の上限 (約 16 kB/s) の半分。`response=True` で
  1 往復ごとに CI を待つため。上げるなら「`response=False` + ホスト側で明示的に
  ペーシング + 受信リングを大きく」だが、流量制御を自前で持つことになる。
