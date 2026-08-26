# OTA (BLE でファーム転送) — 現状と次の一手

最終更新: 2026-08-24 / 実機: pico2_w (RP2350)

## いまできること

**BLE だけでファームウェアを送り、ステージングへ置いて検証する** ところまでは
**実機で完走**している。

```
  deflate: 269611 bytes on the wire (67.2%)
  device: ready
transfer done: 269611 bytes in 9.1s (29.0 kB/s)
  device: done: 401072 bytes crc=dc5d64e5 OK (staged at 0x180000)
  device: time: 10182ms = erase 726 (7 blk) + program 881 + inflate 1541 + link 7034
  device: commit: 401072 bytes -> 0x0 (7 blocks), no return
  [  7.3s] device is back and talking
RESULT: committed and rebooted (verified on attempt 2)
```

**速度 (2026-08-24 に作業。転送 51.3s → 9.1s、全体 78.7s → 19.9s)**

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

## 速度: どこに時間が行っているか (2026-08-24)

| | 前 | 後 |
|---|---|---|
| 転送 | 51.3s (7.6 kB/s) | **9.2s** (28.5 kB/s) |
| 全体 (焼き替え 1 回) | 78.7s | **20.1s** |

デバイス側の内訳 (staging 3 回の中央値): 消去 745ms (7 ブロック) +
書き込み 895ms + 展開 615ms + リンク 8050ms。**リンクが 8 割**。

効いた順に 4 つ。**測ってから直した** —— 最初に効くと思った所は 2 番目だった。

### 1. write with response をやめた (7.6 → 28.5 kB/s)

応答付きは 1 write = 1 往復 ≒ 2 CI (30ms) で、244B/30ms = 8 kB/s が上限。
非応答なら 1 つの接続イベントに複数パケットが載る。**ただし bleak は
response=False を CoreBluetooth へ投げっぱなしにする** (backends の
write_characteristic は非応答では何も待たない)。流量制御は
`peripheral.canSendWriteWithoutResponse()` を自分で見て作る。

★**28.5 kB/s が macOS 側の天井**。ゲートを外しても、オーバーシュートを
2/4/8/16 パケット許しても変わらないことを実測で確認した (64KB を送り分けて
比較)。ゲート無しで 4ms 間隔まで詰めると届かなくなる (= 空を撃っている)。
**上げる手はもう無い。減らす方に行くしかない。**
★2M PHY は CYW43 の BT ファームが非対応なので使えない (ユーザー確認済み)。

### 2. 消去を 4KB セクタから 64KB ブロックへ (5.3s → 0.7s)

消去は「バイトあたり」ではなく **「コマンドあたり」が重い**。4KB セクタ消去
(20h) は 1 回 55ms で、98 回やると 5.4 秒。64KB ブロック消去 (D8h) なら
7 回 0.7 秒。ROM の `flash_range_erase` は範囲がブロックに揃っていれば
自動で D8h を使うので、**呼び方を変えるだけ**で済む。
staging 側と commit 側の両方に効く。

### 3. deflate で送るバイトを 67% に (13.6s → 9.2s)

ARM の像は deflate で 67% に落ちる。展開はデバイス側 (`inflate.hpp`)。
★セクタ差分は**測って捨てた** —— 1 関数だけ変えた 2 つのビルドで
**98/98 セクタ全部が変わる** (コード配置がずれる) ので、意味が無い。

### 4. 固定待ちを消した

「判定を 5 秒待つ」「commit 後 20 秒待つ」をやめて、出た行と実際の再接続で
判断する。★**スキャンで見つかったことを成功の合図にしない** —— macOS は
一度見つけた相手を覚えていて、まだ焼いている最中でも即座に返す
(実測: commit から 0.9 秒で「戻った」と出たが、消去と書き込みだけで 1.7 秒
かかる)。繋いで喋らせるまで確かめる。

### 5. 展開をテーブル引きに (1.54s → 0.62s)

最初は puff.c と同じ「1 記号ずつビットを歩く」だけで書いたが、実機で
0.9〜1.5 秒かかった。deflate の符号は圧倒的に 9 ビット以下なので、
**9 ビットぶんの表 (512 語 = 1KB/表) を引けばほぼ 1 発で決まる**。
表に載らない長い符号だけ元の歩く道へ落ちるので、**正しさの根拠は変わらない**
(表はその手続きの答えを先に並べただけ)。

★引く鍵は**符号を反転したもの**。deflate はビットを LSB 先頭で読むのに
ハフマン符号は MSB 先頭で組まれている。ここを取り違えると「たまに合う」表に
なり、化け方が入力依存になって一番追いにくい。

★表を足したぶん作業領域が 3.3KB になったので、**スタックから .bss へ出した**
(`tiny_inflate::state` を呼び出し側が持つ)。スレッドのスタックに 3.3KB 積むと
静かに食い潰す。

### まだ残っている (やっていない)

- 起動に `sleep_ms(1000)` (USB CDC の列挙待ち) が入っていて、焼き替え 1 回
  ごとに丸ごと乗る。
- **リンクの 8.0s は 28.5 kB/s の天井そのもの**。これ以上は圧縮率でしか
  動かない。LZMA なら 54% (deflate は 67%) まで行けるので 1.5 秒ぶん
  縮むが、展開器は桁違いに重くなる。**割に合うかは未判断**。
- commit 後の復帰確認に 7 秒かかっている。うち大半はデバイスの起動と
  BLE が上がるまでで、こちらの待ち方 (5 秒スキャン + 1 秒リトライ) も
  保守的。

## ★踏んだ: 長い IRQ 停止が CYW43 を道連れにする (2026-08-24)

**転送しながら 64KB ブロック消去 (105ms) をやると CYW43 が壊れる。**

```
got unexpected packet 0
[CYW43] Bus error condition detected 0xb32f
[CYW43] do_ioctl(2, 263, 16): timeout
```

そのまま commit まで行ったら、**再起動後に完全に沈黙した** —— USB は列挙
されるのに CDC は 2 本とも無音、`picotool -f` の強制 BOOTSEL も効かない。
BOOTSEL からの焼き直しでしか戻らなかった。

理屈: `flash_safe_execute` は両コアを止めて IRQ を切る。その間 CYW43 を
相手にする SPI/PIO の面倒を誰も見られない。転送速度が 4 倍になって
パケット密度が上がった結果、停止が転送の途中に当たる確率も上がった。

**対策 (両方入れてある)**:

1. **消去は転送を始める前に済ませる**。ヘッダを受けた時点で必要な範囲を
   全部消し、`ready` を返す。ホストはその行を見てから流し始める
   (`tools/ota_send.py`)。転送中に止まるのは書き込みの ~9ms だけになる。
2. **commit の前に BLE を切る**。commit は両コアを 1.7 秒止めるので、
   繋いだままだと転送中の 16 倍の時間だけ CYW43 を放置することになる。
   ★切るのを頼むだけにして、`gap_disconnect` は **ble_uart の poll ループ**
   が呼ぶ (`REQUEST_DISCONNECT`)。呼び出し元スレッドから btstack を触ると
   それ自体が CYW43 の SPI を壊す (ble_uart.hpp の SEND 廃止と同じ罠)。

**中断された転送も捨てるようにした**。途中で切れると「残りを待つ」状態で
居座り、**次の転送のヘッダをその続きとして食う** (実測: 前の実験の CRC を
`want=` に出して MISMATCH になった)。5 秒バイトが来なければ捨てて待ち受けへ
戻る。

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

### スレッドを選ぶには non-stop が要る — ただし **CLI 限定** (2026-08-25)

**サーバは対応済み** (`qSupported` で `QNonStop+` を出し、`QNonStop:1` を受け、
`%Stop` の非同期通知と `vStopped` まである。D53)。GDB は `set non-stop on`
されていないと `QNonStop:1` を送らないので、**繋ぐ前に**立てる:

```sh
arm-none-eabi-gdb bazel-bin/firmware_bazel/xno_bringup \
  -ex 'set non-stop on' \
  -ex 'directory bazel-flight_robocon_telemetory_sender' \
  -ex 'target remote :3333'
```

実測 (BLE 経由、2.1s、パケットエラー 0):

```
  Id   Target Id     Frame
  1    Thread 1      (running)
  2    Thread 2      (running)
* 3    Thread 3      shizuku::archs::armv8m::syscall (...) at armv8m.hpp:383
  4-13 Thread 4-13   (running)
```

13 本全部見え、`thread N` で切り替わる (`[Switching to thread 6]`)。

### ★★VS Code では non-stop を使わない (表示が嘘になる)

**DAP は単一プロセス内のスレッドを前提**にしていて、停止も再開もプロセス単位。
Shizuku のスレッドは実質「別プロセス」なので、そもそも噛み合わない。実際に
non-stop で繋ぐと、13 本は並ぶが **全部 running か全部 paused のどちらか**に
なり、per-thread の状態が出ない (2026-08-25 実測)。

→ **VS Code は all-stop** (`.vscode` の "GDB over BLE (cppdbg)")。all-stop なら
スタブは**止まっているものだけ**を見せるので、対象 1 本だけが並び、他は GDB
から見えないまま走り続ける。**嘘が無い**。13 本見たいときは CLI を使う。

★**cortex-debug は使わない**。SWD プローブ + all-stop 前提の作りで、
`overrideAttachCommands` を走らせる**前に自分で繋いでしまう**ため、接続前にしか
変えられない設定 (`non-stop` / `mi-async`) を渡せない:

```
Cannot change this setting while the inferior is running.
Failed to launch GDB: ... (from interpreter-exec console "set mi-async on")
```

素の `cppdbg` (ms-vscode.cpptools) なら `setupCommands` が確実に接続前に走る。
`launchCompleteCommand: "None"` を付けて、繋いだあと run も continue も
させないこと (既に走っている的なので)。

### 覗く相手を実行時に選ぶ — `monitor` (2026-08-25 実装)

all-stop では「止まっているものだけ」が GDB に見えるので、**対象を選べないと
合成時に決めた 1 本しか触れない**。`qRcmd` を実装して選べるようにした。

```
(gdb) monitor list
  1  root           running
  3  blink          stopped   <- target
  6  gdbserver      running
  7  ble_uart       running   [transport: 止められない]
 10  telemetry      running
 11  flight_controller running
 12  bno055         running
(gdb) monitor target 10
target is now 10 (telemetry)
```

VS Code なら DEBUG CONSOLE に `-exec monitor target 10`。起動時に決め打ちに
したければ `launch.json` の `postAttachCommands` に書く。

**名前は `DECLARE_NAME` のもの**を出している (`qThreadExtraInfo` を実装した)。
`info threads` にも出る:

```
* 1  Thread 3 (blink [stopped])                syscall (...) at armv8m.hpp:383
  2  Thread 10 (telemetry [stopped] *target*)  syscall (...) at armv8m.hpp:383
```

★★**転送スレッドは選べない**。止めた瞬間に RSP を運ぶ者が居なくなり、
**止めた本人が復旧できない** (無線だと電源再投入まで戻らない)。`ble_uart` が
起動時に `gdb_link_protect_thread()` で申告し、stub が弾く:

```
(gdb) monitor target 7
7 is the debug transport — 止めると自分が喋れなくなる
```

★★★**本来これは System Object が持つべき制約** (誰が誰を止めてよいか)。
stub が自前の表で守るのは、資源の階層がまだ無いための繋ぎ。入ったら移すこと。

### 未解決: 一時停止ボタンが必ず対象スレッドを止める

VS Code で止めるボタンを押すと**必ず「今の対象」が止まる** (`monitor target`
で変えられるようになったので実害は減ったが、選んだスレッドを止めているわけ
ではない)。
`vCont;t` はスレッド指定が無いと `resume_thread_id()` = 既定に倒れるため
(`gdb_stub.cpp`)。RSP の仕様では「スレッドを書かないアクション = 他に
書かれていない全スレッドへの既定動作」なので `vCont;t` 単独は本来
「全部止めろ」だが、**この構成で全部止めると BLE の poll スレッドまで
止まり、デバッガ自身が喋れなくなる**。直すなら「止めてよいスレッド」の
線引きが要る。Shizuku 側 server の話。

★調べる道具は用意してある: `tools/gdb_ble_bridge.py --log FILE` で RSP を
そのまま記録できる (VS Code のデバッグタスクは常に
`.vscode/rsp-last.txt` へ記録する)。

### 未解決: 前のセッションの通知が次へ漏れる疑い

non-stop のセッションのあとに all-stop で繋ぐと、GDB が

```
gdb/thread.c:1434: internal-error: switch_to_thread: Assertion `thr != NULL' failed.
```

で落ちたことが 1 度ある。別の機会に記録を取ったときには、**`QNonStop:1` の
ネゴシエーションより前に** `%Stop:T05thread:3;` が飛んでいた。スタブ側に
「握手が済むまで通知を送らない」判断はあるので (`g_notify_allowed`)、切断時の
リセット漏れが疑わしい。**再現手順は未確定**。

### ★踏んだ: 長い返事が BLE で欠ける (2026-08-25, 修正済み)

D53 で `target.xml` (830 文字) を返すようになった瞬間に出た。症状は

```
Ignoring packet error, continuing...
Ignoring packet error, continuing...
⏱  224.232s        ← attach するだけで。前は 1.5 秒
```

原因は Shizuku 側 `gdb_stub.cpp` の `flush_out()` が満杯のまま `push()` して
いたこと。**`stream::push` は LOSSLESS 旗が無ければ古いレコードを黙って
上書きして `true` を返す**ので、欠けたことに誰も気付けない。リングは
8 スロット x 64B = **512 バイト**しかなく、830 文字は**1 パケットが丸ごと
入らない**ので、待たない限り必ず欠ける。

同じ話の CDC 側 (`write_byte`) は先に直っていた。FIFO が大きいぶん露見しにくく、
**転送をストリームに替えた側だけ取り残されていた**。CDC 側と対称に、空くまで
上限付きで待って譲るようにした。

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
- ★★**橋とテレメトリ/OTA は同じ 1 本の接続を取り合う**。橋が握っている間
  デバイスは advertise しないので、`ota_send.py` は **「device not found」**
  で落ちる —— これが「GDB の後は焼けない」の正体だった (2026-08-24 に再現)。
  VS Code の background タスクはデバッグセッションが終わっても勝手には
  止まらないので、**橋は GDB が切れたら自分から終わる** ようにしてある
  (`--stay` で居座らせられる)。`ota_send.py` 側も、見つからないときは
  橋が動いていないか調べて名指しする。

実測 (2026-08-24, pico2_w, AC 給電・USB 未接続):

```
Breakpoint 1, (anonymous namespace)::blink_main () at firmware_bazel/main.cpp:46
interval_ms(lr)=1400  step(r4)=-100
interval_ms(lr)=1300  step(r4)=-100
interval_ms(lr)=1200  step(r4)=-100
```

ブレークポイント (FPB) も効く。上は OTA で入れた blink の掃引が実機で
動いていることを、LED を目で見るのではなく**レジスタの値で**確かめたもの。

### attach すると必ず `armv8m.hpp:383` で止まっているのはなぜか

**ブレークポイントではない。** スタブは attach された瞬間に対象スレッドを
止める (`kernel_instance.suspend(g_target_thread)`、「繋いできた側は止まって
いることを期待している」)。対象は `main.cpp` が渡した **blink** で、blink は
300〜1500ms の `SLEEP_US` でほぼ寝ているので、**捕まえると必ずその syscall の
中にいる**。`armv8m.hpp:383` は `svc 0` の**直後の行** (`return {r0, r1};`) で、
寝ているスレッドの PC が自然に居る場所。

対象を変えたいなら `main.cpp` の `start_gdb_stub_over_stream()` に渡す
スレッド番号を変える (non-stop なら他のスレッドは走ったまま見える)。

## 既知の未解決

- **advertise していないときは、控えたアドレスで直接繋ぐ** (2026-08-25 対策)。
  BLE のリンクは 1 本しかなく、誰かが繋いでいる間デバイスは advertise しない。
  ★**掴まれている相手でもアドレスが分かれば 0.7 秒で繋がる**ことを実測した。
  `tools/shizuku_link.py` の `find_device()` が「スキャン → ダメなら控えた
  アドレス」を面倒見る (`ota_send.py` / `gdb_ble_bridge.py` の両方が使う)。
  控えは `$TMPDIR/shizuku_device.txt`。
  ★接続ウォッチドッグ (ble_uart) は**この症状には効かない**。テレメトリが
  流れている限り `ATT_EVENT_CAN_SEND_NOW` が来続け、**自分の送信を生存の
  証拠に数えてしまう**ので 20 秒の無音が訪れない。かといって入力だけを
  数えると、受信専用の購読者を切ってしまう。**未解決**。
- **転送を中断すると、その後デバイスが advertise を再開しないことがある**
  (USB CDC は生きていて、センサも回っている)。2026-08-24 にまた踏んだ ——
  ホスト側のプロセスを殺しても `[BLE_UART] disconnected` が出ず、macOS が
  リンクを掴んだままに見える。`picotool reboot -f -a` でも戻らず、
  BOOTSEL からの焼き直しで復帰した。**要調査**。
- 転送中にごく稀に `got unexpected packet 0` が出て、そのまま止まることが
  ある (2026-08-24: 93% で停止)。ホスト側は 10 秒で諦めて「stalled」と
  報告し、デバイス側は 5 秒で捨てて待ち受けに戻るので**どちらも無傷**だが、
  原因は分かっていない。上の CYW43 の話と同根の可能性。
- **GDB のブレークポイントは対象スレッドを選べない**。スタブは RSP に
  スレッドを 1 本しか見せておらず (`thread:1;` 固定、`H` は受けるだけ)、
  対象は `start_gdb_stub_over_stream()` に渡した番号で決め打ち
  (いまは `main.cpp` が blink を渡している)。Shizuku 側の作りの話。
