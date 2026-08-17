# XIP (QSPI フラッシュ) クロック — 実測と設定

最終更新: 2026-08-17 / pico2_w (RP2350 + W25Q32 クラス) / pico-sdk 2.2.0

## 結論

* **`PICO_FLASH_SPI_CLKDIV` / `PICO_FLASH_SPI_RXDELAY` は効かない。**
  SDK 2.2 の RP2350 既定ビルドに boot2 は入っておらず (`main.elf` に `.boot2` 無し)、
  QMI の `M0_TIMING` は**ブート ROM が設定した固定値 (div=3 / rx=2)** のまま走る。
* 分周比が固定なので **フラッシュ速度が `clk_sys` に比例して勝手に動く**。
  150MHz ビルド → フラッシュ 50MHz / 300MHz ビルド → フラッシュ 100MHz。
  前者は遅すぎ、後者は「意図して選んだ値」ではなかった。
* 対処は **目標周波数を宣言して分周比を実行時に導出する** (`flash_clock.cpp`)。
  `SHIZU_FLASH_TARGET_KHZ` (既定 100000) から `div = ceil(clk_sys / target)`。
  この 1 つの値で 150MHz→75MHz、300MHz→100MHz と両方正しくなる。
* **2026-08-17: 300MHz + フラッシュ 100MHz を既定にした。** 引数なしの `cmake` で
  この構成になる (`[CLK] sys=300000000 Hz ... flash=100000000 Hz` で確認)。
  150MHz に戻すには `-DSHIZU_SYS_CLK_KHZ=0` (クロックを一切触らない)。
  既定化の根拠: core0 util 5.6-5.7% / evt=68 / panic 0 / 異常行 0 を反復確認、
  **ダイ温度 38.8℃** (BLE アドバタイズ中・室温・机上)、かつ clk_sys 依存の
  固定分周をすべて導出化して「クロックを変えると黙って壊れる隠れた前提」を消した:

  | 依存 | 状態 |
  |---|---|
  | フラッシュ QMI `M0_TIMING` | 目標周波数から導出 (`flash_clock.cpp`) |
  | CYW43 PIO SPI `CYW43_PIO_CLOCK_DIV_INT` | 目標 75MHz から導出 (CMakeLists)。**4 の直書きをやめた** — 従来は `-DSHIZU_SYS_CLK_KHZ=150000` を明示すると SPI が半速になる潜在バグがあった |
  | WS2812 PIO | 元から `clock_get_hz(clk_sys)` 由来 (`ws2812.pio:42`) |
  | I2C / PWM | SDK が clk_sys から算出 |
  | clk_peri | 300MHz では 48MHz に落ちる。SPI/UART 未使用なので無害。**後で SPI を足すときは要注意** |

## ベンチの回し方

    cmake -S . -B build_xipbench -DSHIZU_XIP_BENCH=ON -DSHIZU_SYS_CLK_KHZ=300000
    cmake --build build_xipbench
    python3 tools/flash_watch.py build_xipbench -t 25 -g "[XIP]"
    python3 tools/xip_bench_check.py build_xipbench/main.bin   # ← 必ず突き合わせる

`[XIP] baseline checksum=...` がホスト側の再計算と一致して初めて「基準が正しい」と
言える。ファーム内の比較だけだと、基準自体が壊れていた場合に全設定が同じ壊れ方を
して「全部 ok」に見える。設計理由は `xip_bench.cpp` の冒頭コメント。

## 実測 (2026-08-17)

`seq_nc` = nocache 窓の連続 32bit 読み、`ifetch` = 64KB に散らした関数をランダムに
呼んだときの 1 呼び出しあたり (= 命令フェッチのストール)。

### clk_sys = 300MHz

| div | flash | seq_nc | seq_c | ifetch | 判定 |
|---|---|---|---|---|---|
| 6 | 50MHz | 22.0 MB/s | 22.0 MB/s | 1144 ns | ok |
| 4 | 75MHz | 30.5 MB/s | 30.9 MB/s | 796 ns | ok |
| **3** | **100MHz** | **37.2 MB/s** | **38.4 MB/s** | **628 ns** | **ok ← 既定** (rx=3 では 636 ns) |
| 2 | 150MHz | 47.7 MB/s | 50.7 MB/s | 458 ns | ok だが **定格外** (W25Q32 は 133MHz) |

### clk_sys = 150MHz

| div | flash | seq_nc | seq_c | ifetch | 判定 |
|---|---|---|---|---|---|
| 6 | 25MHz | 11.0 MB/s | 11.0 MB/s | 2288 ns | ok |
| 4 | 37.5MHz | 15.3 MB/s | 15.5 MB/s | 1591 ns | ok |
| 3 | 50MHz | 18.6 MB/s | 19.2 MB/s | 1257 ns | ok ← **従来の既定はここだった** |
| **2** | **75MHz** | **24.8 MB/s** | **25.9 MB/s** | **902 ns** | **ok ← 現在の既定** |

### RXDELAY の窓 (単位 = `clk_sys` の**半**サイクル)

RXDELAY は 3bit なので全域は 0..7。★**必ず全域を測ること** — 端がどこか分からないと
マージンを語れない。実際 0..4 だけ測った段階では既定 rx=2 が「下1/上2」で妥当に見えたが、
全域を測ると 100MHz の窓は 1..6 で、rx=2 は**下端の隣**だった (下1/上4 = 偏り)。

| clk_sys | div | flash | OK | NG (= 静かに読み違える) | 窓の中心 |
|---|---|---|---|---|---|
| 150MHz | 3 | 50MHz | 0-4+ | — | — |
| 150MHz | 2 | 75MHz | 0-3 | **4, 5** | 1.5 |
| 300MHz | 4 | 75MHz | **0-7 全部** | — (端が見つからない) | — |
| 300MHz | 3 | 100MHz | **1-6** | **0, 7** | **3.5 → 既定 rx=3** |
| 300MHz | 2 | 150MHz | 2-4+ | **0, 1** | — (定格外) |

速いほど**下限が上がり** (往復遅延が相対的に伸びる)、上限は「半サイクル数 × 周期」が
10ns 前後で頭打ちになる。既定 `SHIZU_FLASH_RXDELAY=3` は 100MHz の窓 (1-6) の中心。
rx=2 から 3 へ寄せるコストは ifetch +1.1% / seq -3% だけで、下側マージンが 1→2 段になる。
★クロックを変えたらスイープを回し直すこと。

### 実機の系全体への効き (BLE アドバタイズ中、40-60s × 2 巡、`[CPU]` の分散は 0)

| 構成 | フラッシュ | core0 util |
|---|---|---|
| 300MHz | 50MHz | 6.1 - 6.2 % |
| 300MHz | **100MHz** | **5.7 %** (-7%) |
| 150MHz | 50MHz | 7.1 % |
| 150MHz | **75MHz** | **6.9 %** |

★**75MHz は 50MHz と区別できない** (どちらも 6.1%)。系への効きは 100MHz でだけ
階段状に現れる。「75MHz にすれば安いコストで RXDELAY マージンが買える」という
見立ては実測で否定された — フラッシュ高速化の利得をほぼ全部捨てることになる。

マイクロベンチの -45% に対して系全体は -7% にとどまる。ホットパスの大半は
16KB キャッシュに載っており、フラッシュを待っているのはミスした一部だけ、ということ。
core1 (BLE poll) の util は 51.5% で不変 — こちらはフラッシュ律速ではなく
ポーリング周期律速なので、この軸では動かない。

## フラッシュ実物 (JEDEC ID 実測)

`0x9F` の応答は `mfr=EF type=40 cap=16` = **Winbond W25Q32JV / 4MB**。
データシート上、全 Fast Read 命令の上限は **133MHz**。よって 100MHz は余裕があり、
上の表の 150MHz は(読めてはいても)**定格外で確定**。採用しない理由がこれ。

## フラッシュ操作でタイミングが ROM 既定へ戻る (実測確認済み)

boot2 が無い場合、SDK は `flash_init_boot2_copyout()` が **`BOOTRAM_BASE` (0x400e0000)**
から 64 語コピーしたもの = ブート ROM が boot RAM に置いた XIP セットアップ関数を
再実行する (`flash_enable_xip_via_boot2`)。そして `flash_restore_hardware_state` が
戻すのは QSPI パッドと **m[1] (CS1) だけ**で、**m[0] の timing は保存も復元もされない**。

`flash_do_cmd` は `flash_range_erase` とまったく同じ
`flush_cache → enable_xip_via_boot2 → restore_hardware_state` で終わるので、
**1 バイトも書かずに** `flash_get_unique_id()` 1 回で同じ経路を踏める
(`cmake -DSHIZU_FLASH_DRIFT_PROBE=ON`):

    [FLASHP] before      timing=60007202 flash=75000000 Hz
    [FLASHP] after do_cmd timing=60007203 flash=50000000 Hz drifted=1

core0 util も 6.9% → 7.2% に上がり、性能低下が系の指標にも出た。
ビーコンの `[FLASH] timing drifted -> ... Hz` も発火を確認済み。

* **300MHz では実害なし** — 目標 (div=3) と ROM 既定 (div=3) が同じ値なので失うものが無い。
  差が出るのは 150MHz ビルド (75→50MHz) だけ。
  ★この「applied == ROM 既定」だと**ドリフトが起きても検出できない**罠がある。
  実際 300MHz で最初に測って `drifted=0` を得てしまい、150MHz で測り直して確定させた。
* **現状の実運用では発生しない** — 下記のとおり BTstack の書き込み自体が実行されていない。
* 直すなら **core1 が止まっている `flash_safe_execute` の中**で再設定すること。
  他コアが XIP 転送中に `M0_TIMING` を書き換えるのは安全でない。

## ★別件で見つかった問題: BTstack のペアリング鍵は保存されていない

    [FLASHP] flash_safe_execute(noop) rc=-4      (= PICO_ERROR_NOT_PERMITTED)

`pico_flash/flash.c` は「他コアが走っている && `multicore_lockout_victim_is_initialized`
が false」でこれを返す (Release では `assert` が消えるので**無言で失敗する**)。
このプロジェクトは `flash_safe_execute_core_init()` をどこからも呼んでおらず、
core1 は常に走っている。`btstack_flash_bank.c` の書き込みは全部
`flash_safe_execute` 経由 (:76, :164) なので、**丸ごと no-op になっている**。

再起動のたびに再ペアリングが必要なはず。直すには core1 側で
`flash_safe_execute_core_init()` を呼ぶ必要があるが、`multicore_lockout` は
マルチコア FIFO と IRQ を使うので **Shizuku のコア間機構と競合しないか要確認**。
`PICO_FLASH_ASSUME_CORE1_SAFE=1` で黙らせるのは危険 — core1 が XIP を実行中に
フラッシュ書き込みが走ると即死する。**未対応**。

## ダイ温度の常時監視

`[BEACON] ... die=38.8C` — RP2350 内蔵温度センサ (ADC 最終チャネル) をビーコンが
5 秒ごとに出す (`main.cpp` の `die_temp_x10`)。ADC は clk_usb 由来なので clk_sys を
変えても換算式は変わらない。**触って熱くないことは根拠にならない**ので (ダイと筐体
表面は別、密閉した機体では風も無い)、数字を常にログに残す。これで飛行を含む実運用の
熱データが自動で溜まる。300MHz / vreg 1.20V / 室温・机上・BLE アドバタイズ中で 38.8℃。

## 既定値 (2026-08-17 確定) と、なぜその値なのか

引数なしの `cmake` でこの構成になる。`[CLK]` 行が実際に効いている値を全部印字する:

    [CLK] sys=300000000 Hz peri=48000000 Hz flash=100000000 Hz rx=3 (5000 ps) vreg=1200 mV

| つまみ | 既定 | 根拠 |
|---|---|---|
| `SHIZU_SYS_CLK_KHZ` | **300000** | フラッシュ目標 100MHz に**割り切れる** clk_sys は 300 (÷3) と 400 (÷4) だけ。400MHz は 1.20V の天井の外 (396MHz でハング)。324/348MHz へ上げると分周が 4 に跳んで**フラッシュが 81/87MHz へ下がる** = 上げ損。よって 300MHz が釣り合いの点 |
| `SHIZU_FLASH_TARGET_KHZ` | **100000** | 分周 = `ceil(clk_sys/target)`。定格 133MHz (W25Q32JV 実測) に対し余裕あり |
| `SHIZU_FLASH_RXDELAY_PS` | **5000** | 段数でなく**遅延 [ps]** を宣言し `ps × 2 × clk_sys` で段数を導出。300MHz→3 段 / 150MHz→2 段。実測窓を ns に直すと上限は ~11ns 一定で下限だけがフラッシュ速度で上がるので、5ns は全ケースで内側 |
| `SHIZU_PERI_TARGET_KHZ` | **150000** | SPI/UART の元クロック。`set_sys_clock_khz` は clk_peri を PLL_USB (48MHz) に落とすので、PLL_SYS から分周し直して 150MHz にする。**CPU 300MHz のまま SPI/UART は 150MHz** |
| `SHIZU_CYW43_SPI_TARGET_KHZ` | **75000** | 既知良 (150MHz/div2 と 300MHz/div4 が同じ 75MHz) |
| `SHIZU_VREG_MV` | **1200** | 300MHz に対し 24% の余裕。上げても天井は +16% しか伸びず、恒久的に電力と素子ストレスを払うだけ。熱は律速していない (全負荷 40.2℃ 飽和) |

**★clk_sys 依存の値はすべて「物理量を宣言して導出」に統一した。** 直書きしていたのは
フラッシュ分周・CYW43 分周・RXDELAY 段数の 3 つで、どれも「そのクロックのときだけ
正しい値」だった。導出にすれば clk_sys を変えても意味が変わらない。

### clk_peri (SPI / UART) — CPU 300MHz のまま 150MHz にしてある

`set_sys_clock_khz` は clk_peri を **PLL_USB (48MHz)** へ張り替える (SDK の既定選択)。
48MHz だと **SPI の上限が clk_peri/2 = 24MHz** になる — `spi_set_baudrate` の prescale が
最小 2 だから (`hardware_spi/spi.c` で確認)。UART は 48MHz でも
`UARTCLK/16` = 3Mbaud 出るので困らないが、SPI は用途次第で足りない。

**RP2350 の clk_peri には分周器がある** (`CLOCKS_CLK_PERI_DIV_INT`、2bit = 1..3、0 は 4)。
なので PLL_SYS から分周して「**CPU 300MHz のまま clk_peri 150MHz**」にできる。
150MHz は RP2350 の定格 sys クロックと同値なので周辺としても定格内で、SPI 上限は 75MHz。
分周比は clk_sys から導出する。★`stdio_init_all` より**前**に設定すること
(UART ボーレートは clk_peri から算出される)。

`PICO_CLOCK_ADJUST_PERI_CLOCK_WITH_SYS_CLOCK=1` (SDK オプション) は clk_peri を
clk_sys 直結にするが、300MHz は定格 150MHz の 2 倍なので**使わない**。

実測: `sys=300000000 peri=150000000` で 90s 通し、core0 5.8% / core1 51.4% /
`evt=68` / die 40.7℃ / 異常行 0。XIP チェックサムもホスト計算と一致。
**ただし SPI/UART は実機で結線していないので機能検証はしていない** — clk_peri の
読み戻し値と系の安定性までが確認済みの範囲。

`-DSHIZU_PERI_TARGET_KHZ=0` で SDK 既定 (48MHz) に戻せる。

## CPU クロックの天井 (2026-08-17 実測、`-DSHIZU_OC_PROBE=ON`)

pico2_w 実機 1 個 / 室温 / **core0 単独** (センサ・BLE 未起動)。

| vreg | 通った最高 | ハング |
|---|---|---|
| **1.20V** (出荷値) | **372MHz** | **396MHz** |
| 1.30V (SDK が無条件で許す上限) | **432MHz** | 450MHz |

* **600MHz は不可。** 1.20→1.30V で買えるのは +60MHz (+16%) のみ。1.30V より上は
  `POWMAN_VREG_CTRL_DISABLE_VOLTAGE_LIMIT` が必要 = 素子寿命の領域なので常用しない。
* **★1.30V の天井を常用余裕の根拠にしてはいけない。** 最初 1.30V だけ測って
  「432MHz まで行けるなら 300MHz は余裕」と読みかけたが、出荷電圧で測り直したら
  天井は 372MHz だった。**必ず出荷電圧で測る。**
* **既定 300MHz の余裕 = 372/300 ≈ 24%** (室温・core0 単独)。
* **熱は律速しない**: 300MHz でアイドル 39.3℃ → CPU 全負荷 120s で **40.2℃ で飽和**
  (+1℃、異常行 0)。432MHz でも 38.3℃。天井を決めているのはロジックのタイミングで温度ではない。
* **PLL の制約**: VCO = clk_sys × postdiv が **12MHz の整数倍** かつ 750-1600MHz。
  350 / 440 / 550MHz は**作れない**。postdiv=3 なら clk_sys が 4 の倍数で成立するので
  12MHz 刻みで並べられる。
* **未測定**: 両コア稼働 + 周辺 IRQ での天井。プローブは Shizuku 起動前に走るので core0 単独。
  実運用の天井はこれより低い。Shizuku 起動後にクロックを変えられない理由は
  `g_cycles_per_us` が cpu_manager::init で確定すること、および CYW43 PIO 分周が
  コンパイル時定数であること。

### 探索のやり方 (BOOTSEL 押しゼロで固まりを回収する)

`oc_probe.cpp`。**1 回のブートで階段を登り、各段をその場で印字して排出する。**
**ウォッチドッグをクロック変更の前に張り**、noinit RAM の `attempting_khz` に
「今試している値」を書く。固まれば WDT が再起動し、次のブートは noinit を見て
**登り直さない**ので正常起動して普通に焼き直せる。実際 450MHz と 396MHz の
ハングを 2 回とも BOOTSEL なしで回収できた。

検証は 2 本立て — フラッシュ読みのチェックサム + SRAM/演算の既知答え照合
(生きているのに静かに化けるケースを拾う)。終わったら必ずビルド想定クロックへ戻す。

    cmake -S . -B build_oc -DSHIZU_OC_PROBE=ON -DSHIZU_OC_PROBE_MV=1200   # 出荷電圧で測る
    cmake --build build_oc && picotool load -f build_oc/main.uf2

## その他の注意
* **150MHz のまま 100MHz にはできない。** 分周比は整数なので 150/1.5 は作れず、
  div=1 = 150MHz は定格外。100MHz が要るなら 300MHz へ OC する。
* 「起動した = OK」で判断しないこと。速すぎる設定は落ちるのではなく**静かに読み違える**。
