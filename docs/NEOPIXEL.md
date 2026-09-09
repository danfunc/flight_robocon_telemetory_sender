# ハンズオフ表示 (2026-09-09)

対象: Raspberry Pi Pico 2 W / RP2350 / Shizuku、WS2812 RGB 1 個。
これは XIAO が報告する操縦モードの表示。飛行制御の正常動作を保証する信号ではない。

## 設計

- GP16 を使用。旧 `WS2812_DRIVER.cpp` の配線を踏襲し、使用中の GP0/1
  (UART0)、GP6/7 (I2C1) と競合しない。ADC GP26–28 と既定の
  I2C0 GP4/5、UART1 GP8/9 も温存する。代替機能は PIO 選択時には使わない。
- 旧実装の単線用 `ws2812.pio` を使用。SDK 2.2.0 の
  `pico_generate_pio_header` が毎ビルドで必要に応じて pioasm を実行する。
  SM/PIO は SDK の GPIO 範囲対応 claim 関数で動的確保し、CYW43 と共有する。
  固定 PIO0/SM0 や CPU ビットバングは使わない。
- `shizuku_shell` の UART0 受信だけが `safety::update` を専用 SPSC stream
  に送る。メソッドで stream ID を配線。表示状態は NeoPixel の poll スレッド内に保持。
  登録・配線後に consumer を shell より先に SPAWN する。
  既存の blink/GDB 起動処理の位置は保持している。
- **ストリーム ID 0 の契約と未配線判定**:
  - Shizuku OS の `STREAM_CREATE` は成功時にインデックス `0` を返し得る（`0` は正当なストリーム番号）。成否は戻り値ではなく `error == 0` で判定する。
  - 未配線・未生成・生成失敗の番兵定数にはリポジトリ共通の `xno::NO_STREAM = (uintptr_t)-1`（`tx_frame.hpp`）を使用する。
  - `shizuku_shell`: `g_safety_out_id` の初期値を `xno::NO_STREAM` とし、`STREAM_CREATE` の `error == 0` の場合のみ生成 ID（`0` を含む）を保存。失敗時は `xno::NO_STREAM`。GET メソッド (`method_get_safety_stream`) はこの ID を返す。`poll_loop` 内の PRODUCER バインドも `g_safety_out_id != xno::NO_STREAM` を条件とする。
  - `main.cpp`: `GET_SAFETY_STREAM` 呼び出し後、`safety_stream.error == 0 && safety_stream.value != xno::NO_STREAM` を確認して配線する。ID `0` を未配線として誤拒否しない。
  - `neopixel.cpp`: 入力ストリーム ID `g_input_id` の初期値を `xno::NO_STREAM` とし、`g_input_id != xno::NO_STREAM` の場合のみ `STREAM_OPEN` を実行する。ID `0` を正当な入力として受け付け、未設定時の不正オープンを防ぐ。
- `SAFE2.mode_str` を一次情報にする。XIAO の裁定結果を表すため、単なる
  `gear_us >= 1500` 判定では反映できない安全装置の MANUAL 切替も表示できる。
  gear は数値として検証・転送するが、表示判定の閾値には使わない。
- `BRIGHTNESS = 255`。屋外視認性を優先して赤か緑の単色だけを全開にする。

## 電気的前提・想定結線と電流見積り

採用 LED の正確な型番・改版、実際の給電、既設レベル変換器、配線は未提示である。
実装済みの結線として断定せず、「想定結線」と「実機で確認する項目」を明確に区別する。

### 想定結線
- **給電**: LED は 5 V 給電とする。RP2350 の 3.3 V 出力ピンを LED 電源として使用しない。
- **信号レベル変換**: GP16 の 3.3 V 信号は、3.3 V 入力を受けられる 5 V 動作のレベル変換バッファ（候補: 74AHCT 系等）を介して LED DIN に接続する。採用品の入力仕様（$V_{IH}$ 閾値）は実機担当者が確認する。RP2350 の 3.3 V 出力から 5 V 給電 LED への直結は保証条件としない。
- **共通 GND**: Pico、LED、レベル変換バッファの GND を共通化する。
- **直列ダンピング抵抗**: バッファ出力と LED DIN の間に直列抵抗を挿入する。設計初期値は 330 Ω とし、実配線・波形による確認対象とする。
- **最大明るさ時の電流・電力見積り**:
  - `BRIGHTNESS = 255`（最大輝度）設定でも、点灯パターン上、赤または緑の 1 チャンネルのみが点灯する（白色点灯のように RGB 3 チャンネルが同時点灯することはない）。
  - 従来型の 1 チャンネル 20 mA を仮定した暫定見積りでは、1 素子約 20 mA に内部ロジック回路分を加えた値となる。
  - 5 V 給電時における発光チャンネル分の消費電力は概算で約 0.10 W（5 V × 20 mA）。
  - **注意**: これは採用型番の保証値や実測値ではなく、LED の型番・世代によって異なる。白色 3 チャンネル分の 60 mA を今回の単色電流と記載・想定してはならない。正確な電流値は実機測定で確認する。

| 条件 | 表示 |
| --- | --- |
| 新鮮な SAFE/SAFE2 が AUTO で一致、異常なし | 緑 |
| 新鮮な SAFE/SAFE2 が MANUAL で一致、異常なし | 赤 |
| 起動後未受信、どちらかの有効行から 1 秒以上 | 赤点滅 |
| SAFE の link_ok=0 / failsafe_active=1 | 赤点滅 |
| SAFE2 の LINK_LOST / BAD_PULSE / SW_INHIBIT / 未知 fault | 赤点滅 |
| SAFE/SAFE2 の mode 不一致 | 赤点滅 |

`PILOT_MANUAL` は MANUAL の場合だけ正常扱い。点滅は 250ms ON / 250ms OFF。
2 行間に共通 sequence がないため同一 heartbeat の厳密な照合はできない。
モード切替の途中で一時的に不一致を観測すると赤点滅になる。
SAFE の mask は数値として検証するが、XIAO がその周回で捕捉したパルスの
マスクなので、ギアの有効性を独自判定する条件には使わない。

SAFE 接頭辞の全行は応答なし。不正行・未知 SAFE 種別は freshness を更新しない。
BLE/CDC/PROCESS_CMD からの SAFE でも表示は更新しない。
consumer が遅れた場合は古い stream レコードを上書きし、欠落検出で状態を
リセットする。LED 待ちで UART producer を停止させない。

PIO は GRB 24bit/MSB-first、800kbit/s。設計波形は 1bit=1.25µs、
0 の High=0.25µs、1 の High=0.875µs。通常 20ms ごとに 1 個分を送る。
遅延後の追い付き周回でも送信開始間隔を最低 1ms に制限し、30µs のデータ後に
970µs 以上の Low リセット時間を確保する。毎周回で相対残時間の SLEEP_US を通す。
PIO 確保/初期化失敗は診断して終了。stream 配線失敗は診断し、赤点滅を維持する。
FIFO 満杯は診断して非ブロッキングで再試行するが、表示更新は保証できない。

## 実施済み検証

cwd: `/Users/ishigakiyua/github/flight_robocon_telemetory_sender`

1. `bazel build //firmware_bazel:uf2`: 終了 36。既定 output base が書き込み不可。
2. 一時 output base のみの再実行: 終了 36。install base のロックが書き込み不可。
3. 一時 output user root + 元 cache: 終了 37。repo contents cache のロック失敗。
4. コピーした cache のパスを `content_addressable` にした実行: 終了 32。
   registry をキャッシュから解決できず `Unknown host: bcr.bazel.build`。
5. コピーした cache の root を正しく指定: 終了 1。
   現行 Shizuku が `set_object_handler(entry, object_id)` を要求する一方、
   アプリ側の既存 main が 1 引数だった。Shizuku 本体の main と同じ
   `KERNEL_OBJECT::KERNEL_OBJECT_ID` を追加して整合させた。
6. 以下の最終 UF2 ビルド: 終了 0。

```sh
bazel --batch --output_user_root=/private/tmp/xno-neopixel-bazel-root build //firmware_bazel:uf2 --repository_cache=/private/tmp/xno-neopixel-repo-cache --symlink_prefix=/private/tmp/xno-neopixel-
```

次の host 試験も終了 0、86 checks。firmware と同じ parser/policy ヘッダを使用し、
ハードウェア・Shizuku stream・UART 全体・PIO 波形は試験の対象外。

```sh
bazel --batch --output_user_root=/private/tmp/xno-neopixel-bazel-root test --config=host //:safety_status_test --repository_cache=/private/tmp/xno-neopixel-repo-cache
c++ -std=c++23 -Wall -Wextra -Werror -I. tests/safety_status_test.cpp -o /private/tmp/xno-safety-status-test && /private/tmp/xno-safety-status-test
```

警告は抑制していない。Bazel の依存版差異・非推奨ルール、SDK pioasm の
`variable 'yynerrs_' set but not used`、Shizuku の volatile ++/--、GDB stub の
`null character(s) preserved in literal` が出た。BTstack 生成時は
`[!] PyCryptodome required to calculate GATT Database Hash but not installed (using random value instead)`
と報告された。これは依存先が元から持つ fallback で、本変更で導入したものではない。
GATT hash の再現性は未解決。sysctl 権限と JVM deprecated options の警告も出た。

証跡: `build/neopixel-validation/` (git 管理外)

- `build-api-failure.log`: 最初のコンパイル失敗と初回依存コンパイルの警告原文。
- `build-success.log`: 最終ビルドの警告・成功原文。
- `host-test.log`, `test.log`: Bazel 試験結果と 86 checks の出力。
- `ws2812.pio.h`: pioasm 2.2.0 の生成結果。4 命令 `6221 1123 1400 a442`。
- `xno_bringup.uf2`: 994304 bytes。
  SHA256 `c5806e95b597d833db24462afca22f2d33f5d2125fee521943ec5f1f5ba206f5`。

## 実機受入・実機で確認する項目 (未実施)

本作業では書き込み・実機接続試験は実施していない。以下は「想定結線」に対する実機確認項目。

1. LED の正確な型番、GRB/RGBW、電源と GP16 の論理レベル、共通 GND、DIN の
   向きを確認する。GP16 を電源には使わない。レベル変換バッファ（74AHCT等）の入力閾値特性と 330 Ω ダンピング抵抗の波形品質、屋外での距離・角度ごとの視認性と単色全開時の実電流・消費電力を測定する。
2. 起動ログの `[NEOPIXEL] GP16 PIO... SM...` を記録する。エラーがないこと、
   XIAO 未受信で赤点滅、BLE とセンサが従来どおり動くことを確認する。ストリーム ID 0 が割り当てられた場合でも `[XNO BOOT] safety stream unavailable` とならずに正常配線され、表示が動作することを確認する。
3. 実 XIAO → UART0 DMA → shell parser → safety stream → NeoPixel PIO の
   経路で、実スイッチを AUTO/MANUAL に切り替える。UART0 を受動観測した
   SAFE/SAFE2、LED 動画、GP16 ロジアナ波形を同時に保存する。
   USB/BLE 経由の注入だけでは本番経路検証の合格にしない。
4. UART0 の入力を停止し、最後の有効行受信から 1 秒 + poll/スケジューラ遅延で
   赤点滅になることを確認する。通常時の poll 周期は 20ms。
   SAFE と SAFE2 の片方だけ継続する場合も緑を維持しないことを確認する。
5. 実リンク断の SAFE/SAFE2 を観測し赤点滅、復帰で正常表示すること。
   不正行、未知 SAFE 種別を UART0 から与えてエラー応答が出ないこと。
   本番 heartbeat を止めた状態で USB/BLE に AUTO を入力しても緑にならないこと。
6. GP16 の 24bit GRB、High 幅、Low リセット間隔をロジアナで検証する。
   BLE/センサ負荷中も白色化や別色化がなく、UART 応答に SAFE 由来の返信が
   混ざらないこと。OTW/OTA の受入は別途許可された書換試験で実施する。

表示 timeout は Pico が行を解析した受信時刻に基づく。UART/DMA に残った古い
行の送信時刻や XIAO の reboot/sequence は追跡していない。RTOS 全停止、電源断、
LED/PIO 故障では赤点滅も保証できず、最後の色が残る場合がある。OTW/OTA 中は
heartbeat 解析が止まる既存仕様のため、RTOS が動く間は timeout により赤点滅する。
