# Shizuku / flight_robocon_telemetry — 引き継ぎ資料

最終更新: 2026-07-17。このセッションで実装した内容・現在地・ロードマップ・ビルド/検証手順・
未決定の一方向ドアをまとめる。コード内の詳細コメントと `memory/` の設計メモも併読のこと。

## 0.5 2026-07-17 追加分

**実機検証状況 (2026-07-19)**: 素のビルド (BLE 込み) で panic-ring 沈黙 + BLE フルパス
(ペアリング/暗号化/notify/RX) 動作を確認 → **PSPLIM + MPU W^X は実機合格**。core0
util 52.7% (接続中の既知値) / core1 util 0% (センサ未接続の既知正常) も従来どおり。
未検証: [CONNTEST] (SHIZU_STREAM_SELFTEST=1) と set_affinity のコア移動。

| 機能 | 実体 | 要点 |
|---|---|---|
| **METHOD_CALL の型付きエラー** | kernel.cpp METHOD_CALL/METHOD_EXIT, kernel.hpp `call_error`, call_method.hpp | 未 export = `UNDECLARED_METHOD`、未生成/範囲外 = `BAD_OBJECT` を **panic でなく r0 で呼び出し元へ返す** (affinity 移動で「tid 昇順の初回実行順 = export 順」の暗黙保証が崩れるための防御)。METHOD_EXIT は arg2 をエラーコードとして復元フレーム r0 へ搬送 (既存呼び出しは arg2=0 でワイヤ互換)。ついでにメソッド戻り値が一般オブジェクト呼び出し元の r1 へ透過するようになった (従来は 0 に潰れていた)。エラーは `g_call_errors` カウント + core0 なら `[CALL] err=...` printf |
| **SET_AFFINITY (svc 22)** | kernel_object.cpp, obj_api.hpp `set_affinity(tid, mask)` | u32 一語の advisory 書き込み。確定は try_claim の CAS 側なので、反映は「対象が次に READY になって scheduler が claim するとき」。空マスク/範囲外は黙って無視。アプリのコア移動は `set_affinity(tid, AFFINITY_ALL/CORE1)` 一発 |
| **MPU Step0: W^X** | kernel.cpp `mpu_init_this_core` (per-core banked、core0=cpu_manager::init / core1=init_core1)、トグル `SHIZU_MPU_WX` (kernel.hpp、既定 1) | region0 = XIP 全窓 RO+X (flash への誤ストア即 fault)、region1 = ヒープ先頭 `__end__`〜SRAM 終端 RW+XN (全スレッドスタック = malloc 産 + scratch_x/y を含む。スタック/ヒープからの実行即 fault)。PRIVDEFENA=1 で region 外 (静的データ/ペリフェラル) は無傷。scratch_x/y にコードが無いことは map で確認済み。MemManage は未有効化 = HardFault へエスカレート (PSPLIM と同じ)。実機で問題が出たら `SHIZU_MPU_WX=0` |
| **ストリーム接続 + DMA ポンプ (svc 23)** | kernel_object.cpp `connect_streams`/`stream_pump_core0`, stream.hpp `connect(src, dst)` | src の consumer 席と dst の producer 席を接続が占有 (SPSC 維持、`CONN_OBJ` マーカー)。core0 の scheduler idle ループが毎周回、src 連続可読×dst 連続空きの min を 1 DMA 転送として非ブロッキング発行 → 完了周回で index publish。**中間オブジェクトの pop→push コピーが丸ごと消える**。DMA チャネルは接続ごとにカーネルが claim (オブジェクトへ渡さない = HANDOFF §6 の「DMA 設定はカーネル専有」)。dst へは空きにしか書かない (溢れは src 側に滞留)。src lossy の torn は pop と同じ論理で resync + 破棄。rec_size 不一致は `MISMATCH`。切断 API は未実装 (静的トポロジ前提) |
| **connect 自己テスト** | stream_selftest.cpp `[CONNTEST]` | LOSSLESS の SRC→DST を connect し、consumer は「欠落ゼロ + 連番完全一致」(bad=0) を判定。`SHIZU_STREAM_SELFTEST=1` で有効 |

**実機で次にやる検証**: (1) 素のビルドで panic-ring 沈黙 + BLE 動作 (= PSPLIM + MPU W^X 合格)、
(2) `SHIZU_STREAM_SELFTEST=1` で `[CONNTEST] bad=0` (= DMA ポンプ合格)、(3) どれかのアプリ
スレッドを `set_affinity(tid, AFFINITY_ALL)` でコア移動 (BLE は core0 固定のまま)。

## 0.6 2026-07-20 追加分

**実機検証 (2026-07-20)**: BLE on core1 **合格** — core0 util 52%→**6.0%** / core1 54.7%、
tx 統計 (26 pkts/s)・RSSI・接続維持・panic-ring 沈黙すべて従来どおり。core0 は飛行制御
向けにほぼ空いた。svc 即値ハイブリッド ABI も同ビルドで実動 (全 SVC 経路が通っている)。

| 機能 | 実体 | 要点 |
|---|---|---|
| **BLE ドライバの core1 化 (実験)** | kernel.hpp `SHIZU_BLE_ON_CORE1` (既定 1)、IO_CONTROLLER.cpp | cyw43/btstack/async_context の縛りは「**init を実行したコアからのみ**」なので、init〜poll ループ全体を core1 ピン留めスレッドに載せて整合させる (CYW43 の GPIO IRQ / async_context owner も core1 に付く)。戻すには 0 |
| **生成時 affinity/budget + READY 最終公開** | kernel.cpp `create_thread`/`async_call`、obj_api `async_call(obj, entry, arg, affinity, budget0)` | 「作ってから set_affinity/set_thread_budget」は呼び出し元が grant 期限でプリエンプトされた隙に既定 affinity のまま claim される隙間がある (BLE だと **cyw43_arch_init が core0 で始まる事故**)。affinity/budget を READY 公開前に確定し、READY は release store で最後に publish (旧実装は初期化冒頭で READY にしていた — 他コア affinity では途中初期化 claim になる潜在バグを同時に修正)。ASYNC_CALL ABI: r1=[6:0]obj / [9:8]affinity / [10]budget0 |
| **tx_srcs 排出表のクロスコア安全化** | BLE_UART_DRIVER.cpp | register_tx_stream は呼び出し元コア (TELEMETRY=core0) で走り core1 の poll 走査と並行するため、挿入ソート→ **append + `__dmb` + tx_src_n publish** に変更 (index 不変)。排出側は毎回 argmax で優先度選択 (≤8 要素) |
| **svc 即値ディスパッチ (ハイブリッド ABI)** | kernel_object.cpp 冒頭 dispatch、obj_api `svci<N>(a1,a2,a3)` | svc 命令の**即値が非 0 ならそれが API 番号** (トランポリンは元々即値を r4 で渡していた)、0 なら旧来どおり r0 (YIELD=0 と動的番号用に温存)。番号は静的に決まるので新規コードは `svci` が正。引数は従来同様 r1..r3 (r0 は将来の第 4 引数用に予約)。ヘッダ内の呼び出し (obj_api/stream/call_method/export_method) は変換済み、ドライバ内の runtime 呼び出しは旧方式のまま共存可 |

**既知のクロスコアリスク (BLE on core1 実験の許容事項)**: RX sink 配送 — BLE スレッド
(core1) が受信バイトを call_method で TELEMETRY のハンドラに配送すると、その処理は
core1 上で走り、core0 の TELEMETRY 本体スレッドと内部状態 (コマンド行バッファ /
time-sync オフセット) を共有する。低頻度 (ping/コマンド) なので実験としては許容。
恒久化するなら RX もストリーム化して TELEMETRY 側で pop する形に直すこと。

**実機検証**: フラッシュして (i) [BLE_UART] が従来どおり接続・pairing・tx 統計を刻む、
(ii) [CPU] の **core0 util が ~52% → 大幅減 / core1 util が増える** (cyw43 poll の移動)、
(iii) panic-ring 沈黙、(iv) ping RTT / ctrl_lat が悪化しない、を確認。ダメなら
`SHIZU_BLE_ON_CORE1=0` で即戻せる。

## 0.7 2026-07-20 追加分その2: MPU Step1 の地均し + プロトタイプ

**実機検証結果 (2026-07-20): HW-FAIL。** トグル ON でフラッシュ後、ブート途中
(BNO055 の set_sample_sink/set_calib_sink 登録直後あたり) に HardFault
(`exc_lr=ffffffed psp=2005AB98 msp=20081FC8`) を確認。panic-ring は経由せず
(本物の HardFault)、hardfault_handler のダンプ 1 行目は出せている (完全な core
lockup ではない) が、r0-r3/pc の後続ダンプは今回未取得。**「このカーネル史上初の
CONTROL.nPRIV 切替」のリスクが的中**した形。既定ビルド (トグル OFF) へ即復旧し、
core0 util=5.7%/core1 util=52.7% (BLE-on-core1 併用) で正常動作を再確認済み。
詳細・容疑箇所・次回リトライの前提 (SWD デバッガ必須) は
[[step1-unprivileged-flight-controller]] (memory) 参照。**トグルは既定 0 のまま
リポジトリにコミット**しておくこと。

前セッションの Fable 相談 (非特権分離の設計見解) を受けて、HANDOFF §6 の「着手前に決める
べき一方向ドア」を潰す作業。3 ビルド (既定/pico2/トグル ON) とも警告ゼロでビルド確認済み。

| 機能 | 実体 | 要点 |
|---|---|---|
| **前提1: std::set 根絶** | kernel.hpp `thread_bitmap_t` | object_t::thread_table (所属スレッド集合) が書き込み専用 (insert/clear のみ、どこからも列挙されない) の std::set だったため、SVC トランポリン経由の thread mode で毎回 malloc していた。「std::map 根絶」で狙った対象と同種の見落とし。固定 128bit ビットマップに置換 (malloc ゼロ)。API 形は insert/clear を維持 (呼び出し側は無改造)。将来の MemManage→kill/再起動が「このオブジェクトの全スレッド」を引く用途を想定して test() も用意 |
| **前提2: カーネルデータのヒープ分離** | kernel.cpp `g_context_pool[128]` (直引き) + `g_kernel_arena` (call_stack フレーム用 bump allocator, malloc フォールバック付き)、`kernel_context_for()`/`kernel_arena_alloc_call_stack()` | context_t / call_stack フレームは今まで malloc (汎用ヒープ、__end__ 以降 = MPU W^X region1 の対象) から確保していた。**カーネル簿記が「不特権オブジェクトに渡す region」と同じ土俵にあった**のが Step1 の真の障害 (region の穴あけは PMSAv8 で不可)。静的プール/.bss アリーナへ移すだけで、リンカ配置変更なしに「カーネルデータは __end__ より前 = 不特権へ渡す region に最初から含まれない」を得る。アリーナは 32 スレッド分 (≈76KB) を静的確保し、想定超過時のみ malloc フォールバック (panic させない安全側)。実測: `__end__` が 91.7KB 後退、残りヒープ ≈226KB (十分な余裕) |
| **Step1 プロトタイプ: FLIGHT_CONTROLLER unprivileged 化** | kernel.hpp `SHIZU_STEP1_UNPRIV_FLIGHT_CONTROLLER` (既定 **0**)、`object_t::unprivileged`、obj_api `SET_OBJECT_UNPRIVILEGED`/`set_object_unprivileged()`、context_t.control フィールド | 「特権は current-object の属性」を実装。svc_cpp_handler の METHOD_CALL / トランポリン (一般オブジェクト→カーネルオブジェクト) が object 遷移のたびに `context->control` を書き換え、METHOD_EXIT は call_stack スナップショット (= 遷移前の値。push は書き換えより前) の丸ごと復元で自動的に戻る (明示コード不要)。svc_asm_handler.S の CTX_RESTORE が例外復帰のたびに MRS+BIC+ORR+MSR の read-modify-write で nPRIV ビットだけを適用 (SPSEL/FPCA は温存、遅延 FPU 文脈を壊さない)。FLIGHT_CONTROLLER を選んだ理由: 他オブジェクトを呼ばない「葉」オブジェクト (呼ばれる専門)。既存の region1 (W^X heap, AP=RW+unpriv許可) がそのまま使えるため、**このプロトタイプは特権遷移機構の実地検証が目的で、オブジェクト間の隔離はまだ提供しない**点に注意 (隔離は Step2 の per-subsystem region) |

**既知のリスク (実機で観察すべき点)**:
- pico-sdk の `save_and_disable_interrupts` (PRIMASK 経由の CPSID) は ARMv8-M で
  unprivileged 実行時に **NOP 化される** (フォールトせず、割り込み禁止だけがサイレントに
  効かなくなる)。FLIGHT_CONTROLLER の printf 呼び出し (main/disarm/arm) はこのリスクを
  踏むため `SHIZU_STEP1_UNPRIV_FLIGHT_CONTROLLER` トグル ON では呼ばないようガードした。
  handle_state/handle_read_control/handle_set_command 本体 (memcpy + float 演算のみ) は
  この経路を通らない。
- MemManage は個別有効化していない (HardFault へエスカレート、既存 hardfault_handler が
  ダンプ)。「MemManage→オブジェクト kill/再起動」の回復ポリシーは未実装 — このプロトタイプの
  スコープ外 (HANDOFF §6 Step1 本体の課題として残す)。

**実機検証は上記の通り HW-FAIL で完了 (2026-07-20)**。再挑戦する場合は SWD デバッガ
(picoprobe 等) でブレーク/レジスタ確認できる状態を先に用意すること — シリアルの
HardFault ダンプだけでは nPRIV 遷移バグの特定に情報が足りないと分かった。

### Pico 2 (無印) 一時テストビルド (2026-07-19: pico2_w 故障中の代替)

RP2350 チップは pico2_w と同一なので、上記 (1)(2)(3) の検証は BLE 経路を除きそのまま
できる。BLE (cyw43/btstack/BLE_UART_DRIVER) をビルドから外す:

```
cmake -S . -B build_pico2 -DSHIZU_PICO2_TEST=1
cmake --build build_pico2        # → build_pico2/main.uf2 (通常の build/ とは別)
```

- `SHIZU_NO_BLE=1` が定義され、IO_CONTROLLER は BLE_UART を起動しない
  (`[IO] SHIZU_NO_BLE build` を印字)。ble_dbg/ctrl_lat 計装は main.cpp のスタブが実体化。
- TELEMETRY はそのまま走る: BLE_UART への call_method は `BAD_OBJECT` で無害に返り
  (METHOD_CALL 型付きエラー化の恩恵)、TX ストリームは LOSSLESS 満杯 → 丸ごと破棄で
  詰まらない。センサ融合・[BEACON]/[CPU] 計装・selftest 群は全て生きる。
- ついで修正: connect selftest のストリーム ID を 1/2 → 30/31 へ (ble_tx の
  STREAM_BULK/CTRL=1/2 と衝突していた)。

---

## 0.8 2026-07-31 追加分: RT スケジューラ実機検証 + ドライバのストリーム化

### やったこと
- **ドライバ群をストリーム化** (stream.hpp): sink 方式 (set_sample_sink + CALL_METHOD) と
  手書き `spsc_ring_t` / calib サイドバンドを撤去し、オブジェクト間データ経路を Shizuku
  ストリームへ統一。追加仕様は `open_wait` (登録待ち = 起動順非依存) と ID レジストリ
  `include/driver_streams.hpp`。cmd は MP_PROD (BNO/BME の 2 producer)、calib は片方向 2 本
  (req/resp)。read_latest 等の一発 RPC は call_method のまま据え置き。
- **RT 検証ハーネスを追加**: `rt_sched_test.cpp` (`SHIZU_RT_SCHED_TEST`)、`busy_load.cpp`
  (`SHIZU_BUSY_LOAD`)、host 側 `tools/rt_sched_hosttest.py` (pyserial + bleak)。センサが
  無くても回るよう周期負荷をソフト合成し、締切ジッタを device 側で自己計測する。

### RT 試験の実機結果 (pico2_w, SHIZU_RT_SCHED_TEST=1)
never-yield ホグ (core1, 既定 budget 3ms) を BLE と同居させ、代表レートの victim が締切
ジッタを測る。**CPU ホグ競合下は PASS**:
- ongoing ジッタ (device `late(win)`, 毎 window リセット) が両コアとも **budget(3ms)程度で
  頭打ち** (core1 ~2.96–3.0ms、core0 ~0.93ms)。青天井にならない。
- hog は全 window **ADVANCING** (0 STALLED) = 凍結ウォッチドッグが取り上げ、系は凍結しない。
- **schedulable な victim (period ≥ budget) はレート維持** (100Hz 203/200、200Hz 406/400、
  25Hz 50/50)。各周期タスクが独立に「正しく時を刻む」ことを確認。
- **RT 応答の粒度 = grant budget (3ms) そのもの**。period < budget の 1kHz は 3ms ホグ下で
  毎締切を守れず ~290Hz に律速 = 設計限界 (バグではない)。>~333Hz を競合下で保証したいなら
  `SHIZU_DEFAULT_GRANT_BUDGET_US` を下げる (プリエンプト増と交換) か、そのタスクを
  budget-0 / 優先 / 専用コアにする。

### 今回の「原因」— なぜ数値が二転したか
1. **BLE 接続・ペアリング活動中は RT が繰り返し数秒スタールする (要注意)**。切り分け:
   - BLE **未接続** (advertising のみ) / CPU ホグのみ: RT 健全、ジッタ ~budget (上記 PASS)。
   - host が BLE 接続して write を撃つと、victim の **`late(win)`** (毎 window リセット = ongoing)
     が **~3.1s (core1) / ~1.9s (core0)** に跳ね、ticks が半減 (100Hz 104/200 等)。1 回きりでなく
     **BLE 活動が続く間は再発**。**BLE 切断で完全回復**。
   - 原因は **budget-0 の BLE 例外**: BLE は core1 で budget-0 (バトン、watchdog 対象外) で走り、
     cyw43/btstack の接続/ペアリング処理は長時間 unpreemptible (BLE は「3ms 以上ロックしてよい
     唯一の枠」)。→ この活動中は core1 (センサ同居) の RT 締切が数秒無保護。**接続/ペアリングは
     飛行前に済ませる運用が前提**。
2. **計測の罠**: device の `late(max)` は起動来の累積ピークで**リセットしない**ので、一過性の
   スパイクを保持し続ける (回復しても数字が消えない)。**ongoing の判定は `late(win)` (毎 window)
   で行うこと**。host ツールはこの分離をするよう修正済み。最初の誤 FAIL はこの罠が原因で、
   CPU ホグ単独の実体は PASS。
3. **定常 BLE トラフィック下の RT / ping・throughput はまだクリーンに測れていない**。RX write
   (ping `P` / blast `B`) は LE Secure Connections の numeric-comparison ペアリング (Pico シリアル
   6 桁確認 + **macOS のペアリングダイアログ = 人手クリック**) を要し、ヘッドレスからは完了でき
   ない (実測でも ping 無応答・throughput は素のテレメトリ相当のまま)。→ 上記 (1) の「接続中
   スタール」は**ペアリング交渉を含む活動**で、「**ペアリング完了後**の定常 notify トラフィックが
   RT を崩すか」とは切り分けられていない。後者を測るには GUI クライアント (対話ペアリング対応)
   を RT フラグ build に対して回すこと。

### 関連メモリ
`memory/rt-sched-hw-verified.md` (実測値の詳細)、`memory/object-memory-arena-vs-system-object.md`
(次の保護レイヤ: per-object アリーナ + DMA 越境 + System Object)。

---

## 0.9 2026-08-10 追加分: BLE 接続時フリーズの実測原因 (cyw43 ではなく printf) + SHIZUKU_USB + ログ経路の外出し

### 結論: 「BLE を接続するとフリーズする」の原因は printf だった
実機 A/B (pico2_w, RT_SCHED_TEST 併用, BLE 接続/切断を各 n=3, シリアル非接続で計測):

| 構成 | 単発 `cyw43_arch_poll` 最大 | RT `late(max)` |
|---|---|---|
| Stage1 + stdio 既定 | 1.006 / 1.006 / 1.005 s | 1.008 / 1.008 / 1.031 s |
| Stage1 + stdio 非ブロック | 0.038 / 0.038 / 0.038 s | 0.056 / 0.046 / 0.046 s |

値が常に **~1.005s と一定**なのが手がかりで、これは `PICO_STDIO_DEADLOCK_TIMEOUT_MS`=1000 そのもの。
cyw43 SDK の `do_ioctl timeout` / `STALL` / `could not bring bus up` 警告は**全ログで一度も
出ていない** = 無線側は正常だった。ドライバの printf の大半は btstack コールバック =
`cyw43_arch_poll()` の**中**で出るため、これが「1 秒の LONG poll」に化けて cyw43 のせいだと
誤認していた。

### 待ちは 2 段ある (片方だけでは直らない)
- **(外)** `pico_stdio` の `print_mutex` … `mutex_enter_block_until` は **WFE スピン**で
  `PICO_STDIO_DEADLOCK_TIMEOUT_MS` (既定 1000ms) までコアを握る。**Shizuku の yield を
  通らないのでスケジューラから手が出せない**。実測の 1.0s/2.0s はこれ。
- **(内)** `pico_stdio_usb` の `out_chars` … CDC 満杯で `PICO_STDIO_USB_STDOUT_TIMEOUT_US`
  (既定 500ms) まで `tud_task()` をビジー回し。

`printf` は linker `--wrap` で pico_stdio に固定されており外側を差し替える口が無いので、
外側は待ち時間そのものを 2ms へ詰めて潰す。**リングと組にして初めて安全** (リングのおかげで
mutex 保持が memcpy 数µs になり 2ms タイムアウトはまず発火しない = 出力を落とさない)。
リングだけ入れた中間状態は **1.015s のまま**だったので、両輪を必ず一組で有効化すること
→ 有効化は CMake の `option(SHIZU_USB_DRIVER ... ON)` 1 箇所に集約し、`kernel.hpp` の
`#ifndef` 既定は 0 のままにしてある。

### 実装 (SHIZUKU_USB, object id 7)
| 機能 | 実体 | 要点 |
|---|---|---|
| **非ブロッキング printf** | `SHIZUKU_USB.cpp` / `include/object_headers/SHIZUKU_USB.hpp` | 自前 `stdio_driver_t` がリング (8KB, .bss) へ積んで即 return。排出スレッドが `tud_cdc_write_available()` の**空き分だけ**を SDK の `stdio_usb.out_chars` へ渡すのでブロッキング分岐に構造的に入らない。tinyusb は据え置き (`tud_task()` は SDK の低優先度 IRQ が回すので干渉しない) |
| **USB デバイス名の固有化** | CMakeLists `USBD_MANUFACTURER`/`USBD_PRODUCT` | ホストから product `Shizuku USB` / manufacturer `Shizuku` で見える。**VID/PID (0x2E8A:0x0009) は変えないこと** — `picotool` の reset-via-baud がこれで探すので `load -f` が壊れる。ホスト側ツールは製品名でポートを特定できる |
| **ログ経路の外出し** | `include/log.hpp` / `log.cpp` / `IO_CONTROLLER.cpp` | `sink::{USB, BLE, PRINTK, NONE}`。**ドライバは出力先を知らないし選べない** (呼び出し元の識別は `get_current_obj_id()` = SVC なので名乗って迂回できない)。割り当ては合成側 (IO_CONTROLLER) の 1 箇所だけ。affinity/budget を `async_call` 側が決めるのと同じ「機構はオブジェクト、方針は外」 |

**シンクの保証はそれぞれ違う** (用途で選ぶこと):

| sink | 待つか | スケジューラ起動前 | 溢れたら |
|---|---|---|---|
| `USB` | 待たない | 溜めて後から排出 | 新しい方を破棄 |
| `BLE` | 待たない | 不可 (接続が要る) | メッセージ破棄 |
| `PRINTK` | **待つ** | 可 | 待って出す |
| `NONE` | — | — | 捨てる (整形ごと省略) |

`log::sink::USB` は `shizuku_usb_push()` でリングへ直接積むので **pico_stdio を通らない** =
`print_mutex` を構造的に踏まない。生 `printf` より速く RT に安全。

### 現在の割り当て (IO_CONTROLLER)
全オブジェクト → `USB`、`KERNEL_OBJECT` だけ `PRINTK`。カーネル自身の声は「排出スレッドが
生きていること」を前提にできない (その排出スレッドをスケジュールしているのがカーネル) ため。
**ただし現状このエントリは不発** — `kernel.cpp`/`kernel_object.cpp`/`main.cpp`/`core1_boot.cpp`/
`core1_io.cpp` は意図的に生 `printf` のまま (それでもリング経由で非ブロッキング)。理由:
`log::printf` は `get_current_obj_id()` = SVC を撃つので、**IRQ 文脈 / SVC ハンドラ内 /
Shizuku 起動前 / SENSOR_IO (svc は sleep_us のみ許可) では使えない**。カーネル側が採用する
なら SVC を撃たない `log::printf_to(sink::PRINTK, ...)` を使うこと。

### 実機検証 (すべて pico2_w)
- **凍結解消**: シリアル接続状態 3/3 で単発 poll 0.038〜0.039s / RT `late(max)` 0.046〜0.047s /
  1s 超ゼロ (修正前 1.006〜2.009s)。**ホストがポートを開いたまま読まない**最も厳しい条件でも
  2/2 で 0.039〜0.041s (修正前 1.011s)。既定ビルド (フラグ無し) でも 0.038s。
- **出力欠落 0 バイト** (`[USB] dropped` 未発火)。ログ行数はむしろ 342 → 397 行に増加
  (従来はタイムアウト経路で printf が捨てられていた)。
- **ログ経路の外出し**: ドライバのソースは同一のまま、合成側の割り当てを変えるだけで
  `[BNO055]` の周期ログが 0 行 (`NONE`) / 20 行 (`USB`) / 20 行 (`PRINTK`) と切り替わることを確認。
- **全ドライバ移行後**も healthy (`RT late(max)`=0.004s、`[CALL]`/`[PANIC-RING]` 沈黙)。

### Stage 2 (独自 host-wake IRQ + cyw43 サービススレッド) について
`SHIZU_CYW43_SVC_THREAD` は実装済み・実機で host-wake IRQ が動くことも確認したが、
**この問題は解かない** (秒級ストールが残り、1 試行では 4.0s の RT フリーズと悪化)。
既定 OFF のまま残してある。原因が printf だったので、cyw43 側の対処は不要だった。

### 未検証で残っているもの
- **リング溢れ (drop) 経路**: macOS が USB エンドポイントを吸い続けるためホスト側から
  CDC を満杯にできず、`[USB] dropped` を一度も発火させられていない。起動時に 32KB を
  一気に吐く自己テストを足せば潰せる。
- **`sink::BLE`**: 母艦接続 + 認可が要るので未確認 (経路自体は既存の互換 `send_buf` と同じ)。
  BLE_UART の TX ストリームは SPSC なので、複数オブジェクトが同時に流すと壊れる →
  非ブロッキングなトークンで直列化し、取れなければ**その行を捨てる**実装にしてある。

### 検証ツール (ホスト側、スクラッチパッド)
`trial.py` (リブート→シリアル捕捉→BLE 接続/切断→指標抽出)、`trial_stalled.py`
(ホストが読まない条件)。ポートは製品名 `Shizuku*` で自動発見する。判定は
`poll ever` (単発 poll の起動来最大) と RT `late(max)`、および 1s 超の窓数。

## 0.10 2026-08-17 追加分: XIP (フラッシュ QSPI) クロックを clk_sys から導出

**実測まとめは `docs/XIP_FLASH_CLOCK.md`。要点だけ:**

* **`PICO_FLASH_SPI_CLKDIV` は死んだつまみだった。** SDK 2.2 の RP2350 既定ビルドに
  boot2 は入っておらず (`main.elf` に `.boot2` 無し)、QMI の `M0_TIMING` は
  **ブート ROM の固定値 (div=3 / rx=2)** のまま。§0.8 以前の「300MHz は div=4 にして
  復旧した」という記述は誤りで、効いていたのは `CYW43_PIO_CLOCK_DIV_INT=4` だけ。
* 分周比が固定 = **フラッシュ速度が clk_sys に比例して勝手に動いていた**。
  150MHz ビルド → 50MHz (遅すぎ) / 300MHz ビルド → 100MHz (偶然当たっていた)。
  CYW43 PIO と同じ「固定分周の連鎖」で、向きだけが逆 (黙って定格超過側へ倒れる)。
* **対処**: `flash_clock.cpp` / `include/flash_clock.hpp`。目標周波数
  `SHIZU_FLASH_TARGET_KHZ` (既定 100000) を宣言し、`div = ceil(clk_sys / target)` を
  実行時に導出する。1 つの値で 150MHz→75MHz / 300MHz→100MHz と両方正しくなる。
  `set_sys_clock_khz` の**前**にも「上げた後の clk_sys」で一度適用する
  (先に分周比を上げておかないと、クロックが上がった瞬間だけ定格超過になる)。
  `[CLK] sys=... peri=... flash=...` で実測を必ず印字。
* **ベンチ**: `cmake -DSHIZU_XIP_BENCH=ON` → `[XIP]` スイープ (`xip_bench.cpp`)。
  16KB の XIP キャッシュに載らないよう 64KB に散らした 512 関数をランダムに呼んで
  命令フェッチのストールを測り、同時に rot-xor チェックサムで**読み違いを検出**する
  (速すぎる設定は落ちずに静かに誤読する)。`tools/xip_bench_check.py` で main.bin
  から再計算して突き合わせること — ファーム内比較だけでは基準自体の破損を見逃す。
  危険な設定も RAM 常駐コードで「データ照合が通ってから命令を踏む」順に試すので、
  定格外まで 1 ブートで舐めても BOOTSEL 送りにならない。
* **効き**: 300MHz で 50→100MHz にして命令フェッチ 1144→628 ns/call (-45%)、
  系全体では core0 util 6.15%→5.70% (-7%)。ホットパスの大半はキャッシュに載って
  いるので系全体の伸びは小さい。core1 (BLE poll) はポーリング周期律速で不変。
* **★2026-08-17 追記: 300MHz + フラッシュ 100MHz を既定にした。** 引数なしの `cmake` で
  この構成。戻すには `-DSHIZU_SYS_CLK_KHZ=0` (クロック非変更 = 150MHz)。
  同時に **CYW43 の PIO SPI 分周も導出化** (`SHIZU_CYW43_SPI_TARGET_KHZ`=75000 から
  `ceil`)。従来の `4` 直書きは 300MHz でだけ正しく、`-DSHIZU_SYS_CLK_KHZ=150000` を
  明示すると SPI が半速になる潜在バグだった。WS2812 は元から clk_sys 由来 (`ws2812.pio:42`)、
  I2C/PWM は SDK 算出、clk_peri も **PLL_SYS から分周して 150MHz** に
  引き上げた (`SHIZU_PERI_TARGET_KHZ`、既定 150000)。`set_sys_clock_khz` は clk_peri を
  PLL_USB 48MHz へ落とし、それだと **SPI 上限が clk_peri/2 = 24MHz** になるため。
  RP2350 の clk_peri は分周器付き (2bit) なので **CPU 300MHz のまま周辺 150MHz** が可能で、
  150MHz は定格 sys と同値 = 周辺としても定格内 (SPI 上限 75MHz)。
  → clk_sys 依存の固定分周はこれで全滅。
* **RXDELAY は窓の全域 (0..7) を測って中心に置く。** 100MHz の窓は実測 **rx=1..6**。
  0..4 だけ測った段階では rx=2 が妥当に見えたが、全域では下端の隣だった。
  端を知らずにマージンを語ってはいけない。
  さらに**段数を直書きすると「そのクロックでだけ正しい値」になる** (150MHz では窓が
  0..3 なので 3 は上端)。→ `SHIZU_FLASH_RXDELAY_PS` (既定 5000) で**遅延 [ps] を宣言**し
  `ps×2×clk_sys` で段数を導出する形に統一。300MHz→3 / 150MHz→2。
  **コア電圧も `SHIZU_VREG_MV` (既定 1200) に外出し**。`[CLK]` 行が実効値を全部印字する:
  `sys=300000000 peri=48000000 flash=100000000 rx=3 (5000 ps) vreg=1200 mV`
* **ダイ温度を常時監視**: `[BEACON] ... die=39.3C` (`main.cpp` の `die_temp_x10`、
  ADC 最終チャネル)。300MHz / vreg 1.20V / 室温・机上で 39℃ = 余裕あり。
  触感は根拠にならないので数字で残す。
* **CPU クロックの天井 (実測)**: **1.20V で 372MHz / 1.30V で 432MHz**、その上は
  ハング。**600MHz は不可。** 既定 300MHz の余裕は 24% (室温・core0 単独)。
  熱は律速しない (全負荷 120s で 40.2℃ 飽和)。★**1.30V の天井を常用余裕の根拠に
  しないこと** — 出荷電圧で測ると 60MHz 低い。探索は `-DSHIZU_OC_PROBE=ON`
  (`oc_probe.cpp`): WDT + noinit フラグで固まっても自動復帰し、登り直さないので
  **BOOTSEL 押しゼロ**。PLL は VCO=clk_sys×postdiv が 12MHz 整数倍かつ 750-1600MHz
  でないと作れない (350/440/550MHz は不可)。詳細は `docs/XIP_FLASH_CLOCK.md`。
* **注意**: フラッシュ書き込み (BTstack のペアリング鍵保存) の後は ROM の
  `flash_enter_cmd_xip()` で **ROM 既定へ戻る** (壊れないが黙って遅くなる)。
  ビーコンが `[FLASH] timing drifted` で報告する。再設定するなら core1 が止まって
  いる `flash_safe_execute` の中で。

---

## 0. 一言でいうと

RP2350 (Cortex-M33 デュアルコア) 上の自作協調型マイクロカーネル **Shizuku** と、その上の
**BLE テレメトリ (飛行/ロボコン)** アプリ。センサ (BME280/BNO055) を core1 で読み、core0 で
融合して BLE UART (Nordic UART Service) で母艦へ送る。今セッションで「協調ランタイム」から
「プリエンプション + SMP + 保護に向かうカーネル」へ大きく前進した。

---

## 1. アーキテクチャの核 (5 行で)

- **オブジェクト + スレッド**が基本単位。オブジェクト境界は **SVC 例外**をトラップ口にする。
- ディスパッチは **svc 番号でなく「発行元オブジェクトの種別」**で決まる: カーネルオブジェクト
  (id=0) はプリミティブを直接実行、一般オブジェクトは登録ハンドラ (kernel_obj_svc_handler) へ
  トランポリン (= ハンドラをスレッドモードで実行)。
- メソッド呼び出し = 同一スレッド上で pc + current-object-id を張り替える保護サブルーチン。
- スケジューリング**方針**はカーネルでなくカーネルオブジェクトの `sched_pick_next` が持つ。
- MMU 未使用の単一アドレス空間。オブジェクト間データは「メソッド + ポインタ渡し」か
  lock-free stream (SPSC)。← MPU 導入でここが変わる (§6)。

---

## 2. このセッションで実装したもの (すべてビルド済み)

| 機能 | 実体 | 状態 |
|---|---|---|
| **run_for / GRANT_CPU** 時限実行権移譲 | kernel.cpp, svc_asm_handler.S (PendSV), obj_api.hpp | 実機 ALL PASS |
| **budget スケジューラ** (3ms 凍結ウォッチドッグ) | kernel_object.cpp `sched_pick_next` | 実機動作 |
| **BLE TX 優先度マルチストリーム** (ping HoL 解消) | ble_tx_stream.hpp, BLE_UART_DRIVER.cpp, TELEMETRY_SENDER.cpp | 実機動作。ctrl_lat≈4ms |
| **core1 の Shizuku 化** (ベアメタル→協調スレッド) | core1_boot.cpp, core1_io.cpp | 実機 panic 解消済み |
| **共通 scheduler_idle_loop** (両コア 1 実装) | kernel_object.cpp | 実機動作 |
| **panic noinit RAM リング** (core1/例外文脈でも panic 可視) | panic_ring.hpp, kernel.cpp, main.cpp | 実機沈黙 (=panic ゼロ) |
| **std::map 根絶** (SVC 経路の malloc ゼロ) | kernel_object.cpp `md_table`/`obj_mem` | 実機動作 |
| **SMP ストレス** (2 コア SVC 同時進入 + 跨コア移動) | smp_stress.cpp | 実機 **ADVANCING (PASS)** |
| **MPU Step0: PSPLIM** (スタックオーバーフロー即検出) | kernel.hpp, svc_asm_handler.S, kernel.cpp, shizu.hpp, core1_boot.cpp | ビルド済み・**実機未検証** |

### 特に重要な設計ポイント / ハマりどころ
- **core1 の idle は必ず object 0 (カーネルオブジェクト) のスレッドにする**。一般オブジェクトの
  スレッドから `scheduler_idle_loop` (生 svc 発行) を呼ぶとトランポリンへ誤ルーティングされ、
  r0(切替先 tid) が obj_api::svc_num として誤解釈されて panic する (実障害履歴)。
- **SENSOR_IO (core1) は svc を `sleep_us` のみ許可**。printf/float 禁止、std::map 系 API 禁止、
  I2C バースト中は sleep しない (core1_io.cpp 冒頭コメント参照)。
- **grant scheduler の budget 0** = 無制限バトン組 = {thread0, BLE_UART, core1 idle, SENSOR_IO}。
  これらだけが凍結面。他は 3ms で必ず回収される。

---

## 3. ビルド / 検証

### 環境変数 (CLI ビルドに必須)
```
export PICO_SDK_PATH=/Users/ishigakiyua/.pico-sdk/sdk/2.2.0
export PICO_TOOLCHAIN_PATH=/Users/ishigakiyua/.pico-sdk/toolchain/14_2_Rel1
export PATH=$PICO_TOOLCHAIN_PATH/bin:$PATH
cmake --build build -j8        # build/main.uf2
```
(VS Code の Pico 拡張なら CMakeCache に値が入っているので env 不要。CLI だと
`/external/pico_sdk_import.cmake` で失敗するので上記 export が要る。)

### トグル (include/kernel.hpp)
- `SHIZU_GRANT_SELFTEST` — run_for の A–E 自己テスト (GRANT_TEST obj)。core0 が yield-churn で
  util≈98% に見えるのは selftest ワーカの副作用 (実運用値でない)。
- `SHIZU_SMP_STRESS` — 2 コア SVC ストレス (SMP_STRESS obj)。`[SMPSTRESS] ... ADVANCING` +
  panic-ring 沈黙で 2 コア安全性の合格判定。CXXFLAGS で有効化してビルド:
  `CXXFLAGS="-DSHIZU_SMP_STRESS=1 -DSHIZU_GRANT_SELFTEST=0" cmake -S . -B <dir> ...`
- `SHIZU_STREAM_SELFTEST` — stream API 自己テスト。

### 実機ログの読み方
- `[CPU] coreN util=..%` — スケジューラが実務へ渡した時間 (5s 窓)。**注意**: 高頻度スレッド多数
  だと O(128) スキャン込みで overhead が乗る。selftest=1 の 98% は churn 由来。
- `[PANIC-RING] ...` — panic が出たら boot 時 (前回残骸) とビーコンが印字。**沈黙=panic ゼロ**。
- `[BLE_UART] ctrl_lat 1s: ...` — ping 応答の device 内滞在時間。≈4ms。
- `[SMPSTRESS] c0=.. c1=.. mig=.. ADVANCING` — SMP ストレスのカウンタ。

### PSPLIM の実機検証 (次にやる)
フラッシュして panic-ring が沈黙 + 通常動作すれば PSPLIM 導入 OK。スタックオーバーフローを
わざと起こす (深い再帰など) と HardFault で即停止するはず。

---

## 4. 既知の事実 / 未解決

- **ping RTT >100ms は host 側 (macOS CoreBluetooth + Python GUI)**。device 内 ctrl_lat≈4ms で
  確定。write-without-response のバッチング + notification 配送 + asyncio/tkinter。device を
  いじっても縮まない。別 central (スマホ) で <30ms なら完全確定。
- **core0 util はシナリオ依存**。BLE 接続中は cyw43_arch_poll (HCI 処理) で ~52%、未接続なら安い。
  poll モードの固有コスト。減らすなら poll cadence or threadsafe_background(IRQ) 化 (別最適化)。
- **`sched_pick_next` は O(128) 線形スキャン**。1 grant/sleep 往復 ≈45µs。実アプリ (ms+ 周期)
  では桁で小さいが、高頻度スレッドが増えたら runnable ビットマスク化で潰す (将来メモ)。
- **センサ未接続だと core1 util=0 / reads=0** は正常 (BNO init が chip-ID で失敗)。

---

## 5. 次の一手 (優先順)

1. **PSPLIM の実機確認** (フラッシュして panic 沈黙 + 通常動作)。← 直近
2. **アプリの core1 化 (負荷分散)**。SMP 合格済みなので FLIGHT_CONTROLLER 等を affinity=CORE1/ALL
   に。BLE は core0 固定 (cyw43/btstack がコア縛り)。
3. **MPU Step1** (§6)。特権分離 + ドライバ専用ペリフェラル region + 全 SVC の TT bounds-check。
4. **slack-aware lending** (core1 の余剰を core0 発 budget スレッドへガード付き貸出)。

---

## 6. MPU / 保護ロードマップ (Fable 設計レビュー反映)

**段階計画 (各段で価値が出るので途中で止めても損しない。TF-M/PSA の L1→L2→L3 に対応):**

| 段 | 内容 | IPC/ドライバへの影響 |
|---|---|---|
| **0** | **PSPLIM (実装済み)** + W^X (RAM XN / Flash RO の MPU 属性 region) | 無傷 |
| **1** | カーネル privileged / オブジェクト unprivileged の 2値分離。PRIVDEFENA=1 でカーネルは default map、8 region 全部 unprivileged 用。静的 region (Flash RX / 共有 SRAM RW) + 動的 (ドライバの担当ペリフェラルブロック)。**全 SVC の TT/TTT 命令によるポインタ bounds-check** | ポインタ IPC 無傷 (SRAM 全員 RW 共有)。カーネルデータ/ペリフェラルは保護される |
| **2** | per-subsystem memory domain (Zephyr memory domain 同型)。System/Subsystem 階層に接続 | メソッド引数は**カーネル媒介 bounded copy** へ (callee コピーをカーネルに移すだけでコピー回数不変)。stream データはエンドポイント 2 者の region に |

**確定した設計判断 (Fable):**
- **TrustZone は使わない**。全部 Secure のまま MPU + **ACCESSCTRL** (RP2350 のバスファブリック
  per-peripheral ゲート) で行く。目標は「バグ封じ込め」で悪意防御でない。
- **DMA は MPU を素通りする** → DMA 設定はカーネル専有、ACCESSCTRL で裏取り。
- ドライバは「コントロールプレーン=privileged(SVC) / データプレーン=direct-map unprivileged」。
  direct: 所有インスタンスの FIFO/data レジスタ (性能)。SVC: RESETS/CLOCKS/pinmux/IRQ/DMA。
- **PMSAv7 の知識は捨てる**: v8 は有効 region の overlap 禁止、サブリージョン無効化なし。
  「広い RW + 穴あけ」不可 → **リンカ配置**で解く。base/limit 32B 粒度。
- cyw43+btstack は「独自 subsystem、当面準特権」で割り切る。隔離投資は自作ドライバから。
- MPU 再プログラムは動的 4〜6 region で ~50〜100 サイクル (<0.5µs) = 1kHz/3ms grant に誤差。
  恐れるのは cycles でなく **region 予算 (8 本)** と **コアごと MPU 独立**。
- **脆弱性は region テーブルでなく SVC ABI に住む** (FreeRTOS-MPU の CVE 教訓)。bounds-check は
  最初から。MemManage → オブジェクト kill/再起動の**回復ポリシー**も必須 (I2C バス回復と接続)。

**着手前に決めるべき一方向ドア (これが真のリスク):**
1. **subsystem 別メモリ配置** (リンカセクション / 専用アロケータ)。ドメイン隔離は「データが
   アドレス連続」を要求。共有ヒープ interleave だと region で括れない。MPU レジスタを触る前に決める。
2. **SVC ABI の bounds-check 規律を後付けにしない**。

**プロトタイプで潰す最小の一手 (Step1 の前哨):** core1 の I2C センサスレッド 1 本だけ
unprivileged 化 (Flash RX + 共有 SRAM + I2C block region + PSPLIM + nPRIV)。これで (i) 本当の
SVC ABI 表面、(ii) 1kHz 影響、(iii) MemManage→再起動、(iv) unpriv Thread への EXC_RETURN/CONTROL
復帰の正しさ、が一度に出る。

---

## 7. 上位構想: 階層リソース管理 + ユーザー要望

- **System Object (fair) + Subsystem Object (unfair)** の階層。System は隔離/相互不干渉を保証、
  Subsystem はドメイン固有ポリシー (ドライバ群 / アプリ群を別ドメイン)。ARINC-653 相当。
- **リソース種別ごとに別の木**: CPU 木 (run_for/grant_budget = 実装済みの enforce) と I/O 木
  (BLE TX の優先度 stream = 実装済みの最初の葉) は別軸。混ぜない。CAN/GATT-2 も I/O 木の
  transport 葉。stream が uniform interface。
- **残る新プリミティブ = オブジェクトのグルーピング/所有** (今は 0..127 フラット)。Subsystem =
  {メンバー集合, 親 budget, リソース別ポリシー}。将来 MPU 保護ドメインの単位。
- **ユーザー要望 (未実装、Step2 に落ちる)**:
  - オブジェクトランドの syscall ハンドリングをカーネル空間オブジェクトへ開放 (= 複数ハンドラ
    dispatch。サブシステムが自分の syscall を捌く sub-kernel 化)。
  - MPU 割り付け API の開放 (サブシステムが自ドメイン region を確保)。
  - → どちらも MPU の特権モデル + メモリ配置トポロジが前提。単独で ABI を切ると保護の穴になる。

---

## 8. ファイル地図

- **カーネル中核**: kernel.cpp (SVC/PendSV/SysTick, grant, panic-ring, cpu_busy), kernel.hpp,
  kernel_object.cpp (sched_pick_next, scheduler_idle_loop, kernel_obj_svc_handler, md/mem 表),
  svc_asm_handler.S (context save/restore, PendSV, PSPLIM), shizu.hpp (init, thread0 bootstrap)
- **core1**: core1_boot.cpp (core1 起動 + idle), core1_io.cpp (センサ I/O ループ), core_ring.hpp
- **BLE**: BLE_UART_DRIVER.cpp/.hpp, ble_tx_stream.hpp, ble_uart.gatt
- **アプリ**: TELEMETRY_SENDER.cpp, FLIGHT_CONTROLLER.cpp, BME280/BNO055_DRIVER.cpp
- **API/抽象**: obj_api.hpp (svc ラッパ, run_for, sleep_us), stream.hpp (SPSC), call_method.hpp,
  svc.hpp (型付きエラー), object_id.hpp (オブジェクト番号)
- **計装/テスト**: panic_ring.hpp, smp_stress.cpp, grant_selftest.cpp, stream_selftest.cpp
- **設計メモ**: memory/*.md (MEMORY.md が索引)

---

## 9. Fable (サブエージェント) 設計レビューの所在

このセッションで Fable に投げた設計相談 4 件 (階層リソース管理 / core1 専有の必然性 /
svc-from-core1 根因 + core1-safe / MPU 保護設計) の生出力は会話ログにある。要点は本資料
§6・§7 と各コミットメッセージに反映済み。再相談は同型プロンプトで。
