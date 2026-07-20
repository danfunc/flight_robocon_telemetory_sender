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
