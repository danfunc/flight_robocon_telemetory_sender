# Shizuku / flight_robocon_telemetry — 引き継ぎ資料

最終更新: 2026-07-17。このセッションで実装した内容・現在地・ロードマップ・ビルド/検証手順・
未決定の一方向ドアをまとめる。コード内の詳細コメントと `memory/` の設計メモも併読のこと。

## 0.5 2026-07-17 追加分 (ビルド済み・実機未検証)

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
