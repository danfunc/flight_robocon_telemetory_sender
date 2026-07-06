# センサーストリームプロトコル + core1 I/O 分離 設計書 (draft v0.1)

目的: 生センサ値 (accel 1kHz / gyro 500Hz / mag・baro 20Hz) を PC へダウンリンクし、
PC 側の大規模 EKF の入力にする。同一ストリームを機内 EKF も消費する。
BLE 実効帯域の実測上限 **125 kbps (≈15.6 kB/s)** を設計制約とする。

構成は 2 層:
1. **ワイヤプロトコル** — BLE UART (NUS notification) 上のバイナリフレーム
2. **コア間リング** — core1 (ベアメタル I/O コア) → core0 (Shizuku) の SPSC リングバッファ

両層で同じチャンネル定義/サンプル形式を共有する。

---

## 1. チャンネル定義とレート

| ch_id | 名前   | レート    | ペイロード/サンプル                  | B/s   |
|-------|--------|-----------|--------------------------------------|-------|
| 0x01  | ACC    | 1000 Hz   | 3×int16 (raw LSB)              = 6B  | 6,000 |
| 0x02  | GYR    | 500 Hz(*) | 3×int16 (raw LSB)              = 6B  | 3,000 |
| 0x03  | MAG    | 20 Hz     | 3×int16 (raw LSB)              = 6B  | 120   |
| 0x04  | BARO   | 20 Hz     | press_pa:u32 + temp_cC:i16     = 6B  | 120   |
| 0x10  | FUSED  | 可変      | 機内融合出力 (姿勢/高度/速度)。後日定義 | —   |
| 0x7E  | STATUS | 1 Hz      | drop 数/calib/I2C fail/heartbeat     | ~30   |
| 0x7F  | CONFIG | 変更時    | スケール・ODR・divider のエコー       | —     |

(*) BNO055 の gyro ODR 上限は 523Hz (512 は選べない)。ポーリングは 500Hz の
綺麗なグリッドで行い、2 冪レートが欲しければ PC 側でリサンプルする。

生値はセンサの raw LSB のまま送る (換算は PC 側)。スケール係数
(ACC: 100 LSB/(m/s²) 等、AMG モードのレンジ設定に依存) は CONFIG チャンネルで
接続時と変更時に通知する。EKF のノイズパラメータがレンジに依存するため必須。

### 帯域収支

ペイロード計 ≈ 9.3 kB/s。フレーミングオーバヘッド (~1.3 kB/s) 込みで
**≈10.6 kB/s = 85 kbps → 実測上限 125 kbps の ~68%、ヘッドルーム 32%**。

RSSI 劣化で帯域が細った場合はチャンネルごとのダウンリンク divider (§4) を上げて
縮退する (例: ACC /4 = 250Hz 送信)。機内 EKF は常にフルレートを消費するので、
最悪「ダウンリンクほぼ停止・機内 EKF 専用」までプロトコル変更なしで落とせる。

---

## 2. ワイヤプロトコル (BLE ダウンリンク)

制約: **1 notification = 1 パケット ≤244B (ATT_MTU 247 − 3)。1 LL PDU に収める**
(HCI_ACL_PAYLOAD_SIZE 247+4。これを超える ACL は CYW43 の BT コアが再接続サイクルで
ウェッジする実測既知問題があるため厳守)。

```
パケット:
  magic      : u8   = 0xA7
  ver_flags  : u8   (上位4bit=version(0), 下位4bit=flags)
  seq        : u16  (パケット通番。欠落検知用)
  base_time  : u32  (デバイス時刻 µs の下位32bit)
  チャンク × N (パケット末尾まで):
    ch_id    : u8
    count    : u8   (このチャンクのサンプル数)
    t_offset : u16  (先頭サンプル時刻の base_time からのオフセット µs)
    samples  : count × チャンネル固有サイズ
```

- サンプル i の時刻 = `base_time + t_offset + i × period(ch)`。
  period は CONFIG で通知される公称周期。**per-sample タイムスタンプは送らない**
  (これで per-sample 2〜4B を節約し §1 の収支が成立する)。
  core1 のサンプリングが絶対グリッド (next += period) なので暗黙時刻が成立する。
  グリッドを外れたサンプル (I2C リトライ等) はチャンクを分割して t_offset を打ち直す。
- u32 時刻のラップ (~71 分) は PC 側で単調拡張する。
- 欠落検知: seq のギャップ + STATUS の drop カウンタ。BLE notification は LL 再送で
  順序保証されるが、切断と送信側リング溢れでは欠落する。
- 既存のバイナリバッチ送信 (TELEMETRY_SENDER の flush 機構) の器をそのまま使い、
  フレーム形式のみ本形式へ置き換える。

## 3. アップリンク (コマンド)

既存 rx_sink 経路のテキストコマンドを拡張:

| コマンド | 意味 |
|---|---|
| `D <ch> <div>` | ダウンリンク divider 設定 (1=フル, 0=停止) |
| `Z` | baro ゼロ点再較正 (既存 rezero。core1 へコマンドリング経由で転送) |
| `CS` / `CL <hex>` | BNO055 較正 save/load (既存。core1 経由に変更) |
| `M <mode>` | BNO055 モード切替 (NDOF / AMG) |

## 4. コア間リング (core1 → core0)

```
レコード (12B 固定):
  ch_id  : u8
  flags  : u8   (bit0: グリッド外サンプル, bit1: 直前に欠落あり)
  t_us   : u32  (time_us_64() 下位 32bit。両コア共通のタイマなので直接比較可)
  payload: 6B   (チャンネル固有。BARO は u32+i16 で丁度 6B)
```

- **SPSC**: core1 が唯一の producer、core0 (TELEMETRY_SENDER スレッド) が唯一の
  consumer。インデックスは u32 モノトニック、`wr` の公開前に `__dmb()`。
  RP2350 の SRAM はデータキャッシュ無しなので volatile + DMB で足りる。
- サイズ: 8 KB (682 レコード ≈ 定常 1.52k レコード/s の ~450ms 分)。BLE 切断中も
  機内 EKF は消費し続けるので溢れは通常発生しない。溢れたら古い方を捨てて
  flags.bit1 とドロップカウンタに記録。
- 逆方向 (core0 → core1) は小さなコマンドリング (較正・rezero・モード切替)。
  レコードは 8B 固定 `{op:u8, arg:u8, rsv:u16, arg32:u32}`、16 エントリ。
  op: 1=SET_PAUSED_BNO / 2=SET_PAUSED_BME / 3=SET_READ_MODE /
  4=SET_FFFF_REJECT / 5=REZERO。

### 4.1 step2 (NDOF 経過措置) の一時チャンネル

step2 では現行の NDOF フュージョン出力をそのまま core1 化するため、§1 の
raw ACC/GYR/MAG の代わりに以下の一時チャンネルを流す (AMG 化 = step4 で置換):

| ch_id | 名前   | レート  | ペイロード (6B)                                   |
|-------|--------|---------|---------------------------------------------------|
| 0x11  | EUL    | 100 Hz  | h/r/p 3×int16 (1/16 deg, ±1LSB デッドバンド適用済) |
| 0x12  | LIA    | 100 Hz  | 線形加速度 x/y/z 3×int16 (1/100 m/s²)              |
| 0x13  | GRV    | 100 Hz  | 重力ベクトル x/y/z 3×int16 (1/100 m/s²)            |
| 0x04  | BARO   | ~21 Hz  | press_pa:u32 LE + temp:i16 (0.01℃, core1 整数補償済) |
| 0x05  | GROUND | 較正時  | BARO と同形式。地上基準 (起動時 40 平均 / rezero 20 平均) |
| 0x7E  | STATUS | 1 Hz    | [0]=calib byte, [1]=health(bit0 bno_ok/bit1 bme_ok/bit2 bno_paused/bit3 bme_paused), [2:3]=i2c_fail u16, [4]=recover回数, [5]=reinit回数 |

- EUL/LIA/GRV は同一 t_us の 3 レコードで 1 モーションサンプル。core0 の
  排出側が t_us で組にして bno055_sample_t へ再構成する (float 換算
  /16, /100 は core0)。
- BNO055 の 0xFFFF 破損据え置き・euler デッドバンド・fail_streak →
  recover → 再init のエスカレーションは core1 内で完結する。
- calib ステータス byte の I2C 読み (1B 独立トランザクション) は 1Hz の
  STATUS 生成時にのみ行う (従来の 100Hz 読みから削減)。

### 4.2 較正プロファイルのサイドバンド (ストリーム外)

BNO055 較正オフセットプロファイル (22B、レア) はリングに乗せず、専用の
共有 static 構造体 + seq/ack + DMB で転送する:

```
calib_sideband_t:
  req_seq : u32 (core0 が書く)
  ack_seq : u32 (core1 が書く)
  op      : u8  (1=save: BNO→data / 2=load: data→BNO)
  ok      : u8  (1=I2C 成功)
  data    : u8[22]
```

- core0: (load なら data を書き) op を書く → DMB → req_seq++ で発行。
- core1: req_seq != ack_seq を検出 → CONFIG モードで実行 (save は data へ
  吸い出し) → ok を書く → DMB → ack_seq = req_seq で完了。
- core0 の排出スレッドが ack_seq の変化を監視し、calib_get 用ミラーの更新と
  calib sink への push を行う (既存メソッドのセマンティクスは不変)。

## 5. core1 の責務境界

- **走らせない**: Shizuku (SVC/obj_api/yield 禁止)、printf (stdio mutex が core0 と
  絡むため禁止。異常報告は STATUS レコードで)。
- **専有する**: I2C バスと BNO055/BME280。core0 は I2C を一切触らない。
- ループ: 1kHz グリッド (busy-wait / timer)。毎 tick ACC、2 tick ごと GYR、
  50 tick ごとに MAG と BARO を位相をずらして読む。
- I2C 障害時は既存のエスカレーション (recover → センサ再 init) を core1 内で完結。
  core0 のスケジューラは影響を受けない (従来の「I2C 停滞→全スレッド凍結」問題は
  構造ごと消える)。
- 起動: `multicore_launch_core1()`。core1 の CPACR (FPU) が SDK の per-core init で
  立つことを要確認 (立たない場合 core1 の float で即 HardFault)。core1 は Shizuku の
  例外ハンドラを通らないので FP コンテキスト退避 (svc_asm_handler) とは独立。

core0 側の BNO055_DRIVER / BME280_DRIVER は「I2C を叩く係」から「リングを排出して
ch 別に配る係」へ縮退する (read_latest / sink push の外部インタフェースは維持)。

## 6. 未決事項 / 実装前に要確認

1. **ACC+GYR まとめ読み**: BNO055 の ACC(0x08..0x0D) と GYR(0x14..0x19) レジスタ間の
   連続バースト読み可否。1 トランザクション化できればバス占有 ~38% → ~30%、
   トランザクション数 1500/s → 1000/s。core1 専有なので占有率自体は許容だが確認価値あり。
2. **AMG モードでの 0xFFFF バースト破損の再検証**: 現行の破損対策は NDOF フュージョン
   更新との衝突を想定したもの。AMG 化で消えるか、別の形で出るかは実測。
3. **FUSED チャンネルの形式**: 機内 Mahony/EKF 実装後に定義。
4. **安全装置ボード (Seeed XIAO RP2040/2350) とのリンク**: 別ボード化が決定済み。
   本プロトコルの UART 転用か独自かは未定。ch_id 空間 (0x20..) を予約だけしておく。
5. core1 の CPACR per-core 初期化の確認 (§5)。

## 7. 実装順序

1. FPU 実機テスト (ユーザ実施中) — 本件と独立
2. コア間リング + core1 I/O ループ (まず現行 NDOF のまま core1 化して等価動作を確認)
3. ワイヤプロトコルエンコーダ + PC 側デコーダ (tools/)
4. BNO055 AMG モード化・レート引き上げ (1kHz/500Hz)
5. 機内 Mahony → FUSED チャンネル → 高度 KF/EKF
