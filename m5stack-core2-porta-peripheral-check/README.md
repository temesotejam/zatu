# Core2 Port A 周辺機器チェッカー

M5Stack Core2 のタッチ画面で Port A に接続した機器を一つずつ確認する診断ファームウェアです。画面下部の `SERVO` / `DISTANCE` / `POWER` / `MOTOR` をタップして接続モードを選びます。

既存の `m5stack-cores3-porta-peripheral-check` は変更せず、Core2用を別プロジェクトとして追加しています。

## GitHub Pagesから書き込む

`main`へ反映後、GitHub ActionsがCore2用ファームウェアをPlatformIOでビルドし、ESP32用の結合BINとESP Web Tools用manifestを生成してGitHub Pagesへ公開します。

書き込みページ：

- https://temesotejam.github.io/zatu/

PC版ChromeまたはEdgeでページを開き、Core2をUSB Type-Cケーブルで接続して`Core2へ書き込む`を押します。ポート選択画面では、Core2のCP210xまたはCH9102系USBシリアルポートを選択します。

これはCore2専用です。CoreS3には書き込まないでください。書き込み前は、意図しない駆動を防ぐためサーボとVESCを無負荷または切り離した状態にしてください。

## 安全方針

- PCA9685 はCH0を0〜180°で操作できます。画面遷移時はPWMをFull OFFにします。
- VL53L5CX は8x8・10 Hzの距離測定を開始し、中央ゾーンの距離を表示します。
- INA226 は電圧・シャント電圧を読み、既存の船体設定と同じ2 mΩシャントで電流を換算して表示します。異なるシャント抵抗ではソース内の `SHUNT_OHM` を変更してください。
- VESC は通常モードでDutyを±3%に制限します。`MOTOR`画面の上限解除ボタンを1.5秒長押しすると、その起動中だけ±100%のDuty指令を許可します。解除中は赤く表示され、同ボタンのタップ、画面遷移、再起動、またはスライダーを250 ms操作しない場合にDuty 0へ戻ります。VESC本体の保護・電流・Duty設定は変更しません。

## 配線

Core2 Port A は、黒=GND、赤=5V、黄=GPIO32、白=GPIO33です。**3.3V機器では赤線を接続しません。** 機器のVCCは別の安定した3.3V電源から供給し、GNDだけはCore2と共通にします。I²Cのプルアップ先も3.3Vです。

| 画面モード | 機器 | 接続 | 期待アドレス／値 |
| --- | --- | --- | --- |
| `SERVO` | PCA9685 サーボコントローラ | Port Aの通常I²Cケーブル（赤線未接続） | `0x40`（変更されている場合あり） |
| `DISTANCE` | VL53L5CX ToF距離センサ | Port Aの通常I²Cケーブル（赤線未接続） | `0x29`、中央距離 [mm] |
| `POWER` | INA226 電流・電圧センサ | Port Aの通常I²Cケーブル（赤線未接続） | 製造者ID `0x5449`、bus [V]、shunt [mV]、電流 [A] |
| `MOTOR` | VESC | Port AをUARTとして使う専用ケーブル | 黄/GPIO32 → VESC RX、白/GPIO33 ← VESC TX、黒/GND共通。赤/5Vは未接続。 |

`MOTOR`選択中はGPIO32/GPIO33をUART TX/RX（115200 bps、8N1）に再割当てします。この間はI²Cを停止します。VESC側UARTは**3.3 V TTL**であることを確認してください。5 V/RS-232信号を直接つないではいけません。

## ローカルでビルド・書き込み

```powershell
cd m5stack-core2-porta-peripheral-check
pio run
pio run -t upload
```

シリアル出力は115200 bpsです。起動直後は`SERVO`モードで、出力を一切変更しません。
