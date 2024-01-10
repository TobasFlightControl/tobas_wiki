# Hardware Setup

シミュレーションが成功したことを確認し，実機の設定に入ります．

## 実機の作成

---

以下のサイトを参考に，実機を作成します．

- <a href=https://docs.emlid.com/navio2/hardware-setup target="_blank">Hardware setup | Navio2</a>
- <a href=https://docs.emlid.com/navio2/ardupilot/typical-setup-schemes target="_blank">Typical setup schemes | Navio2</a>

その際に以下の点に注意してください:

### モータの回転方向が Tobas Setup Assistant の設定と一致している

モータ回転方向の設定は`tobas_f450_config/config/f450.tbsf`の`rotor_x/direction`で確認できます．
`rotor_x/link_name`に対して正しい回転方向のモータが取り付けられていることを確認してください．

### ESC のピン番号が Tobas Setup Assistant の設定と一致している

ESC のピン番号の設定は`tobas_f450_config/config/f450.tbsf`の`rotor_x/pin`で確認できます．
この番号と Navio2 に記載されているピン番号が一致していることを確認してください．

## ラズパイの WiFi 設定

---

最初は WiFi に接続できないため，ラズパイをディスプレイ，キーボード，マウスに接続してから電源を入れます．
初期パスワードである`raspberry`を入力してログインしてください．

`/boot/wpa_supplicant.conf`に WiFi の SSID とパスワードを入力します．
以下のように 1 つ以上の`network`を定義してください．
`ssid`が SSID，`psk`がパスワードです．
複数のネットワークが利用可能のときは`priority`が大きいほうが優先されます．

```txt
country=GB
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1

network={
  ssid="your_first_ssid"
  psk="your_first_password"
  key_mgmt=WPA-PSK
  priority=1
}

network={
  ssid="your_second_ssid"
  psk="your_second_password"
  key_mgmt=WPA-PSK
  priority=0
}
```

ラズパイを再起動すると，自動的に WiFi に接続します．
これ以降ラズパイのディスプレイ，キーボード，マウスは必要ありません．

## ラズパイへの SSH 接続

---

PC で以下のコマンドを実行すると，PC からラズパイに SSH 接続します:

```bash
$ ssh pi@navio
```

パスワードは先程と同じく`raspberry`です．
これでラズパイを遠隔で操作できるようになりました．

## キャリブレーション

---

センサー等のキャリブレーションを行います．
PC をラズパイに SSH 接続した状態で以下を実行してください．

### 加速度センサ

以下を実行し，コンソールの指示に従ってください:

```bash
$ ~/tobas-x.x.x/lib/tobas_real/accel_calibration
```

### 地磁気センサ

以下を実行し，コンソールの指示に従ってください:

```bash
$ ~/tobas-x.x.x/lib/tobas_real/mag_calibration
```

### バッテリー電圧

バッテリーと Navio2 が正しく接続していることを確認してください．
以下を実行し，コンソールの指示に従ってください:

```bash
$ ~/tobas-x.x.x/lib/tobas_real/adc_calibration
```

### RC 入力

RC レシーバと Navio2 が正しく接続し，RC レシーバと RC トランスミッタが通信できることを確認してください．
以下を実行し，コンソールの指示に従ってください:

```bash
$ ~/tobas-x.x.x/lib/tobas_real/rcin_calibration
```

### ESC

<span style="color: red;"><strong>警告: プロペラがモータから取り外されていることを確認してください．</strong></span>

以下を確認してください:

- ESC のピン番号と Navio2 のピン番号が一致している．
- バッテリーが Navio2 から取り外され，ラズパイが type-C からのみ給電されている．

以下を実行し，コンソールの指示に従ってください:

```bash
$ su
$ ~/tobas-x.x.x/lib/tobas_real/esc_calibration
```

キャリブレーションが成功したかどうかを確認します．
以下を実行してください:

```bash
$ su
$ roslaunch tobas_motor_test motors_handler.launch
```

外部 PC で以下を実行してください:

```bash
$ roslaunch tobas_motor_test motor_test_gui.launch
```

全てのモータについて以下の点を確認してください:

- スロットルが 0.0 のとき，モータは回転しない.
- スロットルが 0.1 のとき，モータはゆっくり回転する.
- スロットルが 1.0 に向けて上昇するにつれ，回転音が徐々に高くなる．
- 同じモデルの 2 つのモータは，同じスロットルで概ね同じ高さの回転音を発する．

これらの条件が満たされない場合，ESC は正しくキャリブレーションされていません．
その場合は，キャリブレーションをやり直すか，<a href=https://github.com/bitdump/BLHeli target="_blank">BLHeli-Suite</a>
などのツールを用いて PWM の範囲を 1000us ~ 2000us に設定してください．

### センサノイズの計測 (実行しなくてもよい)

<span style="color: red;"><strong>警告: この操作ではプロペラをつけた状態でモータを回転させます．</strong></span>

プロペラの回転により発生する振動は，IMU，特に加速度センサに非常に大きな影響を与えます．
そのため，飛行前にプロペラをつけた状態でモータを回転させてセンサノイズを計測することで，
実際の飛行状態に近いデータを得ることができ，状態推定の精度を向上させることができます．

実行前に以下の点を確認してください:

- バッテリー，ESC，モータ，プロペラ，ラズパイが正しく接続されている．
- <span style="color: red;"><strong>ドローンが動かないようにしっかりと固定されている．</strong></span>

<span style="color: red;"><strong>すぐに Ctrl+C でプログラムを停止できるように構えた状態で</strong></span>以下を実行してください:

```bash
$ su
$ ~/tobas-x.x.x/lib/tobas_real/measure_sensor_noise
```
