# Installation

## Ubuntu 20.04 LTS のデュアルブート

---

Tobas は Ubuntu 20.04 LTS 上で動作します．
Windows を使っている場合は WSL (Windows Subscription for Linux) を利用することもできますが，
不具合の報告もあるためネイティブインストール (Windows とデュアルブート) することを勧めます．
以下のサイトが参考になります:

- <a href=https://guminote.sakura.ne.jp/archives/233 target="_blank">Windows 10 の場合</a>
- <a href=https://jp.minitool.com/partition-disk/windows-11-and-linux-dual-boot.html target="_blank">Windows 11 の場合</a>

いずれにしても，以下の点に注意してください:

- Windows から BitLocker を無効にする (「解除中」ではまだ未完)
- BIOS の設定画面から Secure Boot を無効にする
- Ubuntu のインストール時に WiFi に接続し，"Install third-party"にチェックをつける

## PC への Tobas のインストール

---

tobas-x.x.x.zip をインストールディレクトリ以下に解凍してください．
インストールディレクトリはホームディレクトリ等どこでも構いません．

```bash
$ sudo apt install -y unzip
$ cd path/to/installation/directory/
$ unzip tobas-x.x.x.zip
```

インストールスクリプトを実行してください．

```bash
$ cd tobas-x.x.x/
$ ./lib/tobas_setup/install_prereqs_ubuntu.sh
```

## フライトコントローラのセットアップ

---

TODO: tobas-x.x.x がインストールされた Raspbian イメージをマイクロ SD に書き込む
