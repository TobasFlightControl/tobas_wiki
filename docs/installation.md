# Installation

## Dual Booting Ubuntu 20.04 LTS

---

Tobas operates on Ubuntu 20.04 LTS.
For Windows users, while it's possible to use WSL (Windows Subsystem for Linux), there have been reports of issues.
Therefore, a native installation (dual booting with Windows) is recommended.

Installing Tobas on Your PC

---

Unzip the tobas-x.x.x.zip file into your installation directory.
The installation directory can be the home directory or any other location of your choice.

```bash
$ sudo apt install -y unzip
$ cd path/to/installation/directory/
$ unzip tobas-x.x.x.zip
```

Then, run the installation script:

```bash
$ cd tobas/
$ ./lib/tobas_setup/install_prereqs_ubuntu.sh
```

## フライトコントローラのセットアップ

---

As of now, Tobas has been tested and confirmed to work with the
<a href=https://www.raspberrypi.com/products/raspberry-pi-4-model-b/ target="_blank">Raspberry Pi Model 4B</a>
and <a href=https://navio2.hipi.io/ target="_blank">Navio2</a>.
In the future, there are plans to extend support to more reliable hardware with better real-time capabilities and redundancy.

Write the Raspbian image with tobas-x.x.x installed onto a micro SD card.
TODO
