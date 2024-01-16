# Installation

## Setting Up Dual Boot with Ubuntu 20.04 LTS

---

Tobas is optimized for Ubuntu 20.04 LTS.
For Windows users, although WSL (Windows Subsystem for Linux) is an option, it has shown some compatibility issues.
Hence, a dual-boot installation with Windows is the recommended approach.

## Installing Tobas on Your PC

---

Follow these steps to install Tobas on your computer:

First, extract the 'tobas-x.x.x.zip' file into your chosen installation directory.
This can be your home directory or any other preferred location.
Use the following commands in the terminal:

```bash
$ sudo apt install -y unzip
$ cd path/to/installation/directory/
$ unzip tobas-x.x.x.zip
```

Next, execute the installation script:

```bash
$ cd tobas/
$ ./lib/tobas_setup/install_prereqs_ubuntu.sh
```

## Flight Controller Setup

---

Currently, Tobas has been successfully tested with the
<a href=https://www.raspberrypi.com/products/raspberry-pi-4-model-b/ target="_blank">Raspberry Pi Model 4B</a>
and <a href=https://navio2.hipi.io/ target="_blank">Navio2</a>.
Plans are in place to expand support to other hardware platforms that offer improved real-time performance and redundancy.

Write the Raspbian image with Tobas version 'x.x.x' pre-installed onto a micro SD card.

For more detailed instructions or if you encounter any issues during the installation process, please refer to the Tobas documentation or contact support.

<!-- TODO -->
