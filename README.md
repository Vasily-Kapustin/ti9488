# MSP3520 Driver for ili9488 On Raspberry Pi 
The display documentation: http://www.lcdwiki.com/3.5inch_SPI_Module_ILI9488_SKU:MSP3520

This driver is based on work found here: https://github.com/birdtechstep/tinydrm

I would use the tinydrm drivers but since tinydrm was implemented into Linux as tiny the repo above is stale

Before anyone asks why I'm ripping functions straight out of the mipi_dbi and drm libraries, current versions do not support RGB666 which the controller uses

Driver itself is located in ili9488.c
The code is really in 2 sections the first is the mipi_dbi functions and the second that starts at about line 307 is the driver

This driver is unfortunately OS dependant and is currently intended for rpi 6.6.y (BookWorm)

## Install

### Update
~~~
sudo apt update
sudo apt upgrade
sudo reboot
~~~

### Headers
~~~~
sudo apt install git bc bison flex libssl-dev libncurses5-dev
sudo apt-get install raspberrypi-kernel-headers
~~~~

### ti9488
~~~~
git clone https://github.com/Vasily-Kapustin/ti9488
cd ti9488
make
sudo cp ili9488.ko /lib/modules/`uname -r`/kernel/drivers/gpu/drm/tiny/
sudo depmod
~~~~
If you cannot find the /build directory this means that the headers did not download correctly



### Set up overlay
~~~~
cd rpi-overlays
sudo dtc -@ -I dts -O dtb -o /boot/overlays/generictft-9488-overlay.dtbo generictft-9488-overlay.dts
~~~~

### /boot/firmware/config.txt
~~~~
dtoverlay=generictft-9488-overlay
dtparam=speed=62000000
dtparam=rotation=90
~~~~

## Touchscreen
Currently running into issues with ads7846 driver