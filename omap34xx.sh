#!/bin/sh
#sudo rm -fr drivers/input/misc/rotary_encoder.ko
sudo rm -fr ~/tftproot/omap34xxcam.ko

make CROSS_COMPILE=arm-eabi-  ARCH=arm modules SUBDIRS=drivers/media/video
#cp drivers/input/misc/rotary_encoder.ko  ~/tftproot/
#ls drivers/media/video/*.ko
sudo cp drivers/media/video/omap34xxcam.ko ~/tftproot/
ls ~/tftproot/ -l

