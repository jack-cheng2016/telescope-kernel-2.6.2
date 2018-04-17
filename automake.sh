#!/bin/sh
#make distclean
make omap3_telescope_defconfig
#make uImage
#make CROSS_COMPILE=arm-none-linux-gnueabi- ARCH=arm uImage
make CROSS_COMPILE=arm-eabi- ARCH=arm uImage -j4
