#!/bin/sh

cp arch/x86/configs/android-x86_defconfig .config
make menuconfig
cp .config arch/x86/configs/android-x86_defconfig
rm .config
make mrproper
