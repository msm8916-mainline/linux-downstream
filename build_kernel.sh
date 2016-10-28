#!/bin/bash

export ARCH=arm
export CROSS_COMPILE=../PLATFORM/prebuilts/gcc/linux-x86/arm/arm-eabi-4.8/bin/arm-eabi-

make msm8916_sec_defconfig VARIANT_DEFCONFIG=msm8916_sec_a5_eur_defconfig SELINUX_DEFCONFIG=selinux_defconfig
make -j