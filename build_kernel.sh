#!/bin/bash
export CROSS_COMPILE=/opt/toolchains/arm-eabi-4.7/bin/arm-eabi-
export ARCH=arm
mkdir output
make -C $(pwd) O=output msm8916_sec_defconfig VARIANT_DEFCONFIG=msm8916_sec_a5u_eur_defconfig SELINUX_DEFCONFIG=selinux_defconfig
make -C $(pwd) O=output 
cp $(pwd)/output/arch/arm/boot/zImage $(pwd)/arch/arm/boot/zImage
