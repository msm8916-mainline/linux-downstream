#!/bin/bash
PATH=`pwd`/arm-eabi-4.8/bin:$PATH
mkdir -p out
make -j32 O=out ARCH=arm CROSS_COMPILE=arm-eabi- msm8916_defconfig
make -j32 O=out ARCH=arm CROSS_COMPILE=arm-eabi-
