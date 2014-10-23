mkdir $(pwd)/output
make -C $(pwd) O=$(pwd)/output msm8916_sec_defconfig VARIANT_DEFCONFIG=msm8916_sec_kleos_chnopen_defconfig SELINUX_DEFCONFIG=selinux_defconfig
make -C $(pwd) O=$(pwd)/output
cp $(pwd)/output/arch/arm/boot/zImage $(pwd)/arch/arm/boot/zImage
