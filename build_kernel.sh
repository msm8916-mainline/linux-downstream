mkdir $(pwd)/out
make -C $(pwd) O=$(pwd)/out msm8916_sec_defconfig VARIANT_DEFCONFIG=msm8916_sec_a5u_eur_defconfig SELINUX_DEFCONFIG=selinux_defconfig
make -C $(pwd) O=$(pwd)/out
cp $(pwd)/out/arch/arm/boot/zImage $(pwd)/arch/arm/boot/zImage
