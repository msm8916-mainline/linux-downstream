mkdir output
make -C $(pwd) O=output msm8916_sec_defconfig VARIANT_DEFCONFIG=msm8916_sec_a3u_eur_defconfig SELINUX_DEFCONFIG=selinux_defconfig
make -C $(pwd) O=output 
cp output/arch/arm/boot/zImage arch/arm/boot/zImage
