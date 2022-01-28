################################################################################

1. How to Build
- get Toolchain
From android git server, codesourcery and etc ..
- arm-eabi-4.8
- (git clone https://android.googlesource.com/platform/prebuilts/gcc/linux-x86/arm/arm-eabi-4.8 -b android-5.1.1_r9)

- put the gcc in the right path.
put the arm-eabi-4.8 in the $(kernel directory)/ path

$ ./build_kernel.sh

2. Output files
- Kernel : $(kernel directory)/out/arch/arm/boot/zImage

3. How to Clean
$ rm -rf $(kernel directory)/out
################################################################################
