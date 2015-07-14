
obj-m += exfatfs.o


exfatfs-y := exfat_super.o exfat.o exfat_api.o exfat_blkdev.o exfat_cache.o \
				exfat_data.o exfat_global.o exfat_nls.o exfat_oal.o exfat_upcase.o

#KDIR	:= /lib/modules/$(shell uname -r)/build
KDIR	:= $(ANDROID_PRODUCT_OUT)/obj/KERNEL_OBJ
PWD	:= $(shell pwd)

all:
	$(MAKE) -C $(KDIR) CROSS_COMPILE=$(ARM_EABI_TOOLCHAIN)/arm-eabi- \
		M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

