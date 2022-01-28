# Max1 Kernel module

MODULE_NAME = snd-soc-tfa98xx
ARCH := arm
CROSS_COMPILE := arm-linux-gnueabihf-

# update to point to your path
KDIR := $(HOME)/GIT/linux_3v10/

EXTRA_CFLAGS += -I$(PWD)/include
obj-y += tfa98xx-debug-common.o
obj-y += snd-soc-tfa98xx.o
snd-soc-tfa98xx-objs := tfa98xx.o
snd-soc-tfa98xx-objs += tfa_container.o 
snd-soc-tfa98xx-objs += tfa_dsp.o 
	
MAKEARCH := $(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE)

all:
	$(MAKEARCH) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKEARCH) -C $(KDIR) M=$(PWD) clean
