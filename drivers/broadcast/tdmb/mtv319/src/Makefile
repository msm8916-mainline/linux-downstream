#
# Makefile for the TDMB MTV319 drivers.
#

# Each configuration option enables a list of files.
# LGE

ccflags-$(CONFIG_LGE_BROADCAST_TDMB_MTV319) += -Idrivers/broadcast/tdmb/mtv319/inc

obj-$(CONFIG_LGE_BROADCAST_TDMB_MTV319) += broadcast_mtv319_drv_if.o
obj-$(CONFIG_LGE_BROADCAST_TDMB_MTV319) += broadcast_mtv319.o
obj-$(CONFIG_LGE_BROADCAST_TDMB_MTV319) += tdmb_tunerbbdrv_mtv319.o

obj-$(CONFIG_LGE_BROADCAST_TDMB_MTV319) += mtv319.o
obj-$(CONFIG_LGE_BROADCAST_TDMB_MTV319) += mtv319_port.o
obj-$(CONFIG_LGE_BROADCAST_TDMB_MTV319) += mtv319_rf.o
obj-$(CONFIG_LGE_BROADCAST_TDMB_MTV319) += mtv319_tdmb.o

# for multiple channle decoder (FIC+MSC or MSCs)
#obj-$(CONFIG_LGE_BROADCAST_TDMB_MTV319) += mtv319_cifdec.o

