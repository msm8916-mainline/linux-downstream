#
#  Copyright (c) 2014, UPI-SEMI, Inc. All Rights Reserved.
#

EXTRA_CFLAGS += -DFEATURE_PLAT_LINUX
#EXTRA_CFLAGS += -DFEATURE_BUFFER_ENABLE
EXTRA_CFLAGS += -DPLATFORM_ASUS_Z380KL_COVER
EXTRA_CFLAGS += -DFEATURE_ASUS_DRV_VERSION
EXTRA_CFLAGS += -I$(src)/INC/

upi_gauge-objs += SRC/Adc.o
upi_gauge-objs += SRC/ChargeCtl.o
upi_gauge-objs += SRC/Config.o
upi_gauge-objs += SRC/GaugeMain.o
upi_gauge-objs += SRC/Measurement.o
upi_gauge-objs += SRC/Reg.o
upi_gauge-objs += SRC/Sys.o
upi_gauge-objs += SRC/Capacity.o
upi_gauge-objs += SRC/VCapacity.o
upi_gauge-objs += SRC/CodeTbl.o
upi_gauge-objs += SRC/GasGauge.o
upi_gauge-objs += SRC/I2c.o
upi_gauge-objs += SRC/Platform.o
upi_gauge-objs += SRC/Sbs.o
upi_gauge-objs += SRC/Task.o
upi_gauge-objs += SRC/UpiDB.o
upi_gauge-objs += SRC/Alarm.o

upi_gauge-objs += UpiDB_A023Cl.o
upi_gauge-objs += UpiDB_A0249l.o
upi_gauge-objs += UpiDB_A025Jl.o
upi_gauge-objs += UpiDB_A0262h.o
upi_gauge-objs += UpiDB_A027Co.o

upi_gauge-objs += upi_ug31xx.o

obj-y += upi_gauge.o

