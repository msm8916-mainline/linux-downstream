obj-y += ASH_Interface.o
obj-y += IRsensor_Interface.o
obj-n += HALLsensor_Interface.o

DEBUG = n

ifeq ($(DEBUG),y)
	DEBFLAGS = -O -g -DASH_INTERFACE_DEBUG
else
	DEBFLAGS = -O -g
endif
ccflags-y += $(DEBFLAGS)

