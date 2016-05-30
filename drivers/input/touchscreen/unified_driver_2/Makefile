
obj-y	+=  lgtp_common_driver.o
obj-y	+=  lgtp_platform_api.o
obj-y	+=  lgtp_model_config.o
obj-y   +=  lgtp_touch_notify.o


ifeq ($(CONFIG_TOUCHSCREEN_LGE_SYNAPTICS), y)
obj-y 	+= S3320/
endif

ifeq ($(CONFIG_MSM8916_C90), y)
obj-y   += S3320/
endif

ifeq ($(CONFIG_MSM8916_P1C), y)
obj-y   += S3320/
endif

ifeq ($(CONFIG_TOUCHSCREEN_LU201X), y)
obj-y 	+= LU201X/
endif

ifeq ($(CONFIG_TOUCHSCREEN_LU202X), y)
obj-y 	+= LU202X/
endif

ifeq ($(CONFIG_TOUCHSCREEN_LU201X), y)
obj-y 	+= FT6X36/
endif

ifeq ($(CONFIG_TOUCHSCREEN_LU201X), y)
obj-y 	+= DUMMY/
endif

ifeq ($(CONFIG_TOUCHSCREEN_LU201X), y)
obj-y 	+= MIT200/
endif

ifeq ($(CONFIG_TOUCHSCREEN_LGE_MELFAS), y)
obj-y 	+= MIT200/
endif

ifeq ($(CONFIG_TOUCHSCREEN_MSM8916_YG), y)
obj-y   += S3320/
endif
