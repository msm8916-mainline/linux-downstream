
obj-y	+=  lgtp_common_driver.o
obj-y	+=  lgtp_platform_api.o
obj-y	+=  lgtp_model_config.o

obj-$(CONFIG_TOUCHSCREEN_LGE_SYNAPTICS)	+= S3320/
ifeq ($(CONFIG_MTK_TD4191), y)
obj-y		+= td4191/
endif
obj-$(CONFIG_TOUCHSCREEN_LU201X)	+= LU201X/
obj-$(CONFIG_TOUCHSCREEN_LU202X)	+= LU202X/
obj-$(CONFIG_TOUCHSCREEN_FOCALTECH)	+= FT6X36/
ifeq ($(CONFIG_TOUCHSCREEN_LGE_MELFAS), y)
obj-y	+= 	MIT200/
endif
