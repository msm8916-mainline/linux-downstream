ccflags-y += -Idrivers/media/platform/msm/camera_v2/sensor/cci
ccflags-y += -Idrivers/media/platform/msm/camera_v2
ccflags-y += -Idrivers/media/platform/msm/camera_v2/sensor/io
obj-$(CONFIG_MSMB_CAMERA) += msm_led_flash.o
obj-$(CONFIG_MSMB_CAMERA) += msm_led_trigger.o
obj-$(CONFIG_MSMB_CAMERA) += msm_led_i2c_trigger.o
obj-$(CONFIG_MSMB_CAMERA) += adp1660.o
obj-$(CONFIG_MSMB_CAMERA) += bd7710.o
obj-$(CONFIG_MSMB_CAMERA) += msm_led_torch.o
obj-$(CONFIG_MSMB_CAMERA) += msm_flash.o
#[PLATFORM]-Add-BEGIN by TCTNB.YQJ, 2015/05/15, Add flash_led driver
ifndef CONFIG_TCT_8X16_M823_ORANGE
obj-$(CONFIG_MSMB_CAMERA) += lm3642.o
endif
ifdef CONFIG_TCT_8X16_M823_ORANGE
#obj-$(CONFIG_MSMB_CAMERA) += lm3643_m823.o
#[PLATFORM]-Add-BEGIN by CL, 2015/04/16, for dual led according to Yang jie
#obj-$(CONFIG_MSMB_CAMERA) += lm3643_m823_tuning.o
obj-$(CONFIG_MSMB_CAMERA) += lm3643_m823_tuning_v2.o
endif
#[PLATFORM]-Add-END by TCTNB.YQJ
