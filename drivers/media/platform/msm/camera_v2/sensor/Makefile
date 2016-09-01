ccflags-y += -Idrivers/media/platform/msm/camera_v2
ccflags-y += -Idrivers/media/platform/msm/camera_v2/msm_vb2
ccflags-y += -Idrivers/media/platform/msm/camera_v2/camera
ccflags-y += -Idrivers/media/platform/msm/camera_v2/sensor/io
ccflags-y += -Idrivers/media/platform/msm/camera_v2/sensor/cci
obj-$(CONFIG_MSMB_CAMERA) += cci/ io/ csiphy/ csid/ actuator/ flash/ eeprom/ ois/
obj-$(CONFIG_MSM_CAMERA_SENSOR) += msm_sensor_init.o msm_sensor_driver.o msm_sensor.o
obj-$(CONFIG_MT9M114) += mt9m114.o
obj-$(CONFIG_OV5645) += ov5645.o
obj-$(CONFIG_OV5645) += ov7695.o
obj-$(CONFIG_GC0339_ALTO5) += gc0339_alto5.o
obj-$(CONFIG_TCT_8X16_ALTO5) += ov5670_truly_otp.o
obj-$(CONFIG_GC0310) += gc0310.o
#[PLATFORM]-Add-BEGIN by TCTSZ.yaohui.zeng, 2014/07/26, Add 1st Front camera Hi258 for POP10
obj-$(CONFIG_HI258_POP10) += hi258_pop10.o
#[PLATFORM]-Add-END by TCTSZ.yaohui.zeng, 2014/07/26
#[PLATFORM]-Add-BEGIN by TCTSZ.yaohui.zeng, 2014/09/18, Add 2nd Front camera GC2145 for POP10
obj-$(CONFIG_GC2145_POP10) += gc2145_pop10.o
#[PLATFORM]-Add-END by TCTSZ.yaohui.zeng, 2014/09/18
#[PLATFORM]-Add-BEGIN by TCTSZ.ZKX, 2014/08/26, Add Rear camera compatibility for POP10
obj-$(CONFIG_TCT_8X16_POP10) += s5k5e2_otp.o ov5670_truly_otp.o
#[PLATFORM]-Add-END by TCTSZ.ZKX, 2014/08/26

#[PLATFORM]-Add-BEGIN by TCTNB.YJ, 2014/11/09, Add s5k5e2_otp_alto4
obj-$(CONFIG_TCT_8X16_ALTO4) += s5k5e2_otp_alto4.o
#[PLATFORM]-Add-END by TCTNB.YJ

