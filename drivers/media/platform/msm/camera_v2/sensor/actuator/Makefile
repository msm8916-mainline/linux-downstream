ccflags-y += -Idrivers/media/platform/msm/camera_v2
ccflags-y += -Idrivers/media/platform/msm/camera_v2/sensor/io
ccflags-y += -Idrivers/media/platform/msm/camera_v2/sensor/cci
ifeq ($(CONFIG_ALTEK_AF),y)
obj-$(CONFIG_MSMB_CAMERA) += msm_actuator_altek.o
else
obj-$(CONFIG_MSMB_CAMERA) += msm_actuator.o
endif
