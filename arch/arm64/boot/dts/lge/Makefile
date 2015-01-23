ifeq ($(CONFIG_OF),y)

ifeq ($(CONFIG_MACH_MSM8939_ALTEV2_VZW),y)
        DTS_TARGET := altev2_vzw
else
        DTS_TARGET := altev2_vzw
endif

DTS_TARGET_DIR := msm8939-$(DTS_TARGET)


dtb-y += msm8939-$(DTS_TARGET).dtb \
         msm8936-$(DTS_TARGET).dtb
         
DTB_NAMES := $(subst $\",,$(CONFIG_BUILD_ARM_APPENDED_DTB_IMAGE_NAMES))
ifneq ($(DTB_NAMES),)
DTB_LIST := $(addsuffix .dtb,$(DTB_NAMES))
else
DTB_LIST := $(dtb-y)
endif

targets += dtbs
targets += $(addprefix ../, $(DTB_LIST))
endif

$(obj)/../%.dtb: $(src)/$(DTS_TARGET_DIR)/%.dts FORCE
	$(call if_changed_dep,dtc)

dtbs: $(addprefix $(obj)/../,$(DTB_LIST))

clean-files := *.dtb
