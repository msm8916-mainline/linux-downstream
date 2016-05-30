ifeq ($(CONFIG_OF),y)

ifeq ($(CONFIG_MACH_MSM8939_ALTEV2_VZW),y)
        DTS_TARGET := altev2_vzw
else 
ifeq ($(CONFIG_MACH_MSM8939_P1BDSN_GLOBAL_COM),y)
        DTS_TARGET := p1bdsn_global_com
else
ifeq ($(CONFIG_MACH_MSM8939_P1BSSN_SKT_KR),y)
        DTS_TARGET := p1bssn_skt_kr
else
        DTS_TARGET := altev2_vzw
endif
endif
endif

DTS_TARGET_DIR := msm8939-$(DTS_TARGET)

ifeq ($(CONFIG_MACH_MSM8939_ALTEV2_VZW),y)
dtb-y += msm8939-$(DTS_TARGET).dtb \
         msm8936-$(DTS_TARGET).dtb
else 
ifeq ($(CONFIG_MACH_MSM8939_P1BDSN_GLOBAL_COM),y) 
dtb-y += msm8939-p1bdsn.dtb

else
ifeq ($(CONFIG_MACH_MSM8939_P1BSSN_SKT_KR),y)
dtb-y += msm8939-p1bssn.dtb

else
dtb-y += msm8939-$(DTS_TARGET).dtb \
         msm8936-$(DTS_TARGET).dtb
endif
endif
endif
         
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
