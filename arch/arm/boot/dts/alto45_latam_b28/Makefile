ifeq ($(CONFIG_OF),y)

ifndef CONFIG_TCT_8X16_COMMON
dtb-$(CONFIG_ARCH_MSM8916) += msm8916-sim.dtb \
	msm8916-rumi.dtb \
	msm8916-cdp.dtb \
	msm8916-cdp-smb1360.dtb \
	msm8916-mtp.dtb \
	msm8916-mtp-smb1360.dtb \
	msm8916-qrd-skuh.dtb \
	msm8939-sim.dtb \
	msm8939-rumi.dtb
endif

dtb-$(CONFIG_ARCH_MSM8916) += msm8916-mtp.dtb
#	msm8916-qrd-skuh.dtb \
#	msm8939-sim.dtb \
#	msm8939-rumi.dtb

ifndef CONFIG_TCT_8X16_COMMON
dtb-$(CONFIG_ARCH_MSM) += msmplutonium-sim.dtb
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

$(obj)/../%.dtb: $(src)/%.dts FORCE
	$(call if_changed_dep,dtc)

dtbs: $(addprefix $(obj)/../,$(DTB_LIST))

clean-files := *.dtb
