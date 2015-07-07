ifeq ($(CONFIG_OF),y)

#dtb-$(CONFIG_ARCH_MSM8916) += msm8916-cdp.dtb \
#	msm8916-mtp.dtb \
#	msm8916-512mb-mtp.dtb \
#	msm8939-cdp.dtb \
#	msm8939-mtp.dtb 

dtb-$(CONFIG_ARCH_MSM8916) += msm8916-ze500kl-evb.dtb \
	msm8916-ze500kl-sr1.dtb \
	msm8916-ze500kl-er1.dtb \
	msm8916-ze500kl-er2.dtb \
	msm8916-ze500kg-evb.dtb \
	msm8916-ze500kg-sr1.dtb \
	msm8916-ze500kg-er1.dtb \
	msm8916-ze500kg-er2.dtb

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
