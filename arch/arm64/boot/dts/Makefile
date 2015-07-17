dtb-$(CONFIG_ARCH_VEXPRESS) += rtsm_ve-aemv8a.dtb foundation-v8.dtb
dtb-$(CONFIG_ARCH_XGENE) += apm-mustang.dtb

targets += dtbs

#used for cleaning - not building
ifndef CONFIG_TCT_8X16_COMMON
subdir- := qcom
endif

ifdef CONFIG_TCT_8X16_ALTO5
subdir- := alto5
endif

ifdef CONFIG_TCT_8X16_ALTO4
subdir- := alto4
endif

ifdef CONFIG_TCT_8X16_ALTO5_VDF
subdir- := alto5_vdf
endif

ifdef CONFIG_TCT_8X16_ALTO5_PREMIUM
subdir- := alto5_premium
endif

ifdef CONFIG_TCT_8X16_ALTO45_LATAM_B28
subdir- := alto45_latam_b28
endif

ifdef CONFIG_TCT_8X16_POP10
subdir- := pop10
endif
ifdef CONFIG_TCT_8X16_IDOL3
subdir- := idol3
endif
ifdef CONFIG_TCT_8X16_IDOL347
subdir- := idol347
endif

DTB_NAMES := $(subst $\",,$(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE_NAMES))
ifneq ($(DTB_NAMES),)
DTB_LIST := $(addsuffix .dtb,$(DTB_NAMES))
else
DTB_LIST := $(dtb-y)
endif
targets += $(DTB_LIST)

dtbs: $(addprefix $(obj)/, $(DTB_LIST))

clean-files := dts/*.dtb *.dtb
