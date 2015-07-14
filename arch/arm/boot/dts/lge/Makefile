ifeq ($(CONFIG_OF),y)

dtb-$(CONFIG_MACH_MSM8909_LGPS22)          += msm8909-lgps22.dtb
dtb-$(CONFIG_MACH_MSM8909_LGPS22)          += msm8909-pm8916-lgps22.dtb
dtb-$(CONFIG_MACH_MSM8909_CF_LGU_KR)       += msm8909-cf-kr.dtb
dtb-$(CONFIG_MACH_MSM8909_CF_LGU_KR)       += msm8909-pm8916-cf-kr.dtb
dtb-$(CONFIG_MACH_MSM8909_CF_SKT_KR)       += msm8909-cf-kr.dtb
dtb-$(CONFIG_MACH_MSM8909_CF_SKT_KR)       += msm8909-pm8916-cf-kr.dtb
dtb-$(CONFIG_MACH_MSM8909_CF_KT_KR)        += msm8909-cf-kr.dtb
dtb-$(CONFIG_MACH_MSM8909_CF_KT_KR)        += msm8909-pm8916-cf-kr.dtb
dtb-$(CONFIG_MACH_MSM8909_CF_GLOBAL_COM)        += msm8909-cf_global_com.dtb
dtb-$(CONFIG_MACH_MSM8909_CF_GLOBAL_COM)        += msm8909-pm8916-cf_global_com.dtb
dtb-$(CONFIG_MACH_MSM8909_CF_JCM_JP)       += msm8909-cf_jcm_jp.dtb
dtb-$(CONFIG_MACH_MSM8909_CF_JCM_JP)       += msm8909-pm8916-cf_jcm_jp.dtb


DTB_NAMES := $(subst $\",,$(CONFIG_BUILD_ARM_APPENDED_DTB_IMAGE_NAMES))
ifneq ($(DTB_NAMES),)
DTB_LIST := $(addsuffix .dtb,$(DTB_NAMES))
else
DTB_LIST := $(dtb-y)
endif

targets += dtbs
endif

DTS_TARGET := $(patsubst "%",%,$(call CONFIG_DTS_TARGET,$(1)))

$(obj)/../%.dtb: $(src)/$(DTS_TARGET)/%.dts
	$(call if_changed_dep,dtc)

dtbs: $(addprefix $(obj)/../,$(DTB_LIST))

clean-files := *.dtb
