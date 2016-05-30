ifeq ($(CONFIG_OF),y)

dtb-$(CONFIG_MACH_MSM8916_C50DS_GLOBAL_COM)	+= msm8916-c50ds.dtb
dtb-$(CONFIG_MACH_MSM8916_C50N_GLOBAL_COM)	+= msm8916-c50n.dtb
dtb-$(CONFIG_MACH_MSM8916_C50_GLOBAL_COM)	+= msm8916-c50.dtb
dtb-$(CONFIG_MACH_MSM8916_C50_TRF_US)	+= msm8916-c50.dtb
dtb-$(CONFIG_MACH_MSM8916_C50_CRK_US)	+= msm8916-c50.dtb
dtb-$(CONFIG_MACH_MSM8916_C50_TMO_US)	+= msm8916-c50.dtb
dtb-$(CONFIG_MACH_MSM8916_C50_SPR_US)	+= msm8916-c50.dtb
dtb-$(CONFIG_MACH_MSM8916_C50_MPCS_US)	+= msm8916-c50.dtb
dtb-$(CONFIG_MACH_MSM8916_C50C_VZW)	+= msm8916-c50c.dtb
dtb-$(CONFIG_MACH_MSM8916_C70N_CRK_US)	+= msm8916-c70n.dtb
dtb-$(CONFIG_MACH_MSM8916_C70N_ATT_US)   += msm8916-c70n.dtb
dtb-$(CONFIG_MACH_MSM8916_C70_USC_US)	+= msm8916-c70.dtb
dtb-$(CONFIG_MACH_MSM8916_C70N_TMO_US)	+= msm8916-c70.dtb
dtb-$(CONFIG_MACH_MSM8916_C70N_MPCS_US)	+= msm8916-c70.dtb
dtb-$(CONFIG_MACH_MSM8916_C70N_GLOBAL_COM)	+= msm8916-c70n.dtb
dtb-$(CONFIG_MACH_MSM8916_C70W_SKT_KR)	+= msm8916-c70w.dtb
dtb-$(CONFIG_MACH_MSM8916_C70W_KT_KR)	+= msm8916-c70w.dtb
dtb-$(CONFIG_MACH_MSM8916_C70W_LGU_KR)	+= msm8916-c70w.dtb
dtb-$(CONFIG_MACH_MSM8916_C70_GLOBAL_COM)	+= msm8916-c70.dtb
dtb-$(CONFIG_MACH_MSM8916_C70DS_GLOBAL_COM)	+= msm8916-c70ds.dtb
dtb-$(CONFIG_MACH_MSM8916_STYLUSC_SPR_US)	+= msm8916-stylusc.dtb
dtb-$(CONFIG_MACH_MSM8916_C30DS_GLOBAL_COM)	+= msm8916-c30ds.dtb
dtb-$(CONFIG_MACH_MSM8916_C30_GLOBAL_COM)    += msm8916-c30.dtb
dtb-$(CONFIG_MACH_MSM8916_C30_TRF_US)    += msm8916-c30.dtb
dtb-$(CONFIG_MACH_MSM8916_C30C_TRF_US)    += msm8916-c30c.dtb
dtb-$(CONFIG_MACH_MSM8916_G4STYLUSN_TMO_US)	+= msm8916-g4stylusn.dtb
dtb-$(CONFIG_MACH_MSM8916_G4STYLUSN_MPCS_US)	+= msm8916-g4stylusn.dtb
dtb-$(CONFIG_MACH_MSM8916_G4STYLUSN_RGS_CA)	+= msm8916-g4stylusn.dtb
dtb-$(CONFIG_MACH_MSM8916_G4STYLUSN_VTR_CA)	+= msm8916-g4stylusn.dtb
dtb-$(CONFIG_MACH_MSM8916_G4STYLUSN_GLOBAL_COM)	+= msm8916-g4stylusn.dtb
dtb-$(CONFIG_MACH_MSM8916_G4STYLUSW_KT_KR)      += msm8916-g4stylusw.dtb
dtb-$(CONFIG_MACH_MSM8916_YG_SKT_KR)      += msm8916-yg.dtb
dtb-$(CONFIG_MACH_MSM8916_G4STYLUSDSN_GLOBAL_COM)	+= msm8916-g4stylusdsn.dtb
dtb-$(CONFIG_MACH_MSM8916_G4STYLUS_CRK_US) += msm8916-g4stylus.dtb
dtb-$(CONFIG_MACH_MSM8916_Y50_TRF_US)	+= msm8916-y50.dtb
dtb-$(CONFIG_MACH_MSM8916_Y50C_TRF_US)	+= msm8916-y50c.dtb
dtb-$(CONFIG_MACH_MSM8916_E7IILTE_SPR_US)	+= msm8916-e7iilte.dtb
dtb-$(CONFIG_MACH_MSM8916_C90NAS_SPR_US)	+= msm8916-c90nas.dtb
dtb-$(CONFIG_MACH_MSM8916_C70_RGS_CA)	+= msm8916-c70.dtb
dtb-$(CONFIG_MACH_MSM8916_C90N_GLOBAL_COM)    += msm8916-c90n.dtb
dtb-$(CONFIG_MACH_MSM8939_P1BC_SPR_US)    += msm8939-p1bc.dtb
dtb-$(CONFIG_MACH_MSM8939_P1BDSN_GLOBAL_COM)    += msm8939-p1bdsn.dtb
dtb-$(CONFIG_MACH_MSM8939_P1BSSN_SKT_KR)    += msm8939-p1bssn.dtb
dtb-$(CONFIG_MACH_MSM8939_P1BSSN_BELL_CA)    += msm8939-p1bssn.dtb
dtb-$(CONFIG_MACH_MSM8939_P1BSSN_VTR_CA)    += msm8939-p1bssn.dtb

DTB_NAMES := $(subst $\",,$(CONFIG_BUILD_ARM_APPENDED_DTB_IMAGE_NAMES))
ifneq ($(DTB_NAMES),)
DTB_LIST := $(addsuffix .dtb,$(DTB_NAMES))
else
DTB_LIST := $(dtb-y)
endif

targets += dtbs
endif
#DTS_TARGET := $(patsubst "%",%,$(call CONFIG_DTS_TARGET,$(1)))
ifeq ($(CONFIG_MACH_MSM8916_C50_TRF_US),y)
	DTS_TARGET := msm8916-c50_trf_us
else ifeq ($(CONFIG_MACH_MSM8916_C50_CRK_US),y)
	DTS_TARGET := msm8916-c50_crk_us
else ifeq ($(CONFIG_MACH_MSM8916_C30_TRF_US),y)
	DTS_TARGET := msm8916-c30_trf_us
else ifeq ($(CONFIG_MACH_MSM8916_C30C_TRF_US),y)
	DTS_TARGET := msm8916-c30c_trf_us
else ifeq ($(CONFIG_MACH_MSM8916_C50_TMO_US),y)
	DTS_TARGET := msm8916-c50_tmo_us
else ifeq ($(CONFIG_MACH_MSM8916_C50_SPR_US),y)
	DTS_TARGET := msm8916-c50_spr_us
else ifeq ($(CONFIG_MACH_MSM8916_C50_MPCS_US),y)
	DTS_TARGET := msm8916-c50_mpcs_us
else ifeq ($(CONFIG_MACH_MSM8916_C50C_VZW),y)
	DTS_TARGET := msm8916-c50c_vzw
else ifeq ($(CONFIG_MACH_MSM8916_C70N_CRK_US),y)
	DTS_TARGET := msm8916-c70n_crk_us
else ifeq ($(CONFIG_MACH_MSM8916_C70N_ATT_US),y)
    DTS_TARGET := msm8916-c70n_att_us
else ifeq ($(CONFIG_MACH_MSM8916_C70_USC_US),y)
	DTS_TARGET := msm8916-c70_usc_us
else ifeq ($(CONFIG_MACH_MSM8916_C70N_TMO_US),y)
	DTS_TARGET := msm8916-c70n_tmo_us
else ifeq ($(CONFIG_MACH_MSM8916_C70N_MPCS_US),y)
	DTS_TARGET := msm8916-c70n_mpcs_us
else ifeq ($(CONFIG_MACH_MSM8916_C70N_GLOBAL_COM),y)
	DTS_TARGET := msm8916-c70n_global_com
else ifeq ($(CONFIG_MACH_MSM8916_C70W_SKT_KR),y)
	DTS_TARGET := msm8916-c70w_kr_com
else ifeq ($(CONFIG_MACH_MSM8916_C70W_KT_KR),y)
	DTS_TARGET := msm8916-c70w_kr_com
else ifeq ($(CONFIG_MACH_MSM8916_C70W_LGU_KR),y)
	DTS_TARGET := msm8916-c70w_kr_com
else ifeq ($(CONFIG_MACH_MSM8916_C70_GLOBAL_COM),y)
	DTS_TARGET := msm8916-c70_global_com
else ifeq ($(CONFIG_MACH_MSM8916_C70DS_GLOBAL_COM),y)
	DTS_TARGET := msm8916-c70ds_global_com
else ifeq ($(CONFIG_MACH_MSM8916_STYLUSC_SPR_US),y)
	DTS_TARGET := msm8916-stylusc_spr_us
else ifeq ($(CONFIG_MACH_MSM8916_C30DS_GLOBAL_COM),y)
	DTS_TARGET := msm8916-c30ds_global_com
else ifeq ($(CONFIG_MACH_MSM8916_C30_GLOBAL_COM),y)
    DTS_TARGET := msm8916-c30_global_com
else ifeq ($(CONFIG_MACH_MSM8916_C50N_GLOBAL_COM),y)
	DTS_TARGET := msm8916-c50n_global_com
else ifeq ($(CONFIG_MACH_MSM8916_C50_GLOBAL_COM),y)
	DTS_TARGET := msm8916-c50_global_com
else ifeq ($(CONFIG_MACH_MSM8916_C50DS_GLOBAL_COM),y)
	DTS_TARGET := msm8916-c50ds_global_com
else ifeq ($(CONFIG_MACH_MSM8916_G4STYLUSN_TMO_US),y)
	DTS_TARGET := msm8916-g4stylusn_tmo_us
else ifeq ($(CONFIG_MACH_MSM8916_G4STYLUSN_MPCS_US),y)
	DTS_TARGET := msm8916-g4stylusn_mpcs_us
else ifeq ($(CONFIG_MACH_MSM8916_G4STYLUSN_RGS_CA),y)
	DTS_TARGET := msm8916-g4stylusn_rgs_ca
else ifeq ($(CONFIG_MACH_MSM8916_G4STYLUSN_VTR_CA),y)
	DTS_TARGET := msm8916-g4stylusn_vtr_ca
else ifeq ($(CONFIG_MACH_MSM8916_G4STYLUSN_GLOBAL_COM),y)
	DTS_TARGET := msm8916-g4stylusn_global_com
else ifeq ($(CONFIG_MACH_MSM8916_G4STYLUSW_KT_KR),y)
  DTS_TARGET := msm8916-g4stylusw_kt_kr
else ifeq ($(CONFIG_MACH_MSM8916_YG_SKT_KR),y)
  DTS_TARGET := msm8916-yg_skt_kr
else ifeq ($(CONFIG_MACH_MSM8916_G4STYLUSDSN_GLOBAL_COM),y)
	DTS_TARGET := msm8916-g4stylusdsn_global_com
else ifeq ($(CONFIG_MACH_MSM8916_G4STYLUS_CRK_US),y)
	DTS_TARGET := msm8916-g4stylus_crk_us
else ifeq ($(CONFIG_MACH_MSM8916_Y50_TRF_US),y)
	DTS_TARGET := msm8916-y50_trf_us
else ifeq ($(CONFIG_MACH_MSM8916_Y50C_TRF_US),y)
	DTS_TARGET := msm8916-y50c_trf_us
else ifeq ($(CONFIG_MACH_MSM8916_E7IILTE_SPR_US),y)
	DTS_TARGET := msm8916-e7iilte_spr_us
else ifeq ($(CONFIG_MACH_MSM8916_C90NAS_SPR_US),y)
	DTS_TARGET := msm8916-c90nas_spr_us
else ifeq ($(CONFIG_MACH_MSM8916_C70_RGS_CA),y)
	DTS_TARGET := msm8916-c70_rgs_ca
else ifeq ($(CONFIG_MACH_MSM8916_C90N_GLOBAL_COM),y)
	DTS_TARGET := msm8916-c90n_global_com
else ifeq ($(CONFIG_MACH_MSM8939_P1BC_SPR_US),y)
        DTS_TARGET := msm8939-p1bc_spr_us
else ifeq ($(CONFIG_MACH_MSM8939_P1BDSN_GLOBAL_COM),y)
	DTS_TARGET := msm8939-p1bdsn_global_com
else ifeq ($(CONFIG_MACH_MSM8939_P1BSSN_SKT_KR),y)
        DTS_TARGET := msm8939-p1bssn_skt_kr
else ifeq ($(CONFIG_MACH_MSM8939_P1BSSN_BELL_CA),y)
        DTS_TARGET := msm8939-p1bssn_bell_ca
else ifeq ($(CONFIG_MACH_MSM8939_P1BSSN_VTR_CA),y)
        DTS_TARGET := msm8939-p1bssn_vtr_ca
else
	DTS_TARGET := msm8916-c50ds_global_com
endif

$(obj)/../%.dtb: $(src)/$(DTS_TARGET)/%.dts
	$(call if_changed_dep,dtc)

dtbs: $(addprefix $(obj)/../,$(DTB_LIST))

clean-files := *.dtb
