#ifndef __HARDWARE_INFO_H_
#define __HARDWARE_INFO_H_


enum hw_info {
    ODMM_HD_BOARD_VERSION,
	ODMM_HD_INFO_LCD,
	ODMM_HD_INFO_TP,
	ODMM_HD_INFO_BT,
	ODMM_HD_INFO_WIFI,
	ODMM_HD_INFO_G_SENSOR,
	ODMM_HD_INFO_P_SENSOR,
	ODMM_HD_INFO_CAMERA,
	ODMM_HD_INFO_OTG,
	ODMM_HD_INFO_BACKLIGHT,
	ODMM_HD_INFO_GPS,
	ODMM_HD_INFO_FM,
	ODMM_HD_INFO_PMIC,
	ODMM_HD_INFO_EMMC,
	MAX_HW_INFO_DEVICE,
};

typedef struct {
	char IC_type[64] ;
	char IC_id[64] ;
	char vendor[64] ;
	char firmware[64] ;
	char comments[64] ;
} HARDWARE_INFORMATION_OUTPUT_1;

void hw_info_set(enum hw_info dev_id, char *type, char *id, char *ven,
    char *fw, char *cmt);

#endif
