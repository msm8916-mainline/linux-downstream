#ifndef __CAMERA_INFO_H_
#define __CAMERA_INFO_H_


enum camera_info {
	ODMM_HD_INFO_CAMERA_FRONT,
	ODMM_HD_INFO_CAMERA_REAR,
	MAX_CAMERA_INFO_DEVICE,
	};

typedef struct {
	char IC_type[64] ;
	char IC_id[64] ;
	char vendor[64] ;
	char comments[64] ;
} CAMERA_INFORMATION_OUTPUT_1;

void camera_info_set(enum camera_info dev_id, char *type, char *id, char *ven,
     char *cmt);

#endif
