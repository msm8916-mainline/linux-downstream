#ifndef _LINUX_ELAN_KTF_H
#define _LINUX_ELAN_KTF_H

//#define __devexit
//#define __devinitdata
//#define __devinit 
//#define __devexit_p

#define ELAN_X_MAX      1280
#define ELAN_Y_MAX      2112
#define ELAN_X_MAX_370T  2112
#define ELAN_Y_MAX_370T  1280
#define ELAN_X_MAX_571K 2240
#define ELAN_Y_MAX_571K 1344


//#define ELAN_KTF_NAME "elan-ktf"

#define ELAN_KTF_NAME "ekth3260"

struct elan_ktf_i2c_platform_data {
	uint16_t version;
	int abs_x_min;
	int abs_x_max;
	int abs_y_min;
	int abs_y_max;
	int intr_gpio;
	int rst_gpio;
        int mode_check_gpio;
	int (*power)(int on);
};

#endif /* _LINUX_ELAN_KTF_H */
