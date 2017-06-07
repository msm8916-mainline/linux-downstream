/*
 * include/linux/lu2010.h
 *
 * Copyright (C) 2011 lu2010, Inc.
 *
 */
#ifndef 	__LU2010_H
#define	__LU2010_H

#define LGE_TOUCH_NAME	"lge_touch"

#define LU201X_MAX_KEY 4
#define MAX_FINGER_NUM 2
#define MAX_CHANNEL 34

/* LeadingUI Firmware */
#define FW_SIZE 		30*1024
#define CFG_SIZE 		1*1024
#define FAC_SIZE		1*1024

#define FAC_POS			0xFC00
#define FW_POS			0x8000

/* Knock On/Code */
#define KNOCK_ON_STAUS		0x0082
#define KNOCK_TAP_COUNT		0x0083
#define KNOCK_STATUS		0x00C0
#define KNOCK_TAP_THON		0x00C1
#define KNOCK_EXCEPT_PALM_ONCH	0x00C5
#define KNOCK_WAKEUP_INTERVAL	0x00C9
#define KNOCK_TAPOFF_TIMEOUT	0x00D2
#define KNOCK_ON_TAP_COUNT	0x00D4

/* Touch Event Type */
#define TTYPE_PRESS		0x01
#define TTYPE_MOVE		0x02
#define TTYPE_RELEASE		0x03

/* Key Event Type */
#define KEY_PRESSED		1
#define KEY_RELEASED		0
#define CANCEL_KEY		0xFF

#define SCREEN_MAX_X    	1280
#define SCREEN_MAX_Y    	800
#define PRESS_MAX       	255

#define EVENT_NONE		0x00
#define EVENT_ABS		0x01
#define EVENT_KEY		0x02
#define EVENT_GEST		0x04
#define EVENT_MOUSE		0x08

#define FWSTATUS_NORMAL		0x00
#define FWSTATUS_INITREQ	0xFF
#define FWSTATUS_CHFAIL		0xfe
#define FWSTATUS_CALFAIL	0xfd

#define I2C_DEVICE_ADDRESS_LEN	2
#define MAX_TRANSACTION_LENGTH	8

#define FW_STATUS_REG		0x0000
#define FW_VERSION_REG		0x0080

#define LU201x_MODE_ADDR	0x00E0
#define LU201x_CMDACK_ADDR	0x00ED
#define LU201x_DEVICEID_ADDR	0x10FD
#define LU201x_I2CDONE_ADDR	0x10FF
#define LU201x_CMDReply_ADDR	0x0100

#define CMD_I2C_DONE		0x01
#define CMD_LU201x_CHANGEMODE	0xA3
#define CMD_LU201x_CHCAPTEST	0xB6
#define LU201x_CHCAPTEST_Reply	0xC1

/* Major Mode */
#define CMD_LU201x_NORMODE		0x00
#define CMD_LU201x_PDN			0x01
#define CMD_LU201x_DEBUG		0x02
#define CMD_LU201x_IDLE_DOUBLETAB	0x11
#define CMD_LU201x_IDLE_MULTITAB	0x11

/* Minor Mode */
#define CMD_LU201x_NONE		0x0000
#define CMD_LU201x_REFCAP	0x0000

enum {
	LPWG_READ = 1,
	LPWG_ENABLE,
	LPWG_LCD_X,
	LPWG_LCD_Y,
	LPWG_ACTIVE_AREA_X1,
	LPWG_ACTIVE_AREA_X2,
	LPWG_ACTIVE_AREA_Y1,
	LPWG_ACTIVE_AREA_Y2,
	LPWG_TAP_COUNT,
	LPWG_REPLY,
	LPWG_UPDATE_ALL,
};

enum {
	CMD_LPWG_ENABLE = 1,
	CMD_LPWG_LCD = 2,
	CMD_LPWG_ACTIVE_AREA = 3,
	CMD_LPWG_TAP_COUNT = 4,
	CMD_LPWG_LCD_RESUME_SUSPEND = 6,
	CMD_LPWG_PROX = 7,
	CMD_LPWG_DOUBLE_TAP_CHECK = 8,
	CMD_LPWG_TOTAL_STATUS = 9,
};

enum {
	LPWG_NONE = 0,
	LPWG_DOUBLE_TAP,
	LPWG_MULTI_TAP,
};

enum flash_update_mode {
	NORMAL = 0,
	FIRMWARE
};

enum {
	LCD_OFF = 0,
	LCD_ON,
};

enum power_mode {
	POWER_OFF = 0,
	POWER_ON,
	POWER_SLEEP,
	POWER_WAKE
};

enum {
	GET_REFERENCE = 0,
	GET_DELTA
};

struct lu201x_fw_info {
	u8 fw_ver[2];
	u8 rel_ver[2];

	u32 firmware_crc;
	u8 *fac_raw_data; /* start address of factory data */
	u8 *cfg_raw_data;	/* start address of configuration data */
	u8 *fw_raw_data;	/* start address of firmware data */
	struct mxt_data *data;
};

struct point {
    int x;
    int y;
};

struct point_data {
	u8 status;
	u8 id;
	u16 x;
	u16 y;
};

struct ts_event {
	u16 touch_point;
	u16 prev_touch_point;
	struct point_data point[MAX_FINGER_NUM];
	struct point_data prev_point[MAX_FINGER_NUM];
};

struct LU201xTPD_INFO {
	u8 FWStatus;		// 0x0000
	u8 EventType;		// 0x0001
	u8 VPCount;		// 0x0002
	u8 KeyData[2];		// 0x0003
	u8 Point[8];		// 0x0005 X0 position
};

struct lu201x_platform_data {
	u8 num_touch;
	int model;

	int gpio_reset;
	int gpio_int;
	int ldo_vdd_en;
	int ldo_vio_en;

	unsigned int lcd_max_x;
	unsigned int lcd_max_y;

	const char *fw_name;
	u8 num_keys;
	u8 key_maps[LU201X_MAX_KEY];

	/* LPWG */
	int lpwg_panel_on;
	int lpwg_prox;
	int report_enable;
};

struct lu201x_data {
	struct i2c_client		*client;
	struct input_dev		*input_dev;
	struct lu201x_platform_data 	*pdata;
	struct ts_event			event;
	struct work_struct 		pen_event_work;
	struct workqueue_struct 	*ts_workqueue;
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend		early_suspend;
#endif
#if defined (CONFIG_FB)
	struct notifier_block fb_notif;
#endif
	struct kobject			lge_touch_kobj;
	int				irq;

	struct regulator 		*vdd_io;
	bool 				use_regulator;

	struct LU201xTPD_INFO 		LU201x_tpd;
	struct lu201x_fw_info		*fw_info;
	u8 				reportd_keycode;
	u8				panel_on;

	u8				need_fw;
};

#define TOUCH_INFO_MSG(fmt, args...) 	printk(KERN_ERR "[Touch] " fmt, ##args)
#define TOUCH_ERR_MSG(fmt, args...) printk(KERN_ERR "[Touch E] [%s %d] " fmt, __FUNCTION__, __LINE__, ##args)
#define TOUCH_PATCH_INFO_MSG(fmt, args...) 	printk(KERN_ERR "[Touch Patch] " fmt, ##args)

/****************************************************************************
* Touch FUNCTION Declaration
****************************************************************************/
void LU201x_reset(struct lu201x_data *data, unsigned int on);
void LU201x_power(struct lu201x_data *data, unsigned int on);
static int touch_knock_check(struct lu201x_data *data, u8 tap);

#endif /* _LU2010_TOUCH_H */