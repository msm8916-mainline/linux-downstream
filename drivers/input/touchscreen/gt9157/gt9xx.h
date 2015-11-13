/* drivers/input/touchscreen/gt9xx.h
 *
 * 2010 - 2014 Goodix Technology.
 * Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */

#ifndef _GOODIX_GT9XX_H_
#define _GOODIX_GT9XX_H_

#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>
#include <linux/debugfs.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#define GOODIX_SUSPEND_LEVEL 1
#endif
/***************************PART1:ON/OFF define*******************************/
#define UPDATE_FUNCTIONS      

#define GTP_CUSTOM_CFG        0
#define GTP_CHANGE_X2Y        0
#define GTP_DRIVER_SEND_CFG   1
#define GTP_HAVE_TOUCH_KEY    1
#define GTP_POWER_CTRL_SLEEP  1
#define GTP_ICS_SLOT_REPORT   1

#define GTP_AUTO_UPDATE       1	/* auto update fw by .bin file as default */
#define GTP_HEADER_FW_UPDATE  0	/* auto update fw by gtp_default_FW
				  * in gt9xx_firmware.h, function together
				  * with GTP_AUTO_UPDATE
				  */
#define GTP_AUTO_UPDATE_CFG   1 	/* auto update config by .cfg file,
				  *function together with GTP_AUTO_UPDATE
				  */

#define GTP_COMPATIBLE_MODE   0	/* compatible with GT9XXF */

#define GTP_CREATE_WR_NODE    1
#define GTP_ESD_PROTECT       1	/* esd protection with a cycle of 2 seconds */

#define GTP_WITH_PEN          0
#define GTP_PEN_HAVE_BUTTON   0	/* active pen has buttons, function
				  * together with GTP_WITH_PEN
				  */

#define GTP_GESTURE_WAKEUP    0	/* gesture wakeup */

#define GTP_DEBUG_ON          0
#define GTP_DEBUG_ARRAY_ON    0
#define GTP_DEBUG_FUNC_ON     0

#if GTP_COMPATIBLE_MODE
enum chip_type_t {
	CHIP_TYPE_GT9 = 0,
	CHIP_TYPE_GT9F = 1,
};
#endif
#define GOODIX_MAX_CFG_GROUP	6
#define GTP_FW_NAME_MAXSIZE	50
struct goodix_ts_platform_data {
	int irq_gpio;
	u32 irq_gpio_flags;
	int reset_gpio;
	u32 reset_gpio_flags;
	u32 num_max_touches;
	u32 soft_rst_dly;
	const char *product_id;
	const char *fw_name;
	char *name;
	u32 x_max;
	u32 y_max;
	u32 x_min;
	u32 y_min;
	u32 panel_minx;
	u32 panel_miny;
	u32 panel_maxx;
	u32 panel_maxy;
	bool no_force_update;
	bool i2c_pull_up;
	bool enable_power_off;
	bool fw_vkey_support;
	int config_data_len[GOODIX_MAX_CFG_GROUP];
	u8 *config_data[GOODIX_MAX_CFG_GROUP];
};
struct goodix_ts_data {
	spinlock_t irq_lock;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct goodix_ts_platform_data *pdata;
	struct hrtimer timer;
	struct work_struct work;
	u8 enable;
	u16 addr;
	s32 irq_is_disable;
	s32 use_irq;
	u16 abs_x_max;
	u16 abs_y_max;
	u8 max_touch_num;
	u8 int_trigger_type;
	u8 green_wake_mode;
	u8 enter_update;
	u8 gtp_is_suspend;
	u8 gtp_rawdiff_mode;
	u8 gtp_cfg_len;
	u8 fixed_cfg;
	u8 fw_error;
	u8 pnl_init_error;
	bool fw_loading;
	char fw_name[GTP_FW_NAME_MAXSIZE];
	char *ts_info;
	bool force_update;
	struct regulator *vdd;
	struct regulator *vcc_i2c;
	bool power_on;
#if GTP_WITH_PEN
	struct input_dev *pen_dev;
#endif

#if GTP_ESD_PROTECT
	spinlock_t esd_lock;
	u8 esd_running;
	s32 clk_tick_cnt;
#endif

#if GTP_COMPATIBLE_MODE
	u16 bak_ref_len;
	s32 ref_chk_fs_times;
	s32 clk_chk_fs_times;
	enum chip_type_t chip_type;
	u8 rqst_processing;
	u8 is_950;
#endif
	struct mutex lock;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	struct dentry *debug_base;
};

extern u16 show_len;
extern u16 total_len;

/******************* PART2:TODO define *************************/
/* STEP_1(REQUIRED): Define Configuration Information Group(s)
    Sensor_ID Map:
   sensor_opt1 sensor_opt2 Sensor_ID
    GND         GND         0
    VDDIO       GND         1
    NC          GND         2
    GND         NC/300K     3
    VDDIO       NC/300K     4
    NC          NC/300K     5
*/
/* TODO: define your own default or for Sensor_ID == 0 config here. */
/* The predefined one is just a sample config,
  * which is not suitable for your tp in most cases.
  */
/*
#define CTP_CFG_GROUP1 {\
	0x43, 0xD0, 0x02, 0x00, 0x05, 0x05, 0x35, 0xE3, 0x01, 0x08,\
	0x1E, 0x0F, 0x50, 0x32, 0x03, 0x05, 0x00, 0x00, 0xFF, 0x7F,\
	0x00, 0x00, 0x01, 0x17, 0x19, 0x1D, 0x14, 0x8A, 0x0A, 0x0B,\
	0x28, 0x00, 0x7C, 0x06, 0x00, 0x00, 0x00, 0x83, 0x33, 0x1D,\
	0x28, 0x01, 0x00, 0x00, 0x00, 0x1E, 0x1E, 0x00, 0x00, 0x14,\
	0x2A, 0x1E, 0x37, 0x94, 0xC5, 0x02, 0x08, 0x14, 0x00, 0x04,\
	0x7F, 0x1F, 0x00, 0x70, 0x24, 0x00, 0x66, 0x28, 0x00, 0x5C,\
	0x2D, 0x00, 0x56, 0x33, 0x00, 0x56, 0x18, 0x30, 0x48, 0x00,\
	0xF0, 0x4A, 0x3A, 0xFF, 0xFF, 0x27, 0x0F, 0x00, 0x00, 0x00,\
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00,\
	0x50, 0x32, 0x16, 0x14, 0x12, 0x10, 0x0E, 0x0C, 0x0A, 0x08,\
	0x06, 0x04, 0x02, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,\
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
	0x00, 0x00, 0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0F, 0x10,\
	0x12, 0x13, 0x16, 0x18, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21,\
	0x22, 0x24, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00,\
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,\
	0xFF, 0xFF, 0xFF, 0xFF, 0xB3, 0x01\
}
*/
#define CTP_CFG_GROUP1 {\
0x43, 0xD0, 0x02, 0x00, 0x05, 0x05, 0x35, 0xC1, 0x01, 0x0F,\
0x1E, 0x0F, 0x50, 0x3C, 0x03, 0x05, 0x00, 0x00, 0xFF, 0x7F,\
0x00, 0x00, 0x01, 0x17, 0x19, 0x1D, 0x14, 0x8A, 0x0A, 0x0B,\
0x44, 0x00, 0x97, 0x05, 0x00, 0x0A, 0x00, 0x83, 0x32, 0x1D,\
0x28, 0x0D, 0x00, 0x00, 0x00, 0x1E, 0x1E, 0x00, 0x00, 0x14,\
0x2B, 0x37, 0x55, 0x94, 0xC5, 0x02, 0x08, 0x14, 0x00, 0x04,\
0x93, 0x39, 0x00, 0x89, 0x3E, 0x00, 0x7F, 0x44, 0x00, 0x77,\
0x4A, 0x00, 0x70, 0x51, 0x00, 0x70, 0x18, 0x30, 0x48, 0x00,\
0xF0, 0x4A, 0x3A, 0xDD, 0xDD, 0x27, 0x0F, 0x00, 0x00, 0x00,\
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00,\
0x46, 0x32, 0x16, 0x14, 0x12, 0x10, 0x0E, 0x0C, 0x0A, 0x08,\
0x06, 0x04, 0x02, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,\
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
0x00, 0x00, 0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0F, 0x10,\
0x12, 0x13, 0x16, 0x18, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21,\
0x22, 0x24, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00,\
0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,\
0xFF, 0xFF, 0xFF, 0xFF, 0x6F, 0x01\
}
/* TODO: define your config for Sensor_ID == 1 here, if needed */
#define CTP_CFG_GROUP2 {\
}

/* TODO: define your config for Sensor_ID == 2 here, if needed */
#define CTP_CFG_GROUP3 {\
}

/* TODO: define your config for Sensor_ID == 3 here, if needed */
#define CTP_CFG_GROUP4 {\
}

/* TODO: define your config for Sensor_ID == 4 here, if needed */
#define CTP_CFG_GROUP5 {\
}

/* TODO: define your config for Sensor_ID == 5 here, if needed */
#define CTP_CFG_GROUP6 {\
}

/* STEP_2(REQUIRED): Customize your I/O ports & I/O operations */
#define GTP_IRQ_TAB	{\
	IRQ_TYPE_EDGE_RISING, \
	IRQ_TYPE_EDGE_FALLING, \
	IRQ_TYPE_LEVEL_LOW, \
	IRQ_TYPE_LEVEL_HIGH }

/* STEP_3(optional): Specify your special config info if needed */
#if GTP_CUSTOM_CFG
#define GTP_MAX_HEIGHT   800
#define GTP_MAX_WIDTH    480
#define GTP_INT_TRIGGER  0	/* 0: Rising 1: Falling */
#else
#define GTP_MAX_HEIGHT   4096
#define GTP_MAX_WIDTH    4096
#define GTP_INT_TRIGGER  1
#endif
#define GTP_MAX_TOUCH    5

/* STEP_4(optional): If keys are available and
  * reported as keys, config your key info here
  */
#if GTP_HAVE_TOUCH_KEY
#define GTP_KEY_TAB  {KEY_MENU, KEY_HOMEPAGE, KEY_BACK}
#endif

/***************************PART3:OTHER define*********************************/
#define GTP_DRIVER_VERSION          "V2.2<2014/01/14>"
#define GTP_I2C_NAME                "Goodix-TS"
#define GT91XX_CONFIG_PROC_FILE     "gt9xx_config"
#define GTP_POLL_TIME         10
#define GTP_ADDR_LENGTH       2
#define GTP_CONFIG_MIN_LENGTH 186
#define GTP_CONFIG_MAX_LENGTH 240
#define FAIL                  0
#define SUCCESS               1
#define SWITCH_OFF            0
#define SWITCH_ON             1

/******************** For GT9XXF Start **********************/
#define GTP_REG_BAK_REF                 0x99D0
#define GTP_REG_MAIN_CLK                0x8020
#define GTP_REG_CHIP_TYPE               0x8000
#define GTP_REG_HAVE_KEY                0x804E
#define GTP_REG_MATRIX_DRVNUM           0x8069
#define GTP_REG_MATRIX_SENNUM           0x806A

#define GTP_FL_FW_BURN              0x00
#define GTP_FL_ESD_RECOVERY         0x01
#define GTP_FL_READ_REPAIR          0x02

#define GTP_BAK_REF_SEND                0
#define GTP_BAK_REF_STORE               1
#define CFG_LOC_DRVA_NUM                29
#define CFG_LOC_DRVB_NUM                30
#define CFG_LOC_SENS_NUM                31

#define GTP_CHK_FW_MAX                  40
#define GTP_CHK_FS_MNT_MAX              300
#define GTP_BAK_REF_PATH                "/data/gtp_ref.bin"
#define GTP_MAIN_CLK_PATH               "/data/gtp_clk.bin"
#define GTP_RQST_CONFIG                 0x01
#define GTP_RQST_BAK_REF                0x02
#define GTP_RQST_RESET                  0x03
#define GTP_RQST_MAIN_CLOCK             0x04
#define GTP_RQST_RESPONDED              0x00
#define GTP_RQST_IDLE                   0xFF

/******************** For GT9XXF End **********************/
/* Registers define */
#define GTP_READ_COOR_ADDR	0x814E
#define GTP_REG_SLEEP		0x8040
#define GTP_REG_SENSOR_ID	0x814A
#define GTP_REG_CONFIG_DATA	0x8047
#define GTP_REG_VERSION		0x8140
#define GTP_VALID_ADDR_START	0x8040
#define GTP_VALID_ADDR_END	0x8177

#define RESOLUTION_LOC		3
#define TRIGGER_LOC		8

#define CFG_GROUP_LEN(p_cfg_grp)  (sizeof(p_cfg_grp) / sizeof(p_cfg_grp[0]))
/* Log define */
#define GTP_INFO(fmt, arg...) pr_info("<<-GTP-INFO->> "fmt"\n", ##arg)
#define GTP_ERROR(fmt, arg...) pr_err("<<-GTP-ERROR->> "fmt"\n", ##arg)
#define GTP_DEBUG(fmt, arg...) \
	do {\
		if (GTP_DEBUG_ON)\
			pr_debug("<<-GTP-DEBUG->> [%d]"fmt"\n",\
					(int)__LINE__, ##arg);\
	} while (0)
#define GTP_DEBUG_ARRAY(array, num) \
	do {\
		s32 i;\
		u8 *a = array;\
		if (GTP_DEBUG_ARRAY_ON) {\
			pr_debug("<<-GTP-DEBUG-ARRAY->>\n");\
			for (i = 0; i < (num); i++) {\
				pr_debug("%02x   ", (a)[i]);\
				if ((i + 1) % 10 == 0) \
					pr_debug("\n");\
			} \
			pr_debug("\n");\
		} \
	} while (0)
#define GTP_DEBUG_FUNC() \
	do {\
		if (GTP_DEBUG_FUNC_ON)\
			pr_debug("<<-GTP-FUNC->> Func:%s@Line:%d\n",\
					__func__, __LINE__);\
	} while (0)
#define GTP_SWAP(x, y) \
	do {\
		typeof(x) z = x;\
		x = y;\
		y = z;\
	} while (0)

/*****************************End of Part III********************************/
extern struct i2c_client *i2c_connect_client;
extern u8 config[GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH];
void gtp_reset_guitar(struct i2c_client *client, s32 ms);
s32 gtp_send_cfg(struct i2c_client *client);
s32 gtp_read_version(struct i2c_client *, u16 *);
s32 gtp_i2c_read_dbl_check(struct i2c_client *, u16, u8 *, int);

#ifdef UPDATE_FUNCTIONS
s32 gup_enter_update_mode(struct i2c_client *client);
void gup_leave_update_mode(void);
s32 gup_update_proc(void *dir);
#endif

void gtp_irq_disable(struct goodix_ts_data *);
void gtp_irq_enable(struct goodix_ts_data *);

#if GTP_ESD_PROTECT
void gtp_esd_switch(struct i2c_client *, s32);
#endif

#ifdef CONFIG_GT9XX_TOUCHPANEL_DEBUG
#if GTP_CREATE_WR_NODE
s32 init_wr_node(struct i2c_client *);
void uninit_wr_node(void);
#endif
#endif

#if GTP_AUTO_UPDATE
extern u8 gup_init_update_proc(struct goodix_ts_data *);
#endif

#if GTP_COMPATIBLE_MODE
s32 i2c_read_bytes(struct i2c_client *client, u16 addr, u8 *buf,
			  s32 len);
s32 i2c_write_bytes(struct i2c_client *client, u16 addr, u8 *buf,
			   s32 len);
s32 gup_clk_calibration(void);
s32 gup_fw_download_proc(void *dir, u8 dwn_mode);
u8 gup_check_fs_mounted(char *path_name);
void gtp_recovery_reset(struct i2c_client *client);
s32 gtp_fw_startup(struct i2c_client *client);
#endif

#endif /* _GOODIX_GT9XX_H_ */
