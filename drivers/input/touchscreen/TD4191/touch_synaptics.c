/* Touch_synaptics.c
 *
 * Copyright (C) 2013 LGE.
 *
 * Author: yehan.ahn@lge.com, hyesung.shin@lge.com
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/regulator/machine.h>
#include <linux/async.h>
#include <linux/atomic.h>
#include <linux/file.h>     /*for file access*/
#include <linux/syscalls.h> /*for file access*/
#include <linux/uaccess.h>  /*for file access*/
#include <linux/workqueue.h>
#include <linux/wakelock.h>

#include <linux/input/TD4191/lge_touch_core.h>
#include <linux/input/TD4191/touch_synaptics.h>
#include <linux/firmware.h>
#include "./DS5/RefCode_F54.h"
#include <mach/board_lge.h>

/* RMI4 spec from 511-000405-01 Rev.D
 * Function	Purpose				See page
 * $01		RMI Device Control			45
 * $1A		0-D capacitive button sensors	61
 * $05		Image Reporting			68
 * $07		Image Reporting			75
 * $08		BIST				82
 * $09		BIST				87
 * $11		2-D TouchPad sensors		93
 * $19		0-D capacitive button sensors	141
 * $30		GPIO/LEDs			148
 * $31		LEDs				162
 * $34		Flash Memory Management		163
 * $36		Auxiliary ADC			174
 * $54		Test Reporting			176
 */
struct i2c_client *ds4_i2c_client;
static char power_state;
static char *productcode_parse(unsigned char *product);
static int get_ic_info(struct synaptics_ts_data *ts);
static int read_page_description_table(struct i2c_client *client);

int f54_window_crack_check_mode;
int f54_window_crack;
int after_crack_check;
bool touch_irq_mask = 1;
static unsigned long im_sum = 0, cns_sum = 0,
		     cid_im_sum = 0, freq_scan_im_sum = 0;
static u16 im_aver, cns_aver, freq_scan_im_aver;
static unsigned int cnt;

/*static int ts_suspend = 0;
int thermal_status = 0;
extern int touch_thermal_mode;*/
u8 default_finger_amplitude[2] = {0, };
extern int touch_ta_status;
struct synaptics_ts_f12_info {
	bool ctrl_reg_is_present[32];
	bool data_reg_is_present[16];
	u8 ctrl_reg_addr[32];
	u8 data_reg_addr[16];
};

static struct synaptics_ts_f12_info f12_info;

struct synaptics_ts_data *syna_ts_data = NULL;

#if defined(CONFIG_LGD_INCELL_PHASE3_VIDEO_HD_PT_PANEL)
extern void mdss_lcd_dsv_control(int enable);
#endif
#define RMI_DEVICE_CONTROL			0x01
#define TOUCHPAD_SENSORS			0x12
#define FLASH_MEMORY_MANAGEMENT			0x34
#define VIDEO_CONTROL				0x38
#define ANALOG_CONTROL				0x54
#define CUSTOM_CONTROL				0x51
#define SENSOR_CONTROL				0x55
#define F1A_EXIST                               0x1A

bool wakeup_by_swipe;

/* Register Map & Register bit mask
 * - Please check "One time" this map before using this device driver
 */
/* RMI_DEVICE_CONTROL */
/* Manufacturer ID */
#define MANUFACTURER_ID_REG		(ts->common_fc.dsc.query_base)
/* CUSTOMER_FAMILY QUERY */
#define CUSTOMER_FAMILY_REG		(ts->common_fc.dsc.query_base + 2)
/* FW revision */
#define FW_REVISION_REG			(ts->common_fc.dsc.query_base + 3)
/* Product ID */
#define PRODUCT_ID_REG			(ts->common_fc.dsc.query_base + 11)
#define DEVICE_COMMAND_REG		(ts->common_fc.dsc.command_base)

/* Device Control */
#define DEVICE_CONTROL_REG		(ts->common_fc.dsc.control_base)
/* sleep mode : go to doze mode after 500 ms */
#define DEVICE_CONTROL_NORMAL_OP	0x00
/* sleep mode : go to sleep */
#define DEVICE_CONTROL_SLEEP		0x01
/* sleep mode : go to sleep. no-recalibration */
#define DEVICE_CONTROL_SLEEP_NO_RECAL	0x02
#define DEVICE_CONTROL_NOSLEEP		0x04
#define DEVICE_CHARGER_CONNECTED	0x20
#define DEVICE_CONTROL_CONFIGURED	0x80

/* Interrupt Enable 0 */
#define INTERRUPT_ENABLE_REG		(ts->common_fc.dsc.control_base + 1)
/* Doze Interval : unit 10ms */
#define DOZE_INTERVAL_REG               (ts->common_fc.dsc.control_base + 2)
#define DOZE_WAKEUP_THRESHOLD_REG       (ts->common_fc.dsc.control_base + 3)

/* Device Status */
#define DEVICE_STATUS_REG		(ts->common_fc.dsc.data_base)
#define DEVICE_FAILURE_MASK		0x03
#define DEVICE_CRC_ERROR_MASK		0x04
#define DEVICE_STATUS_FLASH_PROG	0x40
#define DEVICE_STATUS_UNCONFIGURED	0x80

/* Interrupt Status */
#define INTERRUPT_STATUS_REG		(ts->common_fc.dsc.data_base + 1)
#define INTERRUPT_MASK_FLASH		0x01
#define INTERRUPT_MASK_STATUS		0x02
#define INTERRUPT_MASK_ABS0		0x04
#define INTERRUPT_MASK_BUTTON		0x10
#define INTERRUPT_MASK_CUSTOM		0x40

/* TOUCHPAD_SENSORS */
#define FINGER_COMMAND_REG		(ts->finger_fc.dsc.command_base)
#define MOTION_SUPPRESSION		(ts->finger_fc.dsc.control_base + 5)
/* f12_info.ctrl_reg_addr[20] */
#define GLOVED_FINGER_MASK		0x20

/* Finger State */
#define OBJECT_TYPE_AND_STATUS_REG	(ts->finger_fc.dsc.data_base)
#define OBJECT_ATTENTION_REG		(ts->finger_fc.dsc.data_base + 2)
/* Finger Data Register */
#define FINGER_DATA_REG_START		(ts->finger_fc.dsc.data_base)
#define REG_OBJECT_TYPE_AND_STATUS	0
#define REG_X_LSB			1
#define REG_X_MSB			2
#define REG_Y_LSB			3
#define REG_Y_MSB			4
#define REG_Z				5
#define REG_WX				6
#define REG_WY				7

#define MAXIMUM_XY_COORDINATE_REG	(ts->finger_fc.dsc.control_base)

/* ANALOG_CONTROL */
#define ANALOG_COMMAND_REG		(ts->analog_fc.dsc.command_base)
#define ANALOG_CONTROL_REG		(ts->analog_fc.dsc.control_base)
#define THERMAL_UPDATE_INTERVAL_REG     0x2F      /* 1-page */

/* FLASH_MEMORY_MANAGEMENT */
/* Flash Control */
#define FLASH_CONFIG_ID_REG		(ts->flash_fc.dsc.control_base)
#define FLASH_CONTROL_REG		(ts->flash_fc.dsc.data_base + 2)
#define FLASH_STATUS_REG		(ts->flash_fc.dsc.data_base + 3)
#define FLASH_STATUS_MASK		0xFF

/* Page number */
#define COMMON_PAGE         (ts->common_fc.function_page)
#define FINGER_PAGE         (ts->finger_fc.function_page)
#define ANALOG_PAGE         (ts->analog_fc.function_page)
#define FLASH_PAGE          (ts->flash_fc.function_page)
#define SENSOR_PAGE         (ts->sensor_fc.function_page)
#define VIDEO_PAGE          (ts->video_fc.function_page)
#define LPWG_PAGE           (ts->custom_fc.function_page)
#define DEFAULT_PAGE        0x00

/* Check Noise */
#define IM_LSB			(ts->analog_fc.dsc.data_base + 4) /* 1-page */
#define IM_MSB			(ts->analog_fc.dsc.data_base + 5) /* 1-page */
#define CURR_NOISE_STATE	(ts->analog_fc.dsc.data_base + 8) /* 1-page */
#define FREQ_SCAN_IM		(ts->analog_fc.dsc.data_base + 10) /* 1-page */

/* Display */
#define DCS_CMD_DATA_REG		(ts->video_fc.dsc.command_base) /* 1-page */
#define DCS_CMD_SEND_REG		(ts->video_fc.dsc.command_base + 6) /* 1-page */
#define WRITE_DCS_VAL			0x01
#define OPCODE_DELAY            200

/* DCS Opcode */
#define DCS_OPCODE_CTRL     1
#define DCS_COMMAND_SEND    2

/* Others */
#define LPWG_STATUS_REG             (ts->custom_fc.dsc.data_base) /* 4-page */
#define LPWG_DATA_REG               (ts->custom_fc.dsc.data_base + 1) /* 4-page */
#define LPWG_TAPCOUNT_REG           (ts->custom_fc.dsc.control_base) /* 4-page */
#define LPWG_MIN_INTERTAP_REG       (ts->custom_fc.dsc.control_base + 1) /* 4-page */
#define LPWG_MAX_INTERTAP_REG       (ts->custom_fc.dsc.control_base + 2) /* 4-page */
#define LPWG_TOUCH_SLOP_REG         (ts->custom_fc.dsc.control_base + 3) /* 4-page */
#define LPWG_TAP_DISTANCE_REG       (ts->custom_fc.dsc.control_base + 4) /* 4-page */
#define LPWG_TAP_THRESHOLD_REG      (ts->custom_fc.dsc.control_base + 5) /* 4-page */
#define LPWG_INTERRUPT_DELAY_REG    (ts->custom_fc.dsc.control_base + 6) /* 4-page */
#define LPWG_TAPCOUNT_REG2          (ts->custom_fc.dsc.control_base + 7) /* 4-page */
#define LPWG_MIN_INTERTAP_REG2      (ts->custom_fc.dsc.control_base + 8) /* 4-page */
#define LPWG_MAX_INTERTAP_REG2      (ts->custom_fc.dsc.control_base + 9) /* 4-page */
#define LPWG_TOUCH_SLOP_REG2        (ts->custom_fc.dsc.control_base + 10) /* 4-page */
#define LPWG_TAP_DISTANCE_REG2      (ts->custom_fc.dsc.control_base + 11) /* 4-page */
#define LPWG_TAP_THRESHOLD_REG2     (ts->custom_fc.dsc.control_base + 12) /* 4-page */
#define LPWG_INTERRUPT_DELAY_REG2   (ts->custom_fc.dsc.control_base + 13) /* 4-page */
#define LPWG_DEBUG_INT_ENABLE_REG    (ts->custom_fc.dsc.control_base + 15) /* 4-page */
//#define MISC_HOST_CONTROL_REG     (ts->custom_fc.dsc.control_base + 14) /* 4-page TD4191 not support */
#define THERMAL_HIGH_FINGER_AMPLITUDE   0x60 /* finger_amplitude(0x80) = 0.5 */


#define LPWG_DEBUG_COUNT             (ts->custom_fc.dsc.data_base + 0x31)
#define LPWG_DEBUG_INDEX             (ts->custom_fc.dsc.data_base + 0x32)
#define LPWG_DEBUG_REASON            (ts->custom_fc.dsc.data_base + 0x33)
#define LPWG_DEBUG_REASON_REALTIME   (ts->custom_fc.dsc.data_base + 0x49)
#define LPWG_HAS_DEBUG_MODULE       (ts->custom_fc.dsc.query_base + 4)
#define LPWG_MAX_BUFFER             10
#define LPWG_REASON_ALL_ENALBE      0xFFFF
#define LPWG_REASON_ALL_DISABLE     0x00

/* LPWG Control Value */
#define REPORT_MODE_CTRL	1
#define TCI_ENABLE_CTRL		2
#define TAP_COUNT_CTRL		3
#define MIN_INTERTAP_CTRL	4
#define MAX_INTERTAP_CTRL	5
#define TOUCH_SLOP_CTRL		6
#define TAP_DISTANCE_CTRL	7
#define INTERRUPT_DELAY_CTRL    8

#define TCI_ENABLE_CTRL2        22
#define TAP_COUNT_CTRL2         23
#define MIN_INTERTAP_CTRL2      24
#define MAX_INTERTAP_CTRL2      25
#define TOUCH_SLOP_CTRL2        26
#define TAP_DISTANCE_CTRL2      27
#define INTERRUPT_DELAY_CTRL2   28

/* Palm / Hover */
#define PALM_TYPE	3
#define HOVER_TYPE	5
#define MAX_PRESSURE	255

#define I2C_DELAY			50
#define UEVENT_DELAY			200
#define REBASE_DELAY			100
#define CAP_DIFF_MAX             500
#define CAP_MIN_MAX_DIFF             1000
#define KNOCKON_DELAY                   68 /* 700ms */

/* Get user-finger-data from register.
 */
#define TS_SNTS_GET_X_POSITION(_msb_reg, _lsb_reg) \
	(((u16)((_msb_reg << 8)  & 0xFF00)  | (u16)((_lsb_reg) & 0xFF)))
#define TS_SNTS_GET_Y_POSITION(_msb_reg, _lsb_reg) \
	(((u16)((_msb_reg << 8)  & 0xFF00)  | (u16)((_lsb_reg) & 0xFF)))
#define TS_SNTS_GET_WIDTH_MAJOR(_width_x, _width_y) \
	(((_width_x - _width_y) > 0) ? _width_x : _width_y)
#define TS_SNTS_GET_WIDTH_MINOR(_width_x, _width_y) \
	(((_width_x - _width_y) > 0) ? _width_y : _width_x)

#define TS_SNTS_GET_ORIENTATION(_width_y, _width_x) \
	(((_width_y - _width_x) > 0) ? 0 : 1)
#define TS_SNTS_GET_PRESSURE(_pressure) \
	_pressure
#define jitter_abs(x)	(x > 0 ? x : -x)

#define GET_OBJECT_REPORT_INFO(_reg, _type) \
	(((_reg) & ((u8)(1 << (_type)))) >> (_type))

#define GET_HIGH_U8_FROM_U16(_u16_data) \
	((u8)(((_u16_data) & 0xFF00) >> 8))

#define GET_LOW_U8_FROM_U16(_u16_data) \
	((u8)((_u16_data) & 0x00FF))

#define GET_U16_FROM_U8(_u8_hi_data, _u8_lo_data) \
	((u16)(((_u8_hi_data) << 8) | (_u8_lo_data)))

/* Define EE_SHORT in order to enable sensor speed/noise/e_e short test */
//#define EE_SHORT

extern void release_all_touch_event(struct lge_touch_data *ts);

static u8 half_err_cnt;
static int rebase_retry_cnt;
static int ref_chk_rebase_stop;
static int wq_rebase_lock = 1;
static int raw_cap_file_exist;
#define TOUCH_WAKE_COUNTER_LOG_PATH		"/mnt/sdcard/wake_cnt.txt"
enum error_type synaptics_ts_ic_ctrl(struct i2c_client *client, u8 code, u32 value, u32 *ret);
static int set_doze_param(struct synaptics_ts_data *ts, int value);
static bool need_scan_pdt = true;
int mfts_enable;

void touch_enable_irq(unsigned int irq)
{
	if (!touch_irq_mask) {
		touch_irq_mask = 1;
		enable_irq(irq);
	}
}

void touch_disable_irq(unsigned int irq)
{
	if (touch_irq_mask) {
		touch_irq_mask = 0;
		disable_irq_nosync(irq);
	}
}

void write_time_log(char *filename, char *data, int data_include)
{
	int fd = 0;
	char *fname = NULL;
	char time_string[64] = {0};
	struct timespec my_time;
	struct tm my_date;

	mm_segment_t old_fs = get_fs();

	my_time = __current_kernel_time();
	time_to_tm(my_time.tv_sec, sys_tz.tz_minuteswest * 60 * (-1), &my_date);
	snprintf(time_string,
			sizeof(time_string),
			"%02d-%02d %02d:%02d:%02d.%03lu\n\n",
			my_date.tm_mon + 1,
			my_date.tm_mday,
			my_date.tm_hour,
			my_date.tm_min,
			my_date.tm_sec,
			(unsigned long) my_time.tv_nsec / 1000000);

	set_fs(KERNEL_DS);

	if (filename == NULL) {
		if (factory_boot)
			fname = "/data/logger/touch_self_test.txt";
		else
			fname = "/sdcard/touch_self_test.txt";
	} else {
		fname = filename;
	}

	fd = sys_open(fname, O_WRONLY|O_CREAT|O_APPEND, 0666);
	sys_chmod(fname, 0666);

	TOUCH_INFO_MSG("write open %s, fd : %d\n",
			(fd >= 0) ? "success" : "fail", fd);

	if (fd >= 0) {
		if (data_include && data != NULL)
			sys_write(fd, data, strlen(data));
		sys_write(fd, time_string, strlen(time_string));
		sys_close(fd);
	}
	set_fs(old_fs);
}

void write_firmware_version_log(struct synaptics_ts_data *ts)
{
	char *version_string = NULL;
	int ver_outbuf = 0;
	int rc = 0;

	version_string = kzalloc(448 * sizeof(char), GFP_KERNEL);

	if (mfts_enable) {
		read_page_description_table(ts->client);
		rc = get_ic_info(ts);

		if (rc < 0) {
			ver_outbuf += snprintf(version_string+ver_outbuf, 448-ver_outbuf, "-1\n");
			ver_outbuf += snprintf(version_string+ver_outbuf, 448-ver_outbuf,
					"Read Fail Touch IC Info\n");
			kfree(version_string);
			return;
		}
	}
	TOUCH_INFO_MSG("%s: first: version_string %p(%d)\n",
			__func__, version_string, strlen(version_string));

	ver_outbuf += snprintf(version_string+ver_outbuf, 448-ver_outbuf,
			"===== Firmware Info =====\n");

	if (ts->fw_info.fw_version[0] > 0x60) {
		ver_outbuf += snprintf(version_string+ver_outbuf, 448-ver_outbuf,
				"ic_fw_version[%s]\n", ts->fw_info.fw_version);
	} else {
		ver_outbuf += snprintf(version_string+ver_outbuf, 448-ver_outbuf,
				"version : v%d.%02d\n",
				((ts->fw_info.fw_version[3] & 0x80) >> 7),
				(ts->fw_info.fw_version[3] & 0x7F));
	}

	ver_outbuf += snprintf(version_string+ver_outbuf,
			448-ver_outbuf,
			"IC_product_id[%s]\n",
			ts->fw_info.fw_product_id);

	if (!(strncmp(ts->fw_info.fw_product_id, "PLG313", 6)) ||
			!(strncmp(ts->fw_info.fw_product_id, "PLG352", 6)) ||
			!(strncmp(ts->fw_info.fw_product_id, "PLG391", 6))) {
		if (ts->fw_flag == S3528_A0)
			ver_outbuf += snprintf(version_string+ver_outbuf,
					448-ver_outbuf,
					"Touch IC : s3528(A0)\n");
		else
			ver_outbuf += snprintf(version_string+ver_outbuf,
					448-ver_outbuf,
					"Touch IC : s3528(A1)\n");
	} else if (!strncmp(ts->fw_info.fw_product_id, "PLG298", 6)) {
		ver_outbuf += snprintf(version_string + ver_outbuf,
				448 - ver_outbuf, "Touch IC : s3621\n");
	} else if (!strncmp(ts->fw_info.fw_product_id, "PLG463", 6)) {
		ver_outbuf += snprintf(version_string + ver_outbuf,
				448 - ver_outbuf,
				"Touch IC : td4191(family_id=%d, fw_rev=%d)\n\n",
				ts->fw_info.family,
				ts->fw_info.fw_revision);
	} else if (!strncmp(ts->fw_info.fw_product_id, "PLG516", 6)) {
		ver_outbuf += snprintf(version_string + ver_outbuf,
				448 - ver_outbuf,
				"Touch IC : td4191(TFT ITO Panel)(family_id=%d, fw_rev=%d)\n\n",
				ts->fw_info.family,
				ts->fw_info.fw_revision);
	} else {
		ver_outbuf += snprintf(version_string + ver_outbuf,
				448 - ver_outbuf,
				"Touch Product ID read error\n");
	}

	ver_outbuf += snprintf(version_string+ver_outbuf,
			448-ver_outbuf,
			"=========================\n\n");

	write_log(NULL, version_string);
	msleep(30);

	TOUCH_INFO_MSG("%s: after: version_string %p(%d)\n",
			__func__, version_string, strlen(version_string));
	kfree(version_string);
}


/* wrapper function for i2c communication - except defalut page
 * if you have to select page for reading or writing,
 * then using this wrapper function */
int synaptics_ts_page_data_read(struct i2c_client *client,
		u8 page, u8 reg, int size, u8 *data)
{
	DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, page), error);
	DO_SAFE(touch_i2c_read(client, reg, size, data), error);
	DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE),
			error);
	return 0;
error:
	TOUCH_ERR_MSG("%s, %d : read page failed\n", __func__, __LINE__);
	return -EPERM;
}

int synaptics_ts_page_data_write(struct i2c_client *client,
		u8 page, u8 reg, int size, u8 *data)
{
	DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, page), error);
	DO_SAFE(touch_i2c_write(client, reg, size, data), error);
	DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE),
			error);
	return 0;
error:
	TOUCH_ERR_MSG("%s, %d : write page failed\n", __func__, __LINE__);
	return -EPERM;
}


int synaptics_ts_page_data_write_byte(struct i2c_client *client,
		u8 page, u8 reg, u8 data)
{
	DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, page), error);
	DO_SAFE(touch_i2c_write_byte(client, reg, data), error);
	DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE),
			error);
	return 0;
error:
	TOUCH_ERR_MSG("%s, %d : write page byte failed\n", __func__, __LINE__);
	return -EPERM;
}

/**
 * Knock on
 *
 * Type		Value
 *
 * 1		WakeUp_gesture_only=1 / Normal=0
 * 2		TCI enable=1 / disable=0
 * 3		Tap Count
 * 4		Min InterTap
 * 5		Max InterTap
 * 6		Touch Slop
 * 7		Tap Distance
 * 8            Interrupt Delay
 */
static int tci_control(struct synaptics_ts_data *ts, int type, u8 value)
{
	struct i2c_client *client = ts->client;
	u8 buffer[3] = {0};

	switch (type) {
	case REPORT_MODE_CTRL:
		DO_SAFE(touch_i2c_read(ts->client, INTERRUPT_ENABLE_REG,
					1, buffer), error);
		DO_SAFE(touch_i2c_write_byte(ts->client, INTERRUPT_ENABLE_REG,
					value ? buffer[0] & ~INTERRUPT_MASK_ABS0
					: buffer[0] | INTERRUPT_MASK_ABS0),
				error);
#if defined(CONFIG_TOUCHSCREEN_LGE_SYNAPTICS_TD4191)
	//Do nothing
#else
		if (value) {
			buffer[0] = 0x29;
			buffer[1] = 0x29;
			DO_SAFE(touch_i2c_write(client, f12_info.ctrl_reg_addr[15], 2, buffer), error);
		}
#endif
		DO_SAFE(touch_i2c_read(client, f12_info.ctrl_reg_addr[20],
					3, buffer), error);
		buffer[2] = (buffer[2] & 0xfc) | (value ? 0x2 : 0x0);
		DO_SAFE(touch_i2c_write(client, f12_info.ctrl_reg_addr[20],
					3, buffer), error);
		break;
	case TCI_ENABLE_CTRL:
		DO_SAFE(synaptics_ts_page_data_read(client,
					LPWG_PAGE, LPWG_TAPCOUNT_REG,
					1, buffer), error);
		buffer[0] = (buffer[0] & 0xfe) | (value & 0x1);
		DO_SAFE(synaptics_ts_page_data_write(client,
					LPWG_PAGE, LPWG_TAPCOUNT_REG,
					1, buffer), error);
		break;
	case TCI_ENABLE_CTRL2:
		DO_SAFE(synaptics_ts_page_data_read(client, LPWG_PAGE,
					LPWG_TAPCOUNT_REG2, 1, buffer), error);
		buffer[0] = (buffer[0] & 0xfe) | (value & 0x1);
		DO_SAFE(synaptics_ts_page_data_write(client,
					LPWG_PAGE, LPWG_TAPCOUNT_REG2,
					1, buffer), error);
		break;
	case TAP_COUNT_CTRL:
		DO_SAFE(synaptics_ts_page_data_read(client,
					LPWG_PAGE, LPWG_TAPCOUNT_REG,
					1, buffer), error);
		buffer[0] = ((value << 3) & 0xf8) | (buffer[0] & 0x7);
		DO_SAFE(synaptics_ts_page_data_write(client,
					LPWG_PAGE, LPWG_TAPCOUNT_REG,
					1, buffer), error);
		break;
	case TAP_COUNT_CTRL2:
		DO_SAFE(synaptics_ts_page_data_read(client,
					LPWG_PAGE, LPWG_TAPCOUNT_REG2,
					1, buffer), error);
		buffer[0] = ((value << 3) & 0xf8) | (buffer[0] & 0x7);
		DO_SAFE(synaptics_ts_page_data_write(client,
					LPWG_PAGE, LPWG_TAPCOUNT_REG2,
					1, buffer), error);
		break;
	case MIN_INTERTAP_CTRL:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
					LPWG_PAGE, LPWG_MIN_INTERTAP_REG,
					value), error);
		break;
	case MIN_INTERTAP_CTRL2:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
					LPWG_PAGE, LPWG_MIN_INTERTAP_REG2,
					value), error);
		break;
	case MAX_INTERTAP_CTRL:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
					LPWG_PAGE, LPWG_MAX_INTERTAP_REG,
					value), error);
		break;
	case MAX_INTERTAP_CTRL2:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
					LPWG_PAGE, LPWG_MAX_INTERTAP_REG2,
					value), error);
		break;
	case TOUCH_SLOP_CTRL:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
					LPWG_PAGE, LPWG_TOUCH_SLOP_REG,
					value), error);
		break;
	case TOUCH_SLOP_CTRL2:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
					LPWG_PAGE, LPWG_TOUCH_SLOP_REG2,
					value), error);
		break;
	case TAP_DISTANCE_CTRL:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
					LPWG_PAGE, LPWG_TAP_DISTANCE_REG,
					value), error);
		break;
	case TAP_DISTANCE_CTRL2:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
					LPWG_PAGE, LPWG_TAP_DISTANCE_REG2,
					value), error);
		break;
	case INTERRUPT_DELAY_CTRL:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
					LPWG_PAGE, LPWG_INTERRUPT_DELAY_REG,
					value ?
					(buffer[0] = (KNOCKON_DELAY << 1) | 0x1)
					: (buffer[0] = 0)), error);
		break;
	case INTERRUPT_DELAY_CTRL2:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
					LPWG_PAGE, LPWG_INTERRUPT_DELAY_REG2,
					value ?
					(buffer[0] = (KNOCKON_DELAY << 1) | 0x1)
					: (buffer[0] = 0)), error);
		break;
	default:
		break;
	}

	return 0;
error:
	TOUCH_ERR_MSG("%s, %d : tci control failed\n", __func__, __LINE__);
	return -EPERM;

}

static int swipe_enable(struct synaptics_ts_data *ts)
{
	struct i2c_client *client = ts->client;
	struct swipe_data *swp = &ts->ts_swipe_data;
	u8 buf[19] = {0};

	if (swp->support_swipe == NO_SUPPORT_SWIPE) {
		TOUCH_ERR_MSG("support_swipe:%d\n", swp->support_swipe);
		goto error;
	}

	TOUCH_INFO_MSG("%s start (support_swipe:%d)\n", __func__, swp->support_swipe);

	DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, LPWG_PAGE), error);

	DO_SAFE(touch_i2c_read(client, swp->swipe_enable_reg, 1, buf), error);

	if (!(buf[0] & swp->swipe_enable_mask)) {
		DO_SAFE(touch_i2c_write_byte(client, swp->swipe_enable_reg,
					buf[0] | swp->swipe_enable_mask), error);
	}

	DO_SAFE(touch_i2c_write_byte(client, swp->swipe_min_distance_reg,
				swp->swipe_min_distance), error);

	DO_SAFE(touch_i2c_write_byte(client, swp->swipe_ratio_check_min_distance_reg,
				swp->swipe_ratio_check_min_distance), error);

	DO_SAFE(touch_i2c_write_byte(client, swp->swipe_ratio_threshold_reg,
				swp->swipe_ratio_threshold), error);

	DO_SAFE(touch_i2c_write_byte(client, swp->swipe_ratio_check_period_reg,
				swp->swipe_ratio_check_period), error);

	DO_SAFE(touch_i2c_write_byte(client, swp->min_swipe_time_thres_lsb_reg,
				GET_LOW_U8_FROM_U16(swp->min_swipe_time_threshold)), error);

	DO_SAFE(touch_i2c_write_byte(client, swp->min_swipe_time_thres_msb_reg,
				GET_HIGH_U8_FROM_U16(swp->min_swipe_time_threshold)), error);

	DO_SAFE(touch_i2c_write_byte(client, swp->max_swipe_time_thres_lsb_reg,
				GET_LOW_U8_FROM_U16(swp->max_swipe_time_threshold)), error);

	DO_SAFE(touch_i2c_write_byte(client, swp->max_swipe_time_thres_msb_reg,
				GET_HIGH_U8_FROM_U16(swp->max_swipe_time_threshold)), error);

	DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE), error);

	TOUCH_INFO_MSG("%s end\n", __func__);

	return 0;
error:
	DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE), error);
	TOUCH_ERR_MSG("swipe_enable failed\n");
	return -EPERM;

}

static int swipe_disable(struct synaptics_ts_data *ts)
{
	struct i2c_client *client = ts->client;
	struct swipe_data *swp = &ts->ts_swipe_data;
	u8 buf[1] = {0};

	if (swp->support_swipe == NO_SUPPORT_SWIPE) {
		TOUCH_ERR_MSG("support_swipe:%d\n", swp->support_swipe);
		goto error;
	}

	TOUCH_INFO_MSG("%s start (support_swipe:%d)\n", __func__, swp->support_swipe);

	DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, LPWG_PAGE), error);

	DO_SAFE(touch_i2c_read(client, swp->swipe_enable_reg, 1, buf), error);

	if (buf[0] & swp->swipe_enable_mask) {
		DO_SAFE(touch_i2c_write_byte(client, swp->swipe_enable_reg,
					buf[0] & ~swp->swipe_enable_mask), error);
	}

	DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE), error);

	TOUCH_INFO_MSG("%s end\n", __func__);

	return 0;
error:
	DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE), error);
	TOUCH_ERR_MSG("swipe_disable failed\n");
	return -EPERM;

}

/**
 * debug Reason
 *
 *   Error Type            value
 * 1 Distance_Inter_Tap    (1U << 0)
 * 2 Distance TouchSlop    (1U << 1)
 * 3 Timeout Inter Tap     (1U << 2)
 * 4 Multi Finger          (1U << 3)
 * 5 Delay Time            (1U << 4)
 * 6 Palm State            (1U << 5)
 * 7 Active Area           (1U << 6)
 * 8 Tap Count             (1U << 7)
 */
static int lpwg_debug_interrupt_control(struct synaptics_ts_data *ts, u16 value)
{
	struct i2c_client *client = ts->client;
	u8 buffer[2] = {0};

	DO_SAFE(synaptics_ts_page_data_read(client, LPWG_PAGE, LPWG_DEBUG_INT_ENABLE_REG,
					2, buffer), error);
	buffer[0] = value ? ((LPWG_REASON_ALL_ENALBE >> 8) & 0xFF) : LPWG_REASON_ALL_DISABLE;
	buffer[1] = value ? (LPWG_REASON_ALL_ENALBE & 0xFF) : LPWG_REASON_ALL_DISABLE;
	if (synaptics_ts_page_data_write(client, LPWG_PAGE, LPWG_DEBUG_INT_ENABLE_REG,
				2, buffer) < 0) {
		TOUCH_INFO_MSG("LPWG_DEBUG_INT_ENABLE_REG write error \n");
	} else {
		TOUCH_INFO_MSG("LPWG_DEBUG_INT_ENABLE_REG = 0x%02x%02x\n", buffer[0], buffer[1]);
	}
	return 0;
error:
	TOUCH_ERR_MSG("%s, %d : LPWG debug interrupt control failed\n", __func__, __LINE__);
	return -EPERM;
}


static int get_tci_data(struct synaptics_ts_data *ts, int count)
{
	struct i2c_client *client = ts->client;
	u8 i = 0;
	u8 buffer[12][4] = {{0} };

	ts->pw_data.data_num = count;

	if (!count)
		return 0;

	DO_SAFE(synaptics_ts_page_data_read(client,
				LPWG_PAGE, LPWG_DATA_REG, 4 * count,
				&buffer[0][0]), error);

	TOUCH_DEBUG(DEBUG_BASE_INFO || DEBUG_LPWG,
			"%s : knock code coordinates, count = %d\n",
			__func__, count);

	for (i = 0; i < count; i++) {
		ts->pw_data.data[i].x = TS_SNTS_GET_X_POSITION(buffer[i][1],
				buffer[i][0]);
		ts->pw_data.data[i].y = TS_SNTS_GET_Y_POSITION(buffer[i][3],
				buffer[i][2]);

		if (ts->pdata->role->use_security_mode) {
			if (ts->lpwg_ctrl.password_enable || ts->pdata->role->use_security_all) {
				TOUCH_INFO_MSG("LPWG data xxxx, xxxx\n");
			} else {
				TOUCH_INFO_MSG("LPWG data %d, %d\n",
						ts->pw_data.data[i].x,
						ts->pw_data.data[i].y);
			}
		} else {
			TOUCH_INFO_MSG("LPWG data %d, %d\n",
					ts->pw_data.data[i].x,
					ts->pw_data.data[i].y);
		}
	}

	return 0;
error:
	TOUCH_ERR_MSG("%s, %d : get tci_control failed, count : %d\n",
			__func__, __LINE__, count);
	return -EPERM;
}

static void set_lpwg_mode(struct lpwg_control *ctrl, int mode)
{
	ctrl->double_tap_enable =
		((mode == LPWG_DOUBLE_TAP) | (mode == LPWG_PASSWORD)) ? 1 : 0;
	ctrl->password_enable = (mode == LPWG_PASSWORD) ? 1 : 0;
	ctrl->signature_enable = (mode == LPWG_SIGNATURE) ? 1 : 0;
	ctrl->lpwg_is_enabled = ctrl->double_tap_enable
		|| ctrl->password_enable || ctrl->signature_enable;
}

static int sleep_control(struct synaptics_ts_data *ts, int mode, int recal)
{
	u8 curr = 0;
	u8 next = 0;

	/*
	 * NORMAL == 0 : resume & lpwg state
	 * SLEEP  == 1 : uevent reporting time - sleep
	 * NO_CAL == 2 : proxi near - sleep when recal is not needed
	 */
	DO_SAFE(touch_i2c_read(ts->client, DEVICE_CONTROL_REG, 1, &curr),
			error);

	next = (curr & 0xFC) | (mode ? DEVICE_CONTROL_NORMAL_OP : DEVICE_CONTROL_SLEEP);
	/*	(recal ? DEVICE_CONTROL_SLEEP
	 *	: DEVICE_CONTROL_SLEEP_NO_RECAL); */

	TOUCH_DEBUG(DEBUG_BASE_INFO, "%s : curr = [%6s] next[%6s]\n", __func__,
			((curr & BITMASK_2) == 0 ? "NORMAL" :
			 ((curr & BITMASK_2) == 1 ? "SLEEP" : "NO_CAL")),
			((next & BITMASK_2) == 0 ? "NORMAL" :
			 ((next & BITMASK_2) == 1 ? "SLEEP" : "NO_CAL")));

	DO_SAFE(touch_i2c_write_byte(ts->client, DEVICE_CONTROL_REG, next),
			error);

	return 0;
error:
	TOUCH_ERR_MSG("%s, %d : sleep control failed\n", __func__, __LINE__);
	return -EPERM;
}

static int lpwg_control(struct synaptics_ts_data *ts, int mode)
{
	set_lpwg_mode(&ts->lpwg_ctrl, mode);

	switch (mode) {
	case LPWG_SIGNATURE:
		break;
	case LPWG_DOUBLE_TAP:                         /* Only TCI-1 */
		tci_control(ts, TCI_ENABLE_CTRL, 1);  /* Tci-1 enable */
		tci_control(ts, TAP_COUNT_CTRL, 2);   /* tap count = 2 */
		tci_control(ts, MIN_INTERTAP_CTRL, 6); /* min inter_tap
							  = 60ms */
		tci_control(ts, MAX_INTERTAP_CTRL, 70); /* max inter_tap
							   = 700ms */
		tci_control(ts, TOUCH_SLOP_CTRL, 100); /* touch_slop = 10mm */
		tci_control(ts, TAP_DISTANCE_CTRL, 10); /* tap distance
							   = 10mm */
		tci_control(ts, INTERRUPT_DELAY_CTRL, 0); /* interrupt delay
							     = 0ms */
		tci_control(ts, TCI_ENABLE_CTRL2, 0); /* Tci-2 disable */
		tci_control(ts, REPORT_MODE_CTRL, 1); /* wakeup_gesture_only */
		if (ts->ts_swipe_data.support_swipe > NO_SUPPORT_SWIPE) {
			if (ts->lpwg_ctrl.qcover)
				swipe_disable(ts);
			else
				swipe_enable(ts);
		}
		break;
	case LPWG_PASSWORD:                           /* TCI-1 and TCI-2 */
		tci_control(ts, TCI_ENABLE_CTRL, 1);  /* Tci-1 enable */
		tci_control(ts, TAP_COUNT_CTRL, 2);   /* tap count = 2 */
		tci_control(ts, MIN_INTERTAP_CTRL, 6); /* min inter_tap
							  = 60ms */
		tci_control(ts, MAX_INTERTAP_CTRL, 70); /* max inter_tap
							   = 700ms */
		tci_control(ts, TOUCH_SLOP_CTRL, 100); /* touch_slop = 10mm */
		tci_control(ts, TAP_DISTANCE_CTRL, 7); /* tap distance = 7mm */
		tci_control(ts, INTERRUPT_DELAY_CTRL,
				(u8)ts->pw_data.double_tap_check);
		tci_control(ts, TCI_ENABLE_CTRL2, 1); /* Tci-2 ensable */
		tci_control(ts, TAP_COUNT_CTRL2,
				(u8)ts->pw_data.tap_count); /* tap count
							       = user_setting */
		tci_control(ts, MIN_INTERTAP_CTRL2, 6); /* min inter_tap
							   = 60ms */
		tci_control(ts, MAX_INTERTAP_CTRL2, 70); /* max inter_tap
							    = 700ms */
		tci_control(ts, TOUCH_SLOP_CTRL2, 100); /* touch_slop = 10mm */
		tci_control(ts, TAP_DISTANCE_CTRL2, 255); /* tap distance
							     = MAX */
		tci_control(ts, INTERRUPT_DELAY_CTRL2, 0); /* interrupt delay
							      = 0ms */
		tci_control(ts, REPORT_MODE_CTRL, 1); /* wakeup_gesture_only */
		if (ts->ts_swipe_data.support_swipe > NO_SUPPORT_SWIPE) {
			if (ts->lpwg_ctrl.qcover)
				swipe_disable(ts);
			else
				swipe_enable(ts);
		}
		break;
	default:
		tci_control(ts, TCI_ENABLE_CTRL, 0); /* Tci-1 disable */
		tci_control(ts, TCI_ENABLE_CTRL2, 0); /* tci-2 disable */
		tci_control(ts, REPORT_MODE_CTRL, 0); /* normal */
		if (ts->ts_swipe_data.support_swipe > NO_SUPPORT_SWIPE)
			swipe_disable(ts);
		break;
	}
	if (ts->pdata->role->use_debug_reason)
			lpwg_debug_interrupt_control(ts, mode);
	TOUCH_DEBUG(DEBUG_BASE_INFO || DEBUG_LPWG,
				"ts->pdata->role->use_debug_reason : [%s]\n",
				ts->pdata->role->use_debug_reason ? "Enabled" : "Disabled");
	TOUCH_INFO_MSG("%s : lpwg_mode[%d]\n", __func__, mode);
	return 0;
}

static int dcs_opcode_control(struct synaptics_ts_data *ts, int type, u8 value)
{
	struct i2c_client *client = ts->client;

	switch (type) {
	case DCS_OPCODE_CTRL:
		if (synaptics_ts_page_data_write_byte(client,
					ANALOG_PAGE, DCS_CMD_DATA_REG,
					value) < 0) {
			TOUCH_INFO_MSG("DCS_COMMAND_OPCODE_REG write error \n");
			goto error;
		} else {
			TOUCH_INFO_MSG("DCS_COMMAND_OPCODE_REG = 0x%x\n", value);
		}
		break;
	case DCS_COMMAND_SEND:
		if (synaptics_ts_page_data_write_byte(client,
					ANALOG_PAGE, DCS_CMD_SEND_REG,
					value) < 0) {
			TOUCH_INFO_MSG("DCS_COMMAND_SEND_REG write error \n");
			goto error;
		} else {
			TOUCH_INFO_MSG("DCS_COMMAND_SEND_REG\n");
		}
		break;
	default:
		break;
	}

	return 0;
error:
	TOUCH_ERR_MSG("%s, %d : DCS Opcode failed\n", __func__, __LINE__);
	return -EPERM;

}

struct synaptics_ts_exp_fhandler {
	struct synaptics_ts_exp_fn *exp_fn;
	bool inserted;
	bool initialized;
};

static struct synaptics_ts_exp_fhandler prox_fhandler;
static struct synaptics_ts_exp_fhandler rmidev_fhandler;

void synaptics_ts_prox_function(struct synaptics_ts_exp_fn *prox_fn, bool insert)
{
	prox_fhandler.inserted = insert;

	if (insert) {
		prox_fhandler.exp_fn = prox_fn;
	} else {
		prox_fhandler.exp_fn = NULL;
	}

	return;
}

void synaptics_ts_rmidev_function(struct synaptics_ts_exp_fn *rmidev_fn, bool insert)
{
	rmidev_fhandler.inserted = insert;

	if (insert) {
		rmidev_fhandler.exp_fn = rmidev_fn;
	} else {
		rmidev_fhandler.exp_fn = NULL;
	}

	return;
}

void get_f12_info(struct synaptics_ts_data *ts)
{
	int retval;
	struct synaptics_ts_f12_query_5 query_5;
	struct synaptics_ts_f12_query_8 query_8;
	struct synaptics_ts_f12_ctrl_23 ctrl_23;
	int i;
	u8 offset;

	if (!ts) {
		TOUCH_ERR_MSG("ts is null\n");
		return;
	}

	/* ctrl_reg_info setting */
	retval = touch_i2c_read(ts->client, (ts->finger_fc.dsc.query_base + 5),
			sizeof(query_5.data), query_5.data);

	if (retval < 0) {
		TOUCH_ERR_MSG("Failed to read from F12_2D_QUERY_05_Control_Presence register\n");
		return;
	}

	f12_info.ctrl_reg_is_present[0] = query_5.ctrl_00_is_present;
	f12_info.ctrl_reg_is_present[1] = query_5.ctrl_01_is_present;
	f12_info.ctrl_reg_is_present[2] = query_5.ctrl_02_is_present;
	f12_info.ctrl_reg_is_present[3] = query_5.ctrl_03_is_present;
	f12_info.ctrl_reg_is_present[4] = query_5.ctrl_04_is_present;
	f12_info.ctrl_reg_is_present[5] = query_5.ctrl_05_is_present;
	f12_info.ctrl_reg_is_present[6] = query_5.ctrl_06_is_present;
	f12_info.ctrl_reg_is_present[7] = query_5.ctrl_07_is_present;
	f12_info.ctrl_reg_is_present[8] = query_5.ctrl_08_is_present;
	f12_info.ctrl_reg_is_present[9] = query_5.ctrl_09_is_present;
	f12_info.ctrl_reg_is_present[10] = query_5.ctrl_10_is_present;
	f12_info.ctrl_reg_is_present[11] = query_5.ctrl_11_is_present;
	f12_info.ctrl_reg_is_present[12] = query_5.ctrl_12_is_present;
	f12_info.ctrl_reg_is_present[13] = query_5.ctrl_13_is_present;
	f12_info.ctrl_reg_is_present[14] = query_5.ctrl_14_is_present;
	f12_info.ctrl_reg_is_present[15] = query_5.ctrl_15_is_present;
	f12_info.ctrl_reg_is_present[16] = query_5.ctrl_16_is_present;
	f12_info.ctrl_reg_is_present[17] = query_5.ctrl_17_is_present;
	f12_info.ctrl_reg_is_present[18] = query_5.ctrl_18_is_present;
	f12_info.ctrl_reg_is_present[19] = query_5.ctrl_19_is_present;
	f12_info.ctrl_reg_is_present[20] = query_5.ctrl_20_is_present;
	f12_info.ctrl_reg_is_present[21] = query_5.ctrl_21_is_present;
	f12_info.ctrl_reg_is_present[22] = query_5.ctrl_22_is_present;
	f12_info.ctrl_reg_is_present[23] = query_5.ctrl_23_is_present;
	f12_info.ctrl_reg_is_present[24] = query_5.ctrl_24_is_present;
	f12_info.ctrl_reg_is_present[25] = query_5.ctrl_25_is_present;
	f12_info.ctrl_reg_is_present[26] = query_5.ctrl_26_is_present;
	f12_info.ctrl_reg_is_present[27] = query_5.ctrl_27_is_present;
	f12_info.ctrl_reg_is_present[28] = query_5.ctrl_28_is_present;
	f12_info.ctrl_reg_is_present[29] = query_5.ctrl_29_is_present;
	f12_info.ctrl_reg_is_present[30] = query_5.ctrl_30_is_present;
	f12_info.ctrl_reg_is_present[31] = query_5.ctrl_31_is_present;

	offset = 0;

	for (i = 0; i < 32; i++) {
		f12_info.ctrl_reg_addr[i] =
			ts->finger_fc.dsc.control_base + offset;

		if (f12_info.ctrl_reg_is_present[i])
			offset++;
	}

	/* data_reg_info setting */
	retval = touch_i2c_read(ts->client, (ts->finger_fc.dsc.query_base + 8),
			sizeof(query_8.data), query_8.data);

	if (retval < 0) {
		TOUCH_ERR_MSG("Failed to read from F12_2D_QUERY_08_Data_Presence register\n");
		return;
	}

	f12_info.data_reg_is_present[0] = query_8.data_00_is_present;
	f12_info.data_reg_is_present[1] = query_8.data_01_is_present;
	f12_info.data_reg_is_present[2] = query_8.data_02_is_present;
	f12_info.data_reg_is_present[3] = query_8.data_03_is_present;
	f12_info.data_reg_is_present[4] = query_8.data_04_is_present;
	f12_info.data_reg_is_present[5] = query_8.data_05_is_present;
	f12_info.data_reg_is_present[6] = query_8.data_06_is_present;
	f12_info.data_reg_is_present[7] = query_8.data_07_is_present;
	f12_info.data_reg_is_present[8] = query_8.data_08_is_present;
	f12_info.data_reg_is_present[9] = query_8.data_09_is_present;
	f12_info.data_reg_is_present[10] = query_8.data_10_is_present;
	f12_info.data_reg_is_present[11] = query_8.data_11_is_present;
	f12_info.data_reg_is_present[12] = query_8.data_12_is_present;
	f12_info.data_reg_is_present[13] = query_8.data_13_is_present;
	f12_info.data_reg_is_present[14] = query_8.data_14_is_present;
	f12_info.data_reg_is_present[15] = query_8.data_15_is_present;

	offset = 0;

	for (i = 0; i < 16; i++) {
		f12_info.data_reg_addr[i] =
			ts->finger_fc.dsc.data_base + offset;

		if (f12_info.data_reg_is_present[i])
			offset++;
	}

	/* print info */
	for (i = 0; i < 32; i++) {
		if (f12_info.ctrl_reg_is_present[i])
			TOUCH_INFO_MSG("f12_info.ctrl_reg_addr[%d]=0x%02X\n",
					i, f12_info.ctrl_reg_addr[i]);
	}

	for (i = 0; i < 16; i++) {
		if (f12_info.data_reg_is_present[i])
			TOUCH_INFO_MSG("f12_info.data_reg_addr[%d]=0x%02X\n",
					i, f12_info.data_reg_addr[i]);
	}

	retval = touch_i2c_read(ts->client, f12_info.ctrl_reg_addr[23],
			sizeof(ctrl_23.data), ctrl_23.data);
	if (retval < 0) {
		TOUCH_ERR_MSG("Failed to read from F12_2D_CTRL_23 register\n");
		return;
	}

	ts->object_report = ctrl_23.obj_type_enable;

	TOUCH_INFO_MSG("ts->object_report[0x%x]\n", ts->object_report);

	return;
}


int synaptics_get_reference_chk(struct synaptics_ts_data *ts)
{

	int diff_v = 0;
	u16 pre_v = 0;
	int err_cnt_horizon = 0;
	static u8 *err_tot_x;
	static u8 *err_tot_y;
	u16 diff_max = 0;

	/*vertical values*/

	int diff_vertical = 0;
	u16 prev_vertical = 0;
	int err_cnt_vertical = 0;
	u16 vertical_diff_max = 0;

	/*common values*/

	int i = 0;
	static unsigned short *ImagepTest;
	static int rebase = 1;
	u16 curx = 0, cury = 0, ref_min = 5000, ref_max = 0;
	u8 rx_err_weight = 150;
	u8 tx_err_weight = 150;
	u8 line_err_chk = 0;
	u8 err_line_num = 0;

	if (power_state == POWER_ON || power_state == POWER_WAKE) {
		if (need_scan_pdt) {
			touch_disable_irq(ts->client->irq);
			SCAN_PDT();
			need_scan_pdt = false;
			touch_enable_irq(ts->client->irq);
			TOUCH_INFO_MSG("SCAN_PDT\n");
		}

		if (rebase) {
			TOUCH_INFO_MSG("ASSIGN MEMORY\n");
			ASSIGN(err_tot_x = kzalloc(2*TxChannelCount * sizeof(u8), GFP_KERNEL), error);
			ASSIGN(err_tot_y = kzalloc(2*RxChannelCount * sizeof(u8), GFP_KERNEL), error);
			ASSIGN(ImagepTest = kmalloc(TxChannelCount * RxChannelCount * sizeof(unsigned short), GFP_KERNEL), error);

		}


		if (!half_err_cnt)
			half_err_cnt = 10;

		TOUCH_INFO_MSG("START check_diff_node\n");

		if (ref_chk_rebase_stop)
			touch_disable_irq(ts->client->irq);

		if (diffnode(ImagepTest) < 0) {
			TOUCH_INFO_MSG("FAIL check_diff_node\n");
			goto error;
		}

		TOUCH_INFO_MSG("END check_diff_node\n");

#ifdef REFERENCE_CHECK_LOG

		int k = 0;

		for (k = 0; k < TxChannelCount * RxChannelCount; k++) {
			TOUCH_INFO_MSG("%5d", ImagepTest[k]);
			if (k%RxChannelCount == RxChannelCount - 1)
				TOUCH_INFO_MSG("\n");
		}
		for (k = 0; k < TRX_MAX; k++)
			TOUCH_INFO_MSG("pdata->rx_cap[%d] = %3d\n",
					k,
					ts->pdata->rx_cap[k]);
		TOUCH_INFO_MSG("\n\n");
		for(k = 0; k < TRX_MAX; k++)
			TOUCH_INFO_MSG("pdata->tx_cap[%d] = %3d\n",
					k,
					ts->pdata->tx_cap[k]);

#endif

		for (i = 0; i < TxChannelCount * RxChannelCount; i++) {

			if (ref_min > ImagepTest[i])
				ref_min = ImagepTest[i];
			if (ref_max < ImagepTest[i])
				ref_max = ImagepTest[i];

			/*RX check MAX check*/
			if (ts->pdata->ref_chk_option[3]) {
				if ((cury == 0) || (cury == 13)) {
					pre_v = ImagepTest[i];
				} else {
					diff_v = abs(ImagepTest[i] - pre_v);

					if (diff_v > abs(ts->pdata->rx_cap[cury-1]) + rx_err_weight) {
						err_cnt_horizon++;
						if ((err_tot_y[RxChannelCount+cury] == 0) || (err_tot_x[curx] == 0)) {
							++err_tot_x[curx];
							++err_tot_y[RxChannelCount+cury];
						}
						TOUCH_INFO_MSG("Rx_Err Node(%d, %d) raw<%d> prev<%d> diff_v<%d> err_tot_x<%d>\n",
								curx,
								cury,
								ImagepTest[i],
								pre_v,
								diff_v,
								err_tot_x[curx]);
					}
				}
				pre_v = ImagepTest[i];

				if (diff_max < diff_v) {
					diff_max = diff_v;
					if (diff_max > CAP_DIFF_MAX) {
						TOUCH_INFO_MSG("Err Rx_Cnt is %d, Tx_Cnt is %d, limit Cnt is %d\n",
								err_cnt_horizon,
								err_cnt_vertical,
								half_err_cnt);
						TOUCH_INFO_MSG("Diff max exceed H(set : %d) (diff_max : %d)  rebase_retry_cnt(%d)",
								CAP_DIFF_MAX,
								diff_max,
								rebase_retry_cnt);
						if (rebase_retry_cnt >= 100) {
							TOUCH_ERR_MSG("retry_chk\n");
							rebase_retry_cnt = 0;
							TOUCH_INFO_MSG("diff_horizon : over retry_chk : %d\n",
									rebase_retry_cnt);
							goto no_cal;
						} else {
							/*we need to recalibration..*/
							TOUCH_INFO_MSG("WE NEED RECALIBRATION HORIZON\n");
							rebase = 0;
							rebase_retry_cnt++;
							if (wq_rebase_lock) {
								queue_delayed_work(touch_wq,
										&ts->diff_node_timer,
										msecs_to_jiffies(REBASE_DELAY));
								return 0;
							}
						}
					}
				}
			} /*RX check MAX check END*/

			/*TX DIFF MAX check*/
			if (ts->pdata->ref_chk_option[2]) {
				if (curx == 0) {
					prev_vertical = ImagepTest[i];
				} else {
					diff_vertical = abs(ImagepTest[i] - prev_vertical);
					if (diff_vertical > abs(ts->pdata->tx_cap[curx-1]) + tx_err_weight) {

						err_cnt_vertical++;
						if ((err_tot_x[TxChannelCount+curx] == 0) || (err_tot_y[cury] == 0)) {
							++err_tot_y[cury];
							++err_tot_x[TxChannelCount+curx];
						}
						TOUCH_INFO_MSG("Tx_Err Node(%d, %d) raw<%d> prev<%d> diff<%d> err_tot_y<%d>\n",
								curx,
								cury,
								ImagepTest[i],
								prev_vertical,
								diff_vertical,
								err_tot_y[cury]);
					}
				}

				prev_vertical = ImagepTest[i - (RxChannelCount-1)];

				if (vertical_diff_max < diff_vertical) {
					vertical_diff_max = diff_vertical;
					if (vertical_diff_max > CAP_DIFF_MAX) {
						TOUCH_INFO_MSG("Err Rx_Cnt is %d, Tx_Cnt is %d, limit Cnt is %d\n",
								err_cnt_horizon,
								err_cnt_vertical,
								half_err_cnt);
						TOUCH_INFO_MSG("Diff max exceed V(set : %d) (diff_max : %d)  rebase_retry_cnt(%d)",
								CAP_DIFF_MAX, vertical_diff_max, rebase_retry_cnt);
						if (rebase_retry_cnt >= 100) {
							rebase_retry_cnt = 0;
							TOUCH_INFO_MSG("diff_vertical : over retry_chk_temp : %d\n",
									rebase_retry_cnt);
							goto no_cal;
						} else {
							/*we need to recalibration..*/
							TOUCH_INFO_MSG("WE NEED RECALIBRATION VERTICAL\n");
							rebase = 0;
							rebase_retry_cnt++;
							if (wq_rebase_lock) {
								queue_delayed_work(touch_wq,
										&ts->diff_node_timer,
										msecs_to_jiffies(REBASE_DELAY));
								return 0;
							}
						}
					}
				}
			}  /*TX DIFF MAX check END*/

			cury++;
			if (cury >= RxChannelCount) {
				curx++;
				cury = 0;
			}

			if (i >= RxChannelCount * TxChannelCount)
				break;
		}

		/* reference max-min check (difference set to 400)*/
		if ((ref_max - ref_min) > CAP_MIN_MAX_DIFF) {
			TOUCH_INFO_MSG("Reference Max(%d) - Min(%d) exceed (set : %d) (Max - Min : %d) rebase_retry_cnt(%d)",
					ref_max,
					ref_min,
					CAP_MIN_MAX_DIFF,
					(ref_max - ref_min),
					rebase_retry_cnt);
			if (rebase_retry_cnt >= 100) {
				rebase_retry_cnt = 0;
				TOUCH_INFO_MSG("reference max-min : over retry_chk_temp : %d\n",
						rebase_retry_cnt);
				goto no_cal;
			} else {
				/*we need to recalibration..*/
				TOUCH_INFO_MSG("WE NEED RECALIBRATION Min Max\n");
				rebase = 0;
				rebase_retry_cnt++;
				if (wq_rebase_lock) {
					queue_delayed_work(touch_wq,
							&ts->diff_node_timer,
							msecs_to_jiffies(REBASE_DELAY));
					return 0;
				}
			}
		}

		TOUCH_INFO_MSG("Err Rx_Cnt is %d, Tx_Cnt is %d, limit Cnt is %d\n",
				err_cnt_horizon,
				err_cnt_vertical,
				half_err_cnt);
		/*check for sd test*/
		ts->h_err_cnt = err_cnt_horizon;
		ts->v_err_cnt = err_cnt_vertical;
		/*check for sd test*/

		/*Half line of TX Line is fail*/
		for (i = 0; i < TxChannelCount; i++) {
			if (err_tot_x[i] != 0) {
				line_err_chk++;
				err_line_num = i;
			}

			if (err_tot_x[i] >= RxChannelCount/2) {
				TOUCH_INFO_MSG("X%d Line all Fail, calibration skip error count(%d)\n",
						i,
						err_tot_x[i]);
				goto no_cal;
			}
		}
		/*Half line of RX Line is fail*/
		for (i = 0; i < RxChannelCount; i++) {
			if (err_tot_y[i] != 0) {
				line_err_chk++;
				err_line_num = i;
			}
			if (err_tot_y[i] >= TxChannelCount/2) {
				TOUCH_INFO_MSG("Y%d Line all Fail, calibration skip error count(%d)\n",
						i,
						err_tot_y[i]);
				goto no_cal;
			}
		}
		line_err_chk = 0; /*not use*/
		err_line_num = 0; /*not use*/


		/* check err node cnt over > 10 */
		if ((err_cnt_horizon && (err_cnt_horizon >= half_err_cnt))
				|| (err_cnt_vertical && (err_cnt_vertical >= half_err_cnt))) {
			TOUCH_INFO_MSG("Reference Err Calibration: H[%d] V[%d]\n",
					err_cnt_horizon,
					err_cnt_vertical);
			if (rebase_retry_cnt >= 100) {
				TOUCH_INFO_MSG("over rebase_retry_cnt : %d\n",
						rebase_retry_cnt);
				rebase_retry_cnt = 0; /*rebase cnt reset*/
				goto no_cal;
			} else {
				err_cnt_horizon = 0; /*value reset*/
				err_cnt_vertical = 0;
				rebase_retry_cnt++;
				rebase = 0;
				if (wq_rebase_lock) {
					queue_delayed_work(touch_wq,
							&ts->diff_node_timer,
							msecs_to_jiffies(REBASE_DELAY));
					return 0;
				}
			}

		}

	} else {

		TOUCH_INFO_MSG("======= check_diff_node not power on/power wake =======\n");

	}

	TOUCH_INFO_MSG("END TEST\n");
	goto no_cal;
	return 0;

error:
	TOUCH_INFO_MSG("error\n");
	touch_enable_irq(ts->client->irq);
	rebase = 1;
	kfree(err_tot_x);
	kfree(err_tot_y);
	kfree(ImagepTest);
	return -EAGAIN;


no_cal:
	TOUCH_INFO_MSG("no_cal\n");
	if (ref_chk_rebase_stop) {
		touch_enable_irq(ts->client->irq);
		ref_chk_rebase_stop = 0;
	}
	synaptics_ts_init(ts->client);
	rebase = 1;
	/* parameter initialize*/
	rebase_retry_cnt = 0;
	if (!wq_rebase_lock)
		wq_rebase_lock = 1;
	touch_enable_irq(ts->client->irq);
	kfree(err_tot_x);
	kfree(err_tot_y);
	kfree(ImagepTest);
	return 0;

}

static int synaptics_get_cap_diff(struct synaptics_ts_data *ts)
{
	int t_diff = 0;
	int r_diff = 0;
	int x = 0;
	int y = 0;
	int ret = 0;
	s8 *rx_cap_diff = NULL;
	s8 *tx_cap_diff = NULL;
	unsigned short *raw_cap = NULL;
	char *f54_cap_wlog_buf = NULL;
	static int cap_outbuf;
	/* allocation of cap_diff */
	rx_cap_diff = NULL;
	tx_cap_diff = NULL;
	raw_cap = NULL;
	cap_outbuf = 0;

	ASSIGN(rx_cap_diff = kzalloc(RxChannelCount * sizeof(u8),
				GFP_KERNEL), error_mem);
	ASSIGN(tx_cap_diff = kzalloc(TxChannelCount * sizeof(u8),
				GFP_KERNEL), error_mem);
	ASSIGN(raw_cap = kzalloc(TxChannelCount*RxChannelCount * sizeof(unsigned char) * 2,
				GFP_KERNEL), error_mem);
	ASSIGN(f54_cap_wlog_buf = kzalloc(DS5_BUFFER_SIZE, GFP_KERNEL),
			error_mem);

	memset(f54_cap_wlog_buf, 0, DS5_BUFFER_SIZE);

	if (diffnode(raw_cap) < 0) {
		TOUCH_INFO_MSG("check_diff_node fail!!\n");
		kfree(rx_cap_diff);
		kfree(tx_cap_diff);
		kfree(raw_cap);
		kfree(f54_cap_wlog_buf);
		return -EAGAIN;
	}

	ts->bad_sample = 0;
	for (y = 0; y < (int)RxChannelCount - 1; y++) {
		t_diff = 0;

		for (x = 0; x < (int)TxChannelCount; x++)
			t_diff += (raw_cap[x * RxChannelCount + y + 1] - raw_cap[x * RxChannelCount + y]);

		t_diff = t_diff / (int)TxChannelCount;

		if (jitter_abs(t_diff) > 1000)
			ts->bad_sample = 1;

		if (t_diff < -0x7F) /* limit diff max */
			rx_cap_diff[y + 1] = -0x7F;
		else if (t_diff > 0x7F)
			rx_cap_diff[y + 1] = 0x7F;
		else
			rx_cap_diff[y + 1] = (s8)t_diff;

		/*need to modify*/
		cap_outbuf += sprintf(f54_cap_wlog_buf+cap_outbuf,
				"%5d\n",
				rx_cap_diff[y + 1]);
	}

	if (tx_cap_diff != NULL && ts->bad_sample == 0) {

		for (x = 0; x < (int)TxChannelCount - 1; x++) {
			r_diff = 0;
			for (y = 0; y < (int)RxChannelCount; y++)
				r_diff += (raw_cap[(x + 1) * RxChannelCount + y] - raw_cap[x * RxChannelCount + y]);

			/*TOUCH_INFO_MSG("[Tx:%2d]tx_cap_diff[%d] %d ", x, x + 1 ,r_diff);*/
			r_diff = r_diff / (int)RxChannelCount;

			if (jitter_abs(r_diff) > 1000)/*need to tunning limit_value*/
				ts->bad_sample = 1;

			if (r_diff < -0x7F) /* limit diff max */
				tx_cap_diff[x + 1] = -0x7F;
			else if (r_diff > 0x7F)
				tx_cap_diff[x + 1] = 0x7F;
			else
				tx_cap_diff[x + 1] = (s8)r_diff;

			/*need to modify*/
			cap_outbuf += snprintf(f54_cap_wlog_buf+cap_outbuf,
					DS5_BUFFER_SIZE-cap_outbuf,
					"%5d\n",
					tx_cap_diff[x + 1]);
		}
	}
	/*need to modify*/
	if (write_log(CAP_FILE_PATH, f54_cap_wlog_buf) == 1)
		raw_cap_file_exist = 1;
	read_log(CAP_FILE_PATH, ts->pdata);

	/*average of Rx_line Cap Value*/
	kfree(rx_cap_diff);
	kfree(tx_cap_diff);
	kfree(raw_cap);
	kfree(f54_cap_wlog_buf);
	return ret;

error_mem:
	kfree(rx_cap_diff);
	kfree(tx_cap_diff);
	kfree(raw_cap);
	kfree(f54_cap_wlog_buf);

	TOUCH_INFO_MSG("error_mem\n");
	return -ENOMEM;

}

static char *productcode_parse(unsigned char *product)
{
	static char str[128] = {0};
	int len = 0;
	char inch[2] = {0};
	char paneltype = 0;
	char version[2] = {0};

	switch ((product[0] & 0xF0) >> 4) {
	case 0:
		len += snprintf(str + len, sizeof(str)-len, "ELK ");
		break;
	case 1:
		len += snprintf(str + len, sizeof(str)-len, "Suntel ");
		break;
	case 2:
		len += snprintf(str + len, sizeof(str)-len, "Tovis ");
		break;
	case 3:
		len += snprintf(str + len, sizeof(str)-len, "Innotek ");
		break;
	case 4:
		len += snprintf(str + len, sizeof(str)-len, "JDI ");
		break;
	case 5:
		len += snprintf(str + len, sizeof(str)-len, "LGD ");
		break;
	default:
		len += snprintf(str + len, sizeof(str)-len, "Unknown ");
		break;
	}

	len += snprintf(str + len, sizeof(str)-len, "\n");

	switch (product[0] & 0x0F) {
	case 0:
		len += snprintf(str + len, sizeof(str)-len, "0key ");
		break;
	case 2:
		len += snprintf(str + len, sizeof(str)-len, "2Key ");
		break;
	case 3:
		len += snprintf(str + len, sizeof(str)-len, "3Key ");
		break;
	case 4:
		len += snprintf(str + len, sizeof(str)-len, "4Key ");
		break;
	default:
		len += snprintf(str + len, sizeof(str)-len, "Unknown ");
		break;
	}

	len += snprintf(str+len, sizeof(str)-len, "\n");

	switch ((product[1] & 0xF0) >> 4) {
	case 0:
		len += snprintf(str + len, sizeof(str)-len, "Synaptics ");
		break;
	default:
		len += snprintf(str + len, sizeof(str)-len, "Unknown ");
		break;
	}

	len += snprintf(str+len,
			sizeof(str)-len,
			"\n");

	inch[0] = (product[1] & 0x0F);
	inch[1] = ((product[2] & 0xF0) >> 4);
	len += snprintf(str+len,
			sizeof(str)-len,
			"%d.%d\n",
			inch[0],
			inch[1]);

	paneltype = (product[2] & 0x0F);
	len += snprintf(str+len,
			sizeof(str)-len,
			"PanelType %d\n",
			paneltype);

	version[0] = ((product[3] & 0x80) >> 7);
	version[1] = (product[3] & 0x7F);
	len += snprintf(str+len,
			sizeof(str)-len,
			"version : v%d.%02d\n",
			version[0],
			version[1]);

	return str;
}
static void lpwg_timer_func(struct work_struct *work_timer)
{
	struct synaptics_ts_data *ts = container_of(to_delayed_work(work_timer),
			struct synaptics_ts_data, work_timer);

	sleep_control(ts, 0, 1); /* sleep until receive the reply. */
	send_uevent_lpwg(ts->client, LPWG_PASSWORD);
	wake_unlock(&ts->timer_wake_lock);

	TOUCH_DEBUG(DEBUG_LPWG, "u-event timer occur!\n");
	return;
}

static void all_palm_released_func(struct work_struct *work_palm)
{
	struct synaptics_ts_data *ts = container_of(to_delayed_work(work_palm),
			struct synaptics_ts_data, work_palm);

	ts->ts_palm_data.all_palm_released = false;
	TOUCH_INFO_MSG("%s: ABS0 event disable time is expired.\n", __func__);

	return;
}

static ssize_t show_firmware(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);
	int ret = 0;
	int rc = 0;

	mutex_lock(&ts->pdata->thread_lock);
	read_page_description_table(ts->client);
	rc = get_ic_info(ts);

	if (rc < 0) {
		ret += snprintf(buf+ret,
				PAGE_SIZE,
				"-1\n");
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"Read Fail Touch IC Info.\n");
		mutex_unlock(&ts->pdata->thread_lock);
		return ret;
	}
	ret = snprintf(buf+ret,
			PAGE_SIZE-ret,
			"\n======== Firmware Info ========\n");
	if (ts->fw_info.fw_version[0] > 0x60) {
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"ic_fw_version[%s]\n",
				ts->fw_info.fw_version);
	} else {
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
        			"ic_fw_version RAW = %02X %02X %02X %02X\n",
				ts->fw_info.fw_version[0],
				ts->fw_info.fw_version[1],
				ts->fw_info.fw_version[2],
				ts->fw_info.fw_version[3]);
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"=== ic_fw_version info ===\n%s",
				productcode_parse(ts->fw_info.fw_version));
	}
	ret += snprintf(buf+ret, PAGE_SIZE-ret, "IC_product_id[%s]\n",
			ts->fw_info.fw_product_id);
	if (!(strncmp(ts->fw_info.fw_product_id, "PLG313", 6)) ||
			!(strncmp(ts->fw_info.fw_product_id, "PLG352", 6)) ||
			!(strncmp(ts->fw_info.fw_product_id, "PLG391", 6))) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Touch IC : s3528(family_id=%d, fw_rev=%d)\n\n",
				ts->fw_info.family,
				ts->fw_info.fw_revision);
	} else if (!(strncmp(ts->fw_info.fw_product_id, "PLG298", 6))) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Touch IC : s3621\n\n");
	} else if (!strncmp(ts->fw_info.fw_product_id, "PLG463", 6)) {
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"Touch IC : td4191(family_id=%d, fw_rev=%d)\n\n",
				ts->fw_info.family,
				ts->fw_info.fw_revision);
	} else if (!strncmp(ts->fw_info.fw_product_id, "PLG516", 6)) {
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"Touch IC : td4191(TFT ITO Panel)(family_id=%d, fw_rev=%d)\n\n",
				ts->fw_info.family,
				ts->fw_info.fw_revision);
	} else {
		ret += snprintf(buf+ret, PAGE_SIZE - ret,
				"Touch product ID read fail\n\n");
	}

	if (ts->fw_info.fw_image_version[0] > 0x60) {
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"img_fw_version[%s]\n",
				ts->fw_info.fw_image_version);
	} else {
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"img_fw_version RAW = %02X %02X %02X %02X\n",
				ts->fw_info.fw_image_version[0],
				ts->fw_info.fw_image_version[1],
				ts->fw_info.fw_image_version[2],
				ts->fw_info.fw_image_version[3]);
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"=== img_fw_version info ===\n%s",
				productcode_parse(ts->fw_info.fw_image_version));
	}
	ret += snprintf(buf+ret, PAGE_SIZE-ret, "Img_product_id[%s]\n",
			ts->fw_info.fw_image_product_id);
	if (!(strncmp(ts->fw_info.fw_image_product_id, "PLG313", 6)) ||
			!(strncmp(ts->fw_info.fw_image_product_id, "PLG352", 6))
			|| !(strncmp(ts->fw_info.fw_image_product_id,
					"PLG391", 6))) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Touch IC : s3528(family_id = %d, fw_rev = %d)\n",
				ts->fw_info.family,
				ts->fw_info.fw_revision);
	} else if (!strncmp(ts->fw_info.fw_image_product_id, "PLG298", 6)) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Touch IC : s3621\n");
	} else if (!strncmp(ts->fw_info.fw_image_product_id, "PLG463", 6)) {
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"Touch IC : td4191(family_id=%d, fw_rev=%d)\n\n",
				ts->fw_info.family,
				ts->fw_info.fw_revision);
	} else if (!strncmp(ts->fw_info.fw_image_product_id, "PLG516", 6)) {
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"Touch IC : td4191(TFT ITO Panel)(family_id=%d, fw_rev=%d)\n\n",
				ts->fw_info.family,
				ts->fw_info.fw_revision);
	} else {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Touch product ID read fail\n");
	}
	mutex_unlock(&ts->pdata->thread_lock);
	return ret;
}

static ssize_t show_synaptics_fw_version(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);
	int ret = 0;
	int rc = 0;

	mutex_lock(&ts->pdata->thread_lock);
	read_page_description_table(ts->client);
	rc = get_ic_info(ts);

	if (rc < 0) {
		ret += snprintf(buf+ret, PAGE_SIZE-ret, "-1\n");
		ret += snprintf(buf+ret, PAGE_SIZE-ret, "Read Fail Touch IC Info.\n");
		mutex_unlock(&ts->pdata->thread_lock);
		return ret;
	}
	ret = snprintf(buf+ret, PAGE_SIZE-ret, "\n======== Auto Touch Test ========\n");
	if (ts->fw_info.fw_version[0] > 0x60)
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"ic_fw_version[%s]\n",
				ts->fw_info.fw_version);
	else
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret, 
				"version : v%d.%02d\n",
				((ts->fw_info.fw_version[3] & 0x80) >> 7),
				(ts->fw_info.fw_version[3] & 0x7F));
	ret += snprintf(buf+ret,
			PAGE_SIZE-ret,
			"IC_product_id[%s]\n",
			ts->fw_info.fw_product_id);
	if (!(strncmp(ts->fw_info.fw_product_id, "PLG313", 6)) ||
			!(strncmp(ts->fw_info.fw_product_id, "PLG352", 6)) ||
			!(strncmp(ts->fw_info.fw_product_id, "PLG391", 6))) {
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"Touch IC : s3528(family_id=%d, fw_rev=%d)\n\n",
				ts->fw_info.family,
				ts->fw_info.fw_revision);
	} else if (!strncmp(ts->fw_info.fw_product_id, "PLG298", 6)) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Touch IC : s3621\n\n");
	} else if (!strncmp(ts->fw_info.fw_product_id, "PLG463", 6)) {
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"Touch IC : td4191(family_id=%d, fw_rev=%d)\n\n",
				ts->fw_info.family,
				ts->fw_info.fw_revision);
	} else if (!strncmp(ts->fw_info.fw_product_id, "PLG516", 6)) {
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"Touch IC : td4191(TFT ITO Panel)(family_id=%d, fw_rev=%d)\n\n",
				ts->fw_info.family,
				ts->fw_info.fw_revision);
	} else {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Touch product ID read fail\n\n");
	}
	mutex_unlock(&ts->pdata->thread_lock);
	return ret;
}

static ssize_t show_sd(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);

	struct lge_touch_data *lge_ts = i2c_get_clientdata(ts->client);

	int ret = 0;
	int full_raw_cap = 0;
#if defined(EE_SHORT)
	int e_e_short = 0;
	int noise_delta = 0;
	int sensor_speed = 0;
#else
	int electrode_short = 0;
	int short_upper_img = 0;
	int short_lower_img = 0;
#endif
	int channel_status = 0;
	int lower_img = 0;
	int upper_img = 0;
	int speed_lower_img = 0;
	int speed_upper_img = 0;

	char *temp_buf = NULL;
	int len = 0;
	if (power_state == POWER_ON || power_state == POWER_WAKE) {

		mutex_lock(&ts->pdata->thread_lock);

		temp_buf = kzalloc(100, GFP_KERNEL);
		if (!temp_buf) {
			TOUCH_INFO_MSG("%s Failed to allocate memory\n", __func__);
			mutex_unlock(&ts->pdata->thread_lock);
			return 0;
		}
		write_time_log(NULL, NULL, 0);
		msleep(30);
		write_firmware_version_log(ts);

		touch_disable_irq(ts->client->irq);

		TOUCH_INFO_MSG("SOFT_RESET start\n");
		if (lge_ts != NULL) {
			release_all_touch_event(lge_ts);
		} else {
			TOUCH_ERR_MSG("lge_ts is NULL!\n")
		}
		touch_i2c_write_byte(ts->client, DEVICE_COMMAND_REG, 0x01);
		msleep(ts->pdata->role->booting_delay);
		synaptics_ts_init(ts->client);
		TOUCH_INFO_MSG("SOFT_RESET end\n");

		SCAN_PDT();

		lower_img = get_limit(F12_2DTxCount,
				F12_2DRxCount,
				*ts->client,
				ts->pdata,
				"RT78FullRawCapUpperLimit",
				RT78FullRawCapUpper);

		upper_img = get_limit(F12_2DTxCount,
				F12_2DRxCount,
				*ts->client,
				ts->pdata,
				"RT78FullRawCapLowerLimit",
				RT78FullRawCapLower);
#if defined(EE_SHORT)
		/* no use electrode short test */
#else
		short_upper_img = get_limit(F12_2DTxCount,
				F12_2DRxCount,
				*ts->client,
				ts->pdata,
				"RT78ElectodeShortUpperLimit",
				RT78ElectrodeShortUpper);

		short_lower_img = get_limit(F12_2DTxCount,
				F12_2DRxCount,
				*ts->client,
				ts->pdata,
				"RT78ElectodeShortLowerLimit",
				RT78ElectrodeShortLower);
#endif
		speed_lower_img = get_limit(F12_2DTxCount,
				F12_2DRxCount,
				*ts->client,
				ts->pdata,
				"SensorSpeedLowerImageLimit",
				SensorSpeedLowerImage);

		speed_upper_img = get_limit(F12_2DTxCount,
				F12_2DRxCount,
				*ts->client,
				ts->pdata,
				"SensorSpeedUpperImageLimit",
				SensorSpeedUpperImage);

		//Full rawdata - ReadRT78
		if (lower_img < 0 || upper_img < 0) {
			TOUCH_INFO_MSG("[%s] lower return = %d upper return = %d\n",
					__func__, lower_img, upper_img);
			TOUCH_INFO_MSG("[%s][FAIL] Can not check the limit of raw cap\n",
					__func__);
			ret = snprintf(buf+ret, PAGE_SIZE-ret, "Can not check the limit of raw cap\n");
		} else {
			TOUCH_INFO_MSG("[%s] lower return = %d upper return = %d\n",
					__func__, lower_img, upper_img);
			TOUCH_INFO_MSG("[%s][SUCCESS] Can check the limit\n",
					__func__);
			full_raw_cap = F54Test('p', 0, buf);

			if (ts->pdata->ref_chk_option[0]) {
				msleep(30);
				synaptics_get_cap_diff(ts);
			}
			msleep(30);
		}
#if defined(EE_SHORT)
		//E-E short test -
		e_e_short = F54Test('u', 0, buf);
		msleep(30);

		//noise_delta - ReadNoiseReport
		noise_delta = F54Test('x', 0, buf);
		msleep(30);

		//sensor_speed - ReadSensorSpeedReport
		if (speed_lower_img < 0 || speed_upper_img < 0) {
			TOUCH_INFO_MSG("[%s] speed lower return = %d speed upper return = %d\n",
					__func__, speed_lower_img, speed_upper_img);
			TOUCH_INFO_MSG("[%s][FAIL] Can not check the limit of sensor speed image\n",
					__func__);
			ret = snprintf(buf+ret,	PAGE_SIZE-ret, "Can not check the limit of sensor speed image limit\n");
		} else {
			TOUCH_INFO_MSG("[%s] speed lower return = %d speed upper return = %d\n",
					__func__, speed_lower_img, speed_upper_img);
			TOUCH_INFO_MSG("[%s][SUCCESS] Can check the limit of sensor speed image limit\n",
					__func__);
			sensor_speed = F54Test('c', 0, buf);
			msleep(30);
		}
#else
		//Electrode_short - ReadElectodeShortRT78
		if (short_lower_img < 0 || short_upper_img < 0) {
			TOUCH_INFO_MSG("[%s] short lower return = %d short upper return = %d\n",
					__func__, short_lower_img, short_upper_img);
			TOUCH_INFO_MSG("[%s][FAIL] Can not check the limit of Electrode Short\n",
					__func__);
			ret = snprintf(buf+ret, PAGE_SIZE-ret, "Can not check the limit of Electrode Short\n");
		} else {
			TOUCH_INFO_MSG("[%s] short lower return = %d short upper return = %d\n",
					__func__, short_lower_img, short_upper_img);
			TOUCH_INFO_MSG("[%s][SUCCESS] Can check the limit\n",
					__func__);
			electrode_short = F54Test('q', 0, buf);
			msleep(30);
		}
#endif
		synaptics_ts_init(ts->client);

		if (raw_cap_file_exist && ts->pdata->ref_chk_option[0]) {
			/*status change to operate ref_chk*/
			if ((ts->fw_info.family && ts->fw_info.fw_revision) || !(strncmp(ts->fw_info.fw_product_id, "PLG352", 6))) {
				ref_chk_rebase_stop = 1;
				wq_rebase_lock = 0;
				if (synaptics_get_reference_chk(ts) < 0)
					TOUCH_INFO_MSG("ref_chk fail!!\n");
			} else {
				touch_enable_irq(ts->client->irq);
			}
		} else {
			touch_enable_irq(ts->client->irq);
		}
		msleep(30);

		if (ts->h_err_cnt || ts->v_err_cnt || ts->bad_sample)
			full_raw_cap = 0;

		len += snprintf(temp_buf+len,
				PAGE_SIZE-len,
				"Cap Diff : %s\n",
				ts->bad_sample == 0 ? "PASS" : "FAIL");

		if ((ts->fw_info.family && ts->fw_info.fw_revision) || !(strncmp(ts->fw_info.fw_product_id, "PLG352", 6))) {
			len += snprintf(temp_buf+len,
					PAGE_SIZE - len,
					"Error node Check h_err_cnt: %s(err count:%d)\n",
					(ts->h_err_cnt == 0 ? "PASS" : "FAIL"),
					ts->h_err_cnt);
			len += snprintf(temp_buf+len,
					PAGE_SIZE-len,
					"Error node Check v_err_cnt: %s(err count:%d)\n\n",
					(ts->v_err_cnt == 0 ? "PASS" : "FAIL"),
					ts->v_err_cnt);
		}
		write_log(NULL, temp_buf);
		msleep(30);
		write_time_log(NULL, NULL, 0);
		msleep(20);

#if defined(EE_SHORT)
		if( e_e_short && sensor_speed && noise_delta ) {
			channel_status = 1;
		} else {
			channel_status = 0;
		}
#else
		channel_status = electrode_short;
#endif
		ret = snprintf(buf,
				PAGE_SIZE,
				"========RESULT=======\n");
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"Channel Status : %s",
				(channel_status > 0) ? "Pass\n" : "Fail\n");
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"Raw Data : %s",
				(full_raw_cap > 0) ? "Pass\n" : "Fail\n");

		kfree(temp_buf);
		mutex_unlock(&ts->pdata->thread_lock);
	} else {
		write_time_log(NULL, NULL, 0);
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"state=[suspend]. we cannot use I2C, now. Test Result: Fail\n");
	}

	return ret;
}

static ssize_t show_rawdata(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);

	int full_raw_lower_ret = 0;
	int full_raw_upper_ret = 0;
	int ret = 0;

	mutex_lock(&ts->pdata->thread_lock);
	touch_disable_irq(ts->client->irq);

	if (need_scan_pdt) {
		SCAN_PDT();
		need_scan_pdt = false;
	}

	full_raw_lower_ret = get_limit(F12_2DTxCount,
			F12_2DRxCount,
			*ts->client,
			ts->pdata,
			"RT78FullRawCapUpperLimit",
			RT78FullRawCapUpper);


	full_raw_upper_ret = get_limit(F12_2DTxCount,
			F12_2DRxCount,
			*ts->client,
			ts->pdata,
			"RT78FullRawCapLowerLimit",
			RT78FullRawCapLower);

	if (full_raw_lower_ret < 0 || full_raw_upper_ret < 0) {
		TOUCH_INFO_MSG("[%s]: full_raw_lower_ret = %d full_raw_upper_ret = %d\n",
				__func__,
				full_raw_lower_ret,
				full_raw_upper_ret);

		TOUCH_INFO_MSG("[%s][FAIL] Can not check the limit of RT78\n",
				__func__);
		ret = snprintf(buf+ret,
				PAGE_SIZE-ret,
				"Can not check the limit of RT78 limit\n");
	} else {
		TOUCH_INFO_MSG("[%s]: full_raw_lower_ret = %d full_raw_upper_ret = %d\n",
				__func__,
				full_raw_lower_ret,
				full_raw_upper_ret);

		TOUCH_INFO_MSG("[%s][SUCCESS] Can check the limit of RT78\n",
				__func__);
		ret = F54Test('p', 1, buf);
	}
	synaptics_ts_init(ts->client);
	touch_enable_irq(ts->client->irq);

	if (ret == 0)
		ret += snprintf(buf + ret, PAGE_SIZE-ret, "ERROR: full_raw_cap failed.\n");
	mutex_unlock(&ts->pdata->thread_lock);
	return ret;
}

static ssize_t store_rawdata(struct i2c_client *client, const char *buf, size_t count)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);

	int full_raw_lower_ret = 0;
	int full_raw_upper_ret = 0;

	char rawdata_file_path[64] = {0,};
	sscanf(buf, "%s", rawdata_file_path);

	touch_disable_irq(ts->client->irq);

	if (need_scan_pdt) {
		SCAN_PDT();
		need_scan_pdt = false;
	}

	full_raw_lower_ret = get_limit(F12_2DTxCount,
			F12_2DRxCount,
			*ts->client,
			ts->pdata,
			"RT78FullRawCapUpperLimit",
			RT78FullRawCapUpper);


	full_raw_upper_ret = get_limit(F12_2DTxCount,
			F12_2DRxCount,
			*ts->client,
			ts->pdata,
			"RT78FullRawCapLowerLimit",
			RT78FullRawCapLower);

	if (full_raw_lower_ret < 0 || full_raw_upper_ret < 0) {
		TOUCH_INFO_MSG("[%s]: full_raw_lower_ret = %d full_raw_upper_ret = %d\n",
				__func__,
				full_raw_lower_ret,
				full_raw_upper_ret);

		TOUCH_INFO_MSG("[%s][FAIL] Can not check the limit of RT78\n",
				__func__);
	} else {
		TOUCH_INFO_MSG("[%s]: full_raw_lower_ret = %d full_raw_upper_ret = %d\n",
				__func__,
				full_raw_lower_ret,
				full_raw_upper_ret);

		TOUCH_INFO_MSG("[%s][SUCCESS] Can check the limit of RT78\n",
				__func__);

		F54Test('p', 2, rawdata_file_path);
	}
	synaptics_ts_init(ts->client);
	touch_enable_irq(ts->client->irq);

	return count;
}

static ssize_t show_delta(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);

	int ret = 0;
	mutex_lock(&ts->pdata->thread_lock);
	if (need_scan_pdt) {
		SCAN_PDT();
		need_scan_pdt = false;
	}
	touch_disable_irq(ts->client->irq);
	ret = F54Test('m', 2, buf);
	synaptics_ts_init(ts->client);
	touch_enable_irq(ts->client->irq);

	if (ret == 0)
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"ERROR: full_raw_cap failed.\n");
	mutex_unlock(&ts->pdata->thread_lock);
	return ret;
}

static ssize_t show_chstatus(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);

	int ret = 0;
	int high_resistance = 0;
	int trx_to_trx = 0;

	if (power_state == POWER_ON || power_state == POWER_WAKE) {
		mutex_lock(&ts->pdata->thread_lock);
		touch_disable_irq(ts->client->irq);

		if (need_scan_pdt) {
			SCAN_PDT();
			need_scan_pdt = false;
		}
		high_resistance = F54Test('g', 0, buf);
		trx_to_trx = F54Test('f', 0, buf);

		ret = snprintf(buf, PAGE_SIZE-ret, "========RESULT=======\n");

		ret += snprintf(buf+ret, PAGE_SIZE-ret, "TRex Short Report : RESULT: %s",
				(trx_to_trx > 0) ? "Pass\n" : "Fail\n");

		/*High Resistance always return fail, you should see raw data.*/
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"High Resistance   : RESULT: %s",
				(high_resistance > 0) ? "Pass\n" : "Fail\n");

		synaptics_ts_init(ts->client);

		touch_enable_irq(ts->client->irq);
		mutex_unlock(&ts->pdata->thread_lock);

	} else {
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"state=[suspend]. we cannot use I2C, now. Test Result: Fail\n");
	}

	return ret;
}

static ssize_t show_sensor_speed_test(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);

	int ret = 0;
	int sensor_speed = 0;
	int lower_ret = 0;
	int upper_ret = 0;

	if (power_state == POWER_ON || power_state == POWER_WAKE) {
		mutex_lock(&ts->pdata->thread_lock);
		touch_disable_irq(ts->client->irq);

		if (need_scan_pdt) {
			SCAN_PDT();
			need_scan_pdt = false;
		}

		lower_ret = get_limit(F12_2DTxCount,
				F12_2DRxCount,
				*ts->client,
				ts->pdata,
				"SensorSpeedLowerImageLimit",
				SensorSpeedLowerImage);
		upper_ret = get_limit(F12_2DTxCount,
				F12_2DRxCount,
				*ts->client,
				ts->pdata,
				"SensorSpeedUpperImageLimit",
				SensorSpeedUpperImage);

		if (lower_ret < 0 || upper_ret < 0) {
			TOUCH_INFO_MSG("[%s] lower return = %d upper return = %d\n",
					__func__,
					lower_ret,
					upper_ret);
			TOUCH_INFO_MSG("[%s][FAIL] Can not check the limit of sensor speed image\n",
					__func__);
			ret = snprintf(buf+ret,
					PAGE_SIZE-ret,
					"Can not check the limit of sensor speed image limit\n");
		} else {
			TOUCH_INFO_MSG("[%s] lower return = %d upper return = %d\n",
					__func__,
					lower_ret,
					upper_ret);
			TOUCH_INFO_MSG("[%s][SUCCESS] Can check the limit of sensor speed image limit\n",
					__func__);
			sensor_speed = F54Test('c', 0, buf);
		}

		ret = snprintf(buf,
				PAGE_SIZE,
				"========RESULT=======\n");

		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"Sensor Speed Test : RESULT: %s",
				(sensor_speed > 0) ? "Pass\n" : "Fail\n");

		synaptics_ts_init(ts->client);

		touch_enable_irq(ts->client->irq);
		mutex_unlock(&ts->pdata->thread_lock);

	} else {
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"state=[suspend]. we cannot use I2C, now. Test Result: Fail\n");
	}

	return ret;
}

/*
 * show_atcmd_fw_ver
 *
 * show only firmware version.
 * It will be used for AT-COMMAND
 */
static ssize_t show_atcmd_fw_ver(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);

	int ret = 0;

	if (ts->fw_info.fw_version[0] > 0x60)
		ret += snprintf(buf+ret, PAGE_SIZE-ret, "%s\n", ts->fw_info.fw_version);
	else
		ret = snprintf(buf, PAGE_SIZE-ret, "V%d.%02d (0x%X/0x%X/0x%X/0x%X)\n",
				(ts->fw_info.fw_version[3] & 0x80 ? 1 : 0),
				ts->fw_info.fw_version[3] & 0x7F,
				ts->fw_info.fw_version[0],
				ts->fw_info.fw_version[1],
				ts->fw_info.fw_version[2],
				ts->fw_info.fw_version[3]);

	return ret;
}

static ssize_t store_tci(struct i2c_client *client,
		const char *buf, size_t count)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);
	u32 type = 0, value = 0;

	sscanf(buf, "%d %d", &type, &value);

	tci_control(ts, type, (u8)value);

	return count;
}

static ssize_t show_tci(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);
	int ret = 0;
	u8 buffer[7] = {0};

	mutex_lock(&ts->pdata->thread_lock);
	touch_i2c_read(client, f12_info.ctrl_reg_addr[20], 3, buffer);
	ret += snprintf(buf, PAGE_SIZE, "report_mode [%s]\n", (buffer[2] & 0x3) == 0x2 ?
			"WAKEUP_ONLY" : "NORMAL");
	touch_i2c_read(client, f12_info.ctrl_reg_addr[27], 1, buffer);
	ret += snprintf(buf+ret, PAGE_SIZE-ret, "wakeup_gesture [%d]\n", buffer[0]);


	synaptics_ts_page_data_read(client, LPWG_PAGE, LPWG_TAPCOUNT_REG, 7, buffer);
	ret += snprintf(buf+ret, PAGE_SIZE-ret, "\n============ TCI 1 register ============\n");
	ret += snprintf(buf+ret, PAGE_SIZE-ret, "TCI1 [%s]\n", (buffer[0] & 0x1) == 1 ?
			"enabled" : "disabled");
	ret += snprintf(buf+ret, PAGE_SIZE-ret, "TCI1 Tap Count [%d]\n", (buffer[0] & 0xf8) >> 3);
	ret += snprintf(buf+ret, PAGE_SIZE-ret, "TCI1 Min InterTap [%d]\n", buffer[1]);
	ret += snprintf(buf+ret, PAGE_SIZE-ret, "TCI1 Max InterTap [%d]\n", buffer[2]);
	ret += snprintf(buf+ret, PAGE_SIZE-ret, "TCI1 Touch Slop [%d]\n", buffer[3]);
	ret += snprintf(buf+ret, PAGE_SIZE-ret, "TCI1 Tap Distance [%d]\n", buffer[4]);
	ret += snprintf(buf+ret, PAGE_SIZE-ret, "TCI1 Threshold [%d]\n", buffer[5]);
	ret += snprintf(buf+ret, PAGE_SIZE-ret, "TCI1 Interrupt Delay time [%d]\n", buffer[6]);

	synaptics_ts_page_data_read(client, LPWG_PAGE, LPWG_TAPCOUNT_REG2, 7, buffer);
	ret += snprintf(buf+ret, PAGE_SIZE-ret, "\n============ TCI 2 register ============\n");
	ret += snprintf(buf+ret, PAGE_SIZE-ret, "TCI2 [%s]\n", (buffer[0] & 0x1) == 1 ?
			"enabled" : "disabled");
	ret += snprintf(buf+ret, PAGE_SIZE-ret, "TCI2 Tap Count [%d]\n", (buffer[0] & 0xf8) >> 3);
	ret += snprintf(buf+ret, PAGE_SIZE-ret, "TCI2 Min InterTap [%d]\n", buffer[1]);
	ret += snprintf(buf+ret, PAGE_SIZE-ret, "TCI2 Max InterTap [%d]\n", buffer[2]);
	ret += snprintf(buf+ret, PAGE_SIZE-ret, "TCI2 Touch Slop [%d]\n", buffer[3]);
	ret += snprintf(buf+ret, PAGE_SIZE-ret, "TCI2 Tap Distance [%d]\n", buffer[4]);
	ret += snprintf(buf+ret, PAGE_SIZE-ret, "TCI2 Threshold [%d]\n", buffer[5]);
	ret += snprintf(buf+ret, PAGE_SIZE-ret, "TCI2 Interrupt Delay time [%d]\n", buffer[6]);
	mutex_unlock(&ts->pdata->thread_lock);
	return ret;
}

static ssize_t store_opcode(struct i2c_client *client,
		const char *buf, size_t count)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);
	u32 value = 0;

	sscanf(buf, "%x", &value);

	dcs_opcode_control(ts, DCS_OPCODE_CTRL, (u8)value);
	dcs_opcode_control(ts, DCS_COMMAND_SEND, WRITE_DCS_VAL);

	return count;
}

static ssize_t show_opcode(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);

	int ret = 0;

	ret += snprintf(buf+ret, PAGE_SIZE-ret, "Command opcode Register Address : [0x%02x%02x]\n", VIDEO_PAGE, DCS_CMD_DATA_REG);
	ret += snprintf(buf+ret, PAGE_SIZE-ret, "Command DCS Send Register Address : [0x%02x%02x]\n", VIDEO_PAGE, DCS_CMD_SEND_REG);
	ret += snprintf(buf+ret, PAGE_SIZE-ret, "=====  Opcode Usage  =====\n");
	ret += snprintf(buf+ret, PAGE_SIZE-ret, "echo [value] > opcode\n");
	return ret;
}

static ssize_t store_reg_ctrl(struct i2c_client *client,
		const char *buf, size_t count)
{
	u8 buffer[50] = {0};
	char command[6] = {0};
	u32 page = 0;
	u32 reg = 0;
	int offset = 0;
	u32 value = 0;

	sscanf(buf, "%s %x %x %d %x ", command, &page, &reg, &offset, &value);

	if (!strcmp(command, "write")) {
		synaptics_ts_page_data_read(client, page, reg,
				offset+1, buffer);
		buffer[offset] = (u8)value;
		if (synaptics_ts_page_data_write(client, page, reg,
				offset+1, buffer) < 0) {
			TOUCH_INFO_MSG("%s : RMI Write fail....\n", __func__);
		} else {
			TOUCH_INFO_MSG("%s : RMI WRITE - page[0x%02x] reg[0x%02x] offset[+%d] = 0x%x\n",
				__func__, page, reg, offset, buffer[offset]);
		}
	} else if (!strcmp(command, "read")) {
		synaptics_ts_page_data_read(client, page, reg,
				offset+1, buffer);
		TOUCH_DEBUG(DEBUG_BASE_INFO, "page[0x%02x] reg[0x%02x] offset[+%d] = 0x%x\n",
				page, reg, offset, buffer[offset]);
	} else {
		TOUCH_DEBUG(DEBUG_BASE_INFO, "Usage\n");
		TOUCH_DEBUG(DEBUG_BASE_INFO, "Write - page(HEXA) reg(HEXA) offset(DECIMAL) value(HEXA)\n");
		TOUCH_DEBUG(DEBUG_BASE_INFO, "Read - page(HEXA) reg(HEXA) offset(DECIMAL)\n");
	}
	return count;
}

static ssize_t show_object_report(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)get_touch_handle(client);

	int ret = 0;
	u8 object_report_enable_reg_addr = 0;
	u8 object_report_enable_reg = 0;

	object_report_enable_reg_addr = f12_info.ctrl_reg_addr[23];

	mutex_lock(&ts->pdata->thread_lock);
	ret = touch_i2c_read(client, object_report_enable_reg_addr,
			sizeof(object_report_enable_reg),
			&object_report_enable_reg);

	if (ret < 0) {
		ret = snprintf(buf, PAGE_SIZE, "%s: Failed to read object_report_enable register\n", __func__);
	} else {
		u8 temp[8];
		int i;

		for (i = 0; i < 8; i++)
			temp[i] = (object_report_enable_reg >> i) & 0x01;

		ret = snprintf(buf, PAGE_SIZE, "\n============ read object_report_enable register ============\n");
		ret += snprintf(buf+ret, PAGE_SIZE-ret, " Addr  Bit7  Bit6  Bit5  Bit4  Bit3  Bit2  Bit1  Bit0  HEX\n");
		ret += snprintf(buf+ret, PAGE_SIZE-ret, "------------------------------------------------------------\n");
		ret += snprintf(buf+ret, PAGE_SIZE-ret, " 0x%02X   %d     %d     %d     %d     %d     %d     %d     %d    0x%02X\n",
				object_report_enable_reg_addr, temp[7], temp[6], temp[5], temp[4], temp[3], temp[2], temp[1], temp[0], object_report_enable_reg);
		ret += snprintf(buf+ret, PAGE_SIZE-ret, "------------------------------------------------------------\n");
		ret += snprintf(buf+ret, PAGE_SIZE-ret, " Bit0     :     [F]inger                  ->     %s\n", temp[0] ? "Enable" : "Disable");
		ret += snprintf(buf+ret, PAGE_SIZE-ret, " Bit1     :     [S]tylus                  ->     %s\n", temp[1] ? "Enable" : "Disable");
		ret += snprintf(buf+ret, PAGE_SIZE-ret, " Bit2     :     [P]alm                    ->     %s\n", temp[2] ? "Enable" : "Disable");
		ret += snprintf(buf+ret, PAGE_SIZE-ret, " Bit3     :     [U]nclassified Object     ->     %s\n", temp[3] ? "Enable" : "Disable");
		ret += snprintf(buf+ret, PAGE_SIZE-ret, " Bit4     :     [H]overing Finger         ->     %s\n", temp[4] ? "Enable" : "Disable");
		ret += snprintf(buf+ret, PAGE_SIZE-ret, " Bit5     :     [G]loved Finger           ->     %s\n", temp[5] ? "Enable" : "Disable");
		ret += snprintf(buf+ret, PAGE_SIZE-ret, " Bit6     :     [N]arrow Object Swipe     ->     %s\n", temp[6] ? "Enable" : "Disable");
		ret += snprintf(buf+ret, PAGE_SIZE-ret, " Bit7     :     Hand[E]dge                ->     %s\n", temp[7] ? "Enable" : "Disable");
		ret += snprintf(buf+ret, PAGE_SIZE-ret, "============================================================\n\n");
	}
	mutex_unlock(&ts->pdata->thread_lock);
	return ret;
}

static ssize_t store_object_report(struct i2c_client *client,
		const char *buf, size_t count)
{
	int ret;
	char select[16];
	u8 value = 2;
	int select_cnt;
	int i;
	u8 bit_select = 0;
	u8 object_report_enable_reg_addr = 0;
	u8 object_report_enable_reg_old = 0;
	u8 object_report_enable_reg_new = 0;
	u8 old[8];
	u8 new[8];

	sscanf(buf, "%s %hhu", select, &value);

	if ((strlen(select) > 8) || (value > 1)) {
		TOUCH_INFO_MSG("<writing object_report guide>\n");
		TOUCH_INFO_MSG("echo [select] [value] > object_report\n");
		TOUCH_INFO_MSG("select: [F]inger, [S]tylus, [P]alm, [U]nclassified Object, [H]overing Finger, [G]loved Finger, [N]arrow Object Swipe, Hand[E]dge\n");
		TOUCH_INFO_MSG("select length: 1~8, value: 0~1\n");
		TOUCH_INFO_MSG("ex) echo F 1 > object_report         (enable [F]inger)\n");
		TOUCH_INFO_MSG("ex) echo s 1 > object_report         (enable [S]tylus)\n");
		TOUCH_INFO_MSG("ex) echo P 0 > object_report         (disable [P]alm)\n");
		TOUCH_INFO_MSG("ex) echo u 0 > object_report         (disable [U]nclassified Object)\n");
		TOUCH_INFO_MSG("ex) echo HgNe 1 > object_report      (enable [H]overing Finger, [G]loved Finger, [N]arrow Object Swipe, Hand[E]dge)\n");
		TOUCH_INFO_MSG("ex) echo eNGh 1 > object_report      (enable Hand[E]dge, [N]arrow Object Swipe, [G]loved Finger, [H]overing Finger)\n");
		TOUCH_INFO_MSG("ex) echo uPsF 0 > object_report      (disable [U]nclassified Object, [P]alm, [S]tylus, [F]inger)\n");
		TOUCH_INFO_MSG("ex) echo HguP 0 > object_report      (disable [H]overing Finger, [G]loved Finger, [U]nclassified Object, [P]alm)\n");
		TOUCH_INFO_MSG("ex) echo HFnuPSfe 1 > object_report  (enable all object)\n");
		TOUCH_INFO_MSG("ex) echo enghupsf 0 > object_report  (disbale all object)\n");
	} else {
		select_cnt = strlen(select);

		for (i = 0; i < select_cnt; i++) {
			switch ((char)(*(select + i))) {
			case 'F': case 'f':
				bit_select |= (0x01 << 0);
				break;   /* Bit0 : (F)inger*/
			case 'S': case 's':
				bit_select |= (0x01 << 1);
				break;   /* Bit1 : (S)tylus*/
			case 'P': case 'p':
				bit_select |= (0x01 << 2);
				break;   /* Bit2 : (P)alm*/
			case 'U': case 'u':
				bit_select |= (0x01 << 3);
				break;   /* Bit3 : (U)nclassified Object*/
			case 'H': case 'h':
				bit_select |= (0x01 << 4);
				break;   /* Bit4 : (H)overing Finger*/
			case 'G': case 'g':
				bit_select |= (0x01 << 5);
				break;   /* Bit5 : (G)loved Finger*/
			case 'N': case 'n':
				bit_select |= (0x01 << 6);
				break;   /* Bit6 : (N)arrow Object Swipe*/
			case 'E': case 'e':
				bit_select |= (0x01 << 7);
				break;   /* Bit7 : Hand(E)dge*/
			default:
				break;
			}
		}

		object_report_enable_reg_addr = f12_info.ctrl_reg_addr[23];

		ret = touch_i2c_read(client, object_report_enable_reg_addr,
				sizeof(object_report_enable_reg_old),
				&object_report_enable_reg_old);

		if (ret < 0) {
			TOUCH_ERR_MSG("Failed to read object_report_enable_reg old value\n");
			return count;
		}

		object_report_enable_reg_new = object_report_enable_reg_old;

		if (value > 0)
			object_report_enable_reg_new |= bit_select;
		else
			object_report_enable_reg_new &= ~(bit_select);

		ret = touch_i2c_write_byte(client,
				object_report_enable_reg_addr,
				object_report_enable_reg_new);

		if (ret < 0) {
			TOUCH_ERR_MSG("Failed to write object_report_enable_reg new value\n");
			return count;
		}

		ret = touch_i2c_read(client, object_report_enable_reg_addr,
				sizeof(object_report_enable_reg_new),
				&object_report_enable_reg_new);

		if (ret < 0) {
			TOUCH_ERR_MSG("Failed to read object_report_enable_reg new value\n");
			return count;
		}

		for (i = 0; i < 8; i++) {
			old[i] = (object_report_enable_reg_old >> i) & 0x01;
			new[i] = (object_report_enable_reg_new >> i) & 0x01;
		}

		TOUCH_INFO_MSG("======= write object_report_enable register (before) =======\n");
		TOUCH_INFO_MSG(" Addr  Bit7  Bit6  Bit5  Bit4  Bit3  Bit2  Bit1  Bit0  HEX\n");
		TOUCH_INFO_MSG("------------------------------------------------------------\n");
		TOUCH_INFO_MSG(" 0x%02X   %d     %d     %d     %d     %d     %d     %d     %d    0x%02X\n",
				object_report_enable_reg_addr, old[7], old[6],
				old[5], old[4], old[3], old[2], old[1], old[0],
				object_report_enable_reg_old);
		TOUCH_INFO_MSG("============================================================\n");

		TOUCH_INFO_MSG("======= write object_report_enable register (after) ========\n");
		TOUCH_INFO_MSG(" Addr  Bit7  Bit6  Bit5  Bit4  Bit3  Bit2  Bit1  Bit0  HEX\n");
		TOUCH_INFO_MSG("------------------------------------------------------------\n");
		TOUCH_INFO_MSG(" 0x%02X   %d     %d     %d     %d     %d     %d     %d     %d    0x%02X\n",
				object_report_enable_reg_addr, new[7], new[6],
				new[5], new[4], new[3], new[2], new[1], new[0],
				object_report_enable_reg_new);
		TOUCH_INFO_MSG("------------------------------------------------------------\n");
		TOUCH_INFO_MSG(" Bit0     :     [F]inger                  ->     %s\n",
				new[0] ? "Enable" : "Disable");
		TOUCH_INFO_MSG(" Bit1     :     [S]tylus                  ->     %s\n",
				new[1] ? "Enable" : "Disable");
		TOUCH_INFO_MSG(" Bit2     :     [P]alm                    ->     %s\n",
				new[2] ? "Enable" : "Disable");
		TOUCH_INFO_MSG(" Bit3     :     [U]nclassified Object     ->     %s\n",
				new[3] ? "Enable" : "Disable");
		TOUCH_INFO_MSG(" Bit4     :     [H]overing Finger         ->     %s\n",
				new[4] ? "Enable" : "Disable");
		TOUCH_INFO_MSG(" Bit5     :     [G]loved Finger           ->     %s\n",
				new[5] ? "Enable" : "Disable");
		TOUCH_INFO_MSG(" Bit6     :     [N]arrow Object Swipe     ->     %s\n",
				new[6] ? "Enable" : "Disable");
		TOUCH_INFO_MSG(" Bit7     :     Hand[E]dge                ->     %s\n",
				new[7] ? "Enable" : "Disable");
		TOUCH_INFO_MSG("============================================================\n");
	}

	return count;
}

static ssize_t store_boot_mode(struct i2c_client *client,
		const char *buf, size_t count)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);
	int boot_mode = 0;

	sscanf(buf, "%d", &boot_mode);

	switch (boot_mode) {
	case CHARGERLOGO_MODE:
		TOUCH_INFO_MSG("%s: Charger mode!!! Disable irq\n",
				__func__);
		sleep_control(ts, 0, 1);
		touch_disable_irq(ts->client->irq);
		interrupt_control(ts->client,INTERRUPT_DISABLE);
		break;
	case NORMAL_BOOT_MODE:
		TOUCH_INFO_MSG("%s: Normal boot mode!!! Enable irq\n",
				__func__);
		interrupt_control(ts->client,INTERRUPT_ENABLE);
		sleep_control(ts, 1, 1);
		break;
	default:
		break;
	}

	return count;
}

static ssize_t show_ee_test(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);

	int ret = 0;
	int ee_test = 0;

	if (power_state == POWER_ON || power_state == POWER_WAKE) {
		mutex_lock(&ts->pdata->thread_lock);
		if (need_scan_pdt) {
			SCAN_PDT();
			need_scan_pdt = false;
		}

		touch_disable_irq(ts->client->irq);

		ee_test = F54Test('u', 0, buf);

		touch_enable_irq(ts->client->irq);
		synaptics_ts_init(ts->client);

		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"Noise Delta Test : RESULT: %s",
				( ee_test > 0) ? "Pass\n" : "Fail\n");
		mutex_unlock(&ts->pdata->thread_lock);
	} else {
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"state=[suspend]. we cannot use I2C, now. Test Result: Fail\n");
	}

	return ret;

}

static ssize_t show_noise_delta_test(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);

	int ret = 0;
	int noise_delta = 0;

	if (power_state == POWER_ON || power_state == POWER_WAKE) {
		mutex_lock(&ts->pdata->thread_lock);
		if (need_scan_pdt) {
			SCAN_PDT();
			need_scan_pdt = false;
		}

		touch_disable_irq(ts->client->irq);

		noise_delta = F54Test('x', 0, buf);

		touch_enable_irq(ts->client->irq);
		synaptics_ts_init(ts->client);

		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"Noise Delta Test : RESULT: %s",
				( noise_delta > 0) ? "Pass\n" : "Fail\n");
		mutex_unlock(&ts->pdata->thread_lock);
	} else {
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"state=[suspend]. we cannot use I2C, now. Test Result: Fail\n");
	}

	return ret;

}

static ssize_t show_ts_noise(struct i2c_client *client, char *buf)
{
	int ret = 0;

	ret += snprintf(buf+ret,
			PAGE_SIZE-ret,
			"Test Count : %u\n",
			cnt);
	ret += snprintf(buf+ret,
			PAGE_SIZE-ret,
			"Current Noise State : %d\n",
			cns_aver);
	ret += snprintf(buf+ret,
			PAGE_SIZE-ret,
			"Interference Metric : %d\n",
			im_aver);
	ret += snprintf(buf+ret,
			PAGE_SIZE-ret,
			"Freq Scan IM : %d\n",
			freq_scan_im_aver);

	TOUCH_INFO_MSG("Aver: CNS[%5d] IM[%5d] FREQ_SCAN_IM[%5d] (cnt:%u)\n",
			cns_aver, im_aver,
			freq_scan_im_aver,
			cnt);

	return ret;
}

static ssize_t store_ts_noise(struct i2c_client *client,
		const char *buf, size_t count)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);
	int value;

	sscanf(buf, "%d", &value);

	if ((ts->ts_state_flag.check_noise_menu == MENU_OUT)
			&& (value == MENU_ENTER)) {
		ts->ts_state_flag.check_noise_menu = MENU_ENTER;
	} else if ((ts->ts_state_flag.check_noise_menu == MENU_ENTER)
			&& (value == MENU_OUT)) {
		ts->ts_state_flag.check_noise_menu = MENU_OUT;
	} else {
		TOUCH_INFO_MSG("Already entered Check Noise menu .\n");
		TOUCH_INFO_MSG("check_noise_menu:%d, value:%d\n",
				ts->ts_state_flag.check_noise_menu, value);
		return count;
	}

	TOUCH_INFO_MSG("Check Noise = %s\n",
			(ts->ts_state_flag.check_noise_menu == MENU_OUT) ?
			"MENU_OUT" : "MENU_ENTER");
	TOUCH_INFO_MSG("TA state = %s\n",
		(touch_ta_status) ? "TA_CONNECTED" : "TA_DISCONNECTED");

	return count;
}

static ssize_t show_ts_noise_log_enable(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);
	int ret = 0;

	ret += snprintf(buf + ret, PAGE_SIZE-ret, "%d\n",
			ts->ts_state_flag.ts_noise_log_flag);
	TOUCH_INFO_MSG("ts noise log flag = %s\n",
			(ts->ts_state_flag.ts_noise_log_flag
			 == TS_NOISE_LOG_DISABLE) ?
			"TS_NOISE_LOG_DISABLE" : "TS_NOISE_LOG_ENABLE");

	return ret;
}

static ssize_t store_ts_noise_log_enable(struct i2c_client *client,
		const char *buf, size_t count)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);
	int value;

	sscanf(buf, "%d", &value);

	if ((ts->ts_state_flag.ts_noise_log_flag == TS_NOISE_LOG_DISABLE)
			&& (value == TS_NOISE_LOG_ENABLE)) {
		ts->ts_state_flag.ts_noise_log_flag = TS_NOISE_LOG_ENABLE;
	} else if ((ts->ts_state_flag.ts_noise_log_flag == TS_NOISE_LOG_ENABLE)
			&& (value == TS_NOISE_LOG_DISABLE)) {
		ts->ts_state_flag.ts_noise_log_flag = TS_NOISE_LOG_DISABLE;
	} else {
		TOUCH_INFO_MSG("Already Enable TS Noise Log.\n");
		TOUCH_INFO_MSG("ts_noise_log_flag:%d, value:%d\n",
				ts->ts_state_flag.ts_noise_log_flag, value);
		return count;
	}

	TOUCH_INFO_MSG("ts noise log flag = %s\n",
			(ts->ts_state_flag.ts_noise_log_flag ==
			 TS_NOISE_LOG_DISABLE) ?
			"TS_NOISE_LOG_DISABLE" : "TS_NOISE_LOG_ENABLE");
	TOUCH_INFO_MSG("TA state = %s\n",
		(touch_ta_status) ? "TA_CONNECTED" : "TA_DISCONNECTED");

	return count;
}

static ssize_t show_pen_support(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)get_touch_handle(client);
	int ret = 0;
	int pen_support = 0;   /* 1: Support , 0: Not support */

	pen_support = GET_OBJECT_REPORT_INFO(ts->object_report, OBJECT_STYLUS_BIT);

	TOUCH_INFO_MSG("Read Pen Support : %d\n", pen_support);
	ret = snprintf(buf, PAGE_SIZE, "%d\n", pen_support);

	   return ret;
}

static ssize_t show_palm_ctrl_mode(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)get_touch_handle(client);
	int ret = 0;

	ret = snprintf(buf, PAGE_SIZE, "%u\n", ts->pdata->role->palm_ctrl_mode);

	return ret;
}

static ssize_t store_palm_ctrl_mode(struct i2c_client *client, const char *buf, size_t count)
{
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)get_touch_handle(client);
	int value;

	sscanf(buf, "%d", &value);

	if (value < PALM_REJECT_FW || value > PALM_REPORT) {
		TOUCH_INFO_MSG("Invalid palm_ctrl_mode:%d (palm_ctrl_mode -> PALM_REJECT_FW)\n", value);
		value = PALM_REJECT_FW;
	}

	ts->pdata->role->palm_ctrl_mode = value;
	TOUCH_INFO_MSG("palm_ctrl_mode:%u\n", ts->pdata->role->palm_ctrl_mode);

	return count;
}

static ssize_t show_use_hover_finger(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)get_touch_handle(client);
	int ret = 0;

	ret = sprintf(buf, "%u\n", ts->pdata->role->use_hover_finger);

	return ret;
}

static ssize_t store_use_hover_finger(struct i2c_client *client, const char *buf, size_t count)
{
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)get_touch_handle(client);
	int value;

	sscanf(buf, "%d", &value);

	if (value < 0 || value > 1) {
		TOUCH_INFO_MSG("Invalid use_hover_finger value:%d\n", value);
		return count;
	}

	ts->pdata->role->use_hover_finger = value;
	TOUCH_INFO_MSG("use_hover_finger:%u\n", ts->pdata->role->use_hover_finger);

	return count;
}

static ssize_t show_use_rmi_dev(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)get_touch_handle(client);
	int ret = 0;

	ret = sprintf(buf, "%u\n", ts->pdata->role->use_rmi_dev);

	return ret;
}

static ssize_t store_use_rmi_dev(struct i2c_client *client, const char *buf, size_t count)
{
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)get_touch_handle(client);
	int value;

	sscanf(buf, "%d", &value);

	if (value < 0 || value > 1) {
		TOUCH_INFO_MSG("Invalid use_rmi_dev value:%d\n", value);
		return count;
	}

	ts->pdata->role->use_rmi_dev = value;
	TOUCH_INFO_MSG("use_rmi_dev:%u\n", ts->pdata->role->use_rmi_dev);

	return count;
}

static ssize_t show_swipe_param(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)get_touch_handle(client);
	struct swipe_data *swp = &ts->ts_swipe_data;
	int ret = 0;

	ret += snprintf(buf+ret, PAGE_SIZE-ret,
			"support_swipe = %d\n",
			swp->support_swipe);

	ret += snprintf(buf+ret, PAGE_SIZE-ret,
			"swipe_min_distance = 0x%02X (%dmm)\n",
			swp->swipe_min_distance,
			swp->swipe_min_distance);

	ret += snprintf(buf+ret, PAGE_SIZE-ret,
			"swipe_ratio_threshold = 0x%02X (%d%%)\n",
			swp->swipe_ratio_threshold,
			swp->swipe_ratio_threshold);

	ret += snprintf(buf+ret, PAGE_SIZE-ret,
			"swipe_ratio_check_period = 0x%02X (%dframes)\n",
			swp->swipe_ratio_check_period,
			swp->swipe_ratio_check_period);

	ret += snprintf(buf+ret, PAGE_SIZE-ret,
			"swipe_ratio_check_min_distance = 0x%02X (%dmm)\n",
			swp->swipe_ratio_check_min_distance,
			swp->swipe_ratio_check_min_distance);

	ret += snprintf(buf+ret, PAGE_SIZE-ret,
			"min_swipe_time_threshold =  0x%02X (%d0ms)\n",
			swp->min_swipe_time_threshold,
			swp->min_swipe_time_threshold);

	ret += snprintf(buf+ret, PAGE_SIZE-ret,
			"max_swipe_time_threshold = 0x%02X (%d0ms)\n",
			swp->max_swipe_time_threshold,
			swp->max_swipe_time_threshold);

	return ret;
}

static ssize_t store_swipe_param(struct i2c_client *client,
		const char *buf, size_t count)
{
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)get_touch_handle(client);
	struct swipe_data *swp = &ts->ts_swipe_data;
	char select;
	u16 value;

	if (swp->support_swipe == NO_SUPPORT_SWIPE) {
		TOUCH_ERR_MSG("support_swipe:%d\n", swp->support_swipe);
		return count;
	}

	sscanf(buf, "%c %hu", &select, &value);

	if ((select < 'a') || (select > 'f')) {
		TOUCH_INFO_MSG("<writing swipe_param guide>\n");
		TOUCH_INFO_MSG("echo [select] [value] > swipe_param\n");
		TOUCH_INFO_MSG("select: a [swipe min distance], value [0x00~0xFF] (unit:1mm)\n");
		TOUCH_INFO_MSG("select: b [swipe ratio threshold], value [0x00~0xFF] (unit:%%)\n");
		TOUCH_INFO_MSG("select: c [swipe ratio check period], value [0x00~0xFF] (unit:frames)\n");
		TOUCH_INFO_MSG("select: d [swipe ratio check min distance], value [0x00~0xFF] (unit:1mm)\n");
		TOUCH_INFO_MSG("select: e [min swipe time threshold], value [0x00~0xFFFF] (unit:10ms)\n");
		TOUCH_INFO_MSG("select: f [max swipe time threshold], value [0x00~0xFFFF] (unit:10ms)\n");

		return count;
	}

	switch (select) {
		case 'a':
			swp->swipe_min_distance = GET_LOW_U8_FROM_U16(value);
			break;
		case 'b':
			swp->swipe_ratio_threshold = GET_LOW_U8_FROM_U16(value);
			break;
		case 'c':
			swp->swipe_ratio_check_period = GET_LOW_U8_FROM_U16(value);
			break;
		case 'd':
			swp->swipe_ratio_check_min_distance = GET_LOW_U8_FROM_U16(value);
			break;
		case 'e':
			swp->min_swipe_time_threshold = value;
			break;
		case 'f':
			swp->max_swipe_time_threshold = value;
			break;
		default:
			break;
	}

	mutex_lock(&ts->pdata->thread_lock);
	swipe_enable(ts);
	mutex_unlock(&ts->pdata->thread_lock);

	return count;
}

static ssize_t show_mfts_enable(struct i2c_client *client, char *buf)
{
	int ret = 0;
	ret = sprintf(buf, "%d\n", mfts_enable);
	return ret;
}

static ssize_t store_mfts_enable(struct i2c_client *client, const char *buf, size_t count)
{
	int value;

	sscanf(buf, "%d", &value);

	mfts_enable = value;
	TOUCH_INFO_MSG("mfts_enable:%d\n", mfts_enable);

	return count;
}

static ssize_t show_factory_mode(struct i2c_client *client, char *buf)
{
	int ret;

	ret = sprintf(buf, "%d\n", factory_boot);
	TOUCH_INFO_MSG("factory_mode:%d\n", factory_boot);

	return ret;
}

static ssize_t store_factory_mode(struct i2c_client *client, const char *buf, size_t count)
{
	int value;

	sscanf(buf, "%d", &value);

	factory_boot = value;
	TOUCH_INFO_MSG("factory_mode:%d\n", factory_boot);

	return count;
}

static LGE_TOUCH_ATTR(firmware, S_IRUGO | S_IWUSR, show_firmware, NULL);
static LGE_TOUCH_ATTR(sd, S_IRUGO | S_IWUSR, show_sd, NULL);
static LGE_TOUCH_ATTR(rawdata, S_IRUGO | S_IWUSR, show_rawdata, store_rawdata);
static LGE_TOUCH_ATTR(delta, S_IRUGO | S_IWUSR, show_delta, NULL);
static LGE_TOUCH_ATTR(chstatus, S_IRUGO | S_IWUSR, show_chstatus, NULL);
static LGE_TOUCH_ATTR(testmode_ver, S_IRUGO | S_IWUSR, show_atcmd_fw_ver, NULL);
static LGE_TOUCH_ATTR(tci, S_IRUGO | S_IWUSR, show_tci, store_tci);
static LGE_TOUCH_ATTR(reg_ctrl, S_IRUGO | S_IWUSR, NULL, store_reg_ctrl);
static LGE_TOUCH_ATTR(object_report, S_IRUGO | S_IWUSR,
		show_object_report, store_object_report);
static LGE_TOUCH_ATTR(version, S_IRUGO | S_IWUSR,
		show_synaptics_fw_version, NULL);
static LGE_TOUCH_ATTR(bootmode, S_IRUGO | S_IWUSR, NULL, store_boot_mode);
static LGE_TOUCH_ATTR(ts_noise, S_IRUGO | S_IWUSR,
		show_ts_noise, store_ts_noise);
static LGE_TOUCH_ATTR(ts_noise_log_enable, S_IRUGO | S_IWUSR,
		show_ts_noise_log_enable, store_ts_noise_log_enable);
static LGE_TOUCH_ATTR(palm_ctrl_mode, S_IRUGO | S_IWUSR, show_palm_ctrl_mode, store_palm_ctrl_mode);
static LGE_TOUCH_ATTR(use_hover_finger, S_IRUGO | S_IWUSR, show_use_hover_finger, store_use_hover_finger);
static LGE_TOUCH_ATTR(use_rmi_dev, S_IRUGO | S_IWUSR, show_use_rmi_dev, store_use_rmi_dev);
static LGE_TOUCH_ATTR(sensor_speed_test, S_IRUGO | S_IWUSR, show_sensor_speed_test, NULL);
static LGE_TOUCH_ATTR(noise_delta_test, S_IRUGO | S_IWUSR, show_noise_delta_test, NULL);
static LGE_TOUCH_ATTR(ee_test, S_IRUGO | S_IWUSR, show_ee_test, NULL);
static LGE_TOUCH_ATTR(opcode, S_IRUGO | S_IWUSR, show_opcode, store_opcode);
static LGE_TOUCH_ATTR(pen_support, S_IRUGO | S_IWUSR, show_pen_support, NULL);
static LGE_TOUCH_ATTR(swipe_param, S_IRUGO | S_IWUSR, show_swipe_param, store_swipe_param);
static LGE_TOUCH_ATTR(mfts, S_IRUGO | S_IWUSR, show_mfts_enable, store_mfts_enable);
static LGE_TOUCH_ATTR(factory, S_IRUGO | S_IWUSR, show_factory_mode, store_factory_mode);

static struct attribute *lge_specific_touch_attribute_list[] = {
	&lge_touch_attr_firmware.attr,
	&lge_touch_attr_sd.attr,
	&lge_touch_attr_rawdata.attr,
	&lge_touch_attr_delta.attr,
	&lge_touch_attr_chstatus.attr,
	&lge_touch_attr_testmode_ver.attr,
	&lge_touch_attr_tci.attr,
	&lge_touch_attr_reg_ctrl.attr,
	&lge_touch_attr_object_report.attr,
	&lge_touch_attr_version.attr,
	&lge_touch_attr_bootmode.attr,
	&lge_touch_attr_ts_noise.attr,
	&lge_touch_attr_ts_noise_log_enable.attr,
	&lge_touch_attr_palm_ctrl_mode.attr,
	&lge_touch_attr_use_hover_finger.attr,
	&lge_touch_attr_use_rmi_dev.attr,
	&lge_touch_attr_sensor_speed_test.attr,
	&lge_touch_attr_noise_delta_test.attr,
	&lge_touch_attr_ee_test.attr,
	&lge_touch_attr_opcode.attr,
	&lge_touch_attr_pen_support.attr,
	&lge_touch_attr_swipe_param.attr,
	&lge_touch_attr_mfts.attr,
	&lge_touch_attr_factory.attr,
	NULL,
};

static int read_page_description_table(struct i2c_client *client)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);
	struct function_descriptor buffer;

	unsigned short u_address = 0;
	unsigned short page_num = 0;

	TOUCH_TRACE();

	memset(&buffer, 0x0, sizeof(struct function_descriptor));
	memset(&ts->common_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->finger_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->sensor_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->analog_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->flash_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->video_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->custom_fc, 0x0, sizeof(struct ts_ic_function));

	for (page_num = 0; page_num < PAGE_MAX_NUM; page_num++) {
		DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, page_num),
				error);

		for (u_address = DESCRIPTION_TABLE_START; u_address > 10;
			u_address -= sizeof(struct function_descriptor)) {

			DO_SAFE(touch_i2c_read(client, u_address,
						sizeof(buffer),
						(unsigned char *)&buffer) < 0,
					error);

			if (buffer.id == 0)
				break;

			switch (buffer.id) {
			case RMI_DEVICE_CONTROL:
				ts->common_fc.dsc = buffer;
				ts->common_fc.function_page = page_num;
				break;
			case TOUCHPAD_SENSORS:
				ts->finger_fc.dsc = buffer;
				ts->finger_fc.function_page = page_num;
				break;
			case SENSOR_CONTROL:
				ts->sensor_fc.dsc = buffer;
				ts->sensor_fc.function_page = page_num;
				break;
			case ANALOG_CONTROL:
				ts->analog_fc.dsc = buffer;
				ts->analog_fc.function_page = page_num;
				break;
			case FLASH_MEMORY_MANAGEMENT:
				ts->flash_fc.dsc = buffer;
				ts->flash_fc.function_page = page_num;
				break;
			case VIDEO_CONTROL:
				ts->video_fc.dsc = buffer;
				ts->video_fc.function_page = page_num;
				break;
			case CUSTOM_CONTROL:
				ts->custom_fc.dsc = buffer;
				ts->custom_fc.function_page = page_num;
				break;
			default:
				break;
			}
		}
	}

	DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, 0x00), error);

	TOUCH_DEBUG(DEBUG_BASE_INFO,
		"common[%dP:0x%02x] finger[%dP:0x%02x] sensor[%dP:0x%02x]"
		"analog[%dP:0x%02x] flash[%dP:0x%02x] video[%dP:0x%02x] custom[%dP:0x%02x]\n",
			ts->common_fc.function_page, ts->common_fc.dsc.id,
			ts->finger_fc.function_page, ts->finger_fc.dsc.id,
			ts->sensor_fc.function_page, ts->sensor_fc.dsc.id,
			ts->analog_fc.function_page, ts->analog_fc.dsc.id,
			ts->flash_fc.function_page, ts->flash_fc.dsc.id,
			ts->video_fc.function_page, ts->video_fc.dsc.id,
			ts->custom_fc.function_page, ts->custom_fc.dsc.id);

	ERROR_IF(ts->common_fc.dsc.id == 0 || ts->finger_fc.dsc.id == 0 ||
			ts->sensor_fc.dsc.id == 0 || ts->analog_fc.dsc.id == 0 ||
			ts->flash_fc.dsc.id == 0 || ts->video_fc.dsc.id == 0 ||
			ts->custom_fc.dsc.id == 0,
			"page_init_error", init_error);

	get_f12_info(ts);

	return 0;

init_error:
	TOUCH_ERR_MSG("%s, %d : read page failed\n", __func__, __LINE__);
	return -EINVAL;

error:
	TOUCH_ERR_MSG("%s, %d : read page failed\n", __func__, __LINE__);
	return -EIO;
}

static int get_swipe_info(struct synaptics_ts_data *ts)
{
	struct swipe_data *swp = &ts->ts_swipe_data;
	u8 lpwg_properties_reg = ts->custom_fc.dsc.query_base + 4;
	u8 has_swipe_mask = 0x10;
	u8 buf = 0;

	memset(swp, 0, sizeof(struct swipe_data));

	if (!strncmp(ts->fw_info.fw_product_id, "PLG463", 6)
	    || !strncmp(ts->fw_info.fw_product_id, "PLG516", 6)) {

		synaptics_ts_page_data_read(ts->client, LPWG_PAGE,
				lpwg_properties_reg, 1, &buf);

		TOUCH_INFO_MSG("%s: lpwg_properties_reg"
				"[addr:0x%02X, value:0x%02X\n",
				__func__, lpwg_properties_reg, buf);

		if (!(buf & has_swipe_mask)) {
			TOUCH_INFO_MSG("%s: Need to check Has Swipe bit\n", __func__);
			swp->support_swipe = NO_SUPPORT_SWIPE;
			return 0;
		}

		swp->support_swipe = SUPPORT_SWIPE;
		swp->swipe_enable_mask = 0x01;
		swp->swipe_gesture = 0x04;
		swp->swipe_min_distance = 16;
		swp->swipe_ratio_threshold = 200;
		swp->swipe_ratio_check_period = 5;
		swp->swipe_ratio_check_min_distance = 2;
		swp->min_swipe_time_threshold = 0;
		swp->max_swipe_time_threshold = 150;
		swp->swipe_coordinate_start_reg = ts->custom_fc.dsc.data_base + 74;
		swp->swipe_coordinate_end_reg = ts->custom_fc.dsc.data_base + 78;
		swp->swipe_debug_reason_reg = ts->custom_fc.dsc.data_base + 82;
		swp->swipe_time_reg = ts->custom_fc.dsc.data_base + 83;
		swp->swipe_enable_reg = ts->custom_fc.dsc.control_base + 17;
		swp->swipe_min_distance_reg = ts->custom_fc.dsc.control_base + 18;
		swp->swipe_ratio_threshold_reg = ts->custom_fc.dsc.control_base + 19;
		swp->swipe_ratio_check_period_reg = ts->custom_fc.dsc.control_base + 20;
		swp->swipe_ratio_check_min_distance_reg = ts->custom_fc.dsc.control_base + 21;
		swp->min_swipe_time_thres_lsb_reg = ts->custom_fc.dsc.control_base + 22;
		swp->min_swipe_time_thres_msb_reg = ts->custom_fc.dsc.control_base + 23;
		swp->max_swipe_time_thres_lsb_reg = ts->custom_fc.dsc.control_base + 24;
		swp->max_swipe_time_thres_msb_reg = ts->custom_fc.dsc.control_base + 25;

		TOUCH_INFO_MSG("support_swipe = %d\n", swp->support_swipe);
		TOUCH_INFO_MSG("swipe_enable_mask = 0x%02X\n", swp->swipe_enable_mask);
		TOUCH_INFO_MSG("swipe_gesture = 0x%02X\n", swp->swipe_gesture);
		TOUCH_INFO_MSG("swipe_min_distance = %d\n", swp->swipe_min_distance);
		TOUCH_INFO_MSG("swipe_ratio_threshold = %d\n", swp->swipe_ratio_threshold);
		TOUCH_INFO_MSG("swipe_ratio_check_period = %d\n", swp->swipe_ratio_check_period);
		TOUCH_INFO_MSG("swipe_ratio_check_min_distance = %d\n", swp->swipe_ratio_check_min_distance);
		TOUCH_INFO_MSG("min_swipe_time_threshold = %d\n", swp->min_swipe_time_threshold);
		TOUCH_INFO_MSG("max_swipe_time_threshold = %d\n", swp->max_swipe_time_threshold);
		TOUCH_INFO_MSG("swipe_coordinate_start_reg = 0x%02X\n", swp->swipe_coordinate_start_reg);
		TOUCH_INFO_MSG("swipe_coordinate_end_reg = 0x%02X\n", swp->swipe_coordinate_end_reg);
		TOUCH_INFO_MSG("swipe_debug_reason_reg = 0x%02X\n", swp->swipe_debug_reason_reg);
		TOUCH_INFO_MSG("swipe_time_reg = 0x%02X\n", swp->swipe_time_reg)
		TOUCH_INFO_MSG("swipe_enable_reg = 0x%02X\n", swp->swipe_enable_reg);
		TOUCH_INFO_MSG("swipe_min_distance_reg = 0x%02X\n", swp->swipe_min_distance_reg);
		TOUCH_INFO_MSG("swipe_ratio_threshold_reg = 0x%02X\n", swp->swipe_ratio_threshold_reg);
		TOUCH_INFO_MSG("swipe_ratio_check_period_reg = 0x%02X\n", swp->swipe_ratio_check_period_reg);
		TOUCH_INFO_MSG("swipe_ratio_check_min_distance_reg = 0x%02X\n", swp->swipe_ratio_check_min_distance_reg);
		TOUCH_INFO_MSG("min_swipe_time_thres_lsb_reg = 0x%02X\n", swp->min_swipe_time_thres_lsb_reg);
		TOUCH_INFO_MSG("min_swipe_time_thres_msb_reg = 0x%02X\n", swp->min_swipe_time_thres_msb_reg);
		TOUCH_INFO_MSG("max_swipe_time_thres_lsb_reg = 0x%02X\n", swp->max_swipe_time_thres_lsb_reg);
		TOUCH_INFO_MSG("max_swipe_time_thres_msb_reg = 0x%02X\n", swp->max_swipe_time_thres_msb_reg);

	} else {
		TOUCH_INFO_MSG("%s: Need to check fw_product_id\n", __func__);
		swp->support_swipe = NO_SUPPORT_SWIPE;
	}

	return 0;
}

static int get_ic_info(struct synaptics_ts_data *ts)
{
	const struct firmware *fw_entry = NULL;
	const u8 *fw = NULL;

	int rc = 0;
	u8 buf = 0;

	DO_SAFE(synaptics_ts_page_data_read(ts->client,
		LPWG_PAGE, LPWG_HAS_DEBUG_MODULE, 1, &buf), error);
	ts->lpwg_ctrl.has_debug_module = ((buf & 0x0F) == 0x0F) ? 1 : 0;

	TOUCH_DEBUG(DEBUG_BASE_INFO, "addr[0x%x] buf[0x%x] has_d_module[%d]",
		LPWG_HAS_DEBUG_MODULE, buf, ts->lpwg_ctrl.has_debug_module);

	memset(&ts->fw_info, 0, sizeof(struct synaptics_ts_fw_info));

	DO_SAFE(touch_i2c_read(ts->client, PRODUCT_ID_REG,
				sizeof(ts->fw_info.fw_product_id) - 1,
				ts->fw_info.fw_product_id), error);
	DO_SAFE(touch_i2c_read(ts->client, FLASH_CONFIG_ID_REG,
				sizeof(ts->fw_info.fw_version) - 1,
				ts->fw_info.fw_version), error);
	DO_SAFE(touch_i2c_read(ts->client, CUSTOMER_FAMILY_REG, 1,
				&(ts->fw_info.family)), error);
	DO_SAFE(touch_i2c_read(ts->client, FW_REVISION_REG, 1,
				&(ts->fw_info.fw_revision)), error);
	TOUCH_DEBUG(DEBUG_BASE_INFO, "IC FW_PRODUC_ID = %s\n",
			ts->fw_info.fw_product_id);
	TOUCH_DEBUG(DEBUG_BASE_INFO, "IC FW_VERSION = V%d.%02d\n",
			(ts->fw_info.fw_version[3] & 0x80 ? 1 : 0), ts->fw_info.fw_version[3] & 0x7F);
	TOUCH_DEBUG(DEBUG_BASE_INFO, "CUSTOMER_FAMILY_REG = %d\n",
			ts->fw_info.family);
	TOUCH_DEBUG(DEBUG_BASE_INFO, "FW_REVISION_REG = %d\n",
			ts->fw_info.fw_revision);

	if (!strncmp(ts->fw_info.fw_product_id, "PLG463", 6)) {
		rc = request_firmware(&fw_entry,
				ts->pdata->inbuilt_fw_name,
				&ts->client->dev);
		if (rc != 0) {
			TOUCH_ERR_MSG("request_firmware() failed %d\n", rc);
			goto error;
		}
		ts->fw_flag = TD4191;
	} else if (!strncmp(ts->fw_info.fw_product_id, "PLG516", 6)) {
		rc = request_firmware(&fw_entry,
				ts->pdata->inbuilt_fw_name,
				&ts->client->dev);
		if (rc != 0) {
			TOUCH_ERR_MSG("request_firmware() failed %d\n", rc);
			goto error;
		}
		ts->fw_flag = TD4191_TFT_ITO_PANEL;
	} else {
		TOUCH_ERR_MSG("Touch product ID read error\n");
		goto error;
	}

	TOUCH_DEBUG(DEBUG_BASE_INFO, "ts->fw_flag = %d\n", ts->fw_flag);
	fw = fw_entry->data;

	memcpy(ts->fw_info.fw_image_product_id, &fw[0x0010], 6);
	memcpy(ts->fw_info.fw_image_version, &fw[0x11100], 4);

	get_swipe_info(ts);

	release_firmware(fw_entry);
	return 0;
error:
	TOUCH_ERR_MSG("%s, %d : get_ic_info failed\n", __func__, __LINE__);
	if (fw_entry)
		release_firmware(fw_entry);
	return -EIO;
}

static int check_firmware_status(struct synaptics_ts_data *ts)
{
	u8 device_status = 0;
	u8 flash_status = 0;

	DO_SAFE(touch_i2c_read(ts->client, FLASH_STATUS_REG,
				sizeof(flash_status), &flash_status), error);
	DO_SAFE(touch_i2c_read(ts->client, DEVICE_STATUS_REG,
				sizeof(device_status), &device_status), error);

	ts->fw_info.need_rewrite_firmware = 0;

	if ((device_status & DEVICE_STATUS_FLASH_PROG)
			|| (device_status & DEVICE_CRC_ERROR_MASK)
			|| (flash_status & FLASH_STATUS_MASK)) {
		TOUCH_ERR_MSG("FLASH_STATUS[0x%x] DEVICE_STATUS[0x%x]\n",
				(u32)flash_status, (u32)device_status);
		ts->fw_info.need_rewrite_firmware = 1;
	}
	return 0;
error:
	TOUCH_ERR_MSG("%s, %d : check_firmware_status failed\n",
			__func__, __LINE__);
	return -EIO;
}

static const char *f_str[] = {
	"ERROR",
	"DISTANCE_INTER_TAP",
	"DISTANCE_TOUCHSLOP",
	"TIMEOUT_INTER_TAP",
	"MULTI_FINGER",
	"DELAY_TIME",
	"PALM_STATE",
	"ACTIVE_AREA",
	"OVER_TAP"
};

int print_tci_debug_result(struct synaptics_ts_data *ts, int num)
{
	u8 count = 0;
	u8 index = 0;
	u8 buf = 0;

	u8 i = 0;
	u8 addr = 0;
	u8 offset = num ? LPWG_MAX_BUFFER + 2 : 0;

	DO_SAFE(synaptics_ts_page_data_read(ts->client,
		LPWG_PAGE, LPWG_DEBUG_COUNT + offset, 1, &count), error);
	DO_SAFE(synaptics_ts_page_data_read(ts->client,
		LPWG_PAGE, LPWG_DEBUG_INDEX + offset, 1, &index), error);

	for (i = 1; i <= count; i++) {
		addr = LPWG_DEBUG_REASON + offset +
			 ((index + LPWG_MAX_BUFFER - i) % LPWG_MAX_BUFFER);
		DO_SAFE(synaptics_ts_page_data_read(ts->client, LPWG_PAGE,
			 addr, 1, &buf), error);
		TOUCH_DEBUG(DEBUG_BASE_INFO, "TCI(%d)-Debug buffer[%d/%d]: %s\n",
			num, count - i + 1, count,
			(buf > 0 && buf < 9) ? f_str[buf] : f_str[0]);
		if (i == LPWG_MAX_BUFFER)
			break;
	}

	return 0;
error:
	TOUCH_ERR_MSG("%s, %d : print_tci_debug_result failed\n",
			__func__, __LINE__);
	return -EIO;
}

enum error_type synaptics_ts_probe(struct i2c_client *client,
		struct touch_platform_data *lge_ts_data,
		const struct state_info *state,
		struct attribute ***attribute_list)
{
	struct synaptics_ts_data *ts;

	TOUCH_TRACE();

	ASSIGN(ts = devm_kzalloc(&client->dev, sizeof(struct synaptics_ts_data),
				GFP_KERNEL), error);
	set_touch_handle(client, ts);

	ts->client = client;
	ds4_i2c_client = client;
	ts->pdata = lge_ts_data;
	ts->state = state;
#if defined(CONFIG_TOUCHSCREEN_LGE_SYNAPTICS_TD4191)
	// Do nothing
#else
	if (ts->pdata->pwr->use_regulator) {
		DO_IF(IS_ERR(ts->regulator_vdd = regulator_get(&client->dev,
						ts->pdata->pwr->vdd)), error);
		DO_IF(IS_ERR(ts->regulator_vio = regulator_get(&client->dev,
						ts->pdata->pwr->vio)), error);
		if (ts->pdata->pwr->vdd_voltage > 0)
			DO_SAFE(regulator_set_voltage(ts->regulator_vdd,
						ts->pdata->pwr->vdd_voltage,
						ts->pdata->pwr->vdd_voltage),
					error);
		if (ts->pdata->pwr->vio_voltage > 0)
			DO_SAFE(regulator_set_voltage(ts->regulator_vio,
						ts->pdata->pwr->vio_voltage,
						ts->pdata->pwr->vio_voltage),
					error);
	}
#endif
	*attribute_list = lge_specific_touch_attribute_list;
	ts->is_probed = 0;
	ts->is_init = 0;
	ts->lpwg_ctrl.screen = 1;
	ts->lpwg_ctrl.sensor = 1;

	atomic_set(&ts->lpwg_ctrl.is_suspend, 0);
	INIT_DELAYED_WORK(&ts->work_timer, lpwg_timer_func);
	INIT_DELAYED_WORK(&ts->work_palm, all_palm_released_func);
	wake_lock_init(&ts->timer_wake_lock, WAKE_LOCK_SUSPEND, "touch_timer");

	syna_ts_data = ts;
	return NO_ERROR;
error:
	TOUCH_ERR_MSG("%s, %d : synaptics_probe failed\n", __func__, __LINE__);
	return ERROR;
}

enum error_type synaptics_ts_remove(struct i2c_client *client)
{
	struct synaptics_ts_data *ts
		= (struct synaptics_ts_data *)get_touch_handle(client);

	TOUCH_TRACE();

	if (ts->pdata->role->use_hover_finger && prox_fhandler.inserted && prox_fhandler.initialized) {
		prox_fhandler.exp_fn->remove(ts);
		prox_fhandler.initialized = false;
	}

	if (ts->pdata->role->use_rmi_dev && rmidev_fhandler.inserted && rmidev_fhandler.initialized) {
		rmidev_fhandler.exp_fn->remove(ts);
		rmidev_fhandler.initialized = false;
	}
#if defined(CONFIG_TOUCHSCREEN_LGE_SYNAPTICS_TD4191)
	//Do nothing
#else
	if (ts->pdata->pwr->use_regulator) {
		regulator_put(ts->regulator_vio);
		regulator_put(ts->regulator_vdd);
	}
#endif
	wake_lock_destroy(&ts->timer_wake_lock);
	return NO_ERROR;
}

static int lpwg_update_all(struct synaptics_ts_data *ts, bool irqctrl)
{
	int sleep_status = 0;
	int lpwg_status = 0;
	bool req_lpwg_param = false;

	TOUCH_TRACE();

	if (ts->lpwg_ctrl.screen) {
		if (atomic_read(&ts->lpwg_ctrl.is_suspend) == 1) {
			if (power_state == POWER_OFF
					|| power_state == POWER_SLEEP)
				ts->is_init = 0;
		}
		atomic_set(&ts->lpwg_ctrl.is_suspend, 0);
	} else {
		if (atomic_read(&ts->lpwg_ctrl.is_suspend) == 0) {
			atomic_set(&ts->lpwg_ctrl.is_suspend, 1);
			set_doze_param(ts, 3);
		}
	}

	if (ts->lpwg_ctrl.screen) { /* ON(1) */
		sleep_status = 1;
		lpwg_status = 0;
	} else if (!ts->lpwg_ctrl.screen /* OFF(0), CLOSED(0) */
			&& ts->lpwg_ctrl.qcover) {
		sleep_status = 1;
		lpwg_status = 1;
	} else if (!ts->lpwg_ctrl.screen /* OFF(0), OPEN(1), FAR(1) */
			&& !ts->lpwg_ctrl.qcover
			&& ts->lpwg_ctrl.sensor) {
		sleep_status = 1;
		lpwg_status = ts->lpwg_ctrl.lpwg_mode;
	} else if (!ts->lpwg_ctrl.screen /* OFF(0), OPEN(1), NEAR(0) */
			&& !ts->lpwg_ctrl.qcover
			&& !ts->lpwg_ctrl.sensor) {
		if (ts->pdata->role->crack_detection->use_crack_mode && !after_crack_check) {
			TOUCH_INFO_MSG("%s : Crack check not done... use nonsleep mode to check Crack!!\n",
					__func__);
			sleep_status = 1;
			lpwg_status = ts->lpwg_ctrl.lpwg_mode;
		} else {
			sleep_status = 0;
			req_lpwg_param = true;
		}
	}

	DO_SAFE(sleep_control(ts, sleep_status, 0), error);
	if (req_lpwg_param == false)
		DO_SAFE(lpwg_control(ts, lpwg_status), error);

	return NO_ERROR;
error:
	return ERROR;
}

enum error_type synaptics_ts_init(struct i2c_client *client)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);
	u8 buf = 0;
	u8 buf_array[2] = {0};
	int exp_fn_retval;
	u8 motion_suppression_reg_addr;
	int rc = 0;
	u8 lpwg_mode = ts->lpwg_ctrl.lpwg_mode;
	int is_suspend = atomic_read(&ts->lpwg_ctrl.is_suspend);

	TOUCH_TRACE();

	if (ts->is_probed == 0) {
		rc = read_page_description_table(ts->client);
		DO_SAFE(check_firmware_status(ts), error);
		if (rc == -EIO)
			return ERROR;
		get_ic_info(ts);
		if (rc == -EINVAL) {
			TOUCH_INFO_MSG("%s : need to rewrite firmware !!",
				__func__);
			ts->fw_info.need_rewrite_firmware = 1;
		}
		ts->is_probed = 1;
		TOUCH_INFO_MSG("%s : lge_get_boot_mode[%d]\n", __func__, lge_get_boot_mode());
	}

	if (ts->pdata->role->use_hover_finger && prox_fhandler.inserted) {
		if (!prox_fhandler.initialized) {
			exp_fn_retval = prox_fhandler.exp_fn->init(ts);

			if (exp_fn_retval < 0) {
				TOUCH_INFO_MSG("[Touch Proximity] %s: Failed to init proximity settings\n",
						__func__);
			} else
				prox_fhandler.initialized = true;
		} else {
			prox_fhandler.exp_fn->reinit(ts);
		}
	}

	if (ts->pdata->role->use_rmi_dev && rmidev_fhandler.inserted) {
		if (!rmidev_fhandler.initialized) {
			exp_fn_retval = rmidev_fhandler.exp_fn->init(ts);

			if (exp_fn_retval < 0) {
				TOUCH_INFO_MSG("[Touch RMI_Dev] %s: Failed to init rmi_dev settings\n",
						__func__);
			} else {
				rmidev_fhandler.initialized = true;
			}
		}
	}

	DO_SAFE(touch_i2c_write_byte(client, DEVICE_CONTROL_REG,
				DEVICE_CONTROL_NORMAL_OP
				| DEVICE_CONTROL_CONFIGURED), error);

	DO_SAFE(touch_i2c_read(client, INTERRUPT_ENABLE_REG, 1, &buf), error);
	if (!(strncmp(ts->fw_info.fw_product_id, "PLG313", 6)) ||
			!(strncmp(ts->fw_info.fw_product_id, "PLG352", 6)) ||
			!(strncmp(ts->fw_info.fw_product_id, "PLG391", 6)) ||
			!(strncmp(ts->fw_info.fw_product_id, "PLG463", 6)) ||
			!(strncmp(ts->fw_info.fw_product_id, "PLG516", 6))) {
		DO_SAFE(touch_i2c_write_byte(client, INTERRUPT_ENABLE_REG,
					buf | INTERRUPT_MASK_ABS0
					| INTERRUPT_MASK_CUSTOM), error);
	} else {
		DO_SAFE(touch_i2c_write_byte(client, INTERRUPT_ENABLE_REG,
					buf | INTERRUPT_MASK_ABS0), error);
	}

	if (ts->pdata->role->report_mode == REDUCED_REPORT_MODE
			&& !ts->pdata->role->ghost_detection->check_enable.long_press_chk) {
		buf_array[0] = buf_array[1] =
			ts->pdata->role->delta_pos_threshold;
	} else {
		buf_array[0] = buf_array[1] = 0;
		ts->pdata->role->ghost_detection->force_continuous_mode = true;
	}

	motion_suppression_reg_addr = f12_info.ctrl_reg_addr[20];
	DO_SAFE(touch_i2c_write(client, motion_suppression_reg_addr, 2,
				buf_array), error);

	DO_SAFE(touch_i2c_read(client, f12_info.ctrl_reg_addr[15],
				2, default_finger_amplitude), error);

	if (ts->pdata->role->palm_ctrl_mode > PALM_REPORT) {
		TOUCH_INFO_MSG("Invalid palm_ctrl_mode:%u "
				"(palm_ctrl_mode -> PALM_REJECT_FW)\n",
				ts->pdata->role->palm_ctrl_mode);
		ts->pdata->role->palm_ctrl_mode = PALM_REJECT_FW;
	}
	TOUCH_INFO_MSG("palm_ctrl_mode:%u\n", ts->pdata->role->palm_ctrl_mode);

	DO_SAFE(touch_i2c_read(client, f12_info.ctrl_reg_addr[22],
				1, &buf), error);
	buf_array[0] = buf & 0x03;

	if ((ts->pdata->role->palm_ctrl_mode == PALM_REJECT_DRIVER)
			|| (ts->pdata->role->palm_ctrl_mode == PALM_REPORT)) {
		if (buf_array[0] != 0x00) {
			/* PalmFilterMode bits[1:0] (00:Disable palm filter */
			buf &= ~(0x03);
			DO_SAFE(touch_i2c_write_byte(client,
						f12_info.ctrl_reg_addr[22],
						buf), error);
		}
		memset(&ts->ts_palm_data, 0, sizeof(struct palm_data));
	} else {
		if (buf_array[0] != 0x01) {
			/* PalmFilterMode bits[1:0] (01:Enable palm filter) */
			buf &= ~(0x02);
			buf |= 0x01;
			DO_SAFE(touch_i2c_write_byte(client,
						f12_info.ctrl_reg_addr[22],
						buf), error);
		}
	}
#if defined(CONFIG_TOUCHSCREEN_LGE_SYNAPTICS_TD4191)
	//Do nothing
#else
	if (lge_get_boot_mode() == LGE_BOOT_MODE_QEM_56K) {
		TOUCH_INFO_MSG("mini_os_finger_amplitude = 0x%02X\n",
				ts->pdata->role->mini_os_finger_amplitude);
		buf_array[0] = ts->pdata->role->mini_os_finger_amplitude;
		buf_array[1] = ts->pdata->role->mini_os_finger_amplitude;
		DO_SAFE(touch_i2c_write(client, f12_info.ctrl_reg_addr[15],
				2, buf_array), error);
	}
#endif
	if (!(strncmp(ts->fw_info.fw_product_id, "PLG313", 6)) ||
			!(strncmp(ts->fw_info.fw_product_id, "PLG352", 6)) ||
			!(strncmp(ts->fw_info.fw_product_id, "PLG391", 6)) ||
			!(strncmp(ts->fw_info.fw_product_id, "PLG463", 6)) ||
			!(strncmp(ts->fw_info.fw_product_id, "PLG516", 6))) {
		if (ts->pdata->role->use_lpwg_all)
			DO_SAFE(lpwg_update_all(ts, 0), error);
		else
			DO_SAFE(lpwg_control(ts, is_suspend ? lpwg_mode : 0),
					error);
	}

	/* It always should be done last. */
	DO_SAFE(touch_i2c_read(client, INTERRUPT_STATUS_REG, 1, &buf), error);
	ts->is_init = 1;

	return NO_ERROR;
error:
	TOUCH_ERR_MSG("%s, %d : synaptics init failed\n", __func__, __LINE__);
	return ERROR;
}

static int synaptics_ts_noise_log(struct i2c_client *client,
		struct touch_data *curr_data,
		const struct touch_data *prev_data)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);
	u8 buffer[3] = {0};
	u8 buf1 = 0, buf2 = 0, cns = 0;
	u16 im = 0, freq_scan_im = 0;
	int i = 0;

	DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, ANALOG_PAGE), error);

	DO_SAFE(touch_i2c_read(client, IM_LSB, 1, &buf1), error);
	DO_SAFE(touch_i2c_read(client, IM_MSB, 1, &buf2), error);
	im = (buf2 << 8) | buf1;
	im_sum += im;

	DO_SAFE(touch_i2c_read(client, CURR_NOISE_STATE, 1, &cns), error);
	cns_sum += cns;

	DO_SAFE(touch_i2c_read(client, FREQ_SCAN_IM, 2, buffer), error);
	freq_scan_im = (buffer[1]<<8)|buffer[0];
	freq_scan_im_sum += freq_scan_im;

	DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE), error);

	cnt++;

	if ((ts->ts_state_flag.ts_noise_log_flag == TS_NOISE_LOG_ENABLE)
			|| (touch_debug_mask & DEBUG_NOISE)) {
		if (prev_data->total_num != curr_data->total_num)
			TOUCH_INFO_MSG("Curr: CNS[%5d]   IM[%5d]   FREQ_SCAN_IM[%5d]\n",
					cns, im, freq_scan_im);
	}

	for (i = 0; i < MAX_FINGER; i++) {
		if ((prev_data->report_id_mask & (1 << i))
				&& !(curr_data->id_mask & (1 << i))) {
			break;
		}
	}
	if (((i < MAX_FINGER) && curr_data->total_num == 0)
			|| (im_sum >= ULONG_MAX || cns_sum >= ULONG_MAX
				|| cid_im_sum >= ULONG_MAX
				|| freq_scan_im_sum >= ULONG_MAX
				|| cnt >= UINT_MAX)) {
		if ((ts->ts_state_flag.ts_noise_log_flag == TS_NOISE_LOG_ENABLE)
				|| (touch_debug_mask & DEBUG_NOISE))
			TOUCH_INFO_MSG("Aver: CNS[%5lu]   IM[%5lu]   FREQ_SCAN_IM[%5lu] (cnt:%u)\n",
					cns_sum/cnt, im_sum/cnt,
					freq_scan_im_sum/cnt, cnt);

		im_aver = im_sum/cnt;
		cns_aver = cns_sum/cnt;
		freq_scan_im_aver = freq_scan_im_sum/cnt;
	}

	if (prev_data->total_num == 0 && curr_data->total_num != 0) {
		cnt = im_sum = cns_sum = cid_im_sum = freq_scan_im_sum = 0;
		im_aver = cns_aver = freq_scan_im_aver = 0;
	}
	return 0;

error:
	return -EPERM;
}

static void synaptics_ts_check_debug_reason(struct i2c_client *client, u8 cnt)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);
	int i = 0;

	for (i = 0; i < cnt; i++) {
		TOUCH_INFO_MSG("LPWG DEBUG REASON [TCI%d] =", i + 1);
		switch (ts->reason[i]) {
			case DEBUG_DISTANCE_INTER_TAP:
				printk("DISTANCE INTER TAP\n");
				break;
			case DEBUG_DISTANCE_TOUCHSLOP:
				printk("DISTANCE TOUCHSLOP\n");
				break;
			case DEBUG_TIMEOUT_INTER_TAP:
				printk("TIMEOUT INTER TAP\n");
				break;
			case DEBUG_MULTI_FINGER:
				printk("MULTI FINGER\n");
				break;
			case DEBUG_DELAY_TIME:
				printk("DELAY TIME\n");
				break;
			case DEBUG_PALM_STATE:
				printk("PALM STATE\n");
				break;
			case DEBUG_ACTIVE_AREA:
				printk("ACTIVE AREA\n");
				break;
			case DEBUG_TAP_COUNT:
				printk("TAP COUNT\n");
				break;
			default:
				printk("Unknown Debug Reason\n");
				break;
		}
	}
}

#define SWIPE_F_STR_SIZE 8
static const char *swipe_f_str[SWIPE_F_STR_SIZE] = {
	"SUCCESS",
	"FINGER_RELEASED",
	"MULTIPLE_FINGERS",
	"TOO_FAST",
	"TOO_SLOW",
	"UPWARD",
	"RATIO_EXECESS",
	"UNKNOWN"
};

static int print_swipe_debug_reason(struct synaptics_ts_data *ts)
{
	struct swipe_data *swp = &ts->ts_swipe_data;
	u8 buf = 0;

	if (swp->support_swipe == NO_SUPPORT_SWIPE) {
		TOUCH_ERR_MSG("support_swipe:%d\n", swp->support_swipe);
		return -1;
	}

	synaptics_ts_page_data_read(ts->client, LPWG_PAGE,
			swp->swipe_debug_reason_reg, 1, &buf);

	TOUCH_INFO_MSG("last swipe debug reason:%d(%s)\n", buf,
			(buf < SWIPE_F_STR_SIZE) ? swipe_f_str[buf] :
			swipe_f_str[SWIPE_F_STR_SIZE]);

	return 0;
}

static int get_swipe_data(struct i2c_client *client)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);
	u8 swipe_buf[11] = {0};
	u16 swipe_start_x = 0;
	u16 swipe_start_y = 0;
	u16 swipe_end_x = 0;
	u16 swipe_end_y = 0;
	u8 swipe_fail_reason = 0;
	u16 swipe_time = 0;

	DO_SAFE(synaptics_ts_page_data_read(client, LPWG_PAGE,
				ts->ts_swipe_data.swipe_coordinate_start_reg,
				11, swipe_buf), error);
	swipe_start_x = GET_U16_FROM_U8(swipe_buf[1], swipe_buf[0]);
	swipe_start_y = GET_U16_FROM_U8(swipe_buf[3], swipe_buf[2]);
	swipe_end_x = GET_U16_FROM_U8(swipe_buf[5], swipe_buf[4]);
	swipe_end_y = GET_U16_FROM_U8(swipe_buf[7], swipe_buf[6]);
	swipe_fail_reason = swipe_buf[8];
	swipe_time = GET_U16_FROM_U8(swipe_buf[10], swipe_buf[9]);

	if (ts->lpwg_ctrl.password_enable || ts->pdata->role->use_security_all) {
		TOUCH_DEBUG(DEBUG_BASE_INFO || DEBUG_LPWG, "LPWG Swipe Gesture: "
			"start(xxxx,xxxx) end(xxxx,xxxx) "
			"swipe_fail_reason(%d) swipe_time(%dms)\n",
			swipe_fail_reason, swipe_time);
	} else {
		TOUCH_DEBUG(DEBUG_BASE_INFO || DEBUG_LPWG, "LPWG Swipe Gesture: "
			"start(%4d,%4d) end(%4d,%4d) "
			"swipe_fail_reason(%d) swipe_time(%dms)\n",
			swipe_start_x, swipe_start_y,
			swipe_end_x, swipe_end_y,
			swipe_fail_reason, swipe_time);
	}

	if (swipe_fail_reason == 0) {
		ts->pw_data.data_num = 1;
		ts->pw_data.data[0].x = swipe_end_x;
		ts->pw_data.data[0].y = swipe_end_y;
		return 0;
	} else {
		TOUCH_INFO_MSG("swipe fail.\n");
		return -1;
	}
error:
	TOUCH_ERR_MSG("failed to read swipe data.\n");
	return -1;
}

enum error_type synaptics_ts_get_data(struct i2c_client *client,
		struct touch_data *curr_data,
		const struct touch_data *prev_data)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);

	u8  i = 0;
	u8  finger_index = 0;
	u8  object_attn[(MAX_NUM_OF_FINGERS + 7) / 8] = {0};
	u16 object_attn_data = 0;
	u8 curr_object = 0;
	TOUCH_TRACE();

	if (!ts->is_init)
		return IGNORE_EVENT;

	curr_data->total_num = 0;
	curr_data->id_mask = 0;

	switch (ts->pdata->role->palm_ctrl_mode) {
	case PALM_REJECT_DRIVER:
	case PALM_REPORT:
		memset(ts->ts_palm_data.curr_palm_mask, 0,
				sizeof(ts->ts_palm_data.curr_palm_mask));
		break;
	case PALM_REJECT_FW:
	default:
		break;
	}

	DO_SAFE(touch_i2c_read(client, DEVICE_STATUS_REG,
				sizeof(ts->ts_data.device_status_reg),
				&ts->ts_data.device_status_reg), error);
	if (ts->ts_data.device_status_reg != 0x00)
		TOUCH_DEBUG(DEBUG_BASE_INFO || DEBUG_LPWG,
			"!!!INT!!! DEVICE_STATUS_REG = 0x%02X  something wrong!\n",
			ts->ts_data.device_status_reg);

	DO_SAFE(touch_i2c_read(client, INTERRUPT_STATUS_REG,
				sizeof(ts->ts_data.interrupt_status_reg),
				&ts->ts_data.interrupt_status_reg), error);
	if (ts->ts_data.interrupt_status_reg != INTERRUPT_MASK_ABS0
		&& ts->ts_data.interrupt_status_reg != INTERRUPT_MASK_CUSTOM)
		TOUCH_DEBUG(DEBUG_BASE_INFO || DEBUG_LPWG,
			"!!!INT!!! INTERRUPT_STATUS_REG = 0x%02X (0x40==CUSTOM)\n",
			ts->ts_data.interrupt_status_reg);

	if ((ts->ts_data.device_status_reg & DEVICE_FAILURE_MASK) == DEVICE_FAILURE_MASK) {
		if (ts->ts_data.interrupt_status_reg & INTERRUPT_MASK_STATUS) {
			TOUCH_ERR_MSG("%s,: ESD detected !\n", __func__);
			TOUCH_ERR_MSG("%s,: ESD device_status_reg=0x%02X, interrupt_status_reg=0x%02X\n", __func__,
					ts->ts_data.device_status_reg,
					ts->ts_data.interrupt_status_reg);
#if defined(CONFIG_LGD_INCELL_PHASE3_VIDEO_HD_PT_PANEL)
			mdss_lcd_dsv_control(0);
#endif
			return IGNORE_EVENT;
		} else {
			goto error;
		}
	}

	if (ts->pdata->role->use_hover_finger && prox_fhandler.inserted
			&& prox_fhandler.initialized)
		prox_fhandler.exp_fn->attn(ts->ts_data.interrupt_status_reg);

	if (ts->ts_data.interrupt_status_reg & INTERRUPT_MASK_CUSTOM) {
		u8 status = 0;
		u8 buf = 0;

		DO_SAFE(synaptics_ts_page_data_read(client,
					LPWG_PAGE, LPWG_STATUS_REG,
					1, &status), error);
		if ((status & 0x1)) {   /* TCI-1 Double-Tap */
			TOUCH_DEBUG(DEBUG_BASE_INFO || DEBUG_LPWG,
					"LPWG Double-Tap mode\n");
			if (ts->lpwg_ctrl.double_tap_enable) {
				get_tci_data(ts, 2);
				send_uevent_lpwg(ts->client, LPWG_DOUBLE_TAP);
			}
		} else if ((status & 0x2)) { /* TCI-2 Multi-Tap */
			TOUCH_DEBUG(DEBUG_BASE_INFO || DEBUG_LPWG,
					"LPWG Multi-Tap mode\n");
			if (ts->lpwg_ctrl.password_enable) {
				get_tci_data(ts, ts->pw_data.tap_count);
				wake_lock(&ts->timer_wake_lock);
				tci_control(ts, REPORT_MODE_CTRL, 0);
				queue_delayed_work(touch_wq, &ts->work_timer,
						msecs_to_jiffies(UEVENT_DELAY
							- I2C_DELAY));
			}
		} else if (status & 0x04) { /* SWIPE */
			if (ts->ts_swipe_data.support_swipe == SUPPORT_SWIPE) {
				TOUCH_DEBUG(DEBUG_BASE_INFO || DEBUG_LPWG,
				"LPWG SWIPE mode\n");
				if (get_swipe_data(client) == 0) {
					wakeup_by_swipe = true;
					if (send_uevent_lpwg(client, LPWG_SWIPE_DOWN) == 0)
						swipe_disable(ts);
				}
			} else {
				TOUCH_DEBUG(DEBUG_BASE_INFO || DEBUG_LPWG,
					"LPWG SWIPE Not supported");
			}
		} else {
			if (ts->pdata->role->use_debug_reason) {
				TOUCH_DEBUG(DEBUG_BASE_INFO || DEBUG_LPWG,
						"LPWG Debug Detected\n");
				DO_SAFE(synaptics_ts_page_data_read(client,
					LPWG_PAGE, LPWG_DEBUG_REASON_REALTIME,
					1, &buf), error);
				ts->reason[0] = buf & 0x0F;
				ts->reason[1] = (buf & 0xF0) >> 4;
				if (ts->reason) {
					TOUCH_DEBUG(DEBUG_BASE_INFO || DEBUG_LPWG,
						"Debug-Reason TCI1 : [%d], TCI2 : [%d]\n", ts->reason[0], ts->reason[1]);
					synaptics_ts_check_debug_reason(client, MAX_NUM_OF_DEBUG_REASON);
				} else {
					TOUCH_DEBUG(DEBUG_BASE_INFO || DEBUG_LPWG,
						"LPWG Real-Time-Debug-Reason has problem - Unknown debug reason\n");
				}
			} else {
				TOUCH_DEBUG(DEBUG_BASE_INFO || DEBUG_LPWG,
					"LPWG status has problem 0x%02X\n", status);
			}
		}
		return IGNORE_EVENT;
	} else if (ts->ts_data.interrupt_status_reg & INTERRUPT_MASK_ABS0) {
		DO_SAFE(touch_i2c_read(ts->client, OBJECT_ATTENTION_REG,
					(MAX_NUM_OF_FINGERS + 7) / 8,
					object_attn),
					error);
		object_attn_data = (((u16)((object_attn[1] << 8)  & 0xFF00)  | (u16)((object_attn[0]) & 0xFF)));

		for (i = 0; i < MAX_NUM_OF_FINGERS; i++) {
			if (object_attn_data & (0x1 << i)) {
			    curr_object = i + 1;
			}
		}
		if (curr_object > 0) {
			DO_SAFE(touch_i2c_read(ts->client, FINGER_DATA_REG_START,
						(NUM_OF_EACH_FINGER_DATA_REG
						* curr_object),
						ts->ts_data.finger.finger_reg[0]),
					error);

			for (i = 0; i < curr_object; i++) {
				if (ts->ts_data.finger.finger_reg[i][0]
						== F12_FINGER_STATUS
						|| ts->ts_data.finger.finger_reg[i][0]
						== F12_STYLUS_STATUS
						|| ts->ts_data.finger.finger_reg[i][0]
						== F12_PALM_STATUS
						|| ts->ts_data.finger.finger_reg[i][0]
						== F12_GLOVED_FINGER_STATUS) {
					curr_data->abs_data[finger_index].id = i;
					curr_data->abs_data[finger_index].type =
						ts->ts_data.finger.finger_reg[i][REG_OBJECT_TYPE_AND_STATUS];
					curr_data->abs_data[finger_index].x =
						curr_data->abs_data[finger_index].raw_x =
						TS_SNTS_GET_X_POSITION(ts->ts_data.finger.finger_reg[i][REG_X_MSB],
								ts->ts_data.finger.finger_reg[i][REG_X_LSB]);
					curr_data->abs_data[finger_index].y =
						curr_data->abs_data[finger_index].raw_y =
						TS_SNTS_GET_Y_POSITION(ts->ts_data.finger.finger_reg[i][REG_Y_MSB],
								ts->ts_data.finger.finger_reg[i][REG_Y_LSB]);
					curr_data->abs_data[finger_index].width_major =
						TS_SNTS_GET_WIDTH_MAJOR(ts->ts_data.finger.finger_reg[i][REG_WX],
								ts->ts_data.finger.finger_reg[i][REG_WY]);
					curr_data->abs_data[finger_index].width_minor =
						TS_SNTS_GET_WIDTH_MINOR(ts->ts_data.finger.finger_reg[i][REG_WX],
								ts->ts_data.finger.finger_reg[i][REG_WY]);
					curr_data->abs_data[finger_index].orientation =
						TS_SNTS_GET_ORIENTATION(ts->ts_data.finger.finger_reg[i][REG_WY],
								ts->ts_data.finger.finger_reg[i][REG_WX]);
					curr_data->abs_data[finger_index].pressure =
						TS_SNTS_GET_PRESSURE(ts->ts_data.finger.finger_reg[i][REG_Z]);

					if (curr_data->abs_data[finger_index].type
							== F12_PALM_STATUS) {
						switch (ts->pdata->role->palm_ctrl_mode) {
						case PALM_REJECT_DRIVER:
						case PALM_REPORT:
							curr_data->abs_data[finger_index].pressure
								= MAX_PRESSURE;
							ts->ts_palm_data.curr_palm_mask[i] = 1;
							ts->ts_palm_data.palm_coordinate[i].x
								= curr_data->abs_data[finger_index].x;
							ts->ts_palm_data.palm_coordinate[i].y
								= curr_data->abs_data[finger_index].y;
							break;
						case PALM_REJECT_FW:
						default:
							break;
						}
					}

					if (ts->pdata->role->ghost_detection->check_enable.pressure_zero_chk
							&& curr_data->abs_data[finger_index].pressure == 0)
						ts->pdata->role->ghost_detection->pressure_zero = true;

					curr_data->id_mask |= (0x1 << i);
					curr_data->total_num++;

					TOUCH_DEBUG(DEBUG_GET_DATA,
							"<%d> type[%d] pos(%4d,%4d) w_m[%2d] w_n[%2d] o[%2d] p[%2d]\n",
							i, curr_data->abs_data[finger_index].type,
							curr_data->abs_data[finger_index].x,
							curr_data->abs_data[finger_index].y,
							curr_data->abs_data[finger_index].width_major,
							curr_data->abs_data[finger_index].width_minor,
							curr_data->abs_data[finger_index].orientation,
							curr_data->abs_data[finger_index].pressure);

					finger_index++;
				}
			}
		}
		switch (ts->pdata->role->palm_ctrl_mode) {
		case PALM_REJECT_DRIVER:
		case PALM_REPORT:
			for (i = 0; i < MAX_NUM_OF_FINGERS; i++) {
				if (ts->ts_palm_data.curr_palm_mask[i] !=
						ts->ts_palm_data.prev_palm_mask[i]) {
					if (ts->ts_palm_data.curr_palm_mask[i]) {
						ts->ts_palm_data.curr_palm_num++;
						TOUCH_INFO_MSG("Palm is detected : id[%d] "
								"pos[%4d,%4d] total palm:%u\n",
								i, ts->ts_palm_data.palm_coordinate[i].x,
								ts->ts_palm_data.palm_coordinate[i].y,
								ts->ts_palm_data.curr_palm_num);
					} else {
						ts->ts_palm_data.curr_palm_num--;
						TOUCH_INFO_MSG("Palm is released : id[%d] "
								"pos[%4d,%4d] total palm:%u\n",
								i, ts->ts_palm_data.palm_coordinate[i].x,
								ts->ts_palm_data.palm_coordinate[i].y,
								ts->ts_palm_data.curr_palm_num);
					}
				}
			}

			memcpy(ts->ts_palm_data.prev_palm_mask,
					ts->ts_palm_data.curr_palm_mask,
					sizeof(ts->ts_palm_data.prev_palm_mask));

			if (ts->pdata->role->palm_ctrl_mode
					== PALM_REJECT_DRIVER) {
				if (ts->ts_palm_data.curr_palm_num) {
					ts->ts_palm_data.prev_palm_num =
						ts->ts_palm_data.curr_palm_num;
					memset(curr_data, 0,
							sizeof(struct touch_data));
					return NO_FILTER;
				} else {
					if (ts->ts_palm_data.prev_palm_num) {
						ts->ts_palm_data.all_palm_released
							= true;
						queue_delayed_work(touch_wq,
								&ts->work_palm,
								msecs_to_jiffies(50));
						TOUCH_INFO_MSG("All palm is released.\n");
						ts->ts_palm_data.prev_palm_num
							= ts->ts_palm_data.curr_palm_num;
						memset(curr_data, 0, sizeof(struct touch_data));
						return NO_FILTER;
					} else {
						if (ts->ts_palm_data.all_palm_released) {
							ts->ts_palm_data.all_palm_released
								= true;
							cancel_delayed_work(&ts->work_palm);
							queue_delayed_work(touch_wq,
									&ts->work_palm,
									msecs_to_jiffies(50));
							memset(curr_data, 0, sizeof(struct touch_data));
							return NO_FILTER;
						}
					}
				}
			}
			ts->ts_palm_data.prev_palm_num =
				ts->ts_palm_data.curr_palm_num;
			break;
		case PALM_REJECT_FW:
		default:
			break;
		}

		TOUCH_DEBUG(DEBUG_GET_DATA, "ID[0x%x] Total_num[%d]\n",
				curr_data->id_mask, curr_data->total_num);
		if (ts->lpwg_ctrl.password_enable &&
				wake_lock_active(&ts->timer_wake_lock)) {
			if (curr_data->id_mask &
					~(prev_data->id_mask)) {
				/* password-matching will be failed */
				if (cancel_delayed_work(&ts->work_timer)) {
					ts->pw_data.data_num = 1;
					queue_delayed_work(touch_wq,
							&ts->work_timer,
							msecs_to_jiffies(UEVENT_DELAY));
				}
			}
			return IGNORE_EVENT_BUT_SAVE_IT;
		}
		if (ts->lpwg_ctrl.password_enable &&
				atomic_read(&ts->lpwg_ctrl.is_suspend) == 1) {
			TOUCH_INFO_MSG("%s:ignore abs interrupt in suspend\n",
					__func__);
			return IGNORE_EVENT;
		}
	} else if (ts->ts_data.interrupt_status_reg & INTERRUPT_MASK_FLASH) {
		return ERROR;
	} else {
		return IGNORE_EVENT;
	}

	if ((ts->ts_state_flag.ts_noise_log_flag == TS_NOISE_LOG_ENABLE)
			|| (ts->ts_state_flag.check_noise_menu == MENU_ENTER))
		DO_SAFE(synaptics_ts_noise_log(client, curr_data, prev_data),
				error);

	return NO_ERROR;
error:
	TOUCH_ERR_MSG("%s, %d : get data failed. interrupt_status_reg=0x%02X\n", __func__, __LINE__, ts->ts_data.interrupt_status_reg);
	return ERROR;
}

enum error_type synaptics_ts_filter(struct i2c_client *client,
		struct touch_data *curr_data,
		const struct touch_data *prev_data)
{
	/* struct synaptics_ts_data *ts =
	  (struct synaptics_ts_data *)get_touch_handle(client);*/
	int i = 0;

	for (i = 0; i < curr_data->total_num; i++) {
		if (curr_data->abs_data[i].type == HOVER_TYPE)
			curr_data->abs_data[i].pressure = 0;
		else if (curr_data->abs_data[i].type == PALM_TYPE)
			curr_data->abs_data[i].pressure = MAX_PRESSURE;
		else if (curr_data->abs_data[i].pressure == MAX_PRESSURE)
			curr_data->abs_data[i].pressure = MAX_PRESSURE - 1;
	}

	return NO_ERROR;
}
#if defined(CONFIG_TOUCHSCREEN_LGE_SYNAPTICS_TD4191)
enum error_type synaptics_ts_power(struct i2c_client *client, int power_ctrl)
{
	struct synaptics_ts_data *ts
		= (struct synaptics_ts_data *)get_touch_handle(client);

	TOUCH_TRACE();

	if (wakeup_by_swipe) {
		power_state = power_ctrl;
		TOUCH_INFO_MSG("%s : Skip power_control. power_state[%d]\n", __func__, power_state);
		return NO_ERROR;
	}

	switch (power_ctrl) {
	case POWER_OFF:
		if (ts->ts_swipe_data.support_swipe > NO_SUPPORT_SWIPE)
			print_swipe_debug_reason(ts);
		ts->is_init = 0;
		break;
	case POWER_ON:
		break;
	case POWER_SLEEP:
		if (!ts->lpwg_ctrl.lpwg_is_enabled)
			sleep_control(ts, 0, 1);
		break;
	case POWER_WAKE:
		break;
	default:
		break;
	}
	power_state = power_ctrl;

	TOUCH_INFO_MSG("%s : power_state[%d]\n", __func__, power_state);
	return NO_ERROR;
}
#else
enum error_type synaptics_ts_power(struct i2c_client *client, int power_ctrl)
{
	struct synaptics_ts_data *ts
		= (struct synaptics_ts_data *)get_touch_handle(client);

	int i =0;
	int ret = 0;

	TOUCH_TRACE();

	if (wakeup_by_swipe) {
		power_state = power_ctrl;
		TOUCH_INFO_MSG("%s : Skip power_control. power_state[%d]\n", __func__, power_state);
		return NO_ERROR;
	}

	switch (power_ctrl) {
	case POWER_OFF:
		if (ts->ts_swipe_data.support_swipe > NO_SUPPORT_SWIPE)
			print_swipe_debug_reason(ts);

		ts->is_init = 0;

		if ((ts->pdata->int_pin > 0) && (ts->pdata->reset_pin > 0)) {
			gpio_tlmm_config(GPIO_CFG(ts->pdata->int_pin,
				0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN,
				GPIO_CFG_2MA), GPIO_CFG_ENABLE);
			gpio_tlmm_config(GPIO_CFG(ts->pdata->reset_pin,
				0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN,
				GPIO_CFG_6MA), GPIO_CFG_ENABLE);
		}


		i = TOUCH_PWR_NUM-1;
		do {
			if (ts->pdata->pwr[i].type == 1) {
				if (!strncmp(ts->pdata->pwr[i].name, "low", strlen("low"))) {
					gpio_direction_output(ts->pdata->pwr[i].value, 1);
					TOUCH_DEBUG_MSG("power[%d]: gpio[%d] set 1\n", i, ts->pdata->pwr[i].value);
				} else {
					gpio_direction_output(ts->pdata->pwr[i].value, 0);
					TOUCH_DEBUG_MSG("power[%d]: gpio[%d] set 0\n", i, ts->pdata->pwr[i].value);
				}
			} else if (ts->pdata->pwr[i].type == 2) {
				if (ts->vdd_regulator[i] != NULL && !IS_ERR(ts->vdd_regulator[i])) {
					regulator_disable(ts->vdd_regulator[i]);
					TOUCH_DEBUG_MSG("power[%d]: regulator disabled\n", i);
				}
			}
                        mdelay(2);
		} while(--i >= 0);

		break;
	case POWER_ON:
		i = 0;
		do {
			if (ts->pdata->pwr[i].type == 1) {
				if (!strncmp(ts->pdata->pwr[i].name, "low", strlen("low"))) {
					gpio_direction_output(ts->pdata->pwr[i].value, 0);
					TOUCH_DEBUG_MSG("power[%d]: gpio[%d] set 0\n", i, ts->pdata->pwr[i].value);
				} else {
					gpio_direction_output(ts->pdata->pwr[i].value, 1);
					TOUCH_DEBUG_MSG("power[%d]: gpio[%d] set 1\n", i, ts->pdata->pwr[i].value);
				}
			} else if (ts->pdata->pwr[i].type == 2) {
				if (ts->vdd_regulator[i] != NULL && !IS_ERR(ts->vdd_regulator[i])) {
					ret = regulator_enable(ts->vdd_regulator[i]);
					if (ret) {
						TOUCH_INFO_MSG("power[%d]: regulator enable failed ret =%d\n", i, ret );
					} else {
						TOUCH_DEBUG_MSG("power[%d]: regulator enabled\n", i);
					}
				}
			}
			mdelay(2);
		} while(++i < TOUCH_PWR_NUM);

		gpio_tlmm_config(GPIO_CFG(ts->pdata->reset_pin, 0,
			GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_6MA),
			GPIO_CFG_ENABLE);
		gpio_direction_output(ts->pdata->reset_pin, 1);
		break;
	case POWER_SLEEP:
		if ((ts->pdata->reset_pin > 0))  {
			gpio_tlmm_config(GPIO_CFG(ts->pdata->reset_pin, 0,
						GPIO_CFG_INPUT,
						GPIO_CFG_PULL_DOWN,
						GPIO_CFG_6MA), GPIO_CFG_ENABLE);
			gpio_direction_input(ts->pdata->reset_pin);
		}
		if (!ts->lpwg_ctrl.lpwg_is_enabled)
			sleep_control(ts, 0, 1);
		break;
	case POWER_WAKE:
		gpio_tlmm_config(GPIO_CFG(ts->pdata->reset_pin, 0,
					GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP,
					GPIO_CFG_6MA), GPIO_CFG_ENABLE);
		gpio_direction_output(ts->pdata->reset_pin, 1);
		break;
	default:
		break;
	}
	power_state = power_ctrl;

	TOUCH_INFO_MSG("%s : power_state[%d]\n", __func__, power_state);
	return NO_ERROR;
}
#endif

enum error_type synaptics_ts_reset(struct i2c_client *client, int reset_ctrl)
{
	struct synaptics_ts_data *ts
			= (struct synaptics_ts_data *)get_touch_handle(client);

	TOUCH_TRACE();

	switch (reset_ctrl) {
		case SOFT_RESET:
			TOUCH_INFO_MSG("SOFT_RESET start\n");
                        DO_SAFE(touch_i2c_write_byte(ts->client, DEVICE_COMMAND_REG, 0x01),
					error);
			msleep(ts->pdata->role->booting_delay);
			TOUCH_INFO_MSG("SOFT_RESET end\n");
			break;
		case PIN_RESET:
			TOUCH_INFO_MSG("PIN_RESET start\n");
			gpio_direction_output(ts->pdata->reset_pin, 0);
			msleep(ts->pdata->role->reset_delay);
			gpio_direction_output(ts->pdata->reset_pin, 1);
			msleep(ts->pdata->role->booting_delay);
			break;
		default:
			break;
	}
	return NO_ERROR;
error:
	TOUCH_ERR_MSG("error\n");
	msleep(ts->pdata->role->reset_delay);
	return ERROR;
}

enum error_type synaptics_ts_ic_ctrl(struct i2c_client *client,
		u8 code, u32 value, u32 *ret)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);
	u8 buf = 0;
	u8 buf_array[2] = {0};

	u8 curr = 0;
	u8 next = 0;

	switch (code) {
	case IC_CTRL_READ:
		DO_SAFE(touch_i2c_read(client, value, 1, &buf), error);
		*ret = (u32)buf;
		break;
	case IC_CTRL_WRITE:
		DO_SAFE(touch_i2c_write_byte(client, ((value & 0xFFF0) >> 8),
					(value & 0xFF)), error);
		break;
	case IC_CTRL_BASELINE_REBASE:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
					ANALOG_PAGE, ANALOG_COMMAND_REG, value),
				error);
		break;
	case IC_CTRL_REPORT_MODE:
		if (value == REDUCED_REPORT_MODE)
			buf_array[0] = buf_array[1] =
				ts->pdata->role->delta_pos_threshold;
		DO_SAFE(touch_i2c_write(client, f12_info.ctrl_reg_addr[20],
					2, buf_array), error);
		break;
	case IC_CTRL_USB_TA_DETECT:
		DO_SAFE(touch_i2c_read(ts->client, DEVICE_CONTROL_REG, 1, &curr), error);
		next = (curr & 0xDF) | (value ? 0x20:0x00);
		TOUCH_DEBUG(DEBUG_BASE_INFO, "%s : curr DEVICE_CONTROL_REG[%d], next DEVICE_CONTROL_REG[%d]\n", __func__, curr, next);
		DO_SAFE(touch_i2c_write_byte(ts->client, DEVICE_CONTROL_REG, next),error);
		break;
	case IC_CTRL_THERMAL:
		switch (value) {
		case THERMAL_LOW:
			buf = 0x00;
			buf_array[0] = default_finger_amplitude[0];
			buf_array[1] = default_finger_amplitude[1];
			break;
		case THERMAL_HIGH:
			buf = 0x04;
			buf_array[0] = buf_array[1] =
				THERMAL_HIGH_FINGER_AMPLITUDE;
			break;
		default:
			TOUCH_ERR_MSG("Invalid current_thermal_mode (%u)\n",
					value);
			goto error;
		}
		TOUCH_INFO_MSG("High Temp Control(0x%02X), Finger Amplitude Threshold(0x%02X), Small Finger Amplitude Threshold(0x%02X)\n",
				buf, buf_array[0], buf_array[1]);
		//TD4191 not support
		//DO_SAFE(synaptics_ts_page_data_write_byte(client, LPWG_PAGE,
		//			MISC_HOST_CONTROL_REG, buf), error);
		DO_SAFE(touch_i2c_write(client, f12_info.ctrl_reg_addr[15], 2,
					buf_array), error);
		break;
	default:
		break;
	}

	return NO_ERROR;
error:
	TOUCH_ERR_MSG("%s, %d : IC control failed\n", __func__, __LINE__);
	return ERROR;
}

int compare_fw_version(struct i2c_client *client,
		struct touch_fw_info *fw_info) {
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);
	int i = 0;

	TOUCH_DEBUG(DEBUG_BASE_INFO,
			"product_id[%s(ic):%s(fw)] ",
			ts->fw_info.fw_product_id,
			ts->fw_info.fw_image_product_id);
	TOUCH_DEBUG(DEBUG_BASE_INFO,
			"ic_fw_version[V%d.%02d(0x%02X 0x%02X 0x%02X 0x%02X)]\n",
			(ts->fw_info.fw_version[3]
			 & 0x80 ? 1 : 0),
			ts->fw_info.fw_version[3] & 0x7F,
			ts->fw_info.fw_version[0],
			ts->fw_info.fw_version[1],
			ts->fw_info.fw_version[2],
			ts->fw_info.fw_version[3]);
	TOUCH_DEBUG(DEBUG_BASE_INFO,
			"fw_version[V%d.%02d(0x%02X 0x%02X 0x%02X 0x%02X)]\n",
			(ts->fw_info.fw_image_version[3]
			 & 0x80 ? 1 : 0),
			ts->fw_info.fw_image_version[3] & 0x7F,
			ts->fw_info.fw_image_version[0],
			ts->fw_info.fw_image_version[1],
			ts->fw_info.fw_image_version[2],
			ts->fw_info.fw_image_version[3]);
	for (i = 0; i < FW_VER_INFO_NUM; i++) {
		if (ts->fw_info.fw_version[i] !=
				ts->fw_info.fw_image_version[i]) {
			TOUCH_DEBUG(DEBUG_BASE_INFO,
				"fw_version mismatch. ic_fw_version[%d]:0x%02X != fw_version[%d]:0x%02X\n",
				i,
				ts->fw_info.fw_version[i],
				i,
				ts->fw_info.fw_image_version[i]);
			return 1;
		}
	}
	TOUCH_DEBUG(DEBUG_BASE_INFO | DEBUG_FW_UPGRADE, "fw version match.\n");
	return 0;
}
enum error_type synaptics_ts_fw_upgrade(struct i2c_client *client,
		struct touch_fw_info *info, struct touch_firmware_module *fw)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);
	int need_upgrade = 0;
	int rc = 0;
	char path[256];

	if (info->fw_force_upgrade) {
		memcpy(path, info->fw_path, sizeof(path));
		TOUCH_DEBUG(DEBUG_BASE_INFO | DEBUG_FW_UPGRADE,
				"FW: need_upgrade[%d] force[%d] file[%s]\n",
				fw->need_upgrade, info->fw_force_upgrade, path);
		goto firmware;
	}

	if (info->fw_force_upgrade_cat) {
		if (ts->fw_flag == TD4191) {
			memcpy(path, info->fw_path, sizeof(path));
			TOUCH_DEBUG(DEBUG_BASE_INFO | DEBUG_FW_UPGRADE,
					"FW: need_upgrade[%d] force_cat[%d] "
					"file[%s]\n",
					fw->need_upgrade,
					info->fw_force_upgrade_cat, path);
			goto firmware;
		} else if (ts->fw_flag == TD4191_TFT_ITO_PANEL) {
			memcpy(path, info->fw_path, sizeof(path));
			TOUCH_DEBUG(DEBUG_BASE_INFO | DEBUG_FW_UPGRADE,
					"FW: need_upgrade[%d] force_cat[%d] "
					"file[%s]\n",
					fw->need_upgrade,
					info->fw_force_upgrade_cat, path);
			goto firmware;
		} else if (ts->fw_flag == S3528_A0) {
			memcpy(path, info->fw_path, sizeof(path));
			TOUCH_DEBUG(DEBUG_BASE_INFO | DEBUG_FW_UPGRADE,
					"FW: need_upgrade[%d] force_cat[%d] "
					"file[%s]\n",
					fw->need_upgrade,
					info->fw_force_upgrade_cat, path);
			goto firmware;
		} else if (ts->fw_flag == S3528_A1) {
			memcpy(path, info->fw_path_s3528_a1, sizeof(path));
			TOUCH_DEBUG(DEBUG_BASE_INFO | DEBUG_FW_UPGRADE,
					"FW: need_upgrade[%d] force_cat[%d] "
					"file[%s]\n",
					fw->need_upgrade,
					info->fw_force_upgrade_cat, path);
			goto firmware;
		} else if (ts->fw_flag == S3528_A1_SUN) {
			memcpy(path, info->fw_path_s3528_a1_suntel,
					sizeof(path));
			TOUCH_INFO_MSG("FW: need_upgrade[%d] force_cat[%d] "
					"file[%s]\n",
					fw->need_upgrade,
					info->fw_force_upgrade_cat, path);
			goto firmware;
		} else {
			memcpy(path, info->fw_path_s3621, sizeof(path));
			TOUCH_DEBUG(DEBUG_BASE_INFO | DEBUG_FW_UPGRADE,
					"FW: need_upgrade[%d] force_cat[%d] "
					"file[%s]\n",
					fw->need_upgrade,
					info->fw_force_upgrade_cat, path);
			goto firmware;
		}
	}

	need_upgrade = !strncmp(ts->fw_info.fw_product_id,
			ts->fw_info.fw_image_product_id,
			sizeof(ts->fw_info.fw_product_id));

	if (need_upgrade) {
		rc = compare_fw_version(client, info);
		need_upgrade = need_upgrade && rc;

		if (need_upgrade || ts->fw_info.need_rewrite_firmware) {
			TOUCH_DEBUG(DEBUG_BASE_INFO | DEBUG_FW_UPGRADE,
					"FW: start-upgrade - need[%d] rewrite[%d]\n",
					need_upgrade,
					ts->fw_info.need_rewrite_firmware);

			if (ts->fw_flag == TD4191) {
				memcpy(path, info->fw_path, sizeof(path));
				TOUCH_DEBUG(DEBUG_BASE_INFO | DEBUG_FW_UPGRADE,
						"FW: need_upgrade[%d] force[%d] "
						"file[%s]\n",
						fw->need_upgrade,
						info->fw_force_upgrade, path);
				goto firmware;
			} else if (ts->fw_flag == TD4191_TFT_ITO_PANEL) {
				memcpy(path, info->fw_path, sizeof(path));
				TOUCH_DEBUG(DEBUG_BASE_INFO | DEBUG_FW_UPGRADE,
						"FW: need_upgrade[%d] force[%d] "
						"file[%s]\n",
						fw->need_upgrade,
						info->fw_force_upgrade, path);
				goto firmware;
			} else if (ts->fw_flag == S3528_A0) {
				memcpy(path, info->fw_path, sizeof(path));
				TOUCH_DEBUG(DEBUG_BASE_INFO | DEBUG_FW_UPGRADE,
						"FW: need_upgrade[%d] force[%d] "
						"file[%s]\n",
						fw->need_upgrade,
						info->fw_force_upgrade, path);
				goto firmware;
			} else if (ts->fw_flag == S3528_A1) {
				memcpy(path, info->fw_path_s3528_a1, sizeof(path));
				TOUCH_DEBUG(DEBUG_BASE_INFO | DEBUG_FW_UPGRADE,
						"FW: need_upgrade[%d] force[%d] "
						"file[%s]\n",
						fw->need_upgrade,
						info->fw_force_upgrade, path);
				goto firmware;
			} else if (ts->fw_flag == S3528_A1_SUN) {
				memcpy(path, info->fw_path_s3528_a1_suntel,
						sizeof(path));
				TOUCH_INFO_MSG("FW: need_upgrade[%d] force[%d] "
						"file[%s]\n",
						fw->need_upgrade,
						info->fw_force_upgrade, path);
				goto firmware;
			} else {
				memcpy(path, info->fw_path_s3621, sizeof(path));
				TOUCH_DEBUG(DEBUG_BASE_INFO | DEBUG_FW_UPGRADE,
						"FW: need_upgrade[%d] force[%d] "
						"file[%s]\n",
						fw->need_upgrade,
						info->fw_force_upgrade, path);
				goto firmware;
			}
			/* it will be reset and initialized
			 * automatically by lge_touch_core. */
		} else {
			TOUCH_DEBUG(DEBUG_BASE_INFO, "Do not upgrade firmware\n");
		}
	} else {
			TOUCH_DEBUG(DEBUG_BASE_INFO,
					"product_id mismatch. [%s(ic):%s(fw)] \n",
					ts->fw_info.fw_product_id,
					ts->fw_info.fw_image_product_id);
			TOUCH_DEBUG(DEBUG_BASE_INFO, "Do not upgrade firmware\n");
	}
	return NO_UPGRADE;

firmware:
	ts->fw_flag = 0;
	ts->is_probed = 0;
	ts->is_init = 0; /* During upgrading, interrupt will be ignored. */
	info->fw_force_upgrade = 0;
	info->fw_force_upgrade_cat = 0;
	need_scan_pdt = true;
	DO_SAFE(FirmwareUpgrade(ts, path), error);
	return NO_ERROR;
error:
	return ERROR;
}

enum error_type synaptics_ts_fw_recovery(struct i2c_client *client,
		struct touch_fw_info *info)
{
	struct synaptics_ts_data *ts =
			(struct synaptics_ts_data *)get_touch_handle(client);
	char path[256];

	if (info->fw_force_upgrade) {
		memcpy(path, info->fw_path, sizeof(path));
		TOUCH_INFO_MSG("FW_ RECOVERY:  force[%d] file[%s]\n",
					info->fw_force_upgrade, path);
		goto firmware;
	}

	if (SynaScanPDT(ts) == ERROR)
		goto error;

	if (ts->ubootloader_mode) {
		memcpy(path, info->fw_path, sizeof(path));
		TOUCH_INFO_MSG("FW_ RECOVERY:  ubootloader_mode[%d] file[%s]\n",
					ts->ubootloader_mode, path);
		goto firmware;
	}

	TOUCH_INFO_MSG("Don't need to Recovery Firmware.\n");
	return NO_UPGRADE;
firmware:
	ts->is_init = 0;
	info->fw_force_upgrade = 0;
	DO_SAFE(FirmwareRecovery(ts, path),error);
	ts->pdata->role->i2c_addr = client->addr;
	return NO_ERROR;
error:
	TOUCH_ERR_MSG("Fail to Recovery Firmware.\n");
	return ERROR;
}

enum error_type synaptics_ts_notify(struct i2c_client *client,
		u8 code, u32 value)
{
	/* struct synaptics_ts_data *ts =
	 * (struct synaptics_ts_data *)get_touch_handle(client); */

	switch (code) {
	case NOTIFY_TA_CONNECTION:
		break;
	case NOTIFY_TEMPERATURE_CHANGE:
		break;
	case NOTIFY_PROXIMITY:
		break;
	default:
		break;
	}

	return NO_ERROR;
}

enum error_type synaptics_ts_suspend(struct i2c_client *client)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);

	if (ts->ts_swipe_data.support_swipe > NO_SUPPORT_SWIPE)
		wakeup_by_swipe = false;

	if (ts->pdata->role->use_hover_finger && prox_fhandler.inserted
			&& prox_fhandler.initialized)
		prox_fhandler.exp_fn->suspend(ts);

	/*because s3621 doesn't support knock-on*/
	if (!(strncmp(ts->fw_info.fw_product_id, "PLG298", 6)))
		return NO_ERROR;

	if (!atomic_read(&ts->lpwg_ctrl.is_suspend)) {
		DO_SAFE(lpwg_control(ts, ts->lpwg_ctrl.lpwg_mode), error);
		atomic_set(&ts->lpwg_ctrl.is_suspend, 1);
	}
	ts->lpwg_ctrl.screen = 0;

	return NO_ERROR;
error:
	return ERROR;
}

enum error_type synaptics_ts_resume(struct i2c_client *client)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);

	if (ts->pdata->role->use_hover_finger && prox_fhandler.inserted
			&& prox_fhandler.initialized)
		prox_fhandler.exp_fn->resume(ts);

	cancel_delayed_work_sync(&ts->work_timer);

	if (wake_lock_active(&ts->timer_wake_lock))
		wake_unlock(&ts->timer_wake_lock);
#if defined(CONFIG_TOUCHSCREEN_LGE_SYNAPTICS_TD4191)
	//Do nothing
#else
	if (atomic_read(&ts->state->upgrade_state) != UPGRADE_START) {
		if (ts->lpwg_ctrl.has_debug_module) {
			DO_SAFE(print_tci_debug_result(ts, 0), error);
			DO_SAFE(print_tci_debug_result(ts, 1), error);
		}
	}
#endif
	atomic_set(&ts->lpwg_ctrl.is_suspend, 0);
	ts->lpwg_ctrl.screen = 1;

	return NO_ERROR;

#if defined(CONFIG_TOUCHSCREEN_LGE_SYNAPTICS_TD4191)
	//Do nothing
#else
error:
	return ERROR;
#endif
}
enum error_type synaptics_ts_lpwg(struct i2c_client *client,
		u32 code, u32 value, struct point *data)
{
	int i;
	u8 buffer[50] = {0};
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);
	u8 mode = ts->lpwg_ctrl.lpwg_mode;
	u8 doubleTap_area_reg_addr = f12_info.ctrl_reg_addr[18];

	/*because s3621 doesn't support knock-on*/
	if (!(strncmp(ts->fw_info.fw_product_id, "PLG298", 6)))
		return NO_ERROR;

	switch (code) {
	case LPWG_READ:
		memcpy(data, ts->pw_data.data,
				sizeof(struct point) * ts->pw_data.data_num);
		data[ts->pw_data.data_num].x = -1;
		data[ts->pw_data.data_num].y = -1;
		/* '-1' should be assigned to the last data.
		 Each data should be converted to LCD-resolution.*/
		memset(ts->pw_data.data, -1,
				sizeof(struct point)*ts->pw_data.data_num);
		break;
	case LPWG_ENABLE:
		if (!atomic_read(&ts->lpwg_ctrl.is_suspend))
			ts->lpwg_ctrl.lpwg_mode = value;
		break;
	case LPWG_LCD_X:
	case LPWG_LCD_Y:
		/* If touch-resolution is not same with LCD-resolution,
		 position-data should be converted to LCD-resolution.*/
		break;
	case LPWG_ACTIVE_AREA_X1:
		for (i = 0; i < 2; i++) {
			synaptics_ts_page_data_read(client,
					COMMON_PAGE, doubleTap_area_reg_addr,
					i + 1, buffer);
			if (i == 0)
				buffer[i] = value;
			else
				buffer[i] = value >> 8;
			synaptics_ts_page_data_write(client,
					COMMON_PAGE, doubleTap_area_reg_addr,
					i + 1, buffer);
		}
		break;
	case LPWG_ACTIVE_AREA_X2:
		for (i = 4; i < 6; i++) {
			synaptics_ts_page_data_read(client,
					COMMON_PAGE, doubleTap_area_reg_addr,
					i + 1, buffer);
			if (i == 4)
				buffer[i] = value;
			else
				buffer[i] = value >> 8;
			synaptics_ts_page_data_write(client,
					COMMON_PAGE, doubleTap_area_reg_addr,
					i + 1, buffer);
		}
		break;
	case LPWG_ACTIVE_AREA_Y1:
		for (i = 2; i < 4; i++) {
			synaptics_ts_page_data_read(client,
					COMMON_PAGE, doubleTap_area_reg_addr,
					i + 1, buffer);
			if (i == 2)
				buffer[i] = value;
			else
				buffer[i] = value >> 8;
			synaptics_ts_page_data_write(client,
					COMMON_PAGE, doubleTap_area_reg_addr,
					i + 1, buffer);
		}
		break;
	case LPWG_ACTIVE_AREA_Y2:
		/* Quick Cover Area*/
		for (i = 6; i < 8; i++) {
			synaptics_ts_page_data_read(client,
					COMMON_PAGE, doubleTap_area_reg_addr,
					i + 1, buffer);
			if (i == 6)
				buffer[i] = value;
			else
				buffer[i] = value >> 8;
			synaptics_ts_page_data_write(client,
					COMMON_PAGE, doubleTap_area_reg_addr,
					i + 1, buffer);
		}
		sleep_control(ts, 1, 0);
		break;
	case LPWG_TAP_COUNT:
		ts->pw_data.tap_count = value;
		if (ts->lpwg_ctrl.password_enable)
			tci_control(ts, TAP_COUNT_CTRL,
					(u8)ts->pw_data.tap_count);
		break;
	case LPWG_LENGTH_BETWEEN_TAP:
		if (ts->lpwg_ctrl.double_tap_enable
				|| ts->lpwg_ctrl.password_enable)
			tci_control(ts, TAP_DISTANCE_CTRL, value);
		break;
	case LPWG_EARLY_SUSPEND:
		if (mode) { /* wakeup gesture enable */
			if (value) {
				if (atomic_read(&ts->lpwg_ctrl.is_suspend) == 1
						&& (power_state == POWER_OFF
							|| power_state
							== POWER_SLEEP))
					ts->is_init = 0;
				DO_SAFE(lpwg_control(ts, 0), error);
				atomic_set(&ts->lpwg_ctrl.is_suspend, 0);
			} else {
				set_doze_param(ts, 3);
				DO_SAFE(lpwg_control(ts,
						ts->lpwg_ctrl.lpwg_mode),
						error);
				atomic_set(&ts->lpwg_ctrl.is_suspend, 1);
			}
		} else { /* wakeup gesture disable */
			if (value) /* lcd on */
				interrupt_control(ts->client,INTERRUPT_ENABLE);
			else
				interrupt_control(ts->client,INTERRUPT_DISABLE);
		}
		break;
	case LPWG_SENSOR_STATUS:
		if (mode) {
			if (value)  /* Far */
				DO_SAFE(lpwg_control(ts, mode), error);
			else { /* Near */
				if (ts->lpwg_ctrl.password_enable &&
					wake_lock_active(&ts->timer_wake_lock)) {
					cancel_delayed_work_sync(&ts->work_timer);
					tci_control(ts, REPORT_MODE_CTRL, 1);
					wake_unlock(&ts->timer_wake_lock);
				}
			}
		}
		break;
	case LPWG_DOUBLE_TAP_CHECK:
		ts->pw_data.double_tap_check = value;
		if (ts->lpwg_ctrl.password_enable)
			tci_control(ts, INTERRUPT_DELAY_CTRL, value);
		break;
	case LPWG_REPLY:
		if (ts->pdata->role->use_lpwg_all) {
			if (atomic_read(&ts->lpwg_ctrl.is_suspend) == 0) {
				TOUCH_INFO_MSG("%s : screen on\n", __func__);
				break;
			}
			if (wakeup_by_swipe) {
				if (!value)
					wakeup_by_swipe = false;
				TOUCH_INFO_MSG("%s : skip lpwg setting in case of wakeup_by_swipe[%d]\n", __func__, wakeup_by_swipe);
				break;
			}
			DO_SAFE(lpwg_update_all(ts, 1), error);
		} else {
			if (ts->lpwg_ctrl.password_enable && !value)
				DO_SAFE(lpwg_control(ts, mode), error);
		}
		break;
	case LPWG_UPDATE_ALL:
	{
		int *v = (int *) value;
		int mode = *(v + 0);
		int screen = *(v + 1);
		int sensor = *(v + 2);
		int qcover = *(v + 3);

		ts->lpwg_ctrl.lpwg_mode = mode;
		ts->lpwg_ctrl.screen = screen;
		ts->lpwg_ctrl.sensor = sensor;
		ts->lpwg_ctrl.qcover = qcover;

		TOUCH_INFO_MSG("LPWG_UPDATE_ALL: mode[%s], screen[%s], "
				"sensor[%s], qcover[%s]\n",
				ts->lpwg_ctrl.lpwg_mode? "ENABLE" : "DISABLE",
				ts->lpwg_ctrl.screen? "ON" : "OFF",
				ts->lpwg_ctrl.sensor? "FAR" : "NEAR",
				ts->lpwg_ctrl.qcover? "CLOSE" : "OPEN");
		DO_SAFE(lpwg_update_all(ts, 1), error);
		break;
	}
	case LPWG_DEBUG_CTRL:
		DO_SAFE(lpwg_debug_interrupt_control(ts, value), error);
		TOUCH_DEBUG(DEBUG_BASE_INFO || DEBUG_LPWG,
				"ts->pdata->role->use_debug_reason : [%s]\n",
				ts->pdata->role->use_debug_reason ? "Enabled" : "Disabled");
		break;
	default:
		break;
	}
	return NO_ERROR;
error:
	return ERROR;
}

static void synapitcs_change_ime_status(struct i2c_client *client,
		int ime_status)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);

	u8 udata[2] = {0, };
	u8 drumming_address = f12_info.ctrl_reg_addr[10];

	TOUCH_INFO_MSG("%s : IME STATUS is [ %d ]!!!\n", __func__, ime_status);

	touch_i2c_read(ts->client, drumming_address, 2, udata);

	if (ime_status) {
		TOUCH_INFO_MSG("%s : IME on !!\n", __func__);
		udata[0] = 0x06;/*Drumming Acceleration Threshold*/
		udata[1] = 0x0a;/*Minimum Drumming Separation*/
		if (touch_i2c_write(ts->client,
					drumming_address, 2, udata) < 0) {
			TOUCH_ERR_MSG("%s : Touch i2c write fail !!\n",
					__func__);
		}
	} else {
		udata[0] = 0x0a; /*Drumming Acceleration Threshold*/
		udata[1] = 0x0a; /*Minimum Drumming Separation*/
		if (touch_i2c_write(ts->client,
					drumming_address, 2, udata) < 0) {
			TOUCH_ERR_MSG("%s : Touch i2c write fail !!\n",
					__func__);
		}
		TOUCH_INFO_MSG("%s : IME Off\n", __func__);
	}
	TOUCH_INFO_MSG("%s : Done !!\n", __func__);
	return;
}

static int set_doze_param(struct synaptics_ts_data *ts, int value)
{
	u8 buf_array[6] = {0};

	touch_i2c_read(ts->client,
			f12_info.ctrl_reg_addr[27], 6, buf_array);
#if defined(CONFIG_TOUCHSCREEN_LGE_SYNAPTICS_TD4191)
	buf_array[2] = 0x0C;  /* False Activation Threshold */
	buf_array[3] = 0x0B;  /* Max Active Duration */
	buf_array[5] = 0x01;  /* Max Active Duration Timeout */

	DO_SAFE(touch_i2c_write(ts->client, f12_info.ctrl_reg_addr[27],
			6, buf_array), error);
	return 0;
error:
	TOUCH_ERR_MSG("%s : failed to set doze interval\n", __func__);
	return -EPERM;
#else
	/* max active duration */
	if (ts->pw_data.tap_count < 3)
		buf_array[3] = 3;
	else
		buf_array[3] = 3 + ts->pw_data.tap_count;

	buf_array[2] = 0x0C;  /* False Activation Threshold */
	buf_array[4] = 0x01;  /* Timer 1 */
	buf_array[5] = 0x01;  /* Max Active Duration Timeout */

	touch_i2c_write(ts->client, f12_info.ctrl_reg_addr[27],
			6, buf_array);
	DO_SAFE(touch_i2c_write_byte(ts->client,
				DOZE_INTERVAL_REG, 3), error);
	DO_SAFE(touch_i2c_write_byte(ts->client,
				DOZE_WAKEUP_THRESHOLD_REG, 30), error);
	return 0;
error:
	TOUCH_ERR_MSG("%s : failed to set doze interval\n", __func__);
	return -EPERM;
#endif
}

enum window_status synapitcs_check_crack(struct i2c_client *client)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);
	char result[2] = {0x00, };

	if (need_scan_pdt) {
		SCAN_PDT();
		need_scan_pdt = false;
	}

	touch_disable_irq(ts->client->irq);
	F54Test('l', (int)ts->pdata->role->crack_detection->min_cap_value,
			result);
	touch_enable_irq(ts->client->irq);

	TOUCH_INFO_MSG("%s : check window crack = %s\n",
			__func__, result);

	after_crack_check = 1; /* set crack check flag */

	if (strncmp(result, "1", 1) == 0)
		return CRACK;
	else
		return NO_CRACK;
}

struct touch_device_driver synaptics_ts_driver = {
	.probe		= synaptics_ts_probe,
	.remove		= synaptics_ts_remove,
	.suspend	= synaptics_ts_suspend,
	.resume		= synaptics_ts_resume,
	.init		= synaptics_ts_init,
	.data		= synaptics_ts_get_data,
	.filter		= synaptics_ts_filter,
	.power		= synaptics_ts_power,
	.ic_ctrl	= synaptics_ts_ic_ctrl,
	.fw_upgrade	= synaptics_ts_fw_upgrade,
	.fw_recovery = synaptics_ts_fw_recovery,
	.notify		= synaptics_ts_notify,
	.lpwg		= synaptics_ts_lpwg,
	.ime_drumming = synapitcs_change_ime_status,
	.inspection_crack = synapitcs_check_crack,
	.reset		= synaptics_ts_reset,
};

static struct of_device_id match_table[] = {
	{ .compatible = "synaptics,td4191",},
	{ },
};
static void async_touch_init(void *data, async_cookie_t cookie)
{
	touch_driver_register(&synaptics_ts_driver, match_table);
	return;
}


static int touch_init(void)
{
	TOUCH_TRACE();
	async_schedule(async_touch_init, NULL);

	return 0;
}

static void touch_exit(void)
{
	TOUCH_TRACE();
	touch_driver_unregister();
}

module_init(touch_init);
module_exit(touch_exit);

MODULE_AUTHOR("yehan.ahn@lge.com, hyesung.shin@lge.com");
MODULE_DESCRIPTION("LGE Touch Driver");
MODULE_LICENSE("GPL");

