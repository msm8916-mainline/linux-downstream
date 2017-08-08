/* lge_touch_melfas.h
 *
 * Copyright (C) 2013 LGE.
 *
 * Author: WX-BSP-TS@lge.com
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

#include <linux/types.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/gpio.h>
#include <linux/firmware.h>
#include <linux/syscalls.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <asm/unaligned.h>
//#include <soc/qcom/gpiomux.h>
#include <linux/uaccess.h>
#include <linux/time.h>
#include <linux/file.h>
#include <linux/syscalls.h>
#include <linux/async.h>

#include "lge_ts_core.h"

#ifndef LGE_TS_MELFAS_H
#define LGE_TS_MELFAS_H

#ifndef MIP_USE_DEV
#define MIP_USE_DEV 1
#endif

#define FW_BLOCK_SIZE		128
#define FW_MAX_SIZE		(64 * 1024)

#define FINGER_EVENT_SZ		6
#define LPWG_EVENT_SZ		3
#define MAX_PRESSURE		255

#define SECTION_NUM		3
#define PAGE_HEADER		3
#define PAGE_DATA		1024
#define PAGE_CRC		2
#define PACKET_SIZE		(PAGE_HEADER + PAGE_DATA + PAGE_CRC)

#define KNOCKON_DELAY   	700

#define MAX_COL			18
#define MAX_ROW			32

#define MIP_R0_INFO				0x01
#define MIP_R1_VENDOR_INFO			0x00
#define MIP_R1_INFO_VERSION_CUSTOM		0x24
#define MIP_R1_INFO_PRODUCT_NAME		0x00
#define MIP_R1_INFO_IC_NAME			0x71
#define MIP_R1_INFO_RESOLUTION_X		0x10
#define MIP_R1_INFO_RESOLUTION_Y		0x12
#define MIP_R1_INFO_NODE_NUM_X			0x14
#define MIP_R1_INFO_NODE_NUM_Y			0x15
#define MIP_R1_INFO_KEY_NUM			0x16

#define MIP_R0_EVENT				0x02
#define MIP_R1_EVENT_SUPPORTED_FUNC		0x00
#define MIP_R1_EVENT_PACKET_INFO		0x10
#define MIP_R1_EVENT_PACKET_DATA		0x11

#define MIP_R0_CTRL				0x06
#define MIP_R1_CTRL_READY_STATUS		0x00
#define MIP_R1_CTRL_EVENT_READY			0x01
#define MIP_R1_CTRL_MODE			0x10
#define MIP_R1_CTRL_INTERRUPT			0x11

#define MIP_R0_TEST				0x0A
#define MIP_R1_TEST_BUF_ADDR			0x00
#define MIP_R1_TEST_PROTOCOL			0x02
#define MIP_R1_TEST_TYPE			0x10
#define MIP_R1_TEST_DATA_FORMAT			0x20
#define MIP_R1_TEST_ROW_NUM			0x20
#define MIP_R1_TEST_COL_NUM			0x21
#define MIP_R1_TEST_BUFFER_COL_NUM		0x22
#define MIP_R1_TEST_COL_AXIS			0x23
#define MIP_R1_TEST_KEY_NUM			0x24
#define MIP_R1_TEST_DATA_TYPE			0x25

#define MIP_R0_IMAGE				0x0C
#define MIP_R1_IMAGE_BUF_ADDR			0x00
#define MIP_R1_IMAGE_PROTOCOL_ID		0x04
#define MIP_R1_IMAGE_TYPE			0x10
#define MIP_R1_IMAGE_DATA_FORMAT		0x20
#define MIP_R1_IMAGE_ROW_NUM			0x20
#define MIP_R1_IMAGE_COL_NUM			0x21
#define MIP_R1_IMAGE_BUFFER_COL_NUM		0x22
#define MIP_R1_IMAGE_COL_AXIS			0x23
#define MIP_R1_IMAGE_KEY_NUM			0x24
#define MIP_R1_IMAGE_DATA_TYPE			0x25
#define MIP_R1_IMAGE_FINGER_NUM			0x30
#define MIP_R1_IMAGE_FINGER_AREA		0x31

#define MIP_R0_LOG				0x10
#define MIP_R1_LOG_TRIGGER			0x14

#define MIP_EVENT_INPUT_PRESS			0x80
#define MIP_EVENT_INPUT_SCREEN			0x40
#define MIP_EVENT_INPUT_HOVER			0x20
#define MIP_EVENT_INPUT_PALM			0x10
#define MIP_EVENT_INPUT_ID			0x0F

#define MIP_EVENT_GESTURE_DOUBLE_TAP		24
#define MIP_EVENT_GESTURE_MULTI_TAP		25
#define MIP_EVENT_GESTURE_SWIPE		26
#define MIP_EVENT_GESTURE_ALL			0xFFFFFFFF

#define MIP_ALERT_ESD				1
#define MIP_ALERT_WAKEUP			2
#define MIP_ALERT_F1				0xF1

#define MIP_CTRL_STATUS_NONE			0x05
#define MIP_CTRL_STATUS_READY			0xA0
#define MIP_CTRL_STATUS_LOG			0x77

#define MIP_CTRL_MODE_NORMAL			0
#define MIP_CTRL_MODE_PARAM			1
#define MIP_CTRL_MODE_TEST_CM			2

#define MIP_CTRL_POWER_ACTIVE			0
#define MIP_CTRL_POWER_LOW			1

#define MIP_TEST_TYPE_NONE			0
#define MIP_TEST_TYPE_CM_DELTA			1
#define MIP_TEST_TYPE_CM_ABS			2
#define MIP_TEST_TYPE_CM_JITTER			3
#define MIP_TEST_TYPE_SHORT			4

#define MIP_IMG_TYPE_NONE			0
#define MIP_IMG_TYPE_INTENSITY			1
#define MIP_IMG_TYPE_RAWDATA			2
#define MIP_IMG_TYPE_WAIT			255

#define MIP_R0_LPWG				0x0E
//Control
#define MIP_R1_LPWG_START 			0x10
#define MIP_R1_LPWG_ENABLE_SENSING		0x11
#define MIP_R1_SET_WAKEUP_BY_SWIPE		0x12
//Common
#define MIP_R1_LPWG_LCD_STATUS			0x20
#define MIP_R1_LPWG_IDLE_REPORTRATE		0x21
#define MIP_R1_LPWG_ACTIVE_REPORTRATE 		0x22
#define MIP_R1_LPWG_SENSITIVITY			0x23
#define MIP_R1_LPWG_ACTIVE_AREA 		0x24
#define MIP_R1_LPWG_FAIL_REASON 		0x2C
//Knock On
#define MIP_R1_LPWG_ENABLE 			0x40
#define MIP_R1_LPWG_WAKEUP_TAP_COUNT		0x41
#define MIP_R1_LPWG_TOUCH_SLOP	 		0x42
#define MIP_R1_LPWG_MIN_INTERTAP_DISTANCE	0x44
#define MIP_R1_LPWG_MAX_INTERTAP_DISTANCE	0x46
#define MIP_R1_LPWG_MIN_INTERTAP_TIME		0x48
#define MIP_R1_LPWG_MAX_INTERTAP_TIME		0x4A
#define MIP_R1_LPWG_INT_DELAY_TIME		0x4C
//Knock Code
#define MIP_R1_LPWG_ENABLE2			0x50
#define MIP_R1_LPWG_WAKEUP_TAP_COUNT2		0x51
#define MIP_R1_LPWG_TOUCH_SLOP2	 		0x52
#define MIP_R1_LPWG_MIN_INTERTAP_DISTANCE2	0x54
#define MIP_R1_LPWG_MAX_INTERTAP_DISTANCE2	0x56
#define MIP_R1_LPWG_MIN_INTERTAP_TIME2		0x58
#define MIP_R1_LPWG_MAX_INTERTAP_TIME2		0x5A
#define MIP_R1_LPWG_INT_DELAY_TIME2		0x5C
//Swipe
#define MIP_R1_SWIPE_ENABLE                 0x60
#define MIP_R1_SWIPE_DISTANCE_THRESHOLD     0x61
#define MIP_R1_SWIPE_RATIO_THRESHOLD        0x62
#define MIP_R1_SWIPE_RATIO_CHECK_PERIOD     0x63
#define MIP_R1_SWIPE_MIN_TIME_THRESHOLD     0x64
#define MIP_R1_SWIPE_MAX_TIME_THRESHOLD     0x66
#define MIP_R1_SWIPE_INTERRUPT_STATUS       0x68
#define MIP_R1_SWIPE_TOUCH_TIME             0x69

#define MIP_R1_CTRL_POWER_STATE			0x13
#define MIP_R1_CTRL_LPWG_DEBUG_ENABLE		0x1F

#define MIP_LPWG_EVENT_TYPE_FAIL		1

/* MIT 200 Registers */
#define MIT_REGH_CMD				0x10
#define MIT_REGL_UCMD				0xA0
#define MIT_REGL_UCMD_RESULT_LENGTH		0xAE
#define MIT_REGL_UCMD_RESULT			0xAF

#define MIT_UNIV_ENTER_TESTMODE			0x40
#define MIT_UNIV_TESTA_START			0x41
#define MIT_UNIV_GET_RAWDATA			0x44
#define MIT_UNIV_TESTB_START			0x48
#define MIT_UNIV_GET_OPENSHORT_TEST		0x50
#define MIT_UNIV_EXIT_TESTMODE			0x6F
#define MIT_UNIV_GET_READ_OTP_STATUS		0x77
#define MIT_UNIV_SEND_THERMAL_INFO		0x58

#define MIT_CMD_SET_LOG_MODE			0x20

/* Event types */
#define MIT_LOG_EVENT				0xD
#define MIT_LPWG_EVENT				0xE
#define MIT_ERROR_EVENT				0xF
#define MIT_TOUCH_KEY_EVENT			0x40
#define MIT_REQUEST_THERMAL_INFO		0xB
#define MIT_ERRORCODE_FAIL_REASON		0x14

enum  {
	OTP_NOT_SUPPORTED = 0,
	OTP_NONE,
	OTP_APPLIED,
};

enum {
	FAIL_OUT_OF_AREA = 1,
	FAIL_PALM,
	FAIL_DELAY_TIME,
	FAIL_TAP_TIME,
	FAIL_TAP_DISTACE,
	FAIL_TOUCH_SLOPE,
	FAIL_MULTI_TOUCH,
	FAIL_LONG_PRESS,
	FAIL_SWIPE_FINGER_RELEASE,
	FAIL_SWIPE_MULTIPLE_FINGERS,
	FAIL_SWIPE_TOO_FAST,
	FAIL_SWIPE_TOO_SLOW,
	FAIL_SWIPE_UPWARD,
	FAIL_SWIPE_RATIO_EXECESS
};

enum {
        LPWG_DISABLE_SENSING = 0,
        LPWG_ENABLE_SENSING = 1,
};

enum {
	LOG_TYPE_U08	= 2,
	LOG_TYPE_S08,
	LOG_TYPE_U16,
	LOG_TYPE_S16,
	LOG_TYPE_U32	= 8,
	LOG_TYPE_S32,
};

enum {
	RAW_DATA_SHOW	= 0,
	RAW_DATA_STORE,
	OPENSHORT,
	OPENSHORT_STORE,
	CM_DELTA_SHOW,
	CM_DELTA_STORE,
	CM_JITTER_SHOW,
	CM_JITTER_STORE,
};

enum {
	SD_RAWDATA = 0,
	SD_OPENSHORT,
	SD_CM_DELTA,
	SD_CM_JITTER,
};

struct mit_dev {
	u16 x_resolution;
	u16 y_resolution;
	u8 contact_on_event_thres;
	u8 moving_event_thres;
	u8 active_report_rate;
	u8 operation_mode;
	u8 tx_ch_num;
	u8 rx_ch_num;
	u8 row_num;
	u8 col_num;
	u8 key_num;
	u8 lcd_status;
};

struct mit_section {
	u8 version;
	u8 compatible_version;
	u8 start_addr;
	u8 end_addr;
	int offset;
	u32 crc;
};

struct mit_module {
	u8 product_code[16];
	u8 version[2];
	u8 ic_name[5];
	u8 bin_version[2];
	char bin_chip_name[4];
	u8 otp;
};

struct mit_log {
	u8 *data;
	int cmd;
};

struct mit_bin_hdr {
	char	tag[8];
	u16	core_version;
	u16	section_num;
	u16	contains_full_binary;
	u16	reserved0;

	u32	binary_offset;
	u32	binary_length;

	u32	extention_offset;
	u32	reserved1;
} __attribute__ ((packed));

struct mit_fw_img {
	u16	type;
	u16	version;

	u16	start_page;
	u16	end_page;

	u32	offset;
	u32	length;

} __attribute__ ((packed));

/**
* Firmware binary tail info
*/
struct mip_bin_tail {
	u8 tail_mark[4];
	char chip_name[4];
	u32 bin_start_addr;
	u32 bin_length;

	u16 ver_boot;
	u16 ver_core;
	u16 ver_app;
	u16 ver_param;
	u8 boot_start;
	u8 boot_end;
	u8 core_start;
	u8 core_end;
	u8 app_start;
	u8 app_end;
	u8 param_start;
	u8 param_end;

	u8 checksum_type;
	u8 hw_category;
	u16 param_id;
	u32 param_length;
	u32 build_date;
	u32 build_time;

	u32 reserved1;
	u32 reserved2;
	u16 reserved3;
	u16 tail_size;
	u32 crc;
} __attribute__ ((packed));

#define MIP_BIN_TAIL_MARK		{0x4D, 0x42, 0x54, 0x01}	// M B T 0x01
#define MIP_BIN_TAIL_SIZE		64

//Firmware update
#define MIP_FW_UPDATE_DEBUG		0	// 0 (defualt) or 1
#define CHIP_NAME			"MIT300"
#define CHIP_FW_CODE			"T3H0"

/**
* Firmware update error code
*/
enum fw_update_errno{
	fw_err_file_read = -4,
	fw_err_file_open = -3,
	fw_err_file_type = -2,
	fw_err_download = -1,
	fw_err_none = 0,
	fw_err_uptodate = 1,
};


struct mit_data {
	bool probed;

	struct i2c_client *client;
	struct touch_platform_data *pdata;
	struct regulator *vdd_regulator[TOUCH_PWR_NUM];
	struct lpwg_tci_data *lpwg_data;
	struct mit_dev dev;
	bool need_update[SECTION_NUM];
	struct mit_section ts_section[SECTION_NUM];
	struct mit_bin_hdr *fw_hdr;
	struct mit_fw_img* fw_img[SECTION_NUM];
	struct mit_module module;
	char buf[PACKET_SIZE];
	struct mit_log log;
	uint16_t *mit_data[MAX_ROW];
	s16 *intensity_data[MAX_ROW];
	u8 test_mode;
	int count_short;
	int thermal_info_send_block;
	int r_max;
	int r_min;
	int o_max;
	int o_min;
	int d_max;
	int d_min;
	int j_max;
	int j_min;

	struct class *class;
	dev_t mip_dev;
	struct cdev cdev;
	u8 *dev_fs_buf;

};

struct mit_log_pkt {
	u8	marker;
	u8	log_info;
	u8	code;
	u8	element_sz;
	u8	row_sz;
} __attribute__ ((packed));

enum {
	NO_SUPPORT_SWIPE = 0,
	SUPPORT_SWIPE,
};

enum {
	SWIPE_DISABLE = 0,
	SWIPE_ENABLE,
};

#define mit_i2c_write_block(client, buf, len) i2c_master_send(client, buf, len)
extern atomic_t dev_state;
#if defined(TOUCH_USE_DSV)
extern void mdss_dsv_ctl(int mdss_dsv_en);
#endif

int mit_i2c_read(struct i2c_client *client, char *write_buf, unsigned int write_len, char *read_buf, unsigned int read_len);
int mit_isc_fwupdate(struct mit_data *ts, struct touch_fw_info *info);
int mit_set_gpio_mode(struct touch_platform_data *pdata, int mode);
int mit_power_ctrl(struct i2c_client* client, int power_ctrl);
int mit_power_reset(struct mit_data *ts);
ssize_t mit_get_test_result(struct i2c_client *client, char *buf, int type);
ssize_t mit_delta_show(struct i2c_client *client, char *buf);
//ssize_t mit_rawdata_show(struct i2c_client *client,char *buf);
//ssize_t mit_openshort_show(struct i2c_client *client, char *buf);
int mip_isc_read_page(struct mit_data *info, int offset, u8 *data);
int mip_isc_exit(struct mit_data *info);
//fw
int mip_flash_fw(struct mit_data *ts, const u8 *fw_data, size_t fw_size, bool force, bool section);
void mip_reboot(struct i2c_client *client);
int mip_get_fw_version(struct i2c_client *client, u8 *ver_buf);
int mip_get_fw_version_u16(struct i2c_client *client, u16 *ver_buf_u16);
#endif // LGE_TS_MELFAS_H

