/***************************************************************************
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
 *    File  	: lgtp_device_s3320.h
 *    Author(s)   : Branden You < branden.you@lge.com >
 *    Description :
 *
 ***************************************************************************/

#if !defined ( _LGTP_DEVICE_S3320_H_ )
#define _LGTP_DEVICE_S3320_H_

/****************************************************************************
* Nested Include Files
****************************************************************************/
#include <linux/input/lgtp_common.h>


/****************************************************************************
* Mainfest Constants / Defines
****************************************************************************/
#define DESCRIPTION_TABLE_START			0xe9
#define DEFAULT_PAGE					0x00
#define PAGE_SELECT_REG					0xFF
#define PAGE_MAX_NUM					6

#define NUM_OF_EACH_FINGER_DATA_REG		8
#define MAX_NUM_OF_FINGERS				10
#define FW_VER_INFO_NUM					4

/****************************************************************************
* Type Definitions
****************************************************************************/

struct function_descriptor {
	u8 	query_base;
	u8 	command_base;
	u8 	control_base;
	u8 	data_base;
	u8 	int_source_count;
	u8 	id;
};

struct ts_ic_function {
	struct function_descriptor dsc;
	u8 	function_page;
};

struct finger_data {
	u8	finger_reg[MAX_NUM_OF_FINGERS][NUM_OF_EACH_FINGER_DATA_REG];
};

struct button_data {
	u16	key_code;
};

struct cur_touch_data {
	u8	device_status_reg;
	u8	interrupt_status_reg;
	u8	button_data_reg;
	struct finger_data	finger;
	struct button_data	button;
};

struct synaptics_ts_fw_info {
	u8	fw_version[FW_VER_INFO_NUM+1];
	u8	fw_image_version[FW_VER_INFO_NUM+1];
};

struct lpwg_password_data {
	u8		tap_count;
	u8		data_num;
	u8		double_tap_check;
	struct point 	data[MAX_POINT_SIZE_FOR_LPWG];
};

enum {
	TIC_NORMAL = 0,
	TIC_LPWG_OFF,
	TIC_KNOCK_ON_ONLY,
	TIC_KNOCK_CODE_ONLY,
	TIC_KNOCK_ON_CODE
};

struct synaptics_ts_data {
	u8	is_probed;
	u8	is_init;
	struct lpwg_password_data	pw_data;
	struct regulator	*regulator_vdd;
	struct regulator	*regulator_vio;
	struct i2c_client	*client;
	struct ts_ic_function	common_fc;
	struct ts_ic_function	lpwg_fc;
	struct ts_ic_function	finger_fc;
	struct ts_ic_function	button_fc;
	struct ts_ic_function	analog_fc;
	struct ts_ic_function	sensor_fc;
	struct ts_ic_function	flash_fc;
	struct touch_fw_info*	fw_info;
	struct synaptics_ts_fw_info	synaptics_fw_info;
	E_TouchReportMode reportMode;
	LGTcLpwgSetting lpwgSetting;
	char *pFirmwareImage;
#if defined ( USE_OVER_TAP_COUNT_DETECTION_TIMER )
	struct delayed_work	work_timer;
#endif
};

struct synaptics_ts_exp_fn {
	int (*init)(struct synaptics_ts_data *ts);
	void (*remove)(struct synaptics_ts_data *ts);
	void (*reset)(struct synaptics_ts_data *ts);
	void (*reinit)(struct synaptics_ts_data *ts);
	void (*early_suspend)(struct synaptics_ts_data *ts);
	void (*suspend)(struct synaptics_ts_data *ts);
	void (*resume)(struct synaptics_ts_data *ts);
	void (*late_resume)(struct synaptics_ts_data *ts);
	void (*attn)(struct synaptics_ts_data *ts, unsigned char intr_mask);
};

struct synaptics_ts_exp_fhandler {
	struct synaptics_ts_exp_fn *exp_fn;
	bool inserted;
	bool initialized;
};

enum TCI_CTRL {
	REPORT_MODE_CTRL	 = 0,
	TCI_ENABLE_CTRL,
	TAP_COUNT_CTRL,
	MIN_INTERTAP_CTRL,
	MAX_INTERTAP_CTRL,
	TOUCH_SLOP_CTRL,
	TAP_DISTANCE_CTRL,
	INTERRUPT_DELAY_CTRL,

	TCI_ENABLE_CTRL2,
	TAP_COUNT_CTRL2,
	MIN_INTERTAP_CTRL2,
	MAX_INTERTAP_CTRL2,
	TOUCH_SLOP_CTRL2,
	TAP_DISTANCE_CTRL2,
	INTERRUPT_DELAY_CTRL2,
};

enum REPORT_VALUE {
	REG_OBJECT	= 0,
	REG_X_LSB,
	REG_X_MSB,
	REG_Y_LSB,
	REG_Y_MSB,
	REG_Z,
	REG_WX,
	REG_WY,
};

enum DEVICE_CTRL_REG {
	DEVICE_CONTROL_NORMAL_OP		= 0,
	DEVICE_CONTROL_SLEEP			= (1U << 0),
	DEVICE_CONTROL_SLEEP_NO_RECAL	= (1U << 1),
	DEVICE_CONTROL_NOSLEEP			= (1U << 2),
	DEVICE_CONTROL_CONFIGURED		= (1U << 7),
};

enum INTERRUPT_MASK_REG {
	INTERRUPT_MASK_NONE		= 0,
	INTERRUPT_MASK_ABS0		= (1U << 2),
	INTERRUPT_MASK_CUSTOM	= (1U << 6),
};

enum DEVICE_FLASH_REG {
	DEVICE_CRC_ERROR_MASK		= (1U << 2),
	DEVICE_STATUS_FLASH_PROG	= (1U << 6),
};

enum SENSING_CTRL_REG {
	SENSING_CONTROL_AUTO		= 0,
	SENSING_CONTROL_60HZ		= (1U << 1),
	SENSING_CONTROL_120HZ 	    = (1U << 2),
};

enum F54_TEST {
	F54_FULL_RAW_CAP		= 0,
	F54_HIGH_RESISTANCE,
	F54_TRX_TO_TRX
};

/****************************************************************************
* Exported Variables
****************************************************************************/


/****************************************************************************
* Macros
****************************************************************************/


/****************************************************************************
* Global Function Prototypes
****************************************************************************/
extern int FirmwareUpgrade(struct synaptics_ts_data *ts, const char* fw_path);
int synaptics_ts_rmidev_function(struct synaptics_ts_exp_fn *rmidev_fn, bool insert);

#endif /* _LGTP_DEVICE_S3320_H_ */

/* End Of File */

