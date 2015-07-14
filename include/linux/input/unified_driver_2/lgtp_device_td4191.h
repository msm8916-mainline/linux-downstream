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
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/


/****************************************************************************
* Nested Include Files
****************************************************************************/
#include <linux/input/unified_driver_2/lgtp_common.h>

#define DO_IF(do_work, goto_error) 				\
do {								\
	if (do_work) { 						\
		printk(KERN_INFO "[Touch E] Action Failed [%s %d] \n",	\
			__func__, __LINE__); \
		goto goto_error; 				\
	}							\
} while (0)

#define DO_SAFE(do_work, goto_error) 				\
	DO_IF(unlikely((do_work) < 0), goto_error)

/****************************************************************************
* Mainfest Constants / Defines
****************************************************************************/
#define DESCRIPTION_TABLE_START			0xe9
#define DEFAULT_PAGE					0x00
#define PAGE_SELECT_REG					0xFF
#define PAGE_MAX_NUM					5

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

struct synaptics_ts_f12_query_5 {
	union {
		struct {
			unsigned char size_of_query_6;
			struct {
				unsigned char ctrl_00_is_present:1;
				unsigned char ctrl_01_is_present:1;
				unsigned char ctrl_02_is_present:1;
				unsigned char ctrl_03_is_present:1;
				unsigned char ctrl_04_is_present:1;
				unsigned char ctrl_05_is_present:1;
				unsigned char ctrl_06_is_present:1;
				unsigned char ctrl_07_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl_08_is_present:1;
				unsigned char ctrl_09_is_present:1;
				unsigned char ctrl_10_is_present:1;
				unsigned char ctrl_11_is_present:1;
				unsigned char ctrl_12_is_present:1;
				unsigned char ctrl_13_is_present:1;
				unsigned char ctrl_14_is_present:1;
				unsigned char ctrl_15_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl_16_is_present:1;
				unsigned char ctrl_17_is_present:1;
				unsigned char ctrl_18_is_present:1;
				unsigned char ctrl_19_is_present:1;
				unsigned char ctrl_20_is_present:1;
				unsigned char ctrl_21_is_present:1;
				unsigned char ctrl_22_is_present:1;
				unsigned char ctrl_23_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl_24_is_present:1;
				unsigned char ctrl_25_is_present:1;
				unsigned char ctrl_26_is_present:1;
				unsigned char ctrl_27_is_present:1;
				unsigned char ctrl_28_is_present:1;
				unsigned char ctrl_29_is_present:1;
				unsigned char ctrl_30_is_present:1;
				unsigned char ctrl_31_is_present:1;
			} __packed;
		};
		unsigned char data[5];
	};
};

struct synaptics_ts_f12_query_8 {
	union {
		struct {
			unsigned char size_of_query_9;
			struct {
				unsigned char data_00_is_present:1;
				unsigned char data_01_is_present:1;
				unsigned char data_02_is_present:1;
				unsigned char data_03_is_present:1;
				unsigned char data_04_is_present:1;
				unsigned char data_05_is_present:1;
				unsigned char data_06_is_present:1;
				unsigned char data_07_is_present:1;
			} __packed;
			struct {
				unsigned char data_08_is_present:1;
				unsigned char data_09_is_present:1;
				unsigned char data_10_is_present:1;
				unsigned char data_11_is_present:1;
				unsigned char data_12_is_present:1;
				unsigned char data_13_is_present:1;
				unsigned char data_14_is_present:1;
				unsigned char data_15_is_present:1;
			} __packed;
		};
		unsigned char data[3];
	};
};

#define F12_NO_OBJECT_STATUS		(0x00)
#define F12_FINGER_STATUS			(0x01)
#define F12_STYLUS_STATUS			(0x02)
#define F12_PALM_STATUS				(0x03)
#define F12_HOVERING_FINGER_STATUS	(0x05)
#define F12_GLOVED_FINGER_STATUS	(0x06)

#define TD4191    4

#define F35_ERROR_CODE_OFFSET 0
#define F35_CHUNK_NUM_LSB_OFFSET 0
#define F35_CHUNK_NUM_MSB_OFFSET 1
#define F35_CHUNK_DATA_OFFSET 2
#define F35_CHUNK_COMMAND_OFFSET 18

#define F35_CHUNK_SIZE 16
#define F35_ERASE_ALL_WAIT_MS 2000
#define F35_RESET_WAIT_MS 250

struct synaptics_ts_fw_info {
	u8		fw_version[5];
	u8		fw_product_id[11];
	u8		fw_image_version[5];
	u8		fw_image_product_id[11];
	unsigned char	*fw_start;
	unsigned char   family;
	unsigned char   fw_revision;
	unsigned long	fw_size;
	u8		need_rewrite_firmware;
};

struct synaptics_ts_data {
	struct i2c_client	*client;
	struct ts_ic_function	common_fc;
	struct ts_ic_function	custom_fc;
	struct ts_ic_function	finger_fc;
	struct ts_ic_function	button_fc;
	struct ts_ic_function	analog_fc;
	struct ts_ic_function	sensor_fc;
    struct ts_ic_function	video_fc;
	struct ts_ic_function	flash_fc;
    struct synaptics_ts_fw_info	fw_info;
	TouchState currState;
	LpwgSetting lpwgSetting;
    bool ubootloader_mode;
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
	REPORT_MODE_CTRL	 = 1,
	TCI_ENABLE_CTRL,
	TAP_COUNT_CTRL,
	MIN_INTERTAP_CTRL,
	MAX_INTERTAP_CTRL,
	TOUCH_SLOP_CTRL,
	TAP_DISTANCE_CTRL,
	INTERRUPT_DELAY_CTRL,

	TCI_ENABLE_CTRL2     = 22,
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

enum {
	RESET_NONE = 0,
	SOFT_RESET,
	PIN_RESET,
	VDD_RESET,
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
	INTERRUPT_MASK_ABS0		= 0x04,
	INTERRUPT_MASK_CUSTOM   = 0x40,
};

enum DEVICE_FLASH_REG {
	DEVICE_CRC_ERROR_MASK		= (1U << 2),
	DEVICE_STATUS_FLASH_PROG	= (1U << 6),
};

enum SENSING_CTRL_REG {
	SENSING_CONTROL_AUTO		= 0,
	SENSING_CONTROL_60HZ		= (1U << 1),
	SENSING_CONTROL_120HZ		= (1U << 2),
};

enum F54_TEST {
	F54_FULL_RAW_CAP		= 0,
	F54_HIGH_RESISTANCE,
	F54_TRX_TO_TRX,
	F54_FULL_RAW_CAP_TD4191,
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
extern int FirmwareRecovery(struct synaptics_ts_data *ts, const char *fw_path);
extern void SynaScanPDT(struct synaptics_ts_data *ts);
int synaptics_ts_rmidev_function(struct synaptics_ts_exp_fn *rmidev_fn, bool insert);


/* End Of File */

