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
 *    File  	: lgtp_device_s3320.c
 *    Author(s)   : Branden You < branden.you@lge.com >
 *    Description :
 *
 ***************************************************************************/

/****************************************************************************
* Include Files
****************************************************************************/
#include <linux/input/lgtp_common.h>

#include <linux/input/lgtp_common_driver.h>
#include <linux/input/lgtp_platform_api.h>
#include <linux/input/lgtp_model_config.h>
#include <linux/input/lgtp_device_s3320.h>

/****************************************************************************
* Manifest Constants / Defines
****************************************************************************/
/* RMI4 spec from 511-000405-01 Rev.D
 * Function	Purpose				See page
 * $01		RMI Device Control		45
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

#define RMI_DEVICE_CONTROL			0x01
#define TOUCHPAD_SENSORS			0x12
#define FLASH_MEMORY_MANAGEMENT		0x34
#define LPWG_CONTROL				0x51
#define ANALOG_CONTROL				0x54
#define SENSOR_CONTROL				0x55

/* Register Map & Register bit mask
 * - Please check "One time" this map before using this device driver
 */
#define DEVICE_CONTROL_REG 			(ts->common_fc.dsc.control_base)
#define MANUFACTURER_ID_REG			(ts->common_fc.dsc.query_base)
#define CUSTOMER_FAMILY_REG			(ts->common_fc.dsc.query_base+2)
#define FW_REVISION_REG				(ts->common_fc.dsc.query_base+3)
#define PRODUCT_ID_REG				(ts->common_fc.dsc.query_base+11)

#define COMMON_PAGE					(ts->common_fc.function_page)
#define LPWG_PAGE					(ts->lpwg_fc.function_page)
#define FINGER_PAGE					(ts->finger_fc.function_page)
#define ANALOG_PAGE					(ts->analog_fc.function_page)
#define FLASH_PAGE					(ts->flash_fc.function_page)
#define SENSOR_PAGE					(ts->sensor_fc.function_page)

#define DEVICE_STATUS_REG			(ts->common_fc.dsc.data_base)
#define INTERRUPT_STATUS_REG		(ts->common_fc.dsc.data_base+1)
#define INTERRUPT_ENABLE_REG		(ts->common_fc.dsc.control_base+1)

/* TOUCHPAD_SENSORS */
#define FINGER_DATA_REG_START		(ts->finger_fc.dsc.data_base)
#define FINGER_REPORT_REG			(ts->finger_fc.dsc.control_base+7)
#define OBJECT_REPORT_ENABLE_REG	(ts->finger_fc.dsc.control_base+8)
#define WAKEUP_GESTURE_ENABLE_REG	(ts->finger_fc.dsc.control_base+12)

#define DYNAMIC_SENSING_CONTROL_REG	(ts->analog_fc.dsc.control_base+40)

#define FLASH_CONFIG_ID_REG			(ts->flash_fc.dsc.control_base)
#define FLASH_STATUS_REG			(ts->flash_fc.dsc.data_base+3)

#define LPWG_STATUS_REG				(ts->lpwg_fc.dsc.data_base)
#define LPWG_DATA_REG				(ts->lpwg_fc.dsc.data_base+1)
#define LPWG_TAPCOUNT_REG			(ts->lpwg_fc.dsc.control_base)
#define LPWG_MIN_INTERTAP_REG		(ts->lpwg_fc.dsc.control_base+1)
#define LPWG_MAX_INTERTAP_REG		(ts->lpwg_fc.dsc.control_base+2)
#define LPWG_TOUCH_SLOP_REG			(ts->lpwg_fc.dsc.control_base+3)
#define LPWG_TAP_DISTANCE_REG		(ts->lpwg_fc.dsc.control_base+4)
#define LPWG_INTERRUPT_DELAY_REG	(ts->lpwg_fc.dsc.control_base+6)
#define LPWG_TAPCOUNT_REG2			(ts->lpwg_fc.dsc.control_base+7)
#define LPWG_MIN_INTERTAP_REG2		(ts->lpwg_fc.dsc.control_base+8)
#define LPWG_MAX_INTERTAP_REG2		(ts->lpwg_fc.dsc.control_base+9)
#define LPWG_TOUCH_SLOP_REG2		(ts->lpwg_fc.dsc.control_base+10)
#define LPWG_TAP_DISTANCE_REG2		(ts->lpwg_fc.dsc.control_base+11)
#define LPWG_INTERRUPT_DELAY_REG2	(ts->lpwg_fc.dsc.control_base+13)

#define MAX_PRESSURE			255
#define LPWG_BLOCK_SIZE			7
#define KNOCKON_DELAY			68	/* 700ms */

/****************************************************************************
 * Macros
 ****************************************************************************/
/* Get user-finger-data from register.
 */
#define GET_X_POSITION(_msb_reg, _lsb_reg) \
	(((u16)((_msb_reg << 8)  & 0xFF00)  | (u16)((_lsb_reg) & 0xFF)))
#define GET_Y_POSITION(_msb_reg, _lsb_reg) \
	(((u16)((_msb_reg << 8)  & 0xFF00)  | (u16)((_lsb_reg) & 0xFF)))
#define GET_WIDTH_MAJOR(_width_x, _width_y) \
	((_width_x - _width_y) > 0) ? _width_x : _width_y
#define GET_WIDTH_MINOR(_width_x, _width_y) \
	((_width_x - _width_y) > 0) ? _width_y : _width_x

#define GET_ORIENTATION(_width_y, _width_x) \
	((_width_y - _width_x) > 0) ? 0 : 1
#define GET_PRESSURE(_pressure) \
		_pressure

/****************************************************************************
* Type Definitions
****************************************************************************/


/****************************************************************************
* Variables
****************************************************************************/
static struct synaptics_ts_exp_fhandler rmidev_fhandler;
int enable_rmi_dev = 0;

#if defined ( USE_OVER_TAP_COUNT_DETECTION_TIMER )
extern struct workqueue_struct *touch_wq;
#endif

/****************************************************************************
* Extern Function Prototypes
****************************************************************************/
extern int F54TestHandle(struct synaptics_ts_data *ts, int input, int mode, char *buf);//mode:0 => write_log, mode:1 && buf => cat, mode:2 && buf => delta


/****************************************************************************
* Local Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Functions
****************************************************************************/
static int synaptics_ts_page_data_read(struct i2c_client *client,
	 u8 page, u8 reg, int size, u8 *data)
 {
	 DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, page), error);
	 DO_SAFE(touch_i2c_read(client, reg, size, data), error);
	 DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE),
		 error);
 
	 return 0;
 error:
	 return -1;
 }
 
static error_type synaptics_ts_page_data_write(struct i2c_client *client,
	 u8 page, u8 reg, int size, u8 *data)
 {
	 DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, page), error);
	 DO_SAFE(touch_i2c_write(client, reg, size, data), error);
	 DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE),
		 error);
 
	 return NO_ERROR;
 error:
	 return ERROR;
 }
 
static error_type synaptics_ts_page_data_write_byte(struct i2c_client *client,
	 u8 page, u8 reg, u8 data)
 {
	 DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, page), error);
	 DO_SAFE(touch_i2c_write_byte(client, reg, data), error);
	 DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE),
		 error);
 
	 return NO_ERROR;
 error:
	 return ERROR;
 }

static error_type read_page_description_table(struct i2c_client *client)
{
	struct synaptics_ts_data *ts
		= (struct synaptics_ts_data *)get_touch_handle(client);
	struct function_descriptor buffer;

	unsigned short address = 0;
	unsigned short page_num = 0;

	LGTC_FUN();

	memset(&buffer, 0x0, sizeof(struct function_descriptor));
	memset(&ts->common_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->finger_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->lpwg_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->sensor_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->analog_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->flash_fc, 0x0, sizeof(struct ts_ic_function));

	for (page_num = 0; page_num < PAGE_MAX_NUM; page_num++) {
		DO_SAFE(touch_i2c_write_byte(client,
			PAGE_SELECT_REG, page_num), error);

		for (address = DESCRIPTION_TABLE_START; address > 10;
				address -= sizeof(struct function_descriptor)) {
			DO_SAFE(touch_i2c_read(client, address, sizeof(buffer),
				(unsigned char *)&buffer) < 0, error);

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
			case LPWG_CONTROL:
				ts->lpwg_fc.dsc = buffer;
				ts->lpwg_fc.function_page = page_num;
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
			default:
				break;
			}
		}
	}

	DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, 0x00), error);
	ERROR_IF(ts->common_fc.dsc.id == 0 || ts->finger_fc.dsc.id == 0
		|| ts->analog_fc.dsc.id == 0 || ts->flash_fc.dsc.id == 0,
		"page_init_error", error);

	LGTC_DBG("common[%dP:0x%02x] finger[%dP:0x%02x] lpwg[%dP:0x%02x] sensor[%dP:0x%02x]"
		"analog[%dP:0x%02x] flash[%dP:0x%02x]\n",
		ts->common_fc.function_page, ts->common_fc.dsc.id,
		ts->finger_fc.function_page, ts->finger_fc.dsc.id,
		ts->lpwg_fc.function_page, ts->lpwg_fc.dsc.id,
		ts->sensor_fc.function_page, ts->sensor_fc.dsc.id,
		ts->analog_fc.function_page, ts->analog_fc.dsc.id,
		ts->flash_fc.function_page, ts->flash_fc.dsc.id);

	return NO_ERROR;
error:
	LGTC_ERR("Fail to read page description\n");
	return ERROR;
}

static error_type get_fw_ic_info(struct synaptics_ts_data *ts)
{
	DO_SAFE(touch_i2c_read(ts->client, PRODUCT_ID_REG,
		sizeof(ts->fw_info->fw_ic_product_id) - 1,
		ts->fw_info->fw_ic_product_id), error);
	DO_SAFE(touch_i2c_read(ts->client, FLASH_CONFIG_ID_REG,
		FW_VER_INFO_NUM, ts->synaptics_fw_info.fw_version), error);

	sprintf(ts->fw_info->fw_ic_version,"V%d.%02d\n",
		(ts->synaptics_fw_info.fw_version[3] & 0x80 ? 1 : 0), ts->synaptics_fw_info.fw_version[3] & 0x7F);

	if(ts->synaptics_fw_info.fw_version[3] & 0x80)
		ts->fw_info->fw_type = OFFICIAL_VERSION;
	else
		ts->fw_info->fw_type = TEST_VERSION;

	LGTC_DBG("fw_version(ic)[V%d.%02d(0x%02X 0x%02X 0x%02X 0x%02X)]\n",
		(ts->synaptics_fw_info.fw_version[3] & 0x80 ? 1 : 0),
		ts->synaptics_fw_info.fw_version[3] & 0x7F,
		ts->synaptics_fw_info.fw_version[0], ts->synaptics_fw_info.fw_version[1],
		ts->synaptics_fw_info.fw_version[2], ts->synaptics_fw_info.fw_version[3]);

	return NO_ERROR;
error:
	return ERROR;
}

static error_type get_fw_bin_info(struct synaptics_ts_data *ts)
{
	const struct firmware *fw_entry;

	if (request_firmware(&fw_entry, ts->fw_info->fw_path, &ts->client->dev) != 0) {
		LGTC_ERR("request_firmware() failed\n");
		return ERROR;
	}

	memcpy(ts->fw_info->fw_bin_product_id, &fw_entry->data[0x0040], 6);
	memcpy(ts->synaptics_fw_info.fw_image_version, &fw_entry->data[0x16d00], 4);

	sprintf(ts->fw_info->fw_bin_version,"V%d.%02d\n",
		(ts->synaptics_fw_info.fw_image_version[3] & 0x80 ? 1 : 0), ts->synaptics_fw_info.fw_image_version[3] & 0x7F);

	LGTC_DBG("fw_version(fw)[V%d.%02d(0x%02X 0x%02X 0x%02X 0x%02X)]\n",
		(ts->synaptics_fw_info.fw_image_version[3] & 0x80 ? 1 : 0),
		ts->synaptics_fw_info.fw_image_version[3] & 0x7F,
		ts->synaptics_fw_info.fw_image_version[0], ts->synaptics_fw_info.fw_image_version[1],
		ts->synaptics_fw_info.fw_image_version[2], ts->synaptics_fw_info.fw_image_version[3]);

	release_firmware(fw_entry);

	return NO_ERROR;
}

static error_type check_firmware_status(struct synaptics_ts_data *ts)
{
	u8 device_status = 0;
	u8 flash_status = 0;

	touch_i2c_read(ts->client, FLASH_STATUS_REG, sizeof(flash_status), &flash_status);
	touch_i2c_read(ts->client, DEVICE_STATUS_REG, sizeof(device_status), &device_status);

	if ((device_status & DEVICE_STATUS_FLASH_PROG)
			|| (device_status & DEVICE_CRC_ERROR_MASK)
			|| (flash_status & 0xFF)) {
		LGTC_ERR("FLASH_STATUS[0x%x] DEVICE_STATUS[0x%x]\n",
			(u32)flash_status, (u32)device_status);

		return ERROR;
	}

	return NO_ERROR;
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
 * 8		Interrupt Delay
 */
static error_type tci_control(struct synaptics_ts_data *ts, int type, u8 value)
{
	struct i2c_client* client = ts->client;
	u8 buffer[3] = {0};

	switch (type) {
	case REPORT_MODE_CTRL:
		DO_SAFE(touch_i2c_read(ts->client, INTERRUPT_ENABLE_REG, 1, buffer), error);
		DO_SAFE(touch_i2c_write_byte(ts->client, INTERRUPT_ENABLE_REG,
				value ? buffer[0] & ~INTERRUPT_MASK_ABS0 : buffer[0] | INTERRUPT_MASK_ABS0), error);

		DO_SAFE(touch_i2c_read(client, FINGER_REPORT_REG, 3, buffer), error);
		buffer[2] = (buffer[2] & 0xfc) | (value ? 0x2 : 0x0);
		DO_SAFE(touch_i2c_write(client, FINGER_REPORT_REG, 3, buffer), error);
		LGTC_DBG("report mode: %d\n", value);
		break;
	case TCI_ENABLE_CTRL:
		DO_SAFE(synaptics_ts_page_data_read(client,
				LPWG_PAGE, LPWG_TAPCOUNT_REG, 1, buffer), error);
		buffer[0] = (buffer[0] & 0xfe) | (value & 0x1);
		DO_SAFE(synaptics_ts_page_data_write(client,
				LPWG_PAGE, LPWG_TAPCOUNT_REG, 1, buffer), error);
		break;
	case TCI_ENABLE_CTRL2:
		DO_SAFE(synaptics_ts_page_data_read(client,
				LPWG_PAGE, LPWG_TAPCOUNT_REG2, 1, buffer), error);
		buffer[0] = (buffer[0] & 0xfe) | (value & 0x1);
		DO_SAFE(synaptics_ts_page_data_write(client,
				LPWG_PAGE, LPWG_TAPCOUNT_REG2, 1, buffer), error);
		break;
	case TAP_COUNT_CTRL:
		DO_SAFE(synaptics_ts_page_data_read(client,
				LPWG_PAGE, LPWG_TAPCOUNT_REG, 1, buffer), error);
		buffer[0] = ((value << 3) & 0xf8) | (buffer[0] & 0x7);
		DO_SAFE(synaptics_ts_page_data_write(client,
				LPWG_PAGE, LPWG_TAPCOUNT_REG, 1, buffer), error);
		break;
	case TAP_COUNT_CTRL2:
		DO_SAFE(synaptics_ts_page_data_read(client,
				LPWG_PAGE, LPWG_TAPCOUNT_REG2, 1, buffer), error);
		buffer[0] = ((value << 3) & 0xf8) | (buffer[0] & 0x7);
		DO_SAFE(synaptics_ts_page_data_write(client,
				LPWG_PAGE, LPWG_TAPCOUNT_REG2, 1, buffer), error);
		break;
	case MIN_INTERTAP_CTRL:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
				LPWG_PAGE, LPWG_MIN_INTERTAP_REG, value), error);
		break;
	case MIN_INTERTAP_CTRL2:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
				LPWG_PAGE, LPWG_MIN_INTERTAP_REG2, value), error);
		break;
	case MAX_INTERTAP_CTRL:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
				LPWG_PAGE, LPWG_MAX_INTERTAP_REG, value), error);
		break;
	case MAX_INTERTAP_CTRL2:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
				LPWG_PAGE, LPWG_MAX_INTERTAP_REG2, value), error);
		break;
	case TOUCH_SLOP_CTRL:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
				LPWG_PAGE, LPWG_TOUCH_SLOP_REG, value), error);
		break;
	case TOUCH_SLOP_CTRL2:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
				LPWG_PAGE, LPWG_TOUCH_SLOP_REG2, value), error);
		break;
	case TAP_DISTANCE_CTRL:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
				LPWG_PAGE, LPWG_TAP_DISTANCE_REG, value), error);
		break;
	case TAP_DISTANCE_CTRL2:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
				LPWG_PAGE, LPWG_TAP_DISTANCE_REG2, value), error);
		break;
	case INTERRUPT_DELAY_CTRL:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
				LPWG_PAGE, LPWG_INTERRUPT_DELAY_REG,
				value ? (buffer[0] = (KNOCKON_DELAY << 1) | 0x1) : (buffer[0] = 0)), error);
		break;
	case INTERRUPT_DELAY_CTRL2:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
				LPWG_PAGE, LPWG_INTERRUPT_DELAY_REG2,
				value ? (buffer[0] = (KNOCKON_DELAY << 1) | 0x1) : (buffer[0] = 0)), error);
		break;

	default:
		break;
	}

	return NO_ERROR;
error:
	return ERROR;
}

static error_type get_lpwg_data(struct synaptics_ts_data *ts, int count)
{
	struct i2c_client *client = ts->client;
	u8 i = 0;
	u8 buffer[12][4] = { {0} };

	ts->pw_data.data_num = count;

	if (!count)
		return 0;

	DO_SAFE(i = synaptics_ts_page_data_read(client, LPWG_PAGE,
		LPWG_DATA_REG, 4*count, &buffer[0][0]), error);

	for (i = 0; i < count; i++) {
		ts->pw_data.data[i].x
			= GET_X_POSITION(buffer[i][1], buffer[i][0]);
		ts->pw_data.data[i].y
			= GET_Y_POSITION(buffer[i][3], buffer[i][2]);
		LGTC_DBG("LPWG data [%d, %d]\n",
			ts->pw_data.data[i].x, ts->pw_data.data[i].y);
	}

	return NO_ERROR;
error:
	return ERROR;
}


static error_type sleep_control(struct synaptics_ts_data *ts, int mode, int recal)
{
	u8 curr = 0;
	u8 next = 0;

	DO_SAFE(touch_i2c_read(ts->client, DEVICE_CONTROL_REG, 1, &curr), error);

	next = (curr & 0xFC) | (mode ? DEVICE_CONTROL_NORMAL_OP : DEVICE_CONTROL_SLEEP);
	DO_SAFE(touch_i2c_write_byte(ts->client, DEVICE_CONTROL_REG, next), error);

	LGTC_DBG("%s : curr = [%x] next[%x]\n", __func__, curr, next);

	return NO_ERROR;
error:
	return ERROR;
}

static error_type lpwg_control(struct synaptics_ts_data *ts)
{
	u8 buf = 0;

	LGTC_FUN();

	DO_SAFE(touch_i2c_read(ts->client, INTERRUPT_ENABLE_REG, 1, &buf), error);
	DO_SAFE(touch_i2c_write_byte(ts->client, INTERRUPT_ENABLE_REG, 0x00), error);

	switch (ts->reportMode) {
	case T_REPORT_NORMAL:
		tci_control(ts, TCI_ENABLE_CTRL, 0);		// tci-1 disable
		tci_control(ts, TCI_ENABLE_CTRL2, 0);		// tci-2 disable
		tci_control(ts, REPORT_MODE_CTRL, 0);		// normal
		sleep_control(ts, 1, 0);
		break;
	case T_REPORT_KNOCK_ON_ONLY:					// Only TCI-1
		sleep_control(ts, 1, 0);
		tci_control(ts, TCI_ENABLE_CTRL, 1);		// tci enabl
		tci_control(ts, TAP_COUNT_CTRL, 2); 		// tap count = 2
		tci_control(ts, MIN_INTERTAP_CTRL, 0);		// min inter_tap = 0ms
		tci_control(ts, MAX_INTERTAP_CTRL, 70);		// max inter_tap = 700ms
		tci_control(ts, TOUCH_SLOP_CTRL, 100);		// touch_slop = 10mm
		tci_control(ts, TAP_DISTANCE_CTRL, 10);		// tap distance = 10mm
		tci_control(ts, INTERRUPT_DELAY_CTRL, 0);	// interrupt delay = 0ms
		tci_control(ts, TCI_ENABLE_CTRL2, 0);		// tci-2 disable
		tci_control(ts, REPORT_MODE_CTRL, 1);		// wakeup_gesture_only
		break;
	case T_REPORT_KNOCK_ON_CODE:					// TCI-1 and TCI-2
		sleep_control(ts, 1, 0);
		tci_control(ts, TCI_ENABLE_CTRL, 1);		// tci-1 enable
		tci_control(ts, TAP_COUNT_CTRL, 2); 		// tap count = 2
		tci_control(ts, MIN_INTERTAP_CTRL, 0);		// min inter_tap = 0ms
		tci_control(ts, MAX_INTERTAP_CTRL, 70);		// max inter_tap = 700ms
		tci_control(ts, TOUCH_SLOP_CTRL, 100);		// touch_slop = 10mm
		tci_control(ts, TAP_DISTANCE_CTRL, 7);		// tap distance = 7mm
		tci_control(ts, INTERRUPT_DELAY_CTRL, (u8)ts->lpwgSetting.isFirstTwoTapSame);	// interrupt delay = 0ms

		tci_control(ts, TCI_ENABLE_CTRL2, 1);		// tci-2 enable
		tci_control(ts, TAP_COUNT_CTRL2, (u8)ts->lpwgSetting.tapCount); // tap count = "user_setting"
		tci_control(ts, MIN_INTERTAP_CTRL2, 0);		// min inter_tap = 0ms
		tci_control(ts, MAX_INTERTAP_CTRL2, 70);	// max inter_tap = 700ms
		tci_control(ts, TOUCH_SLOP_CTRL2, 100);		// touch_slop = 10mm
		tci_control(ts, TAP_DISTANCE_CTRL2, 255);	// tap distance = MAX
		tci_control(ts, INTERRUPT_DELAY_CTRL2, 0);	// interrupt delay = 0ms

		tci_control(ts, REPORT_MODE_CTRL, 1);		// wakeup_gesture_only
		break;
	case T_REPORT_KNOCK_CODE_ONLY:
		sleep_control(ts, 1, 0);
		tci_control(ts, TCI_ENABLE_CTRL, 0);		// tci-1 enable
		tci_control(ts, TAP_COUNT_CTRL, 2); 		// tap count = 2
		tci_control(ts, MIN_INTERTAP_CTRL, 0);		// min inter_tap = 0ms
		tci_control(ts, MAX_INTERTAP_CTRL, 70);		// max inter_tap = 700ms
		tci_control(ts, TOUCH_SLOP_CTRL, 100);		// touch_slop = 10mm
		tci_control(ts, TAP_DISTANCE_CTRL, 7);		// tap distance = 7mm
		tci_control(ts, INTERRUPT_DELAY_CTRL, (u8)ts->lpwgSetting.isFirstTwoTapSame);	// interrupt delay = 0ms

		tci_control(ts, TCI_ENABLE_CTRL2, 1);		// tci-2 enable
		tci_control(ts, TAP_COUNT_CTRL2, (u8)ts->lpwgSetting.tapCount); // tap count = "user_setting"
		tci_control(ts, MIN_INTERTAP_CTRL2, 0);		// min inter_tap = 0ms
		tci_control(ts, MAX_INTERTAP_CTRL2, 70);	// max inter_tap = 700ms
		tci_control(ts, TOUCH_SLOP_CTRL2, 100);		// touch_slop = 10mm
		tci_control(ts, TAP_DISTANCE_CTRL2, 255);	// tap distance = MAX
		tci_control(ts, INTERRUPT_DELAY_CTRL2, 0);	// interrupt delay = 0ms

		tci_control(ts, REPORT_MODE_CTRL, 1);		// wakeup_gesture_only
		break;
	case T_REPORT_OFF:
		tci_control(ts, TCI_ENABLE_CTRL, 0);		// tci-1 disable
		tci_control(ts, TCI_ENABLE_CTRL2, 0);		// tci-2 disable
		sleep_control(ts, 0, 0);
		break;
	default:
		LGTC_ERR("Impossible Report Mode ( %d )\n", ts->reportMode);
		break;
	}

	DO_SAFE(touch_i2c_write_byte(ts->client, INTERRUPT_ENABLE_REG, buf), error);

	return NO_ERROR;
error:
	return ERROR;
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
		len += sprintf(str+len, "ELK + ");
		break;
	case 1:
		len += sprintf(str+len, "Suntel + ");
		break;
	case 2:
		len += sprintf(str+len, "Tovis + ");
		break;
	case 3:
		len += sprintf(str+len, "Innotek + ");
		break;
	case 4:
		len += sprintf(str+len, "JDI + ");
		break;
	default:
		len += sprintf(str+len, "Unknown + ");
		break;
	}

	switch ((product[1] & 0xF0) >> 4) {
	case 0:
		len += sprintf(str+len, "Synaptics");
		break;
	default:
		len += sprintf(str+len, "Unknown");
		break;
	}

	inch[0] = (product[1] & 0x0F);
	inch[1] = ((product[2] & 0xF0) >> 4);
	len += sprintf(str+len, "[%d.%d]", inch[0], inch[1]);

	len += sprintf(str+len, "\n");

	switch (product[0] & 0x0F) {
	case 0:
		len += sprintf(str+len, "[0key]");
		break;
	case 2:
		len += sprintf(str+len, "[2Key]");
		break;
	case 3:
		len += sprintf(str+len, "[3Key]");
		break;
	case 4:
		len += sprintf(str+len, "[4Key]");
		break;
	default:
		len += sprintf(str+len, "[Unknown]");
		break;
	}

	len += sprintf(str+len, "\n");

	paneltype = (product[2] & 0x0F);
	len += sprintf(str+len, "PanelType [%d] ", paneltype);

	len += sprintf(str+len, "\n");

	version[0] = ((product[3] & 0x80) >> 7);
	version[1] = (product[3] & 0x7F);
	len += sprintf(str+len, "version : v%d.%02d\n", version[0], version[1]);

	return str;
}

#if defined ( USE_OVER_TAP_COUNT_DETECTION_TIMER )
extern void send_uevent_lpwg(struct i2c_client *client, int type);

static void lpwg_timer_func(struct work_struct *work_timer)
{
	struct synaptics_ts_data *ts = container_of(to_delayed_work(work_timer),
		struct synaptics_ts_data, work_timer);

	send_uevent_lpwg(ts->client, LPWG_PASSWORD);

	return;
}
#endif


/****************************************************************************
* Global Functions
****************************************************************************/
int synaptics_ts_rmidev_function(struct synaptics_ts_exp_fn *rmidev_fn, bool insert)
{
	rmidev_fhandler.inserted = insert;

	if (insert)
		rmidev_fhandler.exp_fn = rmidev_fn;
	else
		rmidev_fhandler.exp_fn = NULL;

	return NO_ERROR;
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

	ret = sprintf(buf, "V%d.%02d (0x%X, 0x%X, 0x%X, 0x%X)\n",
			(ts->synaptics_fw_info.fw_version[3]&0x80 ? 1 : 0),
			ts->synaptics_fw_info.fw_version[3]&0x7F,
			ts->synaptics_fw_info.fw_version[0], ts->synaptics_fw_info.fw_version[1],
			ts->synaptics_fw_info.fw_version[2], ts->synaptics_fw_info.fw_version[3]);

	return ret;
}

static ssize_t show_firmware(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts =
			(struct synaptics_ts_data *)get_touch_handle(client);
	int ret = 0;

	ret = sprintf(buf, "\n======== Firmware Info ========\n");

	ret += sprintf(buf+ret,
		"ic_fw_version RAW = %02X %02X %02X %02X\n",
		ts->synaptics_fw_info.fw_version[0], ts->synaptics_fw_info.fw_version[1],
		ts->synaptics_fw_info.fw_version[2], ts->synaptics_fw_info.fw_version[3]);

	ret += sprintf(buf+ret, "=== ic_fw_version info === \n%s",
		productcode_parse(ts->synaptics_fw_info.fw_version));

	ret += sprintf(buf+ret, "IC_product_id[%s]\n\n",
		ts->fw_info->fw_ic_product_id);

	ret += sprintf(buf+ret,
		"img_fw_version RAW = %02X %02X %02X %02X\n",
		ts->synaptics_fw_info.fw_image_version[0],
		ts->synaptics_fw_info.fw_image_version[1],
		ts->synaptics_fw_info.fw_image_version[2],
		ts->synaptics_fw_info.fw_image_version[3]);

	ret += sprintf(buf+ret, "=== img_fw_version info === \n%s",
		productcode_parse(ts->synaptics_fw_info.fw_image_version));

	ret += sprintf(buf+ret, "Img_product_id[%s]\n",
		ts->fw_info->fw_bin_product_id);

	return ret;
}

static ssize_t store_tci(struct i2c_client *client,
	const char *buf, size_t count)
{
	struct synaptics_ts_data *ts
		= (struct synaptics_ts_data *)get_touch_handle(client);
	u32 type = 0, tci_num = 0, value = 0;

	sscanf(buf, "%d %d %d", &type, &tci_num, &value);

	tci_control(ts, type, (u8)value);

	return count;
}

static ssize_t show_tci(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts
		= (struct synaptics_ts_data *)get_touch_handle(client);

	int ret = 0;
	int i = 0;
	u8 buffer[7] = {0};

	touch_i2c_read(client, FINGER_REPORT_REG, 3, buffer);
	ret += sprintf(buf+ret, "report_mode [%s]\n",
		(buffer[2] & 0x3) == 0x2 ? "WAKEUP_ONLY" : "NORMAL");
	touch_i2c_read(client, WAKEUP_GESTURE_ENABLE_REG, 1, buffer);
	ret += sprintf(buf+ret, "wakeup_gesture [%d]\n\n", buffer[0]);

	for (i = 0; i < 2; i++) {
		synaptics_ts_page_data_read(client, LPWG_PAGE,
			LPWG_TAPCOUNT_REG + (i * LPWG_BLOCK_SIZE), 7, buffer);
		ret += sprintf(buf+ret, "TCI - %d\n", i+1);
		ret += sprintf(buf+ret, "TCI [%s]\n",
			(buffer[0] & 0x1) == 1 ? "enabled" : "disabled");
		ret += sprintf(buf+ret, "Tap Count [%d]\n",
			(buffer[0] & 0xf8) >> 3);
		ret += sprintf(buf+ret, "Min InterTap [%d]\n", buffer[1]);
		ret += sprintf(buf+ret, "Max InterTap [%d]\n", buffer[2]);
		ret += sprintf(buf+ret, "Touch Slop [%d]\n", buffer[3]);
		ret += sprintf(buf+ret, "Tap Distance [%d]\n", buffer[4]);
		ret += sprintf(buf+ret, "Interrupt Delay [%d]\n\n", buffer[6]);
	}

	return ret;
}

static ssize_t store_reg_ctrl(struct i2c_client *client,
	const char *buf, size_t count)
{
	u8 buffer[50] = {0};
	char command[6] = {0};
	int page = 0;
	int reg = 0;
	int offset = 0;
	int value = 0;

	sscanf(buf, "%s %d %d %d %d ", command, &page, &reg, &offset, &value);

	if (!strcmp(command, "write")) {
		synaptics_ts_page_data_read(client, page,
			reg, offset+1, buffer);
		buffer[offset] = (u8)value;
		synaptics_ts_page_data_write(client, page,
			reg, offset+1, buffer);
	} else if (!strcmp(command, "read")) {
		synaptics_ts_page_data_read(client, page,
			reg, offset+1, buffer);
		LGTC_DBG("page[%d] reg[%d] offset[%d] = 0x%x\n",
			page, reg, offset, buffer[offset]);
	} else {
		LGTC_DBG("Usage\n");
		LGTC_DBG("Write page reg offset value\n");
		LGTC_DBG("Read page reg offset\n");
	}
	return count;
}

static ssize_t show_object_report(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts
		= (struct synaptics_ts_data *)get_touch_handle(client);

	u8 object_report_enable_reg;
	u8 temp[8];

	int ret = 0;
	int i;

	DO_SAFE(touch_i2c_read(client, OBJECT_REPORT_ENABLE_REG,
		sizeof(object_report_enable_reg), &object_report_enable_reg),
		error);

	for (i = 0; i < 8; i++)
		temp[i] = (object_report_enable_reg >> i) & 0x01;

	ret = sprintf(buf,
		"\n======= read object_report_enable register =======\n");
	ret += sprintf(buf+ret,
		" Addr Bit7 Bit6 Bit5 Bit4 Bit3 Bit2 Bit1 Bit0 HEX\n");
	ret += sprintf(buf+ret,
		"--------------------------------------------------\n");
	ret += sprintf(buf+ret,
		" 0x%02X %4d %4d %4d %4d %4d %4d %4d %4d 0x%02X\n",
		OBJECT_REPORT_ENABLE_REG, temp[7], temp[6],
		temp[5], temp[4], temp[3], temp[2], temp[1], temp[0],
		object_report_enable_reg);
	ret += sprintf(buf+ret,
		"--------------------------------------------------\n");
	ret += sprintf(buf+ret,
		" Bit0  : [F]inger -> %7s\n", temp[0] ? "Enable" : "Disable");
	ret += sprintf(buf+ret,
		" Bit1  : [S]tylus -> %7s\n", temp[1] ? "Enable" : "Disable");
	ret += sprintf(buf+ret,
		" Bit2  : [P]alm -> %7s\n", temp[2] ? "Enable" : "Disable");
	ret += sprintf(buf+ret,
		" Bit3  : [U]nclassified Object -> %7s\n",
		temp[3] ? "Enable" : "Disable");
	ret += sprintf(buf+ret,
		" Bit4  : [H]overing Finger -> %7s\n",
		temp[4] ? "Enable" : "Disable");
	ret += sprintf(buf+ret,
		" Bit5  : [G]loved Finger -> %7s\n",
		temp[5] ? "Enable" : "Disable");
	ret += sprintf(buf+ret,
		" Bit6  : [N]arrow Object Swipe -> %7s\n",
		temp[6] ? "Enable" : "Disable");
	ret += sprintf(buf+ret,
		" Bit7  : Hand[E]dge  -> %7s\n",
		temp[7] ? "Enable" : "Disable");
	ret += sprintf(buf+ret,
		"==================================================\n\n");
error:
	return ret;
}

static ssize_t store_object_report(struct i2c_client *client,
	const char *buf, size_t count)
{
	struct synaptics_ts_data *ts
		= (struct synaptics_ts_data *)get_touch_handle(client);

	char select[16];
	u8 value = 2;
	int select_cnt;
	int i;
	u8 bit_select = 0;
	u8 object_report_enable_reg_old;
	u8 object_report_enable_reg_new;

	sscanf(buf, "%s %hhu", select, &value);

	if ((strlen(select) > 8) || (value > 1)) {
		LGTC_DBG("<writing object_report guide>\n");
		LGTC_DBG("echo [select] [value] > object_report\n");
		LGTC_DBG("select: [F]inger, [S]tylus, [P]alm,"
			" [U]nclassified Object, [H]overing Finger,"
			" [G]loved Finger, [N]arrow Object Swipe,"
			" Hand[E]dge\n");
		LGTC_DBG("select length: 1~8, value: 0~1\n");
		LGTC_DBG("ex) echo F 1 > object_report        "
			" (enable [F]inger)\n");
		LGTC_DBG("ex) echo s 1 > object_report        "
			" (enable [S]tylus)\n");
		LGTC_DBG("ex) echo P 0 > object_report        "
			" (disable [P]alm)\n");
		LGTC_DBG("ex) echo u 0 > object_report        "
			" (disable [U]nclassified Object)\n");
		LGTC_DBG("ex) echo HgNe 1 > object_report     "
			" (enable [H]overing Finger, [G]loved Finger,"
			" [N]arrow Object Swipe, Hand[E]dge)\n");
		LGTC_DBG("ex) echo eNGh 1 > object_report     "
			" (enable Hand[E]dge, [N]arrow Object Swipe,"
			" [G]loved Finger, [H]overing Finger)\n");
		LGTC_DBG("ex) echo uPsF 0 > object_report     "
			" (disable [U]nclassified Object, [P]alm,"
			" [S]tylus, [F]inger)\n");
		LGTC_DBG("ex) echo HguP 0 > object_report     "
			" (disable [H]overing Finger, [G]loved Finger,"
			" [U]nclassified Object, [P]alm)\n");
		LGTC_DBG("ex) echo HFnuPSfe 1 > object_report "
			" (enable all object)\n");
		LGTC_DBG("ex) echo enghupsf 0 > object_report "
			" (disbale all object)\n");
	} else {
		select_cnt = strlen(select);

		for (i = 0; i < select_cnt; i++) {
			switch ((char)(*(select + i))) {
			case 'F': case 'f': /* (F)inger */
				bit_select |= (0x01 << 0);
				break;
			case 'S': case 's': /* (S)tylus */
				bit_select |= (0x01 << 1);
				break;
			case 'P': case 'p': /* (P)alm */
				bit_select |= (0x01 << 2);
				break;
			case 'U': case 'u': /* (U)nclassified Object */
				bit_select |= (0x01 << 3);
				break;
			case 'H': case 'h': /* (H)overing Filter */
				bit_select |= (0x01 << 4);
				break;
			case 'G': case 'g': /* (G)loved Finger */
				bit_select |= (0x01 << 5);
				break;
			case 'N': case 'n': /* (N)arrow Ojbect Swipe */
				bit_select |= (0x01 << 6);
				break;
			case 'E': case 'e': /* Hand (E)dge */
				bit_select |= (0x01 << 7);
				break;
			default:
				break;
			}
		}

		DO_SAFE(touch_i2c_read(client, OBJECT_REPORT_ENABLE_REG,
			sizeof(object_report_enable_reg_old),
			&object_report_enable_reg_old), error);

		object_report_enable_reg_new = object_report_enable_reg_old;

		if (value > 0)
			object_report_enable_reg_new |= bit_select;
		else
			object_report_enable_reg_new &= ~(bit_select);

		DO_SAFE(touch_i2c_write_byte(client,
			OBJECT_REPORT_ENABLE_REG,
			object_report_enable_reg_new), error);
	}

error:
	return count;
}

static ssize_t show_use_rmi_dev(struct i2c_client *client, char *buf)
{
	int ret = 0;

	ret = sprintf(buf, "%u\n", enable_rmi_dev);

	return ret;
}

static ssize_t store_use_rmi_dev(struct i2c_client *client, const char *buf, size_t count)
{
	struct synaptics_ts_data *ts
		= (struct synaptics_ts_data *)get_touch_handle(client);
	int value = 0;

	sscanf(buf, "%d", &value);

	if (value < 0 || value > 1) {
		LGTC_DBG("Invalid enable_rmi_dev value:%d\n", value);
		return count;
	}

	enable_rmi_dev = value;
	LGTC_DBG("enable_rmi_dev:%u\n", enable_rmi_dev);

	if (enable_rmi_dev && rmidev_fhandler.inserted) {
		if (!rmidev_fhandler.initialized) {
			DO_SAFE(rmidev_fhandler.exp_fn->init(ts), error);
			rmidev_fhandler.initialized = true;
		}
	}
	else {
		rmidev_fhandler.exp_fn->remove(ts);
		rmidev_fhandler.initialized = false;
	}

	return count;

error:
	LGTC_ERR("fail to enable_rmi_dev\n");
	return ERROR;
}

static LGE_TOUCH_ATTR(firmware, S_IRUGO | S_IWUSR, show_firmware, NULL);
static LGE_TOUCH_ATTR(fw_ver, S_IRUGO | S_IWUSR, show_atcmd_fw_ver, NULL);
static LGE_TOUCH_ATTR(tci, S_IRUGO | S_IWUSR, show_tci, store_tci);
static LGE_TOUCH_ATTR(reg_ctrl, S_IRUGO | S_IWUSR, NULL, store_reg_ctrl);
static LGE_TOUCH_ATTR(object_report, S_IRUGO | S_IWUSR, show_object_report, store_object_report);
static LGE_TOUCH_ATTR(enable_rmi_dev, S_IRUGO | S_IWUSR, show_use_rmi_dev, store_use_rmi_dev);

static struct attribute *lge_specific_touch_attribute_list[] = {
	&lge_touch_attr_firmware.attr,
	&lge_touch_attr_fw_ver.attr,
	&lge_touch_attr_tci.attr,
	&lge_touch_attr_reg_ctrl.attr,
	&lge_touch_attr_object_report.attr,
	&lge_touch_attr_enable_rmi_dev.attr,
	NULL,
};

error_type synaptics_ts_probe(struct i2c_client *client, struct touch_fw_info* fw_info, struct attribute ***attribute_list)
{
	struct synaptics_ts_data *ts;

	LGTC_FUN();

	ASSIGN(ts = devm_kzalloc(&client->dev,
		sizeof(struct synaptics_ts_data), GFP_KERNEL), error);
	set_touch_handle(client, ts);

	ts->client = client;
	ts->fw_info = fw_info;

	*attribute_list = lge_specific_touch_attribute_list;

	#if defined ( USE_OVER_TAP_COUNT_DETECTION_TIMER )
	INIT_DELAYED_WORK(&ts->work_timer, lpwg_timer_func);
	#endif

	read_page_description_table(ts->client);

	get_fw_bin_info(ts);
	get_fw_ic_info(ts);

	DO_SAFE(check_firmware_status(ts), need_to_rewrite);

	return NO_ERROR;

need_to_rewrite:
	ts->fw_info->fw_force_upgrade = 1;
error:
	LGTC_ERR("Synaptics Probe Failed!!\n");
	return ERROR;
}

error_type synaptics_ts_remove(struct i2c_client *client)
{
	struct synaptics_ts_data *ts
		= (struct synaptics_ts_data *)get_touch_handle(client);

	LGTC_FUN();

	kfree(ts);

	return NO_ERROR;
}

//==========================================================
// Touch IC를 초기화 한다.
// 초기화에 필요한 전원 및 Reset PIN의 컨트롤을 포함한다.
// 함수 수행 이후에는, Normal Mode에서의 정상적인 Touch 동작이 보장되어야 한다.
// Touch IC로 부터 읽어야 하는 정보 ( Firwware 버전 등 ) 를 반환 한다. ( TBD : 반환 Data 정리 필요 )
//==========================================================
error_type synaptics_ts_init(struct i2c_client *client)
{
	struct synaptics_ts_data *ts
		= (struct synaptics_ts_data *)get_touch_handle(client);
	u8 buf = 0;
	u8 buf_array[2] = {0};

	LGTC_FUN();

	DO_SAFE(synaptics_ts_page_data_write_byte(client, ANALOG_PAGE,
		DYNAMIC_SENSING_CONTROL_REG, SENSING_CONTROL_120HZ), error);

	TouchReset();

	DO_SAFE(synaptics_ts_page_data_write_byte(client, ANALOG_PAGE,
		DYNAMIC_SENSING_CONTROL_REG, SENSING_CONTROL_AUTO), error);

	DO_SAFE(touch_i2c_write_byte(client, DEVICE_CONTROL_REG,
		DEVICE_CONTROL_NOSLEEP | DEVICE_CONTROL_CONFIGURED), error);

	DO_SAFE(touch_i2c_read(client, INTERRUPT_ENABLE_REG, 1, &buf), error);
	DO_SAFE(touch_i2c_write_byte(client, INTERRUPT_ENABLE_REG,
		buf | INTERRUPT_MASK_ABS0), error);

	buf_array[0] = buf_array[1] = 0;

	DO_SAFE(touch_i2c_write(client, FINGER_REPORT_REG, 2, buf_array), error);

	/* It always should be done last. */
	DO_SAFE(touch_i2c_read(client, INTERRUPT_STATUS_REG, 1, &buf), error);

	ts->reportMode = T_REPORT_NORMAL;

	return NO_ERROR;
error:
	LGTC_ERR("Fail to initialise Touch IC\n");
	return ERROR;
}

//==========================================================
// Touch Interrupt가 발생하면 WorkQueue내에서 호출되는 함수
// Interrupt가 발생된 원인을 확인하고 적절한 후속처리를 한다.
// Event처리를 위한 Data도 반환한다. ( TBD : Data구조체 정의 필요 )
// 함수가 호출된 후에는, Interrupt가 해제된 상태여야 한다.
//==========================================================
error_type synaptics_ts_isr(struct i2c_client *client, TouchReadData *pData)
{
	struct synaptics_ts_data *ts
		= (struct synaptics_ts_data *)get_touch_handle(client);

	u8  i = 0;
	TouchFingerData *pFingerData = NULL;

	u8 regDevStatus = 0;
	u8 regIntStatus = 0;
	u8 regFingerData[MAX_NUM_OF_FINGERS][NUM_OF_EACH_FINGER_DATA_REG];

	pData->type = NUM_OF_DATA_TYPE;
	pData->count = 0;

	touch_i2c_read(client, DEVICE_STATUS_REG, sizeof(regDevStatus), &regDevStatus);

	touch_i2c_read(client, INTERRUPT_STATUS_REG, sizeof(regIntStatus), &regIntStatus);

	if (regIntStatus == INTERRUPT_MASK_NONE) {
		return NO_ERROR;
	} else if (regIntStatus & INTERRUPT_MASK_ABS0 ) {
		touch_i2c_read(client, FINGER_DATA_REG_START,
			(NUM_OF_EACH_FINGER_DATA_REG * MAX_NUM_OF_FINGERS),
			(u8 *)regFingerData);

		for (i = 0; i < MAX_NUM_OF_FINGERS; i++) {
			if (regFingerData[i][0] & 0x03) {
				pFingerData = &pData->fingerData[pData->count];
				pFingerData->id = i;
				pFingerData->type = regFingerData[i][REG_OBJECT];
				pFingerData->x = GET_X_POSITION( regFingerData[i][REG_X_MSB], regFingerData[i][REG_X_LSB] );
				pFingerData->y = GET_Y_POSITION( regFingerData[i][REG_Y_MSB], regFingerData[i][REG_Y_LSB] );
				pFingerData->width_major = GET_WIDTH_MAJOR( regFingerData[i][REG_WX], regFingerData[i][REG_WY] );
				pFingerData->width_minor = GET_WIDTH_MINOR( regFingerData[i][REG_WX], regFingerData[i][REG_WY] );
				pFingerData->orientation = GET_ORIENTATION( regFingerData[i][REG_WY], regFingerData[i][REG_WX] );
				pFingerData->pressure = GET_PRESSURE( regFingerData[i][REG_Z] );

				pData->count++;
			}
		}

		pData->type = T_DATA_FINGER;

		#if defined ( USE_OVER_TAP_COUNT_DETECTION_TIMER )
		if ( ts->reportMode == T_REPORT_KNOCK_ON_CODE ) {
			pData->type = T_DATA_KNOCK_CODE;

			if (cancel_delayed_work(&ts->work_timer)) {
				pData->count = 2;

				pData->knockData[0].x = 1;
				pData->knockData[0].y = 1;
				pData->knockData[1].x = -1;
				pData->knockData[1].y = -1;

				queue_delayed_work(touch_wq, &ts->work_timer, msecs_to_jiffies(200));
			}

			return IGNORE_EVENT_BUT_SAVE_IT;
		}
		#endif

	} else if (regIntStatus & INTERRUPT_MASK_CUSTOM) {
		u8 status = 0;
		synaptics_ts_page_data_read(client, LPWG_PAGE, LPWG_STATUS_REG, 1, &status);

		if (status & 0x1) { /* TCI-1 : Double-Tap */
			LGTC_LOG( "Knock-on Detected\n" );
			pData->type = T_DATA_KNOCK_ON;
			get_lpwg_data(ts, 2);
		} else if (status & 0x2) {  /* TCI-2 : Multi-Tap */
			LGTC_LOG( "Knock-code Detected\n" );
			pData->type = T_DATA_KNOCK_CODE;
			get_lpwg_data(ts, ts->lpwgSetting.tapCount);
			memcpy(&pData->knockData, &ts->pw_data.data, sizeof(struct point)*ts->pw_data.data_num);
			pData->count = ts->pw_data.data_num;

			#if defined ( USE_OVER_TAP_COUNT_DETECTION_TIMER )
			ts->reportMode = T_REPORT_NORMAL;
			lpwg_control(ts);
			queue_delayed_work(touch_wq, &ts->work_timer, msecs_to_jiffies(180));
			return IGNORE_EVENT;
			#endif
		} else {
			LGTC_ERR( "Unexpected LPWG Interrupt Status ( 0x%X )\n", status);
		}

	} else {
		LGTC_ERR( "Unexpected Interrupt Status ( 0x%X )\n", regIntStatus );
	}

	return NO_ERROR;
}


error_type synaptics_ts_ic_ctrl(struct i2c_client *client,
	u8 code, u32 value, u32 *ret)
{
	u8 buf = 0;

	switch (code) {
	case IC_CTRL_READ:
		DO_SAFE(touch_i2c_read(client, value, 1, &buf), error);
		*ret = (u32)buf;
		break;
	case IC_CTRL_WRITE:
		DO_SAFE(touch_i2c_write_byte(client,
			((value & 0xFF00) >> 8), (value & 0xFF)), error);
		break;
	default:
		break;
	}

	return NO_ERROR;
error:
	return ERROR;
}


//==========================================================
// 인자로 전달된 Firmware를 TouchIC에 Write한다.
// Firmware를 Write하기 전에 필요한 사전처리는 함수 호출전 준비되는 것으로 가정한다.
// Write이후의 Verify도 처리한 후 결과를 반환한다.
// Touch가 정상동작하기 위한 후속처리는 고려하지 않는다.
//==========================================================
error_type synaptics_ts_fw_upgrade(struct i2c_client *client)
{
	struct synaptics_ts_data *ts
		= (struct synaptics_ts_data *)get_touch_handle(client);

	static int rewrite_cnt = 0;

	if(rewrite_cnt > 3) {
		rewrite_cnt = 0;

		return ERROR;
	}

	synaptics_ts_page_data_write_byte(client, ANALOG_PAGE,
		DYNAMIC_SENSING_CONTROL_REG, SENSING_CONTROL_120HZ);

	/* During upgrading, interrupt will be ignored. */
	FirmwareUpgrade(ts, ts->fw_info->fw_path);

	read_page_description_table(ts->client);
	get_fw_ic_info(ts);

	DO_SAFE(check_firmware_status(ts), need_to_rewrite);

	return NO_ERROR;

need_to_rewrite:
	LGTC_ERR( "error : Firmware update Failed!!\n" );

	rewrite_cnt++;

	LGTC_DBG( "retry fw upgrade[%d]\n", rewrite_cnt);
	synaptics_ts_fw_upgrade(ts->client);

	return ERROR;
}

//==========================================================
// 사용될 가능성은 적지만, 혹시나 몰라서 일단 남겨 둠
//==========================================================
error_type synaptics_ts_suspend(struct i2c_client *client)
{
	LGTC_FUN();

	return NO_ERROR;
}

//==========================================================
// 사용될 가능성은 적지만, 혹시나 몰라서 일단 남겨 둠
//==========================================================
error_type synaptics_ts_resume(struct i2c_client *client)
{
	LGTC_FUN();

	return NO_ERROR;
}

//==========================================================
// LPGW Command를 처리하는 함수
// 중복된 Command가 오는 것에 대한 방어 처리를 하면 더 좋을 것임.
//==========================================================
error_type synaptics_ts_lpwg(struct i2c_client *client, E_TouchReportMode reportMode, LGTcLpwgSetting  *pLpwgSetting)
{
	struct synaptics_ts_data *ts
		= (struct synaptics_ts_data *)get_touch_handle(client);

	int result = ERROR;

	LGTC_FUN();

	memcpy(&ts->lpwgSetting, pLpwgSetting, sizeof(LGTcLpwgSetting));

	if( ts->reportMode != reportMode ) {
		ts->reportMode = reportMode;
		result = lpwg_control(ts);
	}

	return result;
}

error_type synaptics_f54_test(struct i2c_client *client,
	char* buf, int* raw_status, int* ch_status)
{
	struct synaptics_ts_data *ts
		= (struct synaptics_ts_data *)get_touch_handle(client);

	int full_raw_cap;
	int trx_to_trx;
	int high_resistance;

	DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, ANALOG_PAGE), error);

	full_raw_cap = F54TestHandle(ts, F54_FULL_RAW_CAP, 0, buf);
	high_resistance = F54TestHandle(ts, F54_HIGH_RESISTANCE, 0, buf);
	trx_to_trx = F54TestHandle(ts, F54_TRX_TO_TRX, 0, buf);

	*raw_status = full_raw_cap;

	if (high_resistance == LGTC_SUCCESS && trx_to_trx == LGTC_SUCCESS)
		*ch_status = LGTC_SUCCESS;
	else
		*ch_status = LGTC_FAIL;

	synaptics_ts_init(client);

	return NO_ERROR;

error:
	return ERROR;
}

struct touch_device_driver synaptics_ts_driver = {
	.probe		= synaptics_ts_probe, /* Phone Power ON시 한 번만 수행하는 초기화 코드 */
	.remove		= synaptics_ts_remove, /* Phone Power OFF시 한 번만 수행하는 마무리 코드 */
	.suspend	= synaptics_ts_suspend, /* Suspend 진입에 필요한 처리를 수행하는 코드 */
	.resume		= synaptics_ts_resume, /* Resume시 필요한 처리를 수행하는 코드 */
	.init  		= synaptics_ts_init, /* Touch IC Reset이후 필요한 초기화 코드 */
	.isr  		= synaptics_ts_isr, /* Interrupt 발생시 필요한 Data를 읽기위한 코드 ( ISR ) */
	.ic_ctrl	= synaptics_ts_ic_ctrl,
	.fw_upgrade 	= synaptics_ts_fw_upgrade,
	.lpwg		= synaptics_ts_lpwg, /* Touch IC에 Command를 전달하기 위한 코드 ( LPWG / Debugging / ... ) ==> 너무 비대해지지 않도록 분리 검토 필요 */
	.sd			= synaptics_f54_test
};

/* End Of File */


