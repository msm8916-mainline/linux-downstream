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
#include <linux/workqueue.h>
#include <linux/wakelock.h>

#include <linux/input/lge_touch_core.h>
#include <linux/input/touch_synaptics.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>

#include "lge_touch_platform.h"
#include "touch_synaptics_i2c.h"
#include "DS5/RefCode_F54.h"

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
#define ANALOG_CONTROL				0x54
#define SENSOR_CONTROL				0x55

/* Register Map & Register bit mask
 * - Please check "One time" this map before using this device driver
 */
#define MANUFACTURER_ID_REG		(ts->common_fc.dsc.query_base)
#define CUSTOMER_FAMILY_REG		(ts->common_fc.dsc.query_base+2)
#define FW_REVISION_REG			(ts->common_fc.dsc.query_base+3)
#define PRODUCT_ID_REG			(ts->common_fc.dsc.query_base+11)

#define DEVICE_COMMAND_REG		(ts->common_fc.dsc.command_base)

#define DEVICE_CONTROL_REG 				(ts->common_fc.dsc.control_base)
#define DEVICE_CONTROL_NORMAL_OP		0x00
#define DEVICE_CONTROL_SLEEP 			0x01
#define DEVICE_CONTROL_SLEEP_NO_RECAL	0x02
#define DEVICE_CONTROL_NOSLEEP			0x04
#define DEVICE_CHARGER_CONNECTED		0x20
#define DEVICE_CONTROL_CONFIGURED		0x80

#define INTERRUPT_ENABLE_REG			(ts->common_fc.dsc.control_base+1)
#define DOZE_INTERVAL_REG				(ts->common_fc.dsc.control_base+2)	/* Doze Interval : unit 10ms */
#define DOZE_WAKEUP_THRESHOLD_REG		(ts->common_fc.dsc.control_base+3)


#define DEVICE_STATUS_REG			(ts->common_fc.dsc.data_base)
#define DEVICE_FAILURE_MASK			0x03
#define DEVICE_CRC_ERROR_MASK		0x04
#define DEVICE_STATUS_FLASH_PROG	0x40
#define DEVICE_STATUS_UNCONFIGURED	0x80

#define INTERRUPT_STATUS_REG		(ts->common_fc.dsc.data_base+1)
#define INTERRUPT_MASK_FLASH		0x01
#define INTERRUPT_MASK_ABS0			0x04
#define INTERRUPT_MASK_BUTTON		0x10
#define INTERRUPT_MASK_CUSTOM		0x40

/* TOUCHPAD_SENSORS */
#define FINGER_COMMAND_REG		(ts->finger_fc.dsc.command_base)
#define FINGER_REPORT_REG		(ts->finger_fc.dsc.control_base+5)
#define OBJECT_REPORT_ENABLE_REG	(ts->finger_fc.dsc.control_base+8)
//#define WAKEUP_GESTURE_ENABLE_REG	(ts->finger_fc.dsc.control_base+11)

#define OBJECT_TYPE_AND_STATUS_REG	(ts->finger_fc.dsc.data_base)
#define OBJECT_ATTENTION_REG		(ts->finger_fc.dsc.data_base+2)
#define FINGER_DATA_REG_START		(ts->finger_fc.dsc.data_base)

#define REG_OBJECT			0
#define REG_X_LSB			1
#define REG_X_MSB			2
#define REG_Y_LSB			3
#define REG_Y_MSB			4
#define REG_Z				5
#define REG_WX				6
#define REG_WY				7

#define MAXIMUM_XY_COORDINATE_REG	(ts->finger_fc.dsc.control_base)

#define ANALOG_COMMAND_REG		(ts->analog_fc.dsc.command_base)
#define ANALOG_CONTROL_REG		(ts->analog_fc.dsc.control_base)

#define FLASH_CONFIG_ID_REG		(ts->flash_fc.dsc.control_base)
#define FLASH_CONTROL_REG		(ts->flash_fc.dsc.data_base+2)
#define FLASH_STATUS_REG		(ts->flash_fc.dsc.data_base+3)
#define FLASH_STATUS_MASK		0xFF

#define COMMON_PAGE				(ts->common_fc.function_page)
#define FINGER_PAGE				(ts->finger_fc.function_page)
#define ANALOG_PAGE				(ts->analog_fc.function_page)
#define FLASH_PAGE				(ts->flash_fc.function_page)
#define SENSOR_PAGE				(ts->sensor_fc.function_page)
#define LPWG_PAGE				0x04

#define LPWG_STATUS_REG				0x00
#define LPWG_DATA_REG				0x01
#define LPWG_TAPCOUNT_REG			0x31
#define LPWG_MIN_INTERTAP_REG		0x32
#define LPWG_MAX_INTERTAP_REG		0x33
#define LPWG_TOUCH_SLOP_REG			0x34
#define LPWG_TAP_DISTANCE_REG		0x35
#define LPWG_INTERRUPT_DELAY_REG	0x37
#define LPWG_BLOCK_SIZE				7

#define LPWG_TAPCOUNT_REG2					0x38
#define LPWG_MIN_INTERTAP_REG2				0x39
#define LPWG_MAX_INTERTAP_REG2				0x3A
#define LPWG_TOUCH_SLOP_REG2				0x3B
#define LPWG_TAP_DISTANCE_REG2				0x3C
#define LPWG_INTERRUPT_DELAY_REG2			0x3E
#define WAKEUP_GESTURE_ENABLE_REG			0x20	/* f12_info.ctrl_reg_addr[27] */
#define MISC_HOST_CONTROL_REG				0x3F
#define THERMAL_HIGH_FINGER_AMPLITUDE		0x60	/* finger_amplitude(0x80) = 0.5 */

#define REPORT_MODE_CTRL		1
#define TCI_ENABLE_CTRL			2
#define TAP_COUNT_CTRL			3
#define MIN_INTERTAP_CTRL		4
#define MAX_INTERTAP_CTRL		5
#define TOUCH_SLOP_CTRL			6
#define TAP_DISTANCE_CTRL		7
#define INTERRUPT_DELAY_CTRL	8

#define TCI_ENABLE_CTRL2		22
#define TAP_COUNT_CTRL2			23
#define MIN_INTERTAP_CTRL2		24
#define MAX_INTERTAP_CTRL2		25
#define TOUCH_SLOP_CTRL2		26
#define TAP_DISTANCE_CTRL2		27
#define INTERRUPT_DELAY_CTRL2	28

#define PALM_TYPE				3
#define HOVER_TYPE				5
#define MAX_PRESSURE			255

#define UEVENT_DELAY			200
#define I2C_DELAY				20
#define KNOCKON_DELAY			68	/* 700ms */

/* Get user-finger-data from register.
 */
#define TS_SNTS_GET_X_POSITION(_msb_reg, _lsb_reg) \
	(((u16)((_msb_reg << 8)  & 0xFF00)  | (u16)((_lsb_reg) & 0xFF)))
#define TS_SNTS_GET_Y_POSITION(_msb_reg, _lsb_reg) \
	(((u16)((_msb_reg << 8)  & 0xFF00)  | (u16)((_lsb_reg) & 0xFF)))
#define TS_SNTS_GET_WIDTH_MAJOR(_width_x, _width_y) \
	((_width_x - _width_y) > 0) ? _width_x : _width_y
#define TS_SNTS_GET_WIDTH_MINOR(_width_x, _width_y) \
	((_width_x - _width_y) > 0) ? _width_y : _width_x

#define TS_SNTS_GET_ORIENTATION(_width_y, _width_x) \
	((_width_y - _width_x) > 0) ? 0 : 1
#define TS_SNTS_GET_PRESSURE(_pressure) \
		_pressure

#if defined(CONFIG_JDI_INCELL_VIDEO_HD_PANEL)
#define PWR_CONTROL_FROM_LCD
#endif

extern struct workqueue_struct *touch_wq;
static struct synaptics_ts_exp_fhandler rmidev_fhandler;
static char power_state = 0;

struct synaptics_ts_f12_info {
	bool ctrl_reg_is_present[32];
	bool data_reg_is_present[16];
	u8 ctrl_reg_addr[32];
	u8 data_reg_addr[16];
};

static struct synaptics_ts_f12_info f12_info;

int enable_rmi_dev = 0;

/* wrapper function for i2c communication - except defalut page
 * if you have to select page for reading or writing,
 * then using this wrapper function */

static int sleep_control(struct synaptics_ts_data *ts, int mode, int recal)
{
	u8 curr = 0;
	u8 next = 0;

	DO_SAFE(touch_i2c_read(ts->client, DEVICE_CONTROL_REG, 1, &curr), error);

	next = (curr & 0xFC) | (mode ? DEVICE_CONTROL_NORMAL_OP : DEVICE_CONTROL_SLEEP);
	DO_SAFE(touch_i2c_write_byte(ts->client, DEVICE_CONTROL_REG, next), error);

	TOUCH_DEBUG(DEBUG_BASE_INFO, "%s : curr = [%x] next[%x]\n", __func__, curr, next);

	return 0;
error:
	return -1;
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
static int tci_control(struct synaptics_ts_data *ts, int type, u8 value)
{
	struct i2c_client* client = ts->client;
	u8 buffer[3] = {0};

	switch (type) {
	case REPORT_MODE_CTRL:
		DO_SAFE(touch_i2c_read(ts->client, INTERRUPT_ENABLE_REG, 1, buffer), error);
		DO_SAFE(touch_i2c_write_byte(ts->client, INTERRUPT_ENABLE_REG,
				value ? buffer[0] & ~INTERRUPT_MASK_ABS0 : buffer[0] | INTERRUPT_MASK_ABS0), error);

		if (value) {
			buffer[0] = 0x29;
			buffer[1] = 0x29;
			DO_SAFE(touch_i2c_write(client, f12_info.ctrl_reg_addr[15], 2, buffer), error);
		}

		DO_SAFE(touch_i2c_read(client, f12_info.ctrl_reg_addr[20], 3, buffer), error);
		buffer[2] = (buffer[2] & 0xfc) | (value ? 0x2 : 0x0);
		DO_SAFE(touch_i2c_write(client, f12_info.ctrl_reg_addr[20], 3, buffer), error);
		TOUCH_DEBUG(DEBUG_BASE_INFO, "report mode: %d\n", value);
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

	return 0;
error:
	return -1;

}

void get_f12_info(struct synaptics_ts_data *ts)
{
	int retval;
	struct synaptics_ts_f12_query_5 query_5;
	struct synaptics_ts_f12_query_8 query_8;
	int i;
	u8 offset;

	if (!ts) {
		TOUCH_ERR_MSG("ts is null\n");
		return;
	}

	/* ctrl_reg_info setting */
	retval = touch_i2c_read(ts->client, (ts->finger_fc.dsc.query_base + 5), sizeof(query_5.data), query_5.data);

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
		f12_info.ctrl_reg_addr[i] = ts->finger_fc.dsc.control_base + offset;

		if (f12_info.ctrl_reg_is_present[i])
			offset++;
	}

/* data_reg_info setting */
	retval = touch_i2c_read(ts->client, (ts->finger_fc.dsc.query_base + 8), sizeof(query_8.data), query_8.data);

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
		f12_info.data_reg_addr[i] = ts->finger_fc.dsc.data_base + offset;

		if (f12_info.data_reg_is_present[i])
			offset++;
	}

/* print info */
	for (i = 0; i < 32; i++) {
		if (f12_info.ctrl_reg_is_present[i])
			TOUCH_INFO_MSG("f12_info.ctrl_reg_addr[%d]=0x%02X\n", i, f12_info.ctrl_reg_addr[i]);
	}

	for (i = 0; i < 16; i++) {
		if (f12_info.data_reg_is_present[i])
			TOUCH_INFO_MSG("f12_info.data_reg_addr[%d]=0x%02X\n", i, f12_info.data_reg_addr[i]);
	}

	return;
}

static int get_tci_data(struct synaptics_ts_data *ts, int count)
{
	struct i2c_client *client = ts->client;
	u8 i = 0;
	u8 buffer[12][4] = { {0} };

	ts->pw_data.data_num = count;

	if (!count)
		return 0;

	DO_SAFE(synaptics_ts_page_data_read(client, LPWG_PAGE,
		LPWG_DATA_REG, 4*count, &buffer[0][0]), error);

	for (i = 0; i < count; i++) {
		ts->pw_data.data[i].x
			= TS_SNTS_GET_X_POSITION(buffer[i][1], buffer[i][0]);
		ts->pw_data.data[i].y
			= TS_SNTS_GET_Y_POSITION(buffer[i][3], buffer[i][2]);
		TOUCH_DEBUG(DEBUG_BASE_INFO || DEBUG_LPWG,
			"LPWG data [%d, %d]\n",
			ts->pw_data.data[i].x, ts->pw_data.data[i].y);
	}

	return 0;
error:
	return -1;
}

static void set_lpwg_mode(struct lpwg_control *ctrl, int mode)
{
	ctrl->double_tap_enable
		= (mode & (LPWG_DOUBLE_TAP | LPWG_PASSWORD)) ? 1 : 0;
	ctrl->password_enable = (mode & LPWG_PASSWORD) ? 1 : 0;
	ctrl->signature_enable = (mode & LPWG_SIGNATURE) ? 1 : 0;
	ctrl->lpwg_is_enabled = ctrl->double_tap_enable
			|| ctrl->password_enable || ctrl->signature_enable;
}

static int lpwg_control(struct synaptics_ts_data *ts, int mode)
{
	u8 buf = 0;

	TOUCH_TRACE();

	set_lpwg_mode(&ts->lpwg_ctrl, mode);

	DO_SAFE(touch_i2c_read(ts->client, INTERRUPT_ENABLE_REG, 1, &buf), error);
	DO_SAFE(touch_i2c_write_byte(ts->client, INTERRUPT_ENABLE_REG, 0x00), error);

	switch (mode) {
	case LPWG_SIGNATURE:
		break;
	case LPWG_DOUBLE_TAP:					// Only TCI-1
		msleep(100);
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
	case LPWG_PASSWORD:					// TCI-1 and TCI-2
		tci_control(ts, TCI_ENABLE_CTRL, 1);		// tci-1 enable
		tci_control(ts, TAP_COUNT_CTRL, 2); 		// tap count = 2
		tci_control(ts, MIN_INTERTAP_CTRL, 0);		// min inter_tap = 0ms
		tci_control(ts, MAX_INTERTAP_CTRL, 70);		// max inter_tap = 700ms
		tci_control(ts, TOUCH_SLOP_CTRL, 100);		// touch_slop = 10mm
		tci_control(ts, TAP_DISTANCE_CTRL, 7);		// tap distance = 7mm
		tci_control(ts, INTERRUPT_DELAY_CTRL, (u8)ts->pw_data.double_tap_check);	// interrupt delay = 0ms

		tci_control(ts, TCI_ENABLE_CTRL2, 1);		// tci-2 enable
		tci_control(ts, TAP_COUNT_CTRL2, (u8)ts->pw_data.tap_count); // tap count = "user_setting"
		tci_control(ts, MIN_INTERTAP_CTRL2, 0);		// min inter_tap = 0ms
		tci_control(ts, MAX_INTERTAP_CTRL2, 70);	// max inter_tap = 700ms
		tci_control(ts, TOUCH_SLOP_CTRL2, 100);		// touch_slop = 10mm
		tci_control(ts, TAP_DISTANCE_CTRL2, 255);	// tap distance = MAX
		tci_control(ts, INTERRUPT_DELAY_CTRL2, 0);	// interrupt delay = 0ms

		tci_control(ts, REPORT_MODE_CTRL, 1);		// wakeup_gesture_only
		break;
	default:
		tci_control(ts, TCI_ENABLE_CTRL, 0);		// tci-1 disable
		tci_control(ts, TCI_ENABLE_CTRL2, 0);		// tci-2 disable
		tci_control(ts, REPORT_MODE_CTRL, 0);		// normal
		sleep_control(ts, 1, 0);
		break;
	}

	DO_SAFE(touch_i2c_write_byte(ts->client, INTERRUPT_ENABLE_REG, buf), error);

	TOUCH_DEBUG(DEBUG_BASE_INFO | DEBUG_LPWG, "lpwg_mode[%d]\n", mode);
	return NO_ERROR;
error:
	return ERROR;
}

int synaptics_ts_rmidev_function(struct synaptics_ts_exp_fn *rmidev_fn, bool insert)
{
	rmidev_fhandler.inserted = insert;

	if (insert)
		rmidev_fhandler.exp_fn = rmidev_fn;
	else
		rmidev_fhandler.exp_fn = NULL;

	return NO_ERROR;
}

static void lpwg_timer_func(struct work_struct *work_timer)
{
	struct synaptics_ts_data *ts = container_of(to_delayed_work(work_timer),
		struct synaptics_ts_data, work_timer);

//	sleep_control(ts, 0, 1); /* sleep until receive the reply. */
	send_uevent_lpwg(ts->client, LPWG_PASSWORD);
	wake_unlock(&ts->timer_wake_lock);

	TOUCH_DEBUG(DEBUG_LPWG, "u-event timer occur!\n");
	return;
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
		len += sprintf(str+len, "ELK ");
		break;
	case 1:
		len += sprintf(str+len, "Suntel ");
		break;
	case 2:
		len += sprintf(str+len, "Tovis ");
		break;
	case 3:
		len += sprintf(str+len, "Innotek ");
		break;
	default:
		len += sprintf(str+len, "Unknown ");
		break;
	}

	len += sprintf(str+len, "\n");

	switch (product[0] & 0x0F) {
	case 0:
		len += sprintf(str+len, "0key ");
		break;
	case 2:
		len += sprintf(str+len, "2Key ");
		break;
	case 3:
		len += sprintf(str+len, "3Key ");
		break;
	case 4:
		len += sprintf(str+len, "4Key ");
		break;
	default:
		len += sprintf(str+len, "Unknown ");
		break;
	}

	len += sprintf(str+len, "\n");

	switch ((product[1] & 0xF0) >> 4) {
	case 0:
		len += sprintf(str+len, "Synaptics ");
		break;
	default:
		len += sprintf(str+len, "Unknown ");
		break;
	}

	len += sprintf(str+len, "\n");

	inch[0] = (product[1] & 0x0F);
	inch[1] = ((product[2] & 0xF0) >> 4);
	len += sprintf(str+len, "%d.%d ", inch[0], inch[1]);

	len += sprintf(str+len, "\n");

	paneltype = (product[2] & 0x0F);
	len += sprintf(str+len, "PanelType %d ", paneltype);

	len += sprintf(str+len, "\n");

	version[0] = ((product[3] & 0x80) >> 7);
	version[1] = (product[3] & 0x7F);
	len += sprintf(str+len, "version : v%d.%02d\n", version[0], version[1]);

	return str;
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
			(ts->fw_info.fw_version[3]&0x80 ? 1 : 0),
			ts->fw_info.fw_version[3]&0x7F,
			ts->fw_info.fw_version[0], ts->fw_info.fw_version[1],
			ts->fw_info.fw_version[2], ts->fw_info.fw_version[3]);

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
		ts->fw_info.fw_version[0], ts->fw_info.fw_version[1],
		ts->fw_info.fw_version[2], ts->fw_info.fw_version[3]);

	ret += sprintf(buf+ret, "=== ic_fw_version info === \n%s",
		productcode_parse(ts->fw_info.fw_version));

	ret += sprintf(buf+ret, "IC_product_id[%s]\n",
		ts->fw_info.fw_product_id);

	ret += sprintf(buf+ret, "Touch IC : %s\n\n", ts->fw_info.fw_product_id);

	ret += sprintf(buf+ret,
		"img_fw_version RAW = %02X %02X %02X %02X\n",
		ts->fw_info.fw_image_version[0],
		ts->fw_info.fw_image_version[1],
		ts->fw_info.fw_image_version[2],
		ts->fw_info.fw_image_version[3]);

	ret += sprintf(buf+ret, "=== img_fw_version info === \n%s",
		productcode_parse(ts->fw_info.fw_image_version));

	ret += sprintf(buf+ret, "Img_product_id[%s]\n",
		ts->fw_info.fw_image_product_id);

	ret += sprintf(buf+ret, "Touch IC : %s\n\n",
		ts->fw_info.fw_image_product_id);

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
		TOUCH_DEBUG(DEBUG_BASE_INFO,
			"page[%d] reg[%d] offset[%d] = 0x%x\n",
			page, reg, offset, buffer[offset]);
	} else {
		TOUCH_DEBUG(DEBUG_BASE_INFO, "Usage\n");
		TOUCH_DEBUG(DEBUG_BASE_INFO, "Write page reg offset value\n");
		TOUCH_DEBUG(DEBUG_BASE_INFO, "Read page reg offset\n");
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
		TOUCH_INFO_MSG("<writing object_report guide>\n");
		TOUCH_INFO_MSG("echo [select] [value] > object_report\n");
		TOUCH_INFO_MSG("select: [F]inger, [S]tylus, [P]alm,"
			" [U]nclassified Object, [H]overing Finger,"
			" [G]loved Finger, [N]arrow Object Swipe,"
			" Hand[E]dge\n");
		TOUCH_INFO_MSG("select length: 1~8, value: 0~1\n");
		TOUCH_INFO_MSG("ex) echo F 1 > object_report        "
			" (enable [F]inger)\n");
		TOUCH_INFO_MSG("ex) echo s 1 > object_report        "
			" (enable [S]tylus)\n");
		TOUCH_INFO_MSG("ex) echo P 0 > object_report        "
			" (disable [P]alm)\n");
		TOUCH_INFO_MSG("ex) echo u 0 > object_report        "
			" (disable [U]nclassified Object)\n");
		TOUCH_INFO_MSG("ex) echo HgNe 1 > object_report     "
			" (enable [H]overing Finger, [G]loved Finger,"
			" [N]arrow Object Swipe, Hand[E]dge)\n");
		TOUCH_INFO_MSG("ex) echo eNGh 1 > object_report     "
			" (enable Hand[E]dge, [N]arrow Object Swipe,"
			" [G]loved Finger, [H]overing Finger)\n");
		TOUCH_INFO_MSG("ex) echo uPsF 0 > object_report     "
			" (disable [U]nclassified Object, [P]alm,"
			" [S]tylus, [F]inger)\n");
		TOUCH_INFO_MSG("ex) echo HguP 0 > object_report     "
			" (disable [H]overing Finger, [G]loved Finger,"
			" [U]nclassified Object, [P]alm)\n");
		TOUCH_INFO_MSG("ex) echo HFnuPSfe 1 > object_report "
			" (enable all object)\n");
		TOUCH_INFO_MSG("ex) echo enghupsf 0 > object_report "
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
		TOUCH_INFO_MSG("Invalid enable_rmi_dev value:%d\n", value);
		return count;
	}

	enable_rmi_dev = value;
	TOUCH_INFO_MSG("enable_rmi_dev:%u\n", enable_rmi_dev);

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
	TOUCH_ERR_MSG("fail to enable_rmi_dev\n");
	return ERROR;
}

static LGE_TOUCH_ATTR(version, S_IRUGO | S_IWUSR, show_firmware, NULL);
static LGE_TOUCH_ATTR(fw_ver, S_IRUGO | S_IWUSR, show_atcmd_fw_ver, NULL);
static LGE_TOUCH_ATTR(tci, S_IRUGO | S_IWUSR, show_tci, store_tci);
static LGE_TOUCH_ATTR(reg_ctrl, S_IRUGO | S_IWUSR, NULL, store_reg_ctrl);
static LGE_TOUCH_ATTR(object_report, S_IRUGO | S_IWUSR,
	show_object_report, store_object_report);
static LGE_TOUCH_ATTR(enable_rmi_dev, S_IRUGO | S_IWUSR, show_use_rmi_dev, store_use_rmi_dev);

static struct attribute *lge_specific_touch_attribute_list[] = {
	&lge_touch_attr_version.attr,
	&lge_touch_attr_tci.attr,
	&lge_touch_attr_reg_ctrl.attr,
	&lge_touch_attr_object_report.attr,
	&lge_touch_attr_fw_ver.attr,
	&lge_touch_attr_enable_rmi_dev.attr,
	NULL,
};

static int read_page_description_table(struct i2c_client *client)
{
	struct synaptics_ts_data *ts
		= (struct synaptics_ts_data *)get_touch_handle(client);
	struct function_descriptor buffer;

	unsigned short address = 0;
	unsigned short page_num = 0;

	TOUCH_TRACE();

	memset(&buffer, 0x0, sizeof(struct function_descriptor));
	memset(&ts->common_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->finger_fc, 0x0, sizeof(struct ts_ic_function));
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

	TOUCH_DEBUG(DEBUG_BASE_INFO,
		"common[%dP:0x%02x] finger[%dP:0x%02x] sensor[%dP:0x%02x]"
		"analog[%dP:0x%02x] flash[%dP:0x%02x]\n",
		ts->common_fc.function_page, ts->common_fc.dsc.id,
		ts->finger_fc.function_page, ts->finger_fc.dsc.id,
		ts->sensor_fc.function_page, ts->sensor_fc.dsc.id,
		ts->analog_fc.function_page, ts->analog_fc.dsc.id,
		ts->flash_fc.function_page, ts->flash_fc.dsc.id);

	get_f12_info(ts);

	return 0;
error:
	return -EIO;
}

static int get_ic_info(struct synaptics_ts_data *ts)
{
	const struct firmware *fw_entry = NULL;
	const char *path = ts->pdata->fw->fw_image;
	const u8 *fw = NULL;
	int rc = 0;

	memset(&ts->fw_info, 0, sizeof(struct synaptics_ts_fw_info));

	DO_SAFE(touch_i2c_read(ts->client, PRODUCT_ID_REG,
		sizeof(ts->fw_info.fw_product_id) - 1,
		ts->fw_info.fw_product_id), error);
	DO_SAFE(touch_i2c_read(ts->client, FLASH_CONFIG_ID_REG,
		sizeof(ts->fw_info.fw_version) - 1,
		ts->fw_info.fw_version), error);

	rc = request_firmware(&fw_entry, path, &ts->client->dev);
	if (rc != 0) {
		TOUCH_ERR_MSG("request_firmware() failed %d\n", rc);
		goto error;
	}

	fw = fw_entry->data;

	memcpy(ts->fw_info.fw_image_product_id, &fw[0x0040], 6);
	memcpy(ts->fw_info.fw_image_version, &fw[0x16d00], 4);
	ts->fw_info.fw_start = (unsigned char *)&fw[0];
	ts->fw_info.fw_size = sizeof(fw);

	release_firmware(fw_entry);
	return 0;
error:
	memset(&fw_entry, 0, sizeof(fw_entry));
	return -1;
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
	return -1;
}
/*
static int set_doze_param(struct synaptics_ts_data *ts)
{
	int interval = 3;
	int wakeup = 30;
	u8 buf[6] = {0};

	DO_SAFE(touch_i2c_read(ts->client, f12_info.ctrl_reg_addr[27], 6, buf), error);

	///max active duration
	if (ts->pw_data.tap_count < 3)
		buf[3] = 3;
	else
		buf[3] = 3 + ts->pw_data.tap_count;

	buf[2] = 0x0C;	// False Activation Threshold
	buf[4] = 0x01;	// Timer 1
	buf[5] = 0x01;	// Max Active Duration Timeout

	DO_SAFE(touch_i2c_write(ts->client, f12_info.ctrl_reg_addr[27], 6, buf), error);
	DO_SAFE(touch_i2c_write_byte(ts->client, DOZE_INTERVAL_REG, interval), error);
	DO_SAFE(touch_i2c_write_byte(ts->client, DOZE_WAKEUP_THRESHOLD_REG, wakeup), error);

	TOUCH_DEBUG(DEBUG_BASE_INFO, "%s: [%d] [%d] [%d] [%d] [%d]\n",
					__func__, interval, wakeup, (int)buf[3], (int)buf[4], (int)buf[5]);
	TOUCH_INFO_MSG("Set doze parameters\n");

	return 0;
error:
	return -1;
}*/

static int lpwg_update_all(struct synaptics_ts_data *ts, bool irqctrl)
{
	int sleep_status = 0;
	int lpwg_status = 0;
	bool req_lpwg_param = false;

//	if (mfts_enable) {
//		goto error;
//	}

	TOUCH_TRACE();

	if(!irqctrl && !atomic_read(&ts->lpwg_ctrl.is_suspend) && !ts->lpwg_ctrl.screen)
		ts->lpwg_ctrl.screen = 1;

	if(ts->lpwg_ctrl.screen) {
		if(atomic_read(&ts->lpwg_ctrl.is_suspend) == 1) {
			if(power_state == POWER_OFF || power_state == POWER_SLEEP)
				ts->is_init = 0;
			if (irqctrl)
				disable_irq_wake(ts->client->irq);
		}

		atomic_set(&ts->lpwg_ctrl.is_suspend, 0);
		TOUCH_INFO_MSG("%s : disable, irqctrl=%d\n", __func__, irqctrl);
	} else {
		if(atomic_read(&ts->lpwg_ctrl.is_suspend) == 0) {
			atomic_set(&ts->lpwg_ctrl.is_suspend, 1);
			if (irqctrl)
				enable_irq_wake(ts->client->irq);
			TOUCH_INFO_MSG("%s : enable, irqctrl=%d\n", __func__, irqctrl);
		//	set_doze_param(ts);
		}
	}

	if(ts->lpwg_ctrl.screen) { 												//  on(1)
		sleep_status = 1;
		lpwg_status = 0;
	} else if(!ts->lpwg_ctrl.screen && ts->lpwg_ctrl.qcover) {				// off(0), closed(0),   --
		sleep_status = 1;
		lpwg_status = 1;
	} else if(!ts->lpwg_ctrl.screen && !ts->lpwg_ctrl.qcover && ts->lpwg_ctrl.sensor) {// off(0),   open(1),  far(1)
		sleep_status = 1;
		lpwg_status = ts->lpwg_ctrl.lpwg_mode;
	} else if(!ts->lpwg_ctrl.screen && !ts->lpwg_ctrl.qcover && !ts->lpwg_ctrl.sensor) {// off(0),   open(1), near(0)
		sleep_status = 0;
		req_lpwg_param = true;
	}

	DO_SAFE(sleep_control(ts, sleep_status, 0), error);
	if(req_lpwg_param == false)
		DO_SAFE(lpwg_control(ts, lpwg_status), error);

	return NO_ERROR;
error:
	return ERROR;
}

enum error_type synaptics_ts_probe(struct i2c_client *client,
	const struct touch_platform_data *lge_ts_data,
	const struct state_info *state, struct attribute ***attribute_list)
{
	struct synaptics_ts_data *ts;

	TOUCH_TRACE();

	ASSIGN(ts = devm_kzalloc(&client->dev,
		sizeof(struct synaptics_ts_data), GFP_KERNEL), error);
	set_touch_handle(client, ts);

	ts->client = client;
	ts->pdata = lge_ts_data;
	ts->state = state;

	if (ts->pdata->pwr->use_regulator) {
		DO_IF(IS_ERR(ts->regulator_vdd = regulator_get(&client->dev,
			ts->pdata->pwr->vdd)), error);
		if (ts->pdata->pwr->vdd_voltage > 0)
			DO_SAFE(regulator_set_voltage(ts->regulator_vdd,
				ts->pdata->pwr->vdd_voltage,
				ts->pdata->pwr->vdd_voltage), error);
	}

	*attribute_list = lge_specific_touch_attribute_list;
	ts->is_probed = 0;
	ts->is_init = 0;

	atomic_set(&ts->lpwg_ctrl.is_suspend, 0);

	INIT_DELAYED_WORK(&ts->work_timer, lpwg_timer_func);
	wake_lock_init(&ts->timer_wake_lock, WAKE_LOCK_SUSPEND, "touch_timer");

	return NO_ERROR;

error:
	TOUCH_ERR_MSG("touch synaptics probe failed\n");

	return ERROR;
}

enum error_type synaptics_ts_remove(struct i2c_client *client)
{
	struct synaptics_ts_data *ts
		= (struct synaptics_ts_data *)get_touch_handle(client);

	TOUCH_TRACE();

	if (ts->pdata->pwr->use_regulator) {
		regulator_put(ts->regulator_vio);
		regulator_put(ts->regulator_vdd);
	}

	wake_lock_destroy(&ts->timer_wake_lock);
	kfree(ts);
	return NO_ERROR;
}

enum error_type synaptics_ts_init(struct i2c_client *client)
{
	struct synaptics_ts_data *ts
		= (struct synaptics_ts_data *)get_touch_handle(client);
	u8 buf = 0;
	u8 buf_array[2] = {0};

//	u8 mode = ts->lpwg_ctrl.lpwg_mode;
//	int is_suspend = atomic_read(&ts->lpwg_ctrl.is_suspend);

	TOUCH_TRACE();

	if (ts->is_probed == 0) {
		read_page_description_table(ts->client);
		get_ic_info(ts);
		check_firmware_status(ts);
		ts->is_probed = 1;
	}

	DO_SAFE(touch_i2c_write_byte(client, DEVICE_CONTROL_REG,
		DEVICE_CONTROL_NOSLEEP | DEVICE_CONTROL_CONFIGURED), error);

	DO_SAFE(touch_i2c_read(client, INTERRUPT_ENABLE_REG, 1, &buf), error);
	DO_SAFE(touch_i2c_write_byte(client, INTERRUPT_ENABLE_REG,
		buf | INTERRUPT_MASK_ABS0), error);

	if (ts->pdata->role->report_mode == REDUCED_REPORT_MODE)
		buf_array[0] = buf_array[1]
			= ts->pdata->role->delta_pos_threshold;
	else
		buf_array[0] = buf_array[1] = 0;

	DO_SAFE(touch_i2c_write(client, FINGER_REPORT_REG, 2, buf_array), error);
//	DO_SAFE(lpwg_control(ts, is_suspend ? mode : 0), error);

	/* It always should be done last. */
	DO_SAFE(touch_i2c_read(client, INTERRUPT_STATUS_REG, 1, &buf), error);

	ts->is_init = 1;

	return NO_ERROR;
error:
	return ERROR;
}

enum error_type synaptics_ts_get_data(struct i2c_client *client,
	struct touch_data *curr_data, const struct touch_data *prev_data)
{
	struct synaptics_ts_data *ts
		= (struct synaptics_ts_data *)get_touch_handle(client);
	struct finger_data *data;
	struct t_data *c_data;

	u8  i = 0;
	u8  finger_index = 0;

	TOUCH_TRACE();

	if (!ts->is_init)
		return IGNORE_EVENT;

	curr_data->total_num = 0;
	curr_data->id_mask = 0;

	DO_SAFE(touch_i2c_read(client, DEVICE_STATUS_REG,
		sizeof(ts->ts_data.device_status_reg),
		&ts->ts_data.device_status_reg), error);

	DO_IF((ts->ts_data.device_status_reg & DEVICE_FAILURE_MASK)
		== DEVICE_FAILURE_MASK, error);

	DO_SAFE(touch_i2c_read(client, INTERRUPT_STATUS_REG,
		sizeof(ts->ts_data.interrupt_status_reg),
		&ts->ts_data.interrupt_status_reg), error);

	TOUCH_DEBUG(DEBUG_GET_DATA,
		"interrupt status 0x%x\n", ts->ts_data.interrupt_status_reg);

	if (enable_rmi_dev && rmidev_fhandler.initialized == true) {
		rmidev_fhandler.exp_fn->attn(ts, ts->ts_data.interrupt_status_reg);
	}

	if (ts->ts_data.interrupt_status_reg & INTERRUPT_MASK_CUSTOM) {
		u8 status = 0;
		DO_SAFE(synaptics_ts_page_data_read(client,
			LPWG_PAGE, LPWG_STATUS_REG, 1, &status), error);

		if (status & 0x1) { /* TCI-1 : Double-Tap */
			if (ts->lpwg_ctrl.double_tap_enable) {
				get_tci_data(ts, 2);
				send_uevent_lpwg(ts->client, LPWG_DOUBLE_TAP);
			}
		} else if (status & 0x2) { /* TCI-2 : Multi-Tap */
			if (ts->lpwg_ctrl.password_enable) {
				get_tci_data(ts, ts->pw_data.tap_count);
				wake_lock(&ts->timer_wake_lock);
				tci_control(ts, REPORT_MODE_CTRL, 0);
				queue_delayed_work(touch_wq, &ts->work_timer,
					msecs_to_jiffies(UEVENT_DELAY - I2C_DELAY));
			}
		} else {
			TOUCH_DEBUG(DEBUG_BASE_INFO | DEBUG_LPWG,
				"LPWG status has problem[%d]\n", status);
		}
		return IGNORE_EVENT;
	} else if (ts->ts_data.interrupt_status_reg & INTERRUPT_MASK_ABS0) {
		DO_SAFE(touch_i2c_read(ts->client, FINGER_DATA_REG_START,
			(NUM_OF_EACH_FINGER_DATA_REG * MAX_NUM_OF_FINGERS),
			ts->ts_data.finger.finger_reg[0]), error);

		for (i = 0; i < MAX_NUM_OF_FINGERS; i++) {
			data = &ts->ts_data.finger;
			if (data->finger_reg[i][0] & 0x03) {
				c_data = &curr_data->abs_data[finger_index];
				c_data->id = i;
				c_data->type =
					data->finger_reg[i][REG_OBJECT];
				c_data->x =
				c_data->raw_x =
					TS_SNTS_GET_X_POSITION(
					data->finger_reg[i][REG_X_MSB],
					data->finger_reg[i][REG_X_LSB]);
				c_data->y =
				c_data->raw_y =
					TS_SNTS_GET_Y_POSITION(
					data->finger_reg[i][REG_Y_MSB],
					data->finger_reg[i][REG_Y_LSB]);
				c_data->width_major =
					TS_SNTS_GET_WIDTH_MAJOR(
					data->finger_reg[i][REG_WX],
					data->finger_reg[i][REG_WY]);
				c_data->width_minor =
					TS_SNTS_GET_WIDTH_MINOR(
					data->finger_reg[i][REG_WX],
					data->finger_reg[i][REG_WY]);
				c_data->orientation =
					TS_SNTS_GET_ORIENTATION(
					data->finger_reg[i][REG_WY],
					data->finger_reg[i][REG_WX]);
				c_data->pressure =
					TS_SNTS_GET_PRESSURE(
					data->finger_reg[i][REG_Z]);

				curr_data->id_mask |= (0x1 << i);
				curr_data->total_num++;

				TOUCH_DEBUG(DEBUG_GET_DATA,
					"<%d> type[%d] pos(%4d,%4d) w_m[%2d]"
					"w_n[%2d] o[%2d] p[%2d]\n",
					i, c_data->type,
					c_data->x, c_data->y,
					c_data->width_major,
					c_data->width_minor,
					c_data->orientation,
					c_data->pressure);

				finger_index++;
			}
		}

		TOUCH_DEBUG(DEBUG_GET_DATA,
			"ID[0x%x] Total_num[%d]\n",
			curr_data->id_mask, curr_data->total_num);

		if (ts->lpwg_ctrl.password_enable &&
				wake_lock_active(&ts->timer_wake_lock)) {
			if (curr_data->id_mask & ~(prev_data->id_mask)) {
				if (cancel_delayed_work(&ts->work_timer)) {
					/* password-matching will be failed */
					ts->pw_data.data_num = 1;
					queue_delayed_work(touch_wq,
						&ts->work_timer,
						msecs_to_jiffies(UEVENT_DELAY));
				}
			}
			return IGNORE_EVENT_BUT_SAVE_IT;
		}
	} else if (ts->ts_data.interrupt_status_reg & INTERRUPT_MASK_FLASH) {
		TOUCH_DEBUG(DEBUG_BASE_INFO, "FLASH Interrupt occurs!\n");
		return ERROR;
	} else {
		return IGNORE_EVENT;
	}

	return NO_ERROR;
error:
	return ERROR;
}

enum error_type synaptics_ts_filter(struct i2c_client *client,
	struct touch_data *curr_data, const struct touch_data *prev_data)
{
	int i = 0;

	for (i = 0; i < curr_data->total_num; i++) {
		if (curr_data->abs_data[i].type == HOVER_TYPE) {
			curr_data->abs_data[i].pressure = 0;
			return NO_FILTER;
		} else if (curr_data->abs_data[i].type == PALM_TYPE) {
			curr_data->abs_data[i].pressure = MAX_PRESSURE;
		} else if (curr_data->abs_data[i].pressure == MAX_PRESSURE) {
			curr_data->abs_data[i].pressure = MAX_PRESSURE - 1;
		}
	}

	return NO_ERROR;
}

enum error_type synaptics_ts_power(struct i2c_client *client, int power_ctrl)
{
	struct synaptics_ts_data *ts
		= (struct synaptics_ts_data *)get_touch_handle(client);

	TOUCH_TRACE();

	switch (power_ctrl) {
#if !defined(PWR_CONTROL_FROM_LCD)
	case POWER_OFF:
		ts->is_init = 0;

		if (ts->pdata->pwr->use_regulator) {
			if (regulator_is_enabled(ts->regulator_vdd))
				regulator_disable(ts->regulator_vdd);
		}
		break;
	case POWER_ON:
		if (ts->pdata->pwr->use_regulator) {
			if (!regulator_is_enabled(ts->regulator_vdd))
				DO_SAFE(regulator_enable(ts->regulator_vdd), error);
		}
		break;
#endif
	case POWER_SLEEP:
//		if (!ts->lpwg_ctrl.lpwg_is_enabled)
//			sleep_control(ts, 0, 1);
		break;
	case POWER_WAKE:
		sleep_control(ts, 1, 1);
		break;
	default:
		break;
	}

    power_state = power_ctrl;

	return NO_ERROR;
#if !defined(PWR_CONTROL_FROM_LCD)
error:
	return ERROR;
#endif
}

enum error_type synaptics_ts_ic_ctrl(struct i2c_client *client,
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

enum error_type synaptics_ts_fw_upgrade(struct i2c_client *client,
	const char* fw_path)
{
	struct synaptics_ts_data *ts
		= (struct synaptics_ts_data *)get_touch_handle(client);

	/* During upgrading, interrupt will be ignored. */
	ts->is_probed = 0;
	ts->is_init = 0;

	DO_SAFE(FirmwareUpgrade(ts, fw_path), error);

	return NO_ERROR;
error:
	return ERROR;
}

enum error_type synaptics_ts_get_fw_info(struct i2c_client *client, struct touch_fw_info *info)
{
	struct synaptics_ts_data *ts =
			(struct synaptics_ts_data *)get_touch_handle(client);

	if(ts->fw_info.fw_version[3] & 0x80)
		info->fw_type = OFFICIAL_VERSION;
	else
		info->fw_type = TEST_VERSION;

	sprintf(info->fw_ic,"V%d.%02d\n",
		(ts->fw_info.fw_version[3] & 0x80 ? 1 : 0),
		ts->fw_info.fw_version[3] & 0x7F);
	TOUCH_DEBUG(DEBUG_BASE_INFO,
		"fw_version(ic)[V%d.%02d(0x%02X 0x%02X 0x%02X 0x%02X)] ",
		(ts->fw_info.fw_version[3] & 0x80 ? 1 : 0),
		ts->fw_info.fw_version[3] & 0x7F,
		ts->fw_info.fw_version[0], ts->fw_info.fw_version[1],
		ts->fw_info.fw_version[2], ts->fw_info.fw_version[3]);

	sprintf(info->fw_bin,"V%d.%02d\n",
		(ts->fw_info.fw_image_version[3] & 0x80 ? 1 : 0),
		ts->fw_info.fw_image_version[3] & 0x7F);
	TOUCH_DEBUG(DEBUG_BASE_INFO,
		"fw_version(fw)[V%d.%02d(0x%02X 0x%02X 0x%02X 0x%02X)]\n",
		(ts->fw_info.fw_image_version[3] & 0x80 ? 1 : 0),
		ts->fw_info.fw_image_version[3] & 0x7F,
		ts->fw_info.fw_image_version[0],
		ts->fw_info.fw_image_version[1],
		ts->fw_info.fw_image_version[2],
		ts->fw_info.fw_image_version[3]);

	strcpy(info->fw_path, ts->pdata->fw->fw_image);
	TOUCH_DEBUG(DEBUG_BASE_INFO, "fw path = %s\n", info->fw_path);

	return NO_ERROR;
}

enum error_type  synaptics_ts_notify(struct i2c_client *client,
	u8 code, u32 value)
{
	switch (code) {
	case NOTIFY_TA_CONNECTION:
		break;
	case NOTIFY_TEMPERATURE_CHANGE:
		break;
	case NOTIFY_PROXIMITY:
		break;
	case NOTIFY_HALL_IC:
		break;
	case NOTIFY_KEYGUARD:
		break;
	case NOTIFY_IME:
		break;
	default:
		break;
	}

	return NO_ERROR;
}

enum error_type synaptics_ts_suspend(struct i2c_client *client)
{
//	struct synaptics_ts_data *ts
//		= (struct synaptics_ts_data *)get_touch_handle(client);

	TOUCH_TRACE();

//	if (!atomic_read(&ts->lpwg_ctrl.is_suspend)) {
//		DO_SAFE(lpwg_control(ts, ts->lpwg_ctrl.lpwg_mode), error);
//		atomic_set(&ts->lpwg_ctrl.is_suspend, 2);
//	}

	return NO_ERROR;
//error:
//	return ERROR;
}

enum error_type synaptics_ts_resume(struct i2c_client *client)
{
	struct synaptics_ts_data *ts
		= (struct synaptics_ts_data *)get_touch_handle(client);

	TOUCH_TRACE();

	cancel_delayed_work_sync(&ts->work_timer);

	if (wake_lock_active(&ts->timer_wake_lock))
		wake_unlock(&ts->timer_wake_lock);

	if (atomic_read(&ts->state->upgrade_state) != UPGRADE_START) {
		/* To resolve 'eternal-death' */
		ts_gpio_set_value(ts->pdata->reset_pin, 0);
		msleep(ts->pdata->role->reset_delay);
		ts_gpio_set_value(ts->pdata->reset_pin, 1);
	}

	atomic_set(&ts->lpwg_ctrl.is_suspend, 0);

	return NO_ERROR;
}

enum error_type synaptics_ts_lpwg(struct i2c_client *client,
	u32 code, u32 value, struct point *data)
{
	struct synaptics_ts_data *ts
		= (struct synaptics_ts_data *)get_touch_handle(client);

	int *v = (int *)value;

	switch (code) {
	case LPWG_ENABLE: /* These codes will be changed in Next OS */
		ts->lpwg_ctrl.lpwg_mode = *v;
		DO_SAFE(lpwg_control(ts, ts->lpwg_ctrl.lpwg_mode), error);
		break;
	case LPWG_DISABLE:
		DO_SAFE(lpwg_control(ts, LPWG_NONE), error);
		break;
	case LPWG_EARLY_SUSPEND:
		break;
	case LPWG_SENSOR_STATUS:
		/*value 0 : near, 1: far*/
		DO_SAFE(sleep_control(ts, *(v + 2), 1), error);
		break;
	case LPWG_UPDATE_ALL:
		ts->lpwg_ctrl.lpwg_mode = *(v + 0);
		ts->lpwg_ctrl.screen = *(v + 1);
		ts->lpwg_ctrl.sensor = *(v + 2);
		ts->lpwg_ctrl.qcover = *(v + 3);

		TOUCH_DEBUG(DEBUG_BASE_INFO || DEBUG_LPWG, "mode[%d], screen[%d], sensor[%d], qcover[%d]\n",
			ts->lpwg_ctrl.lpwg_mode, ts->lpwg_ctrl.screen, ts->lpwg_ctrl.sensor, ts->lpwg_ctrl.qcover);

		DO_SAFE(lpwg_update_all(ts, 1), error);
		break;
	case LPWG_TAP_COUNT:
		ts->pw_data.tap_count = value;
		if (ts->lpwg_ctrl.password_enable) {
			tci_control(ts, TAP_COUNT_CTRL,
				(u8)ts->pw_data.tap_count);
		}
		break;
	case LPWG_LENGTH_BETWEEN_TAP:
		if (ts->lpwg_ctrl.double_tap_enable
				|| ts->lpwg_ctrl.password_enable) {
			tci_control(ts, TAP_DISTANCE_CTRL, value);
		}
		break;
	case LPWG_DOUBLE_TAP_CHECK:
		ts->pw_data.double_tap_check = value;
		if (ts->lpwg_ctrl.password_enable) {
			tci_control(ts, INTERRUPT_DELAY_CTRL, value);
		}
		break;
	case LPWG_READ:
		if (ts->lpwg_ctrl.password_enable)
			memcpy(data, ts->pw_data.data,
				sizeof(struct point)*ts->pw_data.data_num);

		data[ts->pw_data.data_num].x = -1;
		data[ts->pw_data.data_num].y = -1;
		break;
	case LPWG_REPLY:
		TOUCH_DEBUG(DEBUG_BASE_INFO || DEBUG_LPWG, "%s : reply = %d\n", __func__, value);

		if(atomic_read(&ts->lpwg_ctrl.is_suspend) == 0) {
			TOUCH_DEBUG(DEBUG_BASE_INFO || DEBUG_LPWG, "%s : screen on\n", __func__);
			break;
		}
		ts->lpwg_ctrl.screen = value;
		DO_SAFE(lpwg_update_all(ts, 1), error);
		break;
	default:
		break;
	}

	return NO_ERROR;
error:
	return ERROR;
}

enum error_type synaptics_f54_test(struct i2c_client *client,
	char* buf, int* raw_status, int* ch_status)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);
	int full_raw_cap = 0;
	int trx_to_trx = 0;
	int high_resistance = 0;

	SCAN_PDT(client);

	full_raw_cap = F54Test('a', 0, buf);
	msleep(30);

	high_resistance = F54Test('g', 0, buf);
	msleep(50);

	trx_to_trx = F54Test('f', 0, buf);
	msleep(30);

	Reset();

	*raw_status = full_raw_cap;
	*ch_status = trx_to_trx && high_resistance;

	synaptics_ts_init(ts->client);
	msleep(30);

	return NO_ERROR;
}

struct touch_device_driver synaptics_ts_driver = {

	.probe		= synaptics_ts_probe,
	.remove		= synaptics_ts_remove,
	.suspend	= synaptics_ts_suspend,
	.resume		= synaptics_ts_resume,
	.init  		= synaptics_ts_init,
	.data  		= synaptics_ts_get_data,
	.filter		= synaptics_ts_filter,
	.power 		= synaptics_ts_power,
	.ic_ctrl	= synaptics_ts_ic_ctrl,
	.fw_upgrade 	= synaptics_ts_fw_upgrade,
	.fw_ic_info 	= synaptics_ts_get_fw_info,
	.notify		= synaptics_ts_notify,
	.lpwg		= synaptics_ts_lpwg,
	.sd			= synaptics_f54_test
};

static struct of_device_id match_table[] = {
	{ .compatible = "synaptics,s3320",},
	{ },
};

#ifdef CONFIG_MTK_TOUCHPANEL
static struct i2c_board_info __initdata i2c_tpd = {I2C_BOARD_INFO(LGE_TOUCH_NAME, 0x20)};
#endif

static void async_touch_init(void *data, async_cookie_t cookie)
{
	touch_driver_register(&synaptics_ts_driver, match_table);
	return;
}

static int __init touch_init(void)
{
	TOUCH_TRACE();

#ifdef CONFIG_MTK_TOUCHPANEL
	if(ts_dma_allocation()) {
		TOUCH_ERR_MSG("dma allocation fail!!\n");
		return ERROR;
	}

	i2c_register_board_info(0, &i2c_tpd, 1);
#endif

	async_schedule(async_touch_init, NULL);

	return NO_ERROR;
}

static void __exit touch_exit(void)
{
	TOUCH_TRACE();
	touch_driver_unregister();
}

module_init(touch_init);
module_exit(touch_exit);

MODULE_AUTHOR("yehan.ahn@lge.com, hyesung.shin@lge.com");
MODULE_DESCRIPTION("LGE Touch Driver");
MODULE_LICENSE("GPL");

