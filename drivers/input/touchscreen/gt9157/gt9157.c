/* drivers/input/touchscreen/gt9xx.c
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
 * Version: 2.2
 * Authors: andrew@goodix.com, meta@goodix.com
 * Release Date: 2014/01/14
 * Revision record:
 *      V1.0:
 *          first Release. By Andrew, 2012/08/31
 *      V1.2:
 *          modify gtp_reset_guitar,slot report,tracking_id & 0x0F.
 *	By Andrew, 2012/10/15
 *      V1.4:
 *          modify gt9xx_update.c. By Andrew, 2012/12/12
 *      V1.6:
 *          1. new heartbeat/esd_protect mechanism(add external watchdog)
 *          2. doze mode, sliding wakeup
 *          3. 3 more cfg_group(GT9 Sensor_ID: 0~5)
 *          3. config length verification
 *          4. names & comments
 *                  By Meta, 2013/03/11
 *      V1.8:
 *          1. pen/stylus identification
 *          2. read double check & fixed config support
 *          3. new esd & slide wakeup optimization
 *                  By Meta, 2013/06/08
 *      V2.0:
 *          1. compatible with GT9XXF
 *          2. send config after resume
 *                  By Meta, 2013/08/06
 *      V2.2:
 *          1. gt9xx_config for debug
 *          2. gesture wakeup
 *          3. pen separate input device, active-pen button support
 *          4. coordinates & keys optimization
 *                  By Meta, 2014/01/14
 */

#include <linux/irq.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include "gt9xx.h"

#if GTP_ICS_SLOT_REPORT
#include <linux/input/mt.h>
#endif

static const char *goodix_ts_name = "goodix-ts";
static struct workqueue_struct *goodix_wq;
struct i2c_client *i2c_connect_client;
static struct class *class_touchscreen = NULL;
static char *class_ts_info = NULL;
#define GOODIX_COORDS_ARR_SIZE	4
#define GOODIX_VTG_MIN_UV	2600000
#define GOODIX_VTG_MAX_UV	3300000
#define GOODIX_I2C_VTG_MIN_UV	1800000
#define GOODIX_I2C_VTG_MAX_UV	1800000
#define GOODIX_VDD_LOAD_MIN_UA	0
#define GOODIX_VDD_LOAD_MAX_UA	10000
#define GOODIX_VIO_LOAD_MIN_UA	0
#define GOODIX_VIO_LOAD_MAX_UA	10000
#ifdef FEATURE_QRD8X26_UNISCOPE_MODIFY
#define GPIO_VDD_2_8V		56
#endif
#define MAX_BUTTONS		4
#define PROP_NAME_SIZE		24

u8 config[GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH] = {
		GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};

#if GTP_HAVE_TOUCH_KEY
static const u16 touch_key_array[] = GTP_KEY_TAB;
#define GTP_MAX_KEY_NUM  (sizeof(touch_key_array) / sizeof(touch_key_array[0]))
#if GTP_DEBUG_ON
static const int key_codes[] = {KEY_HOMEPAGE, KEY_BACK, KEY_MENU, KEY_SEARCH};
static const char *const key_names[] = {
	"Key_Home", "Key_Back", "Key_Menu", "Key_Search"};
#endif
#endif

static s8 gtp_i2c_test(struct i2c_client *client);
static void gtp_int_sync(struct goodix_ts_data *ts, s32 ms);

static ssize_t gt91xx_config_read_proc(struct file *, char __user *, size_t,
				       loff_t *);
static ssize_t gt91xx_config_write_proc(struct file *, const char __user *,
					size_t, loff_t *);

static struct proc_dir_entry *gt91xx_config_proc;
static const struct file_operations config_proc_ops = {
	.owner = THIS_MODULE,
	.read = gt91xx_config_read_proc,
	.write = gt91xx_config_write_proc,
};

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data);
static int goodix_ts_suspend(struct device *dev);
static int goodix_ts_resume(struct device *dev);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void goodix_ts_early_suspend(struct early_suspend *h);
static void goodix_ts_late_resume(struct early_suspend *h);
#endif

#if GTP_ESD_PROTECT
static struct delayed_work gtp_esd_check_work;
static struct workqueue_struct *gtp_esd_check_workqueue;
static void gtp_esd_check_func(struct work_struct *);
static s32 gtp_init_ext_watchdog(struct i2c_client *client);
#endif

/*********** For GT9XXF Start **********/
#if GTP_COMPATIBLE_MODE
static s32 gtp_esd_recovery(struct i2c_client *client);
static s32 gtp_main_clk_proc(struct goodix_ts_data *ts);
static s32 gtp_bak_ref_proc(struct goodix_ts_data *ts, u8 mode);
#endif
/********** For GT9XXF End **********/

#if GTP_GESTURE_WAKEUP
enum {
	DOZE_DISABLED = 0,
	DOZE_ENABLED = 1,
	DOZE_WAKEUP = 2,
} DOZE_T;
static DOZE_T doze_status = DOZE_DISABLED;
static s8 gtp_enter_doze(struct goodix_ts_data *ts);
#endif

u8 grp_cfg_version;

#define GTP_DEBUGFS_DIR			"ts_debug"
#define GTP_DEBUGFS_FILE_SUSPEND	"suspend"
#define GTP_DEBUGFS_FILE_DATA		"data"
#define GTP_DEBUGFS_FILE_ADDR		"addr"
#define GTP_DEBUGFS_FILE_DUMP_INFO	"dump_info"

#define GOODIX_INFO_MAX_LEN 512

#define CLASS_GOODIX_STORE_TS_INFO(buf, name,factory,\
		fw_ver) \
			snprintf(buf, GOODIX_INFO_MAX_LEN, \
				"name=%s#"\
				"%s#"\
				"fw_ver=%0x",name, factory,\
				fw_ver)

#define GOODIX_STORE_TS_INFO(buf, name, max_tch, product_id, fw_vkey_support) \
				snprintf(buf, GOODIX_INFO_MAX_LEN, \
				"controller\t= goodix\n" \
				"name\t\t= %s\n" \
				"max_touches\t= %d\n" \
				"product\t= 0x%s\n" \
				"fw_vkey_support\t= %s\n" \
				, name, \
				max_tch, product_id, \
				fw_vkey_support)

/*******************************************************
Function:
    Read data from the i2c slave device.
Input:
    client:     i2c device.
    buf[0~1]:   read start address.
    buf[2~len-1]:   read data buffer.
    len:    GTP_ADDR_LENGTH + read bytes count
Output:
    numbers of i2c_msgs to transfer:
      2: succeed, otherwise: failed
*********************************************************/
s32 gtp_i2c_read(struct i2c_client *client, u8 *buf, s32 len)
{
	struct i2c_msg msgs[2];
	s32 ret = -1;
	s32 retries = 0;

	GTP_DEBUG_FUNC();

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr = client->addr;
	msgs[0].len = GTP_ADDR_LENGTH;
	msgs[0].buf = &buf[0];

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr = client->addr;
	msgs[1].len = len - GTP_ADDR_LENGTH;
	msgs[1].buf = &buf[GTP_ADDR_LENGTH];

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret == 2)
			break;
		retries++;
	}
	if ((retries >= 5)) {
#if GTP_COMPATIBLE_MODE
		struct goodix_ts_data *ts = i2c_get_clientdata(client);
#endif

#if GTP_GESTURE_WAKEUP
		/* reset chip would quit doze mode */
		if (DOZE_ENABLED == doze_status)
			return ret;
#endif
		GTP_ERROR
		    ("I2C Read: 0x%04X, %d bytes failed, errcode: %d!",
		     (((u16)(buf[0] << 8)) | buf[1]), len - 2, ret);
#if GTP_COMPATIBLE_MODE
		if (CHIP_TYPE_GT9F == ts->chip_type)
			gtp_recovery_reset(client);
		else
#endif
			gtp_reset_guitar(client, 10);
	}
	return ret;
}

/*******************************************************
Function:
    Write data to the i2c slave device.
Input:
    client:     i2c device.
    buf[0~1]:   write start address.
    buf[2~len-1]:   data buffer
    len:    GTP_ADDR_LENGTH + write bytes count
Output:
    numbers of i2c_msgs to transfer:
    1: succeed, otherwise: failed
*********************************************************/
s32 gtp_i2c_write(struct i2c_client *client, u8 *buf, s32 len)
{
	struct i2c_msg msg;
	s32 ret = -1;
	s32 retries = 0;

	GTP_DEBUG_FUNC();

	msg.flags = !I2C_M_RD;
	msg.addr = client->addr;
	msg.len = len;
	msg.buf = buf;

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)
			break;
		retries++;
	}
	if ((retries >= 5)) {
#if GTP_COMPATIBLE_MODE
		struct goodix_ts_data *ts = i2c_get_clientdata(client);
#endif

#if GTP_GESTURE_WAKEUP
		if (DOZE_ENABLED == doze_status)
			return ret;
#endif
		GTP_ERROR
		    ("I2C Write: 0x%04X, %d bytes failed, errcode: %d!",
		     (((u16)(buf[0] << 8)) | buf[1]), len - 2, ret);
#if GTP_COMPATIBLE_MODE
		if (CHIP_TYPE_GT9F == ts->chip_type)
			gtp_recovery_reset(client);
		else
#endif
			gtp_reset_guitar(client, 10);
	}
	return ret;
}

/*******************************************************
Function:
    i2c read twice, compare the results
Input:
    client:  i2c device
    addr:    operate address
    rxbuf:   read data to store, if compare successful
    len:     bytes to read
Output:
    FAIL:    read failed
    SUCCESS: read successful
*********************************************************/
s32 gtp_i2c_read_dbl_check(struct i2c_client *client, u16 addr, u8 *rxbuf,
			   int len)
{
	u8 buf[16] = {0};
	u8 confirm_buf[16] = {0};
	u8 retry = 0;

	while (retry++ < 3) {
		memset(buf, 0xAA, 16);
		buf[0] = (u8)(addr >> 8);
		buf[1] = (u8)(addr & 0xFF);
		gtp_i2c_read(client, buf, len + 2);

		memset(confirm_buf, 0xAB, 16);
		confirm_buf[0] = (u8)(addr >> 8);
		confirm_buf[1] = (u8)(addr & 0xFF);
		gtp_i2c_read(client, confirm_buf, len + 2);

		if (!memcmp(buf, confirm_buf, len + 2)) {
			memcpy(rxbuf, confirm_buf + 2, len);
			return SUCCESS;
		}
	}
	GTP_ERROR("I2C read 0x%04X, %d bytes, double check failed!", addr, len);
	return FAIL;
}

/*******************************************************
Function:
    Send config.
Input:
    client: i2c device.
Output:
    result of i2c write operation.
    1: succeed, otherwise: failed
*********************************************************/
s32 gtp_send_cfg(struct i2c_client *client)
{
	s32 ret = 2;

#if GTP_DRIVER_SEND_CFG
	s32 retry = 0;
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	if (ts->fixed_cfg) {
		GTP_INFO("Ic fixed config, no config sent!");
		return 0;
	} else if (ts->pnl_init_error) {
		GTP_INFO("Error occured in init_panel, no config sent");
		return 0;
	}

	GTP_INFO("Driver send config.");
	for (retry = 0; retry < 5; retry++) {
		ret =
		    gtp_i2c_write(client, config,
				  GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH);
		if (ret > 0)
			break;
	}
#endif
	return ret;
}

/*******************************************************
Function:
    Disable irq function
Input:
    ts: goodix i2c_client private data
Output:
    None.
*********************************************************/
void gtp_irq_disable(struct goodix_ts_data *ts)
{
	unsigned long irqflags;

	GTP_DEBUG_FUNC();

	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (!ts->irq_is_disable) {
		ts->irq_is_disable = 1;
		disable_irq_nosync(ts->client->irq);
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

/*******************************************************
Function:
    Enable irq function
Input:
    ts: goodix i2c_client private data
Output:
    None.
*********************************************************/
void gtp_irq_enable(struct goodix_ts_data *ts)
{
	unsigned long irqflags = 0;

	GTP_DEBUG_FUNC();

	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (ts->irq_is_disable) {
		enable_irq(ts->client->irq);
		ts->irq_is_disable = 0;
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

/*******************************************************
Function:
    Report touch point event
Input:
    ts: goodix i2c_client private data
    id: trackId
    x:  input x coordinate
    y:  input y coordinate
    w:  input pressure
Output:
    None.
*********************************************************/
static void gtp_touch_down(struct goodix_ts_data *ts, s32 id, s32 x, s32 y,
			   s32 w)
{
//	GTP_INFO("gtp_touch_down!");
#if GTP_CHANGE_X2Y
	GTP_SWAP(x, y);
#endif
#if GTP_ICS_SLOT_REPORT
	input_mt_slot(ts->input_dev, id);
	input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
	input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
#else
	input_report_key(ts->input_dev, BTN_TOUCH, 1);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
	input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
	input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
	input_mt_sync(ts->input_dev);
#endif

	GTP_DEBUG("ID:%d, X:%d, Y:%d, W:%d", id, x, y, w);
//	GTP_INFO("ID:%d, X:%d, Y:%d, W:%d", id, x, y, w);
//	pr_err("GTP down X= %d,y=%d\n",x,y);
}

/*******************************************************
Function:
    Report touch release event
Input:
    ts: goodix i2c_client private data
Output:
    None.
*********************************************************/
static void gtp_touch_up(struct goodix_ts_data *ts, s32 id)
{
//	GTP_INFO("touch up!");
#if GTP_ICS_SLOT_REPORT
	input_mt_slot(ts->input_dev, id);
	input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, -1);
	GTP_DEBUG("Touch id[%2d] release!", id);
#else
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
#endif
}

#if GTP_WITH_PEN
static void gtp_pen_init(struct goodix_ts_data *ts)
{
	s32 ret = 0;

	GTP_INFO("Request input device for pen/stylus.");

	ts->pen_dev = input_allocate_device();
	if (ts->pen_dev == NULL) {
		GTP_ERROR("Failed to allocate input device for pen/stylus.");
		return;
	}

	ts->pen_dev->evbit[0] =
	    BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);

#if GTP_ICS_SLOT_REPORT
	input_mt_init_slots(ts->pen_dev, 16, 0);
#else
	ts->pen_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#endif

	set_bit(BTN_TOOL_PEN, ts->pen_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, ts->pen_dev->propbit);

#if GTP_PEN_HAVE_BUTTON
	input_set_capability(ts->pen_dev, EV_KEY, BTN_STYLUS);
	input_set_capability(ts->pen_dev, EV_KEY, BTN_STYLUS2);
#endif

	input_set_abs_params(ts->pen_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max,
			     0, 0);
	input_set_abs_params(ts->pen_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max,
			     0, 0);
	input_set_abs_params(ts->pen_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(ts->pen_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->pen_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);

	ts->pen_dev->name = "goodix-pen";
	ts->pen_dev->id.bustype = BUS_I2C;

	ret = input_register_device(ts->pen_dev);
	if (ret)
		GTP_ERROR("Register %s input device failed", ts->pen_dev->name);
}

static void gtp_pen_down(s32 x, s32 y, s32 w, s32 id)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);

#if GTP_CHANGE_X2Y
	GTP_SWAP(x, y);
#endif

	input_report_key(ts->pen_dev, BTN_TOOL_PEN, 1);
#if GTP_ICS_SLOT_REPORT
	input_mt_slot(ts->pen_dev, id);
	input_report_abs(ts->pen_dev, ABS_MT_TRACKING_ID, id);
	input_report_abs(ts->pen_dev, ABS_MT_POSITION_X, x);
	input_report_abs(ts->pen_dev, ABS_MT_POSITION_Y, y);
	input_report_abs(ts->pen_dev, ABS_MT_PRESSURE, w);
	input_report_abs(ts->pen_dev, ABS_MT_TOUCH_MAJOR, w);
#else
	input_report_key(ts->pen_dev, BTN_TOUCH, 1);
	input_report_abs(ts->pen_dev, ABS_MT_POSITION_X, x);
	input_report_abs(ts->pen_dev, ABS_MT_POSITION_Y, y);
	input_report_abs(ts->pen_dev, ABS_MT_PRESSURE, w);
	input_report_abs(ts->pen_dev, ABS_MT_TOUCH_MAJOR, w);
	input_report_abs(ts->pen_dev, ABS_MT_TRACKING_ID, id);
	input_mt_sync(ts->pen_dev);
#endif
	GTP_DEBUG("(%d)(%d, %d)[%d]", id, x, y, w);
}

static void gtp_pen_up(s32 id)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);

	input_report_key(ts->pen_dev, BTN_TOOL_PEN, 0);

#if GTP_ICS_SLOT_REPORT
	input_mt_slot(ts->pen_dev, id);
	input_report_abs(ts->pen_dev, ABS_MT_TRACKING_ID, -1);
#else
	input_report_key(ts->pen_dev, BTN_TOUCH, 0);
#endif
}
#endif

/*******************************************************
Function:
    Goodix touchscreen work function
Input:
    work: work struct of goodix_workqueue
Output:
    None.
*********************************************************/
static void goodix_ts_work_func(struct work_struct *work)
{
	u8 end_cmd[3] = {
		GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF, 0};
	u8 point_data[2 + 1 + 8 * GTP_MAX_TOUCH + 1] = {
		GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF};
	u8 touch_num = 0;
	u8 finger = 0;
	static u16 pre_touch;
	static u8 pre_key;
#if GTP_WITH_PEN
	u8 pen_active = 0;
	static u8 pre_pen;
#endif
	u8 key_value = 0;
	u8 *coor_data = NULL;
	s32 input_x = 0;
	s32 input_y = 0;
	s32 input_w = 0;
	s32 id = 0;
	s32 i = 0;
	s32 ret = -1;
	struct goodix_ts_data *ts = NULL;

#if GTP_COMPATIBLE_MODE
	u8 rqst_buf[3] = {0x80, 0x43};	/* for GT9XXF */
#endif

#if GTP_GESTURE_WAKEUP
	u8 doze_buf[3] = {0x81, 0x4B};
#endif

	GTP_DEBUG_FUNC();
	ts = container_of(work, struct goodix_ts_data, work);
	if (ts->enter_update)
		return;

#if GTP_GESTURE_WAKEUP
	if (DOZE_ENABLED == doze_status) {
		ret = gtp_i2c_read(i2c_connect_client, doze_buf, 3);
		GTP_DEBUG("0x814B = 0x%02X", doze_buf[2]);
		if (ret > 0) {
			if ((doze_buf[2] == 'a') || (doze_buf[2] == 'b')
				|| (doze_buf[2] == 'c') || (doze_buf[2] == 'd')
				|| (doze_buf[2] == 'e') || (doze_buf[2] == 'g')
				|| (doze_buf[2] == 'h') || (doze_buf[2] == 'm')
				|| (doze_buf[2] == 'o') || (doze_buf[2] == 'q')
				|| (doze_buf[2] == 's') || (doze_buf[2] == 'v')
				|| (doze_buf[2] == 'w') || (doze_buf[2] == 'y')
				|| (doze_buf[2] == 'z') || (doze_buf[2] == 0x5E)
			    ) {
				if (doze_buf[2] != 0x5E)
					GTP_INFO("Wakeup by gesture(%c), ",
					     doze_buf[2]);
				else
					GTP_INFO("Wakeup by gesture(^), ");
				GTP_INFO("light up the screen!");
				doze_status = DOZE_WAKEUP;
				input_report_key(ts->input_dev, KEY_POWER, 1);
				input_sync(ts->input_dev);
				input_report_key(ts->input_dev, KEY_POWER, 0);
				input_sync(ts->input_dev);
				/* clear 0x814B */
				doze_buf[2] = 0x00;
				gtp_i2c_write(i2c_connect_client, doze_buf, 3);
			} else if ((doze_buf[2] == 0xAA)
				   || (doze_buf[2] == 0xBB)
				   || (doze_buf[2] == 0xAB)
				   || (doze_buf[2] == 0xBA)) {
				char *direction[4] = {
					"Right", "Down", "Up", "Left"};
				u8 type =
				    ((doze_buf[2] & 0x0F) - 0x0A) +
				    (((doze_buf[2] >> 4) & 0x0F) - 0x0A) * 2;

				GTP_INFO("%s slide to light up the screen!",
					 direction[type]);
				doze_status = DOZE_WAKEUP;
				input_report_key(ts->input_dev, KEY_POWER, 1);
				input_sync(ts->input_dev);
				input_report_key(ts->input_dev, KEY_POWER, 0);
				input_sync(ts->input_dev);
				/* clear 0x814B */
				doze_buf[2] = 0x00;
				gtp_i2c_write(i2c_connect_client, doze_buf, 3);
			} else if (0xCC == doze_buf[2]) {
				GTP_INFO
				    ("Double click to light up the screen!");
				doze_status = DOZE_WAKEUP;
				input_report_key(ts->input_dev, KEY_POWER, 1);
				input_sync(ts->input_dev);
				input_report_key(ts->input_dev, KEY_POWER, 0);
				input_sync(ts->input_dev);
				/* clear 0x814B */
				doze_buf[2] = 0x00;
				gtp_i2c_write(i2c_connect_client, doze_buf, 3);
			} else {
				/* clear 0x814B */
				doze_buf[2] = 0x00;
				gtp_i2c_write(i2c_connect_client, doze_buf, 3);
				gtp_enter_doze(ts);
			}
		}
		if (ts->use_irq)
			gtp_irq_enable(ts);
		return;
	}
#endif

	ret = gtp_i2c_read(ts->client, point_data, 12);
	if (ret < 0) {
		GTP_ERROR("I2C transfer error. errno:%d\n ", ret);
		if (ts->use_irq)
			gtp_irq_enable(ts);
		return;
	}

	finger = point_data[GTP_ADDR_LENGTH];

#if GTP_COMPATIBLE_MODE
	/* GT9XXF */
	if ((finger == 0x00) && (CHIP_TYPE_GT9F == ts->chip_type)) {
		ret = gtp_i2c_read(ts->client, rqst_buf, 3);
		if (ret < 0) {
			GTP_ERROR("Read request status error!");
			goto exit_work_func;
		}

		switch (rqst_buf[2]) {
		case GTP_RQST_CONFIG:
			GTP_INFO("Request for config.");
			ret = gtp_send_cfg(ts->client);
			if (ret < 0)
				GTP_ERROR("Request for config unresponded!");
			else {
				rqst_buf[2] = GTP_RQST_RESPONDED;
				gtp_i2c_write(ts->client, rqst_buf, 3);
				GTP_INFO("Request for config responded!");
			}
			break;

		case GTP_RQST_BAK_REF:
			GTP_INFO("Request for backup reference.");
			ts->rqst_processing = 1;
			ret = gtp_bak_ref_proc(ts, GTP_BAK_REF_SEND);
			if (SUCCESS == ret) {
				rqst_buf[2] = GTP_RQST_RESPONDED;
				gtp_i2c_write(ts->client, rqst_buf, 3);
				ts->rqst_processing = 0;
				GTP_INFO
				    ("Request for backup reference responded!");
			} else
				GTP_ERROR
				    ("Requeet for backup reference unresponed");
			break;

		case GTP_RQST_RESET:
			GTP_INFO("Request for reset.");
			gtp_recovery_reset(ts->client);
			break;

		case GTP_RQST_MAIN_CLOCK:
			GTP_INFO("Request for main clock.");
			ts->rqst_processing = 1;
			ret = gtp_main_clk_proc(ts);
			if (FAIL == ret)
				GTP_ERROR
				    ("Request for main clock unresponded!");
			else {
				GTP_INFO("Request for main clock responded!");
				rqst_buf[2] = GTP_RQST_RESPONDED;
				gtp_i2c_write(ts->client, rqst_buf, 3);
				ts->rqst_processing = 0;
				ts->clk_chk_fs_times = 0;
			}
			break;

		default:
			GTP_INFO("Undefined request: 0x%02X", rqst_buf[2]);
			rqst_buf[2] = GTP_RQST_RESPONDED;
			gtp_i2c_write(ts->client, rqst_buf, 3);
			break;
		}
	}
#endif
	if (finger == 0x00) {
		if (ts->use_irq)
			gtp_irq_enable(ts);
		return;
	}

	if ((finger & 0x80) == 0)
		goto exit_work_func;

	touch_num = finger & 0x0f;
	if (touch_num > GTP_MAX_TOUCH)
		goto exit_work_func;

	if (touch_num > 1) {
		u8 buf[8 * GTP_MAX_TOUCH] = {
			(GTP_READ_COOR_ADDR + 10) >> 8,
			(GTP_READ_COOR_ADDR + 10) & 0xff};
		ret = gtp_i2c_read(ts->client, buf, 2 + 8 * (touch_num - 1));
		memcpy(&point_data[12], &buf[2], 8 * (touch_num - 1));
	}
#if (GTP_HAVE_TOUCH_KEY || GTP_PEN_HAVE_BUTTON)
	key_value = point_data[3 + 8 * touch_num];

	if (key_value || pre_key) {
#if GTP_PEN_HAVE_BUTTON
		if (key_value == 0x40) {
			GTP_DEBUG("BTN_STYLUS & BTN_STYLUS2 Down.");
			input_report_key(ts->pen_dev, BTN_STYLUS, 1);
			input_report_key(ts->pen_dev, BTN_STYLUS2, 1);
			pen_active = 1;
		} else if (key_value == 0x10) {
			GTP_DEBUG("BTN_STYLUS Down, BTN_STYLUS2 Up.");
			input_report_key(ts->pen_dev, BTN_STYLUS, 1);
			input_report_key(ts->pen_dev, BTN_STYLUS2, 0);
			pen_active = 1;
		} else if (key_value == 0x20) {
			GTP_DEBUG("BTN_STYLUS Up, BTN_STYLUS2 Down.");
			input_report_key(ts->pen_dev, BTN_STYLUS, 0);
			input_report_key(ts->pen_dev, BTN_STYLUS2, 1);
			pen_active = 1;
		} else {
			GTP_DEBUG("BTN_STYLUS & BTN_STYLUS2 Up.");
			input_report_key(ts->pen_dev, BTN_STYLUS, 0);
			input_report_key(ts->pen_dev, BTN_STYLUS2, 0);
			if ((pre_key == 0x40) || (pre_key == 0x20) ||
			    (pre_key == 0x10))
				pen_active = 1;
		}
		if (pen_active) {
			touch_num = 0;	/* shield pen point */
			/*pre_touch = 0; */
		}
#endif

#if GTP_HAVE_TOUCH_KEY
		if (!pre_touch) {
			for (i = 0; i < GTP_MAX_KEY_NUM; i++) {
#if GTP_DEBUG_ON
				for (ret = 0; ret < 4; ++ret) {
					if (key_codes[ret] ==
					    touch_key_array[i]) {
						GTP_DEBUG("Key: %s %s",
							  key_names[ret],
							  (key_value &
							  (0x01 << i)) ?
							  "Down" : "Up");
						break;
					}
				}
#endif
				input_report_key(ts->input_dev,
						 touch_key_array[i],
						 key_value & (0x01 << i));
			}
			touch_num = 0;	/* shield fingers */
		}
#endif
	}
#endif
	pre_key = key_value;

	GTP_DEBUG("pre_touch:%02x, finger:%02x.", pre_touch, finger);

#if GTP_ICS_SLOT_REPORT

#if GTP_WITH_PEN
	if (pre_pen && (touch_num == 0)) {
		GTP_DEBUG("Pen touch UP(Slot)!");
		gtp_pen_up(0);
		pen_active = 1;
		pre_pen = 0;
	}
#endif
	if (pre_touch || touch_num) {
		s32 pos = 0;
		u16 touch_index = 0;
		u8 report_num = 0;
		coor_data = &point_data[3];

		if (touch_num) {
			id = coor_data[pos] & 0x0F;

#if GTP_WITH_PEN
			id = coor_data[pos];
			if ((id & 0x80)) {
				GTP_DEBUG("Pen touch DOWN(Slot)!");
				input_x =
				    coor_data[pos +
					      1] | (coor_data[pos + 2] << 8);
				input_y =
				    coor_data[pos +
					      3] | (coor_data[pos + 4] << 8);
				input_w =
				    coor_data[pos +
					      5] | (coor_data[pos + 6] << 8);

				gtp_pen_down(input_x, input_y, input_w, 0);
				pre_pen = 1;
				pre_touch = 0;
				pen_active = 1;
			}
#endif

			touch_index |= (0x01 << id);
		}

		GTP_DEBUG("id = %d,touch_index = 0x%x, pre_touch = 0x%x\n", id,
			  touch_index, pre_touch);
		for (i = 0; i < GTP_MAX_TOUCH; i++) {
#if GTP_WITH_PEN
			if (pre_pen == 1)
				break;
#endif

			if ((touch_index & (0x01 << i))) {
				input_x =
				    coor_data[pos + 1] |
					 (coor_data[pos + 2] << 8);
				input_y =
				    coor_data[pos + 3] |
					(coor_data[pos + 4] << 8);
				input_w =
				    coor_data[pos + 5] |
					(coor_data[pos + 6] << 8);

				gtp_touch_down(ts, id, input_x,
						input_y, input_w);
				pre_touch |= 0x01 << i;

				report_num++;
				if (report_num < touch_num) {
					pos += 8;
					id = coor_data[pos] & 0x0F;
					touch_index |= (0x01 << id);
				}
			} else {
				gtp_touch_up(ts, i);
				pre_touch &= ~(0x01 << i);
			}
		}
	}
#else

	if (touch_num) {
		for (i = 0; i < touch_num; i++) {
			coor_data = &point_data[i * 8 + 3];

			id = coor_data[0] & 0x0F;
			input_x = coor_data[1] | (coor_data[2] << 8);
			input_y = coor_data[3] | (coor_data[4] << 8);
			input_w = coor_data[5] | (coor_data[6] << 8);

#if GTP_WITH_PEN
			id = coor_data[0];
			if (id & 0x80) {
				GTP_DEBUG("Pen touch DOWN!");
				gtp_pen_down(input_x, input_y, input_w, 0);
				pre_pen = 1;
				pen_active = 1;
				break;
			} else
#endif
				gtp_touch_down(ts, id, input_x,
						input_y, input_w);
		}
	} else if (pre_touch) {
#if GTP_WITH_PEN
		if (pre_pen == 1) {
			GTP_DEBUG("Pen touch UP!");
			gtp_pen_up(0);
			pre_pen = 0;
			pen_active = 1;
		} else
#endif
		{
			GTP_DEBUG("Touch Release!");
			gtp_touch_up(ts, 0);
		}
	}

	pre_touch = touch_num;
#endif

#if GTP_WITH_PEN
	if (pen_active) {
		pen_active = 0;
		input_sync(ts->pen_dev);
	} else
#endif
		input_sync(ts->input_dev);

exit_work_func:
	if (!ts->gtp_rawdiff_mode) {
		ret = gtp_i2c_write(ts->client, end_cmd, 3);
		if (ret < 0)
			GTP_INFO("I2C write end_cmd error!");
	}
	if (ts->use_irq)
		gtp_irq_enable(ts);
}

/*******************************************************
Function:
    Timer interrupt service routine for polling mode.
Input:
    timer: timer struct pointer
Output:
    Timer work mode.
    HRTIMER_NORESTART: no restart mode
*********************************************************/
static enum hrtimer_restart goodix_ts_timer_handler(struct hrtimer *timer)
{
	struct goodix_ts_data *ts =
	    container_of(timer, struct goodix_ts_data, timer);

	GTP_DEBUG_FUNC();

	queue_work(goodix_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, (GTP_POLL_TIME + 6) * 1000000),
		      HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

/*******************************************************
Function:
    External interrupt service routine for interrupt mode.
Input:
    irq:  interrupt number.
    dev_id: private data pointer
Output:
    Handle Result.
    IRQ_HANDLED: interrupt handled successfully
*********************************************************/
static irqreturn_t goodix_ts_irq_handler(int irq, void *dev_id)
{
	struct goodix_ts_data *ts = dev_id;

	GTP_DEBUG_FUNC();
	gtp_irq_disable(ts);
	queue_work(goodix_wq, &ts->work);

	return IRQ_HANDLED;
}

/*******************************************************
Function:
    Synchronization.
Input:
    ms: synchronization time in millisecond.
Output:
    None.
*******************************************************/
static void gtp_int_sync(struct goodix_ts_data *ts, s32 ms)
{
	gpio_direction_output(ts->pdata->irq_gpio, 0);
	msleep(ms);
	gpio_direction_input(ts->pdata->irq_gpio);
}

/*******************************************************
Function:
    Reset chip.
Input:
    ms: reset time in millisecond
Output:
    None.
*******************************************************/
void gtp_reset_guitar(struct i2c_client *client, s32 ms)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	GTP_DEBUG_FUNC();
	GTP_INFO("Guitar reset");
	/* begin select I2C slave addr */
	gpio_direction_output(ts->pdata->reset_gpio, 0);
	/* T2: > 10ms */
	msleep(ms);
	/* HIGH: 0x28/0x29, LOW: 0xBA/0xBB */
	gpio_direction_output(ts->pdata->irq_gpio, client->addr == 0x14);

	/* T3: > 100us */
	msleep(20);
	gpio_direction_output(ts->pdata->reset_gpio, 1);

	/* T4: > 5ms */
	msleep(20);
	gpio_direction_input(ts->pdata->reset_gpio);
	/* end select I2C slave addr */

#if GTP_COMPATIBLE_MODE
	if (CHIP_TYPE_GT9F == ts->chip_type)
		return;
#endif

	gtp_int_sync(ts, 50);
#if GTP_ESD_PROTECT
	gtp_init_ext_watchdog(client);
#endif
}

#if GTP_GESTURE_WAKEUP
/*******************************************************
Function:
    Enter doze mode for sliding wakeup.
Input:
    ts: goodix tp private data
Output:
    1: succeed, otherwise failed
*******************************************************/
static s8 gtp_enter_doze(struct goodix_ts_data *ts)
{
	s8 ret = -1;
	s8 retry = 0;
	u8 i2c_control_buf[3] = {
		(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 8};

	GTP_DEBUG_FUNC();

	GTP_DEBUG("Entering gesture mode.");
	while (retry++ < 5) {
		i2c_control_buf[0] = 0x80;
		i2c_control_buf[1] = 0x46;
		ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
		if (ret < 0) {
			GTP_DEBUG("failed to set doze flag into 0x8046, %d",
				  retry);
			continue;
		}
		i2c_control_buf[0] = 0x80;
		i2c_control_buf[1] = 0x40;
		ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
		if (ret > 0) {
			doze_status = DOZE_ENABLED;
			GTP_INFO("Gesture mode enabled.");
			return ret;
		}
		msleep(20);
	}
	GTP_ERROR("GTP send gesture cmd failed.");
	return ret;
}
#else
/*******************************************************
Function:
    Enter sleep mode.
Input:
    ts: private data.
Output:
    Executive outcomes.
       1: succeed, otherwise failed.
*******************************************************/
static s8 gtp_enter_sleep(struct goodix_ts_data *ts)
{
	s8 ret = -1;
	s8 retry = 0;
	u8 i2c_control_buf[3] = {
		(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 5};

#if GTP_COMPATIBLE_MODE
	u8 status_buf[3] = {0x80, 0x44};
#endif

	GTP_DEBUG_FUNC();

#if GTP_COMPATIBLE_MODE
	if (CHIP_TYPE_GT9F == ts->chip_type) {
		/* GT9XXF: host interact with ic */
		ret = gtp_i2c_read(ts->client, status_buf, 3);
		if (ret < 0)
			GTP_ERROR("failed to get backup-reference status");

		if (status_buf[2] & 0x80) {
			ret = gtp_bak_ref_proc(ts, GTP_BAK_REF_STORE);
			if (FAIL == ret)
				GTP_ERROR("failed to store bak_ref");
		}
	}
#endif

	gpio_direction_output(ts->pdata->irq_gpio, 0);
	msleep(20);

	while (retry++ < 5) {
		ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
		if (ret > 0) {
			GTP_INFO("GTP enter sleep!");
			return ret;
		}
		msleep(20);
	}
	GTP_ERROR("GTP send sleep cmd failed.");
	return ret;
}
#endif

/*******************************************************
Function:
    Wakeup from sleep.
Input:
    ts: private data.
Output:
    Executive outcomes.
    0: succeed, otherwise: failed.
*******************************************************/
static s8 gtp_wakeup_sleep(struct goodix_ts_data *ts)
{
	u8 retry = 0;
	s8 ret = -1;

	GTP_DEBUG_FUNC();

#if GTP_COMPATIBLE_MODE
	if (CHIP_TYPE_GT9F == ts->chip_type) {
		u8 opr_buf[3] = {0x41, 0x80};

		gpio_direction_output(ts->pdata->irq_gpio, 1);
		msleep(20);

		for (retry = 0; retry < 10; ++retry) {
			/* hold ss51 & dsp */
			opr_buf[2] = 0x0C;
			ret = gtp_i2c_write(ts->client, opr_buf, 3);
			if (FAIL == ret) {
				GTP_ERROR("failed to hold ss51 & dsp!");
				continue;
			}
			opr_buf[2] = 0x00;
			ret = gtp_i2c_read(ts->client, opr_buf, 3);
			if (FAIL == ret) {
				GTP_ERROR("failed to get ss51 & dsp status!");
				continue;
			}
			if (0x0C != opr_buf[2]) {
				GTP_DEBUG("ss51 & dsp not been hold, %d",
					  retry + 1);
				continue;
			}
			GTP_DEBUG("ss51 & dsp confirmed hold");

			ret = gtp_fw_startup(ts->client);
			if (FAIL == ret) {
				GTP_ERROR
				    ("startup GT9XXF failed, process recovery");
				gtp_esd_recovery(ts->client);
			}
			break;
		}
		if (retry >= 10) {
			GTP_ERROR("failed to wakeup, processing esd recovery");
			gtp_esd_recovery(ts->client);
		} else
			GTP_INFO("GT9XXF gtp wakeup success");
		return ret;
	}
#endif

#if GTP_POWER_CTRL_SLEEP
	while (retry++ < 5) {
		gtp_reset_guitar(ts->client, ts->pdata->soft_rst_dly);
		GTP_INFO("GTP wakeup sleep.");
		return 1;
	}
#else
	while (retry++ < 10) {
#if GTP_GESTURE_WAKEUP
		if (DOZE_WAKEUP != doze_status)
			GTP_INFO("Powerkey wakeup.");
		else
			GTP_INFO("Gesture wakeup.");
		doze_status = DOZE_DISABLED;
		gtp_irq_disable(ts);
		gtp_reset_guitar(ts->client, 10);
		gtp_irq_enable(ts);

#else
		gpio_direction_output(ts->pdata->irq_gpio, 1);
		msleep(20);
#endif

		ret = gtp_i2c_test(ts->client);
		if (ret > 0) {
			GTP_INFO("GTP wakeup sleep.");

#if (!GTP_GESTURE_WAKEUP)
			gtp_int_sync(ts, 25);
#if GTP_ESD_PROTECT
			gtp_init_ext_watchdog(ts->client);
#endif
#endif
			return ret;
		}
		gtp_reset_guitar(ts->client, ts->pdata->soft_rst_dly);
	}
#endif

	GTP_ERROR("GTP wakeup sleep failed.");
	return ret;
}

#if GTP_DRIVER_SEND_CFG
static s32 gtp_get_info(struct goodix_ts_data *ts)
{
	u8 opr_buf[6] = {0};
	s32 ret = 0;

	ts->abs_x_max = GTP_MAX_WIDTH;
	ts->abs_y_max = GTP_MAX_HEIGHT;
	ts->int_trigger_type = GTP_INT_TRIGGER;

	opr_buf[0] = (u8)((GTP_REG_CONFIG_DATA + 1) >> 8);
	opr_buf[1] = (u8)((GTP_REG_CONFIG_DATA + 1) & 0xFF);

	ret = gtp_i2c_read(ts->client, opr_buf, 6);
	if (ret < 0)
		return FAIL;

	ts->abs_x_max = (opr_buf[3] << 8) + opr_buf[2];
	ts->abs_y_max = (opr_buf[5] << 8) + opr_buf[4];

	opr_buf[0] = (u8)((GTP_REG_CONFIG_DATA + 6) >> 8);
	opr_buf[1] = (u8)((GTP_REG_CONFIG_DATA + 6) & 0xFF);

	ret = gtp_i2c_read(ts->client, opr_buf, 3);
	if (ret < 0)
		return FAIL;

	ts->int_trigger_type = opr_buf[2] & 0x03;
	GTP_INFO("X_MAX = %d, Y_MAX = %d, TRIGGER = 0x%02x",
		 ts->abs_x_max, ts->abs_y_max, ts->int_trigger_type);

	return SUCCESS;
}
#endif

/*******************************************************
Function:
    Initialize gtp.
Input:
    ts: goodix private data
Output:
    Executive outcomes.
    0: succeed, otherwise: failed
*******************************************************/
static s32 gtp_init_panel(struct goodix_ts_data *ts)
{
	s32 ret = -1;

#if GTP_DRIVER_SEND_CFG
	s32 i = 0;
	u8 check_sum = 0;
	u8 opr_buf[16] = {0};
	u8 sensor_id = 0;

	u8 cfg_info_group1[] = CTP_CFG_GROUP1;
	u8 cfg_info_group2[] = CTP_CFG_GROUP2;
	u8 cfg_info_group3[] = CTP_CFG_GROUP3;
	u8 cfg_info_group4[] = CTP_CFG_GROUP4;
	u8 cfg_info_group5[] = CTP_CFG_GROUP5;
	u8 cfg_info_group6[] = CTP_CFG_GROUP6;
	u8 *send_cfg_buf[] = {
		cfg_info_group1, cfg_info_group2, cfg_info_group3,
		cfg_info_group4, cfg_info_group5, cfg_info_group6
	};
	u8 cfg_info_len[] = {CFG_GROUP_LEN(cfg_info_group1),
		CFG_GROUP_LEN(cfg_info_group2),
		CFG_GROUP_LEN(cfg_info_group3),
		CFG_GROUP_LEN(cfg_info_group4),
		CFG_GROUP_LEN(cfg_info_group5),
		CFG_GROUP_LEN(cfg_info_group6)
	};

	GTP_DEBUG_FUNC();
	GTP_DEBUG("Config Groups\' Lengths: %d, %d, %d, %d, %d, %d",
		  cfg_info_len[0], cfg_info_len[1], cfg_info_len[2],
		  cfg_info_len[3], cfg_info_len[4], cfg_info_len[5]);

#if GTP_COMPATIBLE_MODE
	if (CHIP_TYPE_GT9F == ts->chip_type)
		ts->fw_error = 0;
	else
#endif
	{
		ret = gtp_i2c_read_dbl_check(ts->client, 0x41E4, opr_buf, 1);
		if (SUCCESS == ret) {
			if (opr_buf[0] != 0xBE) {
				ts->fw_error = 1;
				GTP_ERROR("Firmware error, no config sent!");
				return -EINVAL;
			}
		}
	}

	if ((!cfg_info_len[1]) && (!cfg_info_len[2]) &&
	    (!cfg_info_len[3]) && (!cfg_info_len[4]) && (!cfg_info_len[5]))
		sensor_id = 0;
	else {
#if GTP_COMPATIBLE_MODE
		msleep(50);
#endif
		ret = gtp_i2c_read_dbl_check(ts->client,
					GTP_REG_SENSOR_ID, &sensor_id, 1);
		if (SUCCESS == ret) {
			if (sensor_id >= 0x06) {
				GTP_ERROR
				    ("Invalid id(0x%02X), No Config Sent!",
				     sensor_id);
				ts->pnl_init_error = 1;
				return -EINVAL;
			}
		} else {
			GTP_ERROR("Failed to get sensor_id, No config sent!");
			ts->pnl_init_error = 1;
			return -EINVAL;
		}
		GTP_INFO("Sensor_ID: %d", sensor_id);
	}
	ts->gtp_cfg_len = cfg_info_len[sensor_id];
	GTP_INFO("CTP_CONFIG_GROUP%d used, config length: %d", sensor_id + 1,
		 ts->gtp_cfg_len);

	if (ts->gtp_cfg_len < GTP_CONFIG_MIN_LENGTH) {
		GTP_ERROR
		    ("Config Group%d is INVALID CONFIG GROUP(Len: %d)!",
		     sensor_id + 1, ts->gtp_cfg_len);
		ts->pnl_init_error = 1;
		return -EINVAL;
	}
#if GTP_COMPATIBLE_MODE
	if (CHIP_TYPE_GT9F == ts->chip_type)
		ts->fixed_cfg = 0;
	else
#endif
	{
		ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_CONFIG_DATA,
						&opr_buf[0], 1);
		if (ret == SUCCESS) {
			GTP_DEBUG
			    ("CFG_GROUP%d Config Version: %d, 0x%02X",
			     sensor_id + 1, send_cfg_buf[sensor_id][0],
			     send_cfg_buf[sensor_id][0]);
			GTP_DEBUG
			    ("IC Config Version: %d, 0x%02X",
			     opr_buf[0], opr_buf[0]);

			if (opr_buf[0] < 90) {
				/* backup group config version */
				grp_cfg_version = send_cfg_buf[sensor_id][0];
				send_cfg_buf[sensor_id][0] = 0x00;
				ts->fixed_cfg = 0;
			} else {
				/* treated as fixed config, not send config */
				GTP_INFO
				    ("Ic with config version (%d, 0x%02X)",
				     opr_buf[0], opr_buf[0]);
				ts->fixed_cfg = 1;
				gtp_get_info(ts);
				return 0;
			}
		} else {
			GTP_ERROR
			    ("Failed to get ic config version!No config sent!");
			return -EINVAL;
		}
	}

	memset(&config[GTP_ADDR_LENGTH], 0, GTP_CONFIG_MAX_LENGTH);
	memcpy(&config[GTP_ADDR_LENGTH], send_cfg_buf[sensor_id],
	       ts->gtp_cfg_len);

#if GTP_CUSTOM_CFG
	config[RESOLUTION_LOC] = (u8)GTP_MAX_WIDTH;
	config[RESOLUTION_LOC + 1] = (u8)(GTP_MAX_WIDTH >> 8);
	config[RESOLUTION_LOC + 2] = (u8)GTP_MAX_HEIGHT;
	config[RESOLUTION_LOC + 3] = (u8)(GTP_MAX_HEIGHT >> 8);

	if (GTP_INT_TRIGGER == 0)	/* RISING */
		config[TRIGGER_LOC] &= 0xfe;
	else if (GTP_INT_TRIGGER == 1)	/* FALLING */
		config[TRIGGER_LOC] |= 0x01;
#endif /* GTP_CUSTOM_CFG */

	check_sum = 0;
	for (i = GTP_ADDR_LENGTH; i < ts->gtp_cfg_len; i++)
		check_sum += config[i];
	config[ts->gtp_cfg_len] = (~check_sum) + 1;

#else /* driver not send config */

	ts->gtp_cfg_len = GTP_CONFIG_MAX_LENGTH;
	ret = gtp_i2c_read(ts->client, config,
			ts->gtp_cfg_len + GTP_ADDR_LENGTH);
	if (ret < 0) {
		GTP_ERROR
		    ("Read Config Fail, Use Default Resolution & INT Trigger!");
		ts->abs_x_max = GTP_MAX_WIDTH;
		ts->abs_y_max = GTP_MAX_HEIGHT;
		ts->int_trigger_type = GTP_INT_TRIGGER;
	}
#endif /* GTP_DRIVER_SEND_CFG */

	if ((ts->abs_x_max == 0) && (ts->abs_y_max == 0)) {
		ts->abs_x_max =
		    (config[RESOLUTION_LOC + 1] << 8) + config[RESOLUTION_LOC];
		ts->abs_y_max = (config[RESOLUTION_LOC + 3] << 8)
				+ config[RESOLUTION_LOC + 2];
		ts->int_trigger_type = (config[TRIGGER_LOC]) & 0x03;
	}
#if GTP_COMPATIBLE_MODE
	if (CHIP_TYPE_GT9F == ts->chip_type) {
		u8 sensor_num = 0;
		u8 driver_num = 0;
		u8 have_key = 0;

		have_key =
		    (config[GTP_REG_HAVE_KEY - GTP_REG_CONFIG_DATA + 2] & 0x01);

		if (1 == ts->is_950) {
			driver_num = config[GTP_REG_MATRIX_DRVNUM
					- GTP_REG_CONFIG_DATA + 2];
			sensor_num = config[GTP_REG_MATRIX_SENNUM
					- GTP_REG_CONFIG_DATA + 2];
			if (have_key)
				driver_num--;
			ts->bak_ref_len =
			    (driver_num * (sensor_num - 1) + 2) * 2 * 6;
		} else {
			driver_num = (config[CFG_LOC_DRVA_NUM] & 0x1F)
					+ (config[CFG_LOC_DRVB_NUM] & 0x1F);
			if (have_key)
				driver_num--;
			sensor_num = (config[CFG_LOC_SENS_NUM] & 0x0F)
				+ ((config[CFG_LOC_SENS_NUM] >> 4) & 0x0F);
			ts->bak_ref_len =
			    (driver_num * (sensor_num - 2) + 2) * 2;
		}

		GTP_INFO
		    ("%d * %d(key: %d), X_MAX: %d, Y_MAX: %d, TRIGGER: 0x%02x",
		     driver_num, sensor_num, have_key, ts->abs_x_max,
		     ts->abs_y_max, ts->int_trigger_type);
		return 0;
	} else
#endif
	{
#if GTP_DRIVER_SEND_CFG
		ret = gtp_send_cfg(ts->client);
		if (ret < 0)
			GTP_ERROR("Send config error.");
		/* set config version to CTP_CFG_GROUP
		  * for resume to send config
		  */
		config[GTP_ADDR_LENGTH] = grp_cfg_version;
		check_sum = 0;
		for (i = GTP_ADDR_LENGTH; i < ts->gtp_cfg_len; i++)
			check_sum += config[i];
		config[ts->gtp_cfg_len] = (~check_sum) + 1;
#endif
		GTP_INFO("X_MAX: %d, Y_MAX: %d, TRIGGER: 0x%02x", ts->abs_x_max,
			 ts->abs_y_max, ts->int_trigger_type);
	}

	msleep(20);
	return 0;
}

static ssize_t gt91xx_config_read_proc(struct file *file, char __user *page,
				       size_t size, loff_t *ppos)
{
	char *ptr = page;
	char temp_data[GTP_CONFIG_MAX_LENGTH + 2] = {0x80, 0x47};
	int i;

	if (*ppos)
		return 0;

	ptr += snprintf(ptr, 36, "==== GT9XX config init value====\n");
	for (i = 0; i < GTP_CONFIG_MAX_LENGTH; i++) {
		ptr += snprintf(ptr, 4, "0x%02X ", config[i + 2]);
		if (i % 8 == 7)
			ptr += snprintf(ptr, 4, "\n");
	}

	ptr += snprintf(ptr, 4, "\n");
	ptr += snprintf(ptr, 36, "==== GT9XX config real value====\n");
	gtp_i2c_read(i2c_connect_client, temp_data, GTP_CONFIG_MAX_LENGTH + 2);
	for (i = 0; i < GTP_CONFIG_MAX_LENGTH; i++) {
		ptr += snprintf(ptr, 4, "0x%02X ", temp_data[i + 2]);
		if (i % 8 == 7)
			ptr += snprintf(ptr, 4, "\n");
	}
	*ppos += ptr - page;
	return (ssize_t)(ptr - page);
}

static ssize_t gt91xx_config_write_proc(struct file *filp,
					const char __user *buffer,
					size_t count, loff_t *off)
{
	s32 ret = 0;

	GTP_DEBUG("write count %d\n",(int)count);

	if (count > GTP_CONFIG_MAX_LENGTH) {
		GTP_ERROR("size not match [%d:%d]\n", GTP_CONFIG_MAX_LENGTH,
			  (int)count);
		return -EFAULT;
	}

	if (copy_from_user(&config[2], buffer, count)) {
		GTP_ERROR("copy from user fail\n");
		return -EFAULT;
	}

	ret = gtp_send_cfg(i2c_connect_client);
	if (ret < 0)
		GTP_ERROR("send config failed.");

	return count;
}

/*******************************************************
Function:
    Read chip version.
Input:
    client:  i2c device
    version: buffer to keep ic firmware version
Output:
    read operation return.
    2: succeed, otherwise: failed
*******************************************************/
s32 gtp_read_version(struct i2c_client *client, u16 *version)
{
	s32 ret = -1;
	u8 buf[8] = {GTP_REG_VERSION >> 8, GTP_REG_VERSION & 0xff};

	GTP_DEBUG_FUNC();

	ret = gtp_i2c_read(client, buf, sizeof(buf));
	if (ret < 0) {
		GTP_ERROR("GTP read version failed");
		return ret;
	}

	if (version)
		*version = (buf[7] << 8) | buf[6];

	if (buf[5] == 0x00)
		GTP_INFO("IC Version: %c%c%c_%02x%02x", buf[2], buf[3],
			buf[4], buf[7], buf[6]);
	else
		GTP_INFO("IC Version: %c%c%c%c_%02x%02x", buf[2], buf[3],
			buf[4], buf[5], buf[7], buf[6]);

	return ret;
}

/*******************************************************
Function:
    I2c test Function.
Input:
    client:i2c client.
Output:
    Executive outcomes.
    2: succeed, otherwise failed.
*******************************************************/
static s8 gtp_i2c_test(struct i2c_client *client)
{
	u8 test[3] = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};
	u8 retry = 0;
	s8 ret = -1;

	GTP_DEBUG_FUNC();

	while (retry++ < 5) {
		ret = gtp_i2c_read(client, test, 3);
		if (ret > 0)
			return ret;
		GTP_ERROR("GTP i2c test failed time %d.", retry);
		msleep(20);
	}
	return ret;
}

/*******************************************************
Function:
    Request gpio(INT & RST) ports.
Input:
    ts: private data.
Output:
    Executive outcomes.
    >= 0: succeed, < 0: failed
*******************************************************/
static s8 gtp_request_io_port(struct goodix_ts_data *ts)
{
	s32 ret = 0;

	GTP_DEBUG_FUNC();

#ifndef FEATURE_QRD8X26_UNISCOPE_MODIFY
		GTP_ERROR("gpio_request start!");
	ret = gpio_request(ts->pdata->irq_gpio, "GTP_INT_IRQ");
	if (ret < 0) {
		GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d",
			  (s32)ts->pdata->irq_gpio, ret);
		ret = -ENODEV;
	} else {
		ret = gpio_direction_input(ts->pdata->irq_gpio);
		if (ret < 0)
			gpio_free(ts->pdata->irq_gpio);
		ts->client->irq = gpio_to_irq(ts->pdata->irq_gpio);
	}

	ret = gpio_request(ts->pdata->reset_gpio, "GTP_RST_PORT");
	if (ret < 0) {
		GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d",
			  (s32)ts->pdata->reset_gpio, ret);
		ret = -ENODEV;
	}
		GTP_ERROR("gpio_request end!");
#endif
	ret = gpio_direction_input(ts->pdata->reset_gpio);
	if (ret < 0){
		GTP_ERROR("gpio_direction_input ret <0!");
		gpio_free(ts->pdata->reset_gpio);
	}

	gtp_reset_guitar(ts->client, 20);

	return ret;
}

/*******************************************************
Function:
    Request interrupt.
Input:
    ts: private data.
Output:
    Executive outcomes.
    0: succeed, -1: failed.
*******************************************************/
static s8 gtp_request_irq(struct goodix_ts_data *ts)
{
	s32 ret = -1;
	const u8 irq_table[] = GTP_IRQ_TAB;

	GTP_DEBUG_FUNC();
	GTP_DEBUG("INT trigger type:%x", ts->int_trigger_type);
	GTP_DEBUG("ts->client->irq:%d", ts->client->irq);

	ret = request_irq(ts->client->irq, goodix_ts_irq_handler,
			  irq_table[ts->int_trigger_type],
			  ts->client->name, ts);
	if (ret) {
		GTP_ERROR("Request IRQ failed!ERRNO:%d.", ret);
		gpio_direction_input(ts->pdata->irq_gpio);
		gpio_free(ts->pdata->irq_gpio);

		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = goodix_ts_timer_handler;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		return ret;
	} else {
		gtp_irq_disable(ts);
		ts->use_irq = 1;
		return 0;
	}
}

/*******************************************************
Function:
    Request input device Function.
Input:
    ts:private data.
Output:
    Executive outcomes.
    0: succeed, otherwise: failed.
*******************************************************/
static s8 gtp_request_input_dev(struct goodix_ts_data *ts)
{
	s8 ret = -1;
	s8 phys[32];
#if GTP_HAVE_TOUCH_KEY
	u8 index = 0;
#endif

	GTP_DEBUG_FUNC();

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		GTP_ERROR("Failed to allocate input device.");
		return -ENOMEM;
	}

	ts->input_dev->evbit[0] =
	    BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
#if GTP_ICS_SLOT_REPORT
	input_mt_init_slots(ts->input_dev, 16, 0);
#else
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#endif
	__set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

#if GTP_HAVE_TOUCH_KEY
	for (index = 0; index < GTP_MAX_KEY_NUM; index++)
		input_set_capability(ts->input_dev, EV_KEY,
				     touch_key_array[index]);
#endif

#if GTP_GESTURE_WAKEUP
	input_set_capability(ts->input_dev, EV_KEY, KEY_POWER);
#endif

#if GTP_CHANGE_X2Y
	GTP_SWAP(ts->abs_x_max, ts->abs_y_max);
#endif

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max,
			     0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max,
			     0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);

	snprintf(phys, 32, "input/ts");
	ts->input_dev->name = goodix_ts_name;
	ts->input_dev->phys = phys;
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0xDEAD;
	ts->input_dev->id.product = 0xBEEF;
	ts->input_dev->id.version = 10427;

	ret = input_register_device(ts->input_dev);
	if (ret) {
		GTP_ERROR("Register %s input device failed",
			  ts->input_dev->name);
		return -ENODEV;
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = goodix_ts_early_suspend;
	ts->early_suspend.resume = goodix_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

#if GTP_WITH_PEN
	gtp_pen_init(ts);
#endif

	return 0;
}

/************** For GT9XXF Start *************/
#if GTP_COMPATIBLE_MODE
s32 gtp_fw_startup(struct i2c_client *client)
{
	u8 opr_buf[4];
	s32 ret = 0;
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	/* init sw WDT */
	opr_buf[0] = 0xAA;
	ret = i2c_write_bytes(client, 0x8041, opr_buf, 1);
	if (ret < 0)
		return FAIL;

	/* release SS51 & DSP */
	opr_buf[0] = 0x00;
	ret = i2c_write_bytes(client, 0x4180, opr_buf, 1);
	if (ret < 0)
		return FAIL;

	/* int sync */
	gtp_int_sync(ts, 25);

	/* check fw run status */
	ret = i2c_read_bytes(client, 0x8041, opr_buf, 1);
	if (ret < 0)
		return FAIL;

	if (0xAA == opr_buf[0]) {
		GTP_ERROR("IC works abnormally,startup failed.");
		return FAIL;
	}

	GTP_INFO("IC works normally, Startup success.");
	opr_buf[0] = 0xAA;
	i2c_write_bytes(client, 0x8041, opr_buf, 1);
	return SUCCESS;
}

static s32 gtp_esd_recovery(struct i2c_client *client)
{
	s32 retry = 0;
	s32 ret = 0;
	struct goodix_ts_data *ts;

	ts = i2c_get_clientdata(client);

	gtp_irq_disable(ts);

	GTP_INFO("GT9XXF esd recovery mode");
	for (retry = 0; retry < 5; retry++) {
		ret = gup_fw_download_proc(NULL, GTP_FL_ESD_RECOVERY);
		if (FAIL == ret) {
			GTP_ERROR("esd recovery failed %d", retry + 1);
			continue;
		}
		ret = gtp_fw_startup(ts->client);
		if (FAIL == ret) {
			GTP_ERROR("GT9XXF start up failed %d", retry + 1);
			continue;
		}
		break;
	}
	gtp_irq_enable(ts);

	if (retry >= 5) {
		GTP_ERROR("failed to esd recovery");
		return FAIL;
	}

	GTP_INFO("Esd recovery successful");
	return SUCCESS;
}

void gtp_recovery_reset(struct i2c_client *client)
{
#if GTP_ESD_PROTECT
	gtp_esd_switch(client, SWITCH_OFF);
#endif
	GTP_DEBUG_FUNC();

	gtp_esd_recovery(client);

#if GTP_ESD_PROTECT
	gtp_esd_switch(client, SWITCH_ON);
#endif
}

static s32 gtp_bak_ref_proc(struct goodix_ts_data *ts, u8 mode)
{
	s32 ret = 0;
	s32 i = 0;
	s32 j = 0;
	u16 ref_sum = 0;
	u16 learn_cnt = 0;
	u16 chksum = 0;
	s32 ref_seg_len = 0;
	s32 ref_grps = 0;
	struct file *ref_filp = NULL;
	u8 *p_bak_ref;
	struct filename name;

	ret = gup_check_fs_mounted("/data");
	if (FAIL == ret) {
		ts->ref_chk_fs_times++;
		GTP_DEBUG("Ref check /data times/MAX_TIMES: %d / %d",
			  ts->ref_chk_fs_times, GTP_CHK_FS_MNT_MAX);
		if (ts->ref_chk_fs_times < GTP_CHK_FS_MNT_MAX) {
			msleep(50);
			GTP_INFO("/data not mounted.");
			return FAIL;
		}
		GTP_INFO("check /data mount timeout...");
	} else
		GTP_INFO("/data mounted!!!(%d/%d)", ts->ref_chk_fs_times,
			 GTP_CHK_FS_MNT_MAX);

	p_bak_ref = kzalloc(ts->bak_ref_len, GFP_KERNEL);
	if (NULL == p_bak_ref) {
		GTP_ERROR("Allocate memory for p_bak_ref failed!");
		return FAIL;
	}

	if (ts->is_950) {
		ref_seg_len = ts->bak_ref_len / 6;
		ref_grps = 6;
	} else {
		ref_seg_len = ts->bak_ref_len;
		ref_grps = 1;
	}

	name.name = GTP_BAK_REF_PATH;
	ref_filp = file_open_name(&name, O_RDWR | O_CREAT, 0666);
	if (IS_ERR(ref_filp)) {
		GTP_ERROR("Failed to open/create %s.", GTP_BAK_REF_PATH);
		if (GTP_BAK_REF_SEND == mode)
			goto bak_ref_default;
		else
			goto bak_ref_exit;
	}

	switch (mode) {
	case GTP_BAK_REF_SEND:
		GTP_INFO("Send backup-reference");
		ref_filp->f_op->llseek(ref_filp, 0, SEEK_SET);
		ret = ref_filp->f_op->read(ref_filp,
			(char *)p_bak_ref, ts->bak_ref_len, &ref_filp->f_pos);
		if (ret < 0) {
			GTP_ERROR("failed to read bak_ref info from file");
			goto bak_ref_default;
		}
		for (j = 0; j < ref_grps; ++j) {
			ref_sum = 0;
			for (i = 0; i < (ref_seg_len); i += 2)
				ref_sum +=
				    (p_bak_ref[i + j * ref_seg_len] << 8) +
				    p_bak_ref[i + 1 + j * ref_seg_len];
			learn_cnt =
			    (p_bak_ref[j * ref_seg_len + ref_seg_len - 4] << 8)
			    + (p_bak_ref[j * ref_seg_len + ref_seg_len - 3]);
			chksum =
			    (p_bak_ref[j * ref_seg_len + ref_seg_len - 2] << 8)
			    + (p_bak_ref[j * ref_seg_len + ref_seg_len - 1]);
			GTP_DEBUG("learn count = %d", learn_cnt);
			GTP_DEBUG("chksum = %d", chksum);
			GTP_DEBUG("ref_sum = 0x%04X", ref_sum & 0xFFFF);

			if (1 != ref_sum) {
				GTP_INFO
				    ("wrong chksum for bak_ref, reset to 0x00");
				memset(&p_bak_ref[j * ref_seg_len], 0,
				       ref_seg_len);
				p_bak_ref[ref_seg_len + j * ref_seg_len - 1] =
				    0x01;
			} else {
				if (j == (ref_grps - 1))
					GTP_INFO
					    ("backup-reference data in %s used",
					     GTP_BAK_REF_PATH);
			}
		}
		ret = i2c_write_bytes(ts->client, GTP_REG_BAK_REF, p_bak_ref,
					ts->bak_ref_len);
		if (FAIL == ret) {
			GTP_ERROR("send bak_ref failed: iic comm error");
			goto bak_ref_exit;
		}
		break;

	case GTP_BAK_REF_STORE:
		GTP_INFO("Store backup-reference");
		ret = i2c_read_bytes(ts->client, GTP_REG_BAK_REF, p_bak_ref,
					ts->bak_ref_len);
		if (ret < 0) {
			GTP_ERROR("read bak_ref info failed, send default");
			goto bak_ref_default;
		}
		ref_filp->f_op->llseek(ref_filp, 0, SEEK_SET);
		ref_filp->f_op->write(ref_filp, (char *)p_bak_ref,
				      ts->bak_ref_len, &ref_filp->f_pos);
		break;

	default:
		GTP_ERROR("invalid backup-reference request");
		break;
	}
	ret = SUCCESS;
	goto bak_ref_exit;

bak_ref_default:
	for (j = 0; j < ref_grps; ++j) {
		memset(&p_bak_ref[j * ref_seg_len], 0, ref_seg_len);
		p_bak_ref[j * ref_seg_len + ref_seg_len - 1] = 0x01;
	}
	ret = i2c_write_bytes(ts->client, GTP_REG_BAK_REF, p_bak_ref,
				ts->bak_ref_len);
	if (!IS_ERR(ref_filp)) {
		GTP_INFO("write backup-reference data into %s",
			 GTP_BAK_REF_PATH);
		ref_filp->f_op->llseek(ref_filp, 0, SEEK_SET);
		ref_filp->f_op->write(ref_filp, (char *)p_bak_ref,
				      ts->bak_ref_len, &ref_filp->f_pos);
	}
	if (ret == FAIL)
		GTP_ERROR("failed to load the default backup reference");

bak_ref_exit:
	kfree(p_bak_ref);
	if (ref_filp && !IS_ERR(ref_filp))
		filp_close(ref_filp, NULL);
	return ret;
}

static s32 gtp_verify_main_clk(u8 *p_main_clk)
{
	u8 chksum = 0;
	u8 main_clock = p_main_clk[0];
	s32 i = 0;

	if (main_clock < 50 || main_clock > 120)
		return FAIL;

	for (i = 0; i < 5; ++i) {
		if (main_clock != p_main_clk[i])
			return FAIL;
		chksum += p_main_clk[i];
	}
	chksum += p_main_clk[5];
	if ((chksum) == 0)
		return SUCCESS;
	return FAIL;
}

static s32 gtp_main_clk_proc(struct goodix_ts_data *ts)
{
	s32 ret = 0;
	s32 i = 0;
	s32 clk_chksum = 0;
	struct file *clk_filp = NULL;
	u8 p_main_clk[6] = {0};
	struct filename name;

	ret = gup_check_fs_mounted("/data");
	if (FAIL == ret) {
		ts->clk_chk_fs_times++;
		GTP_DEBUG("Clock check /data times/MAX_TIMES: %d / %d",
			  ts->clk_chk_fs_times, GTP_CHK_FS_MNT_MAX);
		if (ts->clk_chk_fs_times < GTP_CHK_FS_MNT_MAX) {
			msleep(50);
			GTP_INFO("/data not mounted.");
			return FAIL;
		}
		GTP_INFO("Check /data mount timeout!");
	} else
		GTP_INFO("/data mounted!!!(%d/%d)", ts->clk_chk_fs_times,
			 GTP_CHK_FS_MNT_MAX);

	name.name = GTP_MAIN_CLK_PATH;
	clk_filp = file_open_name(&name, O_RDWR | O_CREAT, 0666);
	if (IS_ERR(clk_filp))
		GTP_ERROR("%s is unavailable, calculate main clock",
			  GTP_MAIN_CLK_PATH);
	else {
		clk_filp->f_op->llseek(clk_filp, 0, SEEK_SET);
		clk_filp->f_op->read(clk_filp, (char *)p_main_clk, 6,
				     &clk_filp->f_pos);

		ret = gtp_verify_main_clk(p_main_clk);
		if (FAIL == ret) {
			/* recalculate main clock
			  * & rewrite main clock data to file
			  */
			GTP_ERROR
			    ("main clock data in %s is wrong,",
			     GTP_MAIN_CLK_PATH);
		} else {
			GTP_INFO
			    ("main clock data in %s used, main clock freq: %d",
			     GTP_MAIN_CLK_PATH, p_main_clk[0]);
			filp_close(clk_filp, NULL);
			goto update_main_clk;
		}
	}

#if GTP_ESD_PROTECT
	gtp_esd_switch(ts->client, SWITCH_OFF);
#endif
	ret = gup_clk_calibration();
	gtp_esd_recovery(ts->client);

#if GTP_ESD_PROTECT
	gtp_esd_switch(ts->client, SWITCH_ON);
#endif

	GTP_INFO("calibrate main clock: %d", ret);
	if (ret < 50 || ret > 120) {
		GTP_ERROR("wrong main clock: %d", ret);
		goto exit_main_clk;
	}

	for (i = 0; i < 5; ++i) {
		p_main_clk[i] = ret;
		clk_chksum += p_main_clk[i];
	}
	p_main_clk[5] = 0 - clk_chksum;

	if (!IS_ERR(clk_filp)) {
		GTP_DEBUG("write main clock data into %s", GTP_MAIN_CLK_PATH);
		clk_filp->f_op->llseek(clk_filp, 0, SEEK_SET);
		clk_filp->f_op->write(clk_filp, (char *)p_main_clk, 6,
				      &clk_filp->f_pos);
		filp_close(clk_filp, NULL);
	}

update_main_clk:
	ret = i2c_write_bytes(ts->client, GTP_REG_MAIN_CLK, p_main_clk, 6);
	if (FAIL == ret) {
		GTP_ERROR("update main clock failed!");
		return FAIL;
	}
	return SUCCESS;

exit_main_clk:
	if (!IS_ERR(clk_filp))
		filp_close(clk_filp, NULL);
	return FAIL;
}

static s32 gtp_gt9xxf_init(struct i2c_client *client)
{
	s32 ret = 0;

	ret = gup_fw_download_proc(NULL, GTP_FL_FW_BURN);
	if (FAIL == ret)
		return FAIL;

	ret = gtp_fw_startup(client);
	if (FAIL == ret)
		return FAIL;

	return SUCCESS;
}

static void gtp_get_chip_type(struct goodix_ts_data *ts)
{
	u8 opr_buf[10] = {0x00};
	s32 ret = 0;

	msleep(20);

	ret =
	    gtp_i2c_read_dbl_check(ts->client, GTP_REG_CHIP_TYPE, opr_buf, 10);
	if (FAIL == ret) {
		GTP_ERROR("Failed to get chip-type");
		ts->chip_type = CHIP_TYPE_GT9;
		return;
	}

	if (!memcmp(opr_buf, "GOODIX_GT9", 10))
		ts->chip_type = CHIP_TYPE_GT9;
	else	/* GT9XXF */
		ts->chip_type = CHIP_TYPE_GT9F;
	GTP_INFO("Chip Type: %s",
		 (ts->chip_type ==
		  CHIP_TYPE_GT9) ? "GOODIX_GT9" : "GOODIX_GT9F");
}
#endif

static int goodix_ts_get_dt_coords(struct device *dev, char *name,
				   struct goodix_ts_platform_data *pdata)
{
	struct property *prop;
	struct device_node *np = dev->of_node;
	int rc;
	u32 coords[GOODIX_COORDS_ARR_SIZE];

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	rc = of_property_read_u32_array(np, name, coords,
					GOODIX_COORDS_ARR_SIZE);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read %s\n", name);
		return rc;
	}

	if (!strcmp(name, "goodix,panel-coords")) {
		pdata->panel_minx = coords[0];
		pdata->panel_miny = coords[1];
		pdata->panel_maxx = coords[2];
		pdata->panel_maxy = coords[3];
	} else if (!strcmp(name, "goodix,display-coords")) {
		pdata->x_min = coords[0];
		pdata->y_min = coords[1];
		pdata->x_max = coords[2];
		pdata->y_max = coords[3];
	} else {
		dev_err(dev, "unsupported property %s\n", name);
		return -EINVAL;
	}

	return 0;
}

static int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
	return (regulator_count_voltages(reg) > 0) ?
	    regulator_set_optimum_mode(reg, load_uA) : 0;
}

/**
 * goodix_power_on - Turn device power ON
 * @ts: driver private data
 *
 * Returns zero on success, else an error.
 */
static int goodix_power_on(struct goodix_ts_data *ts)
{
	int ret;
#ifdef FEATURE_QRD8X26_UNISCOPE_MODIFY
	int err = 0;
#endif

	if (ts->power_on) {
		dev_info(&ts->client->dev, "Device already power on\n");
		return 0;
	}
#ifdef FEATURE_QRD8X26_UNISCOPE_MODIFY
	/* VDD 2.8V Power on */
	if (gpio_is_valid(GPIO_VDD_2_8V)) {
		err = gpio_direction_output(GPIO_VDD_2_8V, 1);
		if (err) {
			dev_err(&ts->client->dev,
				"set_direction for ft5x06 2.8V gpio failed\n");
			goto free_vdd_gpio;
		}
	}

	msleep(20);
#endif

	if (!IS_ERR(ts->vdd)) {
		ret = regulator_set_voltage(ts->vdd, GOODIX_VTG_MIN_UV,
					    GOODIX_VTG_MAX_UV);
		if (ret) {
			dev_err(&ts->client->dev,
				"Regulator set_vtg failed vdd ret=%d\n", ret);
			goto err_set_vtg_vdd;
		}
		ret = reg_set_optimum_mode_check(ts->vdd,
						 GOODIX_VDD_LOAD_MAX_UA);
		if (ret < 0) {
			dev_err(&ts->client->dev,
				"Regulator vdd set_opt failed rc=%d\n", ret);
			goto err_set_opt_vdd;
		}
		ret = regulator_enable(ts->vdd);
		if (ret) {
			dev_err(&ts->client->dev,
				"Regulator vdd enable failed ret=%d\n", ret);
			goto err_enable_vdd;
		}
	}

	if (!IS_ERR(ts->vcc_i2c)) {
		ret = regulator_set_voltage(ts->vcc_i2c, GOODIX_I2C_VTG_MIN_UV,
					    GOODIX_I2C_VTG_MAX_UV);
		if (ret) {
			dev_err(&ts->client->dev,
				"Regulator set_vtg failed vcc_i2c ret=%d\n",
				ret);
			goto err_set_vtg_vcc_i2c;
		}
		ret = reg_set_optimum_mode_check(ts->vcc_i2c,
						 GOODIX_VIO_LOAD_MAX_UA);
		if (ret < 0) {
			dev_err(&ts->client->dev,
				"Regulator vcc_i2c set_opt failed rc=%d\n",
				ret);
			goto err_set_opt_vcc_i2c;
		}
		ret = regulator_enable(ts->vcc_i2c);
		if (ret) {
			dev_err(&ts->client->dev,
				"Regulator vcc_i2c enable failed ret=%d\n",
				ret);
			regulator_disable(ts->vdd);
			goto err_enable_vcc_i2c;
		}
	}

	ts->power_on = true;
	return 0;

err_enable_vcc_i2c:
err_set_opt_vcc_i2c:
	if (!IS_ERR(ts->vcc_i2c))
		regulator_set_voltage(ts->vcc_i2c, 0, GOODIX_I2C_VTG_MAX_UV);
err_set_vtg_vcc_i2c:
	if (!IS_ERR(ts->vdd))
		regulator_disable(ts->vdd);
err_enable_vdd:
err_set_opt_vdd:
	if (!IS_ERR(ts->vdd))
		regulator_set_voltage(ts->vdd, 0, GOODIX_VTG_MAX_UV);
err_set_vtg_vdd:
	ts->power_on = false;
#ifdef FEATURE_QRD8X26_UNISCOPE_MODIFY
free_vdd_gpio:
	if (gpio_is_valid(GPIO_VDD_2_8V))
		gpio_free(GPIO_VDD_2_8V);
#endif
	return ret;
}

/**
 * goodix_power_off - Turn device power OFF
 * @ts: driver private data
 *
 * Returns zero on success, else an error.
 */
static int goodix_power_off(struct goodix_ts_data *ts)
{
	int ret;
#ifdef FEATURE_QRD8X26_UNISCOPE_MODIFY
	int err = 0;
#endif

	if (!ts->power_on) {
		dev_info(&ts->client->dev, "Device already power off\n");
		return 0;
	}
#ifdef FEATURE_QRD8X26_UNISCOPE_MODIFY
	err = gpio_direction_output(GPIO_VDD_2_8V, 0);
	if (err) {
		dev_err(&ts->client->dev,
			"set_direction for ft5x06 2.8V gpio failed\n");
	}
#endif

	if (!IS_ERR(ts->vcc_i2c)) {
		ret = regulator_set_voltage(ts->vcc_i2c, 0,
					    GOODIX_I2C_VTG_MAX_UV);
		if (ret < 0)
			dev_err(&ts->client->dev,
				"Regulator vcc_i2c set_vtg failed ret=%d\n",
				ret);
		ret = regulator_disable(ts->vcc_i2c);
		if (ret)
			dev_err(&ts->client->dev,
				"Regulator vcc_i2c disable failed ret=%d\n",
				ret);
	}

	if (!IS_ERR(ts->vdd)) {
		ret = regulator_set_voltage(ts->vdd, 0, GOODIX_VTG_MAX_UV);
		if (ret < 0)
			dev_err(&ts->client->dev,
				"Regulator vdd set_vtg failed ret=%d\n", ret);
		ret = regulator_disable(ts->vdd);
		if (ret)
			dev_err(&ts->client->dev,
				"Regulator vdd disable failed ret=%d\n", ret);
	}

	ts->power_on = false;
	return 0;
}

/**
 * goodix_power_init - Initialize device power
 * @ts: driver private data
 *
 * Returns zero on success, else an error.
 */
static int goodix_power_init(struct goodix_ts_data *ts)
{
	int ret;
#ifdef FEATURE_QRD8X26_UNISCOPE_MODIFY
	int err = 0;
#endif

	ts->vdd = regulator_get(&ts->client->dev, "vdd");
	if (IS_ERR(ts->vdd)) {
		ret = PTR_ERR(ts->vdd);
		dev_info(&ts->client->dev,
			 "Regulator get failed vdd ret=%d\n", ret);
	}

	ts->vcc_i2c = regulator_get(&ts->client->dev, "vcc-i2c");
	if (IS_ERR(ts->vcc_i2c)) {
		ret = PTR_ERR(ts->vcc_i2c);
		dev_info(&ts->client->dev,
			 "Regulator get failed vcc_i2c ret=%d\n", ret);
	}
#ifdef FEATURE_QRD8X26_UNISCOPE_MODIFY
	if (regulator_count_voltages(ts->vcc_i2c) > 0) {
		err = regulator_set_voltage(ts->vcc_i2c, GOODIX_I2C_VTG_MIN_UV,
					    GOODIX_I2C_VTG_MIN_UV);
		if (err) {
			dev_err(&ts->client->desv,
				"Regulator set_vtg failed vcc_i2c rc=%d\n",
				err);
			goto err_set_vtg_vcc_i2c;
		}
	}

	err = gpio_request(GPIO_VDD_2_8V, "ft5x06 2.8V");
	if (err)
		dev_err(&ts->client->dev, "ft5x06 2.8V request failed");

	/* request GTP GPIO --Timing 2014-2-14 */
	ret = gpio_request(ts->pdata->irq_gpio, "GTP_INT_IRQ");
	if (ret < 0) {
		GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d",
			  (s32)ts->pdata->irq_gpio, ret);
		ret = -ENODEV;
	} else {
		ret = gpio_direction_input(ts->pdata->irq_gpio);
		if (ret < 0)
			gpio_free(ts->pdata->irq_gpio);
		ts->client->irq = gpio_to_irq(ts->pdata->irq_gpio);
	}

	ret = gpio_request(ts->pdata->reset_gpio, "GTP_RST_PORT");
	if (ret < 0) {
		GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d",
			  (s32)ts->pdata->reset_gpio, ret);
		ret = -ENODEV;
	}
	/* RST and INT output low for poweron timing  --Timing 2014-2-14 */
	err = gpio_direction_output(ts->pdata->irq_gpio, 0);
	err = gpio_direction_output(ts->pdata->reset_gpio, 0);
	err = gpio_direction_output(GPIO_VDD_2_8V, 0);
	msleep(200);

err_set_vtg_vcc_i2c:
	if (!IS_ERR(ts->vdd))
		regulator_disable(ts->vdd);
#endif
	return 0;
}

/**
 * goodix_power_deinit - Deinitialize device power
 * @ts: driver private data
 *
 * Returns zero on success, else an error.
 */
static int goodix_power_deinit(struct goodix_ts_data *ts)
{
	regulator_put(ts->vdd);
	regulator_put(ts->vcc_i2c);
	return 0;
}

/************* For GT9XXF End ************/
static ssize_t gtp_fw_name_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct goodix_ts_data *ts = dev_get_drvdata(dev);

	if (!strlen(ts->fw_name))
		return snprintf(buf, GTP_FW_NAME_MAXSIZE - 1,
				"No fw name has been given.");
	return snprintf(buf, GTP_FW_NAME_MAXSIZE - 1,
				"%s\n", ts->fw_name);
}

static ssize_t gtp_fw_name_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	struct goodix_ts_data *ts = dev_get_drvdata(dev);

	if (size > GTP_FW_NAME_MAXSIZE - 1) {
		dev_err(dev, "FW name size exceeds the limit.");
		return -EINVAL;
	}

	strlcpy(ts->fw_name, buf, size);
	if (ts->fw_name[size - 1] == '\n')
		ts->fw_name[size - 1] = '\0';

	return size;
}

static ssize_t gtp_fw_upgrade_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct goodix_ts_data *ts = dev_get_drvdata(dev);
	return snprintf(buf, 2, "%d\n", ts->fw_loading);
}

static ssize_t gtp_fw_upgrade_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t size)
{
	struct goodix_ts_data *ts = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	if (size > 2)
		return -EINVAL;

	if (sscanf(buf, "%u", &val) != 1)
		return -EINVAL;

	if (ts->gtp_is_suspend) {
		dev_err(&ts->client->dev,
			"Can't start fw upgrade. Device is in suspend state.");
		return -EBUSY;
	}

	mutex_lock(&ts->input_dev->mutex);
	if (!ts->fw_loading && val) {
		disable_irq(ts->client->irq);
		ts->fw_loading = true;
		if (config_enabled(CONFIG_GT9XX_TOUCHPANEL_UPDATE)) {
			ret = gup_update_proc(NULL);
			if (ret == FAIL)
				dev_err(&ts->client->dev,
					"Fail to update GTP firmware.\n");
		}
		ts->fw_loading = false;
		enable_irq(ts->client->irq);
	}
	mutex_unlock(&ts->input_dev->mutex);

	return size;
}

static ssize_t gtp_force_fw_upgrade_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t size)
{
	struct goodix_ts_data *ts = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	if (size > 2)
		return -EINVAL;

	if (sscanf(buf, "%u", &val) != 1)
		return -EINVAL;

	if (ts->gtp_is_suspend) {
		dev_err(&ts->client->dev,
			"Can't start fw upgrade. Device is in suspend state.");
		return -EBUSY;
	}

	mutex_lock(&ts->input_dev->mutex);
	if (!ts->fw_loading && val) {
		disable_irq(ts->client->irq);
		ts->fw_loading = true;
		ts->force_update = true;
		if (config_enabled(CONFIG_GT9XX_TOUCHPANEL_UPDATE)) {
			ret = gup_update_proc(NULL);
			if (ret == FAIL)
				dev_err(&ts->client->dev,
					"Fail to force update GTP firmware.\n");
		}
		ts->force_update = false;
		ts->fw_loading = false;
		enable_irq(ts->client->irq);
	}
	mutex_unlock(&ts->input_dev->mutex);

	return size;
}

static ssize_t goodix_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct goodix_ts_data *ts = dev_get_drvdata(dev);
	if (ts->gtp_is_suspend) {
		dev_info(&ts->client->dev, "Already in suspend state\n");
		return snprintf(buf, 4, "%s\n", "0");
	}
	return snprintf(buf, 4, "%s\n", ts->enable ? "1" : "0");
}

static ssize_t goodix_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct goodix_ts_data *ts = dev_get_drvdata(dev);
	unsigned long val;
	int rc, ret, i;

	if (size > 2)
		return -EINVAL;

	if (ts->gtp_is_suspend) {
		dev_info(&ts->client->dev, "Already in suspend state\n");
		goto no_operation;
	}

	rc = kstrtoul(buf, 10, &val);
	if (rc != 0)
		return rc;

	if (val) {
		ret = gtp_wakeup_sleep(ts);

#if GTP_GESTURE_WAKEUP
		doze_status = DOZE_DISABLED;
#endif

		if (ret <= 0)
			dev_err(&ts->client->dev, "GTP resume failed.\n");

		if (ts->use_irq)
			gtp_irq_enable(ts);
		else
			hrtimer_start(&ts->timer, ktime_set(1, 0),
				      HRTIMER_MODE_REL);

#if GTP_ESD_PROTECT
		ts->gtp_is_suspend = 0;
		gtp_esd_switch(ts->client, SWITCH_ON);
#endif
		ts->enable = true;
	} else {
#if GTP_ESD_PROTECT
		ts->gtp_is_suspend = 1;
		gtp_esd_switch(ts->client, SWITCH_OFF);
#endif

#if GTP_GESTURE_WAKEUP
		ret = gtp_enter_doze(ts);
#else
		if (ts->use_irq)
			gtp_irq_disable(ts);
		else
			hrtimer_cancel(&ts->timer);

		for (i = 0; i < GTP_MAX_TOUCH; i++)
			gtp_touch_up(ts, i);

		input_sync(ts->input_dev);

		ret = gtp_enter_sleep(ts);
#endif
		if (ret <= 0)
			dev_err(&ts->client->dev,
				"GTP early suspend failed.\n");
		/* to avoid waking up while not sleeping,
		 * delay 48 + 10ms to ensure reliability
		 */
		msleep(58);
		ts->enable = false;
	}

no_operation:
	return size;
}

static ssize_t goodix_mt_protocol_type_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	return snprintf(buf, 16, "%s\n", "MT Protocal B");
}

static ssize_t class_ts_info_show(struct class *class,
			struct class_attribute *attr, char *buf)
{
	if (class_ts_info)
		return snprintf(buf, GOODIX_INFO_MAX_LEN, "%s\n", class_ts_info);
	else
		return 0;
}
static ssize_t goodix_ts_info_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct goodix_ts_data *data = dev_get_drvdata(dev);
	return snprintf(buf, GOODIX_INFO_MAX_LEN, "%s\n", data->ts_info);
}

static DEVICE_ATTR(fw_name, (S_IRUGO | S_IWUSR | S_IWGRP),
		   gtp_fw_name_show, gtp_fw_name_store);
static DEVICE_ATTR(update_fw, (S_IRUGO | S_IWUSR | S_IWGRP),
		   gtp_fw_upgrade_show, gtp_fw_upgrade_store);
static DEVICE_ATTR(force_update_fw, (S_IRUGO | S_IWUSR | S_IWGRP),
		   gtp_fw_upgrade_show, gtp_force_fw_upgrade_store);
static DEVICE_ATTR(enable, (S_IRUGO | S_IWUSR | S_IWGRP),
		   goodix_enable_show, goodix_enable_store);
static DEVICE_ATTR(mt_protocol_type, (S_IRUGO | S_IWUSR | S_IWGRP),
		   goodix_mt_protocol_type_show, NULL);
static DEVICE_ATTR(ts_info, (S_IRUGO | S_IWUSR | S_IWGRP),
		   goodix_ts_info_show, NULL);
static CLASS_ATTR(ts_info, 0444, class_ts_info_show, NULL);

static struct attribute *gtp_attrs[] = {
	&dev_attr_fw_name.attr,
	&dev_attr_update_fw.attr,
	&dev_attr_force_update_fw.attr,
	&dev_attr_enable.attr,
	&dev_attr_mt_protocol_type.attr,
	&dev_attr_ts_info.attr,
	NULL
};

static const struct attribute_group gtp_attr_grp = {
	.attrs = gtp_attrs,
};

static int gtp_debug_addr_is_valid(u16 addr)
{
	if (addr < GTP_VALID_ADDR_START || addr > GTP_VALID_ADDR_END) {
		pr_err("GTP reg address is invalid: 0x%x\n", addr);
		return false;
	}

	return true;
}

static int gtp_debug_data_set(void *_data, u64 val)
{
	struct goodix_ts_data *ts = _data;

	mutex_lock(&ts->input_dev->mutex);
	if (gtp_debug_addr_is_valid(ts->addr))
		dev_err(&ts->client->dev,
			"Writing to GTP registers not supported.\n");
	mutex_unlock(&ts->input_dev->mutex);

	return 0;
}

static int gtp_debug_data_get(void *_data, u64 *val)
{
	struct goodix_ts_data *ts = _data;
	int ret;
	u8 buf[3] = {0};

	mutex_lock(&ts->input_dev->mutex);
	buf[0] = ts->addr >> 8;
	buf[1] = ts->addr & 0x00ff;

	if (gtp_debug_addr_is_valid(ts->addr)) {
		ret = gtp_i2c_read(ts->client, buf, 3);
		if (ret < 0)
			dev_err(&ts->client->dev,
				"GTP read register 0x%x failed (%d)\n",
				ts->addr, ret);
		else
			*val = buf[2];
	}
	mutex_unlock(&ts->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_data_fops, gtp_debug_data_get,
			gtp_debug_data_set, "%llx\n");

static int gtp_debug_addr_set(void *_data, u64 val)
{
	struct goodix_ts_data *ts = _data;

	if (gtp_debug_addr_is_valid(val)) {
		mutex_lock(&ts->input_dev->mutex);
		ts->addr = val;
		mutex_unlock(&ts->input_dev->mutex);
	}

	return 0;
}

static int gtp_debug_addr_get(void *_data, u64 *val)
{
	struct goodix_ts_data *ts = _data;

	mutex_lock(&ts->input_dev->mutex);
	if (gtp_debug_addr_is_valid(ts->addr))
		*val = ts->addr;
	mutex_unlock(&ts->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_addr_fops, gtp_debug_addr_get,
			gtp_debug_addr_set, "%llx\n");

static int gtp_debug_suspend_set(void *_data, u64 val)
{
	struct goodix_ts_data *ts = _data;

	mutex_lock(&ts->input_dev->mutex);
	if (val)
		goodix_ts_suspend(&ts->client->dev);
	else
		goodix_ts_resume(&ts->client->dev);
	mutex_unlock(&ts->input_dev->mutex);

	return 0;
}

static int gtp_debug_suspend_get(void *_data, u64 *val)
{
	struct goodix_ts_data *ts = _data;

	mutex_lock(&ts->input_dev->mutex);
	*val = ts->gtp_is_suspend;
	mutex_unlock(&ts->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_suspend_fops, gtp_debug_suspend_get,
			gtp_debug_suspend_set, "%lld\n");

static int goodix_debug_dump_info(struct seq_file *m, void *v)
{
	struct goodix_ts_data *data = m->private;

	seq_printf(m, "%s\n", data->ts_info);

	return 0;
}

static int debugfs_dump_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, goodix_debug_dump_info, inode->i_private);
}

static const struct file_operations debug_dump_info_fops = {
	.owner = THIS_MODULE,
	.open = debugfs_dump_info_open,
	.read = seq_read,
	.release = single_release,
};

static int gtp_debugfs_init(struct goodix_ts_data *data)
{
	data->debug_base = debugfs_create_dir(GTP_DEBUGFS_DIR, NULL);
	if (IS_ERR_OR_NULL(data->debug_base)) {
		dev_err(&data->client->dev, "Failed to create debugfs dir.\n");
		return -EINVAL;
	}

	if ((IS_ERR_OR_NULL(debugfs_create_file(GTP_DEBUGFS_FILE_SUSPEND,
						S_IWUSR | S_IWGRP | S_IRUSR |
						S_IRGRP, data->debug_base, data,
						&debug_suspend_fops)))) {
		dev_err(&data->client->dev, "Failed to create suspend file.\n");
		debugfs_remove_recursive(data->debug_base);
		return -EINVAL;
	}

	if ((IS_ERR_OR_NULL(debugfs_create_file(GTP_DEBUGFS_FILE_DATA,
						S_IWUSR | S_IWGRP | S_IRUSR |
						S_IRGRP, data->debug_base, data,
						&debug_data_fops)))) {
		dev_err(&data->client->dev, "Failed to create data file.\n");
		debugfs_remove_recursive(data->debug_base);
		return -EINVAL;
	}

	if ((IS_ERR_OR_NULL(debugfs_create_file(GTP_DEBUGFS_FILE_ADDR,
						S_IWUSR | S_IWGRP | S_IRUSR |
						S_IRGRP, data->debug_base, data,
						&debug_addr_fops)))) {
		dev_err(&data->client->dev, "Failed to create addr file.\n");
		debugfs_remove_recursive(data->debug_base);
		return -EINVAL;
	}

	if ((IS_ERR_OR_NULL(debugfs_create_file(GTP_DEBUGFS_FILE_DUMP_INFO,
						S_IWUSR | S_IWGRP | S_IRUSR |
						S_IRGRP, data->debug_base, data,
						&debug_dump_info_fops)))) {
		dev_err(&data->client->dev,
			"Failed to create dump info file.\n");
		debugfs_remove_recursive(data->debug_base);
		return -EINVAL;
	}

	return 0;
}

static int goodix_parse_dt(struct device *dev,
			   struct goodix_ts_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;
	struct property *prop;
	u32 temp_val, num_buttons;
	u32 button_map[MAX_BUTTONS];
	char prop_name[PROP_NAME_SIZE];
	int i, read_cfg_num;

	pdata->name = "goodix";
	rc = goodix_ts_get_dt_coords(dev, "goodix,panel-coords", pdata);
	if (rc && (rc != -EINVAL))
		return rc;

	rc = goodix_ts_get_dt_coords(dev, "goodix,display-coords", pdata);
	if (rc)
		return rc;

	pdata->i2c_pull_up = of_property_read_bool(np, "goodix,i2c-pull-up");

	pdata->no_force_update = of_property_read_bool(np,
						"goodix,no-force-update");

	pdata->enable_power_off = of_property_read_bool(np,
						"goodix,enable-power-off");
	pdata->fw_vkey_support = of_property_read_bool(np,
						"goodix,fw-vkey-support");
	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "reset-gpios",
						    0,
						    &pdata->reset_gpio_flags);
	if (pdata->reset_gpio < 0)
		return pdata->reset_gpio;

	pdata->irq_gpio = of_get_named_gpio_flags(np, "interrupt-gpios",
						  0,&pdata->irq_gpio_flags);
	if (pdata->irq_gpio < 0)
		return pdata->irq_gpio;

	rc = of_property_read_string(np, "goodix,product-id",
				     &pdata->product_id);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Failed to parse product_id.");
		return -EINVAL;
	}

	rc = of_property_read_u32(np, "goodix,num_max_touches",
				  &pdata->num_max_touches);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Failed to parse num_max_touches.");
		return -EINVAL;
	}

	rc = of_property_read_u32(np, "goodix,soft_rst_dly",
				  &pdata->soft_rst_dly);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Failed to parse soft reset delay.");
		return -EINVAL;
	}

	rc = of_property_read_string(np, "goodix,fw_name", &pdata->fw_name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Failed to parse firmware name.\n");
		return -EINVAL;
	}

	prop = of_find_property(np, "goodix,button-map", NULL);
	if (prop) {
		num_buttons = prop->length / sizeof(temp_val);
		if (num_buttons > MAX_BUTTONS)
			return -EINVAL;

		rc = of_property_read_u32_array(np,
						"goodix,button-map",
						button_map,
						num_buttons);
		if (rc) {
			dev_err(dev, "Unable to read key codes\n");
			return rc;
		}
	}

	read_cfg_num = 0;
	for (i = 0; i < GOODIX_MAX_CFG_GROUP; i++) {
		snprintf(prop_name, sizeof(prop_name), "goodix,cfg-data%d", i);
		prop = of_find_property(np, prop_name,
					&pdata->config_data_len[i]);
		if (!prop || !prop->value) {
			pdata->config_data_len[i] = 0;
			pdata->config_data[i] = NULL;
			continue;
		}
		pdata->config_data[i] = devm_kzalloc(dev,
						     GTP_CONFIG_MAX_LENGTH +
						     GTP_ADDR_LENGTH,
						     GFP_KERNEL);
		if (!pdata->config_data[i]) {
			dev_err(dev,
				"Not enough memory for panel config data %d\n",
				i);
			return -ENOMEM;
		}
		pdata->config_data[i][0] = GTP_REG_CONFIG_DATA >> 8;
		pdata->config_data[i][1] = GTP_REG_CONFIG_DATA & 0xff;
		memcpy(&pdata->config_data[i][GTP_ADDR_LENGTH],
		       prop->value, pdata->config_data_len[i]);
		read_cfg_num++;
	}
	dev_info(dev, "%d config data read from device tree.\n", read_cfg_num);

	return 0;
}

/*******************************************************
Function:
    I2c probe.
Input:
    client: i2c device struct.
    id: device id.
Output:
    Executive outcomes.
    0: succeed.
*******************************************************/
static int goodix_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	s32 ret = -1;
	struct goodix_ts_platform_data *pdata;
	struct goodix_ts_data *ts;
	u16 version_info;
	GTP_INFO("GTP9517 Driver probe start!\n");

	GTP_DEBUG_FUNC();

	/* do NOT remove these logs */
	GTP_INFO("GTP Driver Version: %s", GTP_DRIVER_VERSION);
	GTP_INFO("GTP Driver Built@%s, %s", __TIME__, __DATE__);
	GTP_INFO("GTP I2C Address: 0x%02x", client->addr);

	i2c_connect_client = client;
	dev_dbg(&client->dev, "GTP I2C Address: 0x%02x\n", client->addr);
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				     sizeof(struct goodix_ts_platform_data),
				     GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev,
				"GTP Failed to allocate memory for pdata\n");
			return -ENOMEM;
		}
		GTP_INFO("devm_kzalloc is success!");

		ret = goodix_parse_dt(&client->dev, pdata);
		if (ret)
			return ret;
		GTP_INFO("parse dt is success!");
	} else
		pdata = client->dev.platform_data;

	if (!pdata) {
		dev_err(&client->dev, "GTP invalid pdata\n");
		return -EINVAL;
	}
	GTP_INFO("get pdata is success!");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		GTP_ERROR("I2C check functionality failed.");
		return -ENODEV;
	}
	GTP_INFO("i2c check is success!");
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		GTP_ERROR("Alloc GFP_KERNEL memory failed.");
		return -ENOMEM;
	}

	memset(ts, 0, sizeof(*ts));
	INIT_WORK(&ts->work, goodix_ts_work_func);
	ts->client = client;
	ts->pdata = pdata;
	spin_lock_init(&ts->irq_lock);
	mutex_init(&ts->lock);
#if GTP_ESD_PROTECT
	ts->clk_tick_cnt = 2 * HZ;
	GTP_DEBUG("Clock ticks for an esd cycle: %d", ts->clk_tick_cnt);
	spin_lock_init(&ts->esd_lock);
#endif
	i2c_set_clientdata(client, ts);

	ts->gtp_rawdiff_mode = 0;

	ts->ts_info =
	    devm_kzalloc(&client->dev, GOODIX_INFO_MAX_LEN, GFP_KERNEL);
	if (!ts->ts_info) {
		dev_err(&client->dev, "Not enough memory\n");
		goto exit_free_client_data;
	}

	GOODIX_STORE_TS_INFO(ts->ts_info, ts->pdata->name,
			     ts->pdata->num_max_touches, ts->pdata->product_id,
			     ts->pdata->fw_vkey_support ? "yes" : "no");

	ret = goodix_power_init(ts);
	if (ret) {
		dev_err(&client->dev, "GTP power init failed\n");
		goto exit_free_ts_info;
	}
	ret = goodix_power_on(ts);
	if (ret) {
		dev_err(&client->dev, "GTP power on failed\n");
		goto exit_deinit_power;
	}
	ret = gtp_request_io_port(ts);
	GTP_ERROR("gtp_request_port start!");
	if (ret < 0) {
		GTP_ERROR("request io port failed!");
		GTP_ERROR("GTP request IO port failed.");
		kfree(ts);
		goto exit_power_off;
	}
	GTP_ERROR("gtp_request_port end!");

	gtp_reset_guitar(ts->client, ts->pdata->soft_rst_dly);
	GTP_ERROR("gtp_reset_guitar end!");

	ret = gtp_i2c_test(client);
	if (ret < 0) {
		GTP_ERROR("I2C communication ERROR!");
		goto exit_power_off;
	}

#if GTP_COMPATIBLE_MODE
	gtp_get_chip_type(ts);
	if (CHIP_TYPE_GT9F == ts->chip_type) {
		ret = gtp_gt9xxf_init(ts->client);
		if (FAIL == ret) {
			GTP_INFO("Failed to init GT9XXF.");
			goto exit_power_off;
		}
	}
#endif

	if (!pdata->no_force_update)
		ts->force_update = true;
	if (pdata->fw_name)
		strlcpy(ts->fw_name, pdata->fw_name,
			strlen(pdata->fw_name) + 1);

	ret = gtp_read_version(client, &version_info);
	if (ret < 0)
		GTP_ERROR("Read version failed.");

	ret = gtp_init_panel(ts);
	if (ret < 0) {
		GTP_ERROR("GTP init panel failed.");
		ts->abs_x_max = GTP_MAX_WIDTH;
		ts->abs_y_max = GTP_MAX_HEIGHT;
		ts->int_trigger_type = GTP_INT_TRIGGER;
	}
#if defined(CONFIG_FB)
	ts->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&ts->fb_notif);
	if (ret)
		dev_err(&ts->client->dev,
			"Unable to register fb_notifier: %d\n", ret);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = goodix_ts_early_suspend;
	ts->early_suspend.resume = goodix_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	/* Create proc file system */
	gt91xx_config_proc =
	    proc_create(GT91XX_CONFIG_PROC_FILE, 0666, NULL, &config_proc_ops);
	if (gt91xx_config_proc == NULL)
		GTP_ERROR("create_proc_entry %s failed\n",
			  GT91XX_CONFIG_PROC_FILE);
	else
		GTP_INFO("create proc entry %s success",
			 GT91XX_CONFIG_PROC_FILE);

#if GTP_AUTO_UPDATE
	ret = gup_init_update_proc(ts);
	if (ret < 0)
		GTP_ERROR("Create update thread error.");
	ret = gtp_read_version(client, &version_info);
	if (ret < 0)
		GTP_ERROR("Read version failed.");
#endif

	ret = gtp_request_input_dev(ts);
	if (ret < 0) {
		GTP_ERROR("GTP request input dev failed");
		goto exit_free_inputdev;
	}

	ret = gtp_request_irq(ts);
	if (ret < 0)
		GTP_INFO("GTP works in polling mode.");
	else
		GTP_INFO("GTP works in interrupt mode.");

	if (ts->use_irq)
		gtp_irq_enable(ts);

	ts->enable = true;

#ifdef CONFIG_GT9XX_TOUCHPANEL_DEBUG
#if GTP_CREATE_WR_NODE
	init_wr_node(client);
#endif
#endif

#if GTP_ESD_PROTECT
	gtp_esd_switch(client, SWITCH_ON);
#endif
	ret = sysfs_create_group(&client->dev.kobj, &gtp_attr_grp);
	if (ret < 0) {
		dev_err(&client->dev, "sys file creation failed.\n");
		goto exit_free_irq;
	}

	ret = gtp_debugfs_init(ts);
	if (ret != 0) {
		dev_err(&client->dev, "Failed to create debugfs entries, %d\n",
			ret);
		goto exit_remove_sysfs;
	}

	class_ts_info = devm_kzalloc(&client->dev, GOODIX_INFO_MAX_LEN, GFP_KERNEL);
	if (!class_ts_info ) {
		dev_err(&client->dev, "class_ts_info Not enough memory\n");
	}
	CLASS_GOODIX_STORE_TS_INFO(class_ts_info,  ts->pdata->name,
			  "DJN_1284a_9157",
			 version_info);
	class_touchscreen = class_create(THIS_MODULE,"touchscreen");
	if (IS_ERR(class_touchscreen ))
		return PTR_ERR(class_touchscreen );

	ret = class_create_file(class_touchscreen,&class_attr_ts_info);
	if (ret)
		dev_err(&client->dev,"create sys file failed \n");
	return 0;

exit_remove_sysfs:
	sysfs_remove_group(&ts->input_dev->dev.kobj, &gtp_attr_grp);
exit_free_irq:
	input_unregister_device(ts->input_dev);
	if (ts->input_dev) {
		input_free_device(ts->input_dev);
		ts->input_dev = NULL;
	}
	if (ts->use_irq)
		free_irq(client->irq, ts);
exit_free_inputdev:
#if defined(CONFIG_FB)
	if (fb_unregister_client(&ts->fb_notif))
		dev_err(&client->dev,
			"Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ts->early_suspend);
#endif
exit_power_off:
	goodix_power_off(ts);
exit_deinit_power:
	goodix_power_deinit(ts);
exit_free_ts_info:
	devm_kfree(&client->dev, ts->ts_info);
exit_free_client_data:
	mutex_destroy(&ts->lock);
	cancel_work_sync(&ts->work);
	i2c_set_clientdata(client, NULL);
	kfree(ts);
	return ret;
}

/*******************************************************
Function:
    Goodix touchscreen driver release function.
Input:
    client: i2c device struct.
Output:
    Executive outcomes. 0---succeed.
*******************************************************/
static int goodix_ts_remove(struct i2c_client *client)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	GTP_DEBUG_FUNC();

	class_remove_file(class_touchscreen , &class_attr_ts_info);
	class_destroy(class_touchscreen );
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif

#ifdef CONFIG_GT9XX_TOUCHPANEL_DEBUG
#if GTP_CREATE_WR_NODE
	uninit_wr_node();
#endif
#endif

#if GTP_ESD_PROTECT
	destroy_workqueue(gtp_esd_check_workqueue);
#endif

	if (ts) {
		if (ts->use_irq) {
			gpio_direction_input(ts->pdata->irq_gpio);
			gpio_free(ts->pdata->irq_gpio);
			free_irq(client->irq, ts);
		} else
			hrtimer_cancel(&ts->timer);
		if (gpio_is_valid(ts->pdata->reset_gpio))
			gpio_free(ts->pdata->reset_gpio);
		if (gpio_is_valid(ts->pdata->irq_gpio))
			gpio_free(ts->pdata->irq_gpio);
	}

	GTP_INFO("GTP driver removing...");
	i2c_set_clientdata(client, NULL);
	input_unregister_device(ts->input_dev);
	debugfs_remove_recursive(ts->debug_base);
	kfree(ts);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
/*******************************************************
Function:
    Early suspend function.
Input:
    h: early_suspend struct.
Output:
    None.
*******************************************************/
static void goodix_ts_early_suspend(struct i2c_client *client,
				    pm_message_t mesg)
{
	struct goodix_ts_data *ts;
	s8 ret = -1;
	ts = container_of(client, struct goodix_ts_data, i2c_client);

	GTP_DEBUG_FUNC();

	GTP_INFO("System suspend.");

	ts->gtp_is_suspend = 1;
#if GTP_ESD_PROTECT
	gtp_esd_switch(ts->client, SWITCH_OFF);
#endif

#if GTP_GESTURE_WAKEUP
	ret = gtp_enter_doze(ts);
#else
	if (ts->use_irq)
		gtp_irq_disable(ts);
	else
		hrtimer_cancel(&ts->timer);
	ret = gtp_enter_sleep(ts);
#endif
	if (ret < 0)
		GTP_ERROR("GTP early suspend failed.");
	/* to avoid waking up while not sleeping
	  * delay 48 + 10ms to ensure reliability
	  */
	msleep(58);
}

/*******************************************************
Function:
    Late resume function.
Input:
    h: early_suspend struct.
Output:
    None.
*******************************************************/
static void goodix_ts_late_resume(struct i2c_client *client)
{
	struct goodix_ts_data *ts;
	s8 ret = -1;
	ts = container_of(client, struct goodix_ts_data, i2c_client);

	GTP_DEBUG_FUNC();

	GTP_INFO("System resume.");

	ret = gtp_wakeup_sleep(ts);

#if GTP_GESTURE_WAKEUP
	doze_status = DOZE_DISABLED;
#endif

	if (ret < 0)
		GTP_ERROR("GTP later resume failed.");
#if (GTP_COMPATIBLE_MODE)
	if (CHIP_TYPE_GT9F == ts->chip_type) {
		/* do nothing */
	} else
#endif
		gtp_send_cfg(ts->client);

	if (ts->use_irq)
		gtp_irq_enable(ts);
	else
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

	ts->gtp_is_suspend = 0;
#if GTP_ESD_PROTECT
	gtp_esd_switch(ts->client, SWITCH_ON);
#endif
}
#endif

#if defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_FB)
/*******************************************************
Function:
	Early suspend function.
Input:
	h: early_suspend struct.
Output:
	None.
*******************************************************/
static int goodix_ts_suspend(struct device *dev)
{
	int ret = -1;
	struct goodix_ts_data *ts = dev_get_drvdata(dev);

	GTP_DEBUG_FUNC();

	if (ts->gtp_is_suspend) {
		dev_dbg(&ts->client->dev, "Already in suspend state.\n");
		return 0;
	}

	mutex_lock(&ts->lock);

	GTP_INFO("System suspend.");

	if (ts->fw_loading) {
		dev_info(&ts->client->dev,
			 "Fw upgrade in progress, can't go to suspend.");
		mutex_unlock(&ts->lock);
		return 0;
	}
#if GTP_ESD_PROTECT
	gtp_esd_switch(ts->client, SWITCH_OFF);
#endif

#if GTP_GESTURE_WAKEUP
	ret = gtp_enter_doze(ts);
#else
	if (ts->use_irq)
		gtp_irq_disable(ts);
	else
		hrtimer_cancel(&ts->timer);
	ret = gtp_enter_sleep(ts);
#endif
	if (ret < 0)
		GTP_ERROR("GTP early suspend failed.");
	/* to avoid waking up while not sleeping
	  * delay 48 + 10ms to ensure reliability
	  */
	msleep(58);

	ts->gtp_is_suspend = 1;
	mutex_unlock(&ts->lock);

	return ret;
}

/*******************************************************
Function:
	Late resume function.
Input:
	h: early_suspend struct.
Output:
	None.
*******************************************************/
static int goodix_ts_resume(struct device *dev)
{
	int ret = -1;
	struct goodix_ts_data *ts = dev_get_drvdata(dev);

	GTP_DEBUG_FUNC();

	if (!ts->gtp_is_suspend) {
		dev_dbg(&ts->client->dev, "Already in awake state.\n");
		return 0;
	}

	mutex_lock(&ts->lock);

	GTP_INFO("System resume.");
	ret = gtp_wakeup_sleep(ts);

#if GTP_GESTURE_WAKEUP
	doze_status = DOZE_DISABLED;
#endif

	if (ret < 0)
		GTP_ERROR("GTP later resume failed.");
#if (GTP_COMPATIBLE_MODE)
	if (CHIP_TYPE_GT9F == ts->chip_type) {
		/* do nothing */
	} else
#endif
		gtp_send_cfg(ts->client);

	if (ts->use_irq)
		gtp_irq_enable(ts);
	else
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

	ts->gtp_is_suspend = 0;
#if GTP_ESD_PROTECT
	gtp_esd_switch(ts->client, SWITCH_ON);
#endif
	mutex_unlock(&ts->lock);
	return ret;
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct goodix_ts_data *ts =
	    container_of(self, struct goodix_ts_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
	    ts && ts->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			goodix_ts_resume(&ts->client->dev);
		else if (*blank == FB_BLANK_POWERDOWN)
			goodix_ts_suspend(&ts->client->dev);
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
/*******************************************************
Function:
	Early suspend function.
Input:
	h: early_suspend struct.
Output:
	None.
*******************************************************/
static void goodix_ts_early_suspend(struct early_suspend *h)
{
	struct goodix_ts_data *ts;

	ts = container_of(h, struct goodix_ts_data, early_suspend);
	goodix_ts_suspend(&ts->client->dev);
	return;
}

/*******************************************************
Function:
	Late resume function.
Input:
	h: early_suspend struct.
Output:
	None.
*******************************************************/
static void goodix_ts_late_resume(struct early_suspend *h)
{
	struct goodix_ts_data *ts;

	ts = container_of(h, struct goodix_ts_data, early_suspend);
	goodix_ts_resume(&ts->client->dev);
	return;
}
#endif
#endif /* !CONFIG_HAS_EARLYSUSPEND && !CONFIG_FB */

#if GTP_ESD_PROTECT
static s32 gtp_i2c_read_no_rst(struct i2c_client *client, u8 *buf, s32 len)
{
	struct i2c_msg msgs[2];
	s32 ret = -1;
	s32 retries = 0;

	GTP_DEBUG_FUNC();

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr = client->addr;
	msgs[0].len = GTP_ADDR_LENGTH;
	msgs[0].buf = &buf[0];

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr = client->addr;
	msgs[1].len = len - GTP_ADDR_LENGTH;
	msgs[1].buf = &buf[GTP_ADDR_LENGTH];

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret == 2)
			break;
		retries++;
	}
	if ((retries >= 5))
		GTP_ERROR("I2C Read: 0x%04X, %d bytes failed, errcode: %d!",
			  (((u16)(buf[0] << 8)) | buf[1]), len - 2, ret);
	return ret;
}

static s32 gtp_i2c_write_no_rst(struct i2c_client *client, u8 *buf, s32 len)
{
	struct i2c_msg msg;
	s32 ret = -1;
	s32 retries = 0;

	GTP_DEBUG_FUNC();

	msg.flags = !I2C_M_RD;
	msg.addr = client->addr;
	msg.len = len;
	msg.buf = buf;

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)
			break;
		retries++;
	}
	if ((retries >= 5))
		GTP_ERROR("I2C Write: 0x%04X, %d bytes failed, errcode: %d!",
			  (((u16)(buf[0] << 8)) | buf[1]), len - 2, ret);
	return ret;
}

/*******************************************************
Function:
    switch on & off esd delayed work
Input:
    client:  i2c device
    on:      SWITCH_ON / SWITCH_OFF
Output:
    void
*********************************************************/
void gtp_esd_switch(struct i2c_client *client, s32 on)
{
	struct goodix_ts_data *ts;

	ts = i2c_get_clientdata(client);
	spin_lock(&ts->esd_lock);

	if (SWITCH_ON == on) {
		if (!ts->esd_running) {
			ts->esd_running = 1;
			spin_unlock(&ts->esd_lock);
			GTP_INFO("Esd started");
			queue_delayed_work(gtp_esd_check_workqueue,
					   &gtp_esd_check_work,
					   ts->clk_tick_cnt);
		} else
			spin_unlock(&ts->esd_lock);
	} else {
		if (ts->esd_running) {
			ts->esd_running = 0;
			spin_unlock(&ts->esd_lock);
			GTP_INFO("Esd cancelled");
			cancel_delayed_work_sync(&gtp_esd_check_work);
		} else
			spin_unlock(&ts->esd_lock);
	}
}

/*******************************************************
Function:
    Initialize external watchdog for esd protect
Input:
    client:  i2c device.
Output:
    result of i2c write operation.
    1: succeed, otherwise: failed
*********************************************************/
static s32 gtp_init_ext_watchdog(struct i2c_client *client)
{
	u8 opr_buffer[3] = {0x80, 0x41, 0xAA};
	GTP_DEBUG("[Esd]Init external watchdog");
	return gtp_i2c_write_no_rst(client, opr_buffer, 3);
}

/*******************************************************
Function:
    Esd protect function.
    External watchdog added by meta, 2013/03/07
Input:
    work: delayed work
Output:
    None.
*******************************************************/
static void gtp_esd_check_func(struct work_struct *work)
{
	s32 i;
	s32 ret = -1;
	struct goodix_ts_data *ts = NULL;
	u8 esd_buf[5] = {0x80, 0x40};

	GTP_DEBUG_FUNC();

	ts = i2c_get_clientdata(i2c_connect_client);

	if (ts->gtp_is_suspend) {
		GTP_INFO("Esd suspended!");
		return;
	}

	for (i = 0; i < 3; i++) {
		ret = gtp_i2c_read_no_rst(ts->client, esd_buf, 4);

		GTP_DEBUG("[Esd]0x8040 = 0x%02X, 0x8041 = 0x%02X", esd_buf[2],
			  esd_buf[3]);
		if ((ret < 0)) {
			/* IIC communication problem */
			continue;
		} else {
			if ((esd_buf[2] == 0xAA) || (esd_buf[3] != 0xAA)) {
				/* IC works abnormally.. */
				u8 chk_buf[4] = {0x80, 0x40};

				gtp_i2c_read_no_rst(ts->client, chk_buf, 4);

				GTP_DEBUG
				    ("[Check]0x8040 = 0x%02X, 0x8041 = 0x%02X",
				     chk_buf[2], chk_buf[3]);

				if ((chk_buf[2] == 0xAA)
				    || (chk_buf[3] != 0xAA)) {
					i = 3;
					break;
				} else
					continue;
			} else {
				/* IC works normally,
				  * Write 0x8040 0xAA,
				  * feed the dog
				  */
				esd_buf[2] = 0xAA;
				gtp_i2c_write_no_rst(ts->client, esd_buf, 3);
				break;
			}
		}
	}
	if (i >= 3) {
#if GTP_COMPATIBLE_MODE
		if (CHIP_TYPE_GT9F == ts->chip_type) {
			if (ts->rqst_processing)
				GTP_INFO("Request processing, no esd recovery");
			else {
				GTP_ERROR("IC working abnormally!");
				esd_buf[0] = 0x42;
				esd_buf[1] = 0x26;
				esd_buf[2] = 0x01;
				esd_buf[3] = 0x01;
				esd_buf[4] = 0x01;
				gtp_i2c_write_no_rst(ts->client, esd_buf, 5);
				msleep(50);
				gtp_esd_recovery(ts->client);
			}
		} else
#endif
		{
			GTP_ERROR
			    ("IC working abnormally! Process reset guitar.");
			esd_buf[0] = 0x42;
			esd_buf[1] = 0x26;
			esd_buf[2] = 0x01;
			esd_buf[3] = 0x01;
			esd_buf[4] = 0x01;
			gtp_i2c_write_no_rst(ts->client, esd_buf, 5);
			msleep(50);
			gtp_reset_guitar(ts->client, 50);
			msleep(50);
			gtp_send_cfg(ts->client);
		}
	}

	if (!ts->gtp_is_suspend)
		queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work,
				   ts->clk_tick_cnt);
	else
		GTP_INFO("Esd suspended!");
	return;
}
#endif

#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
static const struct dev_pm_ops goodix_ts_dev_pm_ops = {
	.suspend = goodix_ts_suspend,
	.resume = goodix_ts_resume,
};
#else
static const struct dev_pm_ops goodix_ts_dev_pm_ops = {
};
#endif

static const struct i2c_device_id goodix_ts_id[] = {
	{GTP_I2C_NAME, 0},
	{}
};

static struct of_device_id goodix_match_table[] = {
	{.compatible = "goodix,gt9157",},
	{},
};

static struct i2c_driver goodix_ts_driver = {
	.probe = goodix_ts_probe,
	.remove = goodix_ts_remove,
#ifdef CONFIG_HAS_EARLYSUSPEND
	.suspend = goodix_ts_early_suspend,
	.resume = goodix_ts_late_resume,
#endif
	.id_table = goodix_ts_id,
	.driver = {
		.name = GTP_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = goodix_match_table,
#if CONFIG_PM
		.pm = &goodix_ts_dev_pm_ops,
#endif
	},
};

/*******************************************************
Function:
    Driver Install function.
Input:
    None.
Output:
    Executive Outcomes. 0---succeed.
********************************************************/
static int __init goodix_ts_init(void)
{
	GTP_DEBUG_FUNC();
	GTP_INFO("GTP driver installing...");
	pr_err("GTP driverf inistall .....");
	goodix_wq = create_singlethread_workqueue("goodix_wq");
	if (!goodix_wq) {
		GTP_ERROR("Creat workqueue failed.");
		return -ENOMEM;
	}
#if GTP_ESD_PROTECT
	INIT_DELAYED_WORK(&gtp_esd_check_work, gtp_esd_check_func);
	gtp_esd_check_workqueue = create_workqueue("gtp_esd_check");
	pr_err("GTP_ESD_PROTECT........");
#endif
	pr_err("add i2c driver.........");
	return i2c_add_driver(&goodix_ts_driver);
}

/*******************************************************
Function:
    Driver uninstall function.
Input:
    None.
Output:
    Executive Outcomes. 0---succeed.
********************************************************/
static void __exit goodix_ts_exit(void)
{
	GTP_DEBUG_FUNC();
	GTP_INFO("GTP driver exited.");
	i2c_del_driver(&goodix_ts_driver);
	if (goodix_wq)
		destroy_workqueue(goodix_wq);
}

module_init(goodix_ts_init);
module_exit(goodix_ts_exit);

MODULE_DESCRIPTION("GTP Series Driver");
MODULE_LICENSE("GPL");
