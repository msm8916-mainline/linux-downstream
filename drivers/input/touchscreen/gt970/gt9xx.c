/* drivers/input/touchscreen/gt9xx.c
 *
 * 2010 - 2013 Goodix Technology.
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
 *          modify gtp_reset_guitar,slot report,tracking_id & 0x0F. By Andrew, 2012/10/15
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
#include "gt9xx.h"

#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/module.h>
#include <linux/input/mt.h>
#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/pinctrl/consumer.h>
#include <linux/bbk_drivers_info.h>

#if GTP_ICS_SLOT_REPORT
    #include <linux/input/mt.h>
#endif
#include <linux/vivo_touchscreen_common.h>
#include <linux/vivo_touchscreen_config.h>
#include "../../../../kernel/irq/internals.h"

/*[BUGFIX]PR-667466. TP INT pull-up enable.*/
//#define GOODIX_PINCTRL_STATE_SLEEP "ts_int_suspend"
//#define GOODIX_PINCTRL_STATE_DEFAULT "ts_int_active"
#define GOODIX_PINCTRL_STATE_SLEEP "gt9xx_int_suspend"
#define GOODIX_PINCTRL_STATE_DEFAULT "gt9xx_int_default"
/*[BUGFIX]PR-667466. TP INT pull-up enable.*/

static const char *goodix_ts_name = "goodix-ts";
static struct workqueue_struct *goodix_wq;
struct i2c_client * i2c_connect_client = NULL;
u8 config[GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH]
                = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};

//just add charge for gt970
static u32 chr_status = 0;				
static const char *global_product_name = NULL;
static const char *global_vdd_mode = NULL;
//struct irq_desc *irq_goodix;

#if GTP_HAVE_TOUCH_KEY
    static const u16 touch_key_array[] = GTP_KEY_TAB;
    #define GTP_MAX_KEY_NUM  (sizeof(touch_key_array)/sizeof(touch_key_array[0]))

#if GTP_DEBUG_ON
    static const int  key_codes[] = {KEY_HOME, KEY_BACK, KEY_MENU, KEY_SEARCH};
    static const char *key_names[] = {"Key_Home", "Key_Back", "Key_Menu", "Key_Search"};
#endif
#endif

#define GOODIX_VTG_MIN_UV	3000000
#define GOODIX_VTG_MAX_UV	3000000
#define GOODIX_I2C_VTG_MIN_UV	1800000
#define GOODIX_I2C_VTG_MAX_UV	1800000
#define GOODIX_VDD_LOAD_MIN_UA	0
#define GOODIX_VDD_LOAD_MAX_UA	10000
#define GOODIX_VIO_LOAD_MIN_UA	0
#define GOODIX_VIO_LOAD_MAX_UA	10000
#define PROP_NAME_SIZE		24
#define GOODIX_COORDS_ARR_SIZE	4

static u8 chip_gt9xxs;  /* true if ic is gt9xxs, like gt915s */

static s8 gtp_i2c_test(struct i2c_client *client);
void gtp_reset_guitar(struct i2c_client *client, s32 ms);
s32 gtp_send_cfg(struct i2c_client *client);
void gtp_int_sync(struct goodix_ts_data *ts, s32 ms);
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data);

static ssize_t gt91xx_config_read_proc(struct file *, char __user *, size_t, loff_t *);
static ssize_t gt91xx_config_write_proc(struct file *, const char __user *, size_t, loff_t *);
static void gtp_set_virtual_key_string(const char *vkey);

static struct proc_dir_entry *gt91xx_config_proc = NULL;
static const struct file_operations config_proc_ops = {
    .owner = THIS_MODULE,
    .read = gt91xx_config_read_proc,
    .write = gt91xx_config_write_proc,
};
/*[BUGFIX]PR-667466. TP INT pull-up enable.*/
struct gtp_pinctrl_info {
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
};

static struct gtp_pinctrl_info gt9xx_pctrl;
#ifdef CONFIG_HAS_EARLYSUSPEND
static void goodix_ts_early_suspend(struct early_suspend *h);
static void goodix_ts_late_resume(struct early_suspend *h);
#endif

#if GTP_CREATE_WR_NODE
extern s32 init_wr_node(struct i2c_client*);
extern void uninit_wr_node(void);
#endif

extern void gesture_clear_wakeup_data(void);
extern s32 gtp_init_node(void);
extern s32 gesture_event_handler(struct input_dev * dev);

#if GTP_AUTO_UPDATE
extern u8 gup_init_update_proc(struct goodix_ts_data *);
#endif

extern int gesture_enabled;

//#if GTP_ESD_PROTECT
static struct delayed_work gtp_esd_check_work;
static struct workqueue_struct * gtp_esd_check_workqueue = NULL;
static void gtp_esd_check_func(struct work_struct *);
static s32 gtp_init_ext_watchdog(struct i2c_client *client);
void gtp_esd_switch(struct i2c_client *, s32);
//#endif
static int gtp_enter_edge_restrain(struct goodix_ts_data *ts);
static int gtp_leave_edge_restrain(struct goodix_ts_data *ts);

/*Add debug interface for input devices.*/
//extern char* g_tp_device_name;
//extern u8 g_tp_cfg_ver;
/*Add debug interface for input devices.*/
/*2014-6-03,Add debug interface for input devices.*/
#ifdef CONFIG_TCT_8X16_POP8LTE
extern u8 g_wakeup_gesture;
#endif
/*Add debug interface for input devices.*/

static void ts_scan_switch(bool on);

#define TOUCHSCREEN_NORMAL  0
#define TOUCHSCREEN_SLEEP   1
#define TOUCHSCREEN_GESTURE 2
static u8 buf_coordinate_num[3] = {0x81, 0x4C};/*gesture coordinate num*/

DOZE_T doze_status = DOZE_DISABLED;
static s8 gtp_enter_doze(struct goodix_ts_data *ts);

u8 grp_cfg_version = 0;

struct gtp_driver_sysfs_entry {
	struct attribute attr;
	ssize_t (*show)(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf);
	ssize_t (*store)(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count);
};

extern int qup_i2c_suspended; // add for i2c timeout

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
    s32 ret=-1;
    s32 retries = 0;
	
	struct goodix_ts_data *ts = i2c_get_clientdata(client);
    
	GTP_DEBUG_FUNC();
	
	if(qup_i2c_suspended > 0)	
	{
		VIVO_TS_LOG_ERR("I2c bus is suspended.");
		return -1;	
	}

    msgs[0].flags = !I2C_M_RD;
    msgs[0].addr  = client->addr;
    msgs[0].len   = GTP_ADDR_LENGTH;
    msgs[0].buf   = &buf[0];
    //msgs[0].scl_rate = 300 * 1000;    // for Rockchip, etc.

    msgs[1].flags = I2C_M_RD;
    msgs[1].addr  = client->addr;
    msgs[1].len   = len - GTP_ADDR_LENGTH;
    msgs[1].buf   = &buf[GTP_ADDR_LENGTH];
    //msgs[1].scl_rate = 300 * 1000;

    while(retries < 5)
    {
        ret = i2c_transfer(client->adapter, msgs, 2);
        if(ret == 2)break;
        retries++;
    }
    if((retries >= 5))
    {
    //#if GTP_GESTURE_WAKEUP
    if(vivo_touchscreen_is_support(FS_DCLICK_WAKE))
	{
        // reset chip would quit doze mode
        if (DOZE_ENABLED == doze_status)
        {
        	VIVO_TS_LOG_ERR("Gesture mode-------------------");
   			gtp_reset_guitar(client, 20);
			gtp_enter_doze(ts);
   			return ret;
        }
    }
        VIVO_TS_LOG_ERR("I2C Read: 0x%04X, %d bytes failed, errcode: %d! Process reset.", (((u16)(buf[0] << 8)) | buf[1]), len-2, ret);
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
s32 gtp_i2c_write(struct i2c_client *client,u8 *buf,s32 len)
{
    struct i2c_msg msg;
    s32 ret = -1;
    s32 retries = 0;

	struct goodix_ts_data *ts = i2c_get_clientdata(client);
    
	GTP_DEBUG_FUNC();
	if(qup_i2c_suspended > 0)	
	{
		VIVO_TS_LOG_ERR("I2c bus is suspended.");
		return -1;	
	}

    msg.flags = !I2C_M_RD;
    msg.addr  = client->addr;
    msg.len   = len;
    msg.buf   = buf;
    //msg.scl_rate = 300 * 1000;    // for Rockchip, etc

    while(retries < 5)
    {
        ret = i2c_transfer(client->adapter, &msg, 1);
        if (ret == 1)break;
        retries++;
    }
    if((retries >= 5))
    {
    	//#if GTP_GESTURE_WAKEUP
	if(vivo_touchscreen_is_support(FS_DCLICK_WAKE))
	{
        if (DOZE_ENABLED == doze_status)
        {
			gtp_reset_guitar(client, 20);
			gtp_enter_doze(ts);
			return ret;
        }
    }
        VIVO_TS_LOG_ERR("I2C Write: 0x%04X, %d bytes failed, errcode: %d! Process reset.", (((u16)(buf[0] << 8)) | buf[1]), len-2, ret);
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
s32 gtp_i2c_read_dbl_check(struct i2c_client *client, u16 addr, u8 *rxbuf, int len)
{
    u8 buf[16] = {0};
    u8 confirm_buf[16] = {0};
    u8 retry = 0;

    while (retry++ < 3)
    {
        memset(buf, 0xAA, 16);
        buf[0] = (u8)(addr >> 8);
        buf[1] = (u8)(addr & 0xFF);
        gtp_i2c_read(client, buf, len + 2);

        memset(confirm_buf, 0xAB, 16);
        confirm_buf[0] = (u8)(addr >> 8);
        confirm_buf[1] = (u8)(addr & 0xFF);
        gtp_i2c_read(client, confirm_buf, len + 2);

        if (!memcmp(buf, confirm_buf, len+2))
        {
            memcpy(rxbuf, confirm_buf+2, len);
            return SUCCESS;
        }
    }
    VIVO_TS_LOG_ERR("I2C read 0x%04X, %d bytes, double check failed!", addr, len);
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

    if (ts->fixed_cfg)
    {
        VIVO_TS_LOG_INF("Ic fixed config, no config sent!");
        return 0;
    }
    else if (ts->pnl_init_error)
    {
        VIVO_TS_LOG_INF("Error occured in init_panel, no config sent");
        return 0;
    }

    VIVO_TS_LOG_INF("Driver send config.");
    for (retry = 0; retry < 5; retry++)
    {
        ret = gtp_i2c_write(client, config , GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH);
        if (ret > 0)
        {
            break;
        }
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
    if (!ts->irq_is_disable)
    {
        ts->irq_is_disable = 1; 
		disable_irq_wake(ts->client->irq);
        disable_irq_nosync(ts->client->irq);
		//mask_irq(irq_goodix);
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
    if (ts->irq_is_disable) 
    {
		enable_irq_wake(ts->client->irq);
        enable_irq(ts->client->irq);
		//unmask_irq(irq_goodix);
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
static void gtp_touch_down(struct goodix_ts_data* ts,s32 id,s32 x,s32 y,s32 w)
{
	static int report_data_sum = 0;
	int cur_report_data_sum = 0;
	static unsigned long report_time = 0;

#if GTP_CHANGE_X2Y
    GTP_SWAP(x, y);
#endif
	cur_report_data_sum = x + y + w;

    if(report_time == 0) {
	    report_time = jiffies;  
	}

	if(report_data_sum != cur_report_data_sum) {
		report_time = jiffies;  
    }

	if(jiffies - report_time > 7*HZ){
		w += 1;
	}

	report_data_sum =x + y + w;

	
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
}

/*******************************************************
Function:
    Report touch release event
Input:
    ts: goodix i2c_client private data
Output:
    None.
*********************************************************/
static void gtp_touch_up(struct goodix_ts_data* ts, s32 id)
{
#if GTP_ICS_SLOT_REPORT
    input_mt_slot(ts->input_dev, id);
    input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, -1);
    //VIVO_TS_LOG_DBG("Touch id[%2d] release!", id);
#else
    input_report_key(ts->input_dev, BTN_TOUCH, 0);
#endif
}

#if GTP_WITH_PEN

static void gtp_pen_init(struct goodix_ts_data *ts)
{
    s32 ret = 0;

    VIVO_TS_LOG_INF("Request input device for pen/stylus.");

    ts->pen_dev = input_allocate_device();
    if (ts->pen_dev == NULL)
    {
        VIVO_TS_LOG_ERR("Failed to allocate input device for pen/stylus.");
        return;
    }

    ts->pen_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
    
#if GTP_ICS_SLOT_REPORT
    input_mt_init_slots(ts->pen_dev, 16);
#else
    ts->pen_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#endif

    set_bit(BTN_TOOL_PEN, ts->pen_dev->keybit);
    set_bit(INPUT_PROP_DIRECT, ts->pen_dev->propbit);
    //set_bit(INPUT_PROP_POINTER, ts->pen_dev->propbit);
 
#if GTP_PEN_HAVE_BUTTON
    input_set_capability(ts->pen_dev, EV_KEY, BTN_STYLUS);
    input_set_capability(ts->pen_dev, EV_KEY, BTN_STYLUS2);
#endif

    input_set_abs_params(ts->pen_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
    input_set_abs_params(ts->pen_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
    input_set_abs_params(ts->pen_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
    input_set_abs_params(ts->pen_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(ts->pen_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);

    ts->pen_dev->name = "goodix-pen";
    ts->pen_dev->id.bustype = BUS_I2C;

    ret = input_register_device(ts->pen_dev);
    if (ret)
    {
        VIVO_TS_LOG_ERR("Register %s input device failed", ts->pen_dev->name);
        return;
    }
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
	
	if(1 == ts->log_switch)
    	VIVO_TS_LOG_DBG("(%d)(%d, %d)[%d]", id, x, y, w);
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

//#if defined(BBK_LARGE_OBJ_SUPPRESSION)
static void gtp_handle_large_obj_supression(struct goodix_ts_data *ts)
{
	VIVO_TS_LOG_INF("TouchScreen large_square_supression!");
	
	input_report_key(ts->input_dev, KEY_TS_LARGE_SUPPRESSION, 1);
	input_sync(ts->input_dev);
	input_report_key(ts->input_dev, KEY_TS_LARGE_SUPPRESSION, 0);
	input_sync(ts->input_dev);
}
//#endif


static void wakeup_system_dclick(struct goodix_ts_data *ts)
{
	input_report_key(ts->input_dev, KEY_WAKEUP, 1);
	input_sync(ts->input_dev);
	//mdelay(10);
	input_report_key(ts->input_dev, KEY_WAKEUP, 0);
	input_sync(ts->input_dev);
}

static void wakeup_system_O(struct goodix_ts_data *ts)
{
	input_report_key(ts->input_dev, KEY_O, 1);
	//mdelay(10);
	input_sync(ts->input_dev);
	input_report_key(ts->input_dev, KEY_O, 0);
	input_sync(ts->input_dev);
}

static void wakeup_system_M(struct goodix_ts_data *ts)
{
	input_report_key(ts->input_dev, KEY_M, 1);
	//mdelay(10);
	input_sync(ts->input_dev);
	input_report_key(ts->input_dev, KEY_M, 0);
	input_sync(ts->input_dev);
}

static void wakeup_system_e(struct goodix_ts_data *ts)
{
	input_report_key(ts->input_dev, KEY_E, 1);
	//mdelay(10);
	input_sync(ts->input_dev);
	input_report_key(ts->input_dev, KEY_E, 0);
	input_sync(ts->input_dev);
}

static void wakeup_system_C(struct goodix_ts_data *ts)
{
	input_report_key(ts->input_dev, KEY_C, 1);
	//mdelay(10);
	input_sync(ts->input_dev);
	input_report_key(ts->input_dev, KEY_C, 0);
	input_sync(ts->input_dev);
}

static void wakeup_system_W(struct goodix_ts_data *ts)
{
	input_report_key(ts->input_dev, KEY_W, 1);
	//mdelay(10);
	input_sync(ts->input_dev);
	input_report_key(ts->input_dev, KEY_W, 0);
	input_sync(ts->input_dev);
}

static void wakeup_system_f(struct goodix_ts_data *ts)
{
	input_report_key(ts->input_dev, KEY_F, 1);
	//mdelay(10);
	input_sync(ts->input_dev);
	input_report_key(ts->input_dev, KEY_F, 0);
	input_sync(ts->input_dev);
}

static void wakeup_system_a(struct goodix_ts_data *ts)
{
	input_report_key(ts->input_dev, KEY_A, 1);
	//mdelay(10);
	input_sync(ts->input_dev);
	input_report_key(ts->input_dev, KEY_A, 0);
	input_sync(ts->input_dev);
}

static void wakeup_system_swipe_down(struct goodix_ts_data *ts)
{
	input_report_key(ts->input_dev, KEY_WAKEUP_SWIPE, 1);
	//mdelay(10);
	input_sync(ts->input_dev);
	input_report_key(ts->input_dev, KEY_WAKEUP_SWIPE, 0);
	input_sync(ts->input_dev);
}

static void wakeup_system_swipe_up(struct goodix_ts_data *ts)
{
	input_report_key(ts->input_dev, KEY_UP, 1);
	//mdelay(10);
	input_sync(ts->input_dev);
	input_report_key(ts->input_dev, KEY_UP, 0);
	input_sync(ts->input_dev);
}

static void wakeup_system_swipe_LR(struct goodix_ts_data *ts, int id, int on)
{
	int key_code = 0;
	if(on & 0x01)
	{
		if(id == 0xAA)
			key_code = KEY_RIGHT;
		if(id == 0xBB)
			key_code = KEY_LEFT;
	}

	if(!key_code)
		return;
	else
	{
		input_report_key(ts->input_dev, key_code, 1);
		mdelay(10);
		input_sync(ts->input_dev);
		input_report_key(ts->input_dev, key_code, 0);
		input_sync(ts->input_dev);
	}
}


/*******************************************************
Function:
    Goodix touchscreen work function
Input:
    work: work struct of goodix_workqueue
Output:
    None.
*********************************************************/
/*qiuguifu add for simulate dclick start*/

#define TRIP_X_AREA	50
#define TRIP_Y_AREA	50
#define DCLICK_TWO_FINGER_AREA_X 100
#define DCLICK_TWO_FINGER_AREA_Y 100
static void goodix_dclick_timer_work_func(struct work_struct *work)
{
	struct goodix_ts_data *sensor = container_of(work, struct goodix_ts_data ,
													dclick_timer_work);
	if(!sensor){
		printk("[GTP]%s:Fail to get info data.\n",__func__);
		return ;
	}
	if (sensor->is_dclick_valide) {
		if (sensor->pressed_count >= 2 && sensor->release_count >= 2)
			wakeup_system_dclick(sensor);
	}

	sensor->is_dclick_valide = false;
	sensor->pressed_count = 0;
	sensor->release_count = 0;
	sensor->first_press_x = -1;
	sensor->first_press_y = -1;
	sensor->second_press_x = -1;
	sensor->second_press_y = -1;
}

static void goodix_dclick_timer_func(unsigned long data)
{
	struct goodix_ts_data *sensor = i2c_get_clientdata(i2c_connect_client);;
	if(!sensor){
		printk("[GTP]%s:Fail to get info data.\n",__func__);
		return ;
	}
	printk(KERN_ERR "[GTP] %s: dclick timer time out\n", __func__);
	printk(KERN_ERR "[GTP] %s: is this time dclick valide -- %s\n",
				__func__, sensor->is_dclick_valide?"yes":"no");
	printk(KERN_ERR "[GTP] %s: pressed_count(%d) release_count(%d)\n",
				__func__, sensor->pressed_count, sensor->release_count);
	printk(KERN_ERR "[GTP] %s: first_press_x(%d) first_press_y(%d)\n",
				__func__, sensor->first_press_x, sensor->first_press_y);
	printk(KERN_ERR "[GTP] %s: second_press_x(%d) second_press_y(%d)\n",
				__func__, sensor->second_press_x, sensor->second_press_y);

	sensor->has_dclick_timer_start = false;
	schedule_work(&sensor->dclick_timer_work);
}

static void goodix_cancel_dclick_trace(struct goodix_ts_data *sensor)
{
	if(!sensor){
		printk("[GTP] %s:Fail to get info data.\n",__func__);
		return ;
	}
	sensor->has_dclick_timer_start = false;
	sensor->is_dclick_valide = false;
	sensor->pressed_count = 0;
	sensor->release_count = 0;
	sensor->first_press_x = -1;
	sensor->first_press_y = -1;
	sensor->second_press_x = -1;
	sensor->second_press_y = -1;
	del_timer_sync(&sensor->dclick_timer);
}

static bool goodix_whether_point_int_dclick_dimension(struct goodix_ts_data *sensor,
				int point_x, int point_y)
{
	if(!sensor){
		printk("[GTP] %s:Fail to get info data.\n",__func__);
		return false;
	}
	if (point_x < sensor->dclick_dimension_x_min 
				|| point_x > sensor->dclick_dimension_x_max
				|| point_y < sensor->dclick_dimension_y_min
				|| point_y > sensor->dclick_dimension_y_max) {
		return false;
	}

	return true;
}

static void goodix_judge_dclick(struct goodix_ts_data *sensor,int x,int y)
{
	int pre_state = sensor->pre_state;
	bool whether_point_legal;
	int finger_0_state = sensor->finger_state;
	
	printk(KERN_ERR "[GTP] %s: enter\n", __func__);

	if (sensor->num_fingers > 1) {
		printk(KERN_ERR "%s: More than one finger pressed on the TP\n", __func__);
		if (sensor->has_dclick_timer_start)
			goodix_cancel_dclick_trace(sensor);
		return;
	}


	if (!pre_state && !finger_0_state) {
		/* Invalide event nothing to do */
		printk(KERN_ERR "[GTP] %s: nothing to do.\n", __func__);
	} else if (!pre_state && finger_0_state) {

		whether_point_legal = goodix_whether_point_int_dclick_dimension(sensor,
										x, y);

		if (!whether_point_legal) {
			if (sensor->has_dclick_timer_start) {
				printk(KERN_ERR "%s: The point not in dclick dimension cancel trace\n",
						__func__);
				goodix_cancel_dclick_trace(sensor);
			}else{
				printk(KERN_ERR "%s: The point not in dclick dimension nothing to do\n",
						__func__);
			}

			return;
		}
		/* the first down event of one time press */
		if (!sensor->has_dclick_timer_start) {
			sensor->first_press_x = x;
			sensor->first_press_y = y;
			sensor->pressed_count++;
			
			printk(KERN_ERR "[GTP] %s: first press start timer\n", __func__);
			mod_timer(&sensor->dclick_timer,
					jiffies + msecs_to_jiffies(500));
			sensor->has_dclick_timer_start = true;
			sensor->is_dclick_valide = true;
		}else{
			sensor->second_press_x = x;
			sensor->second_press_y = y;
			sensor->pressed_count++;

			printk(KERN_ERR "[GTP] %s: second press start x(%d), y(%d)\n", __func__, x, y);

			if ((x - sensor->first_press_x < -DCLICK_TWO_FINGER_AREA_X
						|| x - sensor->first_press_x > DCLICK_TWO_FINGER_AREA_X)
					|| (y - sensor->first_press_y < -DCLICK_TWO_FINGER_AREA_Y
						|| y - sensor->first_press_y > DCLICK_TWO_FINGER_AREA_Y)) {
				printk(KERN_ERR "[GTP] %s: The distance of the two down is too large\n", 
								__func__);
				goodix_cancel_dclick_trace(sensor);
			}
		}
	} else {
		/* the pre_state is down event */
		if (finger_0_state) {
			/* down event trace double click */
			if (sensor->pressed_count == 1 && sensor->release_count == 0) {
				if ((x - sensor->first_press_x < -TRIP_X_AREA 
							|| x - sensor->first_press_x > TRIP_X_AREA) 
						|| (y - sensor->first_press_y < -TRIP_Y_AREA
							|| y - sensor->first_press_y > TRIP_Y_AREA)) {
					printk(KERN_ERR "[GTP] %s: finger triped in one time down\n", __func__);
					goodix_cancel_dclick_trace(sensor);
				}
			}else if (sensor->pressed_count == 2 && sensor->release_count == 1) {
				if ((x - sensor->second_press_x < -TRIP_X_AREA 
							|| x - sensor->second_press_x > TRIP_X_AREA) 
						|| (y - sensor->second_press_y < -TRIP_Y_AREA
							|| y - sensor->second_press_y > TRIP_Y_AREA)) {
					printk(KERN_ERR "[GTP] %s: finger triped in one time down\n", __func__);
					goodix_cancel_dclick_trace(sensor);
				}
			}else{
				/* should not happen nothing to do */
			}
		}else{
			sensor->release_count++;
		}
	}
	//sensor->finger_tracker[0] = finger_0_state;
}

/*qiuguifu add for simulate dclick end*/
static u8 large_touch_mask = 0;	//for large touch mask use
extern bool (*acc_for_ts_judge_dir)(void);
static void goodix_ts_work_func(struct work_struct *work)
{
    u8  end_cmd[3] = {GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF, 0};
    u8  point_data[2 + 1 + 8 * GTP_MAX_TOUCH + 1]={GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF};
    u8  touch_num = 0;
    u8  finger = 0;
    static u16 pre_touch = 0;
    static u8 pre_key = 0;
	static u8 pre_large_press = 0;
	static int finger_flag = 1;
#if GTP_WITH_PEN
    u8 pen_active = 0;
    static u8 pre_pen = 0;
#endif
    u8  key_value = 0;
    u8* coor_data = NULL;
    s32 input_x = 0;
    s32 input_y = 0;
    s32 input_w = 0;
    s32 id = 0;
    s32 i  = 0, j =0;
    s32 ret = -1;
    struct goodix_ts_data *ts = NULL;
    s32 pos = 0;
    u16 touch_index = 0;
    u8 report_num = 0;
	bool is_phone_handstand = false;

#if GTP_GESTURE_WAKEUP
    u8 doze_buf[3] = {0x81, 0x4B};
#endif
    u8 doze_buf[3] = {0x81, 0x4B};
	u8 dclick_data;
	u8 gesture_coordinate[250] = {0x94, 0x20};/*gesture coordinate*/
	s32 err = -1;

    GTP_DEBUG_FUNC();
	VIVO_TS_LOG_DBG("[%s]:Enter work\n", __func__);
    ts = container_of(work, struct goodix_ts_data, work);
    if (ts->enter_update)
    {
        return;
    }
	
	if(vivo_touchscreen_is_support(FS_DCLICK_WAKE))
	{
	if (DOZE_ENABLED == doze_status)
	{	
		if(NULL == acc_for_ts_judge_dir) {
			is_phone_handstand = false;
		} else {
			is_phone_handstand = acc_for_ts_judge_dir();
		}
		if (is_phone_handstand) {
			VIVO_TS_LOG_INF("Phone is handstand\n");
			 // clear 0x814B
			doze_buf[2] = 0x00;
			gtp_i2c_write(i2c_connect_client, doze_buf, 3);
			//gtp_enter_doze(ts);
			goto exit_work_func;
		}
		ret = gtp_i2c_read(i2c_connect_client, doze_buf, 3);
		if(ret < 0)
		{
			VIVO_TS_LOG_ERR("I2C Read request status error. errno:%d ", ret);
           		goto exit_work_func;
		}
		VIVO_TS_LOG_DBG("0x814B = 0x%02X,ret = %d, ts->ts_state.counter = %d", doze_buf[2],ret,ts->ts_state.counter);
		VIVO_TS_LOG_DBG("has_lcd_shutoff = %d, ts_dclick_switch = %d, swipe_down_switch =%d, gesture_switch = %d, gesture_switch_export = %d",ts->has_lcd_shutoff,ts->ts_dclick_switch,ts->swipe_down_switch,ts->gesture_switch,ts->gesture_switch_export);
		if(ts->has_lcd_shutoff && ts->gesture_state)
        	{
           		dclick_data = doze_buf[2];	

			//if(dclick_data !=0x00)
			if((dclick_data == 'w') ||(dclick_data == 'o')||(dclick_data == 'm')\
				||(dclick_data == 'e')||(dclick_data == 'c')||(dclick_data == 0xBB)||(dclick_data == 0xAA)\
				||(dclick_data == 0xAB)||(dclick_data == 0xBA)||(dclick_data == 'f')||(dclick_data == '@'))
		
			{
				/*acquire gesture coordinate */
	            		err = gtp_i2c_read(i2c_connect_client, buf_coordinate_num, 3);
				//GTP_INFO("%s: buf_coordinate_num is %d",__func__,buf_coordinate_num[2]);
				if (err > 0)
				{
					//if(dclick_data != 0xCC)
					//{
						if(buf_coordinate_num[2]==64)
							buf_coordinate_num[2]=63;/*delete the last point*/
						gtp_i2c_read(i2c_connect_client, gesture_coordinate, 2+buf_coordinate_num[2]*4);
					
						for(i=0;i<buf_coordinate_num[2]*4;i++)
						{
							ts->gesture_coordinate[i]= gesture_coordinate[i+2];
							//GTP_INFO("%s: ts->gesture_coordinate[%d] is %d",__func__,i,ts->gesture_coordinate[i]);
						}
						for(i=0,j=0;i<buf_coordinate_num[2]*4; i=i+2)
						{
							ts->buf_gesture[j++]=(u16)( ts->gesture_coordinate[i]|ts->gesture_coordinate[i+1]<<8);
							//GTP_INFO("%s: ts->buf_gesture[%d] is %d",__func__,j,ts->buf_gesture[j]);
						}
					//}
				}else
				{
					VIVO_TS_LOG_INF("TP get buf_gesture_coordinate error\n");
					return;
				}
			}else if((dclick_data!=0) && (dclick_data != 0xCC))    /*Modify for custom gesture*/
			{
				ret = gesture_event_handler(ts->input_dev);
				if (ret >= 0) {
					goto exit_work_func;
				}
			}
			mutex_lock(&ts->gesture_mutex);
			if(atomic_read(&ts->ts_state) == TOUCHSCREEN_GESTURE)
			{
				VIVO_TS_LOG_INF("ts->ts_state is TOUCHSCREEN_GESTURE\n");    
		        	// clear 0x814B
				doze_buf[2] = 0x00;
				gtp_i2c_write(i2c_connect_client, doze_buf, 3);
				//gtp_enter_doze(ts);			
		    	}
			mutex_unlock(&ts->gesture_mutex);
	
			if ((dclick_data == 0xCC) && ts->ts_dclick_switch)
			{
				VIVO_TS_LOG_INF("TP wake up the system with double tap");
				doze_status = DOZE_WAKEUP;
				wakeup_system_dclick(ts);
			}
			else if((dclick_data == 'c') && (ts->gesture_switch & 0x40))
            		{
				VIVO_TS_LOG_INF("TP wake up the system with ==== C ====\n");
				if((ts->buf_gesture[1])<(ts->buf_gesture[buf_coordinate_num[2]*2-1]))
				{
					//doze_status = DOZE_WAKEUP;
					wakeup_system_C(ts);
				}
			}
			else if((dclick_data == 'e') && (ts->gesture_switch & 0x20)) 
            		{
				VIVO_TS_LOG_INF("TP wake up the system with ==== e ====\n");
				//doze_status = DOZE_WAKEUP;
				wakeup_system_e(ts);
			}else if((dclick_data == 'o') && (ts->gesture_switch & 0x04)) 
            		{
				VIVO_TS_LOG_INF("TP wake up the system with ==== O ====\n");
				//doze_status = DOZE_WAKEUP;
				wakeup_system_O(ts);
			}else if((dclick_data == 'w') && (ts->gesture_switch & 0x08)) 
           		{
				VIVO_TS_LOG_INF("TP wake up the system with ==== W ====\n");
				//doze_status = DOZE_WAKEUP;
				wakeup_system_W(ts);
			}
			else if((dclick_data == 0xAB)&& ts->swipe_down_switch) 
			{
				VIVO_TS_LOG_INF("TP wake up the system in swipe down\n");
				//doze_status = DOZE_WAKEUP;
				wakeup_system_swipe_down(ts);
			}
			else if((dclick_data == 0xBA)&& (ts->gesture_switch & 0x02)) 
			{
				VIVO_TS_LOG_INF("TP wake up the system in swipe up\n");
				//doze_status = DOZE_WAKEUP;
				wakeup_system_swipe_up(ts);
			}else if((dclick_data == 'm') && (ts->gesture_switch & 0x10))
            		{
				VIVO_TS_LOG_INF("TP wake up the system with ==== M ====\n");
				//doze_status = DOZE_WAKEUP;
				wakeup_system_M(ts);
			}else if(((dclick_data == 0xAA)||(dclick_data == 0xBB))&& (ts->gesture_switch & 0x01)) 
			{
				VIVO_TS_LOG_INF("TP wake up the system in swipe LR\n");
				//doze_status = DOZE_WAKEUP;
				wakeup_system_swipe_LR(ts,dclick_data,ts->gesture_switch);
			}
			else if((dclick_data == '@') && (ts->gesture_switch_export & 0x02))
            		{
				VIVO_TS_LOG_INF("TP wake up the system with ==== @ ====\n");
				//doze_status = DOZE_WAKEUP;
				wakeup_system_a(ts);
			}else if((dclick_data == 'f') && (ts->gesture_switch_export & 0x04))
            		{
				VIVO_TS_LOG_INF("TP wake up the system with ==== f ====\n");
				//doze_status = DOZE_WAKEUP;
				wakeup_system_f(ts);
			}
			// clear 0x814B
			else if(dclick_data == 0x00)
			{
				VIVO_TS_LOG_INF("dclick_data = 0x00 is a wrong gesture\n");
				doze_buf[2] = 0x00;
        		gtp_i2c_write(i2c_connect_client, doze_buf, 3);
        		gtp_enter_doze(ts);	
			}

			if(ts->has_lcd_shutoff) {
				VIVO_TS_LOG_INF("LCD has shut off");
			}
		}

		if(ts->use_irq)
        	{
            		gtp_irq_enable(ts);
        	}
        	return;
    	}
	}

    ret = gtp_i2c_read(ts->client, point_data, 12);
    if (ret < 0)
    {
        VIVO_TS_LOG_ERR("I2C transfer error. errno:%d ", ret);
        if (ts->use_irq)
        {
            gtp_irq_enable(ts);
        }
        return;
    }

    finger = point_data[GTP_ADDR_LENGTH];
//#if defined(BBK_LARGE_OBJ_SUPPRESSION)
if(vivo_touchscreen_is_support(FS_LARGE_OBJ_SUPPRESSION)) {
	if(finger&(1<<6))
	{	
		ts->largetouch_flag = 1;
		//GTP_INFO("===ts->largetouch_flag is %d!=== ",ts->largetouch_flag);
	}else
	{	
		ts->largetouch_flag = 0;
		//GTP_INFO("===ts->largetouch_flag is %d!=== ",ts->largetouch_flag);
	}
}
//#endif
    if (finger == 0x00)
    {
        if (ts->use_irq)
        {
            gtp_irq_enable(ts);
        }
        return;
    }

    if((finger & 0x80) == 0)
    {
        goto exit_work_func;
    }

	 touch_num = finger & 0x0f;
//#if defined(BBK_LARGE_OBJ_SUPPRESSION)
if(vivo_touchscreen_is_support(FS_LARGE_OBJ_SUPPRESSION)) {
	if(!ts->gtp_is_suspend) 
	{	
		if(touch_num >=3)
		{
			if(1 == ts->log_switch)
	        	VIVO_TS_LOG_INF("====press finger more than 3 !!======= ");
			ts->largetouch_flag = 1;
		}
	}

	if(1 == ts->log_switch)
		VIVO_TS_LOG_INF("+++ts->is_calling is %d ts->largetouch_flag is %d+++",ts->is_calling,ts->largetouch_flag);
	if(ts->is_calling)
	{		
		if(ts->largetouch_flag)
		{	
			gtp_handle_large_obj_supression(ts);
			ts->is_calling = 0;
		}
	}
}
//#endif
	
/*vivo liuyunfeng add for plam rejection*/
	if ((finger & 0x40) == 0x40) 
	{	
		touch_num = 0;
		large_touch_mask = 1;			//large bit enable always				
		if(1 == ts->log_switch)
			VIVO_TS_LOG_INF("Large touch is detected!\n ");
		if(pre_touch)
		{
			for (i = 0; i < GTP_MAX_TOUCH; i++)
			{		
				gtp_touch_up(ts, i);
			}
			input_sync(ts->input_dev); 
			pre_touch = 0;
		}				 
		goto exit_work_func;	  
 	} 
	
	if(1 == large_touch_mask)
	{
		u8 tmp_buf[3] = {0xCE, 0x81};	
		ret = gtp_i2c_read(i2c_connect_client, tmp_buf, 3);  //if touch remain,don't exit large touch mode
		if (ret > 0)
		{
			if((pre_large_press == 0||0 == tmp_buf[2])&&(pre_large_press != tmp_buf[2]))
			{
				VIVO_TS_LOG_INF("0xCE85 = 0x%02X\n", tmp_buf[2]);
				pre_large_press = tmp_buf[2];
			}
			
			if (0 == tmp_buf[2])
			{
				large_touch_mask=0;  //when 0xCE85==0,exit large touch
			}
			else
			{
				touch_num = 0;	  
				if(pre_touch)
				{
					for (i = 0; i < GTP_MAX_TOUCH; i++)
					{		
						gtp_touch_up(ts, i);
					}
					input_sync(ts->input_dev); 
					pre_touch = 0;
				}				 
				goto exit_work_func;	  
			}
		}
	}
/*vivo liuyunfeng add for plam rejection end*/
	
	if (touch_num > GTP_MAX_TOUCH)
    {
        goto exit_work_func;
    }

    if (touch_num > 1)
    {
        u8 buf[8 * GTP_MAX_TOUCH] = {(GTP_READ_COOR_ADDR + 10) >> 8, (GTP_READ_COOR_ADDR + 10) & 0xff};

        ret = gtp_i2c_read(ts->client, buf, 2 + 8 * (touch_num - 1)); 
        memcpy(&point_data[12], &buf[2], 8 * (touch_num - 1));
    }

#if (GTP_HAVE_TOUCH_KEY || GTP_PEN_HAVE_BUTTON)
    key_value = point_data[3 + 8 * touch_num];

    if(key_value || pre_key)
    {
    #if GTP_PEN_HAVE_BUTTON
        if (key_value == 0x40)
        {
            VIVO_TS_LOG_DBG("BTN_STYLUS & BTN_STYLUS2 Down.");
            input_report_key(ts->pen_dev, BTN_STYLUS, 1);
            input_report_key(ts->pen_dev, BTN_STYLUS2, 1);
            pen_active = 1;
        }
        else if (key_value == 0x10)
        {
            VIVO_TS_LOG_DBG("BTN_STYLUS Down, BTN_STYLUS2 Up.");
            input_report_key(ts->pen_dev, BTN_STYLUS, 1);
            input_report_key(ts->pen_dev, BTN_STYLUS2, 0);
            pen_active = 1;
        }
        else if (key_value == 0x20)
        {
            VIVO_TS_LOG_DBG("BTN_STYLUS Up, BTN_STYLUS2 Down.");
            input_report_key(ts->pen_dev, BTN_STYLUS, 0);
            input_report_key(ts->pen_dev, BTN_STYLUS2, 1);
            pen_active = 1;
        }
        else
        {
            VIVO_TS_LOG_DBG("BTN_STYLUS & BTN_STYLUS2 Up.");
            input_report_key(ts->pen_dev, BTN_STYLUS, 0);
            input_report_key(ts->pen_dev, BTN_STYLUS2, 0);
            if ( (pre_key == 0x40) || (pre_key == 0x20) ||
                 (pre_key == 0x10)
               )
            {
                pen_active = 1;
            }
        }
        if (pen_active)
        {
            touch_num = 0;      // shield pen point
            //pre_touch = 0;    // clear last pen status
        }
    #endif

    #if GTP_HAVE_TOUCH_KEY
        if (!pre_touch)
        {
            for (i = 0; i < GTP_MAX_KEY_NUM; i++)
            {
            #if GTP_DEBUG_ON
                for (ret = 0; ret < 4; ++ret)
                {
                    if (key_codes[ret] == touch_key_array[i])
                    {
                        VIVO_TS_LOG_DBG("Key: %s %s", key_names[ret], (key_value & (0x01 << i)) ? "Down" : "Up");
                        break;
                    }
                }
            #endif
                input_report_key(ts->input_dev, touch_key_array[i], key_value & (0x01<<i));  
            }
            touch_num = 0;  // shield fingers
        }
    #endif
    }
#endif
    pre_key = key_value;

   	VIVO_TS_LOG_DBG("pre_touch:%02x, finger:%02x.", pre_touch, finger);

	
#if GTP_ICS_SLOT_REPORT

#if GTP_WITH_PEN
    if (pre_pen && (touch_num == 0))
    {
        VIVO_TS_LOG_DBG("Pen touch UP(Slot)!");
        gtp_pen_up(0);
        pen_active = 1;
        pre_pen = 0;
    }
#endif
    if (pre_touch || touch_num)
    {
        coor_data = &point_data[3];

        if(touch_num)
        {
            id = coor_data[pos] & 0x0F;

        #if GTP_WITH_PEN
            id = coor_data[pos];
            if ((id & 0x80))
            {
                VIVO_TS_LOG_DBG("Pen touch DOWN(Slot)!");
                input_x  = coor_data[pos + 1] | (coor_data[pos + 2] << 8);
                input_y  = coor_data[pos + 3] | (coor_data[pos + 4] << 8);
                input_w  = coor_data[pos + 5] | (coor_data[pos + 6] << 8);

                gtp_pen_down(input_x, input_y, input_w, 0);
                pre_pen = 1;
                pre_touch = 0;
                pen_active = 1;
            }
        #endif

            touch_index |= (0x01<<id);
        }

       	VIVO_TS_LOG_DBG("id = %d,touch_index = 0x%x, pre_touch = 0x%x",id, touch_index,pre_touch);

		/*qiuguifu add for simulate dclick start*/
		if(ts->ts_dclick_simulate_switch == 1){
			input_x  = coor_data[1] | (coor_data[2] << 8);
		       input_y  = coor_data[3] | (coor_data[4] << 8);
		       input_w  = coor_data[5] | (coor_data[6] << 8);
			ts->finger_state = touch_index & (0x01<<0);
			ts->num_fingers = touch_num;
			goodix_judge_dclick(ts,input_x,input_y);
		}
		ts->pre_state = (touch_index & (0x01<<0));
		/*qiuguifu add for simulate dclick end*/
		
		for (i = 0; i < GTP_MAX_TOUCH; i++)
	    {
	        #if GTP_WITH_PEN
	            if (pre_pen == 1)
	            {
	                break;
	            }
	        #endif

	            if ((touch_index & (0x01<<i)))
	            {
	                input_x  = coor_data[pos + 1] | (coor_data[pos + 2] << 8);
	                input_y  = coor_data[pos + 3] | (coor_data[pos + 4] << 8);
	                input_w  = coor_data[pos + 5] | (coor_data[pos + 6] << 8);
	//[PLATFORM]2014/04/11, Add for calibratation
#ifdef CONFIG_TCT_8X16_POP8LTE
					input_x = 800 -input_x;
					input_y =1280 - input_y;
#endif
	//[PLATFORM]
	                gtp_touch_down(ts, id, input_x, input_y, input_w);
	                pre_touch |= 0x01 << i;

	                report_num++;
	                if (report_num < touch_num)
	                {
	                    pos += 8;
	                    id = coor_data[pos] & 0x0F;
	                    touch_index |= (0x01<<id);
	                }
	            }
	            else
	            {
	                gtp_touch_up(ts, i);
	                pre_touch &= ~(0x01 << i);
	            }

				if(pre_touch != 0 || finger != 0x80)
				{
					if(finger_flag == 1)
					{
						finger_flag = 0;
						VIVO_TS_LOG_INF("==finger down== id:%d, input_x:%d, input_y:%d, input_w:%d\n", id, input_x, input_y, input_w);
					}
				}

		}
	}
				
	if(pre_touch == 0 && finger == 0x80)
	{
		finger_flag = 1;
		VIVO_TS_LOG_INF("=========finger up===========\n");
	}	
#else

    if (touch_num)
    {
        for (i = 0; i < touch_num; i++)
        {
            coor_data = &point_data[i * 8 + 3];

            id = coor_data[0] & 0x0F;
            input_x  = coor_data[1] | (coor_data[2] << 8);
            input_y  = coor_data[3] | (coor_data[4] << 8);
            input_w  = coor_data[5] | (coor_data[6] << 8);

        #if GTP_WITH_PEN
            id = coor_data[0];
            if (id & 0x80)
            {
                VIVO_TS_LOG_DBG("Pen touch DOWN!");
                gtp_pen_down(input_x, input_y, input_w, 0);
                pre_pen = 1;
                pen_active = 1;
                break;
            }
            else
        #endif
            {
                gtp_touch_down(ts, id, input_x, input_y, input_w);
            }
        }
    }
    else if (pre_touch)
    {
    #if GTP_WITH_PEN
        if (pre_pen == 1)
        {
			VIVO_TS_LOG_DBG("Pen touch UP!");
            gtp_pen_up(0);
            pre_pen = 0;
            pen_active = 1;
        }
        else
    #endif
        {
			VIVO_TS_LOG_DBG("Touch Release!");
            gtp_touch_up(ts, 0);
        }
    }

    pre_touch = touch_num;
#endif

#if GTP_WITH_PEN
    if (pen_active)
    {
        pen_active = 0;
        input_sync(ts->pen_dev);
    }
    else
#endif
    {
        input_sync(ts->input_dev);
    }

exit_work_func:
    if(!ts->gtp_rawdiff_mode)
    {
        ret = gtp_i2c_write(ts->client, end_cmd, 3);
        if (ret < 0)
        {
            VIVO_TS_LOG_INF("I2C write end_cmd error!");
        }
    }
    if (ts->use_irq)
    {
        gtp_irq_enable(ts);
    }
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
    struct goodix_ts_data *ts = container_of(timer, struct goodix_ts_data, timer);

    GTP_DEBUG_FUNC();

    queue_work(goodix_wq, &ts->work);
    hrtimer_start(&ts->timer, ktime_set(0, (GTP_POLL_TIME+6)*1000000), HRTIMER_MODE_REL);
    return HRTIMER_NORESTART;
}

static void gtp_irq_err_work_func(struct work_struct *work)
{
	struct goodix_ts_data *ts = NULL;
	int count = 0;
	
	ts = container_of(work, struct goodix_ts_data, irq_err_work);
    if (ts->enter_update)
    {
        return;
    }
	
	if (ts->has_lcd_shutoff) 
	{
		while(qup_i2c_suspended > 0 && count < 80) 
		{
			msleep(5);
			count++;
		}
		if (count == 80) 
		{
			VIVO_TS_LOG_ERR("[GTP] The i2c bus still suspend after 100 times try\n");
			return; 
		}
	}

	gtp_irq_disable(ts);
	queue_work(goodix_wq, &ts->work);

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
	int count = 0;

	VIVO_TS_LOG_DBG("enter irq ..\n");
    GTP_DEBUG_FUNC();
	wake_lock_timeout(&ts->suspend_wakelock, 2*HZ);
	if (ts->has_lcd_shutoff) 
	{
		while(qup_i2c_suspended > 0 && count < 20) 
		{
			msleep(5);
			count++;
		}
		
		if (count == 20) 
		{
			VIVO_TS_LOG_ERR("[GTP] The i2c bus still suspend after 20 times try\n");
			queue_work(ts->irq_err_workqueue, &ts->irq_err_work);
			return IRQ_HANDLED;
		}
	}

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
void gtp_int_sync(struct goodix_ts_data *ts, s32 ms)
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
	VIVO_TS_LOG_INF("enter...%s\n", __func__);

    if (ts->enter_update)
    {
		VIVO_TS_LOG_INF("The IC enter Firmware update mode...%s\n", __func__);
        return;
    }

	/* This reset sequence will selcet I2C slave address */
	gpio_direction_output(ts->pdata->reset_gpio, 0);
	msleep(ms);

	if (ts->client->addr == GTP_I2C_ADDRESS_HIGH)
		gpio_direction_output(ts->pdata->irq_gpio, 1);
	else
		gpio_direction_output(ts->pdata->irq_gpio, 0);

	usleep(RESET_DELAY_T3_US);
	gpio_direction_output(ts->pdata->reset_gpio, 1);
	msleep(RESET_DELAY_T4);

	//gpio_direction_input(ts->pdata->reset_gpio);//delete xfsrt
	gtp_int_sync(ts, 50);
//#if GTP_ESD_PROTECT
	if(vivo_touchscreen_is_support(FS_ANTI_ESD))
		gtp_init_ext_watchdog(ts->client);
//#endif
}

#if defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_FB)
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
    u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 8};

    GTP_DEBUG_FUNC();

    VIVO_TS_LOG_DBG("Entering gesture mode.");
    while(retry++ < 5)
    {
        i2c_control_buf[0] = 0x80;
        i2c_control_buf[1] = 0x46;
        ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
        if (ret < 0)
        {
            VIVO_TS_LOG_ERR("failed to set doze flag into 0x8046, %d", retry);
            continue;
        }
        i2c_control_buf[0] = 0x80;
        i2c_control_buf[1] = 0x40;
        ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
        if (ret > 0)
        {
            doze_status = DOZE_ENABLED;
            VIVO_TS_LOG_INF("Gesture mode enabled.");
            return ret;
        }
        msleep(10);
    }
    VIVO_TS_LOG_ERR("GTP send gesture cmd failed.");
    return ret;
}
/*******************************************************
Function:
    Enter sleep mode.
Input:
    ts: private data.
Output:
    Executive outcomes.
       1: succeed, otherwise failed.
*******************************************************/
static s8 gtp_enter_sleep(struct goodix_ts_data * ts)
{
    s8 ret = -1;
    s8 retry = 0;
    u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 5};

    GTP_DEBUG_FUNC();

    GTP_GPIO_OUTPUT(ts->pdata->irq_gpio, 0);
    msleep(5);

    while(retry++ < 5)
    {
        ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
        if (ret > 0)
        {
            VIVO_TS_LOG_INF("GTP enter sleep!");

            return ret;
        }
        msleep(10);
    }
    VIVO_TS_LOG_ERR("GTP send sleep cmd failed.");
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
        >0: succeed, otherwise: failed.
*******************************************************/
static s8 gtp_wakeup_sleep(struct goodix_ts_data * ts)
{
    u8 retry = 0;
    s8 ret = -1;

    GTP_DEBUG_FUNC();
	
#if GTP_POWER_CTRL_SLEEP
    while(retry++ < 5)
    {
        gtp_reset_guitar(ts->client, 20);

        VIVO_TS_LOG_INF("GTP wakeup sleep.");
        return 1;
    }
#else
    while(retry++ < 10)
    {
    if(vivo_touchscreen_is_support(FS_DCLICK_WAKE))
	{
        if (DOZE_WAKEUP != doze_status)
        {
            VIVO_TS_LOG_INF("Powerkey wakeup.");
        }
        else
        {
            VIVO_TS_LOG_INF("Gesture wakeup.");
        }
        doze_status = DOZE_DISABLED;
        gtp_irq_disable(ts);
        gtp_reset_guitar(ts->client, 10);
        gtp_irq_enable(ts);

    }else{
        GTP_GPIO_OUTPUT(ts->pdata->irq_gpio, 1);
        msleep(5);
    }
    
        ret = gtp_i2c_test(ts->client);
        if (ret > 0)
        {
            VIVO_TS_LOG_INF("GTP wakeup sleep.");

		/*reduce esd protect for double click time*/
        #if (0)//!GTP_GESTURE_WAKEUP)
            {
                gtp_int_sync(ts, 25);
            #if GTP_ESD_PROTECT
                gtp_init_ext_watchdog(ts->client);
            #endif
            }
        #endif

            return ret;
        }
        gtp_reset_guitar(ts->client, 20);
    }
#endif

    VIVO_TS_LOG_ERR("GTP wakeup sleep failed.");
    return ret;
}
static s8 gtp_irq_reset(struct goodix_ts_data * ts)
{
    s8 ret = -1;

	VIVO_TS_LOG_INF("~~~enter~~~");
	
	GTP_GPIO_OUTPUT(ts->pdata->irq_gpio, 1);
    mdelay(5);
	ret = gtp_i2c_test(ts->client);
    if (ret > 0)
    {
        VIVO_TS_LOG_INF("gtp_irq_reset.");
        //gtp_int_sync(ts, 25); 
		gpio_direction_output(ts->pdata->irq_gpio, 0);
		mdelay(25);
		gpio_direction_input(ts->pdata->irq_gpio);
		return ret;
    }
	gtp_reset_guitar(ts->client, 20);
	return ret;
}
/*Annotation for Config update */
//#if GTP_DRIVER_SEND_CFG
#if 0
static s32 gtp_get_info(struct goodix_ts_data *ts)
{
    u8 opr_buf[6] = {0};
    s32 ret = 0;

    ts->abs_x_max = GTP_MAX_WIDTH;
    ts->abs_y_max = GTP_MAX_HEIGHT;
    ts->int_trigger_type = GTP_INT_TRIGGER;
  
    opr_buf[0] = (u8)((GTP_REG_CONFIG_DATA+1) >> 8);
    opr_buf[1] = (u8)((GTP_REG_CONFIG_DATA+1) & 0xFF);

    ret = gtp_i2c_read(ts->client, opr_buf, 6);
    if (ret < 0)
    {
        return FAIL;
    }

    ts->abs_x_max = (opr_buf[3] << 8) + opr_buf[2];
    ts->abs_y_max = (opr_buf[5] << 8) + opr_buf[4];

    opr_buf[0] = (u8)((GTP_REG_CONFIG_DATA+6) >> 8);
    opr_buf[1] = (u8)((GTP_REG_CONFIG_DATA+6) & 0xFF);

    ret = gtp_i2c_read(ts->client, opr_buf, 3);
    if (ret < 0)
    {
        return FAIL;
    }
    ts->int_trigger_type = opr_buf[2] & 0x03;
    
    VIVO_TS_LOG_INF("X_MAX = %d, Y_MAX = %d, TRIGGER = 0x%02x",
            ts->abs_x_max,ts->abs_y_max,ts->int_trigger_type);

    return SUCCESS;    
}
#endif
u8 driver_num = 0;
u8 sensor_num = 0;
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
#if 1
   struct goodix_ts_platform_data *pdata = ts->pdata;
    u8 *cfg_info_group1 = &pdata->config_data[0][GTP_ADDR_LENGTH];
    u8 *cfg_info_group2 = &pdata->config_data[1][GTP_ADDR_LENGTH];
    u8 *cfg_info_group3 = &pdata->config_data[2][GTP_ADDR_LENGTH];
    u8 *cfg_info_group4 = &pdata->config_data[3][GTP_ADDR_LENGTH];
    u8 *cfg_info_group5 = &pdata->config_data[4][GTP_ADDR_LENGTH];
    u8 *cfg_info_group6 = &pdata->config_data[5][GTP_ADDR_LENGTH];
    u8 cfg_cmp = 0;
	
    u8 *send_cfg_buf[] = {cfg_info_group1, cfg_info_group2, cfg_info_group3,
                        cfg_info_group4, cfg_info_group5, cfg_info_group6};
    int *cfg_info_len = pdata->config_data_len;
#else
    u8 cfg_info_group1[] = CTP_CFG_GROUP1;
    u8 cfg_info_group2[] = CTP_CFG_GROUP2;
    u8 cfg_info_group3[] = CTP_CFG_GROUP3;
    u8 cfg_info_group4[] = CTP_CFG_GROUP4;
    u8 cfg_info_group5[] = CTP_CFG_GROUP5;
    u8 cfg_info_group6[] = CTP_CFG_GROUP6;
	u8 cfg_cmp = 0;
    u8 *send_cfg_buf[] = {cfg_info_group1, cfg_info_group2, cfg_info_group3,
                        cfg_info_group4, cfg_info_group5, cfg_info_group6};
    u8 cfg_info_len[] = { CFG_GROUP_LEN(cfg_info_group1),
                          CFG_GROUP_LEN(cfg_info_group2),
                          CFG_GROUP_LEN(cfg_info_group3),
                          CFG_GROUP_LEN(cfg_info_group4),
                          CFG_GROUP_LEN(cfg_info_group5),
                          CFG_GROUP_LEN(cfg_info_group6)};
#endif

    GTP_DEBUG_FUNC();
    VIVO_TS_LOG_DBG("Config Groups\' Lengths: %d, %d, %d, %d, %d, %d", 
        	cfg_info_len[0], cfg_info_len[1], cfg_info_len[2], cfg_info_len[3],
        	cfg_info_len[4], cfg_info_len[5]);

        ret = gtp_i2c_read_dbl_check(ts->client, 0x41E4, opr_buf, 1);
        if (SUCCESS == ret) 
        {
            if (opr_buf[0] != 0xBE)
            {
                ts->fw_error = 1;
                VIVO_TS_LOG_ERR("Firmware error, no config sent!");
                return -1;
            }
        }

    if ((!cfg_info_len[1]) && (!cfg_info_len[2]) &&
        (!cfg_info_len[3]) && (!cfg_info_len[4]) &&
        (!cfg_info_len[5]))
    {
        sensor_id = 0; 
    }
    else
    {
        ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_SENSOR_ID, &sensor_id, 1);
        if (SUCCESS == ret)
        {
            if (sensor_id >= 0x06)
            {
                VIVO_TS_LOG_ERR("Invalid sensor_id(0x%02X), No Config Sent!", sensor_id);
                ts->pnl_init_error = 1;
                return -1;
            }
        }
        else
        {
            VIVO_TS_LOG_ERR("Failed to get sensor_id, No config sent!");
            ts->pnl_init_error = 1;
            return -1;
        }
        VIVO_TS_LOG_INF("Sensor_ID: %d", sensor_id);
    }
    ts->gtp_cfg_len = cfg_info_len[sensor_id];
    VIVO_TS_LOG_INF("CTP_CONFIG_GROUP%d used, config length: %d", sensor_id + 1, ts->gtp_cfg_len);

    if (ts->gtp_cfg_len < GTP_CONFIG_MIN_LENGTH)
    {
        VIVO_TS_LOG_ERR("Config Group %d is INVALID CONFIG GROUP(Len: %d)! NO Config Sent! You need to check you header file CFG_GROUP section!", sensor_id+1, ts->gtp_cfg_len);
        ts->pnl_init_error = 1;
        return -1;
    }

        ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_CONFIG_DATA, &opr_buf[0], 1);
        
        if (ret == SUCCESS)
        {
            VIVO_TS_LOG_DBG("CFG_GROUP%d Config Version: %d, 0x%02X; IC Config Version: %d, 0x%02X", sensor_id+1,
                        send_cfg_buf[sensor_id][0], send_cfg_buf[sensor_id][0], opr_buf[0], opr_buf[0]);
/*Annotation for Config update */
#if 0
            if (opr_buf[0] < 90)
            {
                grp_cfg_version = send_cfg_buf[sensor_id][0];       //backup group config version
				//send_cfg_buf[sensor_id][0] = 0x00;
                ts->fixed_cfg = 0;
            }
            else	// treated as fixed config, not send config
            {
                VIVO_TS_LOG_INF("Ic fixed config with config version(%d, 0x%02X)", opr_buf[0], opr_buf[0]);
                ts->fixed_cfg = 1;
                gtp_get_info(ts);
                return 0;
            }
#else
			if (opr_buf[0] > send_cfg_buf[sensor_id][0])
            {
                grp_cfg_version = send_cfg_buf[sensor_id][0];       // backup group config version
                send_cfg_buf[sensor_id][0] = 0x00;
                ts->fixed_cfg = 0;
                cfg_cmp = 1;
            }
#endif
        }
        else
        {
            VIVO_TS_LOG_ERR("Failed to get ic config version!No config sent!");
            return -1;
        }

    memset(&config[GTP_ADDR_LENGTH], 0, GTP_CONFIG_MAX_LENGTH);
    memcpy(&config[GTP_ADDR_LENGTH], send_cfg_buf[sensor_id], ts->gtp_cfg_len);

#if GTP_CUSTOM_CFG
    config[RESOLUTION_LOC]     = (u8)GTP_MAX_WIDTH;
    config[RESOLUTION_LOC + 1] = (u8)(GTP_MAX_WIDTH>>8);
    config[RESOLUTION_LOC + 2] = (u8)GTP_MAX_HEIGHT;
    config[RESOLUTION_LOC + 3] = (u8)(GTP_MAX_HEIGHT>>8);

    if (GTP_INT_TRIGGER == 0)  //RISING
    {
        config[TRIGGER_LOC] &= 0xfe; 
    }
    else if (GTP_INT_TRIGGER == 1)  //FALLING
    {
        config[TRIGGER_LOC] |= 0x01;
    }
#endif// GTP_CUSTOM_CFG

    check_sum = 0;
    for (i = GTP_ADDR_LENGTH; i < ts->gtp_cfg_len; i++)
    {
        check_sum += config[i];
    }
    config[ts->gtp_cfg_len] = (~check_sum) + 1;

#else // driver not send config

    ts->gtp_cfg_len = GTP_CONFIG_MAX_LENGTH;
    ret = gtp_i2c_read(ts->client, config, ts->gtp_cfg_len + GTP_ADDR_LENGTH);
    if (ret < 0)
    {
        VIVO_TS_LOG_ERR("Read Config Failed, Using Default Resolution & INT Trigger!");
        ts->abs_x_max = GTP_MAX_WIDTH;
        ts->abs_y_max = GTP_MAX_HEIGHT;
        ts->int_trigger_type = GTP_INT_TRIGGER;
    }

#endif //GTP_DRIVER_SEND_CFG
    if ((ts->abs_x_max == 0) && (ts->abs_y_max == 0))
    {
        ts->abs_x_max = (config[RESOLUTION_LOC + 1] << 8) + config[RESOLUTION_LOC];
        ts->abs_y_max = (config[RESOLUTION_LOC + 3] << 8) + config[RESOLUTION_LOC + 2];
        ts->int_trigger_type = (config[TRIGGER_LOC]) & 0x03; 
    }
    	driver_num = (config[CFG_LOC_DRVA_NUM]&0x1F) + (config[CFG_LOC_DRVB_NUM]&0x1F);
        sensor_num = (config[CFG_LOC_SENS_NUM]&0x0F) + ((config[CFG_LOC_SENS_NUM]>>4)&0x0F);
    #if GTP_DRIVER_SEND_CFG
        ret = gtp_send_cfg(ts->client);
        if (ret < 0)
        {
            VIVO_TS_LOG_ERR("Send config error.");
        }

		if(cfg_cmp)
        {
        	/*set config version to CTP_CFG_GROUP, for resume to send config*/
        	config[GTP_ADDR_LENGTH] = grp_cfg_version;
        	check_sum = 0;
        	for (i = GTP_ADDR_LENGTH; i < ts->gtp_cfg_len; i++)
        	{
            	check_sum += config[i];
        	}
        	config[ts->gtp_cfg_len] = (~check_sum) + 1;
			msleep(500);
        	ret = gtp_send_cfg(ts->client);
			if (ret < 0)
			{
				VIVO_TS_LOG_ERR("Send config error.");
			}
        }
    #endif
        VIVO_TS_LOG_INF("X_MAX: %d, Y_MAX: %d, TRIGGER: 0x%02x", ts->abs_x_max,ts->abs_y_max,ts->int_trigger_type);
    
    msleep(350);

	/*[BUGFIX]Show TP config version*/
	ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_CONFIG_DATA,
			&grp_cfg_version, 1);
	if(SUCCESS != ret)
	{
		 VIVO_TS_LOG_ERR("Failed to get ic config version!");
	}
	/*[BUGFIX]Show TP config version*/

	//g_tp_cfg_ver = grp_cfg_version;
    return 0;
}

static ssize_t gt91xx_config_read_proc(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
    char *ptr = page;
    char temp_data[GTP_CONFIG_MAX_LENGTH + 2] = {0x80, 0x47};
    int i;

    if (*ppos)
    {
        return 0;
    }
    ptr += sprintf(ptr, "==== GT9XX config init value====\n");

    for (i = 0 ; i < GTP_CONFIG_MAX_LENGTH ; i++)
    {
        ptr += sprintf(ptr, "0x%02X ", config[i + 2]);

        if (i % 8 == 7)
            ptr += sprintf(ptr, "\n");
    }

    ptr += sprintf(ptr, "\n");

    ptr += sprintf(ptr, "==== GT9XX config real value====\n");
    gtp_i2c_read(i2c_connect_client, temp_data, GTP_CONFIG_MAX_LENGTH + 2);
    for (i = 0 ; i < GTP_CONFIG_MAX_LENGTH ; i++)
    {
        ptr += sprintf(ptr, "0x%02X ", temp_data[i+2]);

        if (i % 8 == 7)
            ptr += sprintf(ptr, "\n");
    }
    *ppos += ptr - page;
    return (ptr - page);
}

static ssize_t gt91xx_config_write_proc(struct file *filp, const char __user *buffer, size_t count, loff_t *off)
{
    s32 ret = 0;

    VIVO_TS_LOG_DBG("write count %zd", count);

    if (count > GTP_CONFIG_MAX_LENGTH)
    {
        VIVO_TS_LOG_ERR("size not match [%d:%zd]", GTP_CONFIG_MAX_LENGTH, count);
        return -EFAULT;
    }

    if (copy_from_user(&config[2], buffer, count))
    {
        VIVO_TS_LOG_ERR("copy from user fail");
        return -EFAULT;
    }

    ret = gtp_send_cfg(i2c_connect_client);

    if (ret < 0)
    {
        VIVO_TS_LOG_ERR("send config failed.");
    }

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
s32 gtp_read_version(struct i2c_client *client, u16* version)
{
    s32 ret = -1;
    u8 buf[8] = {GTP_REG_VERSION >> 8, GTP_REG_VERSION & 0xff};

    GTP_DEBUG_FUNC();

    ret = gtp_i2c_read(client, buf, sizeof(buf));
    if (ret < 0)
    {
        VIVO_TS_LOG_ERR("GTP read version failed");
        return ret;
    }

    if (version)
    {
        *version = (buf[7] << 8) | buf[6];
    }
    if (buf[5] == 0x00)
    {
        VIVO_TS_LOG_INF("IC Version: %c%c%c_%02x%02x", buf[2], buf[3], buf[4], buf[7], buf[6]);
    }
    else
    {
        VIVO_TS_LOG_INF("IC Version: %c%c%c%c_%02x%02x", buf[2], buf[3], buf[4], buf[5], buf[7], buf[6]);
    }
    return ret;
}

/*******************************************************
Function:
	Read firmware version
Input:
	client:  i2c device
	version: buffer to keep ic firmware version
Output:
	read operation return.
	0: succeed, otherwise: failed
*******************************************************/
static int gtp_read_fw_version(struct i2c_client *client, u16 *version)
{
	int ret = 0;
	u8 buf[GTP_FW_VERSION_BUFFER_MAXSIZE] = {
		GTP_REG_FW_VERSION >> 8, GTP_REG_FW_VERSION & 0xff };

	ret = gtp_i2c_read(client, buf, sizeof(buf));
	if (ret < 0) {
		VIVO_TS_LOG_ERR("GTP read version failed.\n");
		return -EIO;
	}

	if (version)
		*version = (buf[3] << 8) | buf[2];

	return ret;
}
/*******************************************************
Function:
	Read and check chip id.
Input:
	client:  i2c device
Output:
	read operation return.
	0: succeed, otherwise: failed
*******************************************************/
static int gtp_check_product_id(struct i2c_client *client)
{
	int ret = 0;
	char product_id[GTP_PRODUCT_ID_MAXSIZE];
	//struct goodix_ts_data *ts = i2c_get_clientdata(client);
	/* 04 bytes are used for the Product-id in the register space.*/
	u8 buf[GTP_PRODUCT_ID_BUFFER_MAXSIZE] =	{
		GTP_REG_PRODUCT_ID >> 8, GTP_REG_PRODUCT_ID & 0xff };

	ret = gtp_i2c_read(client, buf, sizeof(buf));
	if (ret < 0) {
		VIVO_TS_LOG_ERR("GTP read product_id failed.\n");
		return -EIO;
	}

	if (buf[5] == 0x00) {
		/* copy (GTP_PRODUCT_ID_MAXSIZE - 1) from buffer. Ex: 915 */
		strlcpy(product_id, &buf[2], GTP_PRODUCT_ID_MAXSIZE - 1);
	} else {
		if (buf[5] == 'S' || buf[5] == 's')
			chip_gt9xxs = 1;
		/* copy GTP_PRODUCT_ID_MAXSIZE from buffer. Ex: 915s */
		strlcpy(product_id, &buf[2], GTP_PRODUCT_ID_MAXSIZE);
	}

	dev_info(&client->dev, "Goodix Product ID = %s\n", product_id);

	//ret = strcmp(product_id, ts->pdata->product_id);
	//if (ret != 0)
	//return -EINVAL;

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

    while(retry++ < 5)
    {
        ret = gtp_i2c_read(client, test, 3);
        if (ret > 0)
        {
            return ret;
        }
        VIVO_TS_LOG_ERR("GTP i2c test failed time %d.",retry);
        msleep(10);
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
#if 1
static int gtp_request_io_port(struct goodix_ts_data *ts)
{
	struct i2c_client *client = ts->client;
	struct goodix_ts_platform_data *pdata = ts->pdata;
	int ret;
	
	if (gpio_is_valid(pdata->irq_gpio)) {
		ret = gpio_request(pdata->irq_gpio, "goodix_ts_irq_gpio");
		if (ret) {
			VIVO_TS_LOG_ERR("Unable to request irq gpio [%d]\n",
				pdata->irq_gpio);
			goto err_pwr_off;
		}
		ret = gpio_direction_input(pdata->irq_gpio);
		if (ret) {
			VIVO_TS_LOG_ERR("Unable to set direction for irq gpio [%d]\n",
				pdata->irq_gpio);
			goto err_free_irq_gpio;
		}
	} else {
		VIVO_TS_LOG_ERR("Invalid irq gpio [%d]!\n",
			pdata->irq_gpio);
		ret = -EINVAL;
		goto err_pwr_off;
	}

	if (gpio_is_valid(pdata->reset_gpio)) {
		ret = gpio_request(pdata->reset_gpio, "goodix_ts_reset_gpio");
		if (ret) {
			VIVO_TS_LOG_ERR("Unable to request reset gpio [%d]\n",
				pdata->reset_gpio);
			goto err_free_irq_gpio;
		}
		ret = gpio_direction_output(pdata->reset_gpio, 0);
		if (ret) {
			VIVO_TS_LOG_ERR("Unable to set direction for reset gpio [%d]\n",
				pdata->reset_gpio);
			goto err_free_reset_gpio;
		}
	} else {
		VIVO_TS_LOG_ERR("Invalid irq gpio [%d]!\n",
			pdata->reset_gpio);
		ret = -EINVAL;
		goto err_free_irq_gpio;
	}
//zhj add for vdd gpio
	if(0 == strcmp(global_vdd_mode,"gpio_mode")){
		if (gpio_is_valid(pdata->vdd_gpio)) {
			ret = gpio_request(pdata->vdd_gpio, "goodix_ts_vdd_gpio");
			if (ret) {
				VIVO_TS_LOG_ERR("Unable to request vdd gpio [%d]\n",
					pdata->vdd_gpio);
				goto err_free_irq_gpio;
			}
		} else {
			VIVO_TS_LOG_ERR("Invalid irq gpio [%d]!\n",
				pdata->vdd_gpio);
			ret = -EINVAL;
			goto err_free_vdd_gpio;
		}
	}
//zhj add end
	/* IRQ GPIO is an input signal, but we are setting it to output
	  * direction and pulling it down, to comply with power up timing
	  * requirements, mentioned in power up timing section of device
	  * datasheet.
	  */
	ret = gpio_direction_output(pdata->irq_gpio, 0);
	if (ret)
		dev_warn(&client->dev,
			"pull down interrupt gpio failed\n");
	ret = gpio_direction_output(pdata->reset_gpio, 0);
	if (ret)
		dev_warn(&client->dev,
			"pull down reset gpio failed\n");
	//gtp_reset_guitar(client, 20);

	//irq_goodix = irq_to_desc(ts->client->irq);
	
	return ret;

err_free_reset_gpio:
	if (gpio_is_valid(pdata->reset_gpio)) {
		gpio_free(pdata->reset_gpio);
	}
err_free_irq_gpio:
	if (gpio_is_valid(pdata->irq_gpio)) {
		gpio_free(pdata->irq_gpio);
	}
err_free_vdd_gpio:
	if (gpio_is_valid(pdata->vdd_gpio)) {
		gpio_free(pdata->vdd_gpio);
	}

err_pwr_off:
	return ret;
}
#else
static s8 gtp_request_io_port(struct goodix_ts_data *ts)
{
    s32 ret = 0;

    GTP_DEBUG_FUNC();
    ret = GTP_GPIO_REQUEST(GTP_INT_PORT, "GTP_INT_IRQ");
    if (ret < 0)
    {
        VIVO_TS_LOG_ERR("Failed to request GPIO:%d, ERRNO:%d", (s32)GTP_INT_PORT, ret);
        ret = -ENODEV;
    }
    else
    {
        GTP_GPIO_AS_INT(GTP_INT_PORT);
        ts->client->irq = GTP_INT_IRQ;
    }

    ret = GTP_GPIO_REQUEST(GTP_RST_PORT, "GTP_RST_PORT");
    if (ret < 0)
    {
        VIVO_TS_LOG_ERR("Failed to request GPIO:%d, ERRNO:%d",(s32)GTP_RST_PORT,ret);
        ret = -ENODEV;
    }

    GTP_GPIO_AS_INPUT(GTP_RST_PORT);

    gtp_reset_guitar(ts->client, 20);

    if(ret < 0)
    {
        GTP_GPIO_FREE(GTP_RST_PORT);
        GTP_GPIO_FREE(GTP_INT_PORT);
    }

    return ret;
}
#endif
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
    VIVO_TS_LOG_DBG("INT trigger type:%x", ts->int_trigger_type);

    ret  = request_irq(ts->client->irq, 
                       goodix_ts_irq_handler,
			//irq_table[ts->int_trigger_type],
			irq_table[ts->int_trigger_type] | IRQF_ONESHOT,
                       ts->client->name,
                       ts);
    if (ret)
    {
        VIVO_TS_LOG_ERR("Request IRQ failed!ERRNO:%d.", ret);
	gpio_direction_input(ts->pdata->irq_gpio);
	gpio_free(ts->pdata->irq_gpio);
        ts->use_irq = 0;
        hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        ts->timer.function = goodix_ts_timer_handler;
        hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
        return -1;
    }
    else
    {
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
#if defined(CONFIG_FB)
static void fb_notifier_suspend_work(struct work_struct *work);
static void fb_notifier_resume_work(struct work_struct *work);
#endif

static s8 gtp_request_input_dev(struct goodix_ts_data *ts)
{
    s8 ret = -1;
    s8 phys[32];
#if GTP_HAVE_TOUCH_KEY
    u8 index = 0;
#endif

    GTP_DEBUG_FUNC();

    ts->input_dev = input_allocate_device();
    if (ts->input_dev == NULL)
    {
        VIVO_TS_LOG_ERR("Failed to allocate input device.");
        return -ENOMEM;
    }

    ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
	
#if GTP_ICS_SLOT_REPORT
//    input_mt_init_slots(ts->input_dev, 16);     // in case of "out of memory"
	input_mt_init_slots(ts->input_dev, 16, 0);
#else
	input_mt_init_slots(ts->input_dev, 10, 0);
    	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#endif
    	__set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

#if GTP_HAVE_TOUCH_KEY
    for (index = 0; index < GTP_MAX_KEY_NUM; index++)
    {
        input_set_capability(ts->input_dev, EV_KEY, touch_key_array[index]);
    }
#endif

#if GTP_GESTURE_WAKEUP
    	input_set_capability(ts->input_dev, EV_KEY, KEY_POWER);
#endif

if(vivo_touchscreen_is_support(FS_LARGE_OBJ_SUPPRESSION)) {
	input_set_capability(ts->input_dev, EV_KEY, KEY_TS_LARGE_SUPPRESSION);
}

if(vivo_touchscreen_is_support(FS_DCLICK_WAKE)){
	input_set_capability(ts->input_dev, EV_KEY, KEY_WAKEUP);
	input_set_capability(ts->input_dev, EV_KEY, KEY_WAKEUP_SWIPE);
	input_set_capability(ts->input_dev, EV_KEY, KEY_LEFT);
	input_set_capability(ts->input_dev, EV_KEY, KEY_RIGHT);
	input_set_capability(ts->input_dev, EV_KEY, KEY_UP);
	input_set_capability(ts->input_dev, EV_KEY, KEY_O);
	input_set_capability(ts->input_dev, EV_KEY, KEY_W);
	input_set_capability(ts->input_dev, EV_KEY, KEY_E);
	input_set_capability(ts->input_dev, EV_KEY, KEY_M);
	input_set_capability(ts->input_dev, EV_KEY, KEY_C);
	input_set_capability(ts->input_dev, EV_KEY, KEY_F);
	input_set_capability(ts->input_dev, EV_KEY, KEY_A);
	input_set_capability(ts->input_dev, EV_KEY, KEY_GESTURE);
}

#if GTP_CHANGE_X2Y
    GTP_SWAP(ts->abs_x_max, ts->abs_y_max);
#endif

    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->pdata->x_max, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->pdata->y_max, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);

    sprintf(phys, "input/ts");
    ts->input_dev->name = goodix_ts_name;
    ts->input_dev->phys = phys;
    ts->input_dev->id.bustype = BUS_I2C;
    ts->input_dev->id.vendor = 0xDEAD;
    ts->input_dev->id.product = 0xBEEF;
    ts->input_dev->id.version = 10427;
 
    ret = input_register_device(ts->input_dev);
    if (ret)
    {
        VIVO_TS_LOG_ERR("Register %s input device failed", ts->input_dev->name);
        return -ENODEV;
    }

#if defined(CONFIG_FB)
	INIT_WORK(&ts->fb_notifier_suspend_work,fb_notifier_suspend_work);
	INIT_WORK(&ts->fb_notifier_resume_work, fb_notifier_resume_work);
	ts->fb_notifier_workqueue= create_singlethread_workqueue("gt970_fb_notifier_wq");
	if (!ts->fb_notifier_workqueue) {
		VIVO_TS_LOG_ERR("%s: can't create fb notiifier worqueue\n", __func__);
		return -ENOMEM;
	}

	ts->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&ts->fb_notif);
	if (ret)
		VIVO_TS_LOG_ERR("Unable to register fb_notifier: %d\n",
			ret);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
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

static int goodix_power_init(struct goodix_ts_data *ts)
{
	int ret;

#if 0
	ts->avdd = regulator_get(&ts->client->dev, "avdd");
	if (IS_ERR(ts->avdd)) {
		ret = PTR_ERR(ts->avdd);
		dev_info(&ts->client->dev,
			"Regulator get failed avdd ret=%d\n", ret);
	}
#endif
	if(0 != strcmp(global_vdd_mode,"gpio_mode")){
	ts->vdd = regulator_get(&ts->client->dev, "vdd");
	if (IS_ERR(ts->vdd)) {
		ret = PTR_ERR(ts->vdd);
		dev_info(&ts->client->dev,
			"Regulator get failed vdd ret=%d\n", ret);
	}

	ts->vcc_i2c = regulator_get(&ts->client->dev, "vcc_i2c");
	if (IS_ERR(ts->vcc_i2c)) {
		ret = PTR_ERR(ts->vcc_i2c);
		dev_info(&ts->client->dev,
			"Regulator get failed vcc_i2c ret=%d\n", ret);
	}
	}
	return 0;
}

static int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
	return (regulator_count_voltages(reg) > 0) ?
		regulator_set_optimum_mode(reg, load_uA) : 0;
}

static int goodix_power_on(struct goodix_ts_data *ts)
{
	int ret;
	struct goodix_ts_platform_data *pdata = ts->pdata;

	if (ts->power_on) {
		dev_info(&ts->client->dev,
				"Device already power on\n");
		return 0;
	}
#if 0
	if (!IS_ERR(ts->avdd)) {
		ret = reg_set_optimum_mode_check(ts->avdd,
			GOODIX_VDD_LOAD_MAX_UA);
		if (ret < 0) {
			VIVO_TS_LOG_ERR("Regulator avdd set_opt failed rc=%d\n", ret);
			goto err_set_opt_avdd;
		}
		ret = regulator_enable(ts->avdd);
		if (ret) {
			VIVO_TS_LOG_ERR("Regulator avdd enable failed ret=%d\n", ret);
			goto err_enable_avdd;
		}
	}
#endif
	if(0 == strcmp(global_vdd_mode,"gpio_mode"))
	{
	//zhj add for vdd gpio
		if (gpio_is_valid(pdata->vdd_gpio)) {
			ret = gpio_direction_output(pdata->vdd_gpio, 1);
			if (ret) {
				VIVO_TS_LOG_ERR("Unable to set direction out to 1 for vdd gpio [%d]\n",pdata->vdd_gpio);
				return ret;
			}
		} else {
			VIVO_TS_LOG_ERR("Invalid irq gpio [%d]!\n",pdata->vdd_gpio);
			ret = -EINVAL;
			goto err_free_vdd_gpio;
		}

	//zhj add end

	}else{
			if (!IS_ERR(ts->vdd)) {
				ret = regulator_set_voltage(ts->vdd, GOODIX_VTG_MIN_UV,
							   GOODIX_VTG_MAX_UV);
				if (ret) {
					VIVO_TS_LOG_ERR("Regulator set_vtg failed vdd ret=%d\n", ret);
					goto err_set_vtg_vdd;
				}
				ret = reg_set_optimum_mode_check(ts->vdd,
					GOODIX_VDD_LOAD_MAX_UA);
				if (ret < 0) {
					VIVO_TS_LOG_ERR("Regulator vdd set_opt failed rc=%d\n", ret);
					goto err_set_opt_vdd;
				}
				ret = regulator_enable(ts->vdd);
				if (ret) {
					VIVO_TS_LOG_ERR("Regulator vdd enable failed ret=%d\n", ret);
					goto err_enable_vdd;
				}
			}

			if (!IS_ERR(ts->vcc_i2c)) {
				ret = regulator_set_voltage(ts->vcc_i2c, GOODIX_I2C_VTG_MIN_UV,
							   GOODIX_I2C_VTG_MAX_UV);
				if (ret) {
					VIVO_TS_LOG_ERR("Regulator set_vtg failed vcc_i2c ret=%d\n", ret);
					goto err_set_vtg_vcc_i2c;
				}
				ret = reg_set_optimum_mode_check(ts->vcc_i2c,
					GOODIX_VIO_LOAD_MAX_UA);
				if (ret < 0) {
					VIVO_TS_LOG_ERR("Regulator vcc_i2c set_opt failed rc=%d\n",	ret);
					goto err_set_opt_vcc_i2c;
				}
				ret = regulator_enable(ts->vcc_i2c);
				if (ret) {
					VIVO_TS_LOG_ERR("Regulator vcc_i2c enable failed ret=%d\n",	ret);
					regulator_disable(ts->vdd);
					goto err_enable_vcc_i2c;
					}
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
err_free_vdd_gpio:
	if (gpio_is_valid(pdata->vdd_gpio)) {
		gpio_free(pdata->vdd_gpio);
	}

#if 0
	if (!IS_ERR(ts->avdd))
		regulator_disable(ts->avdd);
err_enable_avdd:
err_set_opt_avdd:
#endif
	ts->power_on = false;
	return ret;
}

/**
 * goodix_power_off - Turn device power OFF
 * @ts: driver private data
 *
 * Returns zero on success, else an error.
 */
#if 1
static int goodix_power_off(struct goodix_ts_data *ts)
{
	int ret;
	struct goodix_ts_platform_data *pdata = ts->pdata;

	if (!ts->power_on) {
		dev_info(&ts->client->dev,
				"Device already power off\n");
		return 0;
	}
	if(0 == strcmp(global_vdd_mode,"gpio_mode"))
	{		
		//zhj add for vdd gpio
		if (gpio_is_valid(pdata->vdd_gpio)) {
			ret = gpio_direction_output(pdata->vdd_gpio, 0);
			if (ret) {
				VIVO_TS_LOG_ERR("Unable to set direction for vdd gpio [%d]\n",pdata->vdd_gpio);
				goto err_free_vdd_gpio;
			}
		} else {
			VIVO_TS_LOG_ERR("Invalid irq gpio [%d]!\n",pdata->vdd_gpio);
			ret = -EINVAL;
			goto err_free_vdd_gpio;
		}

		//zhj add end
	}else{
		if (!IS_ERR(ts->vcc_i2c)) {
			ret = regulator_set_voltage(ts->vcc_i2c, 0,
				GOODIX_I2C_VTG_MAX_UV);
			if (ret < 0)
				VIVO_TS_LOG_ERR("Regulator vcc_i2c set_vtg failed ret=%d\n", ret);
			ret = regulator_disable(ts->vcc_i2c);
			if (ret)
				VIVO_TS_LOG_ERR("Regulator vcc_i2c disable failed ret=%d\n", ret);
		}

		if (!IS_ERR(ts->vdd)) {
			ret = regulator_set_voltage(ts->vdd, 0, GOODIX_VTG_MAX_UV);
			if (ret < 0)
				VIVO_TS_LOG_ERR("Regulator vdd set_vtg failed ret=%d\n", ret);
			ret = regulator_disable(ts->vdd);
			if (ret)
				VIVO_TS_LOG_ERR("Regulator vdd disable failed ret=%d\n", ret);
		}

	}
#if 0
	if (!IS_ERR(ts->avdd)) {
		ret = regulator_disable(ts->avdd);
		if (ret)
			VIVO_TS_LOG_ERR("Regulator avdd disable failed ret=%d\n", ret);
	}
#endif
err_free_vdd_gpio:
	if (gpio_is_valid(pdata->vdd_gpio)) {
		gpio_free(pdata->vdd_gpio);
	}

	ts->power_on = false;
	return 0;
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
		VIVO_TS_LOG_ERR("Unable to read %s\n", name);
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
		VIVO_TS_LOG_ERR("unsupported property %s\n", name);
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
	const char *str_val = NULL;

	rc = goodix_ts_get_dt_coords(dev, "goodix,panel-coords", pdata);
	if (rc && (rc != -EINVAL))
		return rc;

	rc = goodix_ts_get_dt_coords(dev, "goodix,display-coords", pdata);
	if (rc)
		return rc;

	pdata->i2c_pull_up = of_property_read_bool(np,
						"goodix,i2c-pull-up"); 

	pdata->force_update = of_property_read_bool(np,
						"goodix,force-update");

	pdata->enable_power_off = of_property_read_bool(np,
						"goodix,enable-power-off");

	pdata->have_touch_key = of_property_read_bool(np,
						"goodix,have-touch-key");

	pdata->driver_send_cfg = of_property_read_bool(np,
						"goodix,driver-send-cfg");

	pdata->change_x2y = of_property_read_bool(np,
						"goodix,change-x2y");

	pdata->with_pen = of_property_read_bool(np,
						"goodix,with-pen");

	pdata->slide_wakeup = of_property_read_bool(np,
						"goodix,slide-wakeup");

	pdata->dbl_clk_wakeup = of_property_read_bool(np,
						"goodix,dbl_clk_wakeup");

	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "reset-gpios",
				0, &pdata->reset_gpio_flags);
	if (pdata->reset_gpio < 0)
		return pdata->reset_gpio;

	pdata->irq_gpio = of_get_named_gpio_flags(np, "interrupt-gpios",
				0, &pdata->irq_gpio_flags);
	if (pdata->irq_gpio < 0)
		return pdata->irq_gpio;
//zhj add for vdd gpio
	if(0 == strcmp(global_vdd_mode,"gpio_mode")){
		pdata->vdd_gpio = of_get_named_gpio_flags(np, "goodix,vdd-gpio",
					0, &pdata->vdd_gpio_flags);
		if (pdata->vdd_gpio < 0)
			return pdata->vdd_gpio;
	}
//zhj add end
	VIVO_TS_LOG_INF(" GTP : int = %d,reset = %d, vdd = %d\n",pdata->irq_gpio,pdata->reset_gpio, pdata->vdd_gpio);
	
	rc = of_property_read_string(np, "goodix,product-id",
						&pdata->product_id);
	if (rc && (rc != -EINVAL)) {
		VIVO_TS_LOG_ERR("Failed to parse product_id.");
		return -EINVAL;
	}

	rc = of_property_read_string(np, "goodix,fw_name",
						&pdata->fw_name);
	if (rc && (rc != -EINVAL)) {
		VIVO_TS_LOG_ERR("Failed to parse firmware name.\n");
		return -EINVAL;
	}

	prop = of_find_property(np, "goodix,button-map", NULL);
	if (prop) {
		num_buttons = prop->length / sizeof(temp_val);
		if (num_buttons > MAX_BUTTONS)
			return -EINVAL;

		rc = of_property_read_u32_array(np,
			"goodix,button-map", button_map,
			num_buttons);
		if (rc) {
			VIVO_TS_LOG_ERR("Unable to read key codes\n");
			return rc;
		}
		pdata->num_button = num_buttons;
		memcpy(pdata->button_map, button_map,
			pdata->num_button * sizeof(u32));
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
				GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH,
				GFP_KERNEL);
		if (!pdata->config_data[i]) {
			VIVO_TS_LOG_ERR("Not enough memory for panel config data %d\n",
				i);
			return -ENOMEM;
		}
		pdata->config_data[i][0] = GTP_REG_CONFIG_DATA >> 8;
		pdata->config_data[i][1] = GTP_REG_CONFIG_DATA & 0xff;
		memcpy(&pdata->config_data[i][GTP_ADDR_LENGTH],
				prop->value, pdata->config_data_len[i]);
		read_cfg_num++;
	}
	VIVO_TS_LOG_DBG("%d config data read from device tree.\n", read_cfg_num);

	//chenyunzhe add beg------------------------
	rc = of_property_read_u32(np, "ts-suspend-resume", &temp_val);
	if (rc && (rc != -EINVAL))
		VIVO_TS_LOG_ERR("Unable to read ts-suspend-resume value\n");
	else if (rc != -EINVAL)
		pdata->suspend_resume_methods = temp_val;
		
	rc = of_property_read_u32(np, "ts-fixed-key-type", &temp_val);
	if (rc && (rc != -EINVAL))
		VIVO_TS_LOG_ERR("Unable to read ts-fixed-key-type value\n");
	else if (rc != -EINVAL)
		pdata->fixed_key_type = temp_val;

	rc = of_property_read_string(np,"ts-virt-key",&str_val);
	if (rc) {
		VIVO_TS_LOG_ERR("Unable to read ts-dclick-dimension-y-max value\n");
	} else {
	    pdata->virtual_key_string =  str_val;
	}

	gtp_set_virtual_key_string(pdata->virtual_key_string);
		
	VIVO_TS_LOG_ERR("%s:srm(%d) fkt(%d)\n",__func__,pdata->suspend_resume_methods,pdata->fixed_key_type);	
	VIVO_TS_LOG_ERR("%s:virt-key: %s\n",__func__,pdata->virtual_key_string);
	//chenyunzhe add end------------------------

	return 0;
}


static int gtp_pinctrl_init(struct device *dev)
{
	gt9xx_pctrl.pinctrl = devm_pinctrl_get(dev);

	if (IS_ERR_OR_NULL(gt9xx_pctrl.pinctrl)) {
		VIVO_TS_LOG_ERR("----------Getting pinctrl handle failed---------\n");
		return -EINVAL;
	}
	gt9xx_pctrl.gpio_state_active = pinctrl_lookup_state(
					       gt9xx_pctrl.pinctrl,
					       GOODIX_PINCTRL_STATE_DEFAULT);

	if (IS_ERR_OR_NULL(gt9xx_pctrl.gpio_state_active)) {
		VIVO_TS_LOG_ERR("Failed to get the active state pinctrl handle\n");
		return -EINVAL;
	}
	gt9xx_pctrl.gpio_state_suspend = pinctrl_lookup_state(
						gt9xx_pctrl.pinctrl,
						GOODIX_PINCTRL_STATE_SLEEP);

	if (IS_ERR_OR_NULL(gt9xx_pctrl.gpio_state_suspend)) {
		VIVO_TS_LOG_ERR("Failed to get the suspend state pinctrl handle\n");
		return -EINVAL;
	}
	return 0;
}
/*[BUGFIX]PR-667466. TP INT pull-up enable.*/

static void gtp_log_switch_set(bool on)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
	ts->log_switch = on;
}

struct bbk_drivers_callback_handler gtp_log_switch_handler = {
	.name = "gtp_driver",
	.callback = gtp_log_switch_set,
};

/*liuyunfeng Add for sys file*/
struct kobject gtp_state_kobj;
static ssize_t gtp_log_switch_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
	int val;
	
	if (sscanf(buf, "%d", &val) != 1) {
		VIVO_TS_LOG_ERR("Invalide number of parameters\n");
		return -EINVAL;
	}

	if (val == 0 || val == 1) {
		ts->log_switch = val;
	}else{
		VIVO_TS_LOG_ERR("Invalide parameter\n");
		return -EINVAL;
	}

	return count;
}

static ssize_t gtp_log_switch_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	 struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
	 
	 return sprintf(buf, "log_switch:%d\n",ts->log_switch);
}

static ssize_t gtp_firmware_version_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	int ret = 0;
	int fw_version = 0;

	u8 buf2[GTP_FW_VERSION_BUFFER_MAXSIZE] = {
		GTP_REG_FW_VERSION >> 8, GTP_REG_FW_VERSION & 0xff };

	ret = gtp_i2c_read(i2c_connect_client, buf2, sizeof(buf2));
	if (ret < 0) {
		VIVO_TS_LOG_ERR("GTP read version failed.\n");
		return -EIO;
	}
	fw_version = ((buf2[3] << 8) | buf2[2]);

	VIVO_TS_LOG_INF("firmware_version is %d", fw_version);
	return sprintf(buf, "0x%02x\n", fw_version);
}

static ssize_t gtp_fw_config_version_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	int ret = 0;
	int fw_config_verison = 0;
	int fw_version = 0;

	u8 buf2[3] = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff };

	/*read fw_version*/
	u8 buf3[GTP_FW_VERSION_BUFFER_MAXSIZE] = {
		GTP_REG_FW_VERSION >> 8, GTP_REG_FW_VERSION & 0xff };

	ret = gtp_i2c_read(i2c_connect_client, buf3, sizeof(buf3));
	if (ret < 0) {
		VIVO_TS_LOG_ERR("GTP read version failed.\n");
		return -EIO;
	}
	fw_version = ((buf3[3] << 8) | buf3[2]);
	VIVO_TS_LOG_INF("firmware_version is %d", fw_version);

	/*read fw_config_version end*/
	ret = gtp_i2c_read(i2c_connect_client, buf2, sizeof(buf2));
	if (ret < 0) {
		VIVO_TS_LOG_ERR("GTP read version failed.\n");
		return -EIO;
	}
	fw_config_verison =  buf2[2];

	VIVO_TS_LOG_INF("fw_config_verison is %d", fw_config_verison);

	//return sprintf(buf, "0x%02x\n", fw_config_verison);
	return sprintf(buf, "0x%02x0x%02x\n",fw_version,fw_config_verison);
}

static ssize_t gtp_module_id_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	int ret = 0;
	u8 sensor_id = -1; 
	int module_id = -1;
	const char *product_name = NULL;
	int rc = -1;

	ret = gtp_i2c_read_dbl_check(i2c_connect_client, GTP_REG_SENSOR_ID, &sensor_id, 1);
	if (SUCCESS == ret)
	{
            	if (sensor_id >= 0x06)
            	{
                	VIVO_TS_LOG_ERR("Invalid sensor_id(0x%02X), No Config Sent!", sensor_id);
                	return -1;
            	}
	}else
       {
            	VIVO_TS_LOG_ERR("Failed to get sensor_id, No config sent!");
            	return -1;
	}
	VIVO_TS_LOG_INF("Sensor_ID: %d", sensor_id);

 	vivo_touchscreen_get_product_name(&product_name);
	if(NULL == product_name)
	{		
	    VIVO_TS_LOG_ERR("no product name!! pls check device tree\n");
	    return rc;
	}
	
	VIVO_TS_LOG_INF("product name=%s\n", product_name);
	
	if(!strcmp(product_name,"PD1503V")){
		if(sensor_id == 0)
			module_id = 0x3b;
		else if (sensor_id == 2)
			module_id = 0x70;
		else
			module_id = -1;
	}else if(!strcmp(product_name,"PD1419C")){
		if(sensor_id == 3)		//1419 bie's sensor id changed to 3 form 0
			module_id = 0x3B;	//bie
		else if (sensor_id == 2)
			module_id = 0x10;	//lans
		else
			module_id = -1;
	}else if(!strcmp(product_name,"PD1510")){
		if(sensor_id == 0)
			module_id = 0x3B;	//bie
		else if (sensor_id == 2)
			module_id = 0x70;	//truly
		else if (sensor_id == 3)
			module_id = 0x80;	//yeji
		else
			module_id = -1;
	}else if(!strcmp(product_name,"PD1513")){
		if(sensor_id == 0)		
			module_id = 0x70;	//Truly
		else if (sensor_id == 3)
			module_id = 0x3b;	//lans
		else
			module_id = -1;
	}else if(!strcmp(product_name,"PD1505")){
		if(sensor_id == 0)		
			module_id = 0x40; //jiemian
		else if (sensor_id == 2)
			module_id = 0x80; //yeji
		else if (sensor_id == 3)
			module_id = 0x10; //lans
		else
			module_id = -1;
	}else if(!strcmp(product_name,"PD1523A")){
		if(sensor_id == 0)		
			module_id = 0x3B; //boen
		else if (sensor_id == 2)
			module_id = 0x80; //yeji
		else if (sensor_id == 3)
			module_id = 0x10; //lans
		else
			module_id = -1;
	}else if(!strcmp(product_name,"PD1524")){
		if(sensor_id == 0)		
			module_id = 0x3B; //boen
		else if (sensor_id == 2)
			module_id = 0x80; //yeji
		else if (sensor_id == 3)
			module_id = 0x10; //lans
		else
			module_id = -1;
	}else
	{
		if(sensor_id == 0)
			module_id = 0x70;
		else if (sensor_id == 3)
			module_id = 0x80;
		else
			module_id = -1;
	}

	return sprintf(buf, "%02x\n", module_id);
}

static ssize_t gtp_null_show(struct kobject *kobj,
							 struct kobj_attribute *attr, char *buf)
{
	 return -EINVAL;
}


static ssize_t gtp_null_store(struct kobject *kobj,
							 struct kobj_attribute *attr,  const char *buf, size_t count)
{
	 return -EINVAL;
}

static ssize_t gtp_rawdata_show(struct kobject *kobj,
									struct kobj_attribute *attr, char *buf)
{
    u32 index = 0, i = 0;
    u32 len = 0;
    u8  ctrl_buf[3] = {0x80, 0x40, 0x01};
    s32 ret = 0;
    u8 *data = NULL;
    u8 *data0 = NULL;
    u16 left=0,len1=0,j=0;
    u16 addr = GTP_REG_RAWDATA;
    struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
    VIVO_TS_LOG_DBG("gtp_rawdata_show invoked!driver_num:%d, sensor_num:%d", driver_num, sensor_num);
    if(driver_num*sensor_num == 0)
    {
        VIVO_TS_LOG_DBG("gtp_rawdata_show Invalid data length to show!");
        return 0;
    }
    data = (u8 *)kzalloc(driver_num*sensor_num*2 + 10, GFP_KERNEL);
    if(data == NULL)
    {
        VIVO_TS_LOG_DBG("gtp_rawdata_show cannot alloc memory!");
        return 0;
    } 
    data0 = (u8 *)kzalloc(driver_num*sensor_num*2 + 10, GFP_KERNEL);
    if(data0 == NULL)
    {
        VIVO_TS_LOG_DBG("gtp_rawdata_show cannot alloc memory!");
        return 0;
    }
    ts->gtp_rawdiff_mode = 1;
    ret = gtp_i2c_write(i2c_connect_client, ctrl_buf, sizeof(ctrl_buf));
    if(ret < 0)
    {
        VIVO_TS_LOG_ERR("gtp_rawdata_show enter rawdata mode failed.");
         kfree(data);
	 kfree(data0);
	ts->gtp_rawdiff_mode = 0;
        return 0;
    }
    ctrl_buf[0] = 0x81;
    ctrl_buf[1] = 0x4E;
    ctrl_buf[2] = 0x00;
    VIVO_TS_LOG_DBG("gtp_rawdata_show begin wait data ready flag.");
    do {
        gtp_i2c_read(i2c_connect_client, ctrl_buf, sizeof(ctrl_buf));
        msleep(20);
    }while((ctrl_buf[2] & 0x80) != 0x80);
    VIVO_TS_LOG_DBG("gtp_rawdata_show rawdata ready flag is set.");
//    data[0] = 0x8B; //0xBB;    //0x8B;
//    data[1] = 0x98; //0x10;    //0x98;
//    ret = gtp_i2c_read(i2c_connect_client, data, driver_num*sensor_num*2 + 2);
//    if(ret < 0)
//    {
//        VIVO_TS_LOG_DBG("gtp_rawdata_show read rawdata failed.");
//    }
		 left = driver_num*sensor_num*2;
	  data[0] = (u8)(addr >> 8);
    data[1] = (u8)(addr); 
    while (left > 0)
    {
        data0[0] = (u8)(addr >> 8);
        data0[1] = (u8)(addr);  
        if (left > 254)
        {
            len1 = 254;
        }
        else
        {
            len1 = left;
        }   
        ret = gtp_i2c_read(i2c_connect_client, data0, len1+2);
        memcpy(&data[2+j*254],&data0[2],len1);
    if(ret < 0)
    {
             kfree(data);
	 kfree(data0);
	ts->gtp_rawdiff_mode = 0;
            return FAIL;
        }
        addr += len1;
        left -= len1;
         j++;
    }
    ctrl_buf[0] = 0x80;
    ctrl_buf[1] = 0x40;
    ctrl_buf[2] = 0x00;
    ret = gtp_i2c_write(i2c_connect_client, ctrl_buf, sizeof(ctrl_buf));
    if(ret < 0)
    {
        VIVO_TS_LOG_ERR("gtp_rawdata_show exit rawdata mode failed.");
         kfree(data);
	 kfree(data0);
	ts->gtp_rawdiff_mode = 0;
        return 0;
    }
    ts->gtp_rawdiff_mode = 0;
    ctrl_buf[0] = 0x81;
    ctrl_buf[1] = 0x4E;
    ctrl_buf[2] = 0x00;
    gtp_i2c_write(i2c_connect_client, ctrl_buf, sizeof(ctrl_buf));
    for(index = 0, i = 0; index < driver_num*sensor_num; index++, i += 2)
    {
        if(index && (index % (sensor_num) == 0))
        {
            len += sprintf(&buf[len], "\n");
        }
        len += sprintf(&buf[len], "%04d ", (s16)((((s16)data[2 + i]) << 8) + data[2 + i + 1]));
    }
    VIVO_TS_LOG_DBG("gtp_rawdata_show return len:%d", len);    
    kfree(data);
    kfree(data0);
    return len;
}

static ssize_t gtp_baseline_show(struct kobject *kobj,
									struct kobj_attribute *attr, char *buf)
{
    u32 index = 0, i = 0;
    u32 len = 0;
    u8  ctrl_buf[3] = {0x80, 0x40, 0x01};
    s32 ret = 0;
    u8 *data0 = NULL;
    u8 *data = NULL;
    u16 left=0,len1=0,j=0;
    u16 addr = GTP_REG_BASELINE;
    struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);

    VIVO_TS_LOG_DBG("gtp_baseline_show invoked!driver_num:%d, sensor_num:%d", driver_num, sensor_num);
    if(driver_num*sensor_num == 0)
    {
        VIVO_TS_LOG_DBG("gtp_baseline_show Invalid data length to show!");
        return 0;
    }
    data = (u8 *)kzalloc(driver_num*sensor_num*2 + 10, GFP_KERNEL);
    if(data == NULL)
    {
        VIVO_TS_LOG_DBG("gtp_baseline_show cannot alloc memory!");
        return 0;
    }
     data0 = (u8 *)kzalloc(driver_num*sensor_num*2 + 10, GFP_KERNEL);
    if(data0 == NULL)
    {
        VIVO_TS_LOG_DBG("gtp_baseline_show cannot alloc memory!");
        return 0;
    }
    ts->gtp_rawdiff_mode = 1;
    ret = gtp_i2c_write(i2c_connect_client, ctrl_buf, sizeof(ctrl_buf));
    if(ret < 0)
    {
        VIVO_TS_LOG_ERR("gtp_baseline_show enter rawdata mode failed.");
        kfree(data);
	 kfree(data0);
	ts->gtp_rawdiff_mode = 0;
        return 0;
    }
    ctrl_buf[0] = 0x81;
    ctrl_buf[1] = 0x4E;
    ctrl_buf[2] = 0x00;
    VIVO_TS_LOG_DBG("gtp_baseline_show begin wait data ready flag.");
    do {
        gtp_i2c_read(i2c_connect_client, ctrl_buf, sizeof(ctrl_buf));
        msleep(20);
    }while((ctrl_buf[2] & 0x80) != 0x80);
    VIVO_TS_LOG_DBG("gtp_baseline_show rawdata ready flag is set.");
//    data[1] = 0xC0; //0x10;    //0x98;
//    ret = gtp_i2c_read(i2c_connect_client, data, driver_num*sensor_num*2 + 2);
//    if(ret < 0)
//    {
//        VIVO_TS_LOG_DBG("gtp_baseline_show read rawdata failed.");
//    }
    left = driver_num*sensor_num*2;
    data[0] = (u8)(addr >> 8);
    data[1] = (u8)(addr); 
    while (left > 0)
    {
        data0[0] = (u8)(addr >> 8);
        data0[1] = (u8)(addr);  
        if (left > 254)
        {
            len1 = 254;
        }
        else
        {
            len1 = left;
        }   
        ret = gtp_i2c_read(i2c_connect_client, data0, len1+2);
        memcpy(&data[2+j*254],&data0[2],len1);
    if(ret < 0)
    {
             kfree(data);
	     kfree(data0);
	    ts->gtp_rawdiff_mode = 0;
            return FAIL;
        }
        addr += len1;
        left -= len1;
        j++;
    }
    ctrl_buf[0] = 0x80;
    ctrl_buf[1] = 0x40;
    ctrl_buf[2] = 0x00;
    ret = gtp_i2c_write(i2c_connect_client, ctrl_buf, sizeof(ctrl_buf));
    if(ret < 0)
    {
        VIVO_TS_LOG_ERR("gtp_baseline_show exit rawdata mode failed.");
        kfree(data);
	 kfree(data0);
	ts->gtp_rawdiff_mode = 0;
        return 0;
    }
    ts->gtp_rawdiff_mode = 0;
    ctrl_buf[0] = 0x81;
    ctrl_buf[1] = 0x4E;
    ctrl_buf[2] = 0x00;
    gtp_i2c_write(i2c_connect_client, ctrl_buf, sizeof(ctrl_buf));
    for(index = 0, i = 0; index < driver_num*sensor_num; index++, i += 2)
    {
        if(index && (index % (sensor_num) == 0))
        {
            len += sprintf(&buf[len], "\n");
        }
        len += sprintf(&buf[len], "%04d ", (s16)((((s16)data[2 + i]) << 8) + data[2 + i + 1]));
    }
    VIVO_TS_LOG_DBG("gtp_baseline_show return len:%d", len);    
    kfree(data);
    kfree(data0);
    return len;
}

static ssize_t gtp_delta_show(struct kobject *kobj,
							   struct kobj_attribute *attr, char *buf)
{
    u32 index = 0, i = 0;
    u32 len = 0;
    u8  ctrl_buf[3] = {0x80, 0x40, 0x01};
    s32 ret = 0;
    u8 *data = NULL;
     u8 *data0 = NULL;
     u16 left=0,len1=0,j=0;
    u16 addr = GTP_REG_DELTA;
    struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
	
    VIVO_TS_LOG_DBG("gtp_delta_show invoked!driver_num:%d, sensor_num:%d", driver_num, sensor_num);
    if(driver_num*sensor_num == 0)
    {
        VIVO_TS_LOG_DBG("gtp_delta_show Invalid data length to show!");
        return 0;
    }
    data = (u8 *)kzalloc(driver_num*sensor_num*2 + 10, GFP_KERNEL);
    if(data == NULL)
    {
        VIVO_TS_LOG_DBG("gtp_delta_show cannot alloc memory!");
        return 0;
    }
     data0 = (u8 *)kzalloc(driver_num*sensor_num*2 + 10, GFP_KERNEL);
    if(data0 == NULL)
    {
        VIVO_TS_LOG_DBG("gtp_delta_show cannot alloc memory!");
        return 0;
    }
    ts->gtp_rawdiff_mode = 1;
    ret = gtp_i2c_write(i2c_connect_client, ctrl_buf, sizeof(ctrl_buf));
    if(ret < 0)
    {
        VIVO_TS_LOG_ERR("gtp_delta_show enter rawdata mode failed.");
	ts->gtp_rawdiff_mode = 0;
        kfree(data);
	kfree(data0);
        return 0;
    }
    ctrl_buf[0] = 0x81;
    ctrl_buf[1] = 0x4E;
    ctrl_buf[2] = 0x00;
    VIVO_TS_LOG_DBG("gtp_delta_show begin wait data ready flag.");
    do {
        gtp_i2c_read(i2c_connect_client, ctrl_buf, sizeof(ctrl_buf));
        msleep(20);
    }while((ctrl_buf[2] & 0x80) != 0x80);
    VIVO_TS_LOG_DBG("gtp_delta_show rawdata ready flag is set.");
//    data[0] = 0xBB;    //0x8B;
//    data[1] = 0x10;    //0x98;
//    ret = gtp_i2c_read(i2c_connect_client, data, driver_num*sensor_num*2 + 2);
//    if(ret < 0)
//    {
//        VIVO_TS_LOG_DBG("gtp_delta_show read rawdata failed.");
//    }
    left = driver_num*sensor_num*2;
     data[0] = (u8)(addr >> 8);
    data[1] = (u8)(addr); 
    while (left > 0)
    {
        data0[0] = (u8)(addr >> 8);
        data0[1] = (u8)(addr);  
        if (left > 254)
        {
            len1 = 254;
        }
        else
        {
            len1 = left;
        }   
        ret = gtp_i2c_read(i2c_connect_client, data0, len1+2);
         memcpy(&data[2+j*254],&data0[2],len1);
    if(ret < 0)
    {
             kfree(data);
	 kfree(data0);
	ts->gtp_rawdiff_mode = 0;
            return FAIL;
        }
        addr += len1;
        left -= len1;
        j++;
    }
    ctrl_buf[0] = 0x80;
    ctrl_buf[1] = 0x40;
    ctrl_buf[2] = 0x00;
    ret = gtp_i2c_write(i2c_connect_client, ctrl_buf, sizeof(ctrl_buf));
    if(ret < 0)
    {
        VIVO_TS_LOG_ERR("gtp_delta_show exit rawdata mode failed.");
	 ts->gtp_rawdiff_mode = 0;
        kfree(data);
	kfree(data0);
        return 0;
    }
    ts->gtp_rawdiff_mode = 0;
    ctrl_buf[0] = 0x81;
    ctrl_buf[1] = 0x4E;
    ctrl_buf[2] = 0x00;
    gtp_i2c_write(i2c_connect_client, ctrl_buf, sizeof(ctrl_buf));
    for(index = 0, i = 0; index < driver_num*sensor_num; index++, i += 2)
    {
        if(index && (index % (sensor_num) == 0))
        {
            len += sprintf(&buf[len], "\n");
        }
        len += sprintf(&buf[len], "%04d ", (s16)((((s16)data[2 + i]) << 8) + data[2 + i + 1]));
    }
    VIVO_TS_LOG_ERR("gtp_delta_show return len:%d", len);    
    kfree(data);
   kfree(data0);
    return len;
}

//Provide sensor rx tx to Engineering model
static ssize_t gtp_sensor_rx_tx_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	int Rx, Tx ,temp;
	s32 ret = 0;
	u8 buf1[4]={0x80,0x62};
	u8 buf2[3]={0x80,0x64};
	
	ret = gtp_i2c_read(i2c_connect_client, buf1, 4); 
	Tx=((buf1[2]&0x1f)+(buf1[3]&0x1f));
	
	ret = gtp_i2c_read(i2c_connect_client, buf2, 3);
	Rx=((buf2[2]&0x0f)+(((buf2[2]>>4)&0x0f)));
	
	temp = Tx<<8|Rx;//GT9xx  rx tx opposite So Trades tx rx
	
	VIVO_TS_LOG_INF("rx=%d tx=%d temp = %d",Rx,Tx,temp);
	return sprintf(buf, "%d",temp);
}

static ssize_t gtp_is_calling_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
	int val;
	
	if (sscanf(buf, "%d", &val) != 1) {
		VIVO_TS_LOG_ERR("Invalide number of parameters passed\n");
		return -EINVAL;
	}

	VIVO_TS_LOG_ERR("parameter is %d\n", val);
	if (val == 0) {
		ts->is_calling = 0;
	}else if (val == 1) {
		ts->is_calling = 1;
	}else{
		VIVO_TS_LOG_ERR("Invalide parameter passed(%d)\n", val);
		return -EINVAL;
	}

	return count;
}

static ssize_t gtp_is_calling_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
	
	return sprintf(buf, "is_calling = %d\n", ts->is_calling);
}


static ssize_t gtp_gesture_point_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
	int i;
	u16 buf_gesture[130] = {0};
   	int len = 0;
	   
	for(i=0;i<buf_coordinate_num[2]*2;i=i+2)
	{
		 buf_gesture[i] = ts->buf_gesture[i];
		 buf_gesture[i+1] = ts->buf_gesture[i+1]*1280/1365;
		 VIVO_TS_LOG_INF("x:buf_gesture[%d] = %d y:buf_gesture[%d]  = %d\n",i,buf_gesture[i],i+1,buf_gesture[i+1]);
		 len += sprintf(&buf[len],"%d ",(s16)(buf_gesture[i]));
		 len += sprintf(&buf[len],"%d ",(s16)(buf_gesture[i+1]));
	}
	len += sprintf(&buf[len],"%d ", 65535);
	
	return len;
}

static ssize_t gtp_ipod_flag_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
	int val;
	if (sscanf(buf, "%d", &val) != 1) {
		//dev_err(&ts->input_dev->dev, "Invalide number of parameters passed\n");
		VIVO_TS_LOG_ERR("Invalide number of parameters passed");
		return -EINVAL;
	}
	//dev_err(&ts->input_dev->dev, "parameter is %d\n", val);
	VIVO_TS_LOG_INF("parameter is %d ", val);
	if (val == 0) {
		ts->ts_ipod_flag = 0;
	}else if (val == 1) {
		ts->ts_ipod_flag = 1;
	}else{
		//dev_err(&ts->input_dev->dev, "Invalide parameter passed(%d)\n", val);
		VIVO_TS_LOG_ERR("Invalide parameters passed(%d)", val);
		return -EINVAL;
	}
	return count;
}
static ssize_t gtp_ipod_flag_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);

	return sprintf(buf, "%d\n", ts->ts_ipod_flag);
}
static ssize_t gtp_dclick_flag_info_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	return -EINVAL;
}
static ssize_t gtp_dclick_flag_info_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	return -EINVAL;
}
static ssize_t gtp_edge_suppress_switch_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);


	return sprintf(buf, "edge_suppress_switch: %d\n", ts->edge_suppress_switch);
}

static ssize_t gtp_edge_suppress_switch_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);

	int edge_suppress_switch;
	int retval;

	if (sscanf(buf, "%d", &edge_suppress_switch) != 1) {
		VIVO_TS_LOG_ERR( "Invalide number of parameters passed\n");
		return -EINVAL;
	}

	VIVO_TS_LOG_INF("%s: edge_suppress_switch is  = %d\n", __func__,edge_suppress_switch);

	//ts->edge_suppress_switch = edge_suppress_switch;
	edge_suppress_switch = 0; //turn off edge_suppress_switch;

	if (edge_suppress_switch == 1) 
	{
		retval = gtp_enter_edge_restrain(ts);
		if (retval < 0) {
			VIVO_TS_LOG_ERR("%s: failed to turn on edge_suppress_switch \n",__func__);
			return retval;
		}
	}
	else if(edge_suppress_switch == 0) 
	{
		retval = gtp_leave_edge_restrain(ts);
		if (retval < 0) {
			VIVO_TS_LOG_ERR("%s: failed to turn off edge_suppress_switch \n", __func__);
			return retval;
		}
	}
	else
	{
		VIVO_TS_LOG_ERR( "Invalide parameter(%d)\n", edge_suppress_switch);
		return -EINVAL;
	}

	return count;
}
static ssize_t gtp_dclick_switch_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
	
	return sprintf(buf, "%d\n", ts->ts_dclick_switch);
}
static ssize_t gtp_dclick_switch_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
	int dclick_switch;
	
	if (sscanf(buf, "%d", &dclick_switch) != 1) {
		//dev_err(&ts->input_dev->dev, "Invalide number of parameters passed\n");
		VIVO_TS_LOG_ERR("Invalide number of parameters passed");
		return -EINVAL;
	}
	if (dclick_switch == 0 || dclick_switch == 1) {
		ts->ts_dclick_switch = dclick_switch;
	}else{
		//dev_err(&ts->input_dev->dev, "Invalide parameter(%d)\n", dclick_switch);
		VIVO_TS_LOG_ERR("parameter is %d ", dclick_switch);
		return -EINVAL;
	}
	return count;
}

/**liuyunfeng add for all gesture expect down swipe and dclick ***
* 0x01: Lift or Right swipe  0x02: up swipe 	0x04: O 
* 0x08: W	0x10: M	0x20: e	0x40: C 
*/
static ssize_t gtp_gesture_switch_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
	
	return sprintf(buf, "0x01:LR 0x02:up 0x04:O 0x08:W 0x10:M 0x20:e 0x40:C  gesture_switch = 0x%x\n",ts->gesture_switch);
}
static ssize_t gtp_gesture_switch_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
	int val;

	if (sscanf(buf, "%d", &val) != 1) {
		VIVO_TS_LOG_ERR("Invalide number of parameters");
		return -EINVAL;
	}

	if (val < 0x80) {
		ts->gesture_switch = val;
	}else{
		VIVO_TS_LOG_ERR("Invalide parameter %u ", val);
		return -EINVAL;
	}
	return count;
}
/**liuyunfeng add for gesture_switch_export ***
* 0x01: all gesture 0x02: @ 0x04: f 
* 0x08 0x10 0x20 0x40 are reserve 
*/
static ssize_t gtp_gesture_switch_export_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
	
	return sprintf(buf, "0x01:all 0x02:@ 0x04:f others are reserve gesture_switch_export = 0x%x\n",ts->gesture_switch_export);
}
static ssize_t gtp_gesture_switch_export_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
	int val;

	if (sscanf(buf, "%d", &val) != 1) {
		VIVO_TS_LOG_ERR("Invalide number of parameters\n");
		return -EINVAL;
	}

	if (val < 0x08) {
		ts->gesture_switch_export = val;
	}else{
		VIVO_TS_LOG_ERR("Invalide parameter %u ",val);
		return -EINVAL;
	}
	return count;
}

static ssize_t gtp_swipe_down_switch_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
	
	return sprintf(buf, "%d\n", ts->swipe_down_switch);
}
static ssize_t gtp_swipe_down_switch_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
	int swipe_down_switch;
	
	if (sscanf(buf, "%d", &swipe_down_switch) != 1) {
		//dev_err(&ts->input_dev->dev, "Invalide number of parameters passed\n");
		VIVO_TS_LOG_ERR("Invalide number of parameters passed");
		return -EINVAL;
	}
	if (swipe_down_switch == 0 || swipe_down_switch == 1) {
		ts->swipe_down_switch = swipe_down_switch;
	}else{
		//dev_err(&ts->input_dev->dev, "Invalide parameter(%d)\n", swipe_down_switch);
		VIVO_TS_LOG_ERR("Invalide parameter is %d ", swipe_down_switch);
		return -EINVAL;
	}
	return count;
}
static ssize_t gtp_dclick_simulate_switch_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
	
	return sprintf(buf, "%d\n", ts->ts_dclick_simulate_switch);
}
static ssize_t gtp_dclick_simulate_switch_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
	int dclick_simulate_switch;
	
	if (sscanf(buf, "%d", &dclick_simulate_switch) != 1) {
		//dev_err(&ts->input_dev->dev, "Invalide number of parameters passed\n");
		VIVO_TS_LOG_ERR("Invalide number of parameters passed\n");
		return -EINVAL;
	}
	if (dclick_simulate_switch == 0 || dclick_simulate_switch == 1) {
		ts->ts_dclick_simulate_switch = dclick_simulate_switch;
	}else{
		//dev_err(&ts->input_dev->dev, "Invalide parameter(%d)\n", dclick_simulate_switch);
		VIVO_TS_LOG_ERR("Invalide parameter is %d ", dclick_simulate_switch);
		return -EINVAL;
	}
	return count;
}

static ssize_t gtp_dclick_proximity_switch_show(struct kobject *kobj,
							 struct kobj_attribute *attr, char *buf)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
	
	VIVO_TS_LOG_INF("dclick_proximity_switch(%d)", ts->dclick_proximity_switch);
	return sprintf(buf, "%d\n", ts->dclick_proximity_switch);
}

static ssize_t gtp_dclick_proximity_switch_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
	int dclick_switch;
	
	if (sscanf(buf, "%d", &dclick_switch) != 1) {
		//dev_err(&ts->input_dev->dev, "Invalide number of parameters passed\n");
		VIVO_TS_LOG_ERR("Invalide number of parameters passed");
		return -EINVAL;
	}
	if (dclick_switch == 0 || dclick_switch == 1) {
		ts->dclick_proximity_switch = dclick_switch;
	}
	VIVO_TS_LOG_INF("dclick_proximity_switch(%d)", dclick_switch);
	mutex_lock(&ts->suspend_mutex);
	if (atomic_read(&ts->ts_state) == TOUCHSCREEN_NORMAL) {
		ts->need_change_to_dclick = dclick_switch;
		//dev_err(&ts->input_dev->dev, "Not first switch set in normal mode\n");
		VIVO_TS_LOG_INF("Not first switch set in normal mode");
		mutex_unlock(&ts->suspend_mutex);
		return count;
	}
	/* normal change state */
	ts_scan_switch(dclick_switch);
	mutex_unlock(&ts->suspend_mutex);
	return count;
}
static ssize_t gtp_dclick_lcd_state_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
	
	return sprintf(buf, "lcd_state is %s\n", ts->has_lcd_shutoff == 1?"off":"on");
}
static ssize_t gtp_dclick_lcd_state_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
	int lcd_state;
	
	if (sscanf(buf, "%d", &lcd_state) != 1)
 	{
		//dev_err(&ts->input_dev->dev,  "Invalide number of parameters passed\n");
		VIVO_TS_LOG_ERR("Invalide number of parameters passed");
		return -EINVAL;
	}
	VIVO_TS_LOG_INF("lcd_state(%d)", lcd_state);
	if(lcd_state == 1)
		ts->need_change_to_dclick = 0;
	if (lcd_state == 1 || lcd_state == 0){
		ts->has_lcd_shutoff = (lcd_state == 0);
	}else{
		//dev_err(&ts->input_dev->dev, "Invalide parameters passed\n");
		VIVO_TS_LOG_ERR("Invalide parameters passed");
		return -EINVAL;
	}
	return count;
}

static ssize_t gtp_touch_ic_name_show(struct kobject *kobj,
					 struct kobj_attribute *attr, char *buf)
{
	/*GT970:GT41,GT9159:GT42,GT1151:GT43*/
	VIVO_TS_LOG_INF( "GTP Touch ic is GT970 \n");
	return sprintf(buf, "%d",41 );
 }
static ssize_t gtp_user_defined_gesture_enable_show(struct kobject *kobj,
					 struct kobj_attribute *attr, char *buf)
{
	//VIVO_TS_LOG_INF( "gesture_enabled is %d \n",gesture_enabled);
	return sprintf(buf, "%d",gesture_enabled );
	//return -EINVAL;
 }
static struct gtp_driver_sysfs_entry gtp_firmware_update =
__ATTR(firmware_update, 0644, gtp_null_show, gtp_null_store);

static struct gtp_driver_sysfs_entry gtp_config_version =
 __ATTR(firmware_version, 0644,  gtp_fw_config_version_show,gtp_null_store);

static struct gtp_driver_sysfs_entry gtp_firmware_version =
 __ATTR(fw_version, 0644, gtp_firmware_version_show, gtp_null_store);

static struct gtp_driver_sysfs_entry gtp_firmware_module_id =
 __ATTR(firmware_module_id, 0644, gtp_module_id_show, gtp_null_store);

static struct gtp_driver_sysfs_entry gtp_sensor_rawdata =
__ATTR(sensor_rawdata, 0644,gtp_rawdata_show, gtp_null_store);

static struct gtp_driver_sysfs_entry gtp_sensor_delta =
__ATTR(sensor_delta, 0644,gtp_delta_show, gtp_null_store);

static struct gtp_driver_sysfs_entry gtp_sensor_baseline =
__ATTR(sensor_baseline, 0644,gtp_baseline_show, gtp_null_store);

static struct gtp_driver_sysfs_entry gtp_ts_log_switch =
__ATTR(log_switch, 0644, gtp_log_switch_show, gtp_log_switch_store);

static struct gtp_driver_sysfs_entry gtp_sensor_rx_tx =
__ATTR(sensor_rx_tx, 0644,gtp_sensor_rx_tx_show, gtp_null_store);

static struct gtp_driver_sysfs_entry gtp_ts_is_calling =
__ATTR(ts_is_calling, 0644,gtp_is_calling_show, gtp_is_calling_store);	


static struct gtp_driver_sysfs_entry gtp_gesture_point =
	__ATTR(gesture_point, 0644, gtp_gesture_point_show, gtp_null_store);
static struct gtp_driver_sysfs_entry gtp_ts_ipod_flag =
	__ATTR(ts_ipod_flag, 0644, 
			gtp_ipod_flag_show, gtp_ipod_flag_store);
static struct gtp_driver_sysfs_entry gtp_dclick_flag_info =
	__ATTR(dclick_flag_info, 0644, 
			gtp_dclick_flag_info_show, gtp_dclick_flag_info_store);
static struct gtp_driver_sysfs_entry gtp_edge_suppress_switch =
	__ATTR(edge_suppress_switch, 0644,
			gtp_edge_suppress_switch_show, gtp_edge_suppress_switch_store);
static struct gtp_driver_sysfs_entry gtp_dclick_switch =
	__ATTR(dclick_switch, 0644,
			gtp_dclick_switch_show, gtp_dclick_switch_store);
static struct gtp_driver_sysfs_entry gtp_dclick_simulate_switch =
	__ATTR(dclick_simulate_switch, 0644,
			gtp_dclick_simulate_switch_show, gtp_dclick_simulate_switch_store);
static struct gtp_driver_sysfs_entry gtp_gesture_switch =
	__ATTR(gesture_switch, 0644, gtp_gesture_switch_show, gtp_gesture_switch_store);
static struct gtp_driver_sysfs_entry gtp_gesture_switch_export =
	__ATTR(gesture_switch_export, 0644, gtp_gesture_switch_export_show, gtp_gesture_switch_export_store);
static struct gtp_driver_sysfs_entry gtp_dclick_proximity_switch =
	__ATTR(dclick_proximity_switch, 0644,
			gtp_dclick_proximity_switch_show, gtp_dclick_proximity_switch_store);
static struct gtp_driver_sysfs_entry gtp_dclick_lcd_state =
	__ATTR(dclick_lcd_state, 0644,
			gtp_dclick_lcd_state_show, gtp_dclick_lcd_state_store);
static struct gtp_driver_sysfs_entry gtp_swipe_down_switch =
	__ATTR(swipe_switch, 0644,
			gtp_swipe_down_switch_show, gtp_swipe_down_switch_store);
static struct gtp_driver_sysfs_entry gtp_touchpanel_devices =
	__ATTR(touchpanel_devices, 0644,
			gtp_touch_ic_name_show, gtp_null_store);
static struct gtp_driver_sysfs_entry gtp_user_defined_gesture_enable =
	__ATTR(user_defined_gesture_enable, 0644,
			gtp_user_defined_gesture_enable_show, gtp_null_store);

static struct attribute *gtp_def_attrs[] = {
 	&gtp_firmware_update.attr,
 	&gtp_firmware_version.attr,
	&gtp_config_version.attr,
 	&gtp_firmware_module_id.attr,
 	&gtp_sensor_rawdata.attr,
	&gtp_sensor_delta.attr,
 	&gtp_sensor_baseline.attr,
 	&gtp_ts_log_switch.attr,
 	&gtp_sensor_rx_tx.attr,
 	&gtp_ts_is_calling.attr,
 	&gtp_gesture_point.attr,
 	&gtp_gesture_switch.attr,
 	&gtp_gesture_switch_export.attr,
	&gtp_ts_ipod_flag.attr,
	&gtp_dclick_flag_info.attr,
	&gtp_edge_suppress_switch.attr,
	&gtp_dclick_switch.attr,
	&gtp_dclick_simulate_switch.attr,
	&gtp_dclick_proximity_switch.attr,
	&gtp_dclick_lcd_state.attr,
	&gtp_swipe_down_switch.attr,
	&gtp_touchpanel_devices.attr,
 	&gtp_user_defined_gesture_enable.attr,
 
 NULL
};

static ssize_t gtp_state_show(struct kobject *k, struct attribute *attr, char *buf)
{
 	struct kobj_attribute *kobj_attr;
 	int ret = -EIO;

	 kobj_attr = container_of(attr, struct kobj_attribute, attr);

 	if (kobj_attr->show)
	 ret = kobj_attr->show(k, kobj_attr, buf);

 	return ret;
}

static ssize_t gtp_state_store(struct kobject *k, struct attribute *attr,
			   const char *buf, size_t count)
{
 	struct kobj_attribute *kobj_attr;
 	int ret = -EIO;

 	kobj_attr = container_of(attr, struct kobj_attribute, attr);

 	if (kobj_attr->store)
		 ret = kobj_attr->store(k, kobj_attr, buf, count);

 	return ret;
}
static void gtp_state_release(struct kobject *kobj)
{
	VIVO_TS_LOG_INF("gtp_state_release.\n");
}

static const struct sysfs_ops gtp_state_sysfs_ops = {
 	.show = gtp_state_show,
 	.store = gtp_state_store,
};

static struct kobj_type gtp_state_type = {
 	.sysfs_ops  = &gtp_state_sysfs_ops,
 	.release	 = gtp_state_release,
 	.default_attrs = gtp_def_attrs,
};

static int gtp_state_init(void) 
{ 
 	int ret; 
 	ret = kobject_init_and_add(&gtp_state_kobj, &gtp_state_type,
				 NULL, "touchscreen");
	VIVO_TS_LOG_INF("create mode_id kobject:%d", ret);
 	return ret; 
} 

/***********************************************************************************************
Name	:	 
Input	:	                    
Output	:	
function	:
***********************************************************************************************/
/*lyf add virtual key support */
static const char  *virtual_key_str =NULL;
static void gtp_set_virtual_key_string(const char *vkey)
{
     virtual_key_str = vkey;
}
static ssize_t gtp_vkeys_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
#if 1
    int rc = -EINVAL;
    if(virtual_key_str != NULL)
	{
	    VIVO_TS_LOG_INF("VKEY= %s \n", virtual_key_str);
	    rc = sprintf(buf,"%s\n", virtual_key_str);  
	}
	else
	{
	    VIVO_TS_LOG_ERR("VKEY support failed or please check dts !!\n");
	}
	
	return rc;
#else
	return sprintf(buf,
	__stringify(EV_KEY) ":" __stringify(KEY_MENU) ":92:1380:140:100"
	":" __stringify(EV_KEY) ":" __stringify(KEY_BAR_SWIPE) ":226:1380:128:100"
	":" __stringify(EV_KEY) ":" __stringify(172) ":360:1380:140:100"
	":" __stringify(EV_KEY) ":" __stringify(KEY_BAR_SWIPE) ":500:1380:140:100"
	":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":640:1380:140:100"
	"\n");
#endif
}

static struct kobj_attribute gtp_vkeys_attr = {
	.attr = {
		.name = "virtualkeys.goodix-ts",
		.mode = S_IRUGO,
	},
	.show = &gtp_vkeys_show,
};
static struct attribute *gtp_properties_attrs[] = {
	&gtp_vkeys_attr.attr,
	NULL
};
static struct attribute_group gtp_properties_attr_group = {
	.attrs = gtp_properties_attrs,
};
static struct kobject *properties_kobj;
/*end add virtual key support */
//zhj add for edge_suppress_switch
/*******************************************************
Function:
    Enter edge restrain mode.
Input:
    ts: goodix tp private data
Output:
    1: succeed, otherwise failed
*******************************************************/
static int gtp_enter_edge_restrain(struct goodix_ts_data *ts)
{
    int ret = -1;
    int retry = 0;
    u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 0x41};


    VIVO_TS_LOG_INF("Entering edge restrain mode.");
    while(retry++ < 5)
    {
        i2c_control_buf[0] = 0x80;
        i2c_control_buf[1] = 0x46;
        ret = gtp_i2c_write(i2c_connect_client, i2c_control_buf, 3);
        if (ret < 0)
        {
            VIVO_TS_LOG_ERR("failed to set edge restrain mode flag into 0x8046, %d", retry);
            continue;
        }
        i2c_control_buf[0] = 0x80;
        i2c_control_buf[1] = 0x40;
        ret = gtp_i2c_write(i2c_connect_client, i2c_control_buf, 3);
        if (ret > 0)
        {
            VIVO_TS_LOG_INF("Edge restrain mode enabled.");
            return ret;
        }
        msleep(10);
    }
    VIVO_TS_LOG_ERR("GTP send edge restrain cmd failed.");
    return ret;
}
/*******************************************************
Function:
    Leave edge restrain mode.
Input:
    ts: goodix tp private data
Output:
    1: succeed, otherwise failed
*******************************************************/
static int gtp_leave_edge_restrain(struct goodix_ts_data *ts)
{
    int ret = -1;
    int retry = 0;
    u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 0x42};


    VIVO_TS_LOG_INF("Leaving edge restrain mode.");
    while(retry++ < 5)
    {
        i2c_control_buf[0] = 0x80;
        i2c_control_buf[1] = 0x46;
        ret = gtp_i2c_write(i2c_connect_client, i2c_control_buf, 3);
        if (ret < 0)
        {
            VIVO_TS_LOG_ERR("failed to set leave edge restrain mode flag into 0x8046, %d", retry);
            continue;
        }
        i2c_control_buf[0] = 0x80;
        i2c_control_buf[1] = 0x40;
        ret = gtp_i2c_write(i2c_connect_client, i2c_control_buf, 3);
        if (ret > 0)
        {
            VIVO_TS_LOG_INF("Edge restrain mode disabled.");
            return ret;
        }
        msleep(10);
    }
    VIVO_TS_LOG_ERR("GTP leave edge restrain cmd failed.");
    return ret;
}
//zhj add end


//add charge change func for gt970
//extern int usb_charger_flag;
void gtp_charger_switch(char on_or_off)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
	u8 buf[3] = {0x80, 0x40};
	int ret = 0;
	int run_tag = 0;
	
	VIVO_TS_LOG_INF("enter gtp_charger_switch.product name = %s\n", global_product_name);

	if (ts->enter_update)
    {
		VIVO_TS_LOG_INF("The IC enter Firmware update mode...%s\n", __func__);
        return;
    }
		
	if(0==strcmp(global_product_name,"PD1503V") || 0==strcmp(global_product_name,"PD1419C") ||0==strcmp(global_product_name,"PD1510")
		||0==strcmp(global_product_name,"PD1505") ||0==strcmp(global_product_name,"PD1523A") ||0==strcmp(global_product_name,"PD1524")){
		run_tag = 1;
	}

	
	if(0 == run_tag)
		return ;
	
	VIVO_TS_LOG_INF("[%s] enter , chr_status=%d\n", __func__, on_or_off);	
	chr_status = on_or_off;
		
	if(1 == ts->gtp_is_suspend) {
		if(chr_status)
			VIVO_TS_LOG_INF("[%s] usb in but suspend\n", __func__);
		else	
			VIVO_TS_LOG_INF("[%s] usb out but suspend\n", __func__);
		return;
	}
	
	if(chr_status) {    
		VIVO_TS_LOG_INF("[%s] enter if chr_status\n", __func__);	
		VIVO_TS_LOG_INF("[%s] usb in\n", __func__);
		buf[2] = 0x06;
		ret = gtp_i2c_write(i2c_connect_client, buf, 3);  
		if (ret > 0) {
			VIVO_TS_LOG_INF("[%s] GTP enter charger status!", __func__);
		}
    } else {                 
		VIVO_TS_LOG_INF("[%s] enter else chr_status\n", __func__);	
		VIVO_TS_LOG_INF("[%s] usb out\n", __func__);
		buf[2] = 0x07;
		ret = gtp_i2c_write(i2c_connect_client, buf, 3);  
		if (ret > 0) {
			VIVO_TS_LOG_INF("[%s] GTP leave charger status!", __func__);
		}
    }
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
extern unsigned int is_atboot;

//add for charge judge and log_switch interface. gtp_common_data will be register in probe
static int gtp_get_log_switch(void)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
	
	return ts->log_switch;
}
static vivo_touchscreen_common_data gtp_common_data = {
	.driver_name = DRIVER_NAME,
	.charge_connect_judge = gtp_charger_switch,
    .get_ts_log_switch = gtp_get_log_switch,
};

static int goodix_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    s32 ret = -1;
	struct goodix_ts_platform_data *pdata;
	struct goodix_ts_data *ts;
    u16 version_info;
	int err = 0;
	int i = 0;

	vivo_touchscreen_get_product_name(&global_product_name);
	if(NULL == global_product_name) {
		VIVO_TS_LOG_ERR("no product name! pls check device tree\n");
		return -1;
	}
	VIVO_TS_LOG_INF("product name:%s\n", global_product_name);

	vivo_touchscreen_get_vdd_which_mode(&global_vdd_mode);
	VIVO_TS_LOG_INF("===vdd_ts_mode is :%s ===\n", global_vdd_mode);
	if(NULL == global_vdd_mode) {
		VIVO_TS_LOG_ERR("no ts_vdd_mode! please check device tree\n");
		//return -1;
	}
    GTP_DEBUG_FUNC();

	VIVO_TS_LOG_INF("GTP I2C Address: 0x%02x\n", client->addr);
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct goodix_ts_platform_data), GFP_KERNEL);
		if (!pdata) {
			VIVO_TS_LOG_ERR("GTP Failed to allocate memory for pdata\n");
			return -ENOMEM;
		}
		ret = goodix_parse_dt(&client->dev, pdata);
		if (ret)
			return ret;
	} else {
		pdata = client->dev.platform_data;
	}
	if (!pdata) {
		VIVO_TS_LOG_ERR("GTP invalid pdata\n");
		return -EINVAL;
	}

    //do NOT remove these logs
    VIVO_TS_LOG_INF("GTP Driver Version: %s", GTP_DRIVER_VERSION);
    VIVO_TS_LOG_INF("GTP Driver Built@%s, %s", __TIME__, __DATE__);
    VIVO_TS_LOG_INF("GTP I2C Address: 0x%02x", client->addr);

    i2c_connect_client = client;

/*Add lyf add virtual key support*/
	properties_kobj = kobject_create_and_add("board_properties",
					NULL);
	if (properties_kobj)
		err = sysfs_create_group(properties_kobj,
			&gtp_properties_attr_group);
	if (!properties_kobj || err) {
		VIVO_TS_LOG_ERR("failed to create board_properties\n");
		goto exit_check_functionality_failed;
	}
/*Add lyf add virtual key support*/

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
    {
        VIVO_TS_LOG_ERR("I2C check functionality failed.\n");
        return -ENODEV;
    }

	ts = devm_kzalloc(&client->dev, sizeof(*ts), GFP_KERNEL);
	if (!ts) {
		VIVO_TS_LOG_ERR("no enough memory for ts\n");
		return -ENOMEM;
	}

    memset(ts, 0, sizeof(*ts));

#if 0
	VIVO_TS_LOG_INF(">>--\n");
    	ts->client = client;

	ts->pdata = pdata;
    	i2c_set_clientdata(client, ts);
    	ret = gtp_request_io_port(ts);
    	if (ret < 0)
    	{
        	VIVO_TS_LOG_ERR("GTP request IO port failed.");
        	kfree(ts);
        	return ret;
    	}

	VIVO_TS_LOG_INF(">>--\n");
#endif

    INIT_WORK(&ts->work, goodix_ts_work_func);
    ts->client = client;

	ts->pdata = pdata;

	spin_lock_init(&ts->irq_lock);          // 2.6.39 later
	// ts->irq_lock = SPIN_LOCK_UNLOCKED;   //2.6.39 & before
//#if GTP_ESD_PROTECT
	if(vivo_touchscreen_is_support(FS_ANTI_ESD)) {
		ts->clk_tick_cnt = 2 * HZ;      // HZ: clock ticks in 1 second generated by system
		VIVO_TS_LOG_INF("Clock ticks for an esd cycle: %d", ts->clk_tick_cnt);
		spin_lock_init(&ts->esd_lock);
	}
	// ts->esd_lock = SPIN_LOCK_UNLOCKED;
//#endif
	i2c_set_clientdata(client, ts);

	ts->gtp_rawdiff_mode = 0;
	ts->power_on = false;
	
	ts->log_switch = 0;
	ts->is_calling = 0;
	ts->largetouch_flag = 0;
	ts->max_touch_num = 10;
	mutex_init(&ts->suspend_mutex);
	wake_lock_init(&ts->suspend_wakelock, WAKE_LOCK_SUSPEND, "bbkts_wakelock");

if(vivo_touchscreen_is_support(FS_DCLICK_WAKE)){
	mutex_init(&ts->gesture_mutex);
	ts->ts_ipod_flag = 0;
	ts->ts_dclick_switch = 1;
	ts->ts_dclick_simulate_switch = 0;
	ts->need_change_to_dclick = 0;
	ts->gesture_switch = 0;
	ts->gesture_switch_export = 0;
	ts->swipe_down_switch = 0;
	ts->has_lcd_shutoff = 0;
	ts->gesture_state =0;
	ts->edge_suppress_switch = 0;

	for(i=0; i<130; i++)
	{
		ts->buf_gesture[i]= 65535;
	}
	for(i=0; i<250; i++)
	{
		ts->gesture_coordinate[i] = 0;
	}
	
	atomic_set(&ts->ts_state, TOUCHSCREEN_NORMAL);
	setup_timer(&ts->dclick_timer, 
			goodix_dclick_timer_func, (unsigned long)&ts);
	INIT_WORK(&ts->dclick_timer_work, goodix_dclick_timer_work_func);
	//ts->start = 1;
	ts->has_dclick_timer_start = false;
	ts->is_dclick_valide = false;
	ts->pressed_count = 0;
	ts->release_count = 0;
	ts->first_press_x = -1;
	ts->first_press_y = -1;
	ts->second_press_x = -1;
	ts->second_press_y = -1;
	ts->pre_state = 0;
	ts->dclick_dimension_x_min = 140;
	ts->dclick_dimension_x_max = ts->pdata->x_max - 140;
	ts->dclick_dimension_y_min = 140;
	ts->dclick_dimension_y_max = ts->pdata->y_max - 140;
}

   	ret = gtp_request_io_port(ts);
    	if (ret < 0)
    	{
        	VIVO_TS_LOG_ERR("GTP request IO port failed.");
			i2c_set_clientdata(client, NULL);
        	return ret;
    	}

	ret = goodix_power_init(ts);
	if (ret) {
		VIVO_TS_LOG_ERR("power init failed\n");
		//goto exit_free_io_port;
	}

	ret = goodix_power_on(ts);
	if (ret) {
		VIVO_TS_LOG_ERR("GTP power on failed\n");
		//goto exit_deinit_power;
	}
/*[BUGFIX]PR-667466. TP INT pull-up enable.*/
	gtp_pinctrl_init(&ts->client->dev);
	ret = pinctrl_select_state(gt9xx_pctrl.pinctrl,
			gt9xx_pctrl.gpio_state_active);
	if (ret)
		VIVO_TS_LOG_ERR("cannot set pin to suspend state");
/*[BUGFIX]PR-667466. TP INT pull-up enable.*/

	gtp_reset_guitar(ts->client, 20);

	ret = gtp_i2c_test(client);
	if (ret < 0) {
        VIVO_TS_LOG_ERR("I2C communication ERROR!");
		i2c_set_clientdata(client, NULL);
		return ret;
    }
	/*[FEATURE]do this after  gtp_i2c_test.*/
    	//g_tp_device_name = "gt9159";
	/*[FEATURE]*/
	
#if 0
	VIVO_TS_LOG_INF(">>-- %s line:%d \n", __func__, __LINE__);
    	ret = gtp_i2c_test(client);
	VIVO_TS_LOG_INF(">>-- %s line:%d, ret=%d \n", __func__, __LINE__, ret);
    	if (ret < 0)
    	{
        	VIVO_TS_LOG_ERR("I2C communication ERROR!");
    	}
#endif
    ret = gtp_read_version(client, &version_info);
    if (ret < 0)
    {
        VIVO_TS_LOG_ERR("Read version failed.");
    }

    ret = gtp_init_panel(ts);
    if (ret < 0)
    {
        VIVO_TS_LOG_ERR("GTP init panel failed.");
        //ts->abs_x_max = GTP_MAX_WIDTH;
      //  ts->abs_y_max = GTP_MAX_HEIGHT;
       // ts->int_trigger_type = GTP_INT_TRIGGER;
    }
    // Create proc file system
    gt91xx_config_proc = proc_create(GT91XX_CONFIG_PROC_FILE, 0666, NULL, &config_proc_ops);
    if (gt91xx_config_proc == NULL)
    {
        VIVO_TS_LOG_ERR("create_proc_entry %s failed", GT91XX_CONFIG_PROC_FILE);
    }
    else
    {
        VIVO_TS_LOG_ERR("create proc entry %s success", GT91XX_CONFIG_PROC_FILE);
    }

//#if GTP_ESD_PROTECT
	if(vivo_touchscreen_is_support(FS_ANTI_ESD))
	    gtp_esd_switch(client, SWITCH_ON);
//#endif

	gtp_init_node();

	if((!strcmp(global_product_name,"PD1523A") || !strcmp(global_product_name,"PD1524")) && usb_charger_flag == 1) {
		gtp_charger_switch(1);	
	}

#if GTP_AUTO_UPDATE
	/*if is atboot don't update firmware*/
	if(!is_atboot)
	{
    	ret = gup_init_update_proc(ts);
    	if (ret < 0)
    	{
        	VIVO_TS_LOG_ERR("Create update thread error.");
    	}
	}
#endif
    ret = gtp_request_input_dev(ts);
    if (ret < 0)
    {
        VIVO_TS_LOG_ERR("GTP request input dev failed");
    }
	input_set_drvdata(ts->input_dev, ts);
    ret = gtp_request_irq(ts); 
    if (ret < 0)
    {
        VIVO_TS_LOG_INF("GTP works in polling mode.");
    }
    else
    {
        VIVO_TS_LOG_INF("GTP works in interrupt mode.");
    }

	ret = gtp_read_fw_version(client, &version_info);
	if (ret != 2)
		VIVO_TS_LOG_ERR("GTP firmware version read failed.\n");

	ret = gtp_check_product_id(client);
	if (ret != 0) {
		VIVO_TS_LOG_ERR("GTP Product id doesn't match. ret=%d\n", ret);
		//goto exit_free_irq;
	}

	ret = bbk_drivers_info_switch_register_callback(&gtp_log_switch_handler);

	if (ret < 0) {
		VIVO_TS_LOG_ERR("GTP Failed to register callback handler\n");
	}

    if (ts->use_irq)
    {
        gtp_irq_enable(ts);
    }

#if 0
	ret = enable_irq_wake(client->irq);
	if (ret) {
		VIVO_TS_LOG_ERR("set_irq_wake failed for gpio %d, "
			"irq %d\n", client->irq, client->irq);
		goto exit_check_functionality_failed;
	}
#endif

#if GTP_CREATE_WR_NODE
    init_wr_node(client);
#endif
	INIT_WORK(&ts->irq_err_work, gtp_irq_err_work_func);
	ts->irq_err_workqueue = create_singlethread_workqueue("gtp_err_wq");
	if (!ts->irq_err_workqueue) {
		VIVO_TS_LOG_ERR("can't create irq err worqueue\n");
		goto exit_irq_request_failed;
	}
	
	/*Add by liuyunfeng for system file interface*/
	gtp_state_init();

	if (-1 == register_touchscreen_common_interface(&gtp_common_data)) {
		goto exit_irq_request_failed;	
	}

    return 0;
exit_irq_request_failed:
	destroy_workqueue(ts->irq_err_workqueue);
exit_check_functionality_failed:
	return err;
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

#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&ts->early_suspend);
#elif defined(CONFIG_FB) 
	if (fb_unregister_client(&ts->fb_notif))
		VIVO_TS_LOG_ERR("falied unregister notifier\n");
#endif

#if GTP_CREATE_WR_NODE
    uninit_wr_node();
#endif

//#if GTP_ESD_PROTECT
	if(vivo_touchscreen_is_support(FS_ANTI_ESD))
	    destroy_workqueue(gtp_esd_check_workqueue);
//#endif

    if (ts) 
    {
        if (ts->use_irq)
        {
            GTP_GPIO_AS_INPUT(ts->pdata->irq_gpio);
            GTP_GPIO_FREE(ts->pdata->irq_gpio);
            free_irq(client->irq, ts);
        }
        else
        {
            hrtimer_cancel(&ts->timer);
        }
    }

    VIVO_TS_LOG_INF("GTP driver removing...");
    i2c_set_clientdata(client, NULL);
    input_unregister_device(ts->input_dev);
    kfree(ts);

    return 0;
}


static void ts_scan_switch(bool on)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
	int ret;
	int i;
	GTP_DEBUG_FUNC();
	VIVO_TS_LOG_INF("parameter is %d~~~", on);

	if (!ts) {
		VIVO_TS_LOG_ERR("ts driver data is NULL");
		return;
	}
	if (on) {/*enter gesture mode*/
		/*[BUGFIX]TP INT pull-up enable.*/
		ret = pinctrl_select_state(gt9xx_pctrl.pinctrl,
				gt9xx_pctrl.gpio_state_active);
		if (ret)
			VIVO_TS_LOG_ERR("cannot set pin to suspend state\n");
		/*[BUGFIX]TP INT pull-up enable.*/
		if (ts->use_irq)
			gtp_irq_disable(ts);
		for(i = 0;i < 3;i++){		
			//ret = gtp_wakeup_sleep(ts);
			ret = gtp_irq_reset(ts);
			if(ret > 0){
				VIVO_TS_LOG_INF("success to wakeup from sleep. ");
				break;
			}
			VIVO_TS_LOG_ERR("Fail to wakeup from sleep. retry = %d", i);
			continue;
		}
		if (ts->use_irq)
			gtp_irq_enable(ts);
		mutex_lock(&ts->gesture_mutex);
		atomic_set(&ts->ts_state, TOUCHSCREEN_GESTURE);
		mutex_unlock(&ts->gesture_mutex);
		ts->gesture_state = 1;
		VIVO_TS_LOG_INF("~~~TOUCHSCREEN_GESTURE~~ts->gesture_state is %d~~~", ts->gesture_state);
		gtp_enter_doze(ts);
		//msleep(50);
	}else{/*quit gesture mode*/
		/*[BUGFIX]TP INT pull-up enable.*/
		ret = pinctrl_select_state(gt9xx_pctrl.pinctrl,
				gt9xx_pctrl.gpio_state_active);
		if (ret)
			VIVO_TS_LOG_ERR("cannot set pin to suspend state\n");
		/*[BUGFIX]TP INT pull-up enable.*/
		if (ts->use_irq)
			gtp_irq_disable(ts);
		for(i = 0;i < 3;i++){		
			ret = gtp_wakeup_sleep(ts);
			if(ret > 0){
				VIVO_TS_LOG_INF("success to wakeup from sleep. ");
				break;
			}
			VIVO_TS_LOG_ERR("Fail to wakeup from sleep. retry = %d", i);
			continue;
		}
		ts->gesture_state = 0;
		VIVO_TS_LOG_INF("~~TOUCHSCREEN_SLEEP~~ts->gesture_state is %d~~~", ts->gesture_state);
		mutex_lock(&ts->gesture_mutex);
		atomic_set(&ts->ts_state, TOUCHSCREEN_SLEEP);
		mutex_unlock(&ts->gesture_mutex);
		gtp_enter_sleep(ts);
		/* to avoid waking up while not sleeping,
		 * delay 48 + 10ms to ensure reliability */
		msleep(58);
	}
	/*[BUGFIX]PR-667466. TP INT pull-up enable.*/
	ret = pinctrl_select_state(gt9xx_pctrl.pinctrl,
			gt9xx_pctrl.gpio_state_suspend);
	if (ret)
		VIVO_TS_LOG_ERR("cannot set pin to suspend state\n");
	/*[BUGFIX]PR-667466. TP INT pull-up enable.*/
}

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
	struct goodix_ts_data *ts = dev_get_drvdata(dev);
	int ret = 0;
	int i = 0;	
	
	//close large touch detect while suspend
	large_touch_mask = 0; 

	VIVO_TS_LOG_INF("enter\n");
	gesture_clear_wakeup_data();
	
/*[BUGFIX]PR-667466. TP INT pull-up enable.*/
	ret = pinctrl_select_state(gt9xx_pctrl.pinctrl,
			gt9xx_pctrl.gpio_state_suspend);
	if (ret)
		VIVO_TS_LOG_ERR("cannot set pin to suspend state");
/*[BUGFIX]PR-667466. TP INT pull-up enable.*/
	if (ts->gtp_is_suspend) {
		VIVO_TS_LOG_DBG("Already in suspend state.\n");
		return 0;
	}
//#if defined(BBK_LARGE_OBJ_SUPPRESSION)
if(vivo_touchscreen_is_support(FS_LARGE_OBJ_SUPPRESSION)) {
	if(ts->is_calling == 1)
	{
		msleep(500);		
	}
}

	//mutex_lock(&ts->suspend_mutex);
//#if GTP_ESD_PROTECT
	if(vivo_touchscreen_is_support(FS_ANTI_ESD))
		gtp_esd_switch(ts->client, SWITCH_OFF);
//#endif

	ts->gtp_is_suspend = 1;
if(vivo_touchscreen_is_support(FS_DCLICK_WAKE)){
	if (ts->need_change_to_dclick == 1) {
		ts->need_change_to_dclick = 0;
		ts->gesture_state =0;
		ts_scan_switch(1);
		
	}else{
		mutex_lock(&ts->gesture_mutex);
		atomic_set(&ts->ts_state, TOUCHSCREEN_SLEEP);
		mutex_unlock(&ts->gesture_mutex);
		
		VIVO_TS_LOG_INF("~~~~~gesture enter sleep~~~~~~~\n");
		if (ts->use_irq)
			gtp_irq_disable(ts);

		ret = gtp_enter_sleep(ts);
		if (ret < 0)
			VIVO_TS_LOG_ERR("GTP early suspend failed.\n");
		/* to avoid waking up while not sleeping,
		 * delay 48 + 10ms to ensure reliability */
		msleep(58);
	}
}else{
		VIVO_TS_LOG_INF("~~~~~gesture enter sleep~~~~~~~\n");
		if (ts->use_irq)
			gtp_irq_disable(ts);

		ret = gtp_enter_sleep(ts);
		if (ret < 0)
			VIVO_TS_LOG_DBG("GTP early suspend failed.\n");
		/* to avoid waking up while not sleeping,
		 * delay 48 + 10ms to ensure reliability */
		msleep(58);
	}
	
	//vivo liuyunfeng add for touch up release
	for (i = 0; i < GTP_MAX_TOUCH; i++) {
		gtp_touch_up(ts, i);
	}
	input_sync(ts->input_dev);
	//vivo liuyunfeng add end

	//mutex_unlock(&ts->suspend_mutex);
	VIVO_TS_LOG_INF("====%s===%d\n", __func__,  __LINE__);
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
	struct goodix_ts_data *ts = dev_get_drvdata(dev);
	int ret = 0;
	
	if (!ts->gtp_is_suspend) {
		VIVO_TS_LOG_DBG("Already in awake state.\n");
		return 0;
	}
	VIVO_TS_LOG_INF("~~~%s: line:%d ~~lcd_shutoff:%d~\n", __func__,__LINE__,ts->has_lcd_shutoff);

	//mutex_lock(&ts->suspend_mutex);
//#if GTP_GESTURE_WAKEUP
if(vivo_touchscreen_is_support(FS_DCLICK_WAKE)){
	mutex_lock(&ts->gesture_mutex);
	atomic_set(&ts->ts_state, TOUCHSCREEN_NORMAL);
	mutex_unlock(&ts->gesture_mutex);
	if(ts->has_lcd_shutoff == 1)
	{
		ts->need_change_to_dclick = 1;
	}
	else
	{
		ts->need_change_to_dclick = 0;
	}
	doze_status = DOZE_DISABLED;
}

/*[BUGFIX]TP INT pull-up enable.*/
  	ret = pinctrl_select_state(gt9xx_pctrl.pinctrl,
			gt9xx_pctrl.gpio_state_active);
	if (ret)
		VIVO_TS_LOG_ERR("cannot set pin to suspend state");
/*[BUGFIX]TP INT pull-up enable.*/

	ret = gtp_wakeup_sleep(ts);

	if (ret <= 0)
		VIVO_TS_LOG_ERR("GTP resume failed.\n");

	if (ts->use_irq)
		gtp_irq_enable(ts);

//#if GTP_ESD_PROTECT
	if(vivo_touchscreen_is_support(FS_ANTI_ESD))
		gtp_esd_switch(ts->client, SWITCH_ON);
//#endif
	ts->gtp_is_suspend = 0;
	//add for cahrge
	if(!strcmp(global_product_name,"PD1503V")) {
		gtp_charger_switch(chr_status);	
	}
	if(!strcmp(global_product_name,"PD1419C")) {
		gtp_charger_switch(chr_status);	
	}
	if(!strcmp(global_product_name,"PD1523A")) {
		gtp_charger_switch(chr_status);	
	}
	if(!strcmp(global_product_name,"PD1524")) {
		gtp_charger_switch(chr_status); 
	}	
	//mutex_unlock(&ts->suspend_mutex);
	VIVO_TS_LOG_INF("====%s===%d\n", __func__,  __LINE__);
	
	return ret;
}

#if defined(CONFIG_FB)
static void fb_notifier_resume_work(struct work_struct *work)
{
	struct goodix_ts_data *ts = container_of(work, struct goodix_ts_data,fb_notifier_resume_work);
	int ret;

	mutex_lock(&ts->suspend_mutex);
	ret = goodix_ts_resume(&ts->client->dev);
	if(ret<0) {
		VIVO_TS_LOG_ERR("%s: Failed to resume device\n",__func__);	
	}
	mutex_unlock(&ts->suspend_mutex);


}
static void fb_notifier_suspend_work(struct work_struct *work)
{
	struct goodix_ts_data *ts = container_of(work, struct goodix_ts_data,fb_notifier_suspend_work);
    int ret;

	mutex_lock(&ts->suspend_mutex);
	ret = goodix_ts_suspend(&ts->client->dev);
	if(ret) {
			VIVO_TS_LOG_ERR("%s: Failed to suspend device\n",__func__);
	}
	mutex_unlock(&ts->suspend_mutex);
}

static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	static int fb_suspend_flag = 0;
	int *blank;
	struct goodix_ts_data *ts =
		container_of(self, struct goodix_ts_data, fb_notif);
#if 1
	if (evdata && evdata->data && event == FB_EARLY_EVENT_BLANK && ts) {
		blank = (evdata->data);
		if (*blank == FB_BLANK_UNBLANK && fb_suspend_flag == 1) {
			fb_suspend_flag = 0;
			queue_work(ts->fb_notifier_workqueue,&ts->fb_notifier_resume_work);		
		}
		else if (*blank == FB_BLANK_POWERDOWN){
			fb_suspend_flag = 1;
			queue_work(ts->fb_notifier_workqueue,&ts->fb_notifier_suspend_work);
			}
		}
	
#else
	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
			ts && ts->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK){
			goodix_ts_resume(&ts->client->dev);
		}
		else if (*blank == FB_BLANK_POWERDOWN)
		{
			goodix_ts_suspend(&ts->client->dev);
		}
	}
#endif
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
    s8 ret = -1;
    ts = container_of(h, struct goodix_ts_data, early_suspend);

    GTP_DEBUG_FUNC();

    VIVO_TS_LOG_INF("System suspend.");

    ts->gtp_is_suspend = 1;
//#if GTP_ESD_PROTECT
	if(vivo_touchscreen_is_support(FS_ANTI_ESD))
	    gtp_esd_switch(ts->client, SWITCH_OFF);
//#endif

if(vivo_touchscreen_is_support(FS_DCLICK_WAKE)){//GTP_GESTURE_WAKEUP
    ret = gtp_enter_doze(ts);
}else{
    if (ts->use_irq)
    {
        gtp_irq_disable(ts);
    }
    else
    {
        hrtimer_cancel(&ts->timer);
    }
    ret = gtp_enter_sleep(ts);
}
    if (ret < 0)
    {
        VIVO_TS_LOG_ERR("GTP early suspend failed.");
    }
    // to avoid waking up while not sleeping
    //  delay 48 + 10ms to ensure reliability
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
static void goodix_ts_late_resume(struct early_suspend *h)
{
    struct goodix_ts_data *ts;
    s8 ret = -1;
    ts = container_of(h, struct goodix_ts_data, early_suspend);

    GTP_DEBUG_FUNC();

    VIVO_TS_LOG_INF("System resume.");

    ret = gtp_wakeup_sleep(ts);

if(vivo_touchscreen_is_support(FS_DCLICK_WAKE)){//GTP_GESTURE_WAKEUP
    doze_status = DOZE_DISABLED;
}

    if (ret < 0)
    {
        VIVO_TS_LOG_ERR("GTP later resume failed.");
    }
    gtp_send_cfg(ts->client);

    if (ts->use_irq)
    {
        gtp_irq_enable(ts);
    }
    else
    {
        hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
    }

    ts->gtp_is_suspend = 0;
//#if GTP_ESD_PROTECT
	if(vivo_touchscreen_is_support(FS_ANTI_ESD))
	    gtp_esd_switch(ts->client, SWITCH_ON);
//#endif
}
#endif

#endif


//#if GTP_ESD_PROTECT
s32 gtp_i2c_read_no_rst(struct i2c_client *client, u8 *buf, s32 len)
{
    struct i2c_msg msgs[2];
    s32 ret=-1;
    s32 retries = 0;

    GTP_DEBUG_FUNC();

    msgs[0].flags = !I2C_M_RD;
    msgs[0].addr  = client->addr;
    msgs[0].len   = GTP_ADDR_LENGTH;
    msgs[0].buf   = &buf[0];
    //msgs[0].scl_rate = 300 * 1000;    // for Rockchip, etc.

    msgs[1].flags = I2C_M_RD;
    msgs[1].addr  = client->addr;
    msgs[1].len   = len - GTP_ADDR_LENGTH;
    msgs[1].buf   = &buf[GTP_ADDR_LENGTH];
    //msgs[1].scl_rate = 300 * 1000;

    while(retries < 5)
    {
        ret = i2c_transfer(client->adapter, msgs, 2);
        if(ret == 2)break;
        retries++;
    }
    if ((retries >= 5))
    {
        VIVO_TS_LOG_ERR("I2C Read: 0x%04X, %d bytes failed, errcode: %d!", (((u16)(buf[0] << 8)) | buf[1]), len-2, ret);
    }
    return ret;
}

s32 gtp_i2c_write_no_rst(struct i2c_client *client,u8 *buf,s32 len)
{
    struct i2c_msg msg;
    s32 ret = -1;
    s32 retries = 0;

    GTP_DEBUG_FUNC();

    msg.flags = !I2C_M_RD;
    msg.addr  = client->addr;
    msg.len   = len;
    msg.buf   = buf;
    //msg.scl_rate = 300 * 1000;    //for Rockchip, etc

    while(retries < 5)
    {
        ret = i2c_transfer(client->adapter, &msg, 1);
        if (ret == 1)break;
        retries++;
    }
    if((retries >= 5))
    {
        VIVO_TS_LOG_ERR("I2C Write: 0x%04X, %d bytes failed, errcode: %d!", (((u16)(buf[0] << 8)) | buf[1]), len-2, ret);
    }
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

    if (SWITCH_ON == on)     // switch on esd
    {
        if (!ts->esd_running)
        {
            ts->esd_running = 1;
            spin_unlock(&ts->esd_lock);
            VIVO_TS_LOG_INF("Esd started");
            queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, ts->clk_tick_cnt);
        }
        else
        {
            spin_unlock(&ts->esd_lock);
        }
    }
    else    // switch off esd
    {
        if (ts->esd_running)
        {
            ts->esd_running = 0;
            spin_unlock(&ts->esd_lock);
            VIVO_TS_LOG_INF("Esd cancelled");
            cancel_delayed_work_sync(&gtp_esd_check_work);
        }
        else
        {
            spin_unlock(&ts->esd_lock);
        }
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
    VIVO_TS_LOG_DBG("[Esd]Init external watchdog");
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

    if (ts->gtp_is_suspend)
    {
        VIVO_TS_LOG_INF("Esd suspended!");
        return;
    }

    for (i = 0; i < 3; i++)
    {
        ret = gtp_i2c_read_no_rst(ts->client, esd_buf, 4);
       	VIVO_TS_LOG_DBG("[Esd]0x8040 = 0x%02X, 0x8041 = 0x%02X", esd_buf[2], esd_buf[3]);
        if ((ret < 0))
        {
            // IIC communication problem
            continue;
        }
        else
        {
            if ((esd_buf[2] == 0xAA) || (esd_buf[3] != 0xAA))
            {
                // IC works abnormally..
                u8 chk_buf[4] = {0x80, 0x40};

                gtp_i2c_read_no_rst(ts->client, chk_buf, 4);
               	VIVO_TS_LOG_DBG("[Check]0x8040 = 0x%02X, 0x8041 = 0x%02X", chk_buf[2], chk_buf[3]);

                if ((chk_buf[2] == 0xAA) || (chk_buf[3] != 0xAA))
                {
                    i = 3;
                    break;
                }
                else
                {
                    continue;
                }
            }
            else
            {
                // IC works normally, Write 0x8040 0xAA, feed the dog
                esd_buf[2] = 0xAA; 
                gtp_i2c_write_no_rst(ts->client, esd_buf, 3);
                break;
            }
        }
    }
    if (i >= 3)
    {
            VIVO_TS_LOG_ERR("IC working abnormally! Process reset guitar.");
            esd_buf[0] = 0x42;
            esd_buf[1] = 0x26;
            esd_buf[2] = 0x01;
            esd_buf[3] = 0x01;
            esd_buf[4] = 0x01;
            gtp_i2c_write_no_rst(ts->client, esd_buf, 5);
            msleep(50);
		/*vivo liuyunfeng add for ESD test abnoramll, Reset ic for slove i2c error*/
		gtp_irq_disable(ts);
		gpio_direction_output(ts->pdata->reset_gpio, 0);
		gpio_direction_output(ts->pdata->irq_gpio, 0);
		ret = goodix_power_off(ts);
		if (ret) {
		 	VIVO_TS_LOG_ERR("GTP power off failed\n");
		}
		msleep(40);
		ret = goodix_power_on(ts);
		if (ret) {
		 	VIVO_TS_LOG_ERR("GTP power on failed\n");
		}
            gtp_reset_guitar(ts->client, 50);
            msleep(50);
            gtp_send_cfg(ts->client);
	     gtp_irq_enable(ts);
    }

    if(!ts->gtp_is_suspend)
    {
        queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, ts->clk_tick_cnt);
    }
    else
    {
        VIVO_TS_LOG_INF("Esd suspended!");
    }
    return;
}
//#endif

static const struct i2c_device_id goodix_ts_id[] = {
    { GTP_I2C_NAME, 0 },
    { }
};
static struct of_device_id goodix_match_table[] = {
	{ .compatible = "goodix,gt970", },
	{ },
};

#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
static const struct dev_pm_ops goodix_ts_dev_pm_ops = {
	.suspend = goodix_ts_suspend,
	.resume = goodix_ts_resume,
};
#else
static const struct dev_pm_ops goodix_ts_dev_pm_ops = {
};
#endif

static struct i2c_driver goodix_ts_driver = {
    .probe      = goodix_ts_probe,
    .remove     = goodix_ts_remove,
#ifdef CONFIG_HAS_EARLYSUSPEND
    .suspend    = goodix_ts_early_suspend,
    .resume     = goodix_ts_late_resume,
#endif
    .id_table   = goodix_ts_id,
    .driver = {
        .name     = GTP_I2C_NAME,
        .owner    = THIS_MODULE,
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
    s32 ret;

	if(!vivo_touchscreen_test_ic_in_use(GTP_I2C_NAME))
	{
	    return 0;
	}
    
	if(is_atboot == 1){
		return 0;	
	}
	
    GTP_DEBUG_FUNC();

    VIVO_TS_LOG_INF("GTP driver installing...");
    goodix_wq = create_singlethread_workqueue("goodix_wq");
    if (!goodix_wq)
    {
        VIVO_TS_LOG_ERR("Creat workqueue failed.");
        return -ENOMEM;
    }
//#if GTP_ESD_PROTECT
	if(vivo_touchscreen_is_support(FS_ANTI_ESD)) {
	    INIT_DELAYED_WORK(&gtp_esd_check_work, gtp_esd_check_func);
	    gtp_esd_check_workqueue = create_workqueue("gtp_esd_check");
	}
//#endif

    ret = i2c_add_driver(&goodix_ts_driver);

    return ret; 
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
    VIVO_TS_LOG_INF("GTP driver exited.");
    i2c_del_driver(&goodix_ts_driver);
    if (goodix_wq)
    {
        destroy_workqueue(goodix_wq);
    }
}

late_initcall(goodix_ts_init);
module_exit(goodix_ts_exit);

MODULE_DESCRIPTION("GTP Series Driver");
MODULE_LICENSE("GPL");
