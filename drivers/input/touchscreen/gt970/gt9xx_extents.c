/* drivers/input/touchscreen/gtp_extents.c
 *
 * 2010 - 2014 Goodix Technology.
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
 * Version: 1.0   
 * Revision Record: 
 *      V1.0:  first release. 2014/10/15.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/input.h>

#include <asm/uaccess.h>
#include <linux/proc_fs.h>	/*proc */

#include <asm/ioctl.h>
#include "gt9xx.h"
#include <linux/vivo_touchscreen_common.h>
#include <linux/vivo_touchscreen_config.h>

#define GESTURE_NODE "goodix_gesture"
#define GESTURE_MAX_POINT_COUNT    64
#define GTP_REG_WAKEUP_GESTURE     		0x814B
#define GTP_REG_WAKEUP_GESTURE_DETAIL	0x9420

#pragma pack(1)
typedef struct {
	u8 ic_msg[6];		/*from the first byte */
	u8 gestures[4];
	u8 data[3 + GESTURE_MAX_POINT_COUNT * 4 + 80];	/*80 bytes for extra data */
} st_gesture_data;
#pragma pack()

#define SETBIT(longlong, bit)   (longlong[bit/8] |=  (1 << bit%8))
#define CLEARBIT(longlong, bit) (longlong[bit/8] &=(~(1 << bit%8)))
#define QUERYBIT(longlong, bit) (!!(longlong[bit/8] & (1 << bit%8)))

int gesture_enabled = 0;
//DOZE_T gesture_doze_status = DOZE_DISABLED;
extern DOZE_T doze_status ;

static u8 gestures_flag[32];
static st_gesture_data gesture_data;
static struct mutex gesture_data_mutex;

extern struct i2c_client * i2c_connect_client;
extern s8 	gtp_enter_doze(struct goodix_ts_data *ts);
extern void gtp_esd_switch(struct i2c_client *client, s32 on);
extern void gtp_reset_guitar(struct i2c_client *client, s32 ms);
extern void gtp_irq_disable(struct goodix_ts_data *ts);
extern void gtp_irq_enable(struct goodix_ts_data *ts);
s8 gtp_enter_sleep(struct goodix_ts_data * ts);
extern s32 gtp_i2c_write(struct i2c_client *client,u8 *buf,s32 len);


void gesture_clear_wakeup_data(void);

static s32 i2c_write_bytes(u16 addr, u8 *buf, s32 len)
{
    s32 ret = 0;
    s32 retry = 0;
	struct i2c_msg msg;
	u8 *i2c_buf = NULL;

	i2c_buf = (u8 *)kmalloc(GTP_ADDR_LENGTH + len, GFP_KERNEL);
	if (i2c_buf == NULL) {
		return -ENOMEM;
	}
	
	i2c_buf[0] = (u8)(addr >> 8);
	i2c_buf[1] = (u8)(addr & 0xFF);
	memcpy(i2c_buf + 2, buf, len);

	msg.flags = !I2C_M_RD;
	msg.addr = i2c_connect_client->addr;
	msg.len  = GTP_ADDR_LENGTH + len;
	msg.buf  = i2c_buf;
	
    while (retry++ < 5) {
		ret = i2c_transfer(i2c_connect_client->adapter, &msg, 1);
        if (ret == 1){
                break;
        }  
		VIVO_TS_LOG_ERR("I2C Write: 0x%04X, %d bytes failed, errcode: %d! [%d].", addr, len, ret, retry);
    }
	if (i2c_buf != NULL) {
		kfree(i2c_buf);
	}
	
	if (retry >= 5) {
		VIVO_TS_LOG_ERR("I2c transfer retry timeout.");
		return -1;
	} else {
    	return 1;
	}
}

static s32 i2c_read_bytes(u16 addr, u8 *buf, s32 len)
{
    s32 ret = 0;
    s32 retry = 0;
	u8 i2c_buf[GTP_ADDR_LENGTH];
	struct i2c_msg msg[2] = {
		{	.flags = !I2C_M_RD,
			.addr  = i2c_connect_client->addr,
			.buf   = i2c_buf,
			.len   = GTP_ADDR_LENGTH},
			{
			.flags = I2C_M_RD,
			.addr  = i2c_connect_client->addr,
			.buf   = buf,
			.len   = len,
		}
	};

    i2c_buf[0] = (u8)(addr >> 8);
    i2c_buf[1] = (u8)(addr & 0xFF);
	
    while (retry++ < 5) {
		ret = i2c_transfer(i2c_connect_client->adapter, msg, 2);
        if (ret == 2) {
        	break;
        }
    }
	   
   	if (retry >= 5) {
		VIVO_TS_LOG_ERR("I2c retry timeout, I2C read 0x%04X %d bytes failed!", addr, len);   
		return -1;
	} else {
		return 0;
	}
}

static ssize_t gtp_gesture_data_read(struct file *file, char __user * page, size_t size, loff_t * ppos)
{
	s32 ret = -1;
	VIVO_TS_LOG_DBG("visit gtp_gesture_data_read. ppos:%d", (int)*ppos);
	if (*ppos) {
		return 0;
	}
	if (size == 4) {
		ret = copy_to_user(((u8 __user *) page), "GT1X", 4);
		return 4;
	}
	ret = simple_read_from_buffer(page, size, ppos, &gesture_data, sizeof(gesture_data));

	VIVO_TS_LOG_DBG("Got the gesture data.");
	return ret;
}

static ssize_t gtp_gesture_data_write(struct file *filp, const char __user * buff, size_t len, loff_t * off)
{
	s32 ret = 0;

	GTP_DEBUG_FUNC();

	ret = copy_from_user(&gesture_enabled, buff, 1);
	if (ret) {
		VIVO_TS_LOG_ERR("copy_from_user failed.");
		return -EPERM;
	}

	VIVO_TS_LOG_DBG("gesture enabled:%x, ret:%d", gesture_enabled, ret);

	return len;
}

static u8 is_all_dead(u8 * longlong, s32 size)
{
	int i = 0;
	u8 sum = 0;

	for (i = 0; i < size; i++) {
		sum |= longlong[i];
	}

	return !sum;
}

s8 gtp_enter_doze(struct goodix_ts_data *ts)
{
    s8 ret = -1;
    s8 retry = 0;
    u8 i2c_control_buf[1] = {8};

    GTP_DEBUG_FUNC();

    VIVO_TS_LOG_DBG("Entering doze mode.");
	gesture_clear_wakeup_data();
    while(retry++ < 5)
    {
        ret = i2c_write_bytes(0x8046, i2c_control_buf, 1);
        if (ret < 0)
        {
            VIVO_TS_LOG_DBG("failed to set doze flag into 0x8046, %d", retry);
            continue;
        }

        ret = i2c_write_bytes(0x8040, i2c_control_buf, 1);
        if (ret > 0)
        {
            //gesture_doze_status = DOZE_ENABLED;
            doze_status = DOZE_ENABLED;
            VIVO_TS_LOG_INF("Gesture mode enabled.");
            return ret;
        }
        msleep(10);
    }
    VIVO_TS_LOG_ERR("GTP send doze cmd failed.");
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
s8 gtp_enter_sleep(struct goodix_ts_data * ts)
{
    s8 ret = -1;
    s8 retry = 0;
    u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 5};

    GTP_DEBUG_FUNC();
    gesture_clear_wakeup_data();

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

s32 gesture_event_handler(struct input_dev * dev)
{
	u8 doze_buf[4] = { 0 };
	s32 ret = -1;
	int len, extra_len;

	//if (DOZE_ENABLED == gesture_doze_status) {
	if (DOZE_ENABLED == doze_status){
		ret = i2c_read_bytes(GTP_REG_WAKEUP_GESTURE, doze_buf, 4);
		VIVO_TS_LOG_DBG("0x%x = 0x%02X,0x%02X,0x%02X,0x%02X", GTP_REG_WAKEUP_GESTURE, doze_buf[0], doze_buf[1], doze_buf[2], doze_buf[3]);
		//GTP_DEBUG("0x%x = 0x%02X,0x%02X", GTP_REG_WAKEUP_GESTURE, doze_buf[0], doze_buf[1]);
		if (ret == 0 && doze_buf[0] != 0) {			
			if (!QUERYBIT(gestures_flag, doze_buf[0])) {
				VIVO_TS_LOG_INF("Sorry, this gesture has been disabled.");
				doze_buf[0] = 0x00;
				i2c_write_bytes(GTP_REG_WAKEUP_GESTURE, doze_buf, 1);
				return 0;
			}

			mutex_lock(&gesture_data_mutex);
			len = (doze_buf[1] & 0x7F);
			if (len > GESTURE_MAX_POINT_COUNT) {
				VIVO_TS_LOG_ERR("Gesture contain too many points!(%d)", len);
				len = GESTURE_MAX_POINT_COUNT;
			}
			if (len > 0) {
				ret = i2c_read_bytes(GTP_REG_WAKEUP_GESTURE_DETAIL, &gesture_data.data[4], len * 4);
				if (ret < 0) {
					VIVO_TS_LOG_DBG("Read gesture data failed.");
					mutex_unlock(&gesture_data_mutex);
					return 0;
				}
			}

			extra_len = (doze_buf[1] & 0x80) ? doze_buf[3] : 0;
			if (extra_len > 80) {
				VIVO_TS_LOG_ERR("Gesture contain too many extra data!(%d)", extra_len);
				extra_len = 80;
			}
			if (extra_len > 0) {
				ret = i2c_read_bytes(GTP_REG_WAKEUP_GESTURE + 4, &gesture_data.data[4 + len * 4], extra_len);
				if (ret < 0) {
					VIVO_TS_LOG_DBG("Read extra gesture data failed.");
					mutex_unlock(&gesture_data_mutex);
					return 0;
				}
			}
			doze_buf[2] &= ~0x30;
			doze_buf[2] |= (extra_len > 0? 0x20 : 0x10);
			gesture_data.data[0] = doze_buf[0];	//gesture type
			gesture_data.data[1] = len;	// gesture points number
			gesture_data.data[2] = doze_buf[2];
			gesture_data.data[3] = extra_len;
			mutex_unlock(&gesture_data_mutex);

			VIVO_TS_LOG_DBG("Gesture: 0x%02X, points: %d", doze_buf[0], doze_buf[1]);

			doze_buf[0] = 0;
			i2c_write_bytes(GTP_REG_WAKEUP_GESTURE, doze_buf, 1);
			VIVO_TS_LOG_DBG("++++Gesture++++");
			input_report_key(dev, KEY_GESTURE, 1);
			input_sync(dev);
			input_report_key(dev, KEY_GESTURE, 0);
			input_sync(dev);
			VIVO_TS_LOG_DBG("----Gesture----");
			return 1;
		}
		return 0;
	}
	return -1;
}

void gesture_clear_wakeup_data(void)
{
	mutex_lock(&gesture_data_mutex);
	memset(gesture_data.data, 0, 4);
	mutex_unlock(&gesture_data_mutex);
}
//#endif // GTP_GESTURE_WAKEUP

#define GOODIX_MAGIC_NUMBER        'G'
#define NEGLECT_SIZE_MASK           (~(_IOC_SIZEMASK << _IOC_SIZESHIFT))

#define GESTURE_ENABLE_TOTALLY      _IO(GOODIX_MAGIC_NUMBER, 1)	// 1
#define GESTURE_DISABLE_TOTALLY     _IO(GOODIX_MAGIC_NUMBER, 2)
#define GESTURE_ENABLE_PARTLY       _IO(GOODIX_MAGIC_NUMBER, 3)
#define GESTURE_DISABLE_PARTLY      _IO(GOODIX_MAGIC_NUMBER, 4)
//#define SET_ENABLED_GESTURE         (_IOW(GOODIX_MAGIC_NUMBER, 5, u8) & NEGLECT_SIZE_MASK)
#define GESTURE_DATA_OBTAIN         (_IOR(GOODIX_MAGIC_NUMBER, 6, u8) & NEGLECT_SIZE_MASK)
#define GESTURE_DATA_ERASE          _IO(GOODIX_MAGIC_NUMBER, 7)

#define IO_IIC_READ                  (_IOR(GOODIX_MAGIC_NUMBER, 100, u8) & NEGLECT_SIZE_MASK)
#define IO_IIC_WRITE                 (_IOW(GOODIX_MAGIC_NUMBER, 101, u8) & NEGLECT_SIZE_MASK)
#define IO_RESET_GUITAR              _IO(GOODIX_MAGIC_NUMBER, 102)
#define IO_DISABLE_IRQ               _IO(GOODIX_MAGIC_NUMBER, 103)
#define IO_ENABLE_IRQ                _IO(GOODIX_MAGIC_NUMBER, 104)
#define IO_GET_VERISON               (_IOR(GOODIX_MAGIC_NUMBER, 110, u8) & NEGLECT_SIZE_MASK)
#define IO_PRINT                     (_IOW(GOODIX_MAGIC_NUMBER, 111, u8) & NEGLECT_SIZE_MASK)
#define IO_VERSION                   "V1.0-20141015"

#define CMD_HEAD_LENGTH             20

static s32 io_iic_read(u8 * data, void __user * arg)
{
	s32 err = -1;
	s32 data_length = 0;
	u16 addr = 0;

	err = copy_from_user(data, arg, CMD_HEAD_LENGTH);
	if (err) {
		VIVO_TS_LOG_DBG("Can't access the memory.");
		return err;
	}

	addr = data[0] << 8 | data[1];
	data_length = data[2] << 8 | data[3];

	err = i2c_read_bytes(addr, &data[CMD_HEAD_LENGTH], data_length);
	if (!err) {
		err = copy_to_user(&((u8 __user *) arg)[CMD_HEAD_LENGTH], &data[CMD_HEAD_LENGTH], data_length);
		if (err) {
			VIVO_TS_LOG_ERR("ERROR when copy to user.[addr: %04x], [read length:%d]", addr, data_length);
			return err;
		}
		err = CMD_HEAD_LENGTH + data_length;
	}
	VIVO_TS_LOG_DBG("IIC_READ.addr:0x%4x, length:%d, ret:%d", addr, data_length, err);
	GTP_DEBUG_ARRAY((&data[CMD_HEAD_LENGTH]), data_length);

	return err;
}

static s32 io_iic_write(u8 * data)
{
	s32 err = -1;
	s32 data_length = 0;
	u16 addr = 0;

	addr = data[0] << 8 | data[1];
	data_length = data[2] << 8 | data[3];

	err = i2c_write_bytes(addr, &data[CMD_HEAD_LENGTH], data_length);
	if (!err) {
		err = CMD_HEAD_LENGTH + data_length;
	}

	VIVO_TS_LOG_DBG("IIC_WRITE.addr:0x%4x, length:%d, ret:%d", addr, data_length, err);
	GTP_DEBUG_ARRAY((&data[CMD_HEAD_LENGTH]), data_length);
	return err;
}

//@return, 0:operate successfully
//         > 0: the length of memory size ioctl has accessed,
//         error otherwise.
static long gtp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	u32 value = 0;
	s32 ret = 0;		//the initial value must be 0
	u8 *data = NULL;

	VIVO_TS_LOG_DBG("IOCTL CMD:%x", cmd);
	VIVO_TS_LOG_DBG("command:%d, length:%d, rw:%s", _IOC_NR(cmd), _IOC_SIZE(cmd), (_IOC_DIR(cmd) & _IOC_READ) ? "read" : (_IOC_DIR(cmd) & _IOC_WRITE) ? "write" : "-");

	if (_IOC_DIR(cmd)) {
		s32 err = -1;
		s32 data_length = _IOC_SIZE(cmd);
		data = (u8 *) kzalloc(data_length, GFP_KERNEL);
		memset(data, 0, data_length);

		if (_IOC_DIR(cmd) & _IOC_WRITE) {
			err = copy_from_user(data, (void __user *)arg, data_length);
			if (err) {
				VIVO_TS_LOG_DBG("Can't access the memory.");
				kfree(data);
				return -1;
			}
		}
	} else {
		value = (u32) arg;
	}

	switch (cmd & NEGLECT_SIZE_MASK) {
	case IO_GET_VERISON:
		if ((u8 __user *) arg) {
			ret = copy_to_user(((u8 __user *) arg), IO_VERSION, sizeof(IO_VERSION));
			if (!ret) {
				ret = sizeof(IO_VERSION);
			}
			VIVO_TS_LOG_INF("%s", IO_VERSION);
		}
		break;
	case IO_IIC_READ:
		ret = io_iic_read(data, (void __user *)arg);
		break;

	case IO_IIC_WRITE://write
		ret = io_iic_write(data);
		break;

	case IO_RESET_GUITAR: 
		gtp_reset_guitar(i2c_connect_client, 10);
		break;

	case IO_DISABLE_IRQ: {
		struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
		gtp_irq_disable(ts);
#if GTP_ESD_PROTECT
		gtp_esd_switch(i2c_connect_client, SWITCH_OFF);
#endif
		break;
		}
	case IO_ENABLE_IRQ: {
			struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
			gtp_irq_enable(ts);
		}
#if GTP_ESD_PROTECT
		gtp_esd_switch(i2c_connect_client ,SWITCH_ON);
#endif
		break;

		//print a string to syc log messages between application and kernel.
	case IO_PRINT:
		if (data)
			VIVO_TS_LOG_INF("%s", (char *)data);
		break;

//#if defined(BBK_DCLICK_WAKE)//GTP_GESTURE_WAKEUP
	if(vivo_touchscreen_is_support(FS_DCLICK_WAKE)) {
		case GESTURE_ENABLE_TOTALLY:
			VIVO_TS_LOG_DBG("ENABLE_GESTURE_TOTALLY");
			gesture_enabled = (is_all_dead(gestures_flag, sizeof(gestures_flag)) ? 0 : 1);
			break;

		case GESTURE_DISABLE_TOTALLY:
			VIVO_TS_LOG_DBG("DISABLE_GESTURE_TOTALLY");
			gesture_enabled = 0;
			break;

		case GESTURE_ENABLE_PARTLY://enable
			SETBIT(gestures_flag, (u8) value);
			gesture_enabled = 1;
			VIVO_TS_LOG_DBG("ENABLE_GESTURE_PARTLY, gesture = 0x%02X, gesture_enabled = %d", value, gesture_enabled);
			break;

		case GESTURE_DISABLE_PARTLY:
			ret = QUERYBIT(gestures_flag, (u8) value);
			if (!ret) {
				break;
			}
			CLEARBIT(gestures_flag, (u8) value);
			if (is_all_dead(gestures_flag, sizeof(gestures_flag))) {
				gesture_enabled = 0;
			}
			VIVO_TS_LOG_DBG("DISABLE_GESTURE_PARTLY, gesture = 0x%02X, gesture_enabled = %d", value, gesture_enabled);
			break;

		case GESTURE_DATA_OBTAIN:
			VIVO_TS_LOG_DBG("OBTAIN_GESTURE_DATA");

			mutex_lock(&gesture_data_mutex);
			if (gesture_data.data[1] > GESTURE_MAX_POINT_COUNT) {
				gesture_data.data[1] = GESTURE_MAX_POINT_COUNT;
			}
			if (gesture_data.data[3] > 80) {
				gesture_data.data[3] = 80;
			}
			ret = copy_to_user(((u8 __user *) arg), &gesture_data.data, 4 + gesture_data.data[1] * 4 + gesture_data.data[3]);
			mutex_unlock(&gesture_data_mutex);
			if (ret) {
				VIVO_TS_LOG_ERR("ERROR when copy gesture data to user.");
			} else {
				ret = 4 + gesture_data.data[1] * 4 + gesture_data.data[3];
			}
			break;

		case GESTURE_DATA_ERASE:
			VIVO_TS_LOG_DBG("ERASE_GESTURE_DATA");
			gesture_clear_wakeup_data();
			break;
		}
//#endif // GTP_GESTURE_WAKEUP

	default:
		VIVO_TS_LOG_INF("Unknown cmd.");
		ret = -1;
		break;
	}

	if (data != NULL) {
		kfree(data);
	}

	return ret;
}
/*vivo liuyunfeng add for compatible 32bit system start*/
#ifdef CONFIG_COMPAT
static long gtp_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *arg32 = compat_ptr(arg);

	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;

	return file->f_op->unlocked_ioctl(file, cmd, (unsigned long)arg32);
}
#endif
/*vivo liuyunfeng add end*/
static int gtp_gesture_open(struct inode *node, struct file *flip) {
        VIVO_TS_LOG_DBG("gesture node is opened.");
        return 0;
}

static int gtp_gesture_release(struct inode *node, struct file *filp) {
        VIVO_TS_LOG_DBG("gesture node is closed.");
        return 0;  
}
static const struct file_operations gtp_fops = {
	.owner = THIS_MODULE,
	.open = gtp_gesture_open,
	.release = gtp_gesture_release,
//#if defined(BBK_DCLICK_WAKE)//GTP_GESTURE_WAKEUP
	.read = gtp_gesture_data_read,
	.write = gtp_gesture_data_write,
//#endif
	.unlocked_ioctl = gtp_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = gtp_compat_ioctl,
#endif
};

s32 gtp_init_node(void)
{
//#if defined(BBK_DCLICK_WAKE) //GTP_GESTURE_WAKEUP
if(vivo_touchscreen_is_support(FS_DCLICK_WAKE)) {
	struct proc_dir_entry *proc_entry = NULL;
	mutex_init(&gesture_data_mutex);
	memset(gestures_flag, 0, sizeof(gestures_flag));
	memset((u8 *) & gesture_data, 0, sizeof(st_gesture_data));

	proc_entry = proc_create(GESTURE_NODE, 0666, NULL, &gtp_fops);
	if (proc_entry == NULL) {
		VIVO_TS_LOG_ERR("Couldn't create proc entry[GESTURE_NODE]!");
		return -1;
	} else {
		VIVO_TS_LOG_INF("Create proc entry[GESTURE_NODE] success!");
	}
//#endif
}
	return 0;
}

void gtp_deinit_node(void)
{
//#if defined(BBK_DCLICK_WAKE)//GTP_GESTURE_WAKEUP
	remove_proc_entry(GESTURE_NODE, NULL);
//#endif

}
