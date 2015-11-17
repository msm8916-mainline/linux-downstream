/* drivers/input/touchscreen/ektf.c - ELAN EKTF verions of driver
*
* Copyright (C) 2011 Elan Microelectronics Corporation.
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
* 2014/0/28: The first release, version 0x0006
*             Integrated 2 ,5 ,and 10 fingers driver code together and
*             auto-mapping resolution.
*             Please change following parameters
*                 1. For 5 fingers protocol, please enable ELAN_PROTOCOL.
*                    The packet size is 18 or 24 bytes.
*                 2. For 10 fingers, please enable both ELAN_PROTOCOL and ELAN_TEN_FINGERS.
*                    The packet size is 40 or 4+40+40+40 (Buffer mode) bytes.
*                 3. Please enable the ELAN_BUTTON configuraton to support button.
*		   4. For ektf3k serial, Add Re-Calibration Machanism
*                    So, please enable the define of RE_CALIBRATION.
*
*
*/

/* The ELAN_PROTOCOL support normanl packet format */
#define ELAN_PROTOCOL
//#define ELAN_BUFFER_MODE
#define ELAN_TEN_FINGERS   /* Can not be use to auto-resolution mapping */
//#define ELAN_BUTTON
//#define RE_CALIBRATION   /* The Re-Calibration was designed for ektf3k serial. */
#define ELAN_2WIREICE
#define ELAN_POWER_SOURCE
#define ELAN_RESUME_RST
#define DEVICE_NAME "elan_ktf"
#define EKTF3K_FLASH
//#define PROTOCOL_A    /* multi-touch protocol  */
#define PROTOCOL_B    /* Default: PROTOCOL B */
#define GESTURE_MODE
//#define BURN_BY_8BYTE

#include <linux/module.h>
#include <linux/input.h>
#ifdef PROTOCOL_B
#include <linux/input/mt.h>
#endif
#ifdef PROTOCOL_A
#include <linux/input.h>
#endif
#include <linux/interrupt.h>
//#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <linux/debugfs.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
// for linux 2.6.36.3
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <asm/ioctl.h>
#include <linux/switch.h>
#include <linux/proc_fs.h>
#include <linux/firmware.h>
#include <linux/wakelock.h>
//#include <linux/elan_ktf.h>
#include <linux/kthread.h>
#include "ekth3260.h"
//#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/regulator/machine.h>
// #include <linux/regulator/krait-regulator.h>
//#endif

#include <linux/extcon.h>
// add by leo for early-suspend ++
#ifdef CONFIG_PM_SCREEN_STATE_NOTIFIER
#include <linux/suspend.h>
#endif
// add by leo for early-suspend --

#ifdef ELAN_TEN_FINGERS
#define FINGER_NUM 10
#define PACKET_SIZE		36		/* support 10 fingers packet for nexus7 55 */
#else
#define FINGER_NUM 5
//#define PACKET_SIZE		8 		/* support 2 fingers packet  */
#define PACKET_SIZE		18			/* support 5 fingers packet  */
#endif

#define PWR_STATE_DEEP_SLEEP	0
#define PWR_STATE_NORMAL		1
#define PWR_STATE_MASK			BIT(3)

#define CMD_S_PKT		0x52
#define CMD_R_PKT		0x53
#define CMD_W_PKT		0x54
#define RESET_PKT		0x77
#define CALIB_PKT		0x66

#define HELLO_PKT		0x55
#define TWO_FINGERS_PKT		0x5A
#define FIVE_FINGERS_PKT	0x5D
#define MTK_FINGERS_PKT		0x6D
#define TEN_FINGERS_PKT		0x62
#define BUFFER_PKT		0x63
#define BUFFER55_PKT		0x66
#define GESTURE_PKT		0x88
#define HSYNC_PKT		0x72

#define FW_UPDATE_IN_DRIVER // add by leo for FW update in driver

// Reset pin need to be modified by customer
#define SYSTEM_RESET_PIN_SR 12	// nexus7_grouper TEGRA_GPIO_PH6: 62, nexus7_flo 31

//Add these Define
#define IAP_PORTION
#define PAGERETRY  30
#define IAPRESTART 5


// For Firmware Update
#define ELAN_IOCTLID	0xD0
#define IOCTL_I2C_SLAVE	_IOW(ELAN_IOCTLID,  1, int)
#define IOCTL_MAJOR_FW_VER  _IOR(ELAN_IOCTLID, 2, int)
#define IOCTL_MINOR_FW_VER  _IOR(ELAN_IOCTLID, 3, int)
#define IOCTL_RESET  _IOR(ELAN_IOCTLID, 4, int)
#define IOCTL_IAP_MODE_LOCK  _IOR(ELAN_IOCTLID, 5, int)
#define IOCTL_CHECK_RECOVERY_MODE  _IOR(ELAN_IOCTLID, 6, int)
#define IOCTL_FW_VER  _IOR(ELAN_IOCTLID, 7, int)
#define IOCTL_X_RESOLUTION  _IOR(ELAN_IOCTLID, 8, int)
#define IOCTL_Y_RESOLUTION  _IOR(ELAN_IOCTLID, 9, int)
#define IOCTL_FW_ID  _IOR(ELAN_IOCTLID, 10, int)
#define IOCTL_ROUGH_CALIBRATE  _IOR(ELAN_IOCTLID, 11, int)
#define IOCTL_IAP_MODE_UNLOCK  _IOR(ELAN_IOCTLID, 12, int)
#define IOCTL_I2C_INT  _IOR(ELAN_IOCTLID, 13, int)
#define IOCTL_RESUME  _IOR(ELAN_IOCTLID, 14, int)
#define IOCTL_POWER_LOCK  _IOR(ELAN_IOCTLID, 15, int)
#define IOCTL_POWER_UNLOCK  _IOR(ELAN_IOCTLID, 16, int)
#define IOCTL_FW_UPDATE  _IOR(ELAN_IOCTLID, 17, int)
#define IOCTL_BC_VER  _IOR(ELAN_IOCTLID, 18, int)
#define IOCTL_2WIREICE  _IOR(ELAN_IOCTLID, 19, int)

#define CUSTOMER_IOCTLID	0xA0
#define IOCTL_CIRCUIT_CHECK  _IOR(CUSTOMER_IOCTLID, 1, int)
#define IOCTL_GET_UPDATE_PROGREE	_IOR(CUSTOMER_IOCTLID,  2, int)

#define ELAN_FW_FILENAME  "ElanFW.fw"
/* Debug levels */
#define NO_DEBUG       0
#define DEBUG_ERROR  1
#define DEBUG_INFO     2
#define DEBUG_MESSAGES 5
#define DEBUG_TRACE   10

//#define ESD_CHECK
#ifdef ESD_CHECK
int live_state = 1;
#endif

#define SYSFS_MAX_LEN 100
static unsigned int gPrint_point = 0;
static int debug = DEBUG_INFO;
#define touch_debug(level, ...) \
	do { \
		if (debug >= (level)) \
			printk("[elan]:" __VA_ARGS__); \
	} while (0)


uint8_t RECOVERY = 0x00;
int FW_VERSION = 0x00;
int X_RESOLUTION = 1472;
int Y_RESOLUTION = 2368;
int FW_ID = 0x00;
int work_lock = 0x00;
int power_lock = 0x00;
int circuit_ver = 0x01;
/*++++i2c transfer start+++++++*/
int file_fops_addr = 0x10;
/*++++i2c transfer end+++++++*/
struct mutex ktf_mutex;
int button_state = 0;
static int hsync_count = 0;

#ifdef GESTURE_MODE
enum
{
	ELAN_GESTURE_E,
	ELAN_GESTURE_Z,
	ELAN_GESTURE_O,
	ELAN_GESTURE_V,
	ELAN_GESTURE_N,
	ELAN_GESTURE_W,
	ELAN_GESTURE_M,
	ELAN_GESTURE_C,
	ELAN_GESTURE_S,
	ELAN_GESTURE_UP,
	ELAN_GESTURE_DOWN,
	ELAN_GESTURE_LEFT,
	ELAN_GESTURE_RIGHT,
	ELAN_GESTURE_DOUBLE_CLICK = 0x0f,
	ELAN_GESTURE_COUNT,
};

int gesture_table[ELAN_GESTURE_COUNT] = {0};

int gesture_enable = 0;	// 0:sleep mode, 1:gesture mode
#endif

#ifdef IAP_PORTION
uint8_t ic_status = 0x00;	//0:OK 1:master fail 2:slave fail
int update_progree = 0;
uint8_t I2C_DATA[3] = {0x10, 0x20, 0x21};/*I2C devices address*/
int is_OldBootCode = 0; // 0:new 1:old
//static unsigned char firmware[52800];
static unsigned char firmware_outside[52800];

// add by leo ++
#define I2C_FAILD_RETRY 0
#define HELLOW_IN_GESTURE_WORKAROUND 1
#define REPORT_GESTURE_DCLICK_POINT 0
#define TOUCH_SYSTRACE_TEST 1

static int ekth3260_touch_status = 0;
static int fail_count = 0;

//extern unsigned int entry_mode;
// add by leo --

/*The newest firmware, if update must be changed here*/
static uint8_t file_fw_data_CPT[] =
{
#include "fw_data_0x3037_0x554c_CPT.i"
};

// add by leo for AUO panel ++
static uint8_t file_fw_data_AUO[] =
{
#include "fw_data_0x3038_0x551f_AUO.i"
};
// add by leo for AUO panel --

enum
{
	PageSize		= 132,
	ACK_Fail		= 0x00,
	ACK_OK			= 0xAA,
	ACK_REWRITE		= 0x55,
};

// add by leo for AUO panel ++
int	PageNum		  = sizeof(file_fw_data_CPT) / 132; /*for ektf2xxx/3xxx serial, the page number is 249/351*/
// add by leo for AUO panel --

enum
{
	E_FD			= -1,
};
#endif
//#define _ENABLE_DBG_LEVEL

#ifdef _ENABLE_DBG_LEVEL
#define PROC_FS_NAME    "ektf_dbg"
#define PROC_FS_MAX_LEN 8
static struct proc_dir_entry *dbgProcFile;
#endif

struct elan_ktf_ts_data
{
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct workqueue_struct *elan_irq_wq;
	struct work_struct irq_work;
	struct workqueue_struct *elan_wq;
	struct work_struct work;
	struct workqueue_struct *elan_ac_wq;
	struct delayed_work ac_delay_work;
	struct delayed_work sensor_delay_work;
#ifdef ESD_CHECK
	struct delayed_work check_work; //0430
#endif
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	int intr_gpio;
	int rst_gpio;
	int power_gpio;
	int lcmid0_gpio;
	int lcmid2_gpio;
	// Firmware Information
	int fw_ver;
	int fw_id;
	int bc_ver;
	int x_resolution;
	int y_resolution;
	// For Firmare Update
	struct miscdevice firmware;
	struct wake_lock wakelock;
	// add by leo ++
	struct regulator *vcc_i2c;
	struct regulator *vdd;
	struct switch_dev touch_sdev; // add by leo for switch device
	u8 gesture;
	u32 intr_gpio_flags;
	u32 rst_gpio_flags;
	u32 power_gpio_flags;
	u32 lcmid0_gpio_flags;
	u32 lcmid2_gpio_flags;
	int HW_ID;
	int LCM_ID0;
	int LCM_ID2;
	int fwupdate_result;
	int fwcheck_result;
	int system_suspend;
	// add by leo --
};

static struct elan_ktf_ts_data *private_ts;
static int __fw_packet_handler(struct i2c_client *client);
static int elan_ktf_ts_rough_calibrate(struct i2c_client *client);
#if 0
static int elan_ktf_ts_resume(struct i2c_client *client);
static int elan_ktf_ts_suspend(struct i2c_client *client, pm_message_t mesg); // mark by leo for early-suspend
#endif
void elan_ktf_ts_hw_reset(void);
static int __hello_packet_handler(struct i2c_client *client);

#ifdef IAP_PORTION
static int Update_FW_in_Driver(void *x);

#endif

#ifdef ELAN_2WIREICE
int elan_TWO_WIRE_ICE(struct i2c_client *client);
#endif

// add by leo ++
#define TOUCH_SDEV_NAME "touch"
#define TOUCH_MDEV_NAME	"touch_fw_update"

#define TOUCH_IOCTL_MAGIC 't'
#define TOUCH_IOCTL_FW_CHECK			_IOR(TOUCH_IOCTL_MAGIC, 1, int)
#define TOUCH_IOCTL_FW_UPDATE		_IOR(TOUCH_IOCTL_MAGIC, 2, int)
#define TOUCH_IOCTL_FORCE_FW_UPDATE	_IOR(TOUCH_IOCTL_MAGIC, 3, int)
#define TOUCH_IOCTL_READ_CMD			_IOR(TOUCH_IOCTL_MAGIC, 4, int)
#define TOUCH_IOCTL_WRITE_CMD			_IOW(TOUCH_IOCTL_MAGIC, 5, int)
#define TOUCH_IOCTL_GESTURE_CMD		_IOW(TOUCH_IOCTL_MAGIC, 6, int)

#define NO_NEED_TO_UPDATE	0
#define NEED_TO_UPDATE		1
#define FAIL			0
#define SUCCESS		1

#define eKTH3260_VTG_MIN_UV	2600000
#define eKTH3260_VTG_MAX_UV	3300000
#define eKTH3260_I2C_VTG_MIN_UV	1800000
#define eKTH3260_I2C_VTG_MAX_UV	1800000
#define eKTH3260_VIO_LOAD_MAX_UA	10000

#define EKTH3260_PROC_GESTURE_FILE		"eKTH3260_gesture"
#define EKTH3260_PROC_TP_DEBUG_FILE		"eKTH3260_debug"
#define EKTH3260_PROC_LOGTOOL_FILE		"touch_debug_log"

#define GESTURE_DOUBLE_CLICK 0x80
#define GESTURE_ENABLE 0x40
#define GESTURE_W 0x20
#define GESTURE_S 0x10
#define GESTURE_E 0x08
#define GESTURE_C 0x04
#define GESTURE_Z 0x02
#define GESTURE_V 0x01

// add by leo for early-suspend ++
#ifdef CONFIG_PM_SCREEN_STATE_NOTIFIER
static struct callback_data* callback_struct;
void eKTH3260_screen_chenged_listaner(const int state);
#endif
// add by leo for early-suspend --
static int eKTH3260_fw_version_check(void);

static struct proc_dir_entry *eKTH3260_proc_gesture_file = NULL;
static struct proc_dir_entry *eKTH3260_proc_tp_debug_file = NULL;
static int ekth3260_debug = 0;
static int ekth3260_update_outside = 0;


//user:3
//userdebug:2
//eng:1
extern int build_version;
extern int Read_HW_ID(void);
// add by leo --

static int __elan_ktf_ts_poll(struct i2c_client *client)
{
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
	int status = 0, retry = 20;

	do
	{
		status = gpio_get_value(ts->intr_gpio);
		if(status == 0) break;

		touch_debug(DEBUG_MESSAGES, "%s: status = %d\n", __func__, status);
		retry--;
		mdelay(50);
	}
	while(status == 1 && retry > 0);

	touch_debug(DEBUG_INFO, "%s: poll interrupt status %s\n",
	            __func__, status == 1 ? "high" : "low");
	return (status == 0 ? 0 : -ETIMEDOUT);
}

static int elan_ktf_ts_poll(struct i2c_client *client)
{
	return __elan_ktf_ts_poll(client);
}

/************************************
* Reset TP
*************************************/
void elan_ktf_ts_hw_reset()
{
	struct elan_ktf_ts_data *ts = private_ts;

	gpio_direction_output(ts->rst_gpio, 0);
	msleep(20);
	gpio_direction_output(ts->rst_gpio, 1);
}

int read_fw(char *fw_path)
{
	int pos = 0;
	struct file *firmware_fp;
	mm_segment_t oldfs;
	oldfs = get_fs();
	set_fs(KERNEL_DS);
	firmware_fp = filp_open(fw_path, O_RDONLY, S_IRUSR | S_IRGRP);
	if(PTR_ERR(firmware_fp) == -ENOENT)
	{
		touch_debug(DEBUG_ERROR, "open file error\n");
		return -1;
	}
	PageNum = 0;
	firmware_fp->f_pos = 0;
	for(pos = 0; pos < 500 * 132; pos += 132, PageNum++)
	{
		if(firmware_fp->f_op->read(firmware_fp, firmware_outside + pos,
		                           132, &firmware_fp->f_pos) != 132)
		{
			break;
		}
	}
	touch_debug(DEBUG_INFO, "%s: PageNUM %d, Ver %x %x, ID %x %x\n", __func__, PageNum, firmware_outside[34058], firmware_outside[34059], firmware_outside[34586], firmware_outside[34587]);
	set_fs(oldfs);
	return 0;
}

// For Firmware Update
int elan_iap_open(struct inode *inode, struct file *filp)
{
	touch_debug(DEBUG_MESSAGES, "[ELAN]into elan_iap_open\n");
	if(private_ts == NULL)  touch_debug(DEBUG_ERROR, "private_ts is NULL\n");

	return 0;
}

int elan_iap_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static ssize_t elan_iap_write(struct file *filp, const char *buff, size_t count, loff_t *offp)
{
	int ret;
	char *tmp;
	touch_debug(DEBUG_MESSAGES, "[ELAN]into elan_iap_write\n");
#if 0
	/*++++i2c transfer start+++++++*/
	struct i2c_adapter *adap = private_ts->client->adapter;
	struct i2c_msg msg;
	/*++++i2c transfer end+++++++*/
#endif
	if(count > 8192)
		count = 8192;

	tmp = kmalloc(count, GFP_KERNEL);

	if(tmp == NULL)
		return -ENOMEM;

	if(copy_from_user(tmp, buff, count))
	{
		return -EFAULT;
	}

	/*++++i2c transfer start+++++++*/
#if 0
	//down(&worklock);
	msg.addr = file_fops_addr;
	msg.flags = 0x00;// 0x00
	msg.len = count;
	msg.buf = (char *)tmp;
	//up(&worklock);
	ret = i2c_transfer(adap, &msg, 1);
#else

	ret = i2c_master_send(private_ts->client, tmp, count);
#endif
	/*++++i2c transfer end+++++++*/

	//if (ret != count) printk("ELAN i2c_master_send fail, ret=%d \n", ret);
	kfree(tmp);
	//return ret;
	return (ret == 1) ? count : ret;

}

ssize_t elan_iap_read(struct file *filp, char *buff, size_t count, loff_t *offp)
{
	char *tmp;
	int ret;
	long rc;
	touch_debug(DEBUG_MESSAGES, "[ELAN]into elan_iap_read\n");
#if 0
	/*++++i2c transfer start+++++++*/
	struct i2c_adapter *adap = private_ts->client->adapter;
	struct i2c_msg msg;
	/*++++i2c transfer end+++++++*/
#endif

	if(count > 8192)
		count = 8192;

	tmp = kmalloc(count, GFP_KERNEL);

	if(tmp == NULL)
		return -ENOMEM;
	/*++++i2c transfer start+++++++*/
#if 0
	//down(&worklock);
	msg.addr = file_fops_addr;
	//msg.flags |= I2C_M_RD;
	msg.flags = 0x00;
	msg.flags |= I2C_M_RD;
	msg.len = count;
	msg.buf = tmp;
	//up(&worklock);
	ret = i2c_transfer(adap, &msg, 1);
#else
	ret = i2c_master_recv(private_ts->client, tmp, count);
#endif
	/*++++i2c transfer end+++++++*/
	if(ret >= 0)
		rc = copy_to_user(buff, tmp, count);

	kfree(tmp);

	//return ret;
	return (ret == 1) ? count : ret;

}

static long elan_iap_ioctl(struct file *filp,    unsigned int cmd, unsigned long arg)
{

	int __user *ip = (int __user *)arg;

	touch_debug(DEBUG_MESSAGES, "[ELAN]into elan_iap_ioctl\n");
	touch_debug(DEBUG_MESSAGES, "cmd value %x\n", cmd);

	switch(cmd)
	{
		case IOCTL_I2C_SLAVE:
			private_ts->client->addr = (int __user)arg;
			//file_fops_addr = 0x10;
			break;
		case IOCTL_MAJOR_FW_VER:
			break;
		case IOCTL_MINOR_FW_VER:
			break;
		case IOCTL_RESET:
			// modify
			elan_ktf_ts_hw_reset();
			break;
		case IOCTL_IAP_MODE_LOCK:
			if(work_lock == 0)
			{
				work_lock = 1;
#ifdef ESD_CHECK //0430
				cancel_delayed_work_sync(&private_ts->check_work);
#endif
				disable_irq(private_ts->client->irq);
				cancel_work_sync(&private_ts->work);
			}
			break;
		case IOCTL_IAP_MODE_UNLOCK:
			if(work_lock == 1)
			{
				work_lock = 0;
#ifdef ESD_CHECK
				schedule_delayed_work(&private_ts->check_work, msecs_to_jiffies(2500)); //1218
#endif
				enable_irq(private_ts->client->irq);
			}
			break;
		case IOCTL_CHECK_RECOVERY_MODE:
			return RECOVERY;
			break;
		case IOCTL_FW_VER:
			__fw_packet_handler(private_ts->client);
			return FW_VERSION;
			break;
		case IOCTL_X_RESOLUTION:
			__fw_packet_handler(private_ts->client);
			return X_RESOLUTION;
			break;
		case IOCTL_Y_RESOLUTION:
			__fw_packet_handler(private_ts->client);
			return Y_RESOLUTION;
			break;
		case IOCTL_FW_ID:
			__fw_packet_handler(private_ts->client);
			return FW_ID;
			break;
		case IOCTL_ROUGH_CALIBRATE:
			return elan_ktf_ts_rough_calibrate(private_ts->client);
		case IOCTL_I2C_INT:
			put_user(gpio_get_value(private_ts->intr_gpio), ip);
			break;
		case IOCTL_RESUME:
			//elan_ktf_ts_resume(private_ts->client);
			break;
		case IOCTL_POWER_LOCK:
			power_lock = 1;
			break;
		case IOCTL_POWER_UNLOCK:
			power_lock = 0;
			break;
#ifdef IAP_PORTION
		case IOCTL_GET_UPDATE_PROGREE:
			update_progree = (int __user)arg;
			break;
		case IOCTL_FW_UPDATE:
			//read_test();
			Update_FW_in_Driver(0);
			break;
#endif
#ifdef ELAN_2WIREICE
		case IOCTL_2WIREICE:
			elan_TWO_WIRE_ICE(private_ts->client);
			break;
#endif
		case IOCTL_CIRCUIT_CHECK:
			return circuit_ver;
			break;
		default:
			touch_debug(DEBUG_ERROR, "[elan] Un-known IOCTL Command %d\n", cmd);
			break;
	}
	return 0;
}

struct file_operations elan_touch_fops =
{
	.open =         elan_iap_open,
	.write =        elan_iap_write,
	.read = 	elan_iap_read,
	.release =	elan_iap_release,
	.unlocked_ioctl = elan_iap_ioctl,
};

// add by leo ++
static int ekth3260_open(struct inode *inode, struct file *file)
{
	printk("[elan] %s:[%d]: ++ \n", __func__, __LINE__);
	return 0;
}

static int ekth3260_release(struct inode *inode, struct file *file)
{
	printk("[elan] %s:[%d]: ++ \n", __func__, __LINE__);
	return 0;
}

static long ekth3260_ioctl(struct file *file, unsigned cmd, unsigned long arg)
{
	int ret = 0, val = 0;

	printk("[elan] %s:[%d]: cmd = 0x%x. \n", __func__, __LINE__, (u32)cmd);

	if(_IOC_TYPE(cmd) != TOUCH_IOCTL_MAGIC)
	{
		return -ENOTTY;
	}
	if(_IOC_DIR(cmd) & _IOC_READ)
		ret = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
		ret =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if(ret)
	{
		return -EFAULT;
	}

	switch(cmd)
	{
		case TOUCH_IOCTL_FW_CHECK:

			printk("[elan] %s:[%d]: TOUCH_IOCTL_FW_CHECK. \n", __func__, __LINE__);

			work_lock = 1;
			disable_irq(private_ts->client->irq);
			wake_lock(&private_ts->wakelock);

			__fw_packet_handler(private_ts->client);

			work_lock = 0;
			enable_irq(private_ts->client->irq);
			wake_unlock(&private_ts->wakelock);

			eKTH3260_fw_version_check();
			printk("[elan] %s: ts->fwcheck_result =%d\n", __func__, private_ts->fwcheck_result);
			return put_user(private_ts->fwcheck_result, (unsigned long __user *)arg);
			break;

		case TOUCH_IOCTL_FW_UPDATE:

			printk("[elan] %s:[%d]: TOUCH_IOCTL_FW_UPDATE. \n", __func__, __LINE__);

			work_lock = 1;
			disable_irq(private_ts->client->irq);
			wake_lock(&private_ts->wakelock);

			Update_FW_in_Driver(0);

			work_lock = 0;
			enable_irq(private_ts->client->irq);
			wake_unlock(&private_ts->wakelock);

			if(FW_ID == 0x5808)
				private_ts->fwupdate_result = FAIL;
			else
				private_ts->fwupdate_result = SUCCESS;

			if(private_ts->fwupdate_result == SUCCESS)
				return put_user(0, (unsigned long __user *)arg);
			else
				return put_user(1, (unsigned long __user *)arg);
			break;

		case TOUCH_IOCTL_FORCE_FW_UPDATE:

			printk("[elan] %s:[%d]: TOUCH_IOCTL_FORCE_FW_UPDATE. \n", __func__, __LINE__);

			if(private_ts->fwcheck_result == NEED_TO_UPDATE)
				return put_user(0, (unsigned long __user *)arg);
			else
				return put_user(1, (unsigned long __user *)arg);
			break;

		case TOUCH_IOCTL_READ_CMD:

			printk("[elan] %s:[%d]: TOUCH_IOCTL_READ_CMD. \n", __func__, __LINE__);

			break;

		case TOUCH_IOCTL_WRITE_CMD:

			printk("[elan] %s:[%d]: TOUCH_IOCTL_WRITE_CMD. \n", __func__, __LINE__);

			if(get_user(val, (unsigned long __user *)arg))
				return -EFAULT;

			switch(val)
			{
				case 0:
				case 1:
					break;

				default:
					printk("[elan] %s:[%d]: incorrect val (%d). \n", __func__, __LINE__, val);
			}
			break;

		default:
			printk("[elan] %s:[%d]: incorrect cmd (%d). \n", __func__, __LINE__, _IOC_NR(cmd));
			return -EINVAL;
	}
	return 0;
}

static struct file_operations ekth3260_fops =
{
	.owner = THIS_MODULE,
	.open = ekth3260_open,
	.release = ekth3260_release,
	//.unlocked_ioctl = ekth3260_ioctl,
	.compat_ioctl = ekth3260_ioctl
};

struct miscdevice ekth3260_misc_dev =
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = TOUCH_MDEV_NAME,
	.fops = &ekth3260_fops,
};
// add by leo --

#ifdef IAP_PORTION
int EnterISPMode(struct i2c_client *client)
{
	int len = 0;
	uint8_t isp_cmd[] = {0x45, 0x49, 0x41, 0x50}; //{0x45, 0x49, 0x41, 0x50};
	len = i2c_master_send(private_ts->client, isp_cmd,  4);
	if(len != 4)
	{
		touch_debug(DEBUG_ERROR, "[ELAN] ERROR: EnterISPMode fail! len=%d\r\n", len);
		return -1;
	}
	else
		touch_debug(DEBUG_MESSAGES, "[ELAN] IAPMode write data successfully! cmd = [%2x, %2x, %2x, %2x]\n", isp_cmd[0], isp_cmd[1], isp_cmd[2], isp_cmd[3]);
	return 0;
}

int ExtractPage(struct file *filp, uint8_t * szPage, int byte)
{
	int len = 0;

	len = filp->f_op->read(filp, szPage, byte, &filp->f_pos);
	if(len != byte)
	{
		touch_debug(DEBUG_ERROR, "[ELAN] %s: read page error, read error. len=%d\r\n", __func__, len);
		return -1;
	}

	return 0;
}

int WritePage(const u8 * szPage, int byte)
{
	int len = 0;

	len = i2c_master_send(private_ts->client, szPage,  byte);
	if(len != byte)
	{
		touch_debug(DEBUG_ERROR, "[ELAN] %s: write page error, write error. len=%d\r\n", __func__, len);
		return -1;
	}

	return 0;
}

int GetAckData(struct i2c_client *client)
{
	int len = 0;

	uint8_t buff[2] = {0};

	len = i2c_master_recv(private_ts->client, buff, sizeof(buff));
	if(len != sizeof(buff))
	{
		touch_debug(DEBUG_ERROR, "[ELAN] %s: Read ACK Data error. len=%d\r\n", __func__, len);
		return -1;
	}

	touch_debug(DEBUG_MESSAGES, "[ELAN] %s: %x,%x", __func__, buff[0], buff[1]);
	if(buff[0] == 0xaa && buff[1] == 0xaa)
		return ACK_OK;
	else if(buff[0] == 0x55 && buff[1] == 0x55)
		return ACK_REWRITE;
	else
		return ACK_Fail;

	return 0;
}

void print_progress(int page, int ic_num, int j)
{
	int i, percent, page_tatol = 351, percent_tatol;
	char str[256];
	str[0] = '\0';
	for(i = 0; i < ((page) / 10); i++)
	{
		str[i] = '#';
		str[i + 1] = '\0';
	}
	/*
	page_tatol=page+PageNum*(ic_num-j);
	percent = ((100*page)/(PageNum));
	percent_tatol = ((100*page_tatol)/(PageNum*ic_num));
	*/
	percent = ((100 * page) / (PageNum));
	if((page) == (PageNum))
		percent = 100;

	if((page_tatol) == (PageNum * ic_num))
		percent_tatol = 100;

	touch_debug(DEBUG_INFO, "\rprogress %s| %d %d", str, percent, page);

	if(page == (PageNum))
		touch_debug(DEBUG_INFO, "\n");

}

static int Update_FW_in_Driver(void *x)
{
	int res = 0, ic_num = 1;
	int iPage = 0, rewriteCnt = 0; //rewriteCnt for PAGE_REWRITE
	int i = 0;
	uint8_t data;
	//struct timeval tv1, tv2;

	uint8_t boot_buffer[4] = {0};
#ifdef BURN_BY_8BYTE // 8byte mode
	int byte_count;
#endif

	uint8_t *szBuff = NULL;
	int curIndex = 0;
	mutex_lock(&ktf_mutex);
	touch_debug(DEBUG_INFO, "[ELAN] %s: Update FW\n", __func__);
IAP_RESTART:

	curIndex = 0;
	data = I2C_DATA[0]; //Master
	touch_debug(DEBUG_INFO, "[ELAN] %s: address data=0x%x \r\n", __func__, data);

	if(RECOVERY != 0x80)
	{
		touch_debug(DEBUG_MESSAGES, "[ELAN] Firmware upgrade normal mode !\n");
	}
	else
		touch_debug(DEBUG_MESSAGES, "[ELAN] Firmware upgrade recovery mode !\n");

	/*Send enter bootcode cmd*/
	elan_ktf_ts_hw_reset();
	mdelay(23);
	res = EnterISPMode(private_ts->client);	 //enter ISP mode
	//elan_ktf_ts_poll(private_ts->client);
	mdelay(100);
	/*check enter bootcode cmd*/
	res = i2c_master_recv(private_ts->client, boot_buffer, 4);   //55 aa 33 cc
	touch_debug(DEBUG_MESSAGES, "[ELAN] %s :%x,%x,%x,%x\n", __func__, boot_buffer[0], boot_buffer[1], boot_buffer[2], boot_buffer[3]);

	/* Send Dummy Byte	*/
	res = i2c_master_send(private_ts->client, &data,  sizeof(data));
	if(res != sizeof(data))
	{
		touch_debug(DEBUG_ERROR, "[ELAN] dummy error code = %d\n", res);
	}
#ifdef ESD_CHECK
	live_state = 1;
#endif
	/* Start IAP*/
	for(iPage = 1; iPage <= PageNum; iPage++)
	{
#ifdef BURN_BY_8BYTE // 8byte mode
		// 8 bytes
		//szBuff = fw_data + ((iPage-1) * PageSize);
		for(byte_count = 1; byte_count <= 17; byte_count++)
		{
			if(byte_count != 17)
			{
				//			printk("[ELAN] byte %d\n",byte_count);
				//			printk("curIndex =%d\n",curIndex);
				if(!ekth3260_update_outside)
				{
					// add by leo for AUO panel ++
					//szBuff = file_fw_data + curIndex;
					if(private_ts->LCM_ID2)
						szBuff = file_fw_data_AUO + curIndex;
					else
						szBuff = file_fw_data_CPT + curIndex;
					// add by leo for AUO panel --
				}
				else
				{
					szBuff = firmware_outside + curIndex;
				}
				curIndex =  curIndex + 8;

				//ioctl(fd, IOCTL_IAP_MODE_LOCK, data);
				res = WritePage(szBuff, 8);
			}
			else
			{
				//			printk("byte %d\n",byte_count);
				//			printk("curIndex =%d\n",curIndex);
				if(!ekth3260_update_outside)
				{
					// add by leo for AUO panel ++
					//szBuff = file_fw_data + curIndex;
					if(private_ts->LCM_ID2)
						szBuff = file_fw_data_AUO + curIndex;
					else
						szBuff = file_fw_data_CPT + curIndex;
					// add by leo for AUO panel --
				}
				else
				{
					szBuff = firmware_outside + curIndex;
				}
				curIndex =  curIndex + 4;
				//ioctl(fd, IOCTL_IAP_MODE_LOCK, data);
				res = WritePage(szBuff, 4);
			}
			mdelay(1);
		} // end of for(byte_count=1;byte_count<=17;byte_count++)
#endif
#ifndef BURN_BY_8BYTE // 132byte mode		
		if(!ekth3260_update_outside)
		{
			// add by leo for AUO panel ++
			//szBuff = file_fw_data + curIndex;
			if(private_ts->LCM_ID2)
				szBuff = file_fw_data_AUO + curIndex;
			else
				szBuff = file_fw_data_CPT + curIndex;
			// add by leo for AUO panel --
		}
		else
			szBuff = firmware_outside + curIndex;

		curIndex =  curIndex + PageSize;
		res = WritePage(szBuff, PageSize);
#endif
#if 1 //if use old bootcode
		if(iPage == PageNum || iPage == 1)
		{
			mdelay(600);
		}
		else
		{
			mdelay(50);
		}
#endif

		res = GetAckData(private_ts->client);

		if(ACK_OK != res)
		{
			mdelay(50);
			touch_debug(DEBUG_ERROR, "[ELAN] ERROR: GetAckData fail! res=%d\r\n", res);
			rewriteCnt = rewriteCnt + 1;
			if(rewriteCnt == PAGERETRY)
			{
				touch_debug(DEBUG_ERROR, "[ELAN] %dth page ReWrite %d times fails!\n", iPage, PAGERETRY);
				mutex_unlock(&ktf_mutex);
				return E_FD;
			}
			else
			{
				touch_debug(DEBUG_ERROR, "[ELAN] %d page ReWrite %d times!\n",  iPage, rewriteCnt);
				goto IAP_RESTART;
			}

		}
		else
		{
			print_progress(iPage, ic_num, i);
		}
	} // end of for(iPage = 1; iPage <= PageNum; iPage++)

	RECOVERY = 0;
	res = __hello_packet_handler(private_ts->client);
	if(res > 0)
		touch_debug(DEBUG_ERROR, "[ELAN] Update ALL Firmware successfully!\n");

	mutex_unlock(&ktf_mutex);
	return res;   /* 0:sucessfully, 0x80: Recovery, -1: No response */
}
#endif
// End Firmware Update

// Star 2wireIAP which used I2C to simulate JTAG function
#ifdef ELAN_2WIREICE
static uint8_t file_bin_data[] =
{
#include "2wireice.i"
};

int write_ice_status = 0;
int shift_out_16(struct i2c_client *client)
{
	int res;
	uint8_t buff[] = {0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbf, 0xff};
	res = i2c_master_send(client, buff,  sizeof(buff));
	return res;
}
int tms_reset(struct i2c_client *client)
{
	int res;
	uint8_t buff[] = {0xff, 0xff, 0xff, 0xbf};
	res = i2c_master_send(client, buff,  sizeof(buff));
	return res;
}

int mode_gen(struct i2c_client *client)
{
	int res;
	int retry = 5;
	uint8_t buff[] = {0xff, 0xff, 0xff, 0x31, 0xb7, 0xb7, 0x7b, 0xb7, 0x7b, 0x7b, 0xb7, 0x7b, 0xf3, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xf1};
	uint8_t buff_1[] = {0x2a, 0x6a, 0xa6, 0xa6, 0x6e};
	char mode_buff[2] = {0};
	do
	{
		res = i2c_master_send(client, buff,  sizeof(buff));
		if(res != sizeof(buff))
		{
			touch_debug(DEBUG_ERROR, "[ELAN] ERROR: mode_gen write buff error, write  error. res=%d\r\n", res);
		}
		else
		{
			touch_debug(DEBUG_MESSAGES, "[ELAN] mode_gen write buff successfully.\r\n");
			break;
		}
		mdelay(20);
		retry -= 1;
	}
	while(retry);
	res = i2c_master_recv(client, mode_buff, sizeof(mode_buff));
	if(res != sizeof(mode_buff))
	{
		touch_debug(DEBUG_ERROR, "[ELAN] ERROR: mode_gen read data error, write  error. res=%d\r\n", res);
		return -1;
	}
	else
		touch_debug(DEBUG_MESSAGES, "[ELAN] mode gen read successfully(a6 59)! buff[0]=0x%x  buff[1]=0x%x \r\n", mode_buff[0], mode_buff[1]);

	res = i2c_master_send(client, buff_1,  sizeof(buff_1));
	if(res != sizeof(buff_1))
	{
		touch_debug(DEBUG_ERROR, "[ELAN] ERROR: mode_gen write buff_1 error. res=%d\r\n", res);
		return -1;
	}
	return res;
}

int word_scan_out(struct i2c_client *client)
{
	int res;
	uint8_t buff[] = {0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x26, 0x66};
	res = i2c_master_send(client, buff,  sizeof(buff));
	return res;
}

int long_word_scan_out(struct i2c_client *client)
{
	int res;
	uint8_t buff[] = {0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x26, 0x66};
	res = i2c_master_send(client, buff,  sizeof(buff));
	return res;
}


int bit_manipulation(int TDI, int TMS, int TCK, int TDO, int TDI_1, int TMS_1, int TCK_1, int TDO_1)
{
	int res;
	res = ((TDI << 3 | TMS << 2 | TCK | TDO) << 4) | (TDI_1 << 3 | TMS_1 << 2 | TCK_1 | TDO_1);
	return res;
}

int ins_write(struct i2c_client *client, uint8_t buf)
{
	int res = 0;
	int length = 13;
	uint8_t write_buf[7] = {0};
	int TDI_bit[13] = {0};
	int TMS_bit[13] = {0};
	int i = 0;
	uint8_t buf_rev = 0;
	int TDI = 0, TMS = 0, TCK = 0, TDO = 0;
	int bit_tdi, bit_tms;
	int len;

	for(i = 0; i < 8; i++)
	{
		buf_rev = buf_rev | (((buf >> i) & 0x01) << (7 - i));
	}


	TDI = (0x7 << 10) | buf_rev << 2 | 0x00;
	TMS = 0x1007;
	TCK = 0x2;
	TDO = 1;

	for(len = 0; len <= length - 1; len++)
	{
		bit_tdi = TDI & 0x1;
		bit_tms = TMS & 0x1;
		TDI_bit[length - 1 - len] = bit_tdi;
		TMS_bit[length - 1 - len] = bit_tms;
		TDI = TDI >> 1;
		TMS = TMS >> 1;
	}

	for(len = 0; len <= length - 1; len = len + 2)
	{
		if(len == length - 1 && len % 2 == 0)
			res = bit_manipulation(TDI_bit[len], TMS_bit[len], TCK, TDO, 0, 0, 0, 0);
		else
			res = bit_manipulation(TDI_bit[len], TMS_bit[len], TCK, TDO, TDI_bit[len + 1], TMS_bit[len + 1], TCK, TDO);
		write_buf[len / 2] = res;
	}

	res = i2c_master_send(client, write_buf,  sizeof(write_buf));
	return res;
}


int word_scan_in(struct i2c_client *client, uint16_t buf)
{
	int res = 0;
	uint8_t write_buf[10] = {0};
	int TDI_bit[20] = {0};
	int TMS_bit[20] = {0};


	int TDI =  buf << 2 | 0x00;
	int  TMS = 0x7;
	int  TCK = 0x2;
	int TDO = 1;

	int bit_tdi, bit_tms;
	int len;

	for(len = 0; len <= 19; len++)   //length =20
	{
		bit_tdi = TDI & 0x1;
		bit_tms = TMS & 0x1;

		TDI_bit[19 - len] = bit_tdi;
		TMS_bit[19 - len] = bit_tms;
		TDI = TDI >> 1;
		TMS = TMS >> 1;
	}

	for(len = 0; len <= 19; len = len + 2)
	{
		if(len == 19 && len % 2 == 0)
			res = bit_manipulation(TDI_bit[len], TMS_bit[len], TCK, TDO, 0, 0, 0, 0);
		else
			res = bit_manipulation(TDI_bit[len], TMS_bit[len], TCK, TDO, TDI_bit[len + 1], TMS_bit[len + 1], TCK, TDO);
		write_buf[len / 2] = res;
	}

	res = i2c_master_send(client, write_buf,  sizeof(write_buf));
	return res;
}

int long_word_scan_in(struct i2c_client *client, int buf_1, int buf_2)
{
	uint8_t write_buf[18] = {0};
	uint8_t TDI_bit[36] = {0};
	uint8_t TMS_bit[36] = {0};

	int TDI_1 = buf_1;
	int TDI_2 = (buf_2 << 2) | 0x00;
	int TMS = 0x7;
	int TCK = 0x2;
	int TDO = 1;

	int bit_tdi, bit_tms;
	int len = 0;
	int res = 0;


	for(len = 0; len <= 35; len++)   //length =36
	{

		if(len < 18)
		{
			bit_tdi = TDI_2 & 0x1;
		}
		else
		{
			bit_tdi = TDI_1 & 0x1;
		}
		bit_tms = TMS & 0x1;

		TDI_bit[35 - len] = bit_tdi;
		TMS_bit[35 - len] = bit_tms;
		if(len < 18)
		{
			TDI_2 = TDI_2 >> 1;
		}
		else
		{
			TDI_1 = TDI_1 >> 1;
		}
		TMS = TMS >> 1;
		bit_tdi = 0;
		bit_tms = 0;
	}


	for(len = 0; len <= 35; len = len + 2)
	{
		if(len == 35 && len % 2 == 0)
			res = bit_manipulation(TDI_bit[len], TMS_bit[len], TCK, TDO, 0, 0, 0, 1);
		else
			res = bit_manipulation(TDI_bit[len], TMS_bit[len], TCK, TDO, TDI_bit[len + 1], TMS_bit[len + 1], TCK, TDO);
		write_buf[len / 2] = res;
	}

	res = i2c_master_send(client, write_buf,  sizeof(write_buf));
	return res;
}

uint16_t trimtable[8] = {0};

int Read_SFR(struct i2c_client *client, int open)
{
	uint8_t voltage_recv[2] = {0};

	int count, ret;
	//uint16_t address_1[8]={0x0000,0x0001,0x0002,0x0003,0x0004,0x0005,0x0006,0x0007};

	ins_write(client, 0x6f); // IO write
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	//  0
	ins_write(client, 0x6f);  //IO Write
	long_word_scan_in(client, 0x007f, 0x9002);  //TM=2h
	ins_write(client, 0x68);  //Program Memory Sequential Read
	word_scan_in(client, 0x0000);  //set Address 0x0000
	shift_out_16(client);   //move data to I2C buf

	mdelay(10);
	count = 0;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv));
	if(ret != sizeof(voltage_recv))
	{
		touch_debug(DEBUG_ERROR, "[ELAN] %s: read data error. ret=%d\r\n", __func__, ret);
		return -1;
	}
	else
	{
		trimtable[count] = voltage_recv[0] << 8 | voltage_recv[1];
		touch_debug(DEBUG_MESSAGES, "[ELAN] read data successfully! voltage_recv buff[0]=0x%x  buff[1]=0x%x  trimtable[%d]=0x%x\r\n", voltage_recv[0], voltage_recv[1], count, trimtable[count]);
	}
	//  1
	ins_write(client, 0x6f); // IO write
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0001);
	shift_out_16(client);

	mdelay(1);
	count = 1;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv));
	if(ret != sizeof(voltage_recv))
	{
		touch_debug(DEBUG_ERROR, "[ELAN] %s: read data error. ret=%d\r\n", __func__, ret);
		return -1;
	}
	else
	{
		trimtable[count] = voltage_recv[0] << 8 | voltage_recv[1];
		touch_debug(DEBUG_MESSAGES, "[ELAN] read data successfully! voltage_recv buff[0]=0x%x  buff[1]=0x%x  trimtable[%d]=0x%x\r\n", voltage_recv[0], voltage_recv[1], count, trimtable[count]);
	}


	//  2
	ins_write(client, 0x6f); // IO write
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0002);
	shift_out_16(client);

	mdelay(1);
	count = 2;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv));
	if(ret != sizeof(voltage_recv))
	{
		touch_debug(DEBUG_ERROR, "[ELAN] %s: read data error. ret=%d\r\n", __func__, ret);
		return -1;
	}
	else
	{
		trimtable[count] = voltage_recv[0] << 8 | voltage_recv[1];
		touch_debug(DEBUG_MESSAGES, "[ELAN] read data successfully! voltage_recv buff[0]=0x%x  buff[1]=0x%x  trimtable[%d]=0x%x\r\n", voltage_recv[0], voltage_recv[1], count, trimtable[count]);
	}


	//  3
	ins_write(client, 0x6f); // IO write
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0003);
	shift_out_16(client);

	mdelay(1);
	count = 3;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv));
	if(ret != sizeof(voltage_recv))
	{
		touch_debug(DEBUG_ERROR, "[ELAN] %s: read data error. ret=%d\r\n", __func__, ret);
		return -1;
	}
	else
	{
		trimtable[count] = voltage_recv[0] << 8 | voltage_recv[1];
		touch_debug(DEBUG_MESSAGES, "[ELAN] read data successfully! voltage_recv buff[0]=0x%x  buff[1]=0x%x  trimtable[%d]=0x%x\r\n", voltage_recv[0], voltage_recv[1], count, trimtable[count]);
	}
	//  4
	ins_write(client, 0x6f); // IO write
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0004);
	shift_out_16(client);

	mdelay(1);
	count = 4;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv));
	if(ret != sizeof(voltage_recv))
	{
		touch_debug(DEBUG_ERROR, "[ELAN] %s: read data error. ret=%d\r\n", __func__, ret);
		return -1;
	}
	else
	{
		trimtable[count] = voltage_recv[0] << 8 | voltage_recv[1];
		touch_debug(DEBUG_MESSAGES, "[ELAN] read data successfully! voltage_recv buff[0]=0x%x  buff[1]=0x%x  trimtable[%d]=0x%x\r\n", voltage_recv[0], voltage_recv[1], count, trimtable[count]);
	}


	//  5
	ins_write(client, 0x6f); // IO write
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0005);
	shift_out_16(client);

	mdelay(1);
	count = 5;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv));
	if(ret != sizeof(voltage_recv))
	{
		touch_debug(DEBUG_ERROR, "[ELAN] %s: read data error. ret=%d\r\n", __func__, ret);
		return -1;
	}
	else
	{
		trimtable[count] = voltage_recv[0] << 8 | voltage_recv[1];
		touch_debug(DEBUG_MESSAGES, "[ELAN] read data successfully! voltage_recv buff[0]=0x%x  buff[1]=0x%x  trimtable[%d]=0x%x\r\n", voltage_recv[0], voltage_recv[1], count, trimtable[count]);
	}


	//  6
	ins_write(client, 0x6f); // IO write
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0006);
	shift_out_16(client);

	mdelay(1);
	count = 6;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv));
	if(ret != sizeof(voltage_recv))
	{
		touch_debug(DEBUG_ERROR, "[ELAN] %s: read data error. ret=%d\r\n", __func__, ret);
		return -1;
	}
	else
	{
		trimtable[count] = voltage_recv[0] << 8 | voltage_recv[1];
		touch_debug(DEBUG_MESSAGES, "[ELAN] read data successfully! voltage_recv buff[0]=0x%x  buff[1]=0x%x  trimtable[%d]=0x%x\r\n", voltage_recv[0], voltage_recv[1], count, trimtable[count]);
	}
	//  7
	ins_write(client, 0x6f); // IO write
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0007);
	shift_out_16(client);

	mdelay(1);
	count = 7;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv));
	if(ret != sizeof(voltage_recv))
	{
		touch_debug(DEBUG_ERROR, "[ELAN] %s: read data error. ret=%d\r\n", __func__, ret);
		return -1;
	}
	if(open == 1)
		trimtable[count] = voltage_recv[0] << 8 | (voltage_recv[1] & 0xbf);
	else
		trimtable[count] = voltage_recv[0] << 8 | (voltage_recv[1] | 0x40);
	touch_debug(DEBUG_ERROR, "[ELAN] Open_High_Voltage recv  voltage_recv buff[0]=%x buff[1]=%x, trimtable[%d]=%x \n", voltage_recv[0], voltage_recv[1], count, trimtable[count]);


	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x8000);



	/*
	for (count =0; count <8; count++){

	ins_write(client, 0x6f); // IO write
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, address_1[count]);
		shift_out_16(client);

	mdelay(10);
	//count=6;
		ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv));
		trimtable[count]=voltage_recv[0]<<8 | voltage_recv[1];
	printk("[elan] Open_High_Voltage recv -1 1word =%x %x, trimtable[%d]=%x \n", voltage_recv[0],voltage_recv[1], count, trimtable[count]);

	}

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x8000);

	*/
	return 0;
}

int Write_SFR_2k(struct i2c_client *client, int open)
{

	//set page 1
	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x0001, 0x0100);
	if(open == 1)
	{
		//set HV enable
		touch_debug(DEBUG_MESSAGES, "%s set HV enable\n", __func__);
		ins_write(client, 0x6f);
		long_word_scan_in(client, 0x0050, 0xc041);
	}
	else
	{
		//set HV disable
		touch_debug(DEBUG_MESSAGES, "%s set HV disable\n", __func__);
		ins_write(client, 0x6f);
		long_word_scan_in(client, 0x0050, 0xc040);
	}
	return 0;
}

int Write_SFR(struct i2c_client *client)
{

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9001);


	ins_write(client, 0x66);  // Program Memory Write
	long_word_scan_in(client, 0x0000, trimtable[0]);
	ins_write(client, 0xfd);  //Set up the initial addr for sequential access
	word_scan_in(client, 0x7f);

	ins_write(client, 0x66);
	long_word_scan_in(client, 0x0001, trimtable[1]);
	ins_write(client, 0xfd);
	word_scan_in(client, 0x7f);

	ins_write(client, 0x66);
	long_word_scan_in(client, 0x0002, trimtable[2]);
	ins_write(client, 0xfd);
	word_scan_in(client, 0x7f);

	ins_write(client, 0x66);
	long_word_scan_in(client, 0x0003, trimtable[3]);
	ins_write(client, 0xfd);
	word_scan_in(client, 0x7f);

	ins_write(client, 0x66);
	long_word_scan_in(client, 0x0004, trimtable[4]);
	ins_write(client, 0xfd);
	word_scan_in(client, 0x7f);

	ins_write(client, 0x66);
	long_word_scan_in(client, 0x0005, trimtable[5]);
	ins_write(client, 0xfd);
	word_scan_in(client, 0x7f);

	ins_write(client, 0x66);
	long_word_scan_in(client, 0x0006, trimtable[6]);
	ins_write(client, 0xfd);
	word_scan_in(client, 0x7f);

	ins_write(client, 0x66);
	long_word_scan_in(client, 0x0007, trimtable[7]);
	ins_write(client, 0xfd);
	word_scan_in(client, 0x7f);


	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x7f, 0x8000);
	/*
	for (count=0;count<8;count++){
			ins_write(client, 0x66);
		long_word_scan_in(client, 0x0000+count, trimtable[count]);

	}
	*/

	return 0;
}

int Enter_Mode(struct i2c_client *client)
{
	mode_gen(client);
	tms_reset(client);
	ins_write(client, 0xfc); //system reset
	tms_reset(client);
	return 0;
}
int Open_High_Voltage(struct i2c_client *client, int open)
{
#ifdef EKTF3K_FLASH
	Read_SFR(client, open);
	Write_SFR(client);
	Read_SFR(client, open);

#endif
	Write_SFR_2k(client, open);
	return 0;

}

int Mass_Erase(struct i2c_client *client)
{
	char mass_buff[4] = {0};
	char mass_buff_1[2] = {0};
	int ret, finish = 0, i = 0;
	touch_debug(DEBUG_MESSAGES, "[Elan] Mass_Erase!!!!\n");
	ins_write(client, 0x01); //id code read
	mdelay(2);
	long_word_scan_out(client);

	ret = i2c_master_recv(client, mass_buff, sizeof(mass_buff));
	touch_debug(DEBUG_MESSAGES, "[elan] Mass_Erase mass_buff=%x %x %x %x(c0 08 01 00)\n", mass_buff[0], mass_buff[1], mass_buff[2], mass_buff[3]); //id: c0 08 01 00
	/* / add for test
	ins_write(client, 0xf3);
		word_scan_out(client);
		ret = i2c_master_recv(client, mass_buff_1, sizeof(mass_buff_1));
	printk("[elan] Mass_Erase mass_buff_1=%x %x(a0 00)\n", mass_buff_1[0],mass_buff_1[1]);  // a0 00 : stop
	//add for test

	//read low->high 5th bit
	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);

	// add for test
	ins_write(client, 0xf3);
		word_scan_out(client);
		ret = i2c_master_recv(client, mass_buff_1, sizeof(mass_buff_1));
	printk("[elan] Mass_Erase (II) mass_buff_1=%x %x(40 00)\n", mass_buff_1[0],mass_buff_1[1]);  // 40 00
	//add for test
	mdelay(10); //for malata
	*/

	ins_write(client, 0x6f); //IO Write
	/*add by herman*/
	long_word_scan_in(client, 0x007e, 0x0020);

	long_word_scan_in(client, 0x007f, 0x4000); //orig 4000
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);
	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9040);
	ins_write(client, 0x66); //Program data Write
	long_word_scan_in(client, 0x0000, 0x8765); //change by herman
	ins_write(client, 0x6f); //IO Write
	long_word_scan_in(client, 0x007f, 0x8000);	//clear flash control PROG

	ins_write(client, 0xf3);

	while(finish == 0)
	{
		word_scan_out(client);
		ret = i2c_master_recv(client, mass_buff_1, sizeof(mass_buff_1));
		if(ret != sizeof(mass_buff_1))
		{
			touch_debug(DEBUG_ERROR, "[ELAN] ERROR: read data error. res=%d\r\n", ret);
			return -1;
		}
		else
		{
			finish = (mass_buff_1[1] >> 4) & 0x01;
			touch_debug(DEBUG_MESSAGES, "[ELAN] mass_buff_1[0]=%x, mass_buff_1[1]=%x (80 10)!!!!!!!!!! finish=%d \n", mass_buff_1[0], mass_buff_1[1], finish); //80 10: OK, 80 00: fail
		}
		if(mass_buff_1[1] != I2C_DATA[0] && finish != 1 && i < 100)
		{
			mdelay(100);
			//printk("[elan] mass_buff_1[1] >>4  !=1\n");
			i++;
			if(i == 50)
			{
				touch_debug(DEBUG_ERROR, "[elan] Mass_Erase fail ! \n");
				//return -1;  //for test
			}
		}

	}

	return 0;
}

int Reset_ICE(struct i2c_client *client)
{
	//struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
	int res;
	touch_debug(DEBUG_INFO, "[Elan] Reset ICE!!!!\n");
	ins_write(client, 0x94);
	ins_write(client, 0xd4);
	ins_write(client, 0x20);
	client->addr = I2C_DATA[0];////Modify address before 2-wire
	elan_ktf_ts_hw_reset();
	mdelay(250);
	res = __hello_packet_handler(client);

	return 0;
}

int normal_write_func(struct i2c_client *client, int j, uint8_t *szBuff)
{
	//char buff_check=0;
	uint16_t szbuff = 0, szbuff_1 = 0;
	uint16_t sendbuff = 0;
	int write_byte, iw;

	ins_write(client, 0xfd);
	word_scan_in(client, j * 64);

	ins_write(client, 0x65); //Program data sequential write

	write_byte = 64;

	for(iw = 0; iw < write_byte; iw++)
	{
		szbuff = *szBuff;
		szbuff_1 = *(szBuff + 1);
		sendbuff = szbuff_1 << 8 | szbuff;
		touch_debug(DEBUG_MESSAGES, "[elan]  Write Page sendbuff=0x%04x @@@\n", sendbuff);
		//mdelay(1);
		word_scan_in(client, sendbuff); //data????   buff_read_data
		szBuff += 2;

	}
	return 0;
}

int fastmode_write_func(struct i2c_client *client, int j, uint8_t *szBuff)
{
	uint8_t szfwbuff = 0, szfwbuff_1 = 0;
	uint8_t sendfwbuff[130] = {0};
	uint8_t tmpbuff;
	int i = 0, len = 0;
	private_ts->client->addr = 0x76;

	sendfwbuff[0] = (j * 64) >> 8;
	tmpbuff = ((j * 64) << 8) >> 8;
	sendfwbuff[1] = tmpbuff;
	//printk("fastmode_write_func, sendfwbuff[0]=0x%x, sendfwbuff[1]=0x%x\n", sendfwbuff[0], sendfwbuff[1]);

	for(i = 2; i < 129; i = i + 2)  //  1 Page = 64 word, 1 word=2Byte
	{

		szfwbuff = *szBuff;
		szfwbuff_1 = *(szBuff + 1);
		sendfwbuff[i] = szfwbuff_1;
		sendfwbuff[i + 1] = szfwbuff;
		szBuff += 2;
		//printk("[elan] sendfwbuff[%d]=0x%x, sendfwbuff[%d]=0x%x\n", i, sendfwbuff[i], i+1, sendfwbuff[i+1]);
	}


	len = i2c_master_send(private_ts->client, sendfwbuff,  130);
	if(len != 130)     //address+data(128)
	{
		touch_debug(DEBUG_ERROR, "[ELAN] ERROR: fastmode write page error, write error. len=%d, Page %d\r\n", len, j);
		return -1;
	}
	//printk("fastmode_write_func, send len=%d (130), Page %d --\n", len, j);

	private_ts->client->addr = 0x77;

	return 0;
}


int ektSize;
int lastpage_byte;
int lastpage_flag = 0;
int Write_Page(struct i2c_client *client, int j, uint8_t *szBuff)
{
	int len, finish = 0;
	char buff_read_data[2];
	int i = 0;

	ins_write(client, 0x6f);  //IO Write
	//long_word_scan_in(client,0x007e,0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);
	long_word_scan_in(client, 0x007f, 0x9400);

	ins_write(client, 0x66);   //Program Data Write
	//long_word_scan_in(client,0x0000,0x5a5a);
	long_word_scan_in(client, j * 64, 0x0000);
	//printk("[elan] j*64=0x%x @@ \n", j*64);
	//long_word_scan_in(client, j*64,0x5a5a);  //set ALE

	//normal_write_func(client, j, szBuff); ////////////choose one : normal / fast mode
	fastmode_write_func(client, j, szBuff); //////////

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9000);

	//ins_write(client,0x6f);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0xf3);  //Debug Reg Read

	while(finish == 0)
	{
		word_scan_out(client);
		len = i2c_master_recv(client, buff_read_data, sizeof(buff_read_data));
		if(len != sizeof(buff_read_data))
		{
			touch_debug(DEBUG_ERROR, "[ELAN] ERROR: Write_Page read buff_read_data error, len=%d\r\n", len);
			return E_FD;
		}
		else
		{
			finish = (buff_read_data[1] >> 4) & 0x01;
			printk("[ELAN] read data successfully! buff[0]=0x%x  buff[1]=0x%x  finish=%d\r\n", buff_read_data[0], buff_read_data[1], finish);  //80 10: ok
		}
		if(finish != 1)
		{
			mdelay(10);
			//printk("[elan] Write_Page finish !=1\n");
			i++;
			if(i == 50)
			{
				touch_debug(DEBUG_ERROR, "[elan] Write_Page finish !=1, Page=%d\n", j);
				write_ice_status = 1;
				return -1;
			}
		}

	}
	return 0;
}

int fastmode_read_func(struct i2c_client *client, int j, uint8_t *szBuff)
{
	uint8_t szfrbuff = 0, szfrbuff_1 = 0;
	uint8_t sendfrbuff[2] = {0};
	uint8_t recvfrbuff[130] = {0};
	uint16_t tmpbuff;
	int i = 0, len = 0, retry = 0;


	ins_write(client, 0x67);

	private_ts->client->addr = 0x76;

	sendfrbuff[0] = (j * 64) >> 8;
	tmpbuff = ((j * 64) << 8) >> 8;
	sendfrbuff[1] = tmpbuff;
	//printk("fastmode_write_func, sendfrbuff[0]=0x%x, sendfrbuff[1]=0x%x\n", sendfrbuff[0], sendfrbuff[1]);
	len = i2c_master_send(private_ts->client, sendfrbuff,  sizeof(sendfrbuff));

	len = i2c_master_recv(private_ts->client, recvfrbuff,  sizeof(recvfrbuff));
	//printk("fastmode_read_func, recv len=%d (128)\n", len);

	for(i = 2; i < 129; i = i + 2)
	{
		szfrbuff = *szBuff;
		szfrbuff_1 = *(szBuff + 1);
		szBuff += 2;
		if(recvfrbuff[i] != szfrbuff_1 || recvfrbuff[i + 1] != szfrbuff)
		{
			touch_debug(DEBUG_ERROR, "[elan] @@@@Read Page Compare Fail. recvfrbuff[%d]=%x, recvfrbuff[i+1]=%x, szfrbuff_1=%x, szfrbuff=%x, ,j =%d@@@@@@@@@@@@@@@@\n\n", i, recvfrbuff[i], recvfrbuff[i + 1], szfrbuff_1, szfrbuff, j);
			write_ice_status = 1;
			retry = 1;
		}
		break;//for test
	}

	private_ts->client->addr = 0x77;
	if(retry == 1)
	{
		return -1;
	}
	else
		return 0;
}


int normal_read_func(struct i2c_client *client, int j,  uint8_t *szBuff)
{
	char read_buff[2];
	int m, len, read_byte;
	uint16_t szbuff = 0, szbuff_1 = 0;

	ins_write(client, 0xfd);

	//printk("[elan] Read_Page, j*64=0x%x\n", j*64);
	word_scan_in(client, j * 64);
	ins_write(client, 0x67);

	word_scan_out(client);

	read_byte = 64;
	//for(m=0;m<64;m++){
	for(m = 0; m < read_byte; m++)
	{
		// compare......
		word_scan_out(client);
		len = i2c_master_recv(client, read_buff, sizeof(read_buff));

		szbuff = *szBuff;
		szbuff_1 = *(szBuff + 1);
		szBuff += 2;
		touch_debug(DEBUG_MESSAGES, "[elan] Read Page: byte=%x%x, szbuff=%x%x \n", read_buff[0], read_buff[1], szbuff, szbuff_1);

		if(read_buff[0] != szbuff_1 || read_buff[1] != szbuff)
		{
			touch_debug(DEBUG_ERROR, "[elan] @@@@@@@@@@Read Page Compare Fail. j =%d. m=%d.@@@@@@@@@@@@@@@@\n\n", j, m);
			write_ice_status = 1;
		}
	}
	return 0;
}


int Read_Page(struct i2c_client *client, int j,  uint8_t *szBuff)
{
	int res = 0;
	ins_write(client, 0x6f);
	//long_word_scan_in(client,0x007e,0x0023);
	long_word_scan_in(client, 0x007f, 0x9000);
	ins_write(client, 0x68);

	//mdelay(10); //for malata
	//normal_read_func(client, j,  szBuff); ////////////////choose one: normal / fastmode
	fastmode_read_func(client, j,  szBuff);

	//Clear Flashce
	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x0000);
	if(res == -1)
	{
		return -1;
	}
	return 0;

}



int TWO_WIRE_ICE(struct i2c_client *client)
{
	int i;

	uint8_t *szBuff = NULL;
	//char szBuff[128]={0};
	int curIndex = 0;
	int PageSize = 128;
	int res;
	//int ektSize;
	//test
	write_ice_status = 0;
	ektSize = sizeof(file_bin_data) / PageSize;
	client->addr = 0x77;////Modify address before 2-wire

	touch_debug(DEBUG_INFO, "[Elan] ektSize=%d ,modify address = %x\n ", ektSize, client->addr);

	i = Enter_Mode(client);
	i = Open_High_Voltage(client, 1);
	if(i == -1)
	{
		touch_debug(DEBUG_ERROR, "[Elan] Open High Voltage fail\n");
		return -1;
	}
	//return 0;

	i = Mass_Erase(client);  //mark temp
	if(i == -1)
	{
		touch_debug(DEBUG_ERROR, "[Elan] Mass Erase fail\n");
		return -1;
	}


	//for fastmode
	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007e, 0x0036);
	long_word_scan_in(client, 0x007f, 0x8000);
	long_word_scan_in(client, 0x007e, 0x0023);	//add by herman


	// client->addr = 0x76;////Modify address before 2-wire
	touch_debug(DEBUG_INFO, "[Elan-test] client->addr =%2x\n", client->addr);
	//for fastmode
	for(i = 0 ; i < ektSize; i++)
	{
		szBuff = file_bin_data + curIndex;
		curIndex =  curIndex + PageSize;
		//	printk("[Elan] Write_Page %d........................wait\n ", i);

		res = Write_Page(client, i, szBuff);
		if(res == -1)
		{
			touch_debug(DEBUG_ERROR, "[Elan] Write_Page %d fail\n ", i);
			break;
		}
		//printk("[Elan] Read_Page %d........................wait\n ", i);
		mdelay(1);
		Read_Page(client, i, szBuff);
		//printk("[Elan] Finish  %d  Page!!!!!!!.........wait\n ", i);
	}


	if(write_ice_status == 0)
	{
		touch_debug(DEBUG_INFO, "[elan] Update_FW_Boot Finish!!! \n");
	}
	else
	{
		touch_debug(DEBUG_INFO, "[elan] Update_FW_Boot fail!!! \n");
	}

	i = Open_High_Voltage(client, 0);
	if(i == -1) return -1;  //test

	Reset_ICE(client);

	return 0;
}

int elan_TWO_WIRE_ICE(struct i2c_client *client)  // for driver internal 2-wire ice
{
	work_lock = 1;
	disable_irq(private_ts->client->irq);
	//wake_lock(&private_ts->wakelock);
	TWO_WIRE_ICE(client);
	work_lock = 0;
	enable_irq(private_ts->client->irq);
	//wake_unlock(&private_ts->wakelock);
	return 0;
}
// End 2WireICE
#endif


static int FW_Update(struct i2c_client *client, int recovery)
{
	int res = 0, ic_num = 1;
	int iPage = 0, rewriteCnt = 0; //rewriteCnt for PAGE_REWRITE
	int i = 0;
	uint8_t data;
	int restartCnt = 0; // For IAP_RESTART
	int byte_count;
	const u8 *szBuff = NULL;
	int curIndex = 0;
	// 0x54, 0x00, 0x12, 0x34
	int rc, fw_size;

	/* Star Request Firmware */
	const u8 *fw_data;
	const struct firmware *p_fw_entry;

	touch_debug(DEBUG_INFO, "request_firmware name = %s\n", ELAN_FW_FILENAME);
	rc = request_firmware(&p_fw_entry, ELAN_FW_FILENAME, &client->dev);
	if(rc != 0)
	{
		touch_debug(DEBUG_ERROR, "rc=%d, request_firmware fail\n", rc);
		return -1;
	}
	else
		touch_debug(DEBUG_INFO, "Firmware Size=%zu\n", p_fw_entry->size);

	fw_data = p_fw_entry->data;
	fw_size = p_fw_entry->size;
	/* End Request Firmware */

	touch_debug(DEBUG_INFO, "[ELAN] %s:  ic_num=%d\n", __func__, ic_num);
IAP_RESTART:

	data = I2C_DATA[0]; //Master
	touch_debug(DEBUG_INFO, "[ELAN] %s: address data=0x%x \r\n", __func__, data);

	if(recovery != 0x80)
	{
		touch_debug(DEBUG_INFO, "[ELAN] Firmware upgrade normal mode !\n");
		elan_ktf_ts_hw_reset();
		res = EnterISPMode(private_ts->client);   //enter ISP mode
	}
	else
		touch_debug(DEBUG_INFO, "[ELAN] Firmware upgrade recovery mode !\n");


	touch_debug(DEBUG_INFO, "[ELAN] send one byte data:%x,%x", private_ts->client->addr, data);
	res = i2c_master_send(private_ts->client, &data,  sizeof(data));
	if(res != sizeof(data))
	{
		printk("[ELAN] dummy error code = %d\n", res);
		touch_debug(DEBUG_ERROR, "[ELAN] dummy error code = %d\n", res);
	}
	mdelay(10);
	// Start IAP
	for(iPage = 1; iPage <= PageNum; iPage++)
	{
		for(byte_count = 1; byte_count <= 17; byte_count++)
		{
			if(byte_count != 17)
			{
				szBuff = fw_data + curIndex;
				curIndex =  curIndex + 8;
				res = WritePage(szBuff, 8);
			}
			else
			{

				szBuff = fw_data + curIndex;
				curIndex =  curIndex + 4;
				res = WritePage(szBuff, 4);
			}
		} // end of for(byte_count=1;byte_count<=17;byte_count++)
		if(iPage == 377 || iPage == 1)
		{
			mdelay(600);
		}
		else
		{
			mdelay(50);
		}
		res = GetAckData(private_ts->client);

		if(ACK_OK != res)
		{
			mdelay(50);
			touch_debug(DEBUG_ERROR, "[ELAN] ERROR: GetAckData fail! res=%d\r\n", res);
			if(res == ACK_REWRITE)
			{
				rewriteCnt = rewriteCnt + 1;
				if(rewriteCnt == PAGERETRY)
				{
					touch_debug(DEBUG_ERROR, "[ELAN] ID 0x%02x %dth page ReWrite %d times fails!\n", data, iPage, PAGERETRY);
					return E_FD;
				}
				else
				{
					touch_debug(DEBUG_ERROR, "[ELAN] ---%d--- page ReWrite %d times!\n",  iPage, rewriteCnt);
					goto IAP_RESTART;
				}
			}
			else
			{
				restartCnt = restartCnt + 1;
				if(restartCnt >= 5)
				{
					touch_debug(DEBUG_ERROR, "[ELAN] ID 0x%02x ReStart %d times fails!\n", data, IAPRESTART);
					return E_FD;
				}
				else
				{
					printk("[ELAN] ===%d=== page ReStart %d times!\n",  iPage, restartCnt);
					touch_debug(DEBUG_ERROR, "[ELAN] ===%d=== page ReStart %d times!\n",  iPage, restartCnt);
					goto IAP_RESTART;
				}
			}
		}
		else
		{
			rewriteCnt = 0;
			print_progress(iPage, ic_num, i);
		}

		mdelay(10);
	} // end of for(iPage = 1; iPage <= PageNum; iPage++)

	res = __hello_packet_handler(private_ts->client);
	if(res > 0)	touch_debug(DEBUG_INFO, "[ELAN] Update Firmware successfully!\n");

	return res;
}

static ssize_t set_debug_mesg(struct device *dev,
                              struct device_attribute *attr, const char *buf, size_t count)
{
	int level[SYSFS_MAX_LEN];
	sscanf(buf, "%x", &level[0]);
	debug = level[0];
	//touch_debug(DEBUG_INFO, "debug level %d, size %d\n", debug,count);
	return count;
}
#ifdef GESTURE_MODE
static ssize_t set_gesture(struct device *dev,
                           struct device_attribute *attr, const char *buf, size_t count)
{
	char *buf_bk = kmalloc(count + 1, GFP_KERNEL);
	char *buf_fp;
	memset(buf_bk, 0, count);
	memcpy(buf_bk, buf, count);
	//buf_bk[count+1]='\0';
	buf_fp = buf_bk;
	sscanf(buf_fp, "%x", &gesture_enable);
	buf_fp = strstr(buf_fp, " ");

	printk("gesture_enable %x\n", gesture_enable);
	kfree(buf_bk);
	return count;
}

static ssize_t show_gesture(struct device *dev,
                            struct device_attribute *attr, char *buf)
{
	sprintf(buf, "gesture_enable %d \n", gesture_enable);

	return sprintf(buf, "%s\n", buf);
}
#endif

#if 0
static ssize_t set_i2c_addr(struct device *dev,
                            struct device_attribute *attr, const char *buf, size_t count)
{
	int addr[SYSFS_MAX_LEN];
	sscanf(buf, "%x", &addr[0]);
	private_ts->client->addr = addr[0];
	touch_debug(DEBUG_INFO, "Set i2c addr %x\n", addr[0]);
	return count;
}
#endif

static ssize_t show_debug_mesg(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "Debug level %d \n", debug);
}

static ssize_t show_gpio_int(struct device *dev,
                             struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", gpio_get_value(private_ts->intr_gpio));
}

static ssize_t show_reset(struct device *dev,
                          struct device_attribute *attr, char *buf)
{
	//struct i2c_client *client = to_i2c_client(dev);
	//struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);

	// add by leo for CTS test ++
	if((!ekth3260_debug) && (build_version != 1))
	{
		printk("[elan] %s: skip attribute to avoid CTS test (ekth3260_debug = %d)\n", __func__, ekth3260_debug);
		return sprintf(buf, "[elan] %s: skip attribute to avoid CTS test (ekth3260_debug = %d)\n", __func__, ekth3260_debug);
	}
	// add by leo for CTS test --

	elan_ktf_ts_hw_reset();

	return sprintf(buf, "Reset Touch Screen Controller \n");
}

static ssize_t show_enable_irq(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
	//	struct i2c_client *client = to_i2c_client(dev);
	//	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);

	// add by leo for CTS test ++
	if((!ekth3260_debug) && (build_version != 1))
	{
		printk("[elan] %s: skip attribute to avoid CTS test (ekth3260_debug = %d)\n", __func__, ekth3260_debug);
		return sprintf(buf, "[elan] %s: skip attribute to avoid CTS test (ekth3260_debug = %d)\n", __func__, ekth3260_debug);
	}
	// add by leo for CTS test --

	work_lock = 0;
	enable_irq(private_ts->client->irq);
	wake_unlock(&private_ts->wakelock);

	return sprintf(buf, "Enable IRQ \n");
}

static ssize_t show_disable_irq(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
	//	struct i2c_client *client = to_i2c_client(dev);
	//	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);

	// add by leo for CTS test ++
	if((!ekth3260_debug) && (build_version != 1))
	{
		printk("[elan] %s: skip attribute to avoid CTS test (ekth3260_debug = %d)\n", __func__, ekth3260_debug);
		return sprintf(buf, "[elan] %s: skip attribute to avoid CTS test (ekth3260_debug = %d)\n", __func__, ekth3260_debug);
	}
	// add by leo for CTS test --

	work_lock = 1;
	disable_irq(private_ts->client->irq);
	wake_lock(&private_ts->wakelock);

	return sprintf(buf, "Disable IRQ \n");
}

static ssize_t disable_hsync(struct device *dev,
                             struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret = 0;
	uint8_t cmd[] = {CMD_W_PKT, 0x8c, 0x03, 0x01};
	ret = i2c_master_send(client, cmd, sizeof(cmd));
	if(ret < 0)
	{
		dev_err(&client->dev,
		        "[elan] %s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	return sprintf(buf, "%s\n",
	               (ret >= 0) ? " [Elan] Disable Hsync Finish" : "[Elan] Disable Hsync Fail");
}

static ssize_t show_calibrate(struct device *dev,
                              struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret = 0;

	// add by leo for CTS test ++
	if((!ekth3260_debug) && (build_version != 1))
	{
		printk("[elan] %s: skip attribute to avoid CTS test (ekth3260_debug = %d)\n", __func__, ekth3260_debug);
		return sprintf(buf, "[elan] %s: skip attribute to avoid CTS test (ekth3260_debug = %d)\n", __func__, ekth3260_debug);
	}
	// add by leo for CTS test --

	ret = elan_ktf_ts_rough_calibrate(client);
	return sprintf(buf, "%s\n",
	               (ret == 0) ? " [Elan] Calibrate Finish" : "[Elan] Calibrate Fail");
}

static ssize_t show_fw_update_in_driver(struct device *dev,
                                        struct device_attribute *attr, char *buf)
{
	int ret;
	//struct i2c_client *client = to_i2c_client(dev);

	// add by leo for CTS test ++
	if((!ekth3260_debug) && (build_version != 1))
	{
		printk("[elan] %s: skip attribute to avoid CTS test (ekth3260_debug = %d)\n", __func__, ekth3260_debug);
		return sprintf(buf, "[elan] %s: skip attribute to avoid CTS test (ekth3260_debug = %d)\n", __func__, ekth3260_debug);
	}
	// add by leo for CTS test --

	work_lock = 1;
	disable_irq(private_ts->client->irq);
	wake_lock(&private_ts->wakelock);

	ret = Update_FW_in_Driver(0);

	work_lock = 0;
	enable_irq(private_ts->client->irq);
	wake_unlock(&private_ts->wakelock);

	printk("[elan] %s: update touch FW from '%s'. \n", __func__, ekth3260_update_outside == 0 ? "inside" : "outside");
	return sprintf(buf, "%d\n", ret);
}

static ssize_t show_fw_update(struct device *dev,
                              struct device_attribute *attr, char *buf)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);

	// add by leo for CTS test ++
	if((!ekth3260_debug) && (build_version != 1))
	{
		printk("[elan] %s: skip attribute to avoid CTS test (ekth3260_debug = %d)\n", __func__, ekth3260_debug);
		return sprintf(buf, "[elan] %s: skip attribute to avoid CTS test (ekth3260_debug = %d)\n", __func__, ekth3260_debug);
	}
	// add by leo for CTS test --

	work_lock = 1;
	disable_irq(private_ts->client->irq);
	wake_lock(&private_ts->wakelock);

	ret = FW_Update(client, 0);

	work_lock = 0;
	enable_irq(private_ts->client->irq);
	wake_unlock(&private_ts->wakelock);

	return sprintf(buf, "Update Firmware\n");
}

#ifdef ELAN_2WIREICE
static ssize_t show_2wire(struct device *dev,
                          struct device_attribute *attr, char *buf)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);

	// add by leo for CTS test ++
	if((!ekth3260_debug) && (build_version != 1))
	{
		printk("[elan] %s: skip attribute to avoid CTS test (ekth3260_debug = %d)\n", __func__, ekth3260_debug);
		return sprintf(buf, "[elan] %s: skip attribute to avoid CTS test (ekth3260_debug = %d)\n", __func__, ekth3260_debug);
	}
	// add by leo for CTS test --

	work_lock = 1;
	disable_irq(private_ts->client->irq);
	wake_lock(&private_ts->wakelock);

	ret = TWO_WIRE_ICE(client);

	work_lock = 0;
	enable_irq(private_ts->client->irq);
	wake_unlock(&private_ts->wakelock);

	return sprintf(buf, "Update Firmware by 2wire JTAG\n");
}
#endif

static ssize_t show_fw_version_value(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", ts->fw_ver);
}

static ssize_t show_fw_id_value(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", ts->fw_id);
}

static ssize_t show_bc_version_value(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);

	return sprintf(buf, "Bootcode:%d\n"
	               "IAP:%d\n", ts->fw_ver, ts->fw_id);
}

static ssize_t show_drv_version_value(struct device *dev,
                                      struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", "Elan driver version 0x0006");
}

static ssize_t show_iap_mode(struct device *dev,
                             struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);

	return sprintf(buf, "%s\n",
	               (ts->fw_ver == 0) ? "Recovery" : "Normal");
}

static ssize_t check_fw(struct device *dev,
                        struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	char fw_path[SYSFS_MAX_LEN];
	sscanf(buf, "%d %s", &ekth3260_update_outside, fw_path);
	touch_debug(DEBUG_INFO, "FW is %s\n", fw_path);

	ret = read_fw(fw_path);
	if(ret != 0)
	{
		touch_debug(DEBUG_ERROR, "Open FW file fail!\n");
		return count;
	}
	return count;
}

#ifdef GESTURE_MODE
static DEVICE_ATTR(gesture, S_IWUSR | S_IRUGO, show_gesture, set_gesture);
#endif
static DEVICE_ATTR(debug_mesg, S_IWUSR | S_IRUGO, show_debug_mesg, set_debug_mesg);
static DEVICE_ATTR(check_fw, S_IWUSR | S_IRUGO, NULL, check_fw);
static DEVICE_ATTR(gpio_int, S_IRUGO, show_gpio_int, NULL);
static DEVICE_ATTR(reset, S_IRUGO, show_reset, NULL);
static DEVICE_ATTR(enable_irq, S_IRUGO, show_enable_irq, NULL);
static DEVICE_ATTR(disable_irq, S_IRUGO, show_disable_irq, NULL);
static DEVICE_ATTR(calibrate, S_IRUGO, show_calibrate, NULL);
static DEVICE_ATTR(disable_hsync, S_IRUGO, disable_hsync, NULL);
static DEVICE_ATTR(fw_version, S_IRUGO, show_fw_version_value, NULL);
static DEVICE_ATTR(fw_id, S_IRUGO, show_fw_id_value, NULL);
static DEVICE_ATTR(bc_version, S_IRUGO, show_bc_version_value, NULL);
static DEVICE_ATTR(drv_version, S_IRUGO, show_drv_version_value, NULL);
static DEVICE_ATTR(fw_update_in_driver, S_IRUGO, show_fw_update_in_driver, NULL);
static DEVICE_ATTR(fw_update, S_IRUGO, show_fw_update, NULL);
#ifdef ELAN_2WIREICE
static DEVICE_ATTR(2wire, S_IRUGO, show_2wire, NULL);
#endif
static DEVICE_ATTR(iap_mode, S_IRUGO, show_iap_mode, NULL);

// add by leo ++
static ssize_t elan_touch_status_show(struct device *dev,
                                      struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ekth3260_touch_status);
}
static DEVICE_ATTR(touch_status, S_IRUGO, elan_touch_status_show, NULL);

static ssize_t elan_touch_fw_version_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	// add by leo for CTS test ++
	if((!ekth3260_debug) && (build_version != 1))
	{
		printk("[elan] %s: skip attribute to avoid CTS test (ekth3260_debug = %d)\n", __func__, ekth3260_debug);
		return sprintf(buf, "[elan] %s: skip attribute to avoid CTS test (ekth3260_debug = %d)\n", __func__, ekth3260_debug);
	}
	// add by leo for CTS test --

	work_lock = 1;
	disable_irq(private_ts->client->irq);
	wake_lock(&private_ts->wakelock);

	__fw_packet_handler(private_ts->client);

	work_lock = 0;
	enable_irq(private_ts->client->irq);
	wake_unlock(&private_ts->wakelock);

	return sprintf(buf, "0x%X_0x%X\n", FW_ID, FW_VERSION);
}
static DEVICE_ATTR(tp_fw_version, S_IRUGO, elan_touch_fw_version_show, NULL);

static ssize_t elan_rst_gpio_store(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
	int rst_pin = 0;
	struct elan_ktf_ts_data *ts = private_ts;

	sscanf(buf, "%d\n", &rst_pin);

	gpio_set_value(ts->rst_gpio, (rst_pin > 0 ? 1 : 0));
	printk("[elan] %s: set rst_pin = %s\n", __func__, (rst_pin > 0 ? "High" : "Low"));

	return count;
}
static ssize_t elan_rst_gpio_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int rst_pin = 0;
	struct elan_ktf_ts_data *ts = private_ts;

	rst_pin = gpio_get_value(ts->rst_gpio);
	printk("[elan] %s: get rst_pin = %d\n", __func__, rst_pin);

	return sprintf(buf, "rst_gpio = %s\n", (rst_pin > 0 ? "High" : "Low"));
}
DEVICE_ATTR(rst_gpio, S_IWUSR | S_IRUGO, elan_rst_gpio_show, elan_rst_gpio_store);

static ssize_t elan_touch_fw_check_show(struct device *dev,
                                        struct device_attribute *attr, char *buf)
{
	// add by leo for CTS test ++
	if((!ekth3260_debug) && (build_version != 1))
	{
		printk("[elan] %s: skip attribute to avoid CTS test (ekth3260_debug = %d)\n", __func__, ekth3260_debug);
		return sprintf(buf, "[elan] %s: skip attribute to avoid CTS test (ekth3260_debug = %d)\n", __func__, ekth3260_debug);
	}
	// add by leo for CTS test --

	work_lock = 1;
	disable_irq(private_ts->client->irq);
	wake_lock(&private_ts->wakelock);

	__fw_packet_handler(private_ts->client);

	work_lock = 0;
	enable_irq(private_ts->client->irq);
	wake_unlock(&private_ts->wakelock);

	eKTH3260_fw_version_check();
	printk("[elan] %s: ts->fwcheck_result =%d\n", __func__, private_ts->fwcheck_result);

	return sprintf(buf, "%d\n", private_ts->fwcheck_result);
}
static DEVICE_ATTR(tp_fw_check, S_IRUGO, elan_touch_fw_check_show, NULL);

static ssize_t elan_lcm_id0_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
	printk("[elan] %s: ts->LCM_ID0 = %d\n", __func__, private_ts->LCM_ID0);

	return sprintf(buf, "%d\n", private_ts->LCM_ID0);
}
static DEVICE_ATTR(lcm_id0, S_IRUGO, elan_lcm_id0_show, NULL);

static ssize_t elan_lcm_id2_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
	printk("[elan] %s: ts->LCM_ID2 = %d\n", __func__, private_ts->LCM_ID2);

	return sprintf(buf, "%d\n", private_ts->LCM_ID2);
}
static DEVICE_ATTR(lcm_id2, S_IRUGO, elan_lcm_id2_show, NULL);
//add by leo --

static struct attribute *elan_attributes[] =
{
#ifdef GESTURE_MODE
	&dev_attr_gesture.attr,
#endif
	&dev_attr_debug_mesg.attr,
	&dev_attr_check_fw.attr,
	&dev_attr_gpio_int.attr,
	&dev_attr_reset.attr,
	&dev_attr_enable_irq.attr,
	&dev_attr_disable_irq.attr,
	&dev_attr_calibrate.attr,
	&dev_attr_disable_hsync.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_fw_id.attr,
	&dev_attr_bc_version.attr,
	&dev_attr_drv_version.attr,
	&dev_attr_fw_update_in_driver.attr,
	&dev_attr_fw_update.attr,
#ifdef ELAN_2WIREICE
	&dev_attr_2wire.attr,
#endif
	&dev_attr_iap_mode.attr,
	// add by leo ++
	&dev_attr_rst_gpio.attr,
	&dev_attr_touch_status.attr,
	&dev_attr_tp_fw_version.attr,
	&dev_attr_tp_fw_check.attr,
	&dev_attr_lcm_id0.attr,
	&dev_attr_lcm_id2.attr,
	// add by leo --
	NULL
};

static struct attribute_group elan_attribute_group =
{
	.name = DEVICE_NAME,
	.attrs = elan_attributes,
};

// Start sysfs
static ssize_t elan_ktf_gpio_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
	int ret = 0;

	struct elan_ktf_ts_data *ts = private_ts;

	printk(KERN_ERR "[Elan] %s: START\n", __func__);

	ret = gpio_get_value(ts->intr_gpio);
	touch_debug(DEBUG_MESSAGES, "GPIO_TP_INT_N=%d\n", ts->intr_gpio);
	sprintf(buf, "GPIO_TP_INT_N=%d\n", ret);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(gpio, S_IRUGO, elan_ktf_gpio_show, NULL);

static ssize_t elan_ktf_vendor_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct elan_ktf_ts_data *ts = private_ts;

	sprintf(buf, "%s_x%4.4x\n", "elan_ktf", ts->fw_ver);
	ret = strlen(buf) + 1;
	return ret;
}

static DEVICE_ATTR(vendor, S_IRUGO, elan_ktf_vendor_show, NULL);

static struct kobject *android_touch_kobj;

static int elan_ktf_touch_sysfs_init(void)
{
	int ret ;

	android_touch_kobj = kobject_create_and_add("android_touch", NULL) ;
	if(android_touch_kobj == NULL)
	{
		touch_debug(DEBUG_ERROR, "[elan]%s: subsystem_register failed\n", __func__);
		ret = -ENOMEM;
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_gpio.attr);
	if(ret)
	{
		touch_debug(DEBUG_ERROR, "[elan]%s: sysfs_create_file failed\n", __func__);
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_vendor.attr);
	if(ret)
	{
		touch_debug(DEBUG_ERROR, "[elan]%s: sysfs_create_group failed\n", __func__);
		return ret;
	}
	return 0 ;
}

static void elan_touch_sysfs_deinit(void)
{
	sysfs_remove_file(android_touch_kobj, &dev_attr_vendor.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_gpio.attr);
	kobject_del(android_touch_kobj);
}

// end sysfs

static int elan_ktf_ts_get_data(struct i2c_client *client, uint8_t *cmd, uint8_t *buf, size_t w_size,  size_t r_size)
{
	int rc;

	dev_dbg(&client->dev, "[elan]%s: enter\n", __func__);

	if(buf == NULL)
		return -EINVAL;

	if((i2c_master_send(client, cmd, w_size)) != w_size)
	{
		dev_err(&client->dev,
		        "[elan]%s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	rc = elan_ktf_ts_poll(client);
	if(rc < 0)
		printk("%s: poll is hight\n", __func__);

	if(r_size <= 0) r_size = w_size;

	if(i2c_master_recv(client, buf, r_size) != r_size)	return -EINVAL;

	return 0;
}

static int __hello_packet_handler(struct i2c_client *client)
{
	int rc;
	uint8_t buf_recv[8] = { 0 };

	rc = elan_ktf_ts_poll(client);
	if(rc < 0)
	{
		printk("[elan] %s: Int poll failed!\n", __func__);
	}

	rc = i2c_master_recv(client, buf_recv, 8);
	printk("[elan] %s: hello packet %2x:%2x:%2x:%2x:%2x:%2x:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3] , buf_recv[4], buf_recv[5], buf_recv[6], buf_recv[7]);

	if(buf_recv[0] == 0x55 && buf_recv[1] == 0x55 && buf_recv[2] == 0x80 && buf_recv[3] == 0x80)
	{
		RECOVERY = 0x80;
		return RECOVERY;
	}

	/*Some Elan init don't need Re-Calibration */
	mdelay(300);
	rc = elan_ktf_ts_poll(client);
	if(rc < 0)
	{
		printk("[elan] %s: Int poll failed!\n", __func__);
	}
	rc = i2c_master_recv(client, buf_recv, 8);
	printk("[elan] %s: Try Re-Calibration packet %2x:%2x:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);

	if((buf_recv[0] == 0x55 && buf_recv[1] == 0x55 && buf_recv[2] == 0x55 && buf_recv[3] == 0x55) ||
	        (buf_recv[0] == 0x66 && buf_recv[1] == 0x66 && buf_recv[2] == 0x66 && buf_recv[3] == 0x66))
		ekth3260_touch_status = 1;

	return 0;
}

static int __fw_packet_handler(struct i2c_client *client)
{
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
	int rc;
	//int i;
	int major, minor;
	uint8_t cmd[] = {CMD_R_PKT, 0x00, 0x00, 0x01};/* Get Firmware Version*/
	//uint8_t info_buff[] = { 0x5b, 0x00, 0x00, 0x00, 0x00, 0x00 }; /*Get IC info*/
	//uint8_t info_buff_resp[17] = { 0 };
	uint8_t cmd_id[] = {0x53, 0xf0, 0x00, 0x01}; /*Get firmware ID*/
	uint8_t cmd_bc[] = {CMD_R_PKT, 0x01, 0x00, 0x01};/* Get BootCode Version*/
	uint8_t buf_recv[4] = {0};
	// Firmware version
	rc = elan_ktf_ts_get_data(client, cmd, buf_recv, 4, 4);
	if(rc < 0)
	{
		printk("Get Firmware version error\n");
	}
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_ver = major << 8 | minor;
	FW_VERSION = ts->fw_ver;
	// Firmware ID
	rc = elan_ktf_ts_get_data(client, cmd_id, buf_recv, 4, 4);
	if(rc < 0)
	{
		printk("Get Firmware ID error\n");
	}
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_id = major << 8 | minor;
	FW_ID = ts->fw_id;
	// Bootcode version
	rc = elan_ktf_ts_get_data(client, cmd_bc, buf_recv, 4, 4);
	if(rc < 0)
	{
		printk("Get Bootcode version error\n");
	}
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->bc_ver = major << 8 | minor;

	/*Get XY info*/
	X_RESOLUTION = 1472;
	Y_RESOLUTION = 2368;
	#if 0
	for(i = 0; i < 3; i++)
	{
		rc = elan_ktf_ts_get_data(client, info_buff, info_buff_resp, sizeof(info_buff), sizeof(info_buff_resp));
		if(rc < 0)
		{
			printk("Get XY info error\n");
		}

		if(info_buff_resp[0] == 0x9b)
		{
			ts->x_resolution = (info_buff_resp[2] + info_buff_resp[6]
			                    + info_buff_resp[10] + info_buff_resp[14] - 1) * 64;
			X_RESOLUTION = ts->x_resolution;

			ts->y_resolution = (info_buff_resp[3] + info_buff_resp[7]
			                    + info_buff_resp[11] + info_buff_resp[15] - 1) * 64;
			Y_RESOLUTION = ts->y_resolution;

			printk(KERN_INFO "[elan] %s: read resolution from touch FW.\n", __func__);
			break;
		}
		else
		{
			if(i >= 2)
			{
				printk(KERN_INFO "[elan] %s: set resolution directly by driver.\n", __func__);
				break;
			}
			else
				printk(KERN_INFO "[elan] %s: retry get resolution info.\n", __func__);
		}
	}
	#endif

	printk(KERN_INFO "[elan] %s: Firmware version: 0x%4.4x\n",
	       __func__, ts->fw_ver);
	printk(KERN_INFO "[elan] %s: Firmware ID: 0x%4.4x\n",
	       __func__, ts->fw_id);
	printk(KERN_INFO "[elan] %s: Bootcode Version: 0x%4.4x\n",
	       __func__, ts->bc_ver);
	printk(KERN_INFO "[elan] %s: x resolution: %d, y resolution: %d\n",
	       __func__, X_RESOLUTION, Y_RESOLUTION);

	return 0;
}

static inline int elan_ktf_ts_parse_xy(uint8_t *data,
                                       uint16_t *x, uint16_t *y)
{
	*x = *y = 0;

	*x = (data[0] & 0xf0);
	*x <<= 4;
	*x |= data[1];

	*y = (data[0] & 0x0f);
	*y <<= 8;
	*y |= data[2];

	return 0;
}

static int elan_ktf_ts_setup(struct i2c_client *client)
{
	int rc;

	rc = __hello_packet_handler(client);

	if(rc != 0x80)
	{
		rc = __fw_packet_handler(client);
		if(rc < 0)
			printk("[elan] %s, fw_packet_handler fail, rc = %d", __func__, rc);
		dev_dbg(&client->dev, "[elan] %s: firmware checking done.\n", __func__);
		//Check for FW_VERSION, if 0x0000 means FW update fail!
		if(FW_VERSION == 0x00)
		{
			rc = 0x80;
			printk("[elan] FW_VERSION = %d, last FW update fail\n", FW_VERSION);
		}
	}
	return rc;
}

static int elan_ktf_ts_rough_calibrate(struct i2c_client *client)
{
	uint8_t cmd[] = {CMD_W_PKT, 0x29, 0x00, 0x01};

	//dev_info(&client->dev, "[elan] %s: enter\n", __func__);
	printk("[elan] %s: enter\n", __func__);
	dev_info(&client->dev,
	         "[elan] dump cmd: %02x, %02x, %02x, %02x\n",
	         cmd[0], cmd[1], cmd[2], cmd[3]);

	if((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd))
	{
		dev_err(&client->dev,
		        "[elan] %s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}

#ifdef ELAN_POWER_SOURCE
static unsigned now_usb_cable_status = 0;

#if 0
static int elan_ktf_ts_hw_reset(struct i2c_client *client)
{
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
	touch_debug(DEBUG_INFO, "[ELAN] Start HW reset!\n");
	gpio_direction_output(ts->rst_gpio, 0);
	usleep_range(1000, 1500);
	gpio_direction_output(ts->rst_gpio, 1);
	msleep(5);
	return 0;
}

static int elan_ktf_ts_set_power_source(struct i2c_client *client, u8 state)
{
	uint8_t cmd[] = {CMD_W_PKT, 0x40, 0x00, 0x01};
	int length = 0;

	dev_dbg(&client->dev, "[elan] %s: enter\n", __func__);
	/*0x52 0x40 0x00 0x01  =>    Battery Mode
	0x52 0x41 0x00 0x01  =>   AC Adapter Mode
	0x52 0x42 0x00 0x01 =>    USB Mode */
	cmd[1] |= state & 0x0F;

	dev_dbg(&client->dev,
	        "[elan] dump cmd: %02x, %02x, %02x, %02x\n",
	        cmd[0], cmd[1], cmd[2], cmd[3]);

	down(&pSem);
	length = i2c_master_send(client, cmd, sizeof(cmd));
	up(&pSem);
	if(length != sizeof(cmd))
	{
		dev_err(&client->dev,
		        "[elan] %s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}



static void update_power_source()
{
	unsigned power_source = now_usb_cable_status;
	if(private_ts == NULL || work_lock) return;

	if(private_ts->abs_x_max == ELAN_X_MAX) //TF 700T device
		return; // do nothing for TF700T;

	touch_debug(DEBUG_INFO, "Update power source to %d\n", power_source);
	switch(power_source)
	{
		case USB_NO_Cable:
			elan_ktf_ts_set_power_source(private_ts->client, 0);
			break;
		case USB_Cable:
			elan_ktf_ts_set_power_source(private_ts->client, 1);
			break;
		case USB_AC_Adapter:
			elan_ktf_ts_set_power_source(private_ts->client, 2);
	}
}
#endif

void touch_callback(unsigned cable_status)
{
	now_usb_cable_status = cable_status;
	//update_power_source();
}
#endif

static int elan_ktf_ts_recv_data(struct i2c_client *client, uint8_t *buf, int bytes_to_recv)
{

	int rc;
	if(buf == NULL)
		return -EINVAL;

	memset(buf, 0, bytes_to_recv);

	/* The ELAN_PROTOCOL support normanl packet format */
#ifdef ELAN_PROTOCOL
	rc = i2c_master_recv(client, buf, bytes_to_recv);
	//printk("[elan] Elan protocol rc = %d \n", rc);
	if(rc != bytes_to_recv)
	{
		dev_err(&client->dev, "[elan] %s: i2c_master_recv error?! \n", __func__);
		return -1;
	}

#else
	rc = i2c_master_recv(client, buf, 8);
	if(rc != 8)
	{
		printk("[elan] Read the first package error.\n");
		mdelay(30);
		return -1;
	}
	printk("[elan_debug] %x %x %x %x %x %x %x %x\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
	mdelay(1);

	if(buf[0] == 0x6D)      //for five finger
	{
		rc = i2c_master_recv(client, buf + 8, 8);
		if(rc != 8)
		{
			printk("[elan] Read the second package error.\n");
			mdelay(30);
			return -1;
		}
		printk("[elan_debug] %x %x %x %x %x %x %x %x\n", buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]);
		rc = i2c_master_recv(client, buf + 16, 2);
		if(rc != 2)
		{
			printk("[elan] Read the third package error.\n");
			mdelay(30);
			return -1;
		}
		mdelay(1);
		printk("[elan_debug] %x %x \n", buf[16], buf[17]);
	}
#endif
	//printk("[elan_debug] end ts_work\n");
	return rc;
}

#ifdef PROTOCOL_B
/* Protocol B  */
static int mTouchStatus[FINGER_NUM] = {0};  /* finger_num=10 */
void force_release_pos(struct i2c_client *client)
{
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
	int i;
	for(i = 0; i < FINGER_NUM; i++)
	{
		if(mTouchStatus[i] == 0) continue;
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
		mTouchStatus[i] = 0;
	}

	input_sync(ts->input_dev);
}

static void elan_ktf_ts_report_data(struct i2c_client *client, uint8_t *buf)
{
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
	struct input_dev *idev = ts->input_dev;
	uint16_t x, y, touch_size, pressure_size;
	uint16_t fbits = 0;
	uint8_t i, num;
	uint16_t active = 0;
	uint8_t idx, btn_idx;
	int finger_num;
	static uint8_t size_index[10] = {35, 35, 36, 36, 37, 37, 38, 38, 39, 39};

	//  int j;
	//int checksum = 0;
	//int pressure_size=80;

	/* for 10 fingers */
	if(buf[0] == TEN_FINGERS_PKT)
	{
		finger_num = 10;
		num = buf[2] & 0x0f;
		fbits = buf[2] & 0x30;
		fbits = (fbits << 4) | buf[1];
		idx = 3;
		btn_idx = 33;
	}
	/* for 5 fingers  */
	else if((buf[0] == MTK_FINGERS_PKT) || (buf[0] == FIVE_FINGERS_PKT))
	{
		finger_num = 5;
		num = buf[1] & 0x07;
		fbits = buf[1] >> 3;
		idx = 2;
		btn_idx = 17;
	}
	else
	{
		/* for 2 fingers */
		finger_num = 2;
		num = buf[7] & 0x03;    // for elan old 5A protocol the finger ID is 0x06
		fbits = buf[7] & 0x03;
		//        fbits = (buf[7] & 0x03) >> 1; // for elan old 5A protocol the finger ID is 0x06
		idx = 1;
		btn_idx = 7;
	}

	switch(buf[0])
	{
		case MTK_FINGERS_PKT:
		case TWO_FINGERS_PKT:
		case FIVE_FINGERS_PKT:
		case TEN_FINGERS_PKT:

			for(i = 0; i < finger_num; i++)
			{
				active = fbits & 0x1;
				if(active || mTouchStatus[i])
				{
					input_mt_slot(ts->input_dev, i);
					input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, active);
					if(active)
					{
						elan_ktf_ts_parse_xy(&buf[idx], &x, &y);
						//y=Y_RESOLUTION -y; // mark by leo
						touch_size = ((i & 0x01) ? buf[size_index[i]] : (buf[size_index[i]] >> 4)) & 0x0F;
						pressure_size = touch_size << 4; // shift left touch size value to 4 bits for max pressure value 255
						input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 100);
						input_report_abs(idev, ABS_MT_PRESSURE, 100);
						input_report_abs(idev, ABS_MT_POSITION_X, x);
						input_report_abs(idev, ABS_MT_POSITION_Y, y);
						if(debug > 5)printk("[elan] finger id=%d x=%d y=%d size=%d press=%d \n", i, x, y, touch_size, pressure_size); // add by leo
						if(unlikely(gPrint_point))
							touch_debug(DEBUG_INFO, "[elan] finger id=%d x=%d y=%d size=%d press=%d \n", i, x, y, touch_size, pressure_size);

#if TOUCH_SYSTRACE_TEST
						queue_work(private_ts->elan_wq, &private_ts->work);
#endif
					}
				}
				mTouchStatus[i] = active;
				fbits = fbits >> 1;
				idx += 3;
			}
			if(num == 0)
			{
				//printk("[ELAN] ALL Finger Up\n");
				input_report_key(idev, BTN_TOUCH, 0); //for all finger up
				force_release_pos(client);
			}
			input_sync(idev);
			break;
		case 78://0512
			touch_debug(DEBUG_TRACE, "%x %x %x %x\n", buf[0], buf[1], buf[2], buf[3]);
			break;
		default:
			if(fail_count < 100)
			{
				dev_err(&client->dev,	"[elan] %s: unknown packet type: %0x, times: %d\n", __func__, buf[0], fail_count);
				fail_count++;
			}
			break;
	} // end switch

	return;
}

#endif

#ifdef PROTOCOL_A

/* Protocol A  */
static void elan_ktf_ts_report_data(struct i2c_client *client, uint8_t *buf)
{
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
	struct input_dev *idev = ts->input_dev;
	uint16_t x, y;
	uint16_t fbits = 0;
	uint8_t i, num, reported = 0;
	uint8_t idx, btn_idx;
	int finger_num;

	/* for 10 fingers	*/
	if(buf[0] == TEN_FINGERS_PKT)
	{
		finger_num = 10;
		num = buf[2] & 0x0f;
		fbits = buf[2] & 0x30;
		fbits = (fbits << 4) | buf[1];
		idx = 3;
		btn_idx = 33;
	}
	/* for 5 fingers	*/
	else if((buf[0] == MTK_FINGERS_PKT) || (buf[0] == FIVE_FINGERS_PKT))
	{
		finger_num = 5;
		num = buf[1] & 0x07;
		fbits = buf[1] >> 3;
		idx = 2;
		btn_idx = 17;
	}
	else
	{
		/* for 2 fingers */
		finger_num = 2;
		num = buf[7] & 0x03;		// for elan old 5A protocol the finger ID is 0x06
		fbits = buf[7] & 0x03;
		//        fbits = (buf[7] & 0x03) >> 1;	// for elan old 5A protocol the finger ID is 0x06
		idx = 1;
		btn_idx = 7;
	}

	switch(buf[0])
	{
		case MTK_FINGERS_PKT:
		case TWO_FINGERS_PKT:
		case FIVE_FINGERS_PKT:
		case TEN_FINGERS_PKT:
			//input_report_key(idev, BTN_TOUCH, 1);
			if(num == 0)
			{
				input_report_key(idev, BTN_TOUCH, 0);
#ifdef ELAN_BUTTON
				if(buf[btn_idx] == 0x21)
				{
					button_state = 0x21;
					input_report_key(idev, KEY_BACK, 1);
					input_report_key(idev, KEY_BACK, 0);
					printk("[elan_debug] button %x \n", buf[btn_idx]);
				}
				else if(buf[btn_idx] == 0x41)
				{
					button_state = 0x41;
					input_report_key(idev, KEY_HOME, 1);
				}
				else if(buf[btn_idx] == 0x81)
				{
					button_state = 0x81;
					input_report_key(idev, KEY_MENU, 1);
				}
				else if(button_state == 0x21)
				{
					button_state = 0;
					input_report_key(idev, KEY_BACK, 0);
				}
				else if(button_state == 0x41)
				{
					button_state = 0;
					input_report_key(idev, KEY_HOME, 0);
				}
				else if(button_state == 0x81)
				{
					button_state = 0;
					input_report_key(idev, KEY_MENU, 0);
				}
				else
				{
					dev_dbg(&client->dev, "no press\n");
					input_mt_sync(idev);
				}
#endif
			}
			else
			{
				dev_dbg(&client->dev, "[elan] %d fingers\n", num);
				input_report_key(idev, BTN_TOUCH, 1);
				for(i = 0; i < finger_num; i++)
				{
					if((fbits & 0x01))
					{
						// james			    	elan_ktf_ts_parse_xy(&buf[idx], &x, &y);
						elan_ktf_ts_parse_xy(&buf[idx], &y, &x);
						//x = X_RESOLUTION-x;
						y = Y_RESOLUTION - y;
						//printk("[elan_debug] %s, x=%x, y=%x\n",__func__, x , y);
						input_report_abs(idev, ABS_MT_TRACKING_ID, i);
						input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 100);
						input_report_abs(idev, ABS_MT_PRESSURE, 80);
						input_report_abs(idev, ABS_MT_POSITION_X, x);
						input_report_abs(idev, ABS_MT_POSITION_Y, y);
						input_mt_sync(idev);
						reported++;
						if(debug > 5)printk("[elan] finger id=%d x=%d y=%d \n", i, x, y); // add by leo
						if(unlikely(gPrint_point))
							touch_debug(DEBUG_INFO, "[elan] finger id=%d x=%d y=%d \n", i, x, y);
					} // end if finger status
					fbits = fbits >> 1;
					idx += 3;

				} // end for
			}
			if(reported)
				input_sync(idev);
			else
			{
				input_mt_sync(idev);
				input_sync(idev);
			}

			break;
		default:
			if(fail_count < 100)
			{
				dev_err(&client->dev,	"[elan] %s: unknown packet type: %0x, times: %d\n", __func__, buf[0], fail_count);
				fail_count++;
			}
			break;
	} // end switch

	return;
}
#endif

#ifdef GESTURE_MODE
static inline int elan_ktf_ts_gesture_double_click_xy(uint8_t *data,
        uint16_t *x1, uint16_t *y1,
        uint16_t *x2, uint16_t *y2)
{
	*x1 = *y1 = *x2 = *y2 = 0;

	*x1 = (data[3] << 8) | data[4];
	*y1 = (data[5] << 8) | data[6];
	*x2 = (data[7] << 8) | data[8];
	*y2 = (data[9] << 8) | data[10];

	return 0;
}

#if REPORT_GESTURE_DCLICK_POINT
static void elan_ktf_ts_double_click_report(struct i2c_client *client, uint16_t x, uint16_t y, int active)
{
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
	struct input_dev *idev = ts->input_dev;
	if(active)
	{
		input_mt_slot(idev, 0);
		input_mt_report_slot_state(idev, MT_TOOL_FINGER, 1);
		input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 100);
		input_report_abs(idev, ABS_MT_PRESSURE, 100);
		input_report_abs(idev, ABS_MT_POSITION_X, x);
		input_report_abs(idev, ABS_MT_POSITION_Y, y);
		printk("[elan] %s: finger id=%d x=%d y=%d \n", __func__, 0, x, y);
	}

	if(active == 0)
	{
		//input_report_key(idev, BTN_TOUCH, 0); // for all finger up
		input_mt_slot(idev, 0);
		input_mt_report_slot_state(idev, MT_TOOL_FINGER, 0);
		printk("[elan] %s: dobule click up \n", __func__);
	}
	input_sync(idev);
}
#endif

static void elan_ktf_ts_gesture_report_data(struct i2c_client *client, uint8_t *buf)
{
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
	uint16_t x1, y1, x2, y2;

	printk("[elan] %s: ts->gesture = 0x%02x\n", __func__, ts->gesture);
	switch(buf[1])
	{
		case ELAN_GESTURE_DOUBLE_CLICK:
			if(ts->gesture & GESTURE_DOUBLE_CLICK)
			{
				elan_ktf_ts_gesture_double_click_xy(&buf[0], &x1, &y1, &x2, &y2);
#if REPORT_GESTURE_DCLICK_POINT
				elan_ktf_ts_double_click_report(client, x1, y1, 1);
				elan_ktf_ts_double_click_report(client, x1, y1, 0);
				elan_ktf_ts_double_click_report(client, x2, y2, 1);
				elan_ktf_ts_double_click_report(client, x2, y2, 0);
#endif
#if 0
				input_report_key(ts->input_dev, KEY_POWER, 1);
				input_sync(ts->input_dev);
				msleep(5);
				input_report_key(ts->input_dev, KEY_POWER, 0);
				input_sync(ts->input_dev);
#else
				input_report_key(ts->input_dev, KEY_F24, 1);
				input_sync(ts->input_dev);
				input_report_key(ts->input_dev, KEY_F24, 0);
				input_sync(ts->input_dev);
#endif
				printk("[elan] %s: GESTURE_DOUBLE_CLICK %x\n", __func__, buf[1]);
			}
			else
				printk("[elan] %s: SKIP.. GESTURE_DOUBLE_CLICK %x\n", __func__, buf[1]);
			break;

		case ELAN_GESTURE_W:
			if(ts->gesture & GESTURE_W)
			{
#if 0
				input_report_key(ts->input_dev, KEY_POWER, 1);
				input_sync(ts->input_dev);
				msleep(5);
				input_report_key(ts->input_dev, KEY_POWER, 0);
				input_sync(ts->input_dev);
#else
				input_report_key(ts->input_dev, KEY_F23, 1);
				input_sync(ts->input_dev);
				input_report_key(ts->input_dev, KEY_F23, 0);
				input_sync(ts->input_dev);
#endif
				printk("[elan] %s: GESTURE_W %x\n", __func__, buf[1]);
			}
			else
				printk("[elan] %s: SKIP.. GESTURE_W %x\n", __func__, buf[1]);
			break;

		case ELAN_GESTURE_S:
			if(ts->gesture & GESTURE_S)
			{
#if 0
				input_report_key(ts->input_dev, KEY_POWER, 1);
				input_sync(ts->input_dev);
				msleep(5);
				input_report_key(ts->input_dev, KEY_POWER, 0);
				input_sync(ts->input_dev);
#else
				input_report_key(ts->input_dev, KEY_F22, 1);
				input_sync(ts->input_dev);
				input_report_key(ts->input_dev, KEY_F22, 0);
				input_sync(ts->input_dev);
#endif
				printk("[elan] %s: GESTURE_S %x\n", __func__, buf[1]);
			}
			else
				printk("[elan] %s: SKIP.. GESTURE_S %x\n", __func__, buf[1]);
			break;

		case ELAN_GESTURE_E:
			if(ts->gesture & GESTURE_E)
			{
#if 0
				input_report_key(ts->input_dev, KEY_POWER, 1);
				input_sync(ts->input_dev);
				msleep(5);
				input_report_key(ts->input_dev, KEY_POWER, 0);
				input_sync(ts->input_dev);
#else
				input_report_key(ts->input_dev, KEY_F21, 1);
				input_sync(ts->input_dev);
				input_report_key(ts->input_dev, KEY_F21, 0);
				input_sync(ts->input_dev);
#endif
				printk("[elan] %s: GESTURE_E %x\n", __func__, buf[1]);
			}
			else
				printk("[elan] %s: SKIP.. GESTURE_E %x\n", __func__, buf[1]);
			break;

		case ELAN_GESTURE_C:
			if(ts->gesture & GESTURE_C)
			{
#if 0
				input_report_key(ts->input_dev, KEY_POWER, 1);
				input_sync(ts->input_dev);
				msleep(5);
				input_report_key(ts->input_dev, KEY_POWER, 0);
				input_sync(ts->input_dev);
#else
				input_report_key(ts->input_dev, KEY_F20, 1);
				input_sync(ts->input_dev);
				input_report_key(ts->input_dev, KEY_F20, 0);
				input_sync(ts->input_dev);
#endif
				printk("[elan] %s: GESTURE_C %x\n", __func__, buf[1]);
			}
			else
				printk("[elan] %s: SKIP.. GESTURE_C %x\n", __func__, buf[1]);
			break;

		case ELAN_GESTURE_Z:
			if(ts->gesture & GESTURE_Z)
			{
#if 0
				input_report_key(ts->input_dev, KEY_POWER, 1);
				input_sync(ts->input_dev);
				msleep(5);
				input_report_key(ts->input_dev, KEY_POWER, 0);
				input_sync(ts->input_dev);
#else
				input_report_key(ts->input_dev, KEY_F19, 1);
				input_sync(ts->input_dev);
				input_report_key(ts->input_dev, KEY_F19, 0);
				input_sync(ts->input_dev);
#endif
				printk("[elan] %s: GESTURE_Z %x\n", __func__, buf[1]);
			}
			else
				printk("[elan] %s: SKIP.. GESTURE_Z %x\n", __func__, buf[1]);
			break;

		case ELAN_GESTURE_V:
			if(ts->gesture & GESTURE_V)
			{
#if 0
				input_report_key(ts->input_dev, KEY_POWER, 1);
				input_sync(ts->input_dev);
				msleep(5);
				input_report_key(ts->input_dev, KEY_POWER, 0);
				input_sync(ts->input_dev);
#else
				input_report_key(ts->input_dev, KEY_F18, 1);
				input_sync(ts->input_dev);
				input_report_key(ts->input_dev, KEY_F18, 0);
				input_sync(ts->input_dev);
#endif
				printk("[elan] %s: GESTURE_V %x\n", __func__, buf[1]);
			}
			else
				printk("[elan] %s: SKIP.. GESTURE_V %x\n", __func__, buf[1]);
			break;

		default:
			printk("[elan] %s: unknow packet %x\n", __func__, buf[1]);
			return;
	}
	return;
}
#endif

#ifdef ESD_CHECK
static void elan_ktf_ts_check_work_func(struct work_struct *work)
{

	int res = 0;

	if(live_state == 1)
	{
		live_state = 0;
		schedule_delayed_work(&private_ts->check_work, msecs_to_jiffies(2500));
		return;
	}

	printk(KERN_EMERG "%s, chip may crash, we need to reset it \n", __func__);

	elan_ktf_ts_hw_reset();
	res = __hello_packet_handler(private_ts->client);

	if(res != 0)
	{
		printk(KERN_INFO "Receive hello package fail\n");
	}

	schedule_delayed_work(&private_ts->check_work, msecs_to_jiffies(2500));
}
#endif

static void elan_ktf_ts_work_func(struct work_struct *work)
{
	int rc = 0, i = 0;
	struct elan_ktf_ts_data *ts = private_ts;
	uint8_t buf[PACKET_SIZE] = { 0 };

	disable_irq(ts->client->irq);
	wake_lock(&private_ts->wakelock);
	/*
	if (gpio_get_value(ts->intr_gpio))
	{
		printk("[elan] Detected the jitter on INT pin.\n");
		return IRQ_HANDLED;
	}
	*/

	if(ekth3260_touch_status != 1)
	{
		enable_irq(ts->client->irq);
		wake_unlock(&private_ts->wakelock);
		return;
	}

	rc = elan_ktf_ts_recv_data(ts->client, buf, PACKET_SIZE);
	if(rc < 0)
	{
		// add by leo for testtest ++
		printk("[elan] ");
		for(i = 0; i < PACKET_SIZE; i++)
		{
			printk("%2x, ", buf[i]);
		}
		printk("\n");
		// add by leo for testtest --
		printk("[elan] Received the packet Error.\n");
		enable_irq(ts->client->irq);
		wake_unlock(&private_ts->wakelock);
		return;
	}
	//printk("[elan_debug] %2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x ....., %2x\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[17]);

#ifdef GESTURE_MODE
	if(buf[0] == GESTURE_PKT)
	{
		elan_ktf_ts_gesture_report_data(ts->client, buf);
		wake_unlock(&private_ts->wakelock);
		enable_irq(ts->client->irq);
		return;
	}
#endif

#ifndef ELAN_BUFFER_MODE
	elan_ktf_ts_report_data(ts->client, buf);
#else
	elan_ktf_ts_report_data(ts->client, buf + 4);

	// Second package
	if(((buf[0] == 0x63) || (buf[0] == 0x66)) && ((buf[1] == 2) || (buf[1] == 3)))
	{
		rc = elan_ktf_ts_recv_data(ts->client, buf1, PACKET_SIZE);
		if(rc < 0)
		{
			wake_unlock(&private_ts->wakelock);
			enable_irq(ts->client->irq);
			return;
		}
		elan_ktf_ts_report_data(ts->client, buf1);
		// Final package
		if(buf[1] == 3)
		{
			rc = elan_ktf_ts_recv_data(ts->client, buf1, PACKET_SIZE);
			if(rc < 0)
			{
				wake_unlock(&private_ts->wakelock);
				enable_irq(ts->client->irq);
				return;
			}
			elan_ktf_ts_report_data(ts->client, buf1);
		}
	}
#endif
	enable_irq(ts->client->irq);
	wake_unlock(&private_ts->wakelock);

	return;
}

static irqreturn_t elan_ktf_ts_irq_handler(int irq, void *dev_id)
{
#if 0
	struct elan_ktf_ts_data *ts = private_ts;
	queue_work(ts->elan_irq_wq, &ts->irq_work);

	return IRQ_HANDLED;
#else
	int rc = 0;
	//int i = 0;
#if I2C_FAILD_RETRY
	int recv_retry = 0;
#endif
	struct elan_ktf_ts_data *ts = dev_id;
	uint8_t buf[PACKET_SIZE] = { 0 };
	// add by leo for hello package in gesture mode ++
#if HELLOW_IN_GESTURE_WORKAROUND
	int ret = 0;
	uint8_t gesture_wake_up_cmd[] = {0x54, 0x40, 0x01, 0x01};
	uint8_t gesture_mode_check_cmd[] = {0x53, 0x40, 0x00, 0x01};
	uint8_t buf_recv[4] = { 0 };
#endif
	// add by leo for hello package in gesture mode --

	/*
	if (gpio_get_value(ts->intr_gpio))
	{
		printk("[elan] Detected the jitter on INT pin.\n");
		return IRQ_HANDLED;
	}
	*/
#ifdef ESD_CHECK
	live_state = 1;
#endif

	disable_irq_nosync(ts->client->irq);
	wake_lock(&private_ts->wakelock);

	if(ekth3260_touch_status != 1)
	{
		enable_irq(ts->client->irq);
		wake_unlock(&private_ts->wakelock);
		return IRQ_HANDLED;
	}

	rc = elan_ktf_ts_recv_data(ts->client, buf, PACKET_SIZE);
	if(rc < 0)
	{
		printk("[elan] Received the packet Error.\n");

#if I2C_FAILD_RETRY
		do
		{
			if(debug > 20)printk("[elan] ////////////////////		RETRY %d times !!\n", recv_retry);
			if(ts->system_suspend)
				mdelay(100);
			else
				mdelay(10);

			rc = elan_ktf_ts_recv_data(ts->client, buf, PACKET_SIZE);
			if(rc < 0)
			{
				if(debug > 20)printk("[elan] RETRY %d: ", recv_retry);
				for(i = 0; i < PACKET_SIZE; i++)
				{
					if(debug > 20)printk("%2x, ", buf[i]);
				}
				if(debug > 20)printk("\n");

				if(debug > 20)printk("[elan] RETRY %d Received the packet Error.\n", recv_retry);
				recv_retry++;
			}
			else
			{
				printk("[elan] RETRY %d times pass.\n", recv_retry);
				break;
			}

		} while(recv_retry < 10);
		if(recv_retry >= 10)
		{
			enable_irq(ts->client->irq);
			wake_unlock(&private_ts->wakelock);
			return IRQ_HANDLED;
		}
#else
		enable_irq(ts->client->irq);
		wake_unlock(&private_ts->wakelock);
		return IRQ_HANDLED;
#endif
	}

	// add by leo for testtest ++
#if 0
	printk("[elan] ");
	for(i = 0; i < PACKET_SIZE; i++)
	{
		printk("%2x, ", buf[i]);
	}
	printk("\n");
#endif
	// add by leo for testtest --

	//printk("[elan_debug] %2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x ....., %2x\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[17]);

	// add by leo for Hsync count check ++
	if(buf[0] == HSYNC_PKT)
	{
		if(hsync_count >= 1000)
			hsync_count = 1;
		else
			hsync_count++;
		printk("[elan] %s: into HSYNC count = %d\n", __func__, hsync_count);
		enable_irq(ts->client->irq);
		wake_unlock(&private_ts->wakelock);
		return IRQ_HANDLED;
	}
	// add by leo for Hsync count check ++

	// add by leo for g package in gesture mode ++
#if HELLOW_IN_GESTURE_WORKAROUND
	//printk("[elan] %s: gesture mode check ...\n", __func__);
	if((ts->gesture & GESTURE_ENABLE) && (ts->system_suspend)
	        && ((buf[0] == 0x55 && buf[1] == 0x55 && buf[2] == 0x55 && buf[3] == 0x55)))
	{
		ret = i2c_master_send(ts->client, gesture_wake_up_cmd, sizeof(gesture_wake_up_cmd));
		if(ret < 0)
			printk("[elan] %s: re-send gesture_wake_up_cmd fail\n", __func__);

		msleep(2);

		ret = i2c_master_send(ts->client, gesture_mode_check_cmd, sizeof(gesture_mode_check_cmd));
		if(ret < 0)
			printk("[elan] %s: re-send gesture_mode_check_cmd fail\n", __func__);

		msleep(2);

		rc = i2c_master_recv(ts->client, buf_recv, 4);
		if(rc != 4)
			printk("[elan] %s: re-read gesture mode state error.\n", __func__);

		printk("[elan] %s: re-enter gesture mode check: %2x:%2x:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);

		enable_irq(ts->client->irq);
		wake_unlock(&private_ts->wakelock);
		return IRQ_HANDLED;
	}
#endif
	// add by leo for hello package in gesture mode --

#ifdef GESTURE_MODE
	if(buf[0] == GESTURE_PKT)
	{
		elan_ktf_ts_gesture_report_data(ts->client, buf);
		enable_irq(ts->client->irq);
		wake_unlock(&private_ts->wakelock);
		return IRQ_HANDLED;
	}
#endif

#ifndef ELAN_BUFFER_MODE
	elan_ktf_ts_report_data(ts->client, buf);
#else
	elan_ktf_ts_report_data(ts->client, buf + 4);

	// Second package
	if(((buf[0] == 0x63) || (buf[0] == 0x66)) && ((buf[1] == 2) || (buf[1] == 3)))
	{
		rc = elan_ktf_ts_recv_data(ts->client, buf1, PACKET_SIZE);
		if(rc < 0)
		{
			enable_irq(ts->client->irq);
			wake_unlock(&private_ts->wakelock);
			return IRQ_HANDLED;
		}
		elan_ktf_ts_report_data(ts->client, buf1);
		// Final package
		if(buf[1] == 3)
		{
			rc = elan_ktf_ts_recv_data(ts->client, buf1, PACKET_SIZE);
			if(rc < 0)
			{
				enable_irq(ts->client->irq);
				wake_unlock(&private_ts->wakelock);
				return IRQ_HANDLED;
			}
			elan_ktf_ts_report_data(ts->client, buf1);
		}
	}
#endif

	enable_irq(ts->client->irq);
	wake_unlock(&private_ts->wakelock);

	return IRQ_HANDLED;
#endif
}

static int elan_ktf_ts_register_interrupt(struct i2c_client *client)
{
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
	int err = 0;

	err = request_threaded_irq(client->irq, NULL, elan_ktf_ts_irq_handler,
	                           IRQF_TRIGGER_LOW /*| IRQF_TRIGGER_FALLING*/ | IRQF_ONESHOT,
	                           client->name, ts);
	if(err)
	{
		dev_err(&client->dev, "[elan] %s: request_irq %d failed\n",
		        __func__, client->irq);
	}

	enable_irq_wake(client->irq);
	return err;
}

#ifdef _ENABLE_DBG_LEVEL
static int ektf_proc_read(char *buffer, char **buffer_location, off_t offset, int buffer_length, int *eof, void *data)
{
	int ret;

	touch_debug(DEBUG_MESSAGES, "call proc_read\n");

	if(offset > 0)  /* we have finished to read, return 0 */
		ret  = 0;
	else
		ret = sprintf(buffer, "Debug Level: Release Date: %s\n", "2011/10/05");

	return ret;
}


static int ektf_proc_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
	char procfs_buffer_size = 0;
	int i, ret = 0;
	unsigned char procfs_buf[PROC_FS_MAX_LEN + 1] = {0};
	unsigned int command;

	procfs_buffer_size = count;
	if(procfs_buffer_size > PROC_FS_MAX_LEN)
		procfs_buffer_size = PROC_FS_MAX_LEN + 1;

	if(copy_from_user(procfs_buf, buffer, procfs_buffer_size))
	{
		touch_debug(DEBUG_ERROR, " proc_write faied at copy_from_user\n");
		return -EFAULT;
	}

	command = 0;
	for(i = 0; i < procfs_buffer_size - 1; i++)
	{
		if(procfs_buf[i] >= '0' && procfs_buf[i] <= '9')
			command |= (procfs_buf[i] - '0');
		else if(procfs_buf[i] >= 'A' && procfs_buf[i] <= 'F')
			command |= (procfs_buf[i] - 'A' + 10);
		else if(procfs_buf[i] >= 'a' && procfs_buf[i] <= 'f')
			command |= (procfs_buf[i] - 'a' + 10);

		if(i != procfs_buffer_size - 2)
			command <<= 4;
	}

	command = command & 0xFFFFFFFF;
	switch(command)
	{
		case 0xF1:
			gPrint_point = 1;
			break;
		case 0xF2:
			gPrint_point = 0;
			break;
		case 0xFF:
			ret = elan_ktf_ts_rough_calibrate(private_ts->client);
			break;
	}
	touch_debug(DEBUG_INFO, "Run command: 0x%08X  result:%d\n", command, ret);

	return count; // procfs_buffer_size;
}
#endif // #ifdef _ENABLE_DBG_LEV

#if 0
#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
                                unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	pm_message_t mesg;
	int *blank;
	struct elan_ktf_ts_data *elan_dev_data =
	    container_of(self, struct elan_ktf_ts_data, fb_notif);
	printk("%s fb notifier callback\n", __func__);
	if(evdata && evdata->data && elan_dev_data && private_ts->client)
	{
		if(event == FB_EVENT_BLANK)
		{
			blank = evdata->data;
			if(*blank == FB_BLANK_UNBLANK)
			{
				printk("resume\n");
				elan_ktf_ts_resume(private_ts->client);
			}
			else if(*blank == FB_BLANK_POWERDOWN)
			{
				printk("suspend\n");
				elan_ktf_ts_suspend(private_ts->client, mesg);
			}
		}
	}

	return 0;
}
#endif
#endif

// add by leo for testtest ++
static int eKTH3260_parse_dt(struct device *dev, struct elan_ktf_ts_data *ts)
{
	struct device_node *np = dev->of_node;

	/* reset, irq gpio info */
	ts->rst_gpio = of_get_named_gpio_flags(np, "elan,reset-gpio", 0, &ts->rst_gpio_flags);
	if(ts->rst_gpio < 0)
		return ts->rst_gpio;

	ts->intr_gpio = of_get_named_gpio_flags(np, "elan,intr-gpio", 0, &ts->intr_gpio_flags);
	if(ts->intr_gpio < 0)
		return ts->intr_gpio;

	ts->power_gpio = of_get_named_gpio_flags(np, "elan,power-gpio", 0, &ts->power_gpio_flags);
	if(ts->power_gpio < 0)
		return ts->power_gpio;

	ts->lcmid0_gpio = of_get_named_gpio_flags(np, "elan,lcmid0-gpio", 0, &ts->lcmid0_gpio_flags);
	if(ts->lcmid0_gpio < 0)
		return ts->lcmid0_gpio;

	ts->lcmid2_gpio = of_get_named_gpio_flags(np, "elan,lcmid2-gpio", 0, &ts->lcmid2_gpio_flags);
	if(ts->lcmid2_gpio < 0)
		return ts->lcmid2_gpio;

	printk("[elan] %s: ts->rst_gpio_gpio=%d, ts->intr_gpio=%d,  ts->power_gpio=%d, ts->lcmid0_gpio=%d, ts->lcmid2_gpio=%d\n",
	       __func__, ts->rst_gpio, ts->intr_gpio, ts->power_gpio, ts->lcmid0_gpio, ts->lcmid2_gpio); // add by leo for testtest
	return 0;
}

static int eKTH3260_fw_version_check(void)
{
	if(private_ts->LCM_ID2) // AUO LCM
	{
		if((FW_ID != 0x3037) || (FW_VERSION != 0x5514))
			private_ts->fwcheck_result = 0;
		else
			private_ts->fwcheck_result = 1;
	}
	else // CPT LCM
	{
		if((FW_ID != 0x3038) || (FW_VERSION != 0x5514))
			private_ts->fwcheck_result = 0;
		else
			private_ts->fwcheck_result = 1;
	}
	return 0;
}

static uint8_t new_sensor_cmd[4] = {0x54, 0x8b, 0x01, 0x01};
static uint8_t old_sensor_cmd[4] = {0x54, 0x8b, 0x00, 0x01};
static uint8_t sensor_cmd[4] = {0};

static void elan_sensor_set(struct work_struct *work)
{
	int ret = 0;

	printk("[elan] %s: %x %x %x %x\n", __func__, sensor_cmd[0], sensor_cmd[1], sensor_cmd[2], sensor_cmd[3]);
	ret = i2c_master_send(private_ts->client, sensor_cmd, sizeof(sensor_cmd));
	if(ret < 0)
		printk("[elan] %s: send sensor_cmd cmd fail\n", __func__);
	return;
}

// add by leo for cable status notifier ++
// 0: TYPE_NONE
// 1: CHARGER IN
#define TYPE_NONE	0
#define AC_IN	1
#define PC_IN	2
#define POWERBANK_IN	3
static uint8_t AC_enable_cmd[4] = {0x54, 0x8f, 0x01, 0x01};
static uint8_t PC_enable_cmd[4] = {0x54, 0x8f, 0x02, 0x01};
static uint8_t AC_disable_cmd[4] = {0x54, 0x8f, 0x00, 0x01};
static uint8_t AC_cmd[4] = {0};

static void elan_i2c_set(struct work_struct *work)
{
	int ret = 0;

	printk("[elan] %s: %x %x %x %x\n", __func__, AC_cmd[0], AC_cmd[1], AC_cmd[2], AC_cmd[3]);
	ret = i2c_master_send(private_ts->client, AC_cmd, sizeof(AC_cmd));
	if(ret < 0)
		printk("[elan] %s: send AC cmd fail\n", __func__);
	return;
}

int eKTH3260_cable_status_handler(int state)
{
	printk("[elan] %s: cable state = %d\n", __func__, state);

	switch(state)
	{
		case TYPE_NONE:
			printk("[elan] %s: TYPE_NONE\n", __func__);
			memcpy(AC_cmd, AC_disable_cmd, sizeof(AC_disable_cmd));

			if(private_ts->system_suspend)
				printk("[elan] %s: Skip cable status notifier when system suspend\n", __func__); 
			else
				queue_delayed_work(private_ts->elan_ac_wq, &private_ts->ac_delay_work, 0.1 * HZ);

			break;

		// charger
		case AC_IN:
			printk("[elan] %s: AC_IN\n", __func__);
			memcpy(AC_cmd, AC_enable_cmd, sizeof(AC_enable_cmd));

			if(private_ts->system_suspend)
				printk("[elan] %s: Skip cable status notifier when system suspend\n", __func__); 
			else
				queue_delayed_work(private_ts->elan_ac_wq, &private_ts->ac_delay_work, 0.1 * HZ);

			break;

		case PC_IN:
			printk("[elan] %s: PC_IN\n", __func__);
			memcpy(AC_cmd, PC_enable_cmd, sizeof(PC_enable_cmd));

			if(private_ts->system_suspend)
				printk("[elan] %s: Skip cable status notifier when system suspend\n", __func__); 
			else
				queue_delayed_work(private_ts->elan_ac_wq, &private_ts->ac_delay_work, 0.1 * HZ);

			break;

		case POWERBANK_IN:
			printk("[elan] %s: POWERBANK_IN\n", __func__);
			memcpy(AC_cmd, AC_enable_cmd, sizeof(AC_enable_cmd));

			if(private_ts->system_suspend)
				printk("[elan] %s: Skip cable status notifier when system suspend\n", __func__); 
			else
				queue_delayed_work(private_ts->elan_ac_wq, &private_ts->ac_delay_work, 0.1 * HZ);

			break;

		default:
			printk("[elan] %s: wrong cable type ..\n", __func__);
			break;
	}
	return 0;
}
EXPORT_SYMBOL(eKTH3260_cable_status_handler);

static ssize_t eKTH3260_proc_gesture_read(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
	struct elan_ktf_ts_data *ts = private_ts;

	printk("[elan] %s: START\n", __func__);

	printk("[elan] %s: ts->gesture = 0x%02x \n", __func__, ts->gesture);
	return 0;
}

static ssize_t eKTH3260_proc_gesture_write(struct file *filp, const char __user *buf, size_t count, loff_t *off)
{
	struct elan_ktf_ts_data *ts = private_ts;

	printk("[elan] %s: GESTURE_DOUBLE_CLICK = %c, GESTURE_ENABLE = %c, GESTURE_W = %c, GESTURE_S = %c, GESTURE_E = %c, GESTURE_C = %c, GESTURE_Z = %c, GESTURE_V = %c \n",
	       __func__, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);

	if((buf[0] == '0') && (buf[1] == '0'))
	{
		ts->gesture = (ts->gesture & (~GESTURE_ENABLE));
		printk("%s: Gesture Disable (ts->gesture = 0x%02x)\n", __func__, ts->gesture);
	}
	else
	{
		if(buf[0] != '0')  // double click
			ts->gesture = (ts->gesture | GESTURE_DOUBLE_CLICK);
		else
			ts->gesture = (ts->gesture & (~GESTURE_DOUBLE_CLICK));

		if(buf[2] != '0') // W
			ts->gesture = (ts->gesture | GESTURE_W);
		else
			ts->gesture = (ts->gesture & (~GESTURE_W));

		if(buf[3] != '0') // S
			ts->gesture = (ts->gesture | GESTURE_S);
		else
			ts->gesture = (ts->gesture & (~GESTURE_S));

		if(buf[4] != '0') // e
			ts->gesture = (ts->gesture | GESTURE_E);
		else
			ts->gesture = (ts->gesture & (~GESTURE_E));

		if(buf[5] != '0') // C
			ts->gesture = (ts->gesture | GESTURE_C);
		else
			ts->gesture = (ts->gesture & (~GESTURE_C));

		if(buf[6] != '0') // Z
			ts->gesture = (ts->gesture | GESTURE_Z);
		else
			ts->gesture = (ts->gesture & (~GESTURE_Z));

		if(buf[7] != '0') // V
			ts->gesture = (ts->gesture | GESTURE_V);
		else
			ts->gesture = (ts->gesture & (~GESTURE_V));

		ts->gesture = (ts->gesture | GESTURE_ENABLE);
		printk("%s: Gesture Enable (ts->gesture = 0x%02x) \n", __func__, ts->gesture);
	}

	return count;
}

static const struct file_operations eKTH3260_gesture_fops =
{
	.owner = THIS_MODULE,
	.read = eKTH3260_proc_gesture_read,
	.write = eKTH3260_proc_gesture_write,
};

static void eKTH3260_create_proc_gesture_file(void)
{
	eKTH3260_proc_gesture_file = proc_create(EKTH3260_PROC_GESTURE_FILE, 0666, NULL, &eKTH3260_gesture_fops);
	if(eKTH3260_proc_gesture_file)
		printk("%s:[%d]: proc Gesture file create sucessed\n", __func__, __LINE__);
	else
		printk("%s:[%d]: proc Gesture file create failed\n", __func__, __LINE__);
	return;
}

static void eKTH3260_remove_proc_gesture_file(void)
{
	extern struct proc_dir_entry proc_root;
	printk("%s:[%d]: proc Gesture file removed.\n", __func__, __LINE__);
	remove_proc_entry(EKTH3260_PROC_GESTURE_FILE, &proc_root);

	return;
}

// add by leo for log tool ++
static ssize_t eKTH3260_proc_logtool_read(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
	printk("[elan] %s: debug = %d\n", __func__, debug);
	printk("[elan] %s: ekth3260_debug = %d\n", __func__, ekth3260_debug);
	return 0;
}

static ssize_t eKTH3260_proc_logtool_write(struct file *filp, const char __user *buf, size_t count, loff_t *off)
{
	if(buf[0] == '1')
	{
		debug = 10;
		ekth3260_debug = 1;
	}
	else if(buf[0] == '0')
	{
		debug = 0;
		ekth3260_debug = 0;
	}

	printk("[elan] %s: debug = %d\n", __func__, debug);
	printk("[elan] %s: ekth3260_debug = %d\n", __func__, ekth3260_debug);
	return count;
}

static const struct file_operations eKTH3260_logtool_fops =
{
	.owner = THIS_MODULE,
	.read = eKTH3260_proc_logtool_read,
	.write = eKTH3260_proc_logtool_write,
};

static void eKTH3260_create_logtool_file(void)
{
	eKTH3260_proc_gesture_file = proc_create(EKTH3260_PROC_LOGTOOL_FILE, 0666, NULL, &eKTH3260_logtool_fops);
	if(eKTH3260_proc_gesture_file)
		printk("%s:[%d]: proc log tool file create sucessed\n", __func__, __LINE__);
	else
		printk("%s:[%d]: proc lgo tool file create failed\n", __func__, __LINE__);
	return;
}

static void eKTH3260_remove_logtool_file(void)
{
	extern struct proc_dir_entry proc_root;
	printk("%s:[%d]: proc log tool file removed.\n", __func__, __LINE__);
	remove_proc_entry(EKTH3260_PROC_LOGTOOL_FILE, &proc_root);
	return;
}
// add by leo for log tool --

static ssize_t eKTH3260_proc_tp_debug_read(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
	printk(KERN_ERR "[elan] %s: START\n", __func__);

	ekth3260_debug = !ekth3260_debug;
	msleep(30);
	printk(KERN_ERR "[elan] %s: into HSYNC count  = %d\n", __func__, hsync_count);
	printk(KERN_ERR "[elan] %s: ekth3260_debug = %d\n", __func__, ekth3260_debug);
	return 0;
}

static ssize_t eKTH3260_proc_tp_debug_write(struct file *filp, const char __user *buf, size_t count, loff_t *off)
{
	//uint8_t buf_recv[8] = {0};
	int err = 0;
	struct elan_ktf_ts_data *ts = private_ts;
	uint8_t recv_buf[PACKET_SIZE] = { 0 };

	printk("%s: debug_function START\n", __func__);

	if((buf[0] == 'd') && (buf[1] == 'e') && (buf[2] == 'b') && (buf[3] == 'u') && (buf[4] == 'g'))
	{
		ekth3260_debug = buf[6] - 48;
		//printk("%s: buf[6] = %d\n", __func__, buf[6]);
		printk("%s: ekth3260_debug = %d\n", __func__, ekth3260_debug);
	}
	else if((buf[0] == 'r') && (buf[1] == 's') && (buf[2] == 't') && (buf[3] == 'p') && (buf[4] == 'i') && (buf[5] == 'n'))
	{
		printk("[elan] %s: get rst_pin = %d\n", __func__, gpio_get_value(ts->rst_gpio));

		if(buf[7] == '0')
		{
			gpio_direction_output(ts->rst_gpio, 0);
			printk("[elan] %s: set rst_pin. = %s\n", __func__, "Low");
		}
		else if(buf[7] == '1')
		{
			gpio_direction_output(ts->rst_gpio, 1);
			printk("[elan] %s: set rst_pin. = %s\n", __func__, "High");
		}
		else
		{
			gpio_direction_input(ts->rst_gpio);
			printk("[elan] %s: set rst_pin. = %s\n", __func__, "Input");
		}
	}
	else if((buf[0] == 'i') && (buf[1] == 'n') && (buf[2] == 't') && (buf[3] == 'r') && (buf[4] == 'p') && (buf[5] == 'i') && (buf[6] == 'n'))
	{
		printk("[elan] %s: get intr_pin = %d\n", __func__, gpio_get_value(ts->intr_gpio));

		if(buf[8] == '0')
		{
			gpio_direction_output(ts->intr_gpio, 0);
			printk("[elan] %s: set intr_pin. = %s\n", __func__, "Low");
		}
		else if(buf[8] == '1')
		{
			gpio_direction_output(ts->intr_gpio, 1);
			printk("[elan] %s: set intr_pin. = %s\n", __func__, "High");
		}
		else
		{
			gpio_direction_input(ts->intr_gpio);
			printk("[elan] %s: set intr_pin. = %s\n", __func__, "Input");
		}
	}
	else if((buf[0] == 'p') && (buf[1] == 'o') && (buf[2] == 'w') && (buf[3] == 'e') && (buf[4] == 'r') && (buf[5] == 'p') && (buf[6] == 'i') && (buf[7] == 'n'))
	{
		if(buf[9] == '0')
		{
			gpio_direction_output(ts->power_gpio, 0);
			printk("[elan] %s: set power_pin. = %s\n", __func__, "Low");
		}
		else if(buf[9] == '1')
		{
			gpio_direction_output(ts->power_gpio, 1);
			printk("[elan] %s: set power_pin. = %s\n", __func__, "High");
		}
		else if(buf[9] == '2')
		{
			gpio_direction_input(ts->power_gpio);
			printk("[elan] %s: set power_pin. = %s\n", __func__, "Input");
		}
	}
	else if((buf[0] == 'r') && (buf[1] == 'e') && (buf[2] == 'c') && (buf[3] == 'v'))
	{
		err = elan_ktf_ts_recv_data(ts->client, recv_buf, PACKET_SIZE);
		if(err < 0)
			printk("[elan] Received the packet Error.\n");
	}

	// JUST FOR TEST
	if((buf[0] == 'g') && (buf[1] == 'e') && (buf[2] == 's') && (buf[3] == 't') && (buf[4] == 'u') && (buf[5] == 'r') && (buf[6] == 'e'))
	{
		printk("[elan] %s: debug_function = testtest \n", __func__);
		if(buf[8] == '1')
		{
			printk("%s: report gesture KEY_POWER test. \n", __func__);
			input_report_key(ts->input_dev, KEY_POWER, 1);
			input_sync(ts->input_dev);
			input_report_key(ts->input_dev, KEY_POWER, 0);
			input_sync(ts->input_dev);
		}
		else if(buf[8] == '2')
		{
			printk("[elan] %s: report gesture KEY_POWER2 test. \n", __func__);
			input_report_key(ts->input_dev, KEY_POWER2, 1);
			input_sync(ts->input_dev);
			input_report_key(ts->input_dev, KEY_POWER2, 0);
			input_sync(ts->input_dev);
		}
		else if((buf[8] == 'd') && (buf[9] == 'c'))
		{
			printk("[elan] %s: report gesture Double Click test. \n", __func__);
			input_report_key(ts->input_dev, KEY_F24, 1);
			input_sync(ts->input_dev);
			input_report_key(ts->input_dev, KEY_F24, 0);
			input_sync(ts->input_dev);
		}
		else if(buf[8] == 'w')
		{
			printk("[elan] %s: report gesture W test. \n", __func__);
			input_report_key(ts->input_dev, KEY_F23, 1);
			input_sync(ts->input_dev);
			input_report_key(ts->input_dev, KEY_F23, 0);
			input_sync(ts->input_dev);
		}
		else if(buf[8] == 's')
		{
			printk("[elan] %s: report gesture S test. \n", __func__);
			input_report_key(ts->input_dev, KEY_F22, 1);
			input_sync(ts->input_dev);
			input_report_key(ts->input_dev, KEY_F22, 0);
			input_sync(ts->input_dev);
		}
		else if(buf[8] == 'e')
		{
			printk("[elan] %s: report gesture e test.", __func__);
			input_report_key(ts->input_dev, KEY_F21, 1);
			input_sync(ts->input_dev);
			input_report_key(ts->input_dev, KEY_F21, 0);
			input_sync(ts->input_dev);
		}
		else if(buf[8] == 'c')
		{
			printk("[elan] %s: report gesture C test. \n", __func__);
			input_report_key(ts->input_dev, KEY_F20, 1);
			input_sync(ts->input_dev);
			input_report_key(ts->input_dev, KEY_F20, 0);
			input_sync(ts->input_dev);
		}
		else if(buf[8] == 'z')
		{
			printk("[elan] %s: report gesture S test. \n", __func__);
			input_report_key(ts->input_dev, KEY_F19, 1);
			input_sync(ts->input_dev);
			input_report_key(ts->input_dev, KEY_F19, 0);
			input_sync(ts->input_dev);
		}
		else if(buf[8] == 'v')
		{
			printk("[elan] %s: report gesture V test. \n", __func__);
			input_report_key(ts->input_dev, KEY_F18, 1);
			input_sync(ts->input_dev);
			input_report_key(ts->input_dev, KEY_F18, 0);
			input_sync(ts->input_dev);
		}
		else if(buf[8] == 'o')
		{
			printk("[elan] %s: report gesture V test. \n", __func__);
			input_report_key(ts->input_dev, KEY_F17, 1);
			input_sync(ts->input_dev);
			input_report_key(ts->input_dev, KEY_F17, 0);
			input_sync(ts->input_dev);
		}
		else
		{
			input_report_key(ts->input_dev, BTN_TOUCH, 1);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, 400);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, 500);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 65);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 65);
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 0);
			input_mt_sync(ts->input_dev);

			msleep(30);
			input_report_key(ts->input_dev, BTN_TOUCH, 0);
		}
	}
	return count;
}

static const struct file_operations eKTH3260_tp_debug_fops =
{
	.owner = THIS_MODULE,
	.read = eKTH3260_proc_tp_debug_read,
	.write = eKTH3260_proc_tp_debug_write,
};

static void eKTH3260_create_proc_tp_debug_file(void)
{
	eKTH3260_proc_tp_debug_file = proc_create(EKTH3260_PROC_TP_DEBUG_FILE, 0666, NULL, &eKTH3260_tp_debug_fops);
	if(eKTH3260_proc_tp_debug_file)
		printk("[elan] %s:[%d]: proc TP debug file create sucessed\n", __func__, __LINE__);
	else
		printk("[elan] %s:[%d]: proc TP debug file create failed\n", __func__, __LINE__);
	return;
}

static void eKTH3260_remove_proc_tp_debug_file(void)
{
	extern struct proc_dir_entry proc_root;
	printk("[elan] %s:[%d]: proc debug file removed.\n", __func__, __LINE__);
	remove_proc_entry(EKTH3260_PROC_TP_DEBUG_FILE, &proc_root);

	return;
}

ssize_t touch_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%X_%X\n", FW_ID, FW_VERSION);
}
ssize_t touch_switch_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%d\n", ekth3260_touch_status);
}

#if TOUCH_SYSTRACE_TEST
static void elan_test(struct work_struct *work)
{
	if(debug > 20)printk("[elan] %s: start \n", __func__);
	return;
}
#endif
// add by leo for testtest --

static int elan_ktf_ts_probe(struct i2c_client *client,
                             const struct i2c_device_id *id)
{
	int err = 0;
	int fw_err = 0;
	//struct elan_ktf_i2c_platform_data *pdata;
	struct elan_ktf_ts_data *ts;
	//int New_FW_ID;
	//int New_FW_VER;
	u32 value[2] = {0};
	//struct task_struct *fw_update_thread;
	//int intr_status =0, retry =50;

	printk("////////////////////		elan_ktf_ts_probe START v27 \n"); // add by leo for testtest

	// add by leo for skip COS/POS ++
	//if(entry_mode==4)
	//{
	//    printk("%s:[%d]: In COS, Skip Probe\n", __func__, __LINE__);
	//    return 0;
	//}
	//else if(entry_mode==3)
	//{
	//    printk("%s:[%d]: In POS, Skip Probe\n", __func__, __LINE__);
	//    return 0;
	//}
	// add by leo for skip COS/POS --

	if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		printk(KERN_ERR "[elan] %s: i2c check functionality error\n", __func__);
		err = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(struct elan_ktf_ts_data), GFP_KERNEL);
	if(ts == NULL)
	{
		printk(KERN_ERR "[elan] %s: allocate elan_ktf_ts_data failed\n", __func__);
		err = -ENOMEM;
		goto err_alloc_data_failed;
	}

	if(!of_property_read_u32_array(client->dev.of_node, "interrupts", value, 2))
	{
		printk("int data %d, %d, client->irq %d\n", value[0], value[1], client->irq);
	}

	ts->client = client;
	i2c_set_clientdata(client, ts);
	mutex_init(&ktf_mutex);
	// james: maybe remove
	//pdata = client->dev.platform_data;
	//if (likely(pdata != NULL)) {
	//	ts->intr_gpio = pdata->intr_gpio;
	//}

	private_ts = ts;

	ts->elan_irq_wq = create_singlethread_workqueue("elan_irq_wq");
	if(!ts->elan_irq_wq)
	{
		printk(KERN_ERR "[elan] %s: create elan_irq_wq workqueue failed\n", __func__);
		err = -ENOMEM;
	}
	INIT_WORK(&ts->irq_work, elan_ktf_ts_work_func);

	// add by leo for testtest ++
#if TOUCH_SYSTRACE_TEST
	ts->elan_wq = create_singlethread_workqueue("elan_wq");
	if(!ts->elan_wq)
	{
		printk(KERN_ERR "[elan] %s: create elan_wq workqueue failed\n", __func__);
	}
	INIT_WORK(&ts->work, elan_test);
#endif

	ts->elan_ac_wq = create_singlethread_workqueue("elan_ac_wq");
	if(!ts->elan_ac_wq)
	{
		printk(KERN_ERR "[elan] %s: create elan_ac_wq workqueue failed\n", __func__);
	}
	INIT_DELAYED_WORK(&ts->ac_delay_work, elan_i2c_set);

	// add by leo for EVB with GT928 ++
	ts->HW_ID = Read_HW_ID();
	printk("////////////////////		HW ID = %d\n", ts->HW_ID);
	//if(ts->HW_ID==0)
	//{
	//    printk("////////////////////		Skip probe ELAN eKTH3264 Touch driver\n");
	//    return -ENODEV;
	//}
	// add by leo for EVB with GT928 --

	err = eKTH3260_parse_dt(&client->dev, ts);

	/*init POWER pin*/
	err = gpio_request(ts->power_gpio, "ElanTouch-power");
	if(err < 0)
		printk(KERN_ERR "[elan] %s:Failed to request GPIO%d (ElanTouch-power) error=%d\n", __func__, ts->power_gpio, err);

	err = gpio_direction_output(ts->power_gpio, 1);
	if(err)
	{
		printk(KERN_ERR "[elan] %s:Failed to set reset direction, error=%d\n", __func__, err);
		gpio_free(ts->power_gpio);
	}

	printk(KERN_INFO "[elan] %s touch power enable\n", __func__);
	msleep(300);

	/*init INTERRUPT pin*/
	err = gpio_request(ts->intr_gpio, "ElanTouch-irq");
	if(err < 0)
		printk(KERN_ERR "[elan] %s:Failed to request GPIO%d (ElanTouch-interrupt) error=%d\n", __func__, ts->intr_gpio, err);

	err = gpio_direction_input(ts->intr_gpio);
	if(err)
	{
		printk(KERN_ERR "[elan] %s:Failed to set interrupt direction, error=%d\n", __func__, err);
		gpio_free(ts->intr_gpio);
	}

	/*init RESET pin*/
	err = gpio_request(ts->rst_gpio, "ElanTouch-rst");
	if(err < 0)
		printk(KERN_ERR "[elan] %s:Failed to request GPIO%d (ElanTouch-rst) error=%d\n", __func__, ts->rst_gpio, err);

	err = gpio_direction_output(ts->rst_gpio, 1);
	if(err)
	{
		printk(KERN_ERR "[elan] %s:Failed to set reset direction, error=%d\n", __func__, err);
		gpio_free(ts->rst_gpio);
	}

	/*read LCM ID0 pin*/
	ts->LCM_ID0 = gpio_get_value(ts->lcmid0_gpio);
	printk("[elan] %s lcmid0_pin = %d \n", __func__, ts->LCM_ID0);

	if(ts->LCM_ID0)
		memcpy(sensor_cmd, new_sensor_cmd, sizeof(new_sensor_cmd));
	else
		memcpy(sensor_cmd, old_sensor_cmd, sizeof(old_sensor_cmd));
	INIT_DELAYED_WORK(&ts->sensor_delay_work, elan_sensor_set);

	/*read LCM ID2 pin*/
	ts->LCM_ID2 = gpio_get_value(ts->lcmid2_gpio);
	printk("[elan] %s lcmid2_pin = %d \n", __func__, ts->LCM_ID2);

	if(ts->LCM_ID2)
		PageNum = sizeof(file_fw_data_AUO) / 132;
	else
		PageNum = sizeof(file_fw_data_CPT) / 132;

	/*init regulator setting*/
	ts->vdd = regulator_get(&ts->client->dev, "vdd");
	if(IS_ERR(ts->vdd))
	{
		err = PTR_ERR(ts->vdd);
		dev_info(&ts->client->dev, "Regulator get failed vdd err=%d\n", err);
	}

	if(!IS_ERR(ts->vdd))
	{
		err = regulator_set_voltage(ts->vdd, eKTH3260_VTG_MIN_UV, eKTH3260_VTG_MAX_UV);
		if(err)
		{
			dev_err(&ts->client->dev, "Regulator set_vtg failed vdd err=%d\n", err);
			goto err_i2c_power_on;
		}

		err = regulator_enable(ts->vdd);
		if(err)
		{
			dev_err(&ts->client->dev, "Regulator vdd enable failed err=%d\n", err);
			goto err_i2c_power_on;
		}
	}

	ts->vcc_i2c = regulator_get(&ts->client->dev, "vcc_i2c");
	if(IS_ERR(ts->vcc_i2c))
	{
		err = PTR_ERR(ts->vcc_i2c);
		dev_info(&ts->client->dev, "Regulator get failed vcc_i2c err=%d\n", err);
	}

	if(!IS_ERR(ts->vcc_i2c))
	{
		err = regulator_set_voltage(ts->vcc_i2c, eKTH3260_I2C_VTG_MIN_UV, eKTH3260_I2C_VTG_MAX_UV);
		if(err)
		{
			dev_err(&ts->client->dev,
			        "Regulator set_vtg failed vcc_i2c err=%d\n",
			        err);
			goto err_i2c_power_on;
		}

		err = regulator_enable(ts->vcc_i2c);
		if(err)
		{
			dev_err(&ts->client->dev, "Regulator vcc_i2c enable failed err=%d\n", err);
			goto err_i2c_power_on;
		}
	}
	msleep(100);
	// add by leo for testtest --

	// add by leo ++
	/*
	do {
		intr_status = gpio_get_value(ts->intr_gpio);
		printk("////////////////////		intr_status = %d \n", intr_status);
		retry--;
		mdelay(20);
	} while (retry > 0);
	*/
	// add by leo --

	wake_lock_init(&ts->wakelock, WAKE_LOCK_SUSPEND, "elan-touchscreen");

	elan_ktf_ts_hw_reset();

	msleep(100);
	fw_err = elan_ktf_ts_setup(client);
	if(fw_err < 0)
	{
		printk(KERN_INFO "No Elan chip inside\n");
		//fw_err = -ENODEV;
	}

	// add by leo for set resolution directly ++
        X_RESOLUTION = 1472;
        Y_RESOLUTION = 2368;
        // add by leo for set resolution directly --

	ts->input_dev = input_allocate_device();
	if(ts->input_dev == NULL)
	{
		err = -ENOMEM;
		dev_err(&client->dev, "[elan] Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "elan-touchscreen";

#ifdef PROTOCOL_A
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
#endif
#ifdef ELAN_BUTTON
	set_bit(KEY_BACK, ts->input_dev->keybit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_HOME, ts->input_dev->keybit);
	set_bit(KEY_SEARCH, ts->input_dev->keybit);
#endif

#ifdef PROTOCOL_B
	input_mt_init_slots(ts->input_dev, FINGER_NUM, 0);
#endif

#ifdef PROTOCOL_A
	input_set_abs_params(ts->input_dev, ABS_X, 0,  X_RESOLUTION, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0,  Y_RESOLUTION, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_TOOL_WIDTH, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, FINGER_NUM, 0, 0);
#endif
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, Y_RESOLUTION, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, X_RESOLUTION, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);

	__set_bit(EV_ABS, ts->input_dev->evbit);
	__set_bit(EV_SYN, ts->input_dev->evbit);
	__set_bit(EV_KEY, ts->input_dev->evbit);

	err = input_register_device(ts->input_dev);
	if(err)
	{
		dev_err(&client->dev,
		        "[elan]%s: unable to register %s input device\n",
		        __func__, ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	// add by leo ++
#ifdef GESTURE_MODE
	input_set_capability(ts->input_dev, EV_KEY, KEY_POWER);
	input_set_capability(ts->input_dev, EV_KEY, KEY_POWER2);
	input_set_capability(ts->input_dev, EV_KEY, KEY_F24);
	input_set_capability(ts->input_dev, EV_KEY, KEY_F23);
	input_set_capability(ts->input_dev, EV_KEY, KEY_F22);
	input_set_capability(ts->input_dev, EV_KEY, KEY_F21);
	input_set_capability(ts->input_dev, EV_KEY, KEY_F20);
	input_set_capability(ts->input_dev, EV_KEY, KEY_F19);
	input_set_capability(ts->input_dev, EV_KEY, KEY_F18);
	input_set_capability(ts->input_dev, EV_KEY, KEY_F17);

	ts->gesture = 0;
	printk("[elan] %s: ts->gesture = 0x%02X\n", __func__,  ts->gesture);
#endif

	eKTH3260_create_proc_gesture_file();
	eKTH3260_create_proc_tp_debug_file();
	eKTH3260_create_logtool_file();
	// add by leo --

	elan_ktf_ts_register_interrupt(ts->client);
	disable_irq(ts->client->irq);

	elan_ktf_touch_sysfs_init();

	dev_info(&client->dev, "[elan] Start touchscreen %s in interrupt mode\n",
	         ts->input_dev->name);

	// Firmware Update
	ts->firmware.minor = MISC_DYNAMIC_MINOR;
	ts->firmware.name = "elan-iap";
	ts->firmware.fops = &elan_touch_fops;
	ts->firmware.mode = S_IFREG | S_IRWXUGO;

	if(misc_register(&ts->firmware) < 0)
		printk("[elan] %s: misc_register failed!! \n", __func__);
	else
		printk("[elan] %s: misc_register finished!! \n", __func__);
	// End Firmware Update

	// add by leo ++
	err = misc_register(&ekth3260_misc_dev);
	if(err < 0)
	{
		printk("[elan] %s: [%d]: misc_register failed (%d) \n", __func__, __LINE__, err);
	}
	// add by leo --

	/* register sysfs */
	if(sysfs_create_group(&client->dev.kobj, &elan_attribute_group))
		dev_err(&client->dev, "elan_attribute_group sysfs create group error\n");

#ifdef IAP_PORTION
	if(0)
	{
		printk("[elan]misc_register finished!!\n");
		work_lock = 1;
		disable_irq(ts->client->irq);
		//cancel_work_sync(&ts->work);

		power_lock = 1;
		/* FW ID & FW VER*/
#if 0  /* For ektf21xx and ektf20xx  */
		printk("[ELAN]  [7bd0]=0x%02x,  [7bd1]=0x%02x, [7bd2]=0x%02x, [7bd3]=0x%02x\n",  file_fw_data[31696], file_fw_data[31697], file_fw_data[31698], file_fw_data[31699]);
		New_FW_ID = file_fw_data[31699] << 8  | file_fw_data[31698] ;
		New_FW_VER = file_fw_data[31697] << 8  | file_fw_data[31696] ;
#endif

#if 0   /* for ektf31xx 2 wire ice ex: 2wireice -b xx.bin */
		printk(" [7c16]=0x%02x,  [7c17]=0x%02x, [7c18]=0x%02x, [7c19]=0x%02x\n",  file_fw_data[31766], file_fw_data[31767], file_fw_data[31768], file_fw_data[31769]);
		New_FW_ID = file_fw_data[31769] << 8  | file_fw_data[31768] ;
		New_FW_VER = file_fw_data[31767] << 8  | file_fw_data[31766] ;
#endif
		/* for ektf31xx iap ekt file   */
		/* mark by leo
		printk(" [7bd8]=0x%02x,  [7bd9]=0x%02x, [7bda]=0x%02x, [7bdb]=0x%02x\n",  file_fw_data[31704],file_fw_data[31705],file_fw_data[31706],file_fw_data[31707]);
		New_FW_ID = file_fw_data[31707]<<8  | file_fw_data[31708] ;
		New_FW_VER = file_fw_data[31705]<<8  | file_fw_data[31704] ;
		printk(" FW_ID=0x%x,   New_FW_ID=0x%x \n",  FW_ID, New_FW_ID);
		printk(" FW_VERSION=0x%x,   New_FW_VER=0x%x \n",  FW_VERSION  , New_FW_VER);
		*/
		/* for firmware auto-upgrade
		if (New_FW_ID   ==  FW_ID){
			if (New_FW_VER > (FW_VERSION))
						Update_FW_in_Driver(client, RECOVERY);
		} else {
						printk("FW_ID is different!");
		}
		*/
		/*if (FW_ID == 0)  RECOVERY=0x80;
		Update_FW_in_Driver(client, RECOVERY);*/
		power_lock = 0;

		work_lock = 0;
		enable_irq(ts->client->irq);

	}
#endif

	// add by leo for testtest ++
#ifdef FW_UPDATE_IN_DRIVER
	printk("[elan] %s: ts->LCM_ID0 = %d\n", __func__, ts->LCM_ID0);
	printk("[elan] %s: ts->LCM_ID2 = %d\n", __func__, ts->LCM_ID2);
	printk("[elan] %s: FW_ID = 0x%02x\n", __func__, FW_ID);
	printk("[elan] %s: FW_VERSION = 0x%02x\n", __func__, FW_VERSION);
	printk("[elan] %s: build_version = %d\n", __func__, build_version);

	if(build_version == 1)
		printk("%s: Skip touch FW update in eng mode ...\n", __func__);
	else
	{
		if(ts->LCM_ID2) // AUO LCM
		{
			//if((FW_ID!=0x3038)||(FW_VERSION!=0x5514)||(RECOVERY==0x80))
			if((FW_ID != 0x3038) || (FW_VERSION < 0x551f))
			{
				printk("[elan] %s: Start touch FW update to version 0x3038_0x551f for AUO LCM...\n", __func__);
				//elan_TWO_WIRE_ICE(ts->client);
				Update_FW_in_Driver(0);

				elan_ktf_ts_hw_reset();

				msleep(100);
				fw_err = elan_ktf_ts_setup(client);
				if(fw_err < 0)
				{
					printk(KERN_INFO "No Elan chip inside\n");
				}
			}
			else
				printk("[elan] %s: FW Check Pass !\n", __func__);
		}
		else // CPT LCM
		{
			//if((FW_ID!=0x3037)||(FW_VERSION!=0x5514)||(RECOVERY==0x80))
			if((FW_ID != 0x3037) || (FW_VERSION < 0x554c))
			{
				printk("[elan] %s: Start touch FW update to version 0x3037_0x554c for CPT LCM...\n", __func__);
				//elan_TWO_WIRE_ICE(ts->client);
				Update_FW_in_Driver(0);

				elan_ktf_ts_hw_reset();

				msleep(100);
				fw_err = elan_ktf_ts_setup(client);
				if(fw_err < 0)
				{
					printk(KERN_INFO "No Elan chip inside\n");
				}
			}
			else
				printk("[elan] %s: FW Check Pass !\n", __func__);
		}
	}
#endif

	ts->touch_sdev.name = TOUCH_SDEV_NAME;
	ts->touch_sdev.print_name = touch_switch_name;
	ts->touch_sdev.print_state = touch_switch_state;
	if(switch_dev_register(&ts->touch_sdev) < 0)
	{
		printk("%s: switch_dev_register failed!\n", __func__);
	}
	// add by leo for testtest --

#if 0
	read_test();
	fw_update_thread = kthread_run(Update_FW_in_Driver, NULL, "elan_update");
	if(IS_ERR(fw_update_thread))
	{
		printk("[elan]  failed to create kernel thread\n");
	}
#endif


#ifdef _ENABLE_DBG_LEVEL
	dbgProcFile = create_proc_entry(PROC_FS_NAME, 0600, NULL);
	if(dbgProcFile == NULL)
	{
		remove_proc_entry(PROC_FS_NAME, NULL);
		touch_debug(DEBUG_INFO, " Could not initialize /proc/%s\n", PROC_FS_NAME);
	}
	else
	{
		dbgProcFile->read_proc = ektf_proc_read;
		dbgProcFile->write_proc = ektf_proc_write;
		touch_debug(DEBUG_INFO, " /proc/%s created\n", PROC_FS_NAME);
	}
#endif // #ifdef _ENABLE_DBG_LEVEL

#if 0
#if defined(CONFIG_FB)
	private_ts->fb_notif.notifier_call = fb_notifier_callback;

	fb_register_client(&private_ts->fb_notif);

#endif
#endif

// add by leo for early-suspend ++
// reg callback
#ifdef CONFIG_PM_SCREEN_STATE_NOTIFIER
	callback_struct = register_screen_state_notifier(&eKTH3260_screen_chenged_listaner);
#endif
// add by leo for early-suspend --

#ifdef ESD_CHECK
	INIT_DELAYED_WORK(&ts->check_work, elan_ktf_ts_check_work_func);	// reset if check hang
	schedule_delayed_work(&private_ts->check_work, msecs_to_jiffies(2500)); //1218
#endif

	queue_delayed_work(private_ts->elan_ac_wq, &private_ts->sensor_delay_work, 0.2 * HZ);

	enable_irq(ts->client->irq);
	printk("////////////////////		elan_ktf_ts_probe END \n"); // add by leo for testtest
	return 0;

err_input_register_device_failed:
	if(ts->input_dev)
		input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
	kfree(ts);
#ifdef _ENABLE_DBG_LEVEL
	remove_proc_entry(PROC_FS_NAME, NULL);
#endif

err_i2c_power_on:
err_alloc_data_failed:
err_check_functionality_failed:

	return err;
}

static int elan_ktf_ts_remove(struct i2c_client *client)
{
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);

	elan_touch_sysfs_deinit();

// add by leo for early-suspend ++
//unreg call back
#ifdef CONFIG_PM_SCREEN_STATE_NOTIFIER
	unregister_screen_state_notifier(callback_struct);
#endif
// add by leo for early-suspend --

	// add by leo ++
	eKTH3260_remove_proc_gesture_file();
	eKTH3260_remove_proc_tp_debug_file();
	eKTH3260_remove_logtool_file();
	// add by leo --

	//unregister_early_suspend(&ts->early_suspend);
	free_irq(client->irq, ts);

	input_unregister_device(ts->input_dev);
	kfree(ts);

	return 0;
}

#ifdef CONFIG_PM_SCREEN_STATE_NOTIFIER
static int elan_ktf_ts_set_power_state(struct i2c_client *client, int state)
{
	uint8_t cmd[] = {CMD_W_PKT, 0x50, 0x00, 0x01};

	dev_dbg(&client->dev, "[elan] %s: enter\n", __func__);

	cmd[1] |= (state << 3);

	dev_dbg(&client->dev,
	        "[elan] dump cmd: %02x, %02x, %02x, %02x\n",
	        cmd[0], cmd[1], cmd[2], cmd[3]);

	printk("[elan] dump cmd: %02x, %02x, %02x, %02x\n", cmd[0], cmd[1], cmd[2], cmd[3]);

	if((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd))
	{
		dev_err(&client->dev,
		        "[elan] %s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int elan_ktf_ts_get_power_state(struct i2c_client *client)
{
	int rc = 0;
	uint8_t cmd[] = {CMD_R_PKT, 0x50, 0x00, 0x01};
	uint8_t buf[4], power_state;

	rc = elan_ktf_ts_get_data(client, cmd, buf, 4, 4);
	if(rc)
		return rc;

	power_state = buf[1];
	dev_dbg(&client->dev, "[elan] dump repsponse: %0x\n", power_state);
	power_state = (power_state & PWR_STATE_MASK) >> 3;
	dev_dbg(&client->dev, "[elan] power state = %s\n",
	        power_state == PWR_STATE_DEEP_SLEEP ?
	        "Deep Sleep" : "Normal/Idle");

	return power_state;
}
#endif

#ifdef CONFIG_PM_SCREEN_STATE_NOTIFIER
static int elan_ktf_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int rc = 0, ret = 0;
	uint8_t gesture_wake_up_cmd[] = {0x54, 0x40, 0x01, 0x01};
	uint8_t gesture_mode_check_cmd[] = {0x53, 0x40, 0x00, 0x01};
	uint8_t buf_recv[4] = { 0 };
	struct elan_ktf_ts_data *ts = private_ts;

	if(power_lock == 0) /* The power_lock can be removed when firmware upgrade procedure will not be enter into suspend mode.  */
	{
		ts->system_suspend = 1; // add by leo for hello package in gesture mode ++
#ifdef ESD_CHECK
		cancel_delayed_work_sync(&ts->check_work);
#endif
		printk(KERN_INFO "[elan] %s: enter\n", __func__);
		if(ts->gesture & GESTURE_ENABLE)
		{
			ret = i2c_master_send(client, gesture_wake_up_cmd, sizeof(gesture_wake_up_cmd));
			if(ret < 0)
				printk("[elan] %s: send gesture_wake_up_cmd fail\n", __func__);

			msleep(2);

			// add by leo for hello package in gesture mode ++
			ret = i2c_master_send(client, gesture_mode_check_cmd, sizeof(gesture_mode_check_cmd));
			if(ret < 0)
				printk("[elan] %s: send gesture_mode_check_cmd fail\n", __func__);

			rc = i2c_master_recv(client, buf_recv, 4);
			if(rc != 4)
				printk("[elan] %s: read gesture mode state error.\n", __func__);

			printk("[elan] %s: gesture mode state: %2x:%2x:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);
			// add by leo for hello package in gesture mode --
		}
		else
		{
			printk(KERN_INFO "[elan] %s: enter sleep mode\n", __func__);
			rc = elan_ktf_ts_set_power_state(client, PWR_STATE_DEEP_SLEEP);
		}
	}
	return 0;
}

static int elan_ktf_ts_resume(struct i2c_client *client)
{
	int rc = 0, retry = 3;
#ifdef RE_CALIBRATION
	uint8_t buf_recv[4] = { 0 };
#endif
	struct elan_ktf_ts_data *ts = private_ts;

	if(power_lock == 0) /* The power_lock can be removed when firmware upgrade procedure will not be enter into suspend mode.  */
	{
		ts->system_suspend = 0; // add by leo for hello package in gesture mode ++
		printk(KERN_INFO "[elan] %s: enter\n", __func__);
#ifdef ESD_CHECK
		schedule_delayed_work(&private_ts->check_work, msecs_to_jiffies(2500));
#endif
#ifdef ELAN_RESUME_RST
		printk("[elan] %s: Used Reset instead of command to resume touch panel\n", __func__);
		elan_ktf_ts_hw_reset();
		queue_delayed_work(private_ts->elan_ac_wq, &private_ts->ac_delay_work, 1 * HZ);
		return 0;
#endif
		do
		{
			rc = elan_ktf_ts_set_power_state(client, PWR_STATE_NORMAL);
			mdelay(200);
#ifdef RE_CALIBRATION
			rc = i2c_master_recv(client, buf_recv, 4);
			printk("[elan] %s: Re-Calibration Packet %2x:%2x:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);
			if(buf_recv[0] != 0x66)
			{
				mdelay(200);
				rc = i2c_master_recv(client, buf_recv, 4);
				printk("[elan] %s: Re-Calibration Packet, re-try again %2x:%2x:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);
			}
#endif
			rc = elan_ktf_ts_get_power_state(client);
			if(rc != PWR_STATE_NORMAL)
				printk(KERN_ERR "[elan] %s: wake up tp failed! err = %d\n",
				       __func__, rc);
			else
				break;
		}
		while(--retry);

	}
	return 0;
}
#endif

// add by leo for early-suspend ++
//program call back
#ifdef CONFIG_PM_SCREEN_STATE_NOTIFIER
void eKTH3260_screen_chenged_listaner(const int state)
{
	pm_message_t mesg;

	if(state == NOTIFY_WHEN_SCREEN_OFF)
	{
		/* something you want to do at screen off */
		printk("////////////////////		Screen off suspend\n");
		elan_ktf_ts_suspend(private_ts->client, mesg);
	}
	else if(state == NOTIFY_WHEN_SCREEN_ON)
	{
		/* something you want to do at screen on*/
		printk("////////////////////		Screen on resume\n");
		elan_ktf_ts_resume(private_ts->client);
	}
}
#endif
// add by leo for early-suspend --

static const struct i2c_device_id elan_ktf_ts_id[] =
{
	{ ELAN_KTF_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, elan_ktf_ts_id);

static const struct of_device_id elan_of_match[] =
{
	{ .compatible = "elan,ekth3260" },
	{ /* sentinel */ }
};

static struct i2c_driver ektf_ts_driver =
{
	.probe		= elan_ktf_ts_probe,
	.remove		= elan_ktf_ts_remove,
	//.suspend	= elan_ktf_ts_suspend,
	//.resume		= elan_ktf_ts_resume,
	.id_table	= elan_ktf_ts_id,
	.driver		= {
		.name = ELAN_KTF_NAME,
		.owner = THIS_MODULE,
		.of_match_table = elan_of_match,
	},
};

module_i2c_driver(ektf_ts_driver);


#if 0
static int elan_ktf_ts_init(void)
{
	printk(KERN_INFO "[elan] %s driver version 0x0005: Integrated 2, 5, and 10 fingers together and auto-mapping resolution\n", __func__);
	return i2c_add_driver(&ektf_ts_driver);
}

static void elan_ktf_ts_exit(void)
{
	i2c_del_driver(&ektf_ts_driver);
	return;
}


module_init(elan_ktf_ts_init);
module_exit(elan_ktf_ts_exit);
#endif
MODULE_DESCRIPTION("ELAN KTF2K Touchscreen Driver");
MODULE_LICENSE("GPL");



