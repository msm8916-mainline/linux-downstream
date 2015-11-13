/* drivers/input/touchscreen/ektf2k.c - ELAN EKTF2K verions of driver
opyright (C) 2011 Elan Microelectronics Corporation.
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
 * 2011/12/06: The first release, version 0x0001
 * 2012/2/15:  The second release, version 0x0002 for new bootcode
 * 2012/5/8:   Release version 0x0003 for china market
 *             Integrated 2 and 5 fingers driver code together and
 *             auto-mapping resolution.
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>

#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <linux/ektf2k.h>
#include <linux/regulator/consumer.h>

#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
#include <linux/of_gpio.h>
#include <linux/debugfs.h>
#include <linux/input/mt.h>

#define PACKET_SIZE		34

#define PWR_STATE_DEEP_SLEEP	0
#define PWR_STATE_NORMAL		1
#define PWR_STATE_MASK			BIT(3)

#define CMD_S_PKT			0x52
#define CMD_R_PKT			0x53
#define CMD_W_PKT			0x54

#define HELLO_PKT			0x55
#define NORMAL_PKT			0x5D

#define GPIO_TS_PWD 35
#define TWO_FINGERS_PKT      0x5A
#define FIVE_FINGERS_PKT       0x6D
#define TEN_FINGERS_PKT	0x62

#define RESET_PKT			0x77
#define CALIB_PKT			0xA8

#define SYSTEM_RESET_PIN_SR	10

#define IAP_IN_DRIVER_MODE	1
#define IAP_PORTION             0
#define PAGERETRY  30
#define IAPRESTART 5
/*
#define ELAN_BUTTON
#define ELAN_KEY_BACK	0x10
#define ELAN_KEY_HOME	0x08
#define ELAN_KEY_MENU	0x04
*/

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
#if IAP_PORTION
#define IOCTL_FW_UPDATE  _IOR(ELAN_IOCTLID, 17, int)
#define IOCTL_GET_UPDATE_PROGREE	_IOR(CUSTOMER_IOCTLID,  2, int)
#endif

#define CUSTOMER_IOCTLID	0xA0
#define IOCTL_CIRCUIT_CHECK  _IOR(CUSTOMER_IOCTLID, 1, int)

#define ELAN_VTG_MIN_UV		2700000
#define ELAN_VTG_MAX_UV		3300000
#define ELAN_I2C_VTG_MIN_UV	1800000
#define ELAN_I2C_VTG_MAX_UV	1800000

#define DEBUG_DIR_NAME		"ts_debug"
#define ELAN_INFO_MAX_LEN	512

#define STORE_TS_INFO(buf, name, max_touches, fw_version, \
		fw_vkey_support) \
	snprintf(buf, ELAN_INFO_MAX_LEN, \
			"controller     = elan.\n" \
			"chip name      = %s\n" \
			"max_touches    = %d\n" \
			"driver_ver     = N/A\n" \
			"fw_ver         = 0x%x\n" \
			"fw_vkey_support= %s\n", \
			name, max_touches, fw_version, fw_vkey_support)

static int elan_ktf2k_ts_suspend(struct device *dev);
static int GPIO_TS_RESET;

uint8_t RECOVERY = 0x00;
int FW_VERSION = 0x00;
int X_RESOLUTION = 0x00;
int Y_RESOLUTION = 0x00;
int FW_ID = 0x00;
int work_lock = 0x00;
int power_lock = 0x00;
int circuit_ver = 0x01;
/*++++i2c transfer start+++++++*/
int file_fops_addr = 0x10;
/*++++i2c transfer end+++++++*/
int button_state;
int tpd_down_flag;
#if IAP_PORTION
uint8_t ic_status = 0x00;
int update_progree;
uint8_t I2C_DATA[3] = { 0x15, 0x20, 0x21 };	/*I2C devices address */

int is_OldBootCode;

/*The newest firmware, if update must be changed here*/
static uint8_t file_fw_data[] = {
#include "fw_data.i"
};

enum {
	PageSize = 132,
	PageNum = 249,
	ACK_Fail = 0x00,
	ACK_OK = 0xAA,
	ACK_REWRITE = 0x55,
};

enum {
	E_FD = -1,
};
#endif
struct elan_ktf2k_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct workqueue_struct *elan_wq;
	struct work_struct work;
	int irq;

	int fw_ver;
	int fw_id;
	int x_resolution;
	int y_resolution;

	struct miscdevice firmware;

	struct regulator *vdd;
	struct regulator *vcc_i2c;
	const struct elan_ktf2k_i2c_platform_data *pdata;
	char *ts_info;
	bool enable;
	bool suspended;
	struct dentry *dir;
	char *ts_data;
#ifdef CONFIG_FB
	struct notifier_block fb_notif;
#endif
};

static struct elan_ktf2k_ts_data *private_ts;
static int __fw_packet_handler(struct i2c_client *client);
static int elan_ktf2k_ts_rough_calibrate(struct i2c_client *client);
static int elan_ktf2k_ts_resume(struct device *dev);

/*
#if IAP_PORTION
int Update_FW_One(struct i2c_client *client, int recovery);
static int __hello_packet_handler(struct i2c_client *client);
#endif
*/

int elan_iap_open(struct inode *inode, struct file *filp)
{
	pr_err("[ELAN]into elan_iap_open\n");
	if (private_ts == NULL)
		pr_err("private_ts is NULL~~~");

	return 0;
}

int elan_iap_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static ssize_t elan_iap_write(struct file *filp, const char *buff,
			      size_t count, loff_t *offp)
{
	int ret;
	char *tmp;

	/*++++i2c transfer start+++++++ */
	struct i2c_adapter *adap = private_ts->client->adapter;
	struct i2c_msg msg;
	/*++++i2c transfer end+++++++ */

	pr_info("[ELAN]into elan_iap_write\n");
	if (count > 8192)
		count = 8192;

	tmp = kmalloc(count, GFP_KERNEL);

	if (tmp == NULL)
		return -ENOMEM;

	if (copy_from_user(tmp, buff, count))
		return -EFAULT;

	/*++++i2c transfer start+++++++ */
	msg.addr = file_fops_addr;
	msg.flags = 0x00;
	msg.len = count;
	msg.buf = (char *)tmp;

	ret = i2c_transfer(adap, &msg, 1);

	/*ret = i2c_master_send(private_ts->client, tmp, count); */
	/*++++i2c transfer end+++++++ */

	kfree(tmp);

	return (ret == 1) ? count : ret;

}

ssize_t elan_iap_read(struct file *filp, char *buff, size_t count,
		      loff_t *offp)
{
	char *tmp;
	int ret;
	long rc;
	/*++++i2c transfer start+++++++ */
	struct i2c_adapter *adap = private_ts->client->adapter;
	struct i2c_msg msg;
	/*++++i2c transfer end+++++++ */

	pr_err("[ELAN]into elan_iap_read\n");
	if (count > 8192)
		count = 8192;

	tmp = kmalloc(count, GFP_KERNEL);

	if (tmp == NULL)
		return -ENOMEM;
	/*++++i2c transfer start+++++++ */

	msg.addr = file_fops_addr;

	msg.flags = 0x00;
	msg.flags |= I2C_M_RD;
	msg.len = count;
	msg.buf = tmp;

	ret = i2c_transfer(adap, &msg, 1);
	/*ret = i2c_master_recv(private_ts->client, tmp, count); */
	/*++++i2c transfer end+++++++ */
	if (ret >= 0)
		rc = copy_to_user(buff, tmp, count);

	kfree(tmp);

	return (ret == 1) ? count : ret;

}

static long elan_iap_ioctl(struct file *filp, unsigned int cmd,
			   unsigned long arg)
{
	int __user *ip = (int __user *)arg;
	pr_err("[ELAN]into elan_iap_ioctl\n");
	pr_err("cmd value %x\n", cmd);

	switch (cmd) {
	case IOCTL_I2C_SLAVE:

		file_fops_addr = 0x15;
		break;
	case IOCTL_MAJOR_FW_VER:
		break;
	case IOCTL_MINOR_FW_VER:
		break;
	case IOCTL_RESET:

		gpio_set_value(GPIO_TS_RESET, 0);
		msleep(20);

		gpio_set_value(GPIO_TS_RESET, 1);
		msleep(25);

		break;
	case IOCTL_IAP_MODE_LOCK:
		if (work_lock == 0) {
			work_lock = 1;

			disable_irq(1);
			cancel_work_sync(&private_ts->work);
		}
		break;
	case IOCTL_IAP_MODE_UNLOCK:
		if (work_lock == 1) {
			work_lock = 0;

			enable_irq(1);
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
		return elan_ktf2k_ts_rough_calibrate(private_ts->client);
	case IOCTL_I2C_INT:
		put_user(gpio_get_value(private_ts->irq), ip);
		break;
	case IOCTL_RESUME:
		elan_ktf2k_ts_resume(&private_ts->client->dev);
		break;
	case IOCTL_CIRCUIT_CHECK:
		return circuit_ver;
		break;
	case IOCTL_POWER_LOCK:
		power_lock = 1;
		break;
	case IOCTL_POWER_UNLOCK:
		power_lock = 0;
		break;
#if IAP_PORTION
	case IOCTL_GET_UPDATE_PROGREE:
		update_progree = (int __user)arg;
		break;

	case IOCTL_FW_UPDATE:
		Update_FW_One(private_ts->client, 0);
#endif
	default:
		break;
	}
	return 0;
}

const struct file_operations elan_touch_fops = {
	.open = elan_iap_open,
	.write = elan_iap_write,
	.read = elan_iap_read,
	.release = elan_iap_release,
	.unlocked_ioctl = elan_iap_ioctl,
};

#if IAP_PORTION
int EnterISPMode(struct i2c_client *client, uint8_t *isp_cmd)
{
	char buff[4] = { 0 };
	int len = 0;

	len = i2c_master_send(private_ts->client, isp_cmd, sizeof(isp_cmd));
	if (len != sizeof(buff)) {
		pr_err("[ELAN] ERROR: EnterISPMode fail! len=%d\r\n",
		       len);
		return -EPERM;
	} else
		pr_erro(
		       "[ELAN] IAPMode write data successfully! cmd = [%2x, %2x, %2x, %2x]\n",
		       isp_cmd[0], isp_cmd[1], isp_cmd[2], isp_cmd[3]);
	return 0;
}

int ExtractPage(struct file *filp, uint8_t *szPage, int byte)
{
	int len = 0;

	len = filp->f_op->read(filp, szPage, byte, &filp->f_pos);
	if (len != byte) {
		pr_erro(
		       "[ELAN] ExtractPage ERROR: read page error, read error. len=%d\r\n",
	       len);
	return -EPERM;
	}

	return 0;
}

int WritePage(uint8_t *szPage, int byte)
{
	int len = 0;

	len = i2c_master_send(private_ts->client, szPage, byte);
	if (len != byte) {
		pr_erro(
		       "[ELAN] ERROR: write page error, write error. len=%d\r\n",
		       len);
		return -EPERM;
	}

	return 0;
}

int GetAckData(struct i2c_client *client)
{
	int len = 0;

	char buff[2] = { 0 };

	len = i2c_master_recv(private_ts->client, buff, sizeof(buff));
	if (len != sizeof(buff)) {
		pr_erro(
		       "[ELAN] ERROR: read data error, write 50 times error. len=%d\r\n",
		       len);
		return -EPERM;
	}

	pr_info(KERN_INFO "[ELAN] GetAckData:%x,%x", buff[0], buff[1]);
	if (buff[0] == 0xaa)
		return ACK_OK;
	else if (buff[0] == 0x55 && buff[1] == 0x55)
		return ACK_REWRITE;
	else
		return ACK_Fail;

	return 0;
}

void print_progress(int page, int ic_num, int j)
{
	int i, percent, page_tatol, percent_tatol;
	char str[256];
	str[0] = '\0';
	for (i = 0; i < ((page) / 10); i++) {
		str[i] = '#';
		str[i + 1] = '\0';
	}

	page_tatol = page + 249 * (ic_num - j);
	percent = ((100 * page) / (249));
	percent_tatol = ((100 * page_tatol) / (249 * ic_num));

	if ((page) == (249))
		percent = 100;

	if ((page_tatol) == (249 * ic_num))
		percent_tatol = 100;

	pr_err("\rprogress %s| %d%%", str, percent);

	if (page == (249))
		pr_info("\n");
}

/*
 * Restet and (Send normal_command ?)
 * Get Hello Packet
 */
int IAPReset(struct i2c_client *client)
{
	int res;

	gpio_set_value(SYSTEM_RESET_PIN_SR, 0);
	msleep(20);
	gpio_set_value(SYSTEM_RESET_PIN_SR, 1);
	msleep(100);

	pr_err("[ELAN] read Hello packet data!\n");
	res = __hello_packet_handler(client);
	return res;
}

int Update_FW_One(struct i2c_client *client, int recovery)
{
	int res = 0, ic_num = 1;
	int iPage = 0, rewriteCnt = 0;
	int i = 0;
	uint8_t data;

	int restartCnt = 0;

	int byte_count;
	uint8_t *szBuff = NULL;
	int curIndex = 0;
	uint8_t isp_cmd[] = { 0x54, 0x00, 0x12, 0x34 };

	private_ts = i2c_get_clientdata(client);
	dev_dbg(&client->dev, "[ELAN] %s:  ic_num=%d\n", __func__, ic_num);
IAP_RESTART:
	data = I2C_DATA[0];
	dev_dbg(&client->dev, "[ELAN] %s: address data=0x%x \r\n", __func__,
		data);

	if (recovery != 0x80) {
		pr_err("[ELAN] Firmware upgrade normal mode !\n");
	gpio_set_value(SYSTEM_RESET_PIN_SR, 0);
		msleep(20);
		gpio_set_value(SYSTEM_RESET_PIN_SR, 1);
		msleep(150);
		res = EnterISPMode(private_ts->client, isp_cmd);
	} else
		pr_err("[ELAN] Firmware upgrade recovery mode !\n");

	pr_err("[ELAN] send one byte data:%x,%x",
	       private_ts->client->addr, data);
	res = i2c_master_send(private_ts->client, &data, sizeof(data));
	if (res != sizeof(data))
		pr_err("[ELAN] dummy error code = %d\n", res);
	udelay(10000);

	for (iPage = 1; iPage <= PageNum; iPage++) {
PAGE_REWRITE:
		for (byte_count = 1; byte_count <= 17; byte_count++) {
			if (byte_count != 17) {
				szBuff = file_fw_data + curIndex;
				curIndex = curIndex + 8;
				res = WritePage(szBuff, 8);
			} else {
				szBuff = file_fw_data + curIndex;
				curIndex = curIndex + 4;

				res = WritePage(szBuff, 4);
			}
		}
/*
		szBuff = file_fw_data + curIndex;
		curIndex =  curIndex + PageSize;
		res = WritePage(szBuff, PageSize);
*/

		if (iPage == 249 || iPage == 1)
			msleep(600);
		else
			msleep(50);
		res = GetAckData(private_ts->client);

		if (ACK_OK != res) {
			msleep(50);
			pr_erro(
			       "[ELAN] ERROR: GetAckData fail! res=%d\r\n",
			       res);
			if (res == ACK_REWRITE) {
				rewriteCnt = rewriteCnt + 1;
				if (rewriteCnt == PAGERETRY) {
					pr_erro(
				       "[ELAN] ID 0x%02x %dth page ReWrite %d times fails!\n",
					       data, iPage, PAGERETRY);
					return E_FD;
				} else {
					pr_info(
					       "[ELAN] ---%d--- page ReWrite %d times!\n",
					       iPage, rewriteCnt);
					goto PAGE_REWRITE;
				}
			} else {
				restartCnt = restartCnt + 1;
				if (restartCnt >= 5) {
					pr_erro(
					       "[ELAN] ID 0x%02x ReStart %d times fails!\n",
					       data, IAPRESTART);
					return E_FD;
				} else {
					pr_info(
					       "[ELAN] ===%d=== page ReStart %d times!\n",
					       iPage, restartCnt);
					goto IAP_RESTART;
				}
			}
		} else {
		pr_err("  data : 0x%02x ", data);
			rewriteCnt = 0;
			print_progress(iPage, ic_num, i);
		}

		udelay(10000);
	}

	if (IAPReset(client) > 0)
		pr_err("[ELAN] Update ALL Firmware successfully!\n");
	return 0;
}

#endif

static ssize_t elan_ktf2k_gpio_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct elan_ktf2k_ts_data *ts = private_ts;

	ret = gpio_get_value(ts->irq);
	pr_debug("GPIO_TP_INT_N=%d\n", ts->irq);
	snprintf(buf, 20, "GPIO_TP_INT_N=%d\n", ret);
	ret = strlen(buf) + 1;
	return ret;
}

static DEVICE_ATTR(gpio, S_IRUGO, elan_ktf2k_gpio_show, NULL);

static ssize_t elan_ktf2k_vendor_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct elan_ktf2k_ts_data *ts = private_ts;

	snprintf(buf, 50, "%s_x%4.4x\n", "ELAN_KTF2K", ts->fw_ver);
	ret = strlen(buf) + 1;
	return ret;
}

static DEVICE_ATTR(vendor, S_IRUGO, elan_ktf2k_vendor_show, NULL);

static struct kobject *android_touch_kobj;

static int elan_ktf2k_touch_sysfs_init(void)
{
	int ret;

	android_touch_kobj = kobject_create_and_add("android_touch", NULL);
	if (android_touch_kobj == NULL) {
		pr_err("[elan]%s: subsystem_register failed\n",
		       __func__);
		ret = -ENOMEM;
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_gpio.attr);
	if (ret) {
		pr_err("[elan]%s: sysfs_create_file failed\n",
		       __func__);
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_vendor.attr);
	if (ret) {
		pr_err("[elan]%s: sysfs_create_group failed\n",
		       __func__);
		return ret;
	}
	return 0;
}

static void elan_touch_sysfs_deinit(void)
{
	sysfs_remove_file(android_touch_kobj, &dev_attr_vendor.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_gpio.attr);
	kobject_del(android_touch_kobj);
}

static int __elan_ktf2k_ts_poll(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	int status = 0, retry = 10;

	do {
		status = gpio_get_value(ts->irq);
		pr_err("%s: status = %d\n", __func__, status);
		retry--;
		msleep(20);
	} while (status == 1 && retry > 0);

	pr_err("[elan]%s: poll interrupt status %s\n",
	       __func__, status == 1 ? "high" : "low");
	return (status == 0 ? 0 : -ETIMEDOUT);
}

static int elan_ktf2k_ts_poll(struct i2c_client *client)
{
	return __elan_ktf2k_ts_poll(client);
}

static int elan_ktf2k_ts_get_data(struct i2c_client *client, uint8_t *cmd,
				  uint8_t *buf, size_t size)
{
	int rc;

	dev_dbg(&client->dev, "[elan]%s: enter\n", __func__);

	if (buf == NULL)
		return -EINVAL;

	if ((i2c_master_send(client, cmd, 4)) != 4) {
		dev_err(&client->dev,
			"[elan]%s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	rc = elan_ktf2k_ts_poll(client);
	if (rc < 0)
		return -EINVAL;
	else {
		if (i2c_master_recv(client, buf, size) != size ||
		    buf[0] != CMD_S_PKT)
			return -EINVAL;
	}

	return 0;
}

static int __hello_packet_handler(struct i2c_client *client)
{
	int rc;
	uint8_t buf_recv[8] = { 0 };

/*
	rc = elan_ktf2k_ts_poll(client);
	if (rc < 0) {
		printk( "[elan] %s: Int poll failed!\n", __func__);
		RECOVERY=0x80;
		return RECOVERY;

	}
*/
	rc = i2c_master_recv(client, buf_recv, 8);
	pr_info(
	       "[elan] %s: hello packet %2x:%2X:%2x:%2x:%2x:%2X:%2x:%2x\n",
	       __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3],
	       buf_recv[4], buf_recv[5], buf_recv[6], buf_recv[7]);

	if (buf_recv[0] == 0x55 && buf_recv[1] == 0x55 &&
	    buf_recv[2] == 0x80 && buf_recv[3] == 0x80) {
		RECOVERY = 0x80;
		return RECOVERY;
	}
	if (buf_recv[0] != 0x55 && buf_recv[1] != 0x55)
		return -EPERM;

	return 0;
}

static int __fw_packet_handler(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	int rc;
	int major, minor;
	/* Get Firmware Version */
	uint8_t cmd[] = { CMD_R_PKT, 0x00, 0x00, 0x01 };
	/*Get x resolution */
	uint8_t cmd_x[] = { 0x53, 0x60, 0x00, 0x00 };
	/*Get y resolution */
	uint8_t cmd_y[] = { 0x53, 0x63, 0x00, 0x00 };
	/*Get firmware ID */
	uint8_t cmd_id[] = { 0x53, 0xf0, 0x00, 0x01 };

	uint8_t buf_recv[4] = { 0 };

	rc = elan_ktf2k_ts_get_data(client, cmd, buf_recv, 4);
	if (rc < 0)
		return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_ver = major << 8 | minor;
	FW_VERSION = ts->fw_ver;

	rc = elan_ktf2k_ts_get_data(client, cmd_id, buf_recv, 4);
	if (rc < 0)
		return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_id = major << 8 | minor;
	FW_ID = ts->fw_id;

	rc = elan_ktf2k_ts_get_data(client, cmd_x, buf_recv, 4);
	if (rc < 0)
		return rc;
	minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
	ts->x_resolution = minor;
	X_RESOLUTION = ts->x_resolution;

	rc = elan_ktf2k_ts_get_data(client, cmd_y, buf_recv, 4);
	if (rc < 0)
		return rc;
	minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
	ts->y_resolution = minor;
	Y_RESOLUTION = ts->y_resolution;

	pr_err("[elan] %s: firmware version: 0x%4.4x\n",
	       __func__, ts->fw_ver);
	pr_err("[elan] %s: firmware ID: 0x%4.4x\n",
	       __func__, ts->fw_id);
	pr_err("[elan] %s: x resolution: %d, y resolution: %d\n",
	       __func__, X_RESOLUTION, Y_RESOLUTION);

	return 0;
}

static inline int elan_ktf2k_ts_parse_xy(uint8_t *data,
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

static int elan_ktf2k_ts_setup(struct i2c_client *client)
{

	int rc;

	rc = __hello_packet_handler(client);
	if (rc < 0)
		return rc;

	pr_err("[elan] hellopacket's rc = %d\n", rc);

	msleep(400);
	if (rc != 0x80) {
		rc = __fw_packet_handler(client);
	if (rc < 0)
			pr_info(
			       "[elan] %s, fw_packet_handler fail, rc = %d",
			       __func__, rc);
		dev_dbg(&client->dev, "[elan] %s: firmware checking done.\n",
			__func__);

		if (FW_VERSION == 0x00) {
			rc = 0x80;
			pr_info(
			       "[elan] FW_VERSION = %d, last FW update fail\n",
			       FW_VERSION);
		}
	}
	return rc;
}

static int elan_ktf2k_ts_rough_calibrate(struct i2c_client *client)
{
	uint8_t cmd[] = { CMD_W_PKT, 0x29, 0x00, 0x01 };

	pr_err("[elan] %s: enter\n", __func__);
	dev_info(&client->dev,
		 "[elan] dump cmd: %02x, %02x, %02x, %02x\n",
		 cmd[0], cmd[1], cmd[2], cmd[3]);

	if ((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd)) {
		dev_err(&client->dev,
			"[elan] %s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int elan_ktf2k_ts_set_power_state(struct i2c_client *client, int state)
{
	uint8_t cmd[] = { CMD_W_PKT, 0x50, 0x00, 0x01 };

	dev_dbg(&client->dev, "[elan] %s: enter\n", __func__);

	cmd[1] |= (state << 3);

	dev_dbg(&client->dev,
		"[elan] dump cmd: %02x, %02x, %02x, %02x\n",
		cmd[0], cmd[1], cmd[2], cmd[3]);

	if ((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd)) {
		dev_err(&client->dev,
			"[elan] %s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int elan_ktf2k_ts_get_power_state(struct i2c_client *client)
{
	int rc = 0;
	uint8_t cmd[] = { CMD_R_PKT, 0x50, 0x00, 0x01 };
	uint8_t buf[4], power_state;

	rc = elan_ktf2k_ts_get_data(client, cmd, buf, 4);
	if (rc)
		return rc;

	power_state = buf[1];
	dev_dbg(&client->dev, "[elan] dump repsponse: %0x\n", power_state);
	power_state = (power_state & PWR_STATE_MASK) >> 3;
	dev_dbg(&client->dev, "[elan] power state = %s\n",
		power_state == PWR_STATE_DEEP_SLEEP ?
		"Deep Sleep" : "Normal/Idle");

	return power_state;
}

static int elan_ktf2k_ts_recv_data(struct i2c_client *client, uint8_t *buf)
{

	int rc, bytes_to_recv = PACKET_SIZE;

	if (buf == NULL)
		return -EINVAL;

	memset(buf, 0, bytes_to_recv);

	rc = i2c_master_recv(client, buf, bytes_to_recv);
	if (rc != bytes_to_recv)
		pr_err("[elan] The first package error.\n");
/*
	pr_err("[elan_debug 0-7] %x %x %x %x %x %x %x %x\n", buf[0],
	       buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
	pr_err("[elan_debug 8-15] %x %x %x %x %x %x %x %x\n", buf[8],
	       buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]);
	pr_err("[elan_debug 16-17] %x %x\n", buf[16], buf[17]);
*/
	udelay(1000);

	return rc;
}

//static uint8_t mask_finger[5] = { 0x08, 0x10, 0x20, 0x40, 0x80 };
//static uint8_t temp_table;

static void elan_ktf2k_ts_report_data(struct i2c_client *client, uint8_t *buf)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	struct input_dev *idev = ts->input_dev;
//	int touch_finger[5] = { 0 };
	uint16_t x, y;
	uint8_t num, idx = 0;
	int position;
	int btn_idx = 0, finger_num, fbits;
	static uint16_t pre_fbits=0;
	uint16_t fbits_tmp=0;

	if ((buf[0] == NORMAL_PKT) || (buf[0] == FIVE_FINGERS_PKT)){
		finger_num = 5;
		num = buf[1] & 0x07; 
		fbits = buf[1] >>3;
		idx=2;
		btn_idx=17;
	}
	else if(buf[0] == 0x62){
		finger_num = 10;
		num = buf[2] & 0x0f; 
		fbits = buf[2] & 0x30;	
		fbits = (fbits << 4) | buf[1]; 
		idx=3;
		btn_idx=33;
	}
	else{
		return;
	}

	fbits_tmp = fbits;
	if(fbits || pre_fbits){
		for(position=0; position<finger_num;position++){
			if(fbits&0x01){
				elan_ktf2k_ts_parse_xy(&buf[idx], &x, &y);
				input_mt_report_slot_state(idev, MT_TOOL_FINGER, 1);
				input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 8);
				input_report_abs(idev, ABS_MT_POSITION_X, x);
				input_report_abs(idev, ABS_MT_POSITION_Y, y);
				printk("Touch ID:%d, X:%d, Y:%d, down", position, x, y);
			}
			else if(pre_fbits&0x01){
				input_mt_report_slot_state(idev, MT_TOOL_FINGER, 0);
				printk("Touch ID:%d up", position);
			}
			fbits >>= 1;
			pre_fbits >>= 1;
			idx += 3;	
		}
	}	
	else{
		input_report_key(idev, BTN_TOUCH, 1);
		dev_dbg(&client->dev, "no press\n");
		if (button_state == 0) {
			switch (buf[btn_idx]) {
			case 0x81:
				input_report_key(idev, KEY_MENU, 1);
				button_state = KEY_MENU;
				pr_info(
				       "***********************button_state = %d\n",
				       button_state);
				break;

			case 0x41:
				input_report_key(idev, KEY_HOMEPAGE, 1);
				button_state = KEY_HOMEPAGE;
				pr_info(
				       "***********************button_state = %d\n",
				       button_state);
				break;
			case 0x21:
				input_report_key(idev, KEY_BACK, 1);
				button_state = KEY_BACK;
				pr_info(
				       "***********************button_state = %d\n",
				       button_state);
				break;
			}
		} else if (button_state > 0) {
			if (button_state == KEY_MENU) {

				input_report_key(idev, KEY_MENU, 0);
				button_state = 0;
			} else if (button_state == KEY_HOMEPAGE) {
				input_report_key(idev, KEY_HOMEPAGE, 0);
				button_state = 0;
			} else if (button_state == KEY_BACK) {
				input_report_key(idev, KEY_BACK, 0);
				button_state = 0;
			}
		} else if (button_state == -1) {
			dev_dbg(&client->dev, "no press\n");
			input_report_key(idev, BTN_TOUCH, 0);

			button_state = 0;
		}
	}
	
	pre_fbits = fbits_tmp;
	input_sync(idev);
}

static void elan_ktf2k_ts_work_func(struct work_struct *work)
{
	int rc;
	struct elan_ktf2k_ts_data *ts =
	    container_of(work, struct elan_ktf2k_ts_data, work);
	uint8_t buf[PACKET_SIZE] = { 0 };

	rc = elan_ktf2k_ts_recv_data(ts->client, buf);

	if (rc < 0) {
		enable_irq(ts->client->irq);
		return;
	}

	elan_ktf2k_ts_report_data(ts->client, buf);

	enable_irq(ts->client->irq);

	return;
}

static irqreturn_t elan_ktf2k_ts_irq_handler(int irq, void *dev_id)
{
	struct elan_ktf2k_ts_data *ts = dev_id;
	struct i2c_client *client = ts->client;
	pr_err("[elan] %s \n", __func__);
	dev_dbg(&client->dev, "[elan] %s\n", __func__);
	disable_irq_nosync(ts->client->irq);
	queue_work(ts->elan_wq, &ts->work);

	return IRQ_HANDLED;
}

static int elan_ktf2k_ts_register_interrupt(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	int err = 0;

	err = request_threaded_irq(client->irq, elan_ktf2k_ts_irq_handler,
				   NULL, IRQF_TRIGGER_FALLING, client->name,
				   ts);
	if (err)
		dev_err(&client->dev, "[elan] %s: request_irq %d failed\n",
			__func__, client->irq);

	return err;
}

static ssize_t ekth3248_ts_info_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct elan_ktf2k_ts_data *data = dev_get_drvdata(dev);
	return snprintf(buf, ELAN_INFO_MAX_LEN, "%s\n", data->ts_info);
}

static DEVICE_ATTR(ts_info, 0664, ekth3248_ts_info_show, NULL);

static ssize_t ekth3248_mt_protocol_type_show(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	return snprintf(buf, 16, "%s\n", "MT Protocal B");
}

static DEVICE_ATTR(mt_protocol_type, 0664, ekth3248_mt_protocol_type_show,
		   NULL);

static ssize_t ekth3248_enable_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct elan_ktf2k_ts_data *data = dev_get_drvdata(dev);
	unsigned long val;
	int rc;

	if (size > 2)
		return -EINVAL;

	rc = kstrtoul(buf, 10, &val);
	if (rc != 0)
		return rc;

	if (val) {
		data->enable = true;
		elan_ktf2k_ts_resume(dev);
	} else {
		data->enable = false;
		elan_ktf2k_ts_suspend(dev);
	}

	return size;
}

static ssize_t ekth3248_enable_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct elan_ktf2k_ts_data *data = dev_get_drvdata(dev);

	if (data->suspended) {
		dev_info(&data->client->dev, "Already in suspend state\n");
		return snprintf(buf, 4, "%s\n", "0");
	}

	return snprintf(buf, 4, "%s\n", data->enable ? "1" : "0");
}

static DEVICE_ATTR(enable, 0664, ekth3248_enable_show, ekth3248_enable_store);

static ssize_t ekth3248_update_fw_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{

	return snprintf(buf, 8, "%s\n", "N/A\n");

	return 0;
}

static ssize_t ekth3248_update_fw_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	return size;
}

static DEVICE_ATTR(update_fw, 0664, ekth3248_update_fw_show,
		   ekth3248_update_fw_store);

static ssize_t ekth3248_force_update_fw_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t size)
{
	return size;
}

static DEVICE_ATTR(force_update_fw, 0664, ekth3248_update_fw_show,
		   ekth3248_force_update_fw_store);

static ssize_t ekth3248_fw_name_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 8, "%s\n", "N/A\n");
}

static ssize_t ekth3248_fw_name_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
	return size;
}

static DEVICE_ATTR(fw_name, 0664, ekth3248_fw_name_show,
		   ekth3248_fw_name_store);

static int debug_dump_info(struct seq_file *m, void *v)
{
	struct elan_ktf2k_ts_data *data = m->private;

	seq_printf(m, "%s\n", data->ts_info);

	return 0;
}

static int debugfs_dump_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, debug_dump_info, inode->i_private);
}

static const struct file_operations debug_dump_info_fops = {
	.owner = THIS_MODULE,
	.open = debugfs_dump_info_open,
	.read = seq_read,
	.release = single_release,
};

static int debug_suspend_set(void *_data, u64 val)
{
	struct elan_ktf2k_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (val)
		elan_ktf2k_ts_suspend(&data->client->dev);
	else
		elan_ktf2k_ts_resume(&data->client->dev);

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

static int debug_suspend_get(void *_data, u64 *val)
{
	struct elan_ktf2k_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);
	*val = data->suspended;
	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_suspend_fops, debug_suspend_get,
			debug_suspend_set, "%lld\n");

static int debug_data_info(struct seq_file *m, void *v)
{
	struct elan_ktf2k_ts_data *data = m->private;
	int rc;
	uint8_t buf[18];
	uint8_t buf1[64], buf2[64], buf3[32];
	/* Get Firmware Version */
	uint8_t cmd[] = { CMD_R_PKT, 0x00, 0x00, 0x01 };
	/*Get firmware ID */
	uint8_t cmd_id[] = { 0x53, 0xf0, 0x00, 0x01 };

	int major, minor;
	uint8_t buf_recv[4] = { 0 };

	mutex_lock(&data->input_dev->mutex);

	memset(buf, 0, 18);
	memset(buf1, 0, 64);
	memset(buf2, 0, 64);
	memset(buf3, 0, 32);

	rc = i2c_master_recv(private_ts->client, buf, 18);
	if (rc != 18)
		pr_err("[elan] The first package error.\n");

	snprintf(buf1, 64, "[elan_debug 0-7] %x %x %x %x %x %x %x %x\n", buf[0],
	       buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
	snprintf(buf2, 64, "[elan_debug 8-15] %x %x %x %x %x %x %x %x\n",
			buf[8], buf[9], buf[10], buf[11], buf[12], buf[13],
			buf[14], buf[15]);
	snprintf(buf3, 32, "[elan_debug 16-17] %x %x\n", buf[16], buf[17]);

	rc = elan_ktf2k_ts_get_data(private_ts->client, cmd, buf_recv, 4);
	if (rc < 0)
		return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	FW_VERSION = major << 8 | minor;

	rc = elan_ktf2k_ts_get_data(private_ts->client, cmd_id, buf_recv, 4);
	if (rc < 0)
		return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	FW_ID = major << 8 | minor;

	udelay(1000);

	mutex_unlock(&data->input_dev->mutex);

	snprintf(data->ts_data, 128, "%s%s%sFW_VERSION = 0x%x\nFW_ID = 0x%x\n",
				buf1, buf2, buf3, FW_VERSION, FW_ID);
	seq_printf(m, "%s\n", data->ts_data);

	return 0;
}

static int debugfs_data_open(struct inode *inode, struct file *file)
{
	return single_open(file, debug_data_info, inode->i_private);
}

static const struct file_operations debug_data_fops = {
	.owner = THIS_MODULE,
	.open = debugfs_data_open,
	.read = seq_read,
	.release = single_release,
};

#ifdef CONFIG_FB
static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct elan_ktf2k_ts_data *ts =
	    container_of(self, struct elan_ktf2k_ts_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
	    ts && ts->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			elan_ktf2k_ts_resume(&ts->client->dev);
		else if (*blank == FB_BLANK_POWERDOWN)
			elan_ktf2k_ts_suspend(&ts->client->dev);
	}

	return 0;
}
#endif

static int elan_ktf2k_ts_power_init(struct elan_ktf2k_ts_data *data, bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;

	data->vdd = regulator_get(&data->client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		rc = PTR_ERR(data->vdd);
		dev_err(&data->client->dev,
			"Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		rc = regulator_set_voltage(data->vdd, ELAN_VTG_MIN_UV,
					   ELAN_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator set_vtg failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}

	data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
	if (IS_ERR(data->vcc_i2c)) {
		rc = PTR_ERR(data->vcc_i2c);
		dev_err(&data->client->dev,
			"Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0) {
		rc = regulator_set_voltage(data->vcc_i2c, ELAN_I2C_VTG_MIN_UV,
					   ELAN_I2C_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}

	return 0;

reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);
reg_vdd_set_vtg:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, ELAN_VTG_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, ELAN_VTG_MAX_UV);

	regulator_put(data->vdd);

	if (regulator_count_voltages(data->vcc_i2c) > 0)
		regulator_set_voltage(data->vcc_i2c, 0, ELAN_I2C_VTG_MAX_UV);

	regulator_put(data->vcc_i2c);
	return 0;
}

static int elan_ktf2k_ts_power_on(struct elan_ktf2k_ts_data *data, bool on)
{
	int rc;

	if (!on)
		goto power_off;

	rc = regulator_enable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(data->vdd);
	}

	return rc;

power_off:
	rc = regulator_disable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_disable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c disable failed rc=%d\n", rc);
		/*regulator_enable(data->vdd);*/
	}

	return rc;
}

static const struct dev_pm_ops ektf2k_ts_pm_ops = {
#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
	.suspend = elan_ktf2k_ts_suspend,
	.resume = elan_ktf2k_ts_resume,
#endif
};

static int elan_ktf2k_ts_parse_dt(struct device *dev,
				  struct elan_ktf2k_i2c_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	int rc;

	u32 temp_val;

	pdata->name = "elan";
	rc = of_property_read_string(np, "elan,name", &pdata->name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read name\n");
		return rc;
	}

	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "elan,reset-gpio",
						    0,
						    &pdata->reset_gpio_flags);
	if (pdata->reset_gpio < 0)
		return pdata->reset_gpio;
	GPIO_TS_RESET = pdata->reset_gpio;

	pdata->irq_gpio = of_get_named_gpio_flags(np, "elan,irq-gpio",
						  0, &pdata->irq_gpio_flags);
	if (pdata->irq_gpio < 0)
		return pdata->irq_gpio;
	rc = of_property_read_u32(np, "elan,soft-reset-delay-ms", &temp_val);
	if (!rc)
		pdata->soft_rst_dly = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "elan,num-max-touches", &temp_val);
	if (!rc)
		pdata->num_max_touches = temp_val;
	else
		return rc;

	pdata->fw_vkey_support = of_property_read_bool(np,
						       "elan,fw-vkey-support");
	return 0;
}

static int elan_ktf2k_ts_probe(struct i2c_client *client,
			       const struct i2c_device_id *id)
{
	int err = 0;
	int fw_err = 0;
	struct elan_ktf2k_i2c_platform_data *pdata;
	struct elan_ktf2k_ts_data *ts;
	struct dentry *temp;

	button_state = 0;
	tpd_down_flag = 0;

#if IAP_PORTION
	ic_status = 0x00;
	update_progree = 0;
	is_OldBootCode = 0;
#endif

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				     sizeof(struct
					    elan_ktf2k_i2c_platform_data),
				     GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		err = elan_ktf2k_ts_parse_dt(&client->dev, pdata);
		if (err) {
			dev_err(&client->dev, "DT parsing failed\n");
			return err;
		}
	} else
		pdata = client->dev.platform_data;

	if (!pdata) {
		dev_err(&client->dev, "Invalid pdata\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("[elan] %s: i2c check functionality error\n",
		       __func__);
		return -ENODEV;
	}

	ts = kzalloc(sizeof(struct elan_ktf2k_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		pr_err(
		       "[elan] %s: allocate elan_ktf2k_ts_data failed\n",
		       __func__);
		return -ENOMEM;
	}
	ts->client = client;
	ts->pdata = pdata;

	err = elan_ktf2k_ts_power_init(ts, true);
	if (err) {
		dev_err(&client->dev, "power init failed");
		goto err_alloc_memory;
	}

	err = elan_ktf2k_ts_power_on(ts, true);
	if (err) {
		dev_err(&client->dev, "power init failed");
		goto err_power_init;
	}

	err = gpio_request(pdata->irq_gpio, "ts_irq_gpio");
	if (err) {
		dev_err(&client->dev, "irq gpio request failed");
		goto err_power_on;
	}
	err = gpio_direction_input(pdata->irq_gpio);
	if (err) {
		dev_err(&client->dev, "set_direction for irq gpio failed\n");
		goto free_irq_gpio;
	}

	err = gpio_request(pdata->reset_gpio, "ts_reset_gpio");
	if (err) {
		dev_err(&client->dev, "reset gpio request failed");
		goto free_irq_gpio;
	}

	err = gpio_direction_output(pdata->reset_gpio, 0);
	if (err) {
		dev_err(&client->dev, "set_direction for reset gpio failed\n");
		goto free_reset_gpio;
	}

	ts->elan_wq = create_singlethread_workqueue("elan_wq");
	if (!ts->elan_wq) {
		pr_err("[elan] %s: create workqueue failed\n",
		       __func__);
		err = -ENOMEM;
		goto free_reset_gpio;
	}

	INIT_WORK(&ts->work, elan_ktf2k_ts_work_func);

	ts->irq = pdata->irq_gpio;
	i2c_set_clientdata(client, ts);

	gpio_set_value(pdata->reset_gpio, 1);
	msleep(200);
	ts->client->irq = gpio_to_irq(pdata->irq_gpio);

	fw_err = elan_ktf2k_ts_setup(client);
	if (fw_err < 0) {
		pr_err("No Elan chip inside\n");
		goto err_request_gpio_fail;
	}

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		return -ENOMEM;
		dev_err(&client->dev,
			"[elan] Failed to allocate input device\n");
	}
	ts->input_dev->name = "elan-touchscreen";

	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, 832, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, 1536, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);

	input_set_capability(ts->input_dev, EV_KEY, KEY_HOMEPAGE);
	input_set_capability(ts->input_dev, EV_KEY, KEY_BACK);
	input_set_capability(ts->input_dev, EV_KEY, KEY_MENU);
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	input_mt_init_slots(ts->input_dev, pdata->num_max_touches, 0);

	err = input_register_device(ts->input_dev);
	if (err) {
		dev_err(&client->dev,
			"[elan]%s: unable to register %s input device\n",
			__func__, ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	private_ts = ts;
	elan_ktf2k_ts_register_interrupt(ts->client);

	private_ts = ts;

	err = device_create_file(&client->dev, &dev_attr_ts_info);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto err_input_register_device_failed;
	}

	err = device_create_file(&client->dev, &dev_attr_mt_protocol_type);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto free_ts_info;
	}

	err = device_create_file(&client->dev, &dev_attr_enable);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto free_type;
	}

	err = device_create_file(&client->dev, &dev_attr_fw_name);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto free_enable;
	}

	err = device_create_file(&client->dev, &dev_attr_update_fw);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto free_fw_name_sys;
	}

	err = device_create_file(&client->dev, &dev_attr_force_update_fw);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto free_update_fw_sys;
	}

	ts->ts_info = devm_kzalloc(&client->dev, ELAN_INFO_MAX_LEN, GFP_KERNEL);
	if (!ts->ts_info) {
		dev_err(&client->dev, "Not enough memory\n");
		goto free_force_update_fw_sys;
	}
	ts->enable = true;

	STORE_TS_INFO(ts->ts_info, ts->pdata->name,
		      ts->pdata->num_max_touches,
			  FW_VERSION,
		      ts->pdata->fw_vkey_support ? "yes" : "no");
	ts->dir = debugfs_create_dir(DEBUG_DIR_NAME, NULL);
	if (ts->dir == NULL || IS_ERR(ts->dir)) {
		pr_err("debugfs_create_dir failed(%ld)\n", PTR_ERR(ts->dir));
		err = PTR_ERR(ts->dir);
		goto free_debug_dir;
	}

	ts->ts_data = devm_kzalloc(&client->dev,
			128, GFP_KERNEL);
	if (!ts->ts_data) {
		dev_err(&client->dev, "Not enough memory\n");
		goto free_debug_dir;
	}

	temp =
		debugfs_create_file("data", S_IRUSR, ts->dir,
				ts, &debug_data_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("suspend", S_IRUSR | S_IWUSR, ts->dir,
			ts, &debug_suspend_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp =
		debugfs_create_file("dump_info", S_IRUSR | S_IWUSR,
				ts->dir, ts, &debug_dump_info_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	elan_ktf2k_touch_sysfs_init();

	dev_info(&client->dev,
			"[elan] Start touchscreen %s in interrupt mode\n",
			ts->input_dev->name);

	ts->firmware.minor = MISC_DYNAMIC_MINOR;
	ts->firmware.name = "elan-iap";
	ts->firmware.fops = &elan_touch_fops;
	ts->firmware.mode = S_IFREG | S_IRWXUGO;

	if (misc_register(&ts->firmware) < 0)
		pr_err("[ELAN]misc_register failed!!");
	else
		pr_err("[ELAN]misc_register finished!!");

#if IAP_PORTION
	if (1) {
		pr_err("[ELAN]misc_register finished!!");
		work_lock = 1;
		disable_irq(ts->client->irq);
		cancel_work_sync(&ts->work);

		power_lock = 1;
		/* FW ID & FW VER */
		pr_info(
				"[7bd0]=0x%02x,[7bd1]=0x%02x,[7bd2]=0x%02x,[7bd3]=0x%02x\n",
				file_fw_data[31696], file_fw_data[31697],
				file_fw_data[31698], file_fw_data[31699]);
		New_FW_ID = file_fw_data[31699] << 8 | file_fw_data[31698];
		New_FW_VER = file_fw_data[31697] << 8 | file_fw_data[31696];
		pr_err("FW_ID=0x%x, New_FW_ID=0x%x\n",
				FW_ID, New_FW_ID);
		pr_err("FW_VERSION=0x%x, New_FW_VER=0x%x\n",
				FW_VERSION, New_FW_VER);

		/* for firmware auto-upgrade
		   if (New_FW_ID   ==  FW_ID){
		   if (New_FW_VER > (FW_VERSION))
		   Update_FW_One(client, RECOVERY);
		   } else {
		   printk("FW_ID is different!");
		   }
		 */
		if (FW_ID == 0)
			RECOVERY = 0x80;
		Update_FW_One(client, RECOVERY);
		power_lock = 0;

		work_lock = 0;
		enable_irq(ts->client->irq);
	}
#endif

#ifdef CONFIG_FB
	ts->suspended = false;
	ts->fb_notif.notifier_call = fb_notifier_callback;
	err = fb_register_client(&ts->fb_notif);

	if (err)
		dev_err(&client->dev, "Unable to register fb_notifier: %d\n",
				err);
#endif
	return 0;
free_debug_dir:
	debugfs_remove_recursive(ts->dir);
	kfree(ts->ts_data);
free_force_update_fw_sys:
	device_remove_file(&client->dev, &dev_attr_force_update_fw);
free_update_fw_sys:
	device_remove_file(&client->dev, &dev_attr_update_fw);
free_fw_name_sys:
	device_remove_file(&client->dev, &dev_attr_fw_name);
free_enable:
	device_remove_file(&client->dev, &dev_attr_enable);
free_type:
	device_remove_file(&client->dev, &dev_attr_mt_protocol_type);
free_ts_info:
	device_remove_file(&client->dev, &dev_attr_ts_info);

err_input_register_device_failed:
	if (ts->input_dev)
		input_free_device(ts->input_dev);
err_request_gpio_fail:
	if (ts->elan_wq)
		destroy_workqueue(ts->elan_wq);
free_reset_gpio:
	if (gpio_is_valid(pdata->reset_gpio))
		gpio_free(pdata->reset_gpio);
free_irq_gpio:
	if (gpio_is_valid(pdata->irq_gpio))
		gpio_free(pdata->irq_gpio);
err_power_on:
	elan_ktf2k_ts_power_on(ts, false);
err_power_init:
	elan_ktf2k_ts_power_init(ts, false);
err_alloc_memory:
	kfree(ts);
	return err;
}

static int elan_ktf2k_ts_remove(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);

	elan_touch_sysfs_deinit();

	free_irq(client->irq, ts);

	if (ts->elan_wq)
		destroy_workqueue(ts->elan_wq);
	input_unregister_device(ts->input_dev);
	kfree(ts);

	return 0;
}

static int elan_ktf2k_ts_suspend(struct device *dev)
{
	struct elan_ktf2k_ts_data *ts = dev_get_drvdata(dev);
	int rc = 0;

	pr_err("[elan] %s: enter\n", __func__);

	if (ts->suspended) {
		dev_info(dev, "Already in suspend state\n");
		return 0;
	}

	disable_irq(ts->client->irq);

	rc = cancel_work_sync(&ts->work);
	if (rc)
		enable_irq(ts->client->irq);

	rc = elan_ktf2k_ts_set_power_state(ts->client,
			PWR_STATE_DEEP_SLEEP);

	ts->suspended = true;

	return 0;
}

static int elan_ktf2k_ts_resume(struct device *dev)
{
	struct elan_ktf2k_ts_data *ts = dev_get_drvdata(dev);

	int rc = 0, retry = 5;

	pr_err("[elan] %s: enter\n", __func__);

	if (!ts->suspended) {
		dev_dbg(dev, "Already in awake state\n");
		return 0;
	}

	do {
		rc = elan_ktf2k_ts_set_power_state(ts->client,
				PWR_STATE_NORMAL);
		rc = elan_ktf2k_ts_get_power_state(ts->client);
		if (rc != PWR_STATE_NORMAL)
			pr_err(
					"[elan] %s: wake up tp failed! err = %d\n",
					__func__, rc);
		else
			break;
	} while (--retry);

	enable_irq(ts->client->irq);
	ts->suspended = false;

	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id ektf3xxx_match_table[] = {
	{.compatible = "Elan-TS,EKTF3248",},
	{},
};
#else
#define  ektf3xxx_match_table NULL
#endif
static const struct i2c_device_id elan_ktf2k_ts_id[] = {
	{ELAN_KTF2K_NAME, 0},
	{}
};

static struct i2c_driver ektf2k_ts_driver = {
	.probe = elan_ktf2k_ts_probe,
	.remove = elan_ktf2k_ts_remove,
	.id_table = elan_ktf2k_ts_id,
	.driver = {
		.name = ELAN_KTF2K_NAME,
		.of_match_table = ektf3xxx_match_table,
#ifdef CONFIG_PM
		.pm = &ektf2k_ts_pm_ops,
#endif
	},
};

static int __init elan_ktf2k_ts_init(void)
{
	pr_err("[elan] %s driver version 0x0003\n", __func__);
	return i2c_add_driver(&ektf2k_ts_driver);
}

static void __exit elan_ktf2k_ts_exit(void)
{
	i2c_del_driver(&ektf2k_ts_driver);
	return;
}

module_init(elan_ktf2k_ts_init);
module_exit(elan_ktf2k_ts_exit);

MODULE_DESCRIPTION("ELAN KTF2K Touchscreen Driver");
MODULE_LICENSE("GPL");
