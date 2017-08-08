/* lge_ts_melfas.c
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

/* History :
 *
 */

#include "lge_ts_melfas.h"
#if defined(CONFIG_LGE_LCD_ESD)
int lge_mdss_report_panel_dead(void);
#endif
struct mit_data* ts_data = NULL;

#define ts_pdata		((ts)->pdata)
#define ts_caps		(ts_pdata->caps)
#define ts_role		(ts_pdata->role)
#define ts_pwr		(ts_pdata->pwr)

/* LPWG Control Value */
#define IDLE_REPORTRATE_CTRL    1
#define ACTIVE_REPORTRATE_CTRL  2
#define SENSITIVITY_CTRL        3

#define TCI_ENABLE_CTRL         11
#define TOUCH_SLOP_CTRL         12
#define TAP_MIN_DISTANCE_CTRL   13
#define TAP_MAX_DISTANCE_CTRL   14
#define MIN_INTERTAP_CTRL       15
#define MAX_INTERTAP_CTRL       16
#define TAP_COUNT_CTRL          17
#define INTERRUPT_DELAY_CTRL    18

#define TCI_ENABLE_CTRL2        21
#define TOUCH_SLOP_CTRL2        22
#define TAP_MIN_DISTANCE_CTRL2  23
#define TAP_MAX_DISTANCE_CTRL2  24
#define MIN_INTERTAP_CTRL2      25
#define MAX_INTERTAP_CTRL2      26
#define TAP_COUNT_CTRL2         27
#define INTERRUPT_DELAY_CTRL2   28

#define LPWG_STORE_INFO_CTRL    31
#define LPWG_START_CTRL         32
#define LPWG_PANEL_DEBUG_CTRL   33
#define LPWG_FAIL_REASON_CTRL   34
#define I2C_RETRY_COUNT			3

u8 event_size = 6;
int event_format = 0;
u8 category = 0;
int lockscreen_stat = 0;

static int mit_get_packet(struct i2c_client *client);
static int mit_power(struct i2c_client* client, int power_ctrl);
void reset_pin_ctrl(struct mit_data* ts, int on_off, int delay);
int mit_power_reset(struct mit_data *ts);


/**
* Reboot chip
*
* Caution : IRQ must be disabled before mip_reboot and enabled after mip_reboot.
*/
void mip_reboot(struct i2c_client *client)
{
	struct mit_data* ts = get_touch_handle_(client);
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

	TOUCH_INFO_MSG("%s : [START]\n", __func__);

	i2c_lock_adapter(adapter);
	mit_power_reset(ts);
	i2c_unlock_adapter(adapter);

	TOUCH_INFO_MSG("%s : [DONE]\n", __func__);
}

int mit_i2c_dummy(struct i2c_client *client, char *write_buf, unsigned int write_len)
{
	int retry = I2C_RETRY_COUNT;
	int res = 0;

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = write_buf,
			.len = write_len,
		},
	};

	while(retry--) {
		TOUCH_INFO_MSG("%s : dummy msg\n", __func__);
		res = i2c_transfer(client->adapter, &msg[0], 1);

		if(res == ARRAY_SIZE(msg)) {
			return 0;
		} else if(res < 0) {
			TOUCH_ERR_MSG("%s [ERROR] i2c_transfer - errno[%d]\n",__func__, res);
		} else if(res != ARRAY_SIZE(msg)) {
			TOUCH_ERR_MSG("%s [ERROR] i2c_transfer - size[%d] result[%d]\n", __func__, ARRAY_SIZE(msg), res);
		} else {
			TOUCH_ERR_MSG("%s [ERROR] unknown error [%d]\n", __func__, res);
		}
	}

	TOUCH_INFO_MSG("%s : send dummy msg done\n", __func__);

	//mip_reboot(client);
	return 1;
}

int mit_i2c_read(struct i2c_client *client, char *write_buf, unsigned int write_len, char *read_buf, unsigned int read_len)
{
	int retry = I2C_RETRY_COUNT;
	int res;

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = write_buf,
			.len = write_len,
		}, {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = read_buf,
			.len = read_len,
		},
	};

	while(retry--) {
		res = i2c_transfer(client->adapter, &msg[0], 1);
		res += i2c_transfer(client->adapter, &msg[1], 1);

		if(res == ARRAY_SIZE(msg)) {
			return 0;
		} else if(res < 0) {
			TOUCH_ERR_MSG("%s [ERROR] i2c_transfer - errno[%d]\n",__func__, res);
		} else if(res != ARRAY_SIZE(msg)) {
			TOUCH_ERR_MSG("%s [ERROR] i2c_transfer - size[%d] result[%d]\n", __func__, ARRAY_SIZE(msg), res);
		} else {
			TOUCH_ERR_MSG("%s [ERROR] unknown error [%d]\n", __func__, res);
		}
	}

	//mip_reboot(client);
	return 1;
}

int mip_i2c_write(struct i2c_client *client, char *write_buf, unsigned int write_len)
{
	int retry = I2C_RETRY_COUNT;
	int res;

	while(retry--) {
		res = i2c_master_send(client, write_buf, write_len);

		if(res == write_len) {
			return 0;
		} else if(res < 0) {

			TOUCH_ERR_MSG("%s [ERROR] i2c_master_send - errno [%d]\n", __func__, res);
		} else if(res != write_len) {
			TOUCH_ERR_MSG("%s [ERROR] length mismatch - write[%d] result[%d]\n", __func__, write_len, res);
		} else {
			TOUCH_ERR_MSG("%s [ERROR] unknown error [%d]\n", __func__, res);
		}
	}

	//mip_reboot(client);
	return 1;
}

static int mit_get_otp(struct mit_data *ts) {
#if 0
	uint8_t read_buf[16] = {0};
	uint8_t write_buf[4] = {0};
	struct i2c_msg msg[] = {
		{
			.addr = ts->client->addr,
			.flags = 0,
			.buf = write_buf,
		},{
			.addr = ts->client->addr,
			.flags = 1,
		},
	};
	write_buf[0] = MIT_REGH_CMD;
	write_buf[1] = MIT_REGL_UCMD;
	write_buf[2] = MIT_UNIV_GET_READ_OTP_STATUS;
	msg[0].len = 3;

	if (i2c_transfer(ts->client->adapter, &msg[0], 1) != 1) {
		TOUCH_INFO_MSG("%s : i2c transfer failed\n", __func__);
		return -EIO;
	}

	if (mit_i2c_read(ts->client, MIT_REGL_UCMD_RESULT_LENGTH, read_buf, 1) < 0) {
		TOUCH_INFO_MSG("%s : Fail to get MIT_REGL_UCMD_RESULT_LENGTH \n", __func__);
		return -EIO;
	}

	if (mit_i2c_read(ts->client, MIT_REGL_UCMD_RESULT, read_buf, 1) < 0) {
		TOUCH_INFO_MSG("%s : Fail to get MIT_REGL_UCMD_RESULT \n", __func__);
		return -EIO;
	}

	ts->module.otp = read_buf[0];
#endif
	return 0;
}

static int mit_get_lpwg_lcd_status(struct mit_data* ts)
{
	u8 wbuf[2] = {0};
	u8 rbuf[1] = {0};

	wbuf[0] = MIP_R0_LPWG;
	wbuf[1] = MIP_R1_LPWG_LCD_STATUS;
	if(mit_i2c_read(ts->client, wbuf, 2, rbuf, 1)) {
		TOUCH_ERR_MSG("%s [ERROR] read LCD Status Register\n", __func__);
		return -EIO;
	}
	ts->pdata->lpwg_lcd_status = rbuf[0];
	return 0;
}

int mip_get_fw_version(struct i2c_client *client, u8 *ver_buf)
{
	u8 rbuf[2] = {0};
	u8 wbuf[2];

	wbuf[0] = MIP_R0_INFO;
	wbuf[1] = MIP_R1_INFO_VERSION_CUSTOM;
	if(mit_i2c_read(client, wbuf, 2, rbuf, 2)) {
		TOUCH_ERR_MSG("%s [ERROR]\n", __func__);
		memset(ver_buf, 0xFF, sizeof(ver_buf));
		return -EIO;
	};

	ver_buf[0] = rbuf[1];
	ver_buf[1] = rbuf[0];

	return 0;
}
/**
* Read chip firmware version for u16
*/
int mip_get_fw_version_u16(struct i2c_client *client, u16 *ver_buf_u16)
{
	u8 rbuf[2];

	if(mip_get_fw_version(client, rbuf)){
		goto ERROR;
	}

	ver_buf_u16[0] = (rbuf[0] << 8) | rbuf[1];

	return 0;

ERROR:
	memset(ver_buf_u16, 0xFFFF, sizeof(ver_buf_u16));

	TOUCH_ERR_MSG("%s [ERROR]\n", __func__);
	return 1;
}

static int mit_get_ic_info(struct mit_data *ts, struct touch_fw_info *fw_info)
{
	struct i2c_client *client = ts->client;
	int i = 0;
	int otp_check_max = 20;
	u8 wbuf[8];
	u8 rbuf[64] = {0};

	TOUCH_TRACE_FUNC();

	//Product name
	wbuf[0] = MIP_R0_INFO;
	wbuf[1] = MIP_R1_INFO_PRODUCT_NAME;
	if(mit_i2c_read(client, wbuf, 2, rbuf, 16)) {
		TOUCH_ERR_MSG("%s [ERROR] read product name\n", __func__);
		return -EIO;
	}
	memcpy((u8 *) &ts->module.product_code, rbuf, 16);

	//Ic name
	wbuf[0] = MIP_R0_INFO;
	wbuf[1] = MIP_R1_INFO_IC_NAME;
	if(mit_i2c_read(client, wbuf, 2, rbuf, 4)) {
		TOUCH_ERR_MSG("%s [ERROR] read ic name\n", __func__);
		return -EIO;
	}
	memcpy((u8 *) &ts->module.ic_name, rbuf, 4);

	//Firmware version
	if(mip_get_fw_version(client, rbuf)<0) {
		TOUCH_ERR_MSG("%s [ERROR] get fw version\n", __func__);
		return -EIO;
	}
	memcpy((u8 *) &ts->module.version, rbuf, 2);

	//Resolution
	wbuf[0] = MIP_R0_INFO;
	wbuf[1] = MIP_R1_INFO_RESOLUTION_X;
	if(mit_i2c_read(client, wbuf, 2, rbuf, 7)) {
		TOUCH_ERR_MSG("%s [ERROR] get resulution\n", __func__);
		ts->dev.x_resolution = 720;
		ts->dev.x_resolution = 1280;
		return -EIO;
	} else {
		ts->dev.x_resolution = (rbuf[0]) | (rbuf[1] << 8);
		ts->dev.y_resolution = (rbuf[2]) | (rbuf[3] << 8);
	}

	//Node info
	ts->dev.col_num = rbuf[4];
	ts->dev.row_num = rbuf[5];
	ts->dev.key_num = rbuf[6];

	wbuf[0] = MIP_R0_EVENT;
	wbuf[1] = MIP_R1_EVENT_SUPPORTED_FUNC;
	if(mit_i2c_read(client, wbuf, 2, rbuf, 7)) {
		TOUCH_ERR_MSG("%s [ERROR] read node info\n", __func__);
		return -EIO;
	}
	event_format = (rbuf[4]) | (rbuf[5] << 8);
	event_size = rbuf[6];

	for (i = 0; i < otp_check_max; i++) { // need to time check for OTP status
		if (mit_get_otp(ts) < 0) {
			TOUCH_INFO_MSG("failed to get the otp-enable\n");
			return -EIO;
		}
		if (ts->module.otp == OTP_APPLIED)
			break;
		msleep(5);
	}

	if (ts->pdata->panel_on) {
		TOUCH_INFO_MSG("====== LCD  ON  ======\n");
	} else {
		TOUCH_INFO_MSG("====== LCD  OFF ======\n");
	}
	TOUCH_INFO_MSG("======================\n");
	TOUCH_INFO_MSG("F/W Version : %X.%02X \n", ts->module.version[0], ts->module.version[1]);
	TOUCH_INFO_MSG("F/W Product : %s \n", ts->module.product_code);
	TOUCH_INFO_MSG("F/W Row(node_y) : %d, Col(node_x) : %d \n", ts->dev.row_num, ts->dev.col_num);
	TOUCH_INFO_MSG("IC Name : %c%c%c%c \n", ts->module.ic_name[0], ts->module.ic_name[1], ts->module.ic_name[2], ts->module.ic_name[3]);
	TOUCH_INFO_MSG("max_x[%d] max_y[%d]\n", ts->dev.x_resolution, ts->dev.y_resolution);
	if (ts->module.otp == OTP_NOT_SUPPORTED) {
		TOUCH_INFO_MSG("OTP : F/W Not support \n");
	} else {
		TOUCH_INFO_MSG("OTP : %s \n", (ts->module.otp == OTP_APPLIED) ? "Applied" : "None");
	}
	TOUCH_INFO_MSG("======================\n");

	return 0;
}

static void write_file(char *filename, char *data, int time)
{
	int fd = 0;
	char time_string[64] = {0};
	struct timespec my_time;
	struct tm my_date;
	mm_segment_t old_fs = get_fs();

	set_fs(KERNEL_DS);
	fd = sys_open(filename, O_WRONLY|O_CREAT|O_APPEND, 0666);
	if (fd >= 0) {
		if (time > 0) {
			my_time = __current_kernel_time();
			time_to_tm(my_time.tv_sec, sys_tz.tz_minuteswest * 60 * (-1), &my_date);
			snprintf(time_string, 64, "\n%02d-%02d %02d:%02d:%02d.%03lu \n\n\n",
				my_date.tm_mon + 1,my_date.tm_mday,
				my_date.tm_hour, my_date.tm_min, my_date.tm_sec,
				(unsigned long) my_time.tv_nsec / 1000000);
			sys_write(fd, time_string, strlen(time_string));
		}
		sys_write(fd, data, strlen(data));
		sys_close(fd);
	}
	set_fs(old_fs);
}

#if 0
static int read_file(char *filename, char *data, size_t length)
{
	int fd = 0;
	int len = 0;
	mm_segment_t old_fs = get_fs();

	set_fs(KERNEL_DS);
	fd = sys_open(filename, O_RDONLY, 0666);
	if (fd >= 0) {
		len = sys_read(fd, data, length );
		sys_close(fd);
		if (len <= 0) {
			TOUCH_INFO_MSG("%s sys_read Err len = %d\n", __func__, len);
			goto SYSFS_ERROR;
		}
	} else {
		goto SYSFS_ERROR;
	}
	set_fs(old_fs);
	return 0;

SYSFS_ERROR :
	TOUCH_INFO_MSG("read file fail \n");
	set_fs(old_fs);
	return -1;
}
#endif

int mit_atoi(char *str)
{
	int i = 0;
	int minus = 0;
	int result = 0;
	int check = 0;

	if ( str[0] == '-' ) {
		i++;
		minus = 1;
	}

	while ((str[i] >= '0') && (str[i] <= '9')) {
		result = (10 * result) + (str[i] - '0');
		i++;
		check = 1;
	}

	if (!check) {
		TOUCH_INFO_MSG("atoi fail\n");
		return 0xFFF;
	}
	return (minus) ? ((-1) * result) : result;
}

static void mit_battery_thermal(struct mit_data *ts, char caller)
{
	return;
#if 0
	char data[32] = {0};
	short is_present = 2;
	short ret = 0xFFF;
	uint8_t write_buf[8] = {0};
	struct i2c_msg msg[] = {
		{
			.addr = ts->client->addr,
			.flags = 0,
			.buf = write_buf,
		},{
			.addr = ts->client->addr,
			.flags = 1,
		},
	};

	if (!gpio_get_value(ts->pdata->reset_pin)) {
		return;
	}

	if (read_file(BATT_THERMAL, data, sizeof(data) - 1) < 0)
		goto DATA_ERROR;

	ret = (short)mit_atoi(data + 7);

	if (ret >= 2000)
		goto DATA_ERROR;

	if (ret != 0xFFF) {
		if (ret == -300) {
			memset(data, 0, sizeof(data));
			if (read_file(BATT_PRESENT, data, sizeof(data) - 1) < 0)
				goto DATA_ERROR;
			is_present = (short)mit_atoi(data);

			if (is_present == 0) {
				ret = 300;
				TOUCH_INFO_MSG("No Battery\n");
			} else
				TOUCH_INFO_MSG("present battery  = %d\n", is_present);
		}

		TOUCH_INFO_MSG("Thermal value %d %s\n", ret, (caller ? "[IC]" : ""));
		write_buf[0] = MIT_REGH_CMD;
		write_buf[1] = MIT_REGL_UCMD;
		write_buf[2] = MIT_UNIV_SEND_THERMAL_INFO;
		write_buf[3] = (char)((ret & 0xFF00) >> 8);
		write_buf[4] = (char)(ret & 0xFF);
		msg[0].len = 5;

		if (i2c_transfer(ts->client->adapter, &msg[0], 1) != 1) {
			TOUCH_INFO_MSG("%s : i2c transfer failed\n", __func__);
		}
		return;
	}

DATA_ERROR :
	TOUCH_INFO_MSG("%s failed\n", __func__);
	return;
#endif
}

#if defined(TOUCH_USE_DSV)
void mit_dsv_control(struct i2c_client *client)
{
	struct mit_data *ts = (struct mit_data *) get_touch_handle_(client);
	int set_value = ts->pdata->use_dsv;

	if (set_value)
		msleep(200);
	//mdss_dsv_ctl(set_value);

	TOUCH_INFO_MSG("dsv_ctrl onoff: %d\n", set_value);
}
#endif

static int set_tci_info(struct i2c_client *client)
{
	struct mit_data *ts = get_touch_handle_(client);
	TOUCH_INFO_MSG("Setting tci info data\n");

	//common
	ts->pdata->tci_info->idle_report_rate = 20;
	ts->pdata->tci_info->active_report_rate = 40;
	/* change ContactOnThd value of Firmware for CY/K */
	ts->pdata->tci_info->sensitivity = 30;

	//double tap only
	ts->pdata->tci_info->touch_slope = 10;
	ts->pdata->tci_info->min_distance = 0;
	ts->pdata->tci_info->max_distance = 10;
	ts->pdata->tci_info->min_intertap = 0;
	ts->pdata->tci_info->max_intertap = 700;
	ts->pdata->tci_info->tap_count = 2;
	//multitap only
	ts->pdata->tci_info->touch_slope_2 = 10;
	ts->pdata->tci_info->min_distance_2 = 0;
	ts->pdata->tci_info->max_distance_2 = 65535;
	ts->pdata->tci_info->min_intertap_2 = 0;
	ts->pdata->tci_info->max_intertap_2 = 700;
	ts->pdata->tci_info->interrupt_delay_2 = 250;

	return 0;
}

#ifndef MIP_USE_DEV
#define MIP_USE_DEV 1
#endif
// for debug touchscreen
#if MIP_USE_DEV

/**
* Dev node output to user
*/
static ssize_t mip_dev_fs_read(struct file *fp, char *rbuf, size_t cnt, loff_t *fpos)
{
	struct mit_data *ts = fp->private_data;
	int ret = 0;

	//dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	ret = copy_to_user(rbuf, ts->dev_fs_buf, cnt);

	//dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);

	return ret;
}

/**
* Dev node input from user
*/
static ssize_t mip_dev_fs_write(struct file *fp, const char *wbuf, size_t cnt, loff_t *fpos)
{
	struct mit_data *ts = fp->private_data;
	u8 *buf;
	int ret = 0;
	int cmd = 0;

	//dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	buf = kzalloc(cnt + 1, GFP_KERNEL);

	if ((buf == NULL) || copy_from_user(buf, wbuf, cnt)) {
		dev_err(&ts->client->dev, "%s [ERROR] copy_from_user\n", __func__);
		ret = -EIO;
		goto EXIT;
	}

	cmd = buf[cnt - 1];

	if(cmd == 1){
		//dev_dbg(&info->client->dev, "%s - cmd[%d] w_len[%d] r_len[%d]\n", __func__, cmd, (cnt - 2), buf[cnt - 2]);

		if(mit_i2c_read(ts->client, buf, (cnt - 2), ts->dev_fs_buf, buf[cnt - 2])){
			dev_err(&ts->client->dev, "%s [ERROR] mip_i2c_read\n", __func__);
		}
		//print_hex_dump(KERN_ERR, MIP_DEVICE_NAME" : input ", DUMP_PREFIX_OFFSET, 16, 1, wbuf, cnt, false);
		//print_hex_dump(KERN_ERR, MIP_DEVICE_NAME" : output ", DUMP_PREFIX_OFFSET, 16, 1, info->dev_fs_buf, buf[cnt - 2], false);
	}
	else if(cmd == 2){
		//dev_dbg(&info->client->dev, "%s - cmd[%d] w_len[%d]\n", __func__, cmd, (cnt - 1));
	if(mip_i2c_write(ts->client, buf, (cnt - 1)) ){
			dev_err(&ts->client->dev, "%s [ERROR] mip_i2c_write\n", __func__);
		}
		//print_hex_dump(KERN_ERR, MIP_DEVICE_NAME" : input ", DUMP_PREFIX_OFFSET, 16, 1, wbuf, cnt, false);
	}
	else{
		goto EXIT;
	}

EXIT:
	kfree(buf);

	//dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);

	return ret;
}

/**
* Open dev node
*/
static int mip_dev_fs_open(struct inode *node, struct file *fp)
{
	struct mit_data *ts = container_of(node->i_cdev, struct mit_data, cdev);

	//dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	fp->private_data = ts;

	ts->dev_fs_buf = kzalloc(1024 * 4, GFP_KERNEL);

	//dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);

	return 0;
}

/**
* Close dev node
*/
static int mip_dev_fs_release(struct inode *node, struct file *fp)
{
	struct mit_data *ts = fp->private_data;

	//dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	kfree(ts->dev_fs_buf);

	//dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);

	return 0;
}

/**
* Dev node info
*/
static struct file_operations mip_dev_fops = {
	.owner	= THIS_MODULE,
	.open	= mip_dev_fs_open,
	.release	= mip_dev_fs_release,
	.read	= mip_dev_fs_read,
	.write	= mip_dev_fs_write,
};

/**
* Create dev node
*/
int mip_dev_create(struct mit_data *ts)
{
	struct i2c_client *client = ts->client;
	int ret = 0;

	dev_dbg(&ts->client->dev, "%s [START]\n", __func__);

	if (alloc_chrdev_region(&ts->mip_dev, 0, 1, "lge_touch")) {
		dev_err(&client->dev, "%s [ERROR] alloc_chrdev_region\n", __func__);
		ret = -ENOMEM;
		goto ERROR;
	}

	cdev_init(&ts->cdev, &mip_dev_fops);
	ts->cdev.owner = THIS_MODULE;

	if (cdev_add(&ts->cdev, ts->mip_dev, 1)) {
		dev_err(&client->dev, "%s [ERROR] cdev_add\n", __func__);
		ret = -EIO;
		goto ERROR;
	}

	dev_dbg(&ts->client->dev, "%s [DONE]\n", __func__);
	return 0;

ERROR:
	dev_err(&ts->client->dev, "%s [ERROR]\n", __func__);
	return 0;
}

#endif

static int mit_probe(struct i2c_client *client, struct touch_platform_data *pdata)
{
	struct mit_data *ts = NULL;
	int ret = 0;
	int i = 0;
	char gpio_request_name[16] = {0};

	TOUCH_TRACE_FUNC();

	ts = devm_kzalloc(&client->dev, sizeof(struct mit_data), GFP_KERNEL);

	if (ts == NULL) {
		TOUCH_ERR_MSG("Can not allocate memory\n");
		return -ENOMEM;
	}

	ts->client = client;
	ts->pdata = pdata;
	ts->log.data = ts->buf;

	ts->pdata->tap_count = 4;		// default tap count set
	ts->pdata->lpwg_prox = 1;		// default proxi sensor information
	ts->pdata->lpwg_debug_enable = 0;	// lpwg debug enable currently
	ts->pdata->lpwg_fail_reason = 1;
	ts->pdata->active_area_x1 = ts->pdata->active_area_gap;
	ts->pdata->active_area_x2 = ts->pdata->caps->lcd_x - ts->pdata->active_area_gap;
	ts->pdata->active_area_y1 = ts->pdata->active_area_gap;
	ts->pdata->active_area_y2 = ts->pdata->caps->lcd_y - ts->pdata->active_area_gap;

  ts->enable_sensing = 1;
  
  mutex_init(&ts->proximity_lock);

	set_touch_handle_(client, ts);
	set_tci_info(client);

	for (i = 0; i < TOUCH_PWR_NUM; ++i) {
		if (ts_pwr[i].type == 1 && gpio_is_valid(ts_pwr[i].value)) {
			snprintf(gpio_request_name, 16, "touch_vdd_%d", i);
			if (!strncmp(ts_pwr[i].name, "low", strlen("low")))
				ret = gpio_request_one(ts_pwr[i].value, GPIOF_OUT_INIT_LOW, gpio_request_name);
			else
				ret = gpio_request_one(ts_pwr[i].value, GPIOF_OUT_INIT_HIGH, gpio_request_name);

			if (ret) {
				ts_pwr[i].value = -1;
				goto err_regulator_get;
			}
		} else if (ts_pwr[i].type == 2) {
			ts->vdd_regulator[i] = regulator_get(&client->dev, ts_pwr[i].name);
			if (IS_ERR(ts->vdd_regulator[i])) {
				ret = PTR_ERR(ts->vdd_regulator[i]);
				TOUCH_ERR_MSG("Can NOT get regulator : %s, ret = %d\n", ts_pwr[i].name, ret);
				goto err_regulator_get;
			}

			if (regulator_count_voltages(ts->vdd_regulator[i]) > 0) {
				ret = regulator_set_voltage(ts->vdd_regulator[i], ts_pwr[i].value, ts_pwr[i].value);
				if (ret) {
					TOUCH_ERR_MSG("Error(ret=%d) set regulator(%s) voltage %d\n", ret, ts_pwr[i].name, ts_pwr[i].value);
					goto err_regulator_get;
				}
			}
		}
	}

	for (i = 0; i < MAX_ROW; i++) {
		ts->mit_data[i] = kzalloc(sizeof(uint16_t) * MAX_COL, GFP_KERNEL);
		if (ts->mit_data[i] == NULL) {
			TOUCH_ERR_MSG("mit_data kzalloc error\n");
			return -ENOMEM;
		}
		ts->intensity_data[i] = kzalloc(sizeof(uint16_t) * MAX_COL, GFP_KERNEL);
		if (ts->intensity_data[i] == NULL) {
			TOUCH_ERR_MSG("intensity_data kzalloc error\n");
			return -ENOMEM;
		}
	}
	ts_data = ts;


#if MIP_USE_DEV
	//Create dev node (optional)
	if(mip_dev_create(ts)){
		dev_err(&client->dev, "%s [ERROR] mip_dev_create\n", __func__);
		ret = -EAGAIN;
		goto ERROR;
	}

	//Create dev
	ts->class = class_create(THIS_MODULE, "lge_touch");
	device_create(ts->class, NULL, ts->mip_dev, NULL, "lge_touch");
#endif

	return 0;

err_regulator_get:
	do {
		if (ts_pwr[i].type == 1) {
			if (gpio_is_valid(ts_pwr[i].value))
				gpio_free(ts_pwr[i].value);
		} else if (ts_pwr[i].type == 2) {
			if (ts->vdd_regulator != NULL && !IS_ERR(ts->vdd_regulator[i]))
				regulator_put(ts->vdd_regulator[i]);
		}
	} while(--i >= 0);
	return ret;
ERROR:
	return ret;

}

static void mit_remove(struct i2c_client* client)
{
	struct mit_data *ts = get_touch_handle_(client);
	int i = TOUCH_PWR_NUM-1;

	TOUCH_TRACE_FUNC();

	do {
		if (ts_pwr[i].type == 1) {
			if (!strncmp(ts_pwr[i].name, "low", strlen("low")))
				gpio_direction_output(ts_pwr[i].value, 1);
			else
				gpio_direction_output(ts_pwr[i].value, 0);

			if (gpio_is_valid(ts_pwr[i].value))
				gpio_free(ts_pwr[i].value);
		} else if (ts_pwr[i].type == 2) {
			if (ts->vdd_regulator[i] != NULL && !IS_ERR(ts->vdd_regulator[i])) {
				regulator_put(ts->vdd_regulator[i]);
			}
		}
	} while(--i >= 0);

	for (i = 0; i < MAX_ROW; i++) {
		if (ts->mit_data[i] != NULL) {
			kfree(ts->mit_data[i]);
		}
		if (ts->intensity_data[i] != NULL) {
			kfree(ts->intensity_data[i]);
		}
	}

  mutex_destroy(&ts->proximity_lock);

#if MIP_USE_DEV
	device_destroy(ts->class, ts->mip_dev);
	class_destroy(ts->class);
#endif
}

static int mit_init(struct i2c_client* client, struct touch_fw_info* fw_info)
{
	struct mit_data *ts = get_touch_handle_(client);

	TOUCH_TRACE_FUNC();

	ts->probed = true;
	return 0;
}

static int mit_touch_event(struct i2c_client *client, struct touch_data *data, u8 *buf, int sz)
{
	u8 touch_count = 0;
	u8 state = 0;
	int i = 0;
	int id = 0, x = 0, y = 0;
	int pressure = 0;
	int touch_major = 0, touch_minor = 0;
	int palm = 0;
	int size = 0;

	TOUCH_TRACE_FUNC();

	for (i = 0; i < sz; i += FINGER_EVENT_SZ) {
		u8 *tmp = &buf[i];

		//Report input data
		if ((tmp[0] & MIP_EVENT_INPUT_SCREEN) == 0) {
			//Touchkey Event
			TOUCH_INFO_MSG("use sofrware key\n");
		} else {
			//Touchscreen Event Protocol Type
			if(event_format == 0) {
				id = (tmp[0] & 0xf) - 1;
				x = tmp[2] | ((tmp[1] & 0xf) << 8);
				y = tmp[3] | (((tmp[1] >> 4) & 0xf) << 8);
				pressure = tmp[4];
				touch_major = tmp[5];
				palm = (tmp[0] & MIP_EVENT_INPUT_PALM) >> 4;
			} else if(event_format == 1) {
				id = (tmp[0] & 0xf) - 1;
				x = tmp[2] | ((tmp[1] & 0xf) << 8);
				y = tmp[3] | (((tmp[1] >> 4) & 0xf) << 8);
				pressure = tmp[4];
				size = tmp[5];
				touch_major = tmp[6];
				touch_minor = tmp[7];
				palm = (tmp[0] & MIP_EVENT_INPUT_PALM) >> 4;
			} else if(event_format == 2) {
				id = (tmp[0] & 0xf) - 1;
				x = tmp[2] | ((tmp[1] & 0xf) << 8);
				y = tmp[3] | (((tmp[1] >> 4) & 0xf) << 8);
				pressure = tmp[4];
				touch_major = tmp[5];
				touch_minor = tmp[6];
				palm = (tmp[0] & MIP_EVENT_INPUT_PALM) >> 4;
			} else {
				TOUCH_INFO_MSG("%s [ERROR] Unknown event format [%d]\n", __func__, event_format);
				return -EIO;
			}

			if((id > MAX_FINGER - 1) || (id < 0))
			{
				TOUCH_INFO_MSG("%s [ERROR] Abnormal id [%d]\n", __func__, id);
				return -EIO;
			}

			if((tmp[0] & MIP_EVENT_INPUT_PRESS) == 0) {
				if (palm) {
					TOUCH_INFO_MSG("Palm released : %d \n", pressure);
					return -EIO;
				}
				data->curr_data[id].status = FINGER_RELEASED;
			} else {
				if (palm) {
					TOUCH_INFO_MSG("Palm detected : %d \n", pressure);
					return -EIO;
				}
				data->curr_data[id].id = id;
				data->curr_data[id].x_position = x;
				data->curr_data[id].y_position = y;
				data->curr_data[id].width_major = touch_major;
				data->curr_data[id].width_minor = 0;
				data->curr_data[id].width_orientation = 0;

				// check pressure on Finger Touch
				if(pressure < 1 ) pressure = 1;
				else if(pressure > MAX_PRESSURE - 1) pressure = MAX_PRESSURE - 1;

				data->curr_data[id].pressure = pressure;

				data->curr_data[id].status = FINGER_PRESSED;
				touch_count++;
			}

			if (unlikely(touch_debug_mask_ & DEBUG_GET_DATA)) {
				TOUCH_INFO_MSG("<%d> pos(%4d,%4d) w_m[%2d] w_n[%2d] w_o[%2d] p[%2d] s[%d]\n",
					id, data->curr_data[id].x_position, data->curr_data[id].y_position,
					data->curr_data[id].width_major, data->curr_data[id].width_minor,
					data->curr_data[id].width_orientation, data->curr_data[id].pressure, state);
			}

			if (data->curr_data[id].status == FINGER_PRESSED
				&& data->prev_data[id].status <= FINGER_RELEASED
				&& !data->curr_data[id].point_log_state) {
					data->curr_data[id].touch_conut = 0;
					++data->touch_count_num;
					if (likely(touch_debug_mask_ & DEBUG_ABS_POINT)) {
						if (lockscreen_stat == 1) {
							TOUCH_INFO_MSG("%d finger pressed : <%d> x[XXX] y[XXX] z[XXX]\n",
							data->touch_count_num, id);
						} else {
							TOUCH_INFO_MSG("%d finger pressed : <%d> x[%3d] y[%3d] z[%3d]\n",
							data->touch_count_num, id,
							data->curr_data[id].x_position,
							data->curr_data[id].y_position,
							data->curr_data[id].pressure);
						}
					}
					data->curr_data[id].point_log_state = 1;
			}

			else if (data->curr_data[id].status == FINGER_RELEASED
				&& data->prev_data[id].point_log_state) {
					data->touch_count_num--;

					if (likely(touch_debug_mask_ & DEBUG_ABS_POINT)) {
						if (lockscreen_stat == 1) {
							TOUCH_INFO_MSG("touch_release[%s] : <%d> x[XXX] y[XXX] M:XX\n",
							data->palm?"Palm":" ", id);
						} else {
							TOUCH_INFO_MSG("touch_release[%s] : <%d> x[%3d] y[%3d] M:%d\n",
							data->palm?"Palm":" ", id,
							data->prev_data[id].x_position,
							data->prev_data[id].y_position,
							data->curr_data[id].touch_conut);
						}
					}
					data->curr_data[id].point_log_state = 0;
			} else {
				data->curr_data[id].touch_conut++;
			}
		}
	}
	data->total_num = touch_count;
	return 0;
}
static int mit_fail_reason_event(struct i2c_client *client, struct touch_data *data, u8 *buf, int sz)
{

	u8 wbuf[4];
	u8 gesture_code = buf[1];

	TOUCH_INFO_MSG("%s - gesture_code[%d]\n", __func__, gesture_code);
	switch(gesture_code){
		case MIP_LPWG_EVENT_TYPE_FAIL:
			switch (buf[2]) {
				case FAIL_OUT_OF_AREA:	TOUCH_INFO_MSG("LPWG FAIL REASON = Out of Area\n");		break;
				case FAIL_PALM:			TOUCH_INFO_MSG("LPWG FAIL REASON = Palm\n");			break;
				case FAIL_DELAY_TIME:	TOUCH_INFO_MSG("LPWG FAIL REASON = Delay Time\n");		break;
				case FAIL_TAP_TIME:		TOUCH_INFO_MSG("LPWG FAIL REASON = Tap Time\n");		break;
				case FAIL_TAP_DISTACE:	TOUCH_INFO_MSG("LPWG FAIL REASON = Tap Distance\n");	break;
				case FAIL_TOUCH_SLOPE:	TOUCH_INFO_MSG("LPWG FAIL REASON = Touch Slope\n");		break;
				case FAIL_MULTI_TOUCH:	TOUCH_INFO_MSG("LPWG FAIL REASON = Multi Touch\n");		break;
				case FAIL_LONG_PRESS:	TOUCH_INFO_MSG("LPWG FAIL REASON = Long Press\n");		break;
				default	:				TOUCH_INFO_MSG("LPWG FAIL REASON = Unknown Reason\n");	break;
			}
			break;
		default:
			//Re-enter nap mode
			wbuf[0] = MIP_R0_CTRL;
			wbuf[1] = MIP_R1_CTRL_POWER_STATE;
			wbuf[2] = MIP_CTRL_POWER_LOW;
			if(mip_i2c_write(client, wbuf, 3)){
				TOUCH_INFO_MSG("%s [ERROR] mip_i2c_write\n", __func__);
				return -EIO;
			}
			break;
	}

	return 0;
}

static int mit_lpwg_event(struct i2c_client *client, struct touch_data *data, u8 *buf, int sz)
{
	struct mit_data *ts = get_touch_handle_(client);
	u8 wbuf[4];
	u8 gesture_code = buf[1];
	int i = 0, x = 0, y = 0;

	memset(ts->pdata->lpwg_x, 0, sizeof(int)*10);
	memset(ts->pdata->lpwg_y, 0, sizeof(int)*10);
	memset(&ts->pdata->lpwg_size, 0, sizeof(int));

	TOUCH_INFO_MSG("%s - gesture_code[%d]\n", __func__, gesture_code);

	switch(gesture_code){
		case MIP_EVENT_GESTURE_DOUBLE_TAP:
			ts->pdata->send_lpwg = LPWG_DOUBLE_TAP;
			for(i = 2; i < sz; i += LPWG_EVENT_SZ){
				u8 *tmp = &buf[i];
				x = tmp[1] | ((tmp[0] & 0xf) << 8);
				y = tmp[2] | (((tmp[0] >> 4) & 0xf) << 8);
				TOUCH_INFO_MSG("LPWG[%d] - %d TAP x[XXX] y[XXX] \n", gesture_code, (i+1)/LPWG_EVENT_SZ);
				ts->pdata->lpwg_x[((i + 1) / LPWG_EVENT_SZ) - 1] = x;
				ts->pdata->lpwg_y[((i + 1) / LPWG_EVENT_SZ) - 1] = y;
				ts->pdata->lpwg_size++;
			}
			break;
		case MIP_EVENT_GESTURE_MULTI_TAP:
			ts->pdata->send_lpwg = LPWG_MULTI_TAP;
			for(i = 2; i < sz; i += LPWG_EVENT_SZ){
				u8 *tmp = &buf[i];
				x = tmp[1] | ((tmp[0] & 0xf) << 8);
				y = tmp[2] | (((tmp[0] >> 4) & 0xf) << 8);
				TOUCH_INFO_MSG("LPWG[%d] - %d TAP x[XXX] y[XXX] \n", gesture_code, (i+1)/LPWG_EVENT_SZ);
				ts->pdata->lpwg_x[((i + 1) / LPWG_EVENT_SZ) - 1] = x;
				ts->pdata->lpwg_y[((i + 1) / LPWG_EVENT_SZ) - 1] = y;
				ts->pdata->lpwg_size++;
			}
			break;
		default:
			//Re-enter nap mode
			wbuf[0] = MIP_R0_CTRL;
			wbuf[1] = MIP_R1_CTRL_POWER_STATE;
			wbuf[2] = MIP_CTRL_POWER_LOW;
			if(mip_i2c_write(client, wbuf, 3)){
				TOUCH_INFO_MSG("%s [ERROR] mip_i2c_write\n", __func__);
				return -EIO;
			}
			break;
	}

	return 0;
}
#if 0
static int mit_log_event(struct i2c_client *client, struct mit_data *ts)
{
	struct mit_log_pkt *pkt = (struct mit_log_pkt *) ts->buf;
	char *tmp = NULL;
	int len = 0;
	u8 row_num = 0;

	TOUCH_TRACE_FUNC();

	if ((pkt->log_info & 0x7) == 0x1) {
		pkt->element_sz = 0;
		pkt->row_sz = 0;
		return -EIO;
	}

	switch (pkt->log_info >> 4) {
		case LOG_TYPE_U08:
		case LOG_TYPE_S08:
			len = pkt->element_sz;
			break;
		case LOG_TYPE_U16:
		case LOG_TYPE_S16:
			len = pkt->element_sz * 2;
			break;
		case LOG_TYPE_U32:
		case LOG_TYPE_S32:
			len = pkt->element_sz * 4;
			break;
		default:
			dev_err(&client->dev, "invalied log type\n");
			return -EIO;
	}

	tmp = ts->buf + sizeof(struct mit_log_pkt);
	row_num = pkt->row_sz ? pkt->row_sz : 1;

	while (row_num--) {
		mit_i2c_read(client, MIT_REGL_UCMD_RESULT, tmp, len);
		tmp += len;
	}

	return 0;
}
#endif
static int mit_get_packet(struct i2c_client *client)
{
	struct mit_data *ts = get_touch_handle_(client);
	u8 sz = 0;
	u8 wbuf[8];
	u8 rbuf[256] = {0};
	int size = 0;

	TOUCH_TRACE_FUNC();

	//send Dummy packet(lpwg mode)
	if(ts->pdata->lpwg_mode_old){
		if(mit_i2c_dummy(client, wbuf, 2)){
			TOUCH_INFO_MSG("%s [ERROR] Dummy packet\n", __func__);
			return -EIO;
		}
	}
	//Read packet info
	wbuf[0] = MIP_R0_EVENT;
	wbuf[1] = MIP_R1_EVENT_PACKET_INFO;
	if(mit_i2c_read(client, wbuf, 2, rbuf, 1)){
		TOUCH_INFO_MSG("%s [ERROR] Read packet info\n", __func__);
		return -EIO;
	}
	size = (rbuf[0] & 0x7F);
	category = ((rbuf[0] >> 7) & 0x1);
	sz = ((rbuf[0] >> 7) & 0x1);
	if (likely(touch_debug_mask_ & DEBUG_GET_PACKET))
		TOUCH_INFO_MSG("%s - packet info : size[%d] category[%d], type[%d]\n", __func__, size, category, sz);

	//Check size
	if(size <= 0){
		TOUCH_INFO_MSG("%s [ERROR] Packet size [%d]\n", __func__, size);
		return -EIO;
	}

	//Read packet data
	wbuf[0] = MIP_R0_EVENT;
	wbuf[1] = MIP_R1_EVENT_PACKET_DATA;
//	memset(ts->buf, 0, size * ts->pdata->caps->max_id);
	if(mit_i2c_read(client, wbuf, 2, ts->buf, size)) {
		TOUCH_INFO_MSG("%s [ERROR] Read packet data\n", __func__);
		return -EIO;
	}

	return size;
}

static int mit_get_data(struct i2c_client *client, struct touch_data *data)
{
	struct mit_data *ts = get_touch_handle_(client);
	int sz = 0;
	u8 alert_type = 0;
	TOUCH_TRACE_FUNC();
	sz = mit_get_packet(client);
	if (sz == 0)
		return 0;
	if ((sz) < 0)
		return -EIO;

	if(category == 0) {
		mit_touch_event(client, data, ts->buf, sz);
	} else {
		alert_type = ts->buf[0];
		TOUCH_INFO_MSG("%s - alert type [%d]\n", __func__, alert_type);
		if(alert_type == MIP_ALERT_WAKEUP){
			if (mit_lpwg_event(client, data, ts->buf, sz) < 0)
				goto err_event_type;
		} else if(alert_type == MIP_ALERT_F1){
			if (mit_fail_reason_event(client, data, ts->buf, sz) < 0)
				goto err_event_type;
		} else if(alert_type == MIP_ALERT_ESD){
			TOUCH_INFO_MSG("%s - MIP_ALERT_ESD\n", __func__);
#if defined(CONFIG_LGE_LCD_ESD)
			lge_mdss_report_panel_dead();
#endif
		} else {
			TOUCH_INFO_MSG("%s [ERROR] Unknown alert type [%d]\n", __func__, alert_type);
		}
	}

	return 0;

err_event_type:
	TOUCH_ERR_MSG("Unkown, event type(alert_type) 0x%x\n", alert_type);
	return -EIO;
}

static int mit_sleep(struct i2c_client *client)
{
	return 0;
}

static int mit_wake(struct i2c_client *client)
{
	return 0;
}

void reset_pin_ctrl(struct mit_data* ts, int on_off, int delay)
{
	gpio_direction_output(ts_pdata->reset_pin, on_off);
	TOUCH_INFO_MSG("%s\n", on_off ? "power: reset_pin high" : "power: reset_pin low");

        if( delay <= 0 || delay > 1000 )
        {
            TOUCH_INFO_MSG("%s exeeds limit %d\n", __func__, delay);
            return;
        }

	if(delay<=20)
		mdelay(delay);
        else
		msleep(delay);

}

static int mit_power(struct i2c_client* client, int power_ctrl)
{
	struct mit_data* ts = get_touch_handle_(client);
	int i = 0;
	int ret = 0;

	TOUCH_POWER_MSG("%s = %d\n", __func__, power_ctrl);

	if (ts->pdata->curr_pwr_state == power_ctrl) {
		TOUCH_INFO_MSG("Ignore Power Control : curr_pwr_state = %d\n", power_ctrl);
		return 0;
	}

	switch (power_ctrl) {
	case POWER_OFF:
		i = TOUCH_PWR_NUM-1;
		do {
			if (ts_pwr[i].type == 1) {
				if (!strncmp(ts_pwr[i].name, "low", strlen("low"))) {
					gpio_direction_output(ts_pwr[i].value, 1);
					TOUCH_POWER_MSG("power[%d]: gpio[%d] set 1\n", i, ts_pwr[i].value);
				} else {
					gpio_direction_output(ts_pwr[i].value, 0);
					TOUCH_POWER_MSG("power[%d]: gpio[%d] set 0\n", i, ts_pwr[i].value);
				}
			} else if (ts_pwr[i].type == 2) {
				if (ts->vdd_regulator[i] != NULL && !IS_ERR(ts->vdd_regulator[i])) {
					regulator_disable(ts->vdd_regulator[i]);
					TOUCH_POWER_MSG("power[%d]: regulator disabled\n", i);
				}
			}
                        mdelay(2);
		} while(--i >= 0);
		TOUCH_INFO_MSG("Power Off \n");
		break;
	case POWER_ON:
		i = 0;
		do {
			if (ts_pwr[i].type == 1) {
				if (!strncmp(ts_pwr[i].name, "low", strlen("low"))) {
					gpio_direction_output(ts_pwr[i].value, 0);
					TOUCH_POWER_MSG("power[%d]: gpio[%d] set 0\n", i, ts_pwr[i].value);
				} else {
					gpio_direction_output(ts_pwr[i].value, 1);
					TOUCH_POWER_MSG("power[%d]: gpio[%d] set 1\n", i, ts_pwr[i].value);
				}
			} else if (ts_pwr[i].type == 2) {
				if (ts->vdd_regulator[i] != NULL && !IS_ERR(ts->vdd_regulator[i])) {
					ret = regulator_enable(ts->vdd_regulator[i]);
					if (ret) {
						TOUCH_INFO_MSG("power[%d]: regulator enable failed ret =%d\n", i, ret );
					} else {
						TOUCH_POWER_MSG("power[%d]: regulator enabled\n", i);
					}
				}
			}
			mdelay(2);
		} while(++i < TOUCH_PWR_NUM);

		TOUCH_INFO_MSG("Power On \n");

		if (!ts->thermal_info_send_block) {
			mit_battery_thermal(ts, 0);
		}

		break;

	case POWER_SLEEP:
		if (mit_sleep(client))
			return -EIO;
		break;

	case POWER_WAKE:
		if (mit_wake(client))
			return -EIO;
		break;

	case POWER_RESET:
		TOUCH_INFO_MSG("Power Reset \n");
		mit_power_reset(ts);
		return 0;
		break;

	default:
		return -EIO;
		break;
	}

	ts->pdata->curr_pwr_state = power_ctrl;

	return 0;
}

int mit_power_ctrl(struct i2c_client* client, int power_ctrl)
{
	TOUCH_INFO_MSG("%s : %d \n", __func__, power_ctrl);

	return mit_power(client, power_ctrl);
}
EXPORT_SYMBOL(mit_power_ctrl);

int mit_power_reset(struct mit_data *ts)
{
	TOUCH_INFO_MSG("Power Reset \n");

	reset_pin_ctrl(ts, 0, 2);
	reset_pin_ctrl(ts, 1, ts->pdata->role->booting_delay);

	return 0;
}

static int mit_firmware_img_parse_show(const char *image_bin, char *show_buf, int ret)
{
	struct mit_bin_hdr *fw_hdr = NULL;
	return 0;

	fw_hdr = (struct mit_bin_hdr *) image_bin;

	ret += sprintf(show_buf + ret, "mit_fw_hdr:\n");
	ret += sprintf(show_buf + ret, "\ttag[%c%c%c%c%c%c%c%c]\n",
			fw_hdr->tag[0], fw_hdr->tag[1], fw_hdr->tag[2], fw_hdr->tag[3],
			fw_hdr->tag[4], fw_hdr->tag[5], fw_hdr->tag[6], fw_hdr->tag[7]);
	ret += sprintf(show_buf + ret, "\tcore_version[0x%02x]\n", fw_hdr->core_version);
	ret += sprintf(show_buf + ret, "\tsection_num[%d]\n", fw_hdr->section_num);
	ret += sprintf(show_buf + ret, "\tcontains_full_binary[%d]\n", fw_hdr->contains_full_binary);
	ret += sprintf(show_buf + ret, "\tbinary_offset[%d (0x%04x)]\n", fw_hdr->binary_offset, fw_hdr->binary_offset);
	ret += sprintf(show_buf + ret, "\tbinary_length[%d]\n", fw_hdr->binary_length);

	return ret;
}

static int mit_fw_upgrade(struct i2c_client* client, struct touch_fw_info *info)
{
	struct mit_data *ts = get_touch_handle_(client);
	int ret = 0;

	TOUCH_TRACE_FUNC();

	touch_disable(ts->client->irq);

	if (info->fw)
		ret = mit_isc_fwupdate(ts, info);

	touch_enable(ts->client->irq);

	return ret;
}

static int mit_set_active_area(struct mit_data* ts, u8 mode)
{
	char write_buf[255] = {0};

	if (mode) {
		write_buf[0] = MIP_R0_LPWG;
		write_buf[1] = MIP_R1_LPWG_ACTIVE_AREA;
		write_buf[2] = ts->pdata->active_area_x1 >> 8;
		write_buf[3] = ts->pdata->active_area_x1 & 0xFF;
		write_buf[4] = ts->pdata->active_area_y1 >> 8;
		write_buf[5] = ts->pdata->active_area_y1 & 0xFF;
		write_buf[6] = ts->pdata->active_area_x2 >> 8;
		write_buf[7] = ts->pdata->active_area_x2 & 0xFF;
		write_buf[8] = ts->pdata->active_area_y2 >> 8;
		write_buf[9] = ts->pdata->active_area_y2 & 0xFF;

		if (i2c_master_send(ts->client, write_buf, 10) != 10) {
			TOUCH_INFO_MSG("MIP_R1_LPWG_ACTIVE_AREA write error \n");
			return -EIO;
		} else {
			TOUCH_INFO_MSG("MIP_R1_LPWG_ACTIVE_AREA\n");
		}
	} else {
		TOUCH_INFO_MSG("None Active Area \n");
	}

	return 0;
}

static int tci_control(struct mit_data* ts, int type, u16 value)
{
	char write_buf[255] = {0};
	/* Common Reg */
	switch (type) {
	case IDLE_REPORTRATE_CTRL:
		write_buf[0] = MIP_R0_LPWG;
		write_buf[1] = MIP_R1_LPWG_IDLE_REPORTRATE;
		write_buf[2] = value;
		if (i2c_master_send(ts->client, write_buf, 3) != 3) {
			TOUCH_INFO_MSG("MIP_R1_LPWG_IDLE_REPORTRATE write error \n");
			return -EIO;
		} else {
			TOUCH_INFO_MSG("MIP_R1_LPWG_IDLE_REPORTRATE\n");
		}
		break;
	case ACTIVE_REPORTRATE_CTRL:
		write_buf[0] = MIP_R0_LPWG;
		write_buf[1] = MIP_R1_LPWG_ACTIVE_REPORTRATE;
		write_buf[2] = value;
		if (i2c_master_send(ts->client, write_buf, 3) != 3) {
			TOUCH_INFO_MSG("MIP_R1_LPWG_ACTIVE_REPORTRATE write error \n");
			return -EIO;
		} else {
			TOUCH_INFO_MSG("MIP_R1_LPWG_ACTIVE_REPORTRATE\n");
		}
		break;
	case SENSITIVITY_CTRL:
		write_buf[0] = MIP_R0_LPWG;
		write_buf[1] = MIP_R1_LPWG_SENSITIVITY;
		write_buf[2] = value;
		if (i2c_master_send(ts->client, write_buf, 3) != 3) {
			TOUCH_INFO_MSG("MIP_R1_LPWG_SENSITIVITY write error \n");
			return -EIO;
		} else {
			TOUCH_INFO_MSG("MIP_R1_LPWG_SENSITIVITY = %d \n", write_buf[2]);
		}
		break;
	/* TCI1 reg */
	case TCI_ENABLE_CTRL:
		write_buf[0] = MIP_R0_LPWG;
		write_buf[1] = MIP_R1_LPWG_ENABLE;
		write_buf[2] = value;
		if (i2c_master_send(ts->client, write_buf, 3) != 3) {
			TOUCH_INFO_MSG("MIP_R1_LPWG_ENABLE write error \n");
			return -EIO;
		} else {
			TOUCH_INFO_MSG("MIP_R1_LPWG_ENABLE = %d \n", write_buf[2]);
		}
		break;
	case TAP_COUNT_CTRL:
		write_buf[0] = MIP_R0_LPWG;
		write_buf[1] = MIP_R1_LPWG_WAKEUP_TAP_COUNT;
		write_buf[2] = value;
		if (i2c_master_send(ts->client, write_buf, 3) != 3) {
			TOUCH_INFO_MSG("MIP_R1_LPWG_WAKEUP_TAP_COUNT write error \n");
			return -EIO;
		} else {
			TOUCH_INFO_MSG("MIP_R1_LPWG_WAKEUP_TAP_COUNT\n");
		}
		break;
	case TOUCH_SLOP_CTRL:
		write_buf[0] = MIP_R0_LPWG;
		write_buf[1] = MIP_R1_LPWG_TOUCH_SLOP;
		write_buf[2] = value;
		if (i2c_master_send(ts->client, write_buf, 3) != 3) {
			TOUCH_INFO_MSG("MIP_R1_LPWG_TOUCH_SLOP write error \n");
			return -EIO;
		} else {
			TOUCH_INFO_MSG("MIP_R1_LPWG_TOUCH_SLOP\n");
		}
		break;
	case TAP_MIN_DISTANCE_CTRL:
		write_buf[0] = MIP_R0_LPWG;
		write_buf[1] = MIP_R1_LPWG_MIN_INTERTAP_DISTANCE;
		write_buf[2] = value;
		if (i2c_master_send(ts->client, write_buf, 3) != 3) {
			TOUCH_INFO_MSG("MIP_R1_LPWG_MIN_INTERTAP_DISTANCE write error \n");
			return -EIO;
		} else {
			TOUCH_INFO_MSG("MIP_R1_LPWG_MIN_INTERTAP_DISTANCE\n");
		}
		break;
	case TAP_MAX_DISTANCE_CTRL:
		write_buf[0] = MIP_R0_LPWG;
		write_buf[1] = MIP_R1_LPWG_MAX_INTERTAP_DISTANCE;
		write_buf[2] = value;
		if (i2c_master_send(ts->client, write_buf, 3) != 3) {
			TOUCH_INFO_MSG("MIP_R1_LPWG_MAX_INTERTAP_DISTANCE write error \n");
			return -EIO;
		} else {
			TOUCH_INFO_MSG("MIP_R1_LPWG_MAX_INTERTAP_DISTANCE\n");
		}
		break;
	case MIN_INTERTAP_CTRL:
		write_buf[0] = MIP_R0_LPWG;
		write_buf[1] = MIP_R1_LPWG_MIN_INTERTAP_TIME;
		write_buf[2] = (value >> 8);
		write_buf[3] = (value & 0xFF);
		if (i2c_master_send(ts->client, write_buf, 4) != 4) {
			TOUCH_INFO_MSG("MIP_R1_LPWG_MIN_INTERTAP_TIME write error \n");
			return -EIO;
		} else {
			TOUCH_INFO_MSG("MIP_R1_LPWG_MIN_INTERTAP_TIME\n");
		}
		break;
	case MAX_INTERTAP_CTRL:
		write_buf[0] = MIP_R0_LPWG;
		write_buf[1] = MIP_R1_LPWG_MAX_INTERTAP_TIME;
		write_buf[2] = (value >> 8);
		write_buf[3] = (value & 0xFF);
		if (i2c_master_send(ts->client, write_buf, 4) != 4) {
			TOUCH_INFO_MSG("MIP_R1_LPWG_MAX_INTERTAP_TIME write error \n");
			return -EIO;
		} else {
			TOUCH_INFO_MSG("MIP_R1_LPWG_MAX_INTERTAP_TIME\n");
		}
		break;
	case INTERRUPT_DELAY_CTRL:
		write_buf[0] = MIP_R0_LPWG;
		write_buf[1] = MIP_R1_LPWG_INT_DELAY_TIME;
		write_buf[2] = ((value ? KNOCKON_DELAY : 0) >> 8);
		write_buf[3] = ((value ? KNOCKON_DELAY : 0) & 0xFF);
		if (i2c_master_send(ts->client, write_buf, 4) != 4) {
			TOUCH_INFO_MSG("MIP_R1_LPWG_INT_DELAY_TIME write error \n");
			return -EIO;
		} else {
			TOUCH_INFO_MSG("MIP_R1_LPWG_INT_DELAY_TIME\n");
		}
		break;
	/* TCI2 reg */
	case TCI_ENABLE_CTRL2:
		write_buf[0] = MIP_R0_LPWG;
		write_buf[1] = MIP_R1_LPWG_ENABLE2;
		write_buf[2] = value;
		if (i2c_master_send(ts->client, write_buf, 3) != 3) {
			TOUCH_INFO_MSG("MIP_R1_LPWG_ENABLE2 write error \n");
			return -EIO;
		} else {
			TOUCH_INFO_MSG("MIP_R1_LPWG_ENABLE2 = %d\n", write_buf[2]);
		}
		break;
	case TAP_COUNT_CTRL2:
		write_buf[0] = MIP_R0_LPWG;
		write_buf[1] = MIP_R1_LPWG_WAKEUP_TAP_COUNT2;
		write_buf[2] = value;
		if (i2c_master_send(ts->client, write_buf, 3) != 3) {
			TOUCH_INFO_MSG("MIP_R1_LPWG_WAKEUP_TAP_COUNT2 write error \n");
			return -EIO;
		} else {
			TOUCH_INFO_MSG("MIP_R1_LPWG_WAKEUP_TAP_COUNT2 = %d\n", write_buf[2]);
		}
		break;
	case TOUCH_SLOP_CTRL2:
		write_buf[0] = MIP_R0_LPWG;
		write_buf[1] = MIP_R1_LPWG_TOUCH_SLOP2;
		write_buf[2] = value;
		if (i2c_master_send(ts->client, write_buf, 3) != 3) {
			TOUCH_INFO_MSG("MIP_R1_LPWG_TOUCH_SLOP2 write error \n");
			return -EIO;
		} else {
			TOUCH_INFO_MSG("MIP_R1_LPWG_TOUCH_SLOP2\n");
		}
		break;
	case TAP_MIN_DISTANCE_CTRL2:
		write_buf[0] = MIP_R0_LPWG;
		write_buf[1] = MIP_R1_LPWG_MIN_INTERTAP_DISTANCE2;
		write_buf[2] = value;
		if (i2c_master_send(ts->client, write_buf, 3) != 3) {
			TOUCH_INFO_MSG("MIP_R1_LPWG_MIN_INTERTAP_DISTANCE2 write error \n");
			return -EIO;
		} else {
			TOUCH_INFO_MSG("MIP_R1_LPWG_MIN_INTERTAP_DISTANCE2\n");
		}
		break;
	case TAP_MAX_DISTANCE_CTRL2:
		write_buf[0] = MIP_R0_LPWG;
		write_buf[1] = MIP_R1_LPWG_MAX_INTERTAP_DISTANCE2;
		write_buf[2] = value;
		if (i2c_master_send(ts->client, write_buf, 3) != 3) {
			TOUCH_INFO_MSG("MIP_R1_LPWG_MAX_INTERTAP_DISTANCE2 write error \n");
			return -EIO;
		} else {
			TOUCH_INFO_MSG("MIP_R1_LPWG_MAX_INTERTAP_DISTANCE2\n");
		}
		break;
	case MIN_INTERTAP_CTRL2:
		write_buf[0] = MIP_R0_LPWG;
		write_buf[1] = MIP_R1_LPWG_MIN_INTERTAP_TIME2;
		write_buf[2] = (value >> 8);
		write_buf[3] = (value & 0xFF);
		if (i2c_master_send(ts->client, write_buf, 4) != 4) {
			TOUCH_INFO_MSG("MIP_R1_LPWG_MIN_INTERTAP_TIME2 write error \n");
			return -EIO;
		} else {
			TOUCH_INFO_MSG("MIP_R1_LPWG_MIN_INTERTAP_TIME2\n");
		}
		break;
	case MAX_INTERTAP_CTRL2:
		write_buf[0] = MIP_R0_LPWG;
		write_buf[1] = MIP_R1_LPWG_MAX_INTERTAP_TIME2;
		write_buf[2] = (value >> 8);
		write_buf[3] = (value & 0xFF);
		if (i2c_master_send(ts->client, write_buf, 4) != 4) {
			TOUCH_INFO_MSG("MIP_R1_LPWG_MAX_INTERTAP_TIME2 write error \n");
			return -EIO;
		} else {
			TOUCH_INFO_MSG("MIP_R1_LPWG_MAX_INTERTAP_TIME2\n");
		}
		break;
	case INTERRUPT_DELAY_CTRL2:
		write_buf[0] = MIP_R0_LPWG;
		write_buf[1] = MIP_R1_LPWG_INT_DELAY_TIME2;
		write_buf[2] = ((value ? KNOCKON_DELAY : 0) >> 8);
		write_buf[3] = ((value ? KNOCKON_DELAY : 0) & 0xFF);
		if (i2c_master_send(ts->client, write_buf, 4) != 4) {
			TOUCH_INFO_MSG("MIP_R1_LPWG_INT_DELAY_TIME2 write error \n");
			return -EIO;
		} else {
			TOUCH_INFO_MSG("MIP_R1_LPWG_INT_DELAY_TIME2\n");
		}
		break;
	case LPWG_STORE_INFO_CTRL:
		TOUCH_INFO_MSG("not used\n");
		break;
	case LPWG_START_CTRL:
		write_buf[0] = MIP_R0_LPWG;
		write_buf[1] = MIP_R1_LPWG_START;
		write_buf[2] = value;
		if (i2c_master_send(ts->client, write_buf, 3) != 3) {
			TOUCH_INFO_MSG("MIP_R1_LPWG_START write error \n");
			return -EIO;
		} else {
			TOUCH_INFO_MSG("MIP_R1_LPWG_START\n");
		}
		break;
	case LPWG_PANEL_DEBUG_CTRL:
		write_buf[0] = MIP_R0_CTRL;
		write_buf[1] = MIP_R1_CTRL_LPWG_DEBUG_ENABLE;
		write_buf[2] = value;
		if (i2c_master_send(ts->client, write_buf, 3) != 3) {
			TOUCH_INFO_MSG("MIP_R1_CTRL_LPWG_DEBUG_ENABLE write error \n");
			return -EIO;
		} else {
			TOUCH_INFO_MSG("MIP_R1_CTRL_LPWG_DEBUG_ENABLE = %d \n", write_buf[2]);
		}
		break;
	case LPWG_FAIL_REASON_CTRL:
		write_buf[0] = MIP_R0_LPWG;
		write_buf[1] = MIP_R1_LPWG_FAIL_REASON;
		write_buf[2] = value;
		if (i2c_master_send(ts->client, write_buf, 3) != 3) {
			TOUCH_INFO_MSG("MIP_R1_LPWG_FAIL_REASON write error \n");
			return -EIO;
		} else {
			TOUCH_INFO_MSG("MIP_R1_LPWG_FAIL_REASON = %d \n", write_buf[2]);
		}
		break;
	default:
		break;
	}

	return 0;
}

/**
* LPWG Config Public
*/
int mip_lpwg_config(struct i2c_client* client)
{
	struct mit_data *ts = get_touch_handle_(client);
	u8 wbuf[32];

	TOUCH_INFO_MSG("%s [START]\n", __func__);

	wbuf[0] = MIP_R0_LPWG;
	wbuf[1] = MIP_R1_LPWG_IDLE_REPORTRATE;
	wbuf[2] = ts->pdata->tci_info->idle_report_rate;	// MIP_ADDR_SET_LPWG_IDLE_REPORTRATE
	wbuf[3] = ts->pdata->tci_info->active_report_rate;	// MIP_ADDR_SET_LPWG_ACTIVE_REPORTRATE
	wbuf[4] = ts->pdata->tci_info->sensitivity;		// MIP_ADDR_SET_LPWG_SENSITIVITY
	wbuf[5] = (ts->pdata->active_area_x1) & 0xFF;		// MIP_ADDR_SET_LPWG_ACTIVE_AREA (horizontal start low byte)
	wbuf[6] = (ts->pdata->active_area_x1) >>8 & 0xFF;	// MIP_ADDR_SET_LPWG_ACTIVE_AREA (horizontal start low byte)
	wbuf[7] = (ts->pdata->active_area_y1) & 0xFF;		// MIP_ADDR_SET_LPWG_ACTIVE_AREA (vertical start low byte)
	wbuf[8] = ((ts->pdata->active_area_y1) >>8) & 0xFF;	// MIP_ADDR_SET_LPWG_ACTIVE_AREA (vertical start high byte)
	wbuf[9] = (ts->pdata->active_area_x2) & 0xFF;		// MIP_ADDR_SET_LPWG_ACTIVE_AREA (horizontal end low byte)
	wbuf[10] = ((ts->pdata->active_area_x2) >>8) & 0xFF;	// MIP_ADDR_SET_LPWG_ACTIVE_AREA (horizontal end high byte)
	wbuf[11] = (ts->pdata->active_area_y2) & 0xFF;		// MIP_ADDR_SET_LPWG_ACTIVE_AREA (vertical end low byte)
	wbuf[12] = ((ts->pdata->active_area_y2)>>8) & 0xFF;	// MIP_ADDR_SET_LPWG_ACTIVE_AREA (vertical end high byte)
	wbuf[13] = ts->pdata->lpwg_fail_reason;			// MIP_ADDR_SET_LPWG_FAIL_REASON

	TOUCH_INFO_MSG("	[Hz] : idle(%4d), active(%4d)\n", wbuf[2], wbuf[3]);
	TOUCH_INFO_MSG("	sensitivity(%4d)\n", wbuf[4]);
	TOUCH_INFO_MSG("	[area start] hori.(%4d) vert.(%4d)\n", (wbuf[6]<<8)|wbuf[5], (wbuf[8]<<8)|wbuf[7]);
	TOUCH_INFO_MSG("	[area end  ] hori.(%4d) vert.(%4d)\n", (wbuf[10]<<8)|wbuf[9], (wbuf[12]<<8)|wbuf[11]);
	TOUCH_INFO_MSG("	fail reason(%d)\n", wbuf[13]);

	if(mip_i2c_write(client, wbuf, 14)) {
		TOUCH_ERR_MSG("%s [ERROR] mip_i2c_write\n", __func__);
		return 1;
	}

	TOUCH_INFO_MSG("%s [DONE]\n", __func__);
	return 0;
}

/**
* LPWG Config Knock-on
*/
int mip_lpwg_config_knock_on(struct i2c_client* client)
{
	struct mit_data *ts = get_touch_handle_(client);
	u8 wbuf[32];

	TOUCH_INFO_MSG("%s [START]\n", __func__);

	wbuf[0] = MIP_R0_LPWG;
	wbuf[1] = MIP_R1_LPWG_ENABLE;
	wbuf[2] = 1;							// MIP_ADDR_SET_LPWG_ENABLE
	wbuf[3] = 2;							// MIP_ADDR_SET_LPWG_TAP_COUNT
	wbuf[4] = ts->pdata->tci_info->touch_slope & 0xFF;		// MIP_ADDR_SET_LPWG_TOUCH_SLOP (low byte)
	wbuf[5] = ts->pdata->tci_info->touch_slope >>8 & 0xFF;		// MIP_ADDR_SET_LPWG_TOUCH_SLOP (high byte)
	wbuf[6] = (ts->pdata->tci_info->min_distance) & 0xFF;		// MIP_ADDR_SET_LPWG_MIN_DISTANCE (low byte)
	wbuf[7] = (ts->pdata->tci_info->min_distance) >>8 & 0xFF;	// MIP_ADDR_SET_LPWG_MIN_DISTANCE (high byte)
	wbuf[8] = (ts->pdata->tci_info->max_distance) & 0xFF;		// MIP_ADDR_SET_LPWG_MAX_DISTANCE (low byte)
	wbuf[9] = (ts->pdata->tci_info->max_distance) >>8 & 0xFF;	// MIP_ADDR_SET_LPWG_MAX_DISTANCE (high byte)
	wbuf[10] = (ts->pdata->tci_info->min_intertap) & 0xFF;		// MIP_ADDR_SET_LPWG_MIN_INTERTAP_TIME (low byte)
	wbuf[11] = (ts->pdata->tci_info->min_intertap) >>8 & 0xFF;	// MIP_ADDR_SET_LPWG_MIN_INTERTAP_TIME (high byte)
	wbuf[12] = (ts->pdata->tci_info->max_intertap) & 0xFF;		// MIP_ADDR_SET_LPWG_MAX_INTERTAP_TIME (low byte)
	wbuf[13] = (ts->pdata->tci_info->max_intertap) >>8 & 0xFF;	// MIP_ADDR_SET_LPWG_MAX_INTERTAP_TIME (high byte)
	wbuf[14] = (ts->pdata->double_tap_check ? KNOCKON_DELAY : 0) & 0xFF;		// MIP_ADDR_SET_LPWG_MAX_INTERTAP_DELAY (low byte)
	wbuf[15] = ((ts->pdata->double_tap_check ? KNOCKON_DELAY : 0) >> 8) & 0xFF;	// MIP_ADDR_SET_LPWG_MAX_INTERTAP_DELAY (high byte)

	TOUCH_INFO_MSG("	enable(%4d), tap_count(%4d) touch_slope(%4d)\n", wbuf[2], wbuf[3], (wbuf[5]<<8)|wbuf[4]);
	TOUCH_INFO_MSG("	[intertap distance] min(%4d), max(%4d)\n", (wbuf[7]<<8)|wbuf[6], (wbuf[9]<<8)|wbuf[8]);
	TOUCH_INFO_MSG("	[intertap time    ] min(%4d), max(%4d)\n", (wbuf[11]<<8)|wbuf[10], (wbuf[13]<<8)|wbuf[12]);
	TOUCH_INFO_MSG("	interrupt_delay(%4d)\n", (wbuf[15]<<8)|wbuf[14]);

	if(mip_i2c_write(client, wbuf, 16)){
		TOUCH_ERR_MSG("%s [ERROR] mip_i2c_write\n", __func__);
		return 1;
	}

	TOUCH_INFO_MSG("%s [DONE]\n", __func__);
	return 0;
}

/**
* LPWG Config Knock-code
*/
int mip_lpwg_config_knock_code(struct i2c_client* client)
{
	struct mit_data *ts = get_touch_handle_(client);
	u8 wbuf[32];

	TOUCH_INFO_MSG("%s [START]\n", __func__);

	wbuf[0] = MIP_R0_LPWG;
	wbuf[1] = MIP_R1_LPWG_ENABLE2;
	wbuf[2] = 1;							// MIP_ADDR_SET_LPWG_ENABLE
	wbuf[3] = ts->pdata->tap_count;					// MIP_ADDR_SET_LPWG_TAP_COUNT
	wbuf[4] = ts->pdata->tci_info->touch_slope_2 & 0xFF;		// MIP_ADDR_SET_LPWG_TOUCH_SLOP (low byte)
	wbuf[5] = ts->pdata->tci_info->touch_slope_2 >>8 & 0xFF;	// MIP_ADDR_SET_LPWG_TOUCH_SLOP (high byte)
	wbuf[6] = (ts->pdata->tci_info->min_distance_2) & 0xFF;		// MIP_ADDR_SET_LPWG_MIN_DISTANCE (low byte)
	wbuf[7] = (ts->pdata->tci_info->min_distance_2) >>8 & 0xFF;	// MIP_ADDR_SET_LPWG_MIN_DISTANCE (high byte)
	wbuf[8] = (ts->pdata->tci_info->max_distance_2) & 0xFF;		// MIP_ADDR_SET_LPWG_MAX_DISTANCE (low byte)
	wbuf[9] = (ts->pdata->tci_info->max_distance_2) >>8 & 0xFF;	// MIP_ADDR_SET_LPWG_MAX_DISTANCE (high byte)
	wbuf[10] = (ts->pdata->tci_info->min_intertap_2) & 0xFF;	// MIP_ADDR_SET_LPWG_MIN_INTERTAP_TIME (low byte)
	wbuf[11] = (ts->pdata->tci_info->min_intertap_2) >>8 & 0xFF;	// MIP_ADDR_SET_LPWG_MIN_INTERTAP_TIME (high byte)
	wbuf[12] = (ts->pdata->tci_info->max_intertap_2) & 0xFF;	// MIP_ADDR_SET_LPWG_MAX_INTERTAP_TIME (low byte)
	wbuf[13] = (ts->pdata->tci_info->max_intertap_2) >>8 & 0xFF;	// MIP_ADDR_SET_LPWG_MAX_INTERTAP_TIME (high byte)
	wbuf[14] = (ts->pdata->tci_info->interrupt_delay_2) & 0xFF;	// MIP_ADDR_SET_LPWG_MAX_INTERTAP_DELAY (low byte)
	wbuf[15] = (ts->pdata->tci_info->interrupt_delay_2) >>8 & 0xFF;	// MIP_ADDR_SET_LPWG_MAX_INTERTAP_DELAY (high byte)


	TOUCH_INFO_MSG("	enable(%4d), tap_count(%4d) touch_slope(%4d)\n", wbuf[2], wbuf[3], (wbuf[5]<<8)|wbuf[4]);
	TOUCH_INFO_MSG("	[intertap distance] min(%4d), max(%4d)\n", (wbuf[7]<<8)|wbuf[6], (wbuf[9]<<8)|wbuf[8]);
	TOUCH_INFO_MSG("	[intertap time    ] min(%4d), max(%4d)\n", (wbuf[11]<<8)|wbuf[10], (wbuf[13]<<8)|wbuf[12]);
	TOUCH_INFO_MSG("	interrupt_delay(%4d)\n", (wbuf[15]<<8)|wbuf[14]);

	if(mip_i2c_write(client, wbuf, 16)){
		TOUCH_ERR_MSG("%s [ERROR] mip_i2c_write\n", __func__);
		return 1;
	}

	TOUCH_INFO_MSG("%s [DONE]\n", __func__);
	return 0;
}

/**
* Set LPWG Debug Enable
*/
int mip_lpwg_debug_enable(struct i2c_client* client)
{
	struct mit_data *ts = get_touch_handle_(client);
	u8 wbuf[4];
	TOUCH_INFO_MSG("%s [START]\n", __func__);

	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_LPWG_DEBUG_ENABLE;
	wbuf[2] = ts->pdata->lpwg_debug_enable;
	TOUCH_INFO_MSG("%s lpwg_debug_enable : %d \n", __func__, wbuf[2]);
	if(mip_i2c_write(client, wbuf, 3)){
		TOUCH_ERR_MSG("%s [ERROR] mip_i2c_write\n", __func__);
		return 1;
	}

	TOUCH_INFO_MSG("%s [DONE]\n", __func__);
	return 0;
}

int mip_lpwg_enable_sensing(struct i2c_client* client, bool bEnable)
{

        u8 wbuf[4];

        struct mit_data *ts = get_touch_handle_(client);
        TOUCH_INFO_MSG("%s [START] bEnable : %d\n", __func__, bEnable);

	//send Dummy packet(lpwg mode)
	if(ts->pdata->lpwg_mode_old){
		if(mit_i2c_dummy(client, wbuf, 2)){
			TOUCH_INFO_MSG("%s [ERROR] Dummy packet\n", __func__);
			return -EIO;
		}
	}

        wbuf[0] = MIP_R0_LPWG;
        wbuf[1] = MIP_R1_LPWG_ENABLE_SENSING;
        wbuf[2] = bEnable;

        if(mip_i2c_write(client, wbuf, 3)){
                TOUCH_ERR_MSG("%s [ERROR] mip_i2c_write\n", __func__);
                return 1;
        }

        TOUCH_INFO_MSG("%s [DONE]\n", __func__);
        return 0;


}
/**
* LPWG Control Start
*/
int mip_lpwg_start(struct i2c_client* client)
{
	u8 wbuf[4];

	TOUCH_INFO_MSG("%s [START]\n", __func__);

	wbuf[0] = MIP_R0_LPWG;
	wbuf[1] = MIP_R1_LPWG_START;
	wbuf[2] = 1;

	if(mip_i2c_write(client, wbuf, 3)){
		TOUCH_ERR_MSG("%s [ERROR] mip_i2c_write\n", __func__);
		return 1;
	}

	TOUCH_INFO_MSG("%s [DONE]\n", __func__);
	return 0;
}

static int lpwg_control(struct mit_data* ts, u8 mode)
{
	switch (mode) {
	case LPWG_SIGNATURE:
		goto LPWG_NOT_USE;
		break;
	case LPWG_DOUBLE_TAP:
		mip_lpwg_config(ts->client);
		mip_lpwg_config_knock_on(ts->client);
		if(ts->pdata->lpwg_debug_enable){
			mip_lpwg_debug_enable(ts->client);
		}
                mip_lpwg_start(ts->client);
                mip_lpwg_enable_sensing(ts->client, ts->enable_sensing);
		break;
	case LPWG_MULTI_TAP:
		mip_lpwg_config(ts->client);
		mip_lpwg_config_knock_on(ts->client);
		mip_lpwg_config_knock_code(ts->client);
		if(ts->pdata->lpwg_debug_enable){
			mip_lpwg_debug_enable(ts->client);
		}
                mip_lpwg_start(ts->client);
                mip_lpwg_enable_sensing(ts->client, ts->enable_sensing);
		break;
	default:
		tci_control(ts, TCI_ENABLE_CTRL, 0);
		tci_control(ts, TCI_ENABLE_CTRL2, 0);
		goto LPWG_NOT_USE;
		break;
	}

	TOUCH_INFO_MSG("[lpwg mode] old(%d), current(%d)\n", ts->pdata->lpwg_mode_old, mode);
	ts->pdata->lpwg_mode_old = mode;

LPWG_NOT_USE:
	return 0;
}

int mit_ic_ctrl(struct i2c_client *client, u32 code, u32 value)
{
	struct mit_data* ts = (struct mit_data *) get_touch_handle_(client);
	struct ic_ctrl_param *param = (struct ic_ctrl_param *) value;
	int ret = 0;
	char *buf = NULL;
	TOUCH_TRACE_FUNC();

	switch (code) {
		case IC_CTRL_FIRMWARE_IMG_SHOW:
			ret = mit_firmware_img_parse_show((const char *) param->v1, (char *) param->v2, param->v3);
			break;

		case IC_CTRL_INFO_SHOW:
			mit_get_ic_info(ts, NULL);
			if (param) {
				buf = (char *) param->v1;
				if (buf) {
					if (ts->pdata->panel_on) {
						ret += sprintf(buf + ret, "====== LCD  ON  ======\n");
					} else {
						ret += sprintf(buf + ret, "====== LCD  OFF ======\n");
					}
					ret += sprintf(buf + ret, "======================\n");
					ret += sprintf(buf + ret, "F/W Version : %X.%02X \n", ts->module.version[0], ts->module.version[1]);
					ret += sprintf(buf + ret, "F/W Product : %s \n", ts->module.product_code);
					ret += sprintf(buf + ret, "F/W Row : %d, Col : %d\n", ts->dev.row_num, ts->dev.col_num);
					ret += sprintf(buf + ret, "IC Name : [%c%c%c%c] \n", ts->module.ic_name[0], ts->module.ic_name[1], ts->module.ic_name[2], ts->module.ic_name[3]);
					if (ts->module.otp == OTP_NOT_SUPPORTED) {
						ret += sprintf(buf + ret, "OTP : F/W Not support \n");
					} else {
						ret += sprintf(buf + ret, "OTP : %s \n", (ts->module.otp == OTP_APPLIED) ? "Applied" : "None");
					}
					ret += sprintf(buf + ret, "======================\n");
				}
			}
			break;

		case IC_CTRL_FIRMWARE_VERSION_SHOW:
			if (param) {
				buf = (char *) param->v1;
				if (buf) {
					ret = sprintf(buf, "\n======== IC Firmware Info ========\n");
					ret += sprintf(buf + ret, "FW Version: %X.%02X\n", ts->module.version[0], ts->module.version[1]);
					ret += sprintf(buf + ret, "IC Name : [%c%c%c%c] \n", ts->module.ic_name[0], ts->module.ic_name[1], ts->module.ic_name[2], ts->module.ic_name[3]);
					ret += sprintf(buf + ret, "F/W Product : %s \n", ts->module.product_code);
					ret += sprintf(buf + ret, "F/W Row : %d, Col : %d\n", ts->dev.row_num, ts->dev.col_num);

					ret += sprintf(buf + ret, "\n======== BIN Firmware Info ========\n");
					ret += sprintf(buf + ret, "FW Version: %X.%02X\n", ts->module.bin_version[0], ts->module.bin_version[1]);
					ret += sprintf(buf + ret, "IC Name : %s \n", ts->module.bin_chip_name);

				}
			}
			break;

		case IC_CTRL_TESTMODE_VERSION_SHOW:
			if (ts->module.product_code[0])
				TOUCH_INFO_MSG("F/W : %X.%02X (%s)\n", ts->module.version[0], ts->module.version[1], ts->module.product_code);
			if (param) {
				buf = (char *) param->v1;
				if (buf) {
					ret += sprintf(buf + ret, "%X.%02X(%s)\n", ts->module.version[0], ts->module.version[1], ts->module.product_code);
				}
			}
			break;

		case IC_CTRL_SAVE_IC_INFO:
			mit_get_ic_info(ts, NULL);
			break;

		case IC_CTRL_LPWG:
			if ( (u8)param->v1 == ts->pdata->lpwg_mode_old ) {
				TOUCH_INFO_MSG("LPWG mode is already setted [ Current : %d / Old : %d]\n", (u8)param->v1, ts->pdata->lpwg_mode_old);
				return 0;
			} else {
				TOUCH_INFO_MSG("Current lpwg_mode is [%d] & ts->pdata->lpwg_mode_old [%d]\n", (u8)param->v1, ts->pdata->lpwg_mode_old);
			}

			if ( lpwg_control(ts, (u8)param->v1) == -EIO ) {
				TOUCH_ERR_MSG("LPWG Control fail, so retry to Reset Touch IC \n");

				touch_disable(ts->client->irq);

				mit_power(ts->client, POWER_OFF);
				msleep(200);
				mit_power(ts->client, POWER_ON);
				msleep(ts->pdata->role->reset_delay);

				touch_enable(ts->client->irq);

				if ( lpwg_control(ts, (u8)param->v1) == -EIO )
					TOUCH_ERR_MSG("After reset, LPWG Control is fail...again -> Please check Touch IC \n");
			}
#if defined(TOUCH_USE_DSV)
			if (ts_pdata->enable_sensor_interlock) {
				if (ts_pdata->sensor_value) {
					ts_pdata->use_dsv = 1;
					mit_dsv_control(client);
				}
			} else {
				if (ts_pdata->use_dsv) {
					mit_dsv_control(client);
				}
			}
#endif
			break;

		case IC_CTRL_ACTIVE_AREA:
			mit_set_active_area(ts, (u8)param->v1);
			break;
	}
	return ret;
}

static int mit_reg_control_store(struct i2c_client *client, const char *buf)
{
	struct mit_data *ts = (struct mit_data *) get_touch_handle_(client);
	int cmd = 0;
	int ret = 0;
	int reg_addr[2] = {0};
	int value = 0;
	uint8_t write_buf[50] = {0};
	uint8_t read_buf[50] = {0};
	int i = 0;
	int len = 2;

	if ( sscanf(buf, "%d %x %x %d", &cmd, &reg_addr[0], &reg_addr[1], &value) != 4) {
		TOUCH_INFO_MSG("data parsing fail.\n");
		TOUCH_INFO_MSG("%d, 0x%x, 0x%x, %d\n", cmd, reg_addr[0], reg_addr[1], value);
		return -EINVAL;
	}
	TOUCH_INFO_MSG("%d, 0x%x, 0x%x, %d\n", cmd, reg_addr[0], reg_addr[1], value);

	switch (cmd) {
		case 1:
			write_buf[0] = reg_addr[0];
			write_buf[1] = reg_addr[1];

			ret = i2c_master_send(ts->client, write_buf,len);
			if (ret < 0) {
				TOUCH_INFO_MSG("i2c master send fail\n");
				break;
			}
			ret = i2c_master_recv(ts->client, read_buf, value);
			if (ret < 0) {
				TOUCH_INFO_MSG("i2c master recv fail\n");
				break;
			}
			for (i = 0; i < value; i ++) {
				TOUCH_INFO_MSG("read_buf=[%d]\n",read_buf[i]);
			}
			TOUCH_INFO_MSG("register read done\n");
			break;
		case 2:
			write_buf[0] = reg_addr[0];
			write_buf[1] = reg_addr[1];
			if (value >= 256) {
				write_buf[2] = (value >> 8);
				write_buf[3] = (value & 0xFF);
				len = len + 2;
			} else {
				write_buf[2] = value;
				len++;
			}
			ret = i2c_master_send(ts->client, write_buf, len);
			if (ret < 0) {
				TOUCH_INFO_MSG("i2c master send fail\n");
				break;
			}
			TOUCH_INFO_MSG("register write done\n");
			break;
		default:
			TOUCH_INFO_MSG("usage: echo [1(read)|2(write)], [reg address0], [reg address1], [length(read)|value(write)] > reg_control\n");
			TOUCH_INFO_MSG("  - Register Set or Read\n");
			break;
	}
	return ret;
}

static int mit_tci_store(struct i2c_client *client, const char *buf)
{
	struct mit_data *ts = (struct mit_data *) get_touch_handle_(client);
	int ret = 0;
	int value = 0;
	int type = 0;

	if (sscanf(buf, "%d %d", &type, &value) != 2) {
		TOUCH_INFO_MSG("data parsing fail.\n");
		TOUCH_INFO_MSG("%d, %d\n", type, value);
		return -EINVAL;
	}

	TOUCH_INFO_MSG("%s - TCI reg control type : %d, value : %d\n", __func__, type, value);

	switch(type) {
		case IDLE_REPORTRATE_CTRL:
			ts->pdata->tci_info->idle_report_rate = value;
		break;
		case ACTIVE_REPORTRATE_CTRL:
			ts->pdata->tci_info->active_report_rate =value;
			break;
		case SENSITIVITY_CTRL:
			ts->pdata->tci_info->sensitivity = value;
			break;
		case TCI_ENABLE_CTRL:
			TOUCH_INFO_MSG("You can't control TCI_ENABLE_CTRL register\n");
			return 0;
		case TOUCH_SLOP_CTRL:
			ts->pdata->tci_info->touch_slope = value;
			break;
		case TAP_MIN_DISTANCE_CTRL:
			ts->pdata->tci_info->min_distance = value;
			break;
		case TAP_MAX_DISTANCE_CTRL:
			ts->pdata->tci_info->max_distance = value;
			break;
		case MIN_INTERTAP_CTRL:
			ts->pdata->tci_info->min_intertap = value;
			break;
		case MAX_INTERTAP_CTRL:
			ts->pdata->tci_info->max_intertap = value;
			break;
		case TAP_COUNT_CTRL:
			ts->pdata->tci_info->tap_count = value;
			break;
		case INTERRUPT_DELAY_CTRL:
			TOUCH_INFO_MSG("You can't control INTERRUPT_DELAY_CTRL register\n");
			return 0;
		case TCI_ENABLE_CTRL2:
			TOUCH_INFO_MSG("You can't control TCI_ENABLE_CTRL2 register\n");
			return 0;
		case TOUCH_SLOP_CTRL2:
			ts->pdata->tci_info->touch_slope_2 = value;
			break;
		case TAP_MIN_DISTANCE_CTRL2:
			ts->pdata->tci_info->min_distance_2 = value;
			break;
		case TAP_MAX_DISTANCE_CTRL2:
			ts->pdata->tci_info->max_distance_2 = value;
			break;
		case MIN_INTERTAP_CTRL2:
			ts->pdata->tci_info->min_intertap_2 = value;
			break;
		case MAX_INTERTAP_CTRL2:
			ts->pdata->tci_info->max_intertap_2 = value;
			break;
		case TAP_COUNT_CTRL2:
			TOUCH_INFO_MSG("You can't control TAP_COUNT_CTRL2 register\n");
			return 0;
		case INTERRUPT_DELAY_CTRL2:
			ts->pdata->tci_info->interrupt_delay_2 = value;
			break;
		case LPWG_STORE_INFO_CTRL:
			TOUCH_INFO_MSG("not used\n");
			return 0;
		case LPWG_START_CTRL:
			TOUCH_INFO_MSG("You can't control LPWG_START_CTRL register\n");
			return 0;
		default:
			TOUCH_INFO_MSG("incorrect command\n");
			return 0;
	}

	tci_control(ts, type, value);

	return ret;
}

static ssize_t mit_rawdata_show(struct i2c_client *client, char *buf)
{
	struct mit_data *ts = (struct mit_data *) get_touch_handle_(client);
	int ret = 0;
	ts->pdata->selfdiagnostic_state[SD_RAWDATA] = 1;	// rawdata

	TOUCH_TRACE_FUNC();
	printk(KERN_ERR "%s\n", __func__);

	ret = mit_get_test_result(client, buf, RAW_DATA_SHOW);
	if (ret < 0) {
		memset(buf, 0, PAGE_SIZE);
		ret = snprintf(buf, PAGE_SIZE, "failed to get raw data\n");
	}
	return ret;
}

static ssize_t mit_rawdata_store(struct i2c_client *client, const char *buf)
{
	int ret = 0;
	char temp_buf[255] = {0};
	TOUCH_TRACE_FUNC();
	strncpy(temp_buf,buf,strlen(temp_buf));

	ret = mit_get_test_result(client, temp_buf, RAW_DATA_STORE);

	return ret;
}

static ssize_t mit_chstatus_show(struct i2c_client *client, char *buf)
{
	int ret = 0;
	int len = 0;
	struct mit_data *ts = (struct mit_data *) get_touch_handle_(client);
	ts->pdata->selfdiagnostic_state[SD_CM_DELTA] = 1;	// cm delta
	ts->pdata->selfdiagnostic_state[SD_JITTER] = 1;		// jitter
	ts->pdata->selfdiagnostic_state[SD_OPENSHORT] = 1;	// openshort
	ts->pdata->selfdiagnostic_state[SD_MUXSHORT] = 1;	// muxshort
	//ts->pdata->selfdiagnostic_state[SD_SLOPE] = 1;	// slope

	TOUCH_TRACE_FUNC();
	TOUCH_INFO_MSG("mit_chstatus_show\n");

	ret = mit_get_test_result(client, buf, OPENSHORT);
	memset(buf, 0, PAGE_SIZE);
	if (ret < 0) {
		TOUCH_INFO_MSG("Failed to get OPEN SHORT Test result. \n");
		ret = snprintf(buf, PAGE_SIZE, "failed to OPEN SHORT data\n");
		goto error;
	}

/*
	ret = mit_get_test_result(client, buf, SLOPE);
	memset(buf, 0, PAGE_SIZE);
	if (ret < 0) {
		TOUCH_INFO_MSG("Failed to get SLOPE Test result. \n");
		ret = snprintf(buf, PAGE_SIZE, "failed to SLOPE data\n");
		goto error;
	}
*/

	len = snprintf(buf, PAGE_SIZE - len, "Firmware Version : %X.%02X \n", ts->module.version[0], ts->module.version[1]);
	len += snprintf(buf + len, PAGE_SIZE - len, "FW Product : %s \n", ts->module.product_code);
	len += snprintf(buf + len, PAGE_SIZE - len, "=======RESULT========\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "OPEN /  CM DELTA Test : %s\n", ts->pdata->selfdiagnostic_state[SD_CM_DELTA]==1 ? "PASS" : "FAIL");
	len += snprintf(buf + len, PAGE_SIZE - len, "OPEN /  JITTER Test   : %s\n", ts->pdata->selfdiagnostic_state[SD_JITTER]==1 ? "PASS" : "FAIL");
	len += snprintf(buf + len, PAGE_SIZE - len, "OPEN /  SHORT Test    : %s\n", ts->pdata->selfdiagnostic_state[SD_OPENSHORT]==1 ? "PASS" : "FAIL");
	len += snprintf(buf + len, PAGE_SIZE - len, "MUX /  SHORT Test     : %s\n", ts->pdata->selfdiagnostic_state[SD_MUXSHORT]==1 ? "PASS" : "FAIL");
	//len += snprintf(buf + len, PAGE_SIZE - len, "SLOPE Test : %s\n", ts->pdata->selfdiagnostic_state[SD_SLOPE] == 1 ? "PASS" : "FAIL");

	return len;

error:
	return ret;
}

static ssize_t mit_chstatus_store(struct i2c_client *client, const char *buf)
{
	int ret = 0;
	char temp_buf[255] = {0};
	TOUCH_TRACE_FUNC();
	strncpy(temp_buf,buf,strlen(temp_buf));

	ret = mit_get_test_result(client, temp_buf, OPENSHORT_STORE);

	return ret;
}

static int melfas_delta_show(struct i2c_client* client, char *buf)
{
	int ret = 0;

	TOUCH_TRACE_FUNC();

	ret = mit_delta_show(client, buf);

	return ret;
}

static ssize_t mit_self_diagnostic_show(struct i2c_client *client, char *buf)
{
	struct mit_data *ts = (struct mit_data *) get_touch_handle_(client);
	int len = 0;
	int ret = 0;
	int row = 0;
	int col = 0;
	u32 limit_upper = 0;
	u32 limit_lower = 0;
	char *sd_path = "/sdcard/touch_self_test.txt";
	ts->pdata->selfdiagnostic_state[SD_RAWDATA] = 1;	/* rawdata */
	ts->pdata->selfdiagnostic_state[SD_CM_DELTA] = 1;	/* cm delta */
	ts->pdata->selfdiagnostic_state[SD_JITTER] = 1;		/* jitter */
	ts->pdata->selfdiagnostic_state[SD_OPENSHORT] = 1;	/* openshort */
	ts->pdata->selfdiagnostic_state[SD_MUXSHORT] = 1;	// muxshort
	//ts->pdata->selfdiagnostic_state[SD_SLOPE] = 1;		/*  slope */

	mit_get_otp(ts);

	write_file(sd_path, buf, 1);
	msleep(30);

	ret = mit_get_test_result(client, buf, OPENSHORT);
	if (ret < 0) {
		TOUCH_ERR_MSG("failed to get open short data\n");
		memset(buf, 0, PAGE_SIZE);
		ts->d_max = 0;
		ts->d_min = 0;
		ts->j_max = 0;
		ts->j_min = 0;
		ts->o_max = 0;
		ts->o_min = 0;
		ts->pdata->selfdiagnostic_state[SD_OPENSHORT] = 0;
		len += snprintf(buf, PAGE_SIZE, "failed to get open short data\n\n");
	}
/*
	write_file(sd_path, buf, 0);
	msleep(30);

	memset(buf, 0, PAGE_SIZE);
	ts->pdata->selfdiagnostic_state[SD_SLOPE] = 1;	// slope
	ret = mit_get_test_result(client, buf, SLOPE);
	if (ret < 0) {
		TOUCH_ERR_MSG("failed to get slope data\n");
		memset(buf, 0, PAGE_SIZE);
		ts->s_max = 0;
		ts->s_min = 0;
		ts->pdata->selfdiagnostic_state[SD_SLOPE] = 0;
		len = snprintf(buf, PAGE_SIZE, "failed to get slope data\n\n");
	}
*/
	write_file(sd_path, buf, 0);
	msleep(30);

	memset(buf, 0, PAGE_SIZE);
	ts->pdata->selfdiagnostic_state[SD_RAWDATA] = 1;	/* rawdata */
	ret = mit_get_test_result(client, buf, RAW_DATA_SHOW);
	if (ret < 0) {
		TOUCH_ERR_MSG("failed to get raw data\n");
		memset(buf, 0, PAGE_SIZE);
		ts->r_max = 0;
		ts->r_min = 0;
		ts->pdata->selfdiagnostic_state[SD_RAWDATA] = 0;
		ret = snprintf(buf, PAGE_SIZE, "failed to get raw data\n\n");
	}

	if (ts->module.otp == OTP_APPLIED) {
		limit_upper = ts->pdata->limit->raw_data_otp_max + ts->pdata->limit->raw_data_margin;
		limit_lower = ts->pdata->limit->raw_data_otp_min - ts->pdata->limit->raw_data_margin;
		ret += sprintf(buf+ret,"RAW DATA SPEC (UPPER : %d  LOWER : %d  MARGIN : %d)\n",
			ts->pdata->limit->raw_data_otp_max , ts->pdata->limit->raw_data_otp_min, ts->pdata->limit->raw_data_margin);
		TOUCH_INFO_MSG("RAW DATA SPEC (UPPER : %d  LOWER : %d MARGIN : %d)\n",
			ts->pdata->limit->raw_data_otp_max , ts->pdata->limit->raw_data_otp_min, ts->pdata->limit->raw_data_margin);
	} else {
		limit_upper = ts->pdata->limit->raw_data_max + ts->pdata->limit->raw_data_margin;
		limit_lower = ts->pdata->limit->raw_data_min - ts->pdata->limit->raw_data_margin;
		ret += sprintf(buf+ret,"RAW DATA SPEC (UPPER : %d  LOWER : %d MARGIN : %d)\n",
			ts->pdata->limit->raw_data_max , ts->pdata->limit->raw_data_min, ts->pdata->limit->raw_data_margin);
		TOUCH_INFO_MSG("RAW DATA SPEC (UPPER : %d  LOWER : %d MARGIN : %d)\n",
			ts->pdata->limit->raw_data_max , ts->pdata->limit->raw_data_min, ts->pdata->limit->raw_data_margin);
	}

	if (ts->pdata->selfdiagnostic_state[SD_RAWDATA] == 0) {

		for (row = 0 ; row < MAX_ROW; row++) {
				ret += sprintf(buf+ret,"[%2d]  ",row);
				printk("[Touch] [%2d]  ",row);

			for (col = 0 ; col < MAX_COL ; col++) {

					if (ts->mit_data[row][col] <= limit_upper && ts->mit_data[row][col] >= limit_lower ){
							ret += sprintf(buf+ret," ,");
							printk(" ,");
						}else{
							ret += sprintf(buf+ret,"X,");
							printk("X,");
						}
			}
				printk("\n");
				ret += sprintf(buf+ret,"\n");
		}
		ret += sprintf(buf+ret,"RawData : FAIL\n\n");
		TOUCH_INFO_MSG("RawData : FAIL\n\n");
	}else {
		ret += sprintf(buf+ret,"RawData : PASS\n\n");
		TOUCH_INFO_MSG("RawData : PASS\n\n");
	}
	write_file(sd_path, buf, 0);
	msleep(30);

	TOUCH_INFO_MSG("Firmware Version : %X.%02X \n", ts->module.version[0], ts->module.version[1]);
	TOUCH_INFO_MSG("FW Product : %s \n", ts->module.product_code);


	if (ts->module.otp == OTP_NOT_SUPPORTED) {
		TOUCH_INFO_MSG("OTP : F/W Not support \n");
	} else {
		TOUCH_INFO_MSG("OTP : %s \n", (ts->module.otp == OTP_APPLIED) ? "Applied" : "None");
	}
	TOUCH_INFO_MSG("=====================\n");
	if (ts->pdata->check_openshort){
		TOUCH_INFO_MSG("CM Delta  : %5d , %5d\n", ts->d_max, ts->d_min);
		TOUCH_INFO_MSG("Jitter    : %5d , %5d\n", ts->j_max, ts->j_min);
		TOUCH_INFO_MSG("OpenShort : %5d , %5d\n", ts->o_max, ts->o_min);
		TOUCH_INFO_MSG("MuxShort  : %5d , %5d\n", ts->m_max, ts->m_min);
	}
	//TOUCH_INFO_MSG("Slope     : %5d , %5d\n", ts->s_max, ts->s_min);
	TOUCH_INFO_MSG("Rawdata   : %5d , %5d\n", ts->r_max, ts->r_min);
	TOUCH_INFO_MSG("=======RESULT========\n");
	//TOUCH_INFO_MSG("Channel Status : %s\n", (ts->pdata->selfdiagnostic_state[SD_OPENSHORT] * ts->pdata->selfdiagnostic_state[SD_SLOPE]) == 1 ? "PASS" : "FAIL");
	TOUCH_INFO_MSG("Channel Status : %s\n", (ts->pdata->selfdiagnostic_state[SD_CM_DELTA] * ts->pdata->selfdiagnostic_state[SD_JITTER] * ts->pdata->selfdiagnostic_state[SD_OPENSHORT] * ts->pdata->selfdiagnostic_state[SD_MUXSHORT]) == 1 ? "PASS" : "FAIL");
	TOUCH_INFO_MSG("Raw Data       : %s\n", ts->pdata->selfdiagnostic_state[SD_RAWDATA] == 1 ? "PASS" : "FAIL");


	//TOUCH_INFO_MSG("SD_OPENSHORT : %d , SD_SLOPE : %d\n", ts->pdata->selfdiagnostic_state[SD_OPENSHORT] , ts->pdata->selfdiagnostic_state[SD_SLOPE]);


	memset(buf, 0, PAGE_SIZE);
	len = snprintf(buf, PAGE_SIZE , "Firmware Version : %X.%02X \n", ts->module.version[0], ts->module.version[1]);
	len += snprintf(buf + len, PAGE_SIZE - len, "FW Product : %s \n", ts->module.product_code);
	if (ts->module.otp == OTP_NOT_SUPPORTED) {
		len += snprintf(buf + len, PAGE_SIZE - len, "OTP : F/W Not support \n");
	} else {
		len += snprintf(buf + len, PAGE_SIZE - len, "OTP : %s \n", (ts->module.otp == OTP_APPLIED) ? "Applied" : "None");
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "=====================\n");
	if (ts->pdata->check_openshort){
		len += snprintf(buf + len, PAGE_SIZE - len, "CM Delta : %5d , %5d\n", ts->d_max, ts->d_min);
		len += snprintf(buf + len, PAGE_SIZE - len, "Jitter : %5d , %5d\n", ts->j_max, ts->j_min);
		len += snprintf(buf + len, PAGE_SIZE - len, "OpenShort : %5d , %5d\n", ts->o_max, ts->o_min);
		len += snprintf(buf + len, PAGE_SIZE - len, "MuxShort : %5d , %5d\n", ts->m_max, ts->m_min);
	}
	//len += snprintf(buf + len, PAGE_SIZE - len, "Slope     : %5d , %5d\n", ts->s_max, ts->s_min);
	len += snprintf(buf + len, PAGE_SIZE - len, "Rawdata   : %5d , %5d\n", ts->r_max, ts->r_min);
	len += snprintf(buf + len, PAGE_SIZE - len, "=======RESULT========\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "Channel Status : %s\n", (ts->pdata->selfdiagnostic_state[SD_CM_DELTA] * ts->pdata->selfdiagnostic_state[SD_JITTER] * ts->pdata->selfdiagnostic_state[SD_OPENSHORT] * ts->pdata->selfdiagnostic_state[SD_MUXSHORT]) == 1 ? "PASS" : "FAIL");
	//len += snprintf(buf + len, PAGE_SIZE - len, "Channel Status : %s\n", (ts->pdata->selfdiagnostic_state[SD_OPENSHORT] * ts->pdata->selfdiagnostic_state[SD_SLOPE]) == 1 ? "PASS" : "FAIL");
	len += snprintf(buf + len, PAGE_SIZE - len, "Raw Data : %s\n", ts->pdata->selfdiagnostic_state[SD_RAWDATA] == 1 ? "PASS" : "FAIL");
	write_file(sd_path, buf, 0);
	return len;
}

static int mit_sensing_block_control(struct i2c_client *client, u8 type, u8 onoff)
{
	struct mit_data* ts = get_touch_handle_(client);
	u8 wbuf[6] = {32, 0, 81, 1, 0, 0};
	int i = 0;

	switch(type) {
		case 0 :
			break;
		case 81 :
		case 82 :
		case 83 :
			wbuf[2] = (u8)type;
			break;
		default :
			TOUCH_INFO_MSG("not support %d \n", type);
			return 0;
	}

	if (onoff)
		wbuf[4] = 1;
	else
		wbuf[4] = 0;

	if (type == 0) {
		for (i = 81; i <= 83; i++) {
			wbuf[2] = i;
			i2c_master_send(ts->client, wbuf, 6);
			TOUCH_INFO_MSG("Sensing Block (%d) : %s \n", wbuf[2], onoff ? "On" : "Off");
		}
	} else {
		i2c_master_send(ts->client, wbuf, 6);
		TOUCH_INFO_MSG("Sensing Block (%d) : %s \n", wbuf[2], onoff ? "On" : "Off");
	}

	wbuf[0] = 0x1F;
	wbuf[1] = 0xFF;
	wbuf[2] = 1;
	i2c_master_send(ts->client, wbuf, 3);


	if (onoff) {
		touch_disable(ts->client->irq);
		mit_power_reset(ts);
		touch_enable(ts->client->irq);
	}

	return 0;
}

static ssize_t mit_fw_dump_show(struct i2c_client *client, char *buf)
{
	struct mit_data *ts = (struct mit_data *) get_touch_handle_(client);
	int len = 0;
	u8 *pDump = NULL;
	int readsize = 0;
	int addr = 0;
	int retrycnt = 0;
	int fd = 0;
	char *dump_path = "/sdcard/touch_dump.fw";
	mm_segment_t old_fs = get_fs();

	TOUCH_INFO_MSG("F/W Dumping... \n");

	touch_disable(ts->client->irq);

	pDump = kzalloc(FW_MAX_SIZE, GFP_KERNEL);

RETRY :
	readsize = 0;
	retrycnt++;
	mit_power_reset(ts);
	msleep(50);

	for (addr = 0; addr < FW_MAX_SIZE; addr += FW_BLOCK_SIZE ) {
		if ( mip_isc_read_page(ts, addr, &pDump[addr]) ) {
			TOUCH_INFO_MSG("F/W Read failed \n");
			if (retrycnt > 10) {
				len += snprintf(buf + len, PAGE_SIZE - len, "dump failed \n");
				goto EXIT;
			}
			else
				goto RETRY;
		}

		readsize += FW_BLOCK_SIZE;
		if (readsize % (FW_BLOCK_SIZE * 20) == 0) {
			TOUCH_INFO_MSG("\t Dump %5d / %5d bytes\n", readsize, FW_MAX_SIZE);
		}
	}

	TOUCH_INFO_MSG("\t Dump %5d / %5d bytes\n", readsize, FW_MAX_SIZE);

	set_fs(KERNEL_DS);
	fd = sys_open(dump_path, O_WRONLY|O_CREAT, 0666);
	if (fd >= 0) {
		sys_write(fd, pDump, FW_MAX_SIZE);
		sys_close(fd);
		len += snprintf(buf + len, PAGE_SIZE - len, "%s saved \n", dump_path);
	} else {
		len += snprintf(buf + len, PAGE_SIZE - len, "%s open failed \n", dump_path);
	}

	set_fs(old_fs);

EXIT :
	kfree(pDump);

	mip_isc_exit(ts);

	mit_power_reset(ts);

	touch_enable(ts->client->irq);

	return len;
}

static ssize_t mit_lpwg_show(struct i2c_client *client, char *buf)
{
	struct mit_data *ts = (struct mit_data *) get_touch_handle_(client);
	int len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "LPWG Mode : %X (0:None, 1:Knock-On, 10, Knock-Code) \n", ts->pdata->lpwg_mode);

	return len;
}

static ssize_t mit_lpwg_store(struct i2c_client *client, char* buf1, const char *buf2 )
{
	struct mit_data *ts = (struct mit_data *) get_touch_handle_(client);
	int mode = LPWG_NONE;
	int tap_count = 0;
	int tap_check = 0;

	/* set tap_count */
	if (buf1 != NULL && !strcmp(buf1,"tap_count")) {
		sscanf(buf2, "%d" ,&tap_count);
		ts->pdata->tap_count = tap_count;
		TOUCH_INFO_MSG("Set Touch Tap Count  = %d \n", ts->pdata->tap_count);
		return 0;
	}

	/* set active area */
	if (buf1 != NULL && !strcmp(buf1,"area")) {
		ts->pdata->active_area_x1 = ts->pdata->active_area_gap;
		ts->pdata->active_area_x2 = ts->pdata->caps->lcd_x - ts->pdata->active_area_gap;
		ts->pdata->active_area_y1 = ts->pdata->active_area_gap;
		ts->pdata->active_area_y2 = ts->pdata->caps->lcd_y - ts->pdata->active_area_gap;
		TOUCH_DEBUG_MSG("Active Area - X1:%d, X2:%d, Y1:%d, Y2:%d\n",ts->pdata->active_area_x1, ts->pdata->active_area_x2, ts->pdata->active_area_y1, ts->pdata->active_area_y2);
		return 0;
	}

	/* set double tap check */
	if (buf1 != NULL && !strcmp(buf1,"tap_check")) {
		sscanf(buf2, "%d" ,&tap_check);
		ts->pdata->double_tap_check = tap_check;
		TOUCH_INFO_MSG("Double Tap Check  = %d \n", ts->pdata->double_tap_check);
		return 0;
	}

	if (buf1 != NULL && !strcmp(buf1,"update_all")) {
		/* set lpwg mode */
		if (buf2 == NULL) {
			TOUCH_INFO_MSG(" mode is NULL, Can't not set LPWG\n");
			return 0;
		}
		sscanf(buf2, "%X", &mode);
		ts->pdata->lpwg_mode = (u8)mode;

		/* Proximity Sensor on/off */
		mutex_lock(&ts->proximity_lock);
		if (ts->pdata->panel_on == 0 && ts->pdata->lpwg_panel_on == 0) {
			TOUCH_INFO_MSG("SUSPEND AND SET\n");
			if (!ts->pdata->lpwg_mode && !ts->pdata->lpwg_prox) {
				touch_disable_wake(ts->client->irq);
				touch_disable(ts->client->irq);

				ts->enable_sensing = 0;
                                mip_lpwg_start(ts->client);
                                mip_lpwg_enable_sensing(client, LPWG_DISABLE_SENSING);


				if (wake_lock_active(&touch_wake_lock))
					wake_unlock(&touch_wake_lock);
				mit_power_ctrl(client, ts_role->suspend_pwr);
				atomic_set(&dev_state,DEV_SUSPEND);
				ts->pdata->lpwg_mode_old = LPWG_NONE;
				TOUCH_INFO_MSG(" Proxi-status is [Near] / lpwg_mode_old is [%d]\n",ts->pdata->lpwg_mode_old);
				TOUCH_INFO_MSG("SUSPEND AND SET power off\n");
#if defined(TOUCH_USE_DSV)
				if (ts_pdata->enable_sensor_interlock) {
					ts->pdata->use_dsv = 0;
					mit_dsv_control(ts->client);
				}
#endif
			} else {
				mit_power_ctrl(client, ts_role->resume_pwr);
				wake_lock_timeout(&touch_wake_lock, msecs_to_jiffies(1000));
				ts->enable_sensing = 1;
				mit_ic_ctrl(client, IC_CTRL_LPWG, (u32)&(ts->pdata->lpwg_mode));

				atomic_set(&dev_state,DEV_RESUME_ENABLE);
				touch_enable(ts->client->irq);
				touch_enable_wake(ts->client->irq);
				TOUCH_INFO_MSG("SUSPEND AND SET power on\n");
			}
		} else {
			TOUCH_INFO_MSG("PANEL ON \n");
                        atomic_set(&dev_state,DEV_RESUME_ENABLE);
                        touch_enable(ts->client->irq);
                        touch_disable_wake(ts->client->irq);
		}

	       mutex_unlock(&ts->proximity_lock);
	}
	TOUCH_INFO_MSG("%s %X \n", __func__, ts->pdata->lpwg_mode);

	return 0;
}

static ssize_t mit_keyguard_info_store(struct i2c_client *client, const char *buf )
{
#if defined(TOUCH_USE_DSV)
	struct mit_data *ts = (struct mit_data *) get_touch_handle_(client);
#endif
	int value;

	if (sscanf(buf, "%d", &value) != 1)
		return -EINVAL;

	switch(value) {
		case KEYGUARD_RESERVED:
			lockscreen_stat = 0;
			TOUCH_INFO_MSG("%s : Lockscreen unlocked, lockscreen_stat = %d\n", __func__, lockscreen_stat);

#if defined(TOUCH_USE_DSV)
			ts->pdata->sensor_value = 0;
			if (ts->pdata->enable_sensor_interlock) {
				ts->pdata->use_dsv = 0;
			}
#endif
			break;
		case KEYGUARD_ENABLE:
			lockscreen_stat = 1;
			TOUCH_INFO_MSG("%s : Lockscreen locked, lockscreen_stat = %d\n", __func__, lockscreen_stat);
			break;
		default:
			break;
	}

	return 0;
}


static int mit_sysfs(struct i2c_client *client, char *buf1, const char *buf2, u32 code)
{
	int ret = 0;
	struct ic_ctrl_param param;
	struct mit_data *ts = (struct mit_data *) get_touch_handle_(client);
	int type = 0;
	int onoff = 0;

	TOUCH_TRACE_FUNC();

	if (code != SYSFS_TESTMODE_VERSION_SHOW && code != SYSFS_KEYGUARD_STORE) {
		power_lock(POWER_SYSFS_LOCK);
		mit_power(client, POWER_ON);
		msleep(ts->pdata->role->reset_delay);
	}

	switch (code) {
		case SYSFS_VERSION_SHOW :
			param.v1 = (u32) buf1;
			ret = mit_ic_ctrl(client, IC_CTRL_INFO_SHOW, (u32) &param);
			break;
		case SYSFS_FIRMWARE_SHOW :
			param.v1 = (u32) buf1;
			ret = mit_ic_ctrl(client, IC_CTRL_FIRMWARE_VERSION_SHOW, (u32) &param);
			break;
		case SYSFS_TESTMODE_VERSION_SHOW :
			param.v1 = (u32) buf1;
			ret = mit_ic_ctrl(client, IC_CTRL_TESTMODE_VERSION_SHOW, (u32) &param);
			break;
		case SYSFS_REG_CONTROL_STORE:
			ret = mit_reg_control_store(client, buf2);
			break;
		case SYSFS_LPWG_TCI_STORE:
			ret = mit_tci_store(client, buf2);
			break;
		case SYSFS_CHSTATUS_SHOW:
			touch_disable(ts->client->irq);
			ret = mit_chstatus_show(client, buf1);
			touch_enable(ts->client->irq);
			break;
		case SYSFS_CHSTATUS_STORE:
			touch_disable(ts->client->irq);
			ret = mit_chstatus_store(client, buf2);
			touch_enable(ts->client->irq);
			break;
		case SYSFS_RAWDATA_SHOW:
			touch_disable(ts->client->irq);
			ret = mit_rawdata_show(client, buf1);
			touch_enable(ts->client->irq);
			break;
		case SYSFS_RAWDATA_STORE:
			touch_disable(ts->client->irq);
			ret = mit_rawdata_store(client, buf2);
			touch_enable(ts->client->irq);
			break;
		case SYSFS_DELTA_SHOW:
			ret = melfas_delta_show(client, buf1);
			break;
		case SYSFS_SELF_DIAGNOSTIC_SHOW:
			touch_disable(ts->client->irq);
			ret = mit_self_diagnostic_show(client, buf1);
			touch_enable(ts->client->irq);
			break;
		case SYSFS_SENSING_ALL_BLOCK_CONTROL :
			sscanf(buf1, "%d", &onoff);
			type = 0;
			ret = mit_sensing_block_control(client, (u8)type, (u8)onoff);
			break;
		case SYSFS_SENSING_BLOCK_CONTROL :
			sscanf(buf1, "%d", &onoff);
			sscanf(buf2, "%d", &type);
			ret = mit_sensing_block_control(client, (u8)type, (u8)onoff);
			break;
		case SYSFS_FW_DUMP :
			ret = mit_fw_dump_show(client, buf1);
			break;
		case SYSFS_LPWG_SHOW :
			ret = mit_lpwg_show(client, buf1);
			break;
		case SYSFS_LPWG_STORE :
			ret = mit_lpwg_store(client, buf1 ,buf2);
			break;
		case SYSFS_KEYGUARD_STORE :
			ret = mit_keyguard_info_store(client, buf2);
			break;
		case SYSFS_LPWG_DEBUG_STORE:
			tci_control(ts, LPWG_PANEL_DEBUG_CTRL, ts->pdata->lpwg_debug_enable);
			break;
		case SYSFS_LPWG_REASON_STORE:
			tci_control(ts, LPWG_FAIL_REASON_CTRL, ts->pdata->lpwg_fail_reason);
			break;
		case SYSFS_LPWG_LCD_STATUS_SHOW:
			mit_get_lpwg_lcd_status(ts);
			break;
	}

	if (code != SYSFS_TESTMODE_VERSION_SHOW && code != SYSFS_KEYGUARD_STORE) {
		power_unlock(POWER_SYSFS_LOCK);
	}
	return ret;
}

struct touch_device_driver mit_driver = {
	.probe = mit_probe,
	.remove = mit_remove,
	.init = mit_init,
	.data = mit_get_data,
	.power = mit_power,
	.fw_upgrade = mit_fw_upgrade,
	.ic_ctrl = mit_ic_ctrl,
	.sysfs = mit_sysfs,
#if defined(TOUCH_USE_DSV)
	.dsv_control = mit_dsv_control,
#endif
};

static void async_touch_init(void *data, async_cookie_t cookie)
{
	touch_driver_register_(&mit_driver);
	return;
}

static int __init touch_init(void)
{
	TOUCH_TRACE_FUNC();
	async_schedule(async_touch_init, NULL);

	return 0;
}

static void __exit touch_exit(void)
{
	TOUCH_TRACE_FUNC();
	touch_driver_unregister_();
}

module_init(touch_init);
module_exit(touch_exit);

MODULE_AUTHOR("WX-BSP-TS@lge.com");
MODULE_DESCRIPTION("LGE Touch Driver");
MODULE_LICENSE("GPL");

