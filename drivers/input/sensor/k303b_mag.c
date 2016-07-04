/******************** (C) COPYRIGHT 2013 STMicroelectronics ********************
*
* File Name          : k303b_mag.c
* Authors            : MSH - C&I BU - Application Team
*		     : Matteo Dameno (matteo.dameno@st.com)
*		     : Denis Ciocca (denis.ciocca@st.com)
*		     : Both authors are willing to be considered the contact
*		     : and update points for the driver.
* Version            : V.1.0.6_ST
* Date               : 2014/Jun/18
* Description        : K303B magnetometer driver
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
********************************************************************************
*******************************************************************************/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>

#include "k303b.h"
#include "lge_log.h"

#define SENSOR_TAG				"[LGE_Accelerometer]"
#define LGE_MAGNETOMETER_NAME	"lge_magnetometer"

#define	I2C_AUTO_INCREMENT	(0x80)
#define MS_TO_NS(x)		(x*1000000L)

#define	MAG_G_MAX_POS		983520	/** max positive value mag [ugauss] */
#define	MAG_G_MAX_NEG		983040	/** max negative value mag [ugauss] */

#define FUZZ			0
#define FLAT			0

/* Address registers */
#define REG_WHOAMI_ADDR		(0x0F)	/** Who am i address register */
#define REG_CNTRL1_ADDR		(0x20)	/** CNTRL1 address register */
#define REG_CNTRL2_ADDR		(0x21)	/** CNTRL2 address register */
#define REG_CNTRL3_ADDR		(0x22)	/** CNTRL3 address register */
#define REG_CNTRL4_ADDR		(0x23)	/** CNTRL4 address register */
#define REG_CNTRL5_ADDR		(0x24)	/** CNTRL5 address register */

#define REG_MAG_DATA_ADDR	(0x28)	/** Mag. data low address register */

/* Sensitivity */
#define SENSITIVITY_MAG_4G	146156	/**	ngauss/LSB	*/
#define SENSITIVITY_MAG_8G	292312	/**	ngauss/LSB	*/
#define SENSITIVITY_MAG_10G	365364	/**	ngauss/LSB	*/
#define SENSITIVITY_MAG_16G	584454	/**	ngauss/LSB	*/

/* ODR */
#define ODR_MAG_MASK		(0X1C)	/* Mask for odr change on mag */
#define K303B_MAG_ODR0_625	(0x00)	/* 0.625Hz output data rate */
#define K303B_MAG_ODR1_25	(0x04)	/* 1.25Hz output data rate */
#define K303B_MAG_ODR2_5	(0x08)	/* 2.5Hz output data rate */
#define K303B_MAG_ODR5	(0x0C)	/* 5Hz output data rate */
#define K303B_MAG_ODR10	(0x10)	/* 10Hz output data rate */
#define K303B_MAG_ODR20	(0x14)	/* 20Hz output data rate */
#define K303B_MAG_ODR40	(0x18)	/* 40Hz output data rate */
#define K303B_MAG_ODR80	(0x1C)	/* 80Hz output data rate */

/* Magnetic sensor mode */
#define MSMS_MASK		(0x03)	/* Mask magnetic sensor mode */
#define POWEROFF_MAG		(0x02)	/* Power Down */
#define CONTINUOS_CONVERSION	(0x00)	/* Continuos Conversion */

/* X and Y axis operative mode selection */
#define X_Y_PERFORMANCE_MASK		(0x60)
#define X_Y_LOW_PERFORMANCE		(0x00)
#define X_Y_MEDIUM_PERFORMANCE		(0x20)
#define X_Y_HIGH_PERFORMANCE		(0x40)
#define X_Y_ULTRA_HIGH_PERFORMANCE	(0x60)

/* Z axis operative mode selection */
#define Z_PERFORMANCE_MASK		(0x0c)
#define Z_LOW_PERFORMANCE		(0x00)
#define Z_MEDIUM_PERFORMANCE		(0x04)
#define Z_HIGH_PERFORMANCE		(0x08)
#define Z_ULTRA_HIGH_PERFORMANCE	(0x0c)

/* Default values loaded in probe function */
#define WHOIAM_VALUE		(0x3d)	/** Who Am I default value */
#define REG_DEF_CNTRL1		(0xE0)	/** CNTRL1 default value */
#define REG_DEF_CNTRL2		(0x60)	/** CNTRL2 default value */
#define REG_DEF_CNTRL3		(0x03)	/** CNTRL3 default value */
#define REG_DEF_CNTRL4		(0x0C)	/** CNTRL4 default value */
#define REG_DEF_CNTRL5		(0x40)	/** CNTRL5 default value */

#define REG_DEF_ALL_ZEROS	(0x00)

struct workqueue_struct *k303b_workqueue;


struct {
	unsigned int cutoff_us;
	u8 value;
} k303b_mag_odr_table[] = {
		{  12, K303B_MAG_ODR80  },
		{  25, K303B_MAG_ODR40   },
		{  50, K303B_MAG_ODR20   },
		{  100, K303B_MAG_ODR10 },
		{ 200, K303B_MAG_ODR5 },
		{ 400, K303B_MAG_ODR2_5},
		{ 800, K303B_MAG_ODR1_25},
		{ 1600, K303B_MAG_ODR0_625},
};

struct interrupt_enable {
	atomic_t enable;
	u8 address;
	u8 mask;
};

struct interrupt_value {
	int value;
	u8 address;
};


struct k303b_status {
	struct i2c_client *client;
	struct k303b_mag_platform_data *pdata_mag;

	struct mutex lock;
	struct work_struct input_work_mag;

	struct hrtimer hr_timer_mag;
	ktime_t ktime_mag;

	struct input_dev *input_dev_mag;

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;

	atomic_t enabled_mag;

	int on_before_suspend;
	int use_smbus;

	u32 sensitivity_mag;

	u8 xy_mode;
	u8 z_mode;
};

static struct k303b_mag_platform_data default_k303b_mag_pdata = {
	.poll_interval = 100,
	.min_interval = K303B_MAG_MIN_POLL_PERIOD_MS,
	.fs_range = K303B_MAG_FS_16G,
	.axis_map_x = 1,
	.axis_map_y = 0,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 1,
	.negate_z = 1,
};

struct reg_rw {
	u8 address;
	u8 default_value;
	u8 resume_value;
};

struct reg_r {
	u8 address;
	u8 value;
};

static struct status_registers {
	struct reg_r who_am_i;
	struct reg_rw cntrl1;
	struct reg_rw cntrl2;
	struct reg_rw cntrl3;
	struct reg_rw cntrl4;
	struct reg_rw cntrl5;
} status_registers = {
	.who_am_i.address = REG_WHOAMI_ADDR,
	.who_am_i.value = WHOIAM_VALUE,
	.cntrl1.address = REG_CNTRL1_ADDR,
	.cntrl1.default_value = REG_DEF_CNTRL1,
	.cntrl2.address = REG_CNTRL2_ADDR,
	.cntrl2.default_value = REG_DEF_CNTRL2,
	.cntrl3.address = REG_CNTRL3_ADDR,
	.cntrl3.default_value = REG_DEF_CNTRL3,
	.cntrl4.address = REG_CNTRL4_ADDR,
	.cntrl4.default_value = REG_DEF_CNTRL4,
	.cntrl5.address = REG_CNTRL5_ADDR,
	.cntrl5.default_value = REG_DEF_CNTRL5,
};

static int k303b_i2c_read(struct k303b_status *stat, u8 *buf, int len)
{
	int ret;
	u8 reg = buf[0];
	u8 cmd = reg;


	if (len > 1)
		cmd = (I2C_AUTO_INCREMENT | reg);
	if (stat->use_smbus) {
		if (len == 1) {
			ret = i2c_smbus_read_byte_data(stat->client, cmd);
			buf[0] = ret & 0xff;
#ifdef DEBUG
			dev_warn(&stat->client->dev,
				"i2c_smbus_read_byte_data: ret=0x%02x, len:%d ,"
				"command=0x%02x, buf[0]=0x%02x\n",
				ret, len, cmd , buf[0]);
#endif
		} else if (len > 1) {
			ret = i2c_smbus_read_i2c_block_data(stat->client,
								cmd, len, buf);
		} else
			ret = -1;

		if (ret < 0) {
			SENSOR_ERR("read transfer error: len:%d, command=0x%02x",
				len, cmd);
			return 0;
		}
		return len;
	}

	ret = i2c_master_send(stat->client, &cmd, sizeof(cmd));
	if (ret != sizeof(cmd))
		return ret;

	return i2c_master_recv(stat->client, buf, len);
}

static int k303b_i2c_write(struct k303b_status *stat, u8 *buf, int len)
{
	int ret;
	u8 reg, value;

	if (len > 1)
		buf[0] = (I2C_AUTO_INCREMENT | buf[0]);

	reg = buf[0];
	value = buf[1];

	if (stat->use_smbus) {
		if (len == 1) {
			ret = i2c_smbus_write_byte_data(stat->client,
								reg, value);
#ifdef DEBUG
			dev_warn(&stat->client->dev,
				"i2c_smbus_write_byte_data: ret=%d, len:%d, "
				"command=0x%02x, value=0x%02x\n",
				ret, len, reg , value);
#endif
			return ret;
		} else if (len > 1) {
			ret = i2c_smbus_write_i2c_block_data(stat->client,
							reg, len, buf + 1);
			return ret;
		}
	}

	ret = i2c_master_send(stat->client, buf, len+1);
	return (ret == len+1) ? 0 : ret;
}

static int k303b_hw_init(struct k303b_status *stat)
{
	int err = -1;
	u8 buf[6];

	SENSOR_FUN();

	buf[0] = status_registers.who_am_i.address;
	err = k303b_i2c_read(stat, buf, 1);

	if (err < 0) {
		SENSOR_ERR("Error reading WHO_AM_I: is device available/working?");
		goto err_firstread;
	} else
		stat->hw_working = 1;

	if (buf[0] != status_registers.who_am_i.value) {
		SENSOR_ERR("device unknown. Expected: 0x%02x, Replies: 0x%02x",
				status_registers.who_am_i.value, buf[0]);
		err = -1;
		goto err_unknown_device;
	}

	status_registers.cntrl1.resume_value =
					status_registers.cntrl1.default_value;
	status_registers.cntrl2.resume_value =
					status_registers.cntrl2.default_value;
	status_registers.cntrl3.resume_value =
					status_registers.cntrl3.default_value;
	status_registers.cntrl4.resume_value =
					status_registers.cntrl4.default_value;
	status_registers.cntrl5.resume_value =
					status_registers.cntrl5.default_value;

	buf[0] = status_registers.cntrl1.address;
	buf[1] = status_registers.cntrl1.default_value;
	buf[2] = status_registers.cntrl2.default_value;
	buf[3] = status_registers.cntrl3.default_value;
	buf[4] = status_registers.cntrl4.default_value;
	buf[5] = status_registers.cntrl5.default_value;

	err = k303b_i2c_write(stat, buf, 5);
	if (err < 0) {
		SENSOR_ERR("Error initializing CLTR_REG registers");
		goto err_reginit;
	}

	stat->xy_mode = X_Y_ULTRA_HIGH_PERFORMANCE;
	stat->z_mode = Z_ULTRA_HIGH_PERFORMANCE;
	stat->hw_initialized = 1;
	SENSOR_LOG("hw init done");

	return 0;

err_reginit:
err_unknown_device:
err_firstread:
	stat->hw_working = 0;
	stat->hw_initialized = 0;
	return err;
}

static int k303b_mag_device_power_off(struct k303b_status *stat)
{
	int err;
	u8 buf[2];

	buf[0] = status_registers.cntrl3.address;
	buf[1] = ((MSMS_MASK & POWEROFF_MAG) |
		((~MSMS_MASK) & status_registers.cntrl3.resume_value));

	err = k303b_i2c_write(stat, buf, 1);
	if (err < 0)
		SENSOR_ERR("magnetometer soft power off failed: %d", err);

	if (stat->pdata_mag->power_off)
		stat->pdata_mag->power_off(stat->pdata_mag);

	atomic_set(&stat->enabled_mag, 0);

	return 0;
}

static int k303b_mag_device_power_on(struct k303b_status *stat)
{
	int err = -1;
	u8 buf[6];

	if (stat->pdata_mag->power_on) {
		err = stat->pdata_mag->power_on(stat->pdata_mag);
		if (err < 0) {
			SENSOR_ERR("magnetometer power_on failed: %d", err);
			return err;
		}
	}

	buf[0] = status_registers.cntrl1.address;
	buf[1] = status_registers.cntrl1.resume_value;
	err = k303b_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.cntrl3.address;
	buf[1] = ((MSMS_MASK & CONTINUOS_CONVERSION) |
		((~MSMS_MASK) & status_registers.cntrl3.resume_value));


	err = k303b_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	atomic_set(&stat->enabled_mag, 1);

	return 0;

err_resume_state:
	atomic_set(&stat->enabled_mag, 0);
	SENSOR_ERR("magnetometer hw power on error 0x%02x,0x%02x: %d",
							buf[0], buf[1], err);
	return err;
}

static int k303b_mag_update_fs_range(struct k303b_status *stat,
								u8 new_fs_range)
{
	int err = -1;
	u32 sensitivity;
	u8 updated_val;
	u8 buf[2];

	switch (new_fs_range) {
	case K303B_MAG_FS_4G:
		sensitivity = SENSITIVITY_MAG_4G;
		break;
	case K303B_MAG_FS_8G:
		sensitivity = SENSITIVITY_MAG_8G;
		break;
	case K303B_MAG_FS_10G:
		sensitivity = SENSITIVITY_MAG_10G;
		break;
	case K303B_MAG_FS_16G:
		sensitivity = SENSITIVITY_MAG_16G;
		break;
	default:
		SENSOR_ERR("invalid magnetometer fs range requested: %u",
								new_fs_range);
		return -EINVAL;
	}

	buf[0] = status_registers.cntrl2.address;
	err = k303b_i2c_read(stat, buf, 1);
	if (err < 0)
		goto error;

	status_registers.cntrl2.resume_value = buf[0];
	updated_val = (K303B_MAG_FS_MASK & new_fs_range);
	buf[1] = updated_val;
	buf[0] = status_registers.cntrl2.address;

	err = k303b_i2c_write(stat, buf, 1);
	if (err < 0)
		goto error;
	status_registers.cntrl2.resume_value = updated_val;
	stat->sensitivity_mag = sensitivity;

	return err;

error:
	SENSOR_ERR("update magnetometer fs range failed 0x%02x,0x%02x: %d",
							buf[0], buf[1], err);
	return err;
}

static int k303b_mag_update_odr(struct k303b_status *stat,
						unsigned int poll_interval_ms)
{
	int err = -1;
	u8 config[2];
	int i;

	for (i = ARRAY_SIZE(k303b_mag_odr_table) - 1; i >= 0; i--) {
		if ((k303b_mag_odr_table[i].cutoff_us <= poll_interval_ms)
								|| (i == 0))
			break;
	}

	config[1] = ((ODR_MAG_MASK & k303b_mag_odr_table[i].value) |
		((~ODR_MAG_MASK) & status_registers.cntrl1.resume_value));

	if (atomic_read(&stat->enabled_mag)) {
		config[0] = status_registers.cntrl1.address;
		err = k303b_i2c_write(stat, config, 1);
		if (err < 0)
			goto error;
	}
	status_registers.cntrl1.resume_value = config[1];
	stat->ktime_mag = ktime_set(0, MS_TO_NS(poll_interval_ms));
	return err;

error:
	SENSOR_ERR("update magnetometer odr failed 0x%02x,0x%02x: %d",
						config[0], config[1], err);

	return err;
}

static int k303b_mag_update_operative_mode(struct k303b_status *stat,
							int axis, u8 value)
{
	int err = -1;
	u8 config[2];
	u8 mask;
	u8 addr;

	if (axis == 0) {
		config[0] = REG_CNTRL1_ADDR;
		mask = X_Y_PERFORMANCE_MASK;
		addr = REG_CNTRL1_ADDR;
	} else {
		config[0] = REG_CNTRL4_ADDR;
		mask = Z_PERFORMANCE_MASK;
		addr = REG_CNTRL4_ADDR;
	}
	err = k303b_i2c_read(stat, config, 1);
	if (err < 0)
		goto error;
	config[1] = ((mask & value) |
		((~mask) & config[0]));

	config[0] = addr;
	err = k303b_i2c_write(stat, config, 1);
	if (err < 0)
		goto error;
	if (axis == 0)
		stat->xy_mode = value;
	else
		stat->z_mode = value;

	return err;

error:
	SENSOR_ERR("update operative mode failed 0x%02x,0x%02x: %d",
						config[0], config[1], err);

	return err;
}

static int k303b_validate_polling(unsigned int *min_interval,
					unsigned int *poll_interval,
					unsigned int min, u8 *axis_map_x,
					u8 *axis_map_y, u8 *axis_map_z,
					struct i2c_client *client)
{
	*min_interval = max(min, *min_interval);
	*poll_interval = max(*poll_interval, *min_interval);

	if (*axis_map_x > 2 || *axis_map_y > 2 || *axis_map_z > 2) {
		SENSOR_ERR("invalid axis_map value x:%u y:%u z%u",
					*axis_map_x, *axis_map_y, *axis_map_z);
		return -EINVAL;
	}

	return 0;
}

static int k303b_validate_negate(u8 *negate_x, u8 *negate_y, u8 *negate_z,
						struct i2c_client *client)
{
	if (*negate_x > 1 || *negate_y > 1 || *negate_z > 1) {
		SENSOR_ERR("invalid negate value x:%u y:%u z:%u",
					*negate_x, *negate_y, *negate_z);
		return -EINVAL;
	}
	return 0;
}

static int k303b_mag_validate_pdata(struct k303b_status *stat)
{
	int res = -1;

	res = k303b_validate_polling(&stat->pdata_mag->min_interval,
				&stat->pdata_mag->poll_interval,
				(unsigned int)K303B_MAG_MIN_POLL_PERIOD_MS,
				&stat->pdata_mag->axis_map_x,
				&stat->pdata_mag->axis_map_y,
				&stat->pdata_mag->axis_map_z,
				stat->client);
	if (res < 0)
		return -EINVAL;

	res = k303b_validate_negate(&stat->pdata_mag->negate_x,
				&stat->pdata_mag->negate_y,
				&stat->pdata_mag->negate_z,
				stat->client);
	if (res < 0)
		return -EINVAL;

	return 0;
}

static int k303b_mag_enable(struct k303b_status *stat)
{
	int err;

	if (!atomic_cmpxchg(&stat->enabled_mag, 0, 1)) {
		err = k303b_mag_device_power_on(stat);
		if (err < 0) {
			atomic_set(&stat->enabled_mag, 0);
			return err;
		}
		hrtimer_start(&stat->hr_timer_mag,
					stat->ktime_mag, HRTIMER_MODE_REL);
	}

	return 0;
}

static int k303b_mag_disable(struct k303b_status *stat)
{
	if (atomic_cmpxchg(&stat->enabled_mag, 1, 0)) {
		cancel_work_sync(&stat->input_work_mag);
		hrtimer_cancel(&stat->hr_timer_mag);
		k303b_mag_device_power_off(stat);
	}

	return 0;
}

static int k303b_mag_get_data(struct k303b_status *stat, int *xyz)
{
	int err = -1;
	u8 mag_data[6];
	s32 hw_d[3] = { 0 };

	mag_data[0] = (REG_MAG_DATA_ADDR);
	err = k303b_i2c_read(stat, mag_data, 6);
	if (err < 0)
		return err;

	hw_d[0] = (s16)((mag_data[1] << 8) | mag_data[0]);
	hw_d[1] = (s16)((mag_data[3] << 8) | mag_data[2]);
	hw_d[2] = (s16)((mag_data[5] << 8) | mag_data[4]);

#ifdef DEBUG
	pr_debug("%s read x=0x%02x 0x%02x (regH regL), x=%d (dec) [LSB]\n",
		K303B_MAG_DEV_NAME, mag_data[1], mag_data[0], hw_d[0]);
	pr_debug("%s read y=0x%02x 0x%02x (regH regL), y=%d (dec) [LSB]\n",
		K303B_MAG_DEV_NAME, mag_data[3], mag_data[2], hw_d[1]);
	pr_debug("%s read z=0x%02x 0x%02x (regH regL), z=%d (dec) [LSB]\n",
		K303B_MAG_DEV_NAME, mag_data[5], mag_data[4], hw_d[2]);
#endif

	//printk(KERN_WARNING "k303b_mag2 = %d,%d,%d\n", hw_d[0], hw_d[1], hw_d[2]);

/*
	hw_d[0] = hw_d[0] * stat->sensitivity_mag;
	hw_d[1] = hw_d[1] * stat->sensitivity_mag;
	hw_d[2] = hw_d[2] * stat->sensitivity_mag;
*/
	xyz[0] = ((stat->pdata_mag->negate_x) ?
				(-hw_d[stat->pdata_mag->axis_map_x])
					: (hw_d[stat->pdata_mag->axis_map_x]));
	xyz[1] = ((stat->pdata_mag->negate_y) ?
				(-hw_d[stat->pdata_mag->axis_map_y])
					: (hw_d[stat->pdata_mag->axis_map_y]));
	xyz[2] = ((stat->pdata_mag->negate_z) ?
				(-hw_d[stat->pdata_mag->axis_map_z])
					: (hw_d[stat->pdata_mag->axis_map_z]));

	return err;
}

static void k303b_mag_input_cleanup(struct k303b_status *stat)
{
	input_unregister_device(stat->input_dev_mag);
	input_free_device(stat->input_dev_mag);
}

static ssize_t attr_get_polling_rate_mag(struct device *dev,
						struct device_attribute *attr,
		char *buf)
{
	unsigned int val;
	struct k303b_status *stat = dev_get_drvdata(dev);

	mutex_lock(&stat->lock);
	val = stat->pdata_mag->poll_interval;
	mutex_unlock(&stat->lock);
	return sprintf(buf, "%u\n", val);
}

static ssize_t attr_set_polling_rate_mag(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct k303b_status *stat = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (kstrtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;
	interval_ms = max_t(unsigned int, (unsigned int)interval_ms,
						stat->pdata_mag->min_interval);
	mutex_lock(&stat->lock);
	stat->pdata_mag->poll_interval = (unsigned int)interval_ms;
	k303b_mag_update_odr(stat, interval_ms);
	mutex_unlock(&stat->lock);
	return size;
}

static ssize_t attr_get_enable_mag(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct k303b_status *stat = dev_get_drvdata(dev);
	int val = (int)atomic_read(&stat->enabled_mag);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable_mag(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct k303b_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		k303b_mag_enable(stat);
	else
		k303b_mag_disable(stat);

	return size;
}

static ssize_t attr_get_range_mag(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u8 val;
	int range = 2;
	struct k303b_status *stat = dev_get_drvdata(dev);

	mutex_lock(&stat->lock);
	val = stat->pdata_mag->fs_range;
	switch (val) {
	case K303B_MAG_FS_4G:
		range = 4;
		break;
	case K303B_MAG_FS_8G:
		range = 8;
		break;
	case K303B_MAG_FS_10G:
		range = 10;
		break;
	case K303B_MAG_FS_16G:
		range = 16;
		break;
	}
	mutex_unlock(&stat->lock);

	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range_mag(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct k303b_status *stat = dev_get_drvdata(dev);
	unsigned long val;
	u8 range;
	int err;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;
	switch (val) {
	case 4:
		range = K303B_MAG_FS_4G;
		break;
	case 8:
		range = K303B_MAG_FS_8G;
		break;
	case 10:
		range = K303B_MAG_FS_10G;
		break;
	case 16:
		range = K303B_MAG_FS_16G;
		break;
	default:
		SENSOR_ERR("magnetometer invalid range request: %lu, discarded", val);
		return -EINVAL;
	}
	mutex_lock(&stat->lock);
	err = k303b_mag_update_fs_range(stat, range);
	if (err < 0) {
		mutex_unlock(&stat->lock);
		return err;
	}
	stat->pdata_mag->fs_range = range;
	mutex_unlock(&stat->lock);
	SENSOR_LOG("magnetometer range set to: %lu g", val);

	return size;
}

static ssize_t attr_get_sensor_data(struct device *dev, 
                struct device_attribute *attr, char *buf)
{
    int xyz[3] = { 0, };
    int err = 0;
    struct k303b_status *stat = dev_get_drvdata(dev);
    err = k303b_mag_get_data(stat, xyz);
    if (err < 0)
        return sprintf(buf, "Read Data failed");
    
    return sprintf(buf, "Read Data x=%d y=%d z=%d \n",
            xyz[0], xyz[1], xyz[2]);
}

static ssize_t attr_get_xy_mode(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u8 val;
	char mode[13];
	struct k303b_status *stat = dev_get_drvdata(dev);

	mutex_lock(&stat->lock);
	val = stat->xy_mode;
	switch (val) {
	case X_Y_HIGH_PERFORMANCE:
		strcpy(&(mode[0]), "high");
		break;
	case X_Y_LOW_PERFORMANCE:
		strcpy(&(mode[0]), "low");
		break;
	case X_Y_MEDIUM_PERFORMANCE:
		strcpy(&(mode[0]), "medium");
		break;
	case X_Y_ULTRA_HIGH_PERFORMANCE:
		strcpy(&(mode[0]), "ultra_high");
		break;
	}
	mutex_unlock(&stat->lock);
	return sprintf(buf, "%s\n", mode);
}

static ssize_t attr_set_xy_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct k303b_status *stat = dev_get_drvdata(dev);
	u8 mode;
	int err;

	err = strncmp(buf, "high", 4);
	if (err == 0) {
		mode = X_Y_HIGH_PERFORMANCE;
		goto valid;
	}
	err = strncmp(buf, "low", 3);
	if (err == 0) {
		mode = X_Y_LOW_PERFORMANCE;
		goto valid;
	}
	err = strncmp(buf, "medium", 6);
	if (err == 0) {
		mode = X_Y_MEDIUM_PERFORMANCE;
		goto valid;
	}
	err = strncmp(buf, "ultra_high", 10);
	if (err == 0) {
		mode = X_Y_ULTRA_HIGH_PERFORMANCE;
		goto valid;
	}
	goto error;

valid:
	err = k303b_mag_update_operative_mode(stat, 0, mode);
	if (err < 0)
		goto error;

	SENSOR_LOG("magnetometer x_y op. mode set to: %s", buf);
	return size;

error:
	SENSOR_ERR("magnetometer invalid value request: %s, discarded", buf);

	return -EINVAL;
}

static ssize_t attr_get_z_mode(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u8 val;
	char mode[13];
	struct k303b_status *stat = dev_get_drvdata(dev);

	mutex_lock(&stat->lock);
	val = stat->z_mode;
	switch (val) {
	case Z_HIGH_PERFORMANCE:
		strcpy(&(mode[0]), "high");
		break;
	case Z_LOW_PERFORMANCE:
		strcpy(&(mode[0]), "low");
		break;
	case Z_MEDIUM_PERFORMANCE:
		strcpy(&(mode[0]), "medium");
		break;
	case Z_ULTRA_HIGH_PERFORMANCE:
		strcpy(&(mode[0]), "ultra_high");
		break;
	}
	mutex_unlock(&stat->lock);
	return sprintf(buf, "%s\n", mode);
}

static ssize_t attr_set_z_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct k303b_status *stat = dev_get_drvdata(dev);
	u8 mode;
	int err;

	err = strncmp(buf, "high", 4);
	if (err == 0) {
		mode = Z_HIGH_PERFORMANCE;
		goto valid;
	}
	err = strncmp(buf, "low", 3);
	if (err == 0) {
		mode = Z_LOW_PERFORMANCE;
		goto valid;
	}
	err = strncmp(buf, "medium", 6);
	if (err == 0) {
		mode = Z_MEDIUM_PERFORMANCE;
		goto valid;
	}
	err = strncmp(buf, "ultra_high", 10);
	if (err == 0) {
		mode = Z_ULTRA_HIGH_PERFORMANCE;
		goto valid;
	}
	goto error;

valid:
	err = k303b_mag_update_operative_mode(stat, 1, mode);
	if (err < 0)
		goto error;

	SENSOR_LOG("magnetometer z op. mode set to: %s", buf);
	return size;

error:
	SENSOR_ERR("magnetometer invalid value request: %s, discarded", buf);

	return -EINVAL;
}

#define MAG_SELF_TEST_16G_MAX_LSB	(-1711)
#define MAG_SELF_TEST_16G_MIN_LSB	(-5133)
#define MAG_SELF_TEST_16G_MAX_LSB_Z	(-171)
#define MAG_SELF_TEST_16G_MIN_LSB_Z	(-1711)

static ssize_t attr_get_selftest(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct k303b_status *stat = dev_get_drvdata(dev);
	int val, i, en_state = 0;
	ssize_t ret;
	u8 x[8];
	s32 NO_ST[3] = {0, 0, 0};
	s32 ST[3] = {0, 0, 0};
	s32 ST_MIN_LSB, ST_MAX_LSB;

	en_state = atomic_read(&stat->enabled_mag);
	k303b_mag_disable(stat);

	k303b_mag_device_power_on(stat);

	x[0] = REG_CNTRL1_ADDR;
	x[1] = 0x1c;
	x[2] = 0x60;
	k303b_i2c_write(stat, x, 2);

	mdelay(20);

	x[0] = REG_CNTRL3_ADDR;
	x[1] = 0x00;
	k303b_i2c_write(stat, x, 1);

	mdelay(20);

	x[0] = REG_MAG_DATA_ADDR;
	k303b_i2c_read(stat, x, 6);

	for (i = 0; i < 6; i++) {
		while (1) {
			x[0] = 0x27;
			val = k303b_i2c_read(stat, x, 1);
			if (val < 0) {
				ret = sprintf(buf, "I2C fail. (%d)\n", val);
				goto ST_EXIT;
			}
			if (x[0] & 0x08)
				break;
		}
		x[0] = REG_MAG_DATA_ADDR;
		k303b_i2c_read(stat, x, 6);
		if (i > 0) {
			NO_ST[0] += (s16)((x[1] << 8) | x[0]);
			NO_ST[1] += (s16)((x[3] << 8) | x[2]);
			NO_ST[2] += (s16)((x[5] << 8) | x[4]);
		}
	}
	NO_ST[0] /= 5;
	NO_ST[1] /= 5;
	NO_ST[2] /= 5;

	x[0] = REG_CNTRL1_ADDR;
	x[1] = 0x1d;
	k303b_i2c_write(stat, x, 1);

	mdelay(60);

	x[0] = REG_MAG_DATA_ADDR;
	k303b_i2c_read(stat, x, 6);

	for (i = 0; i < 6; i++) {
		while (1) {
			x[0] = 0x27;
			val = k303b_i2c_read(stat, x, 1);
			if (val < 0) {
				ret = sprintf(buf, "I2C fail. (%d)\n", val);
				goto ST_EXIT;
			}
			if (x[0] & 0x08)
				break;
		}
		x[0] = REG_MAG_DATA_ADDR;
		k303b_i2c_read(stat, x, 6);
		if (i > 0) {
			ST[0] += (s16)((x[1] << 8) | x[0]);
			ST[1] += (s16)((x[3] << 8) | x[2]);
			ST[2] += (s16)((x[5] << 8) | x[4]);
		}
	}
	ST[0] /= 5;
	ST[1] /= 5;
	ST[2] /= 5;

	for (val = 1, i = 0; i < 3; i++) {
		ST[i] -= NO_ST[i];

		switch (i) {
		case 0:
		case 1:
			ST_MIN_LSB = MAG_SELF_TEST_16G_MIN_LSB;
			ST_MAX_LSB = MAG_SELF_TEST_16G_MAX_LSB;
			break;
		case 2:
			ST_MIN_LSB = MAG_SELF_TEST_16G_MIN_LSB_Z;
			ST_MAX_LSB = MAG_SELF_TEST_16G_MAX_LSB_Z;
			break;
		default:
			ST_MIN_LSB = 0;
			ST_MAX_LSB = 0;
		}

		if ((ST_MIN_LSB > ST[i]) || (ST[i] > ST_MAX_LSB)) {
			SENSOR_ERR("ST[%d]: Out of range!! (%d)", i, ST[i]);
			val = 0;
		}
	}

	if (val)
		ret = sprintf(buf, "Self test: OK (%d, %d, %d)\n", ST[0], ST[1], ST[2]);
	else
		ret = sprintf(buf, "Self test: NG (%d, %d, %d)\n", ST[0], ST[1], ST[2]);

ST_EXIT:
	x[0] = REG_CNTRL1_ADDR;
	x[1] = 0x00;
	k303b_i2c_write(stat, x, 1);
	x[0] = REG_CNTRL3_ADDR;
	x[1] = 0x00;
	k303b_i2c_write(stat, x, 1);

	k303b_mag_device_power_off(stat);

	if (en_state) k303b_mag_enable(stat);

	return ret;
}


/* power on/off function*/
int k303b_mag_power(struct k303b_mag_platform_data *pdata_mag, int on)
{
	int ret=0;

	if (on) {
		//ret = regulator_enable(pdata_mag->vdd_reg);
		//ret += regulator_enable(pdata_mag->vdd_i2c);
		pr_info("k303b mag power on [status : %d ]\n", ret);
		mdelay(30);
	} else {
		//ret = regulator_disable(pdata_mag->vdd_reg);
		//ret += regulator_disable(pdata_mag->vdd_i2c);
		pr_info("k303b mag power off [status : %d ]\n", ret);
	}
	return ret;
}

int k303b_mag_power_on(struct k303b_mag_platform_data *pdata_mag)
{
	int ret;
	ret =  k303b_mag_power(pdata_mag, 1);
	return ret;
}

int k303b_mag_power_off(struct k303b_mag_platform_data *pdata_mag)
{
	int ret;
	ret =  k303b_mag_power(pdata_mag, 0);
	return ret;
}
/* LGE Add start DTS parse function */
static int k303b_parse_dt(struct device *dev, struct k303b_mag_platform_data *pdata_mag)
{
	int rc;
	int ret, err = 0;
	struct device_node *np = dev->of_node;
	struct sensor_dt_to_pdata_map *itr;
	struct sensor_dt_to_pdata_map map[] = {
		//{"st,irq-gpio",     &pdata->gpio_int1,      DT_REQUIRED,        DT_GPIO,    0},
		{"st,fs_range",        &pdata_mag->fs_range,        DT_SUGGESTED,       DT_U32,     0},
		{"st,axis_map_x",      &pdata_mag->axis_map_x,     DT_SUGGESTED,       DT_U32,     0},
		{"st,axis_map_y",      &pdata_mag->axis_map_y,     DT_SUGGESTED,       DT_U32,     0},
		{"st,axis_map_z",      &pdata_mag->axis_map_z,     DT_SUGGESTED,       DT_U32,     0},
		{"st,negate_x",        &pdata_mag->negate_x,       DT_SUGGESTED,       DT_U32,     0},
		{"st,negate_y",        &pdata_mag->negate_y,       DT_SUGGESTED,       DT_U32,     0},
		{"st,negate_z",        &pdata_mag->negate_z,       DT_SUGGESTED,       DT_U32,     0},
		{"st,poll_interval",   &pdata_mag->poll_interval,  DT_SUGGESTED,       DT_U32,     0},
		{"st,min_interval",    &pdata_mag->min_interval,   DT_SUGGESTED,       DT_U32,     0},
		{NULL,              NULL,                   0,                  0,          0},
	};
	
	for (itr = map; itr->dt_name ; ++itr) {
		switch (itr->type) {
			/*case DT_GPIO:
				ret = of_get_named_gpio(np, itr->dt_name, 0);
				if (ret >= 0) {
					*((int *) itr->ptr_data) = ret;
					ret = 0;
				}
				break;*/
			case DT_U32:
				ret = of_property_read_u32(np, itr->dt_name,
						(u32 *) itr->ptr_data);
				break;
			case DT_BOOL:
				*((bool *) itr->ptr_data) =
					of_property_read_bool(np, itr->dt_name);
				ret = 0;
				break;
			default:
				SENSOR_ERR("%d is an unknown DT entry type",itr->type);
				ret = -EBADE;
		}

		if (ret) {
			*((int *)itr->ptr_data) = itr->default_val;
			if (itr->status < DT_OPTIONAL) {
				SENSOR_LOG("Missing '%s' DT entry",itr->dt_name);
				/* cont on err to dump all missing entries */
				if (itr->status == DT_REQUIRED && !err)
					err = ret;
			}
		}
	}

	pdata_mag->vdd_reg = regulator_get(dev, "st,vdd_ana");
	if (IS_ERR(pdata_mag->vdd_reg)) {
		rc = PTR_ERR(pdata_mag->vdd_reg);
		SENSOR_ERR("Regulator get failed vdd_ana-supply rc=%d", rc);
		return rc;
	}
	pdata_mag->vdd_i2c = regulator_get(dev, "st,vddio_i2c");
	if (IS_ERR(pdata_mag->vdd_i2c)) {
		rc = PTR_ERR(pdata_mag->vdd_i2c);
		SENSOR_ERR("Regulator get failed vddio_i2c-supply rc=%d", rc);
		return rc;
	}
	rc = regulator_enable(pdata_mag->vdd_reg);
	mdelay(30);
    rc += regulator_enable(pdata_mag->vdd_i2c);
	mdelay(80);
    /* debug print disable */
	/*
	   dev_info(dev, "parse_dt data [gpio_int = %d]\n", pdata_mag->gpio_int);
	 */
return 0;
}
#if 0
static int write_bit_on_register(struct k303b_status *stat, u8 address,
					u8 *resume_value, u8 mask, int value)
{
	int err;
	u8 updated_val;
	u8 buf[2];
	u8 val = 0x00;

	buf[0] = address;
	err = k303b_i2c_read(stat, buf, 1);
	if (err < 0)
		return -1;

	if (resume_value != NULL)
		*resume_value = buf[0];

	if (mask == 0)
		updated_val = (u8)value;
	else {
		if (value > 0)
			val = 0xFF;

		updated_val = (mask & val) | ((~mask) & buf[0]);
	}

	buf[1] = updated_val;
	buf[0] = address;

	err = k303b_i2c_write(stat, buf, 1);
	if (err < 0)
		return -1;

	if (resume_value != NULL)
		*resume_value = updated_val;

	return err;
}
#endif

static DEVICE_ATTR(enable, 0666, attr_get_enable_mag, attr_set_enable_mag);
static DEVICE_ATTR(poll_delay, 0666, attr_get_polling_rate_mag, attr_set_polling_rate_mag);
static DEVICE_ATTR(range, 0666, attr_get_range_mag, attr_set_range_mag);
static DEVICE_ATTR(value, 0666, attr_get_sensor_data, NULL);
static DEVICE_ATTR(x_y_opearative_mode, 0664, attr_get_xy_mode, attr_set_xy_mode);
static DEVICE_ATTR(z_opearative_mode, 0664, attr_get_z_mode, attr_set_z_mode);
static DEVICE_ATTR(selftest, 0444, attr_get_selftest, NULL);
#ifdef DEBUG
//static DEVICE_ATTR(reg_value, 0600, attr_reg_get, attr_reg_set);
//static DEVICE_ATTR(reg_addr, 0200, NULL, attr_addr_set);
#endif

static struct attribute *k303b_mag_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_poll_delay.attr,
	&dev_attr_range.attr,
    &dev_attr_value.attr,
	&dev_attr_x_y_opearative_mode.attr,
	&dev_attr_z_opearative_mode.attr,
	&dev_attr_selftest.attr,
#ifdef DEBUG
//	&dev_attr_reg_value.attr,
//	&dev_attr_reg_addr.attr,
#endif
	NULL,
};
static const struct attribute_group k303b_mag_attr_group = {
	.attrs = k303b_mag_attributes,
};


int k303b_mag_input_open(struct input_dev *input)
{
	SENSOR_FUN();	
	return 0;//k303b_mag_enable(stat);
}

void k303b_mag_input_close(struct input_dev *dev)
{
	struct k303b_status *stat = input_get_drvdata(dev);
	SENSOR_FUN();
	k303b_mag_disable(stat);
}

static void k303b_mag_report_values(struct k303b_status *stat, int *xyz)
{
	input_report_abs(stat->input_dev_mag, ABS_X, xyz[0]);
	input_report_abs(stat->input_dev_mag, ABS_Y, xyz[1]);
	input_report_abs(stat->input_dev_mag, ABS_Z, xyz[2]);
	input_sync(stat->input_dev_mag);
}

static int k303b_mag_input_init(struct k303b_status *stat)
{
	int err;

	stat->input_dev_mag = input_allocate_device();
	if (!stat->input_dev_mag) {
		err = -ENOMEM;
		SENSOR_ERR("magnetometer input device allocation failed");
		goto err0;
	}

	stat->input_dev_mag->open = k303b_mag_input_open;
	stat->input_dev_mag->close = k303b_mag_input_close;

	stat->input_dev_mag->name = K303B_MAG_DEVICE_CUSTOM_NAME;
	stat->input_dev_mag->dev.init_name = LGE_MAGNETOMETER_NAME;

	stat->input_dev_mag->id.bustype = BUS_I2C;

	input_set_drvdata(stat->input_dev_mag, stat);

	set_bit(EV_ABS, stat->input_dev_mag->evbit);

	input_set_abs_params(stat->input_dev_mag, ABS_X,
				-MAG_G_MAX_NEG, MAG_G_MAX_POS, FUZZ, FLAT);
	input_set_abs_params(stat->input_dev_mag, ABS_Y,
				-MAG_G_MAX_NEG, MAG_G_MAX_POS, FUZZ, FLAT);
	input_set_abs_params(stat->input_dev_mag, ABS_Z,
				-MAG_G_MAX_NEG, MAG_G_MAX_POS, FUZZ, FLAT);

	err = input_register_device(stat->input_dev_mag);
	if (err) {
		SENSOR_ERR("unable to register magnetometer input device %s",
				stat->input_dev_mag->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(stat->input_dev_mag);
err0:
	return err;
}

static void k303b_input_cleanup(struct k303b_status *stat)
{
	input_unregister_device(stat->input_dev_mag);
	input_free_device(stat->input_dev_mag);
}

static void poll_function_work_mag(struct work_struct *input_work_mag)
{
	struct k303b_status *stat;
	int xyz[3] = { 0 };
	int err;

	stat = container_of((struct work_struct *)input_work_mag,
			struct k303b_status, input_work_mag);

	mutex_lock(&stat->lock);

	if (atomic_read(&stat->enabled_mag)) {
		err = k303b_mag_get_data(stat, xyz);
		if (err < 0)
			SENSOR_ERR("get_magnetometer_data failed");
		else
			k303b_mag_report_values(stat, xyz);
	}

	mutex_unlock(&stat->lock);
	hrtimer_start(&stat->hr_timer_mag, stat->ktime_mag, HRTIMER_MODE_REL);
}

enum hrtimer_restart poll_function_read_mag(struct hrtimer *timer)
{
	struct k303b_status *stat;


	stat = container_of((struct hrtimer *)timer,
				struct k303b_status, hr_timer_mag);

	queue_work(k303b_workqueue, &stat->input_work_mag);
	return HRTIMER_NORESTART;
}

static int k303b_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct k303b_status *stat;

	u32 smbus_func = I2C_FUNC_SMBUS_BYTE_DATA |
			I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_I2C_BLOCK;

	int err = -1;

	SENSOR_FUN();

	stat = kzalloc(sizeof(struct k303b_status), GFP_KERNEL);
	if (stat == NULL) {
		err = -ENOMEM;
		SENSOR_ERR("failed to allocate memory for module data: %d", err);
		goto exit_check_functionality_failed;
	}

	stat->use_smbus = 0;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		SENSOR_LOG("client not i2c capable");
		if (i2c_check_functionality(client->adapter, smbus_func)) {
			stat->use_smbus = 1;
			SENSOR_LOG("client using SMBUS");
		} else {
			err = -ENODEV;
			SENSOR_ERR("client nor SMBUS capable");
			goto exit_check_functionality_failed;
		}
	}

	if (k303b_workqueue == 0)
		k303b_workqueue = create_workqueue("k303b_workqueue");

	hrtimer_init(&stat->hr_timer_mag, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	stat->hr_timer_mag.function = &poll_function_read_mag;

	mutex_init(&stat->lock);
	mutex_lock(&stat->lock);

	stat->client = client;
	i2c_set_clientdata(client, stat);

	stat->pdata_mag = kmalloc(sizeof(*stat->pdata_mag), GFP_KERNEL);
	if (stat->pdata_mag == NULL) {
		err = -ENOMEM;
		SENSOR_ERR("failed to allocate memory for pdata: %d", err);
		goto err_mutexunlock;
	}

        // Initialize
        stat->pdata_mag->power_on = NULL;
        stat->pdata_mag->power_off = NULL;
        stat->pdata_mag->init = NULL;
        stat->pdata_mag->exit = NULL;

	if(client->dev.of_node) {
		/* device tree type */
		err = k303b_parse_dt(&client->dev,stat->pdata_mag);

	} else {
	    if (client->dev.platform_data == NULL) {
		    memcpy(stat->pdata_mag, &default_k303b_mag_pdata,
			    			sizeof(*stat->pdata_mag));
		    SENSOR_LOG("using default plaform_data for magnetometer");
	    }
	    else {
		    memcpy(stat->pdata_mag, client->dev.platform_data,
						    sizeof(*stat->pdata_mag));
	    }
    }
	stat->pdata_mag->power_on = k303b_mag_power_on;
	stat->pdata_mag->power_off = k303b_mag_power_off;

	err = k303b_mag_validate_pdata(stat);
	if (err < 0) {
		SENSOR_ERR("failed to validate platform data for magnetometer");
		goto exit_kfree_pdata;
	}

/*
	if (stat->pdata_mag->init) {
		err = stat->pdata_mag->init();
		if (err < 0) {
			dev_err(&client->dev,
				"magnetometer init failed: %d\n", err);
			goto err_pdata_mag_init;
		}

	}*/

	err = k303b_hw_init(stat);
	if (err < 0) {
		SENSOR_ERR("hw init failed: %d", err);
		goto err_hw_init;
	}

	err = k303b_mag_device_power_on(stat);
	if (err < 0) {
		SENSOR_ERR("magnetometer power on failed: %d", err);
		goto err_pdata_init;
	}

	err = k303b_mag_update_fs_range(stat, stat->pdata_mag->fs_range);
	if (err < 0) {
		SENSOR_ERR("update_fs_range on magnetometer failed");
		goto  err_power_off_mag;
	}

	err = k303b_mag_update_odr(stat, stat->pdata_mag->poll_interval);
	if (err < 0) {
		SENSOR_ERR("update_odr on magnetometer failed");
		goto  err_power_off;
	}

	err = k303b_mag_input_init(stat);
	if (err < 0) {
		SENSOR_ERR("magnetometer input init failed");
		goto err_power_off;
	}

	err = sysfs_create_group(&stat->input_dev_mag->dev.kobj, &k303b_mag_attr_group);
	if (err < 0) {
		SENSOR_ERR("sysfs register failed");
		goto err_input_cleanup;
	}

	k303b_mag_device_power_off(stat);

	INIT_WORK(&stat->input_work_mag, poll_function_work_mag);

	mutex_unlock(&stat->lock);
	SENSOR_LOG("probed");
	return 0;

err_input_cleanup:
	k303b_input_cleanup(stat);
	sysfs_remove_group(&stat->input_dev_mag->dev.kobj, &k303b_mag_attr_group);
err_power_off:
err_power_off_mag:
	k303b_mag_device_power_off(stat);
err_hw_init:
err_pdata_init:
//err_pdata_mag_init:
//	if (stat->pdata_mag->exit)
//		stat->pdata_mag->exit();
exit_kfree_pdata:
	kfree(stat->pdata_mag);
err_mutexunlock:
	mutex_unlock(&stat->lock);
	kfree(stat);
	if (!k303b_workqueue) {
		flush_workqueue(k303b_workqueue);
		destroy_workqueue(k303b_workqueue);
	}
exit_check_functionality_failed:
	SENSOR_ERR("Driver Init failed");
	return err;
}

static int k303b_remove(struct i2c_client *client)
{
	struct k303b_status *stat = i2c_get_clientdata(client);

	k303b_mag_disable(stat);
	k303b_mag_input_cleanup(stat);

#if 1 /* LGE_SENSOR_FACTORY_AAT */
	sysfs_remove_group(&stat->input_dev_mag->dev.kobj, &k303b_mag_attr_group);
#else
	remove_sysfs_interfaces(&client->dev);
#endif
	//if (stat->pdata_mag->exit)
	//	stat->pdata_mag->exit();

	if (!k303b_workqueue) {
		flush_workqueue(k303b_workqueue);
		destroy_workqueue(k303b_workqueue);
	}

	kfree(stat->pdata_mag);
	kfree(stat);
	return 0;
}


#ifdef CONFIG_PM
static int k303b_mag_resume(struct i2c_client *client)
{
	struct k303b_status *stat = i2c_get_clientdata(client);
	SENSOR_FUN();
	if (stat->on_before_suspend)
		return k303b_mag_enable(stat);

	return 0;
}

static int k303b_mag_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct k303b_status *stat = i2c_get_clientdata(client);
	SENSOR_FUN();
	stat->on_before_suspend = atomic_read(&stat->enabled_mag);
	return k303b_mag_disable(stat);
}
#else
#define k303b_mag_suspend	NULL
#define k303b_mag_resume	NULL
#endif /* CONFIG_PM */


static const struct i2c_device_id k303b_id[] = {
		{ K303B_MAG_DEV_NAME, 0 },
		{ },
};

MODULE_DEVICE_TABLE(i2c, k303b_id);
#ifdef CONFIG_OF
static struct of_device_id k303b_mag_match_table[] = {
	    { .compatible = "st,k303b_mag",},
		{ },
};
#else
#define k303b_mag_match_table NULL
#endif

static struct i2c_driver k303b_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name = K303B_MAG_DEV_NAME,
            .of_match_table = k303b_mag_match_table,
	},
	.probe = k303b_probe,
	.remove = k303b_remove,
	.suspend = k303b_mag_suspend,
	.resume = k303b_mag_resume,
	.id_table = k303b_id,
};

static int __init k303b_init(void)
{
	SENSOR_FUN();
	return i2c_add_driver(&k303b_driver);
}

static void __exit k303b_exit(void)
{
	SENSOR_FUN();
	i2c_del_driver(&k303b_driver);
}

module_init(k303b_init);
module_exit(k303b_exit);

MODULE_DESCRIPTION("k303b magnetometer driver");
MODULE_AUTHOR("Matteo Dameno, Denis Ciocca, STMicroelectronics");
MODULE_LICENSE("GPL");
