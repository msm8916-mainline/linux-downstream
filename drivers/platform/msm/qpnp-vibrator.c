/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/of_device.h>
#include <linux/spmi.h>
#include <linux/qpnp/pwm.h>
#include <linux/err.h>
/* QCI Begin: ALT Vibrator Daniel Wang 20140709 */
#include <linux/delay.h>
/* QCI End: ALT Vibrator Daniel Wang 20140709 */
#include "../../staging/android/timed_output.h"

/* QCI Begin: ALT Vibrator Daniel Wang 20140709 */
/**
 * Qpnp vib messages
 */
#define QPNP_VIB_MSG_ACTION(M, ...) printk("[qvib][%s][%05d][act]: " M "\n", __func__, __LINE__, ##__VA_ARGS__);
#define QPNP_VIB_MSG_ERROR(M, ...) printk("[qvib][%s][%05d][err]: " M "\n", __func__, __LINE__, ##__VA_ARGS__);
#define QPNP_VIB_MSG_INFO(M, ...) printk("[qvib][%s][%05d][inf]: " M "\n", __func__, __LINE__, ##__VA_ARGS__);
#define QPNP_VIB_MSG_FUNCIN printk("[qvib][%s][%05d][act]: ++++++++++.\n", __func__, __LINE__);
#define QPNP_VIB_MSG_FUNCOUT printk("[qvib][%s][%05d][act]: ----------.\n", __func__, __LINE__);
/* QCI End: ALT Vibrator Daniel Wang 20140709 */
#define QPNP_VIB_VTG_CTL(base)		(base + 0x41)
#define QPNP_VIB_EN_CTL(base)		(base + 0x46)

#define QPNP_VIB_MAX_LEVEL		31
#define QPNP_VIB_MIN_LEVEL		12

#define QPNP_VIB_DEFAULT_TIMEOUT	15000
#define QPNP_VIB_DEFAULT_VTG_LVL	3100

#define QPNP_VIB_EN			BIT(7)
#define QPNP_VIB_VTG_SET_MASK		0x1F
#define QPNP_VIB_LOGIC_SHIFT		4

enum qpnp_vib_mode {
	QPNP_VIB_MANUAL,
	QPNP_VIB_DTEST1,
	QPNP_VIB_DTEST2,
	QPNP_VIB_DTEST3,
};

struct qpnp_pwm_info {
	struct pwm_device *pwm_dev;
	u32 pwm_channel;
	u32 duty_us;
	u32 period_us;
};

struct qpnp_vib {
	struct spmi_device *spmi;
	struct hrtimer vib_timer;
	struct timed_output_dev timed_dev;
	struct work_struct work;
	struct qpnp_pwm_info pwm_info;
	enum   qpnp_vib_mode mode;

	u8  reg_vtg_ctl;
	u8  reg_en_ctl;
	u8  active_low;
	u16 base;
	int state;
	int vtg_level;
	int timeout;
	struct mutex lock;
};

/* QCI Begin: ALT Vibrator Daniel Wang 20140709 */
/**
 * Vib lifetest type
 */
typedef struct qpnp_vib_lifetest_struct
{
	bool enable;
	int count;
	int cycle;
	int offtime;
	int ontime;
	struct qpnp_vib* vib;
	struct work_struct work;
}qpnp_vib_lifetest_type;

/**
 * Vib lifetest
 */
static qpnp_vib_lifetest_type qpnp_vib_lifetest =
{
	false,
	0,
	120000,
	1000,
	2000,
	NULL
};
/* QCI End: ALT Vibrator Daniel Wang 20140709 */

static int qpnp_vib_read_u8(struct qpnp_vib *vib, u8 *data, u16 reg)
{
	int rc;

	rc = spmi_ext_register_readl(vib->spmi->ctrl, vib->spmi->sid,
							reg, data, 1);
	if (rc < 0)
		dev_err(&vib->spmi->dev,
			"Error reading address: %X - ret %X\n", reg, rc);

	return rc;
}

static int qpnp_vib_write_u8(struct qpnp_vib *vib, u8 *data, u16 reg)
{
	int rc;

	rc = spmi_ext_register_writel(vib->spmi->ctrl, vib->spmi->sid,
							reg, data, 1);
	if (rc < 0)
		dev_err(&vib->spmi->dev,
			"Error writing address: %X - ret %X\n", reg, rc);

	return rc;
}

static int qpnp_vibrator_config(struct qpnp_vib *vib)
{
	u8 reg = 0;
	int rc;

	/* Configure the VTG CTL regiser */
	rc = qpnp_vib_read_u8(vib, &reg, QPNP_VIB_VTG_CTL(vib->base));
	if (rc < 0)
		return rc;
	reg &= ~QPNP_VIB_VTG_SET_MASK;
	reg |= (vib->vtg_level & QPNP_VIB_VTG_SET_MASK);
	rc = qpnp_vib_write_u8(vib, &reg, QPNP_VIB_VTG_CTL(vib->base));
	if (rc)
		return rc;
	vib->reg_vtg_ctl = reg;

	/* Configure the VIB ENABLE regiser */
	rc = qpnp_vib_read_u8(vib, &reg, QPNP_VIB_EN_CTL(vib->base));
	if (rc < 0)
		return rc;
	reg |= (!!vib->active_low) << QPNP_VIB_LOGIC_SHIFT;
	if (vib->mode != QPNP_VIB_MANUAL) {
		vib->pwm_info.pwm_dev = pwm_request(vib->pwm_info.pwm_channel,
								 "qpnp-vib");
		if (IS_ERR_OR_NULL(vib->pwm_info.pwm_dev)) {
			dev_err(&vib->spmi->dev, "vib pwm request failed\n");
			return -ENODEV;
		}

		rc = pwm_config(vib->pwm_info.pwm_dev, vib->pwm_info.duty_us,
						vib->pwm_info.period_us);
		if (rc < 0) {
			dev_err(&vib->spmi->dev, "vib pwm config failed\n");
			pwm_free(vib->pwm_info.pwm_dev);
			return -ENODEV;
		}

		reg |= BIT(vib->mode - 1);
	}

	rc = qpnp_vib_write_u8(vib, &reg, QPNP_VIB_EN_CTL(vib->base));
	if (rc < 0)
		return rc;
	vib->reg_en_ctl = reg;

	return rc;
}

static int qpnp_vib_set(struct qpnp_vib *vib, int on)
{
	int rc;
	u8 val;

	if (on) {
		if (vib->mode != QPNP_VIB_MANUAL)
			pwm_enable(vib->pwm_info.pwm_dev);
		else {
			val = vib->reg_en_ctl;
			val |= QPNP_VIB_EN;
			rc = qpnp_vib_write_u8(vib, &val,
					QPNP_VIB_EN_CTL(vib->base));
			if (rc < 0)
				return rc;
			vib->reg_en_ctl = val;
		}
	} else {
		if (vib->mode != QPNP_VIB_MANUAL)
			pwm_disable(vib->pwm_info.pwm_dev);
		else {
			val = vib->reg_en_ctl;
			val &= ~QPNP_VIB_EN;
			rc = qpnp_vib_write_u8(vib, &val,
					QPNP_VIB_EN_CTL(vib->base));
			if (rc < 0)
				return rc;
			vib->reg_en_ctl = val;
		}
	}

	return 0;
}

static void qpnp_vib_enable(struct timed_output_dev *dev, int value)
{
	struct qpnp_vib *vib = container_of(dev, struct qpnp_vib,
					 timed_dev);

	mutex_lock(&vib->lock);
	hrtimer_cancel(&vib->vib_timer);

	if (value == 0)
		vib->state = 0;
	else {
		value = (value > vib->timeout ?
				 vib->timeout : value);
		vib->state = 1;
		hrtimer_start(&vib->vib_timer,
			      ktime_set(value / 1000, (value % 1000) * 1000000),
			      HRTIMER_MODE_REL);
	}
	mutex_unlock(&vib->lock);
	schedule_work(&vib->work);
}

static void qpnp_vib_update(struct work_struct *work)
{
	struct qpnp_vib *vib = container_of(work, struct qpnp_vib,
					 work);
	qpnp_vib_set(vib, vib->state);
}

static int qpnp_vib_get_time(struct timed_output_dev *dev)
{
	struct qpnp_vib *vib = container_of(dev, struct qpnp_vib,
							 timed_dev);

	if (hrtimer_active(&vib->vib_timer)) {
		ktime_t r = hrtimer_get_remaining(&vib->vib_timer);
		return (int)ktime_to_us(r);
	} else
		return 0;
}

static enum hrtimer_restart qpnp_vib_timer_func(struct hrtimer *timer)
{
	struct qpnp_vib *vib = container_of(timer, struct qpnp_vib,
							 vib_timer);

	vib->state = 0;
	schedule_work(&vib->work);

	return HRTIMER_NORESTART;
}

/* QCI Begin: ALT Vibrator Daniel Wang 20140709 */
/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
/**
  * Function
  * qpnp_vib_lifetest_count_show
  *
  * Description
  * Show vib lifetest counts.
  *
  * Dependencies
  * None
  *
  * Arguments
  * device: Device is sys/class/timed_output/vibrator.
  * device_attribute: Device attribute.
  * buffer: Echo buffer.
  *
  * Return value
  * None
  *
  * Side effects
  * None
  */
/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
static ssize_t qpnp_vib_lifetest_count_show(struct device* device, struct device_attribute* device_attribute, char* buffer)
{
	/**
	 * Init varaible
	 */
	ssize_t result = 0;

	/**
	 * Check parameter
	 */
	if(buffer == NULL)
	{
		QPNP_VIB_MSG_ERROR("Failed for null-point buffer!");
		result = -1;
		return result;
	}

	/**
	 * Echo count
	 */
	result = sprintf(buffer, "%d\n", qpnp_vib_lifetest.count);
	return result;
}

/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
/**
  * Function
  * qpnp_vib_lifetest_count_store
  *
  * Description
  * Store vib lifetest count.
  *
  * Dependencies
  * None
  *
  * Arguments
  * device: Device is sys/class/timed_output/vibrator.
  * device_attribute: Device attribute.
  * buffer: Input buffer.
  * count: Correct return.
  *
  * Return value
  * None
  *
  * Side effects
  * None
  */
/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
static ssize_t qpnp_vib_lifetest_count_store(struct device* device, struct device_attribute* device_attribute, const char* buffer, size_t ret_count)
{
	/**
	 * Init varaibles
	 */
	int count = 0;
	ssize_t result = 0;

	/**
	 * Check parameter
	 */
	if(buffer == NULL)
	{
		QPNP_VIB_MSG_ERROR("Failed for null-point buffer!");
		result = -EINVAL;
		return result;
	}

	/**
	 * Sscanf count
	 */
	result = sscanf(buffer, "%d", &count);
	if(result != 1 || count != 0)
	{
		QPNP_VIB_MSG_ERROR("Failed for invalid count!");
		result = -EINVAL;
		return result;
	}

	/**
	 * Save count
	 */
	qpnp_vib_lifetest.count = count;

	result = ret_count;
	return result;
}

static DEVICE_ATTR(lifetest_count, S_IRUGO|S_IWUSR, qpnp_vib_lifetest_count_show, qpnp_vib_lifetest_count_store);

/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
/**
  * Function
  * qpnp_vib_lifetest_cycle_show
  *
  * Description
  * Show vib lifetest cycle.
  *
  * Dependencies
  * None
  *
  * Arguments
  * device: Device is sys/class/timed_output/vibrator.
  * device_attribute: Device attribute.
  * buffer: Echo buffer.
  *
  * Return value
  * None
  *
  * Side effects
  * None
  */
/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
static ssize_t qpnp_vib_lifetest_cycle_show(struct device* device, struct device_attribute* device_attribute, char* buffer)
{
	/**
	 * Init varaible
	 */
	ssize_t result = 0;

	/**
	 * Check parameter
	 */
	if(buffer == NULL)
	{
		QPNP_VIB_MSG_ERROR("Failed for null-point buffer!");
		result = -1;
		return result;
	}

	/**
	 * Echo cycle
	 */
	result = sprintf(buffer, "%d\n", qpnp_vib_lifetest.cycle);

	return result;
}

/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
/**
  * Function
  * qpnp_vib_lifetest_cycle_store
  *
  * Description
  * Store vib lifetest cycle.
  *
  * Dependencies
  * None
  *
  * Arguments
  * device: Device is sys/class/timed_output/vibrator.
  * device_attribute: Device attribute.
  * buffer: Input buffer.
  * count: Correct return.
  *
  * Return value
  * None
  *
  * Side effects
  * None
  */
/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
static ssize_t qpnp_vib_lifetest_cycle_store(struct device* device, struct device_attribute* device_attribute, const char* buffer, size_t count)
{
	/**
	 * Init varaibles
	 */
	int cycle = 0;
	ssize_t result = 0;

	/**
	 * Check parameter
	 */
	if(buffer == NULL)
	{
		QPNP_VIB_MSG_ERROR("Failed for null-point buffer!");
		result = -EINVAL;
		return result;
	}

	/**
	 * Sscanf cycle
	 */
	result = sscanf(buffer, "%d", &cycle);
	if(result != 1 || cycle < 1)
	{
		QPNP_VIB_MSG_ERROR("Failed for invalid cycle!");
		result = -EINVAL;
		return result;
	}

	/**
	 * Save cycle
	 */
	qpnp_vib_lifetest.cycle = cycle;

	result = count;
	return result;
}

static DEVICE_ATTR(lifetest_cycle, S_IRUGO|S_IWUSR, qpnp_vib_lifetest_cycle_show, qpnp_vib_lifetest_cycle_store);

/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
/**
  * Function
  * qpnp_vib_lifetest_enable_show
  *
  * Description
  * Show vib lifetest enable.
  *
  * Dependencies
  * None
  *
  * Arguments
  * dev: Device is sys/class/timed_output/vibrator.
  * attr: Device attribute.
  * buf: Echo buffer.
  *
  * Return value
  * None
  *
  * Side effects
  * None
  */
/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
static ssize_t qpnp_vib_lifetest_enable_show(struct device* device, struct device_attribute* device_attribute, char *buffer)
{
	/**
	 * Init varaible
	 */
	ssize_t result = 0;

	/**
	 * Check parameter
	 */
	if(buffer == NULL)
	{
		QPNP_VIB_MSG_ERROR("Failed for null-point buffer!");
		result = -1;
		return result;
	}

	/**
	 * Echo enable
	 */
	result = sprintf(buffer, "%d\n", qpnp_vib_lifetest.enable);

	return result;
}

/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
/**
  * Function
  * qpnp_vib_lifetest_enable_store
  *
  * Description
  * Store vib lifetest enable.
  *
  * Dependencies
  * None
  *
  * Arguments
  * device: Device is sys/class/timed_output/vibrator.
  * device_attribute: Device attribute.
  * buffer: Input buffer.
  * count: Correct return.
  *
  * Return value
  * None
  *
  * Side effects
  * None
  */
/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
static ssize_t qpnp_vib_lifetest_enable_store(struct device* device, struct device_attribute* device_attribute, const char* buffer, size_t count)
{
	/**
	 * Init varaibles
	 */
	int enable = 0;
	ssize_t result = 0;

	/**
	 * Check parameter
	 */
	if(buffer == NULL)
	{
		QPNP_VIB_MSG_ERROR("Failed for null-point buffer!");
		result = -EINVAL;
		return result;
	}

	/**
	 * Sscanf enable
	 */
	result = sscanf(buffer, "%d", &enable);
	if(result != 1 || enable > 1 || enable < 0)
	{
		QPNP_VIB_MSG_ERROR("Failed for invalid enable!");
		result = -EINVAL;
		return result;
	}

	/**
	 * Check enable
	 */
	if(qpnp_vib_lifetest.enable == enable)
	{
		goto out;
	}

	/**
	 * Save enable
	 */
	qpnp_vib_lifetest.enable = enable;

	/**
	 * Start/stop vib
	 */
	if(qpnp_vib_lifetest.enable == true)
	{
		QPNP_VIB_MSG_ACTION("Start vib lifetest.");
		QPNP_VIB_MSG_ACTION("Schedule vib lifetest work.");
		schedule_work(&qpnp_vib_lifetest.work);
	}
	else
	{
		QPNP_VIB_MSG_ACTION("Stop vib lifetest.");
		QPNP_VIB_MSG_ACTION("Stop vib.");
		qpnp_vib_set(qpnp_vib_lifetest.vib, 0);

		QPNP_VIB_MSG_ACTION("cancel vib lifetest work.");
		cancel_work_sync(&qpnp_vib_lifetest.work);
	}

out:
	result = count;
	return count;
}

static DEVICE_ATTR(lifetest_enable, S_IRUGO|S_IWUSR, qpnp_vib_lifetest_enable_show, qpnp_vib_lifetest_enable_store);

/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
/**
  * Function
  * qpnp_vib_lifetest_offtime_show
  *
  * Description
  * Show vib lifetest offtime.
  *
  * Dependencies
  * None
  *
  * Arguments
  * device: Device is sys/class/timed_output/vibrator.
  * device_attribute: Device attribute.
  * buffer: Echo buffer.
  *
  * Return value
  * None
  *
  * Side effects
  * None
  */
/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
static ssize_t qpnp_vib_lifetest_offtime_show(struct device* device, struct device_attribute* device_attribute, char* buffer)
{
	/**
	 * Init varaible
	 */
	ssize_t result = 0;

	/**
	 * Check parameter
	 */
	if(buffer == NULL)
	{
		QPNP_VIB_MSG_ERROR("Failed for null-point buffer!");
		result = -1;
		return result;
	}

	/**
	 * Echo offtime
	 */
	result = sprintf(buffer, "%d\n", qpnp_vib_lifetest.offtime);

	return result;
}

/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
/**
  * Function
  * qpnp_vib_lifetest_offtime_store
  *
  * Description
  * Store vib lifetest offtime.
  *
  * Dependencies
  * None
  *
  * Arguments
  * device: Device is sys/class/timed_output/vibrator.
  * device_attribute: Device attribute.
  * buffer: Input buffer.
  * count: Correct return.
  *
  * Return value
  * None
  *
  * Side effects
  * None
  */
/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
static ssize_t qpnp_vib_lifetest_offtime_store(struct device* device, struct device_attribute* device_attribute, const char* buffer, size_t count)
{
	/**
	 * Init varaibles
	 */
	int offtime = 0;
	ssize_t result = 0;

	/**
	 * Check parameter
	 */
	if(buffer == NULL)
	{
		QPNP_VIB_MSG_ERROR("Failed for null-point buffer!");
		result = -EINVAL;
		return result;
	}

	/**
	 * Sscanf offtime
	 */
	result = sscanf(buffer, "%d", &offtime);
	if(result != 1 || offtime < 1)
	{
		QPNP_VIB_MSG_ERROR("Failed for invalid offtime!");
		result = -EINVAL;
		return result;
	}

	/**
	 * Save offtime
	 */
	qpnp_vib_lifetest.offtime = offtime;

	result = count;
	return result;
}

static DEVICE_ATTR(lifetest_offtime, S_IRUGO|S_IWUSR, qpnp_vib_lifetest_offtime_show, qpnp_vib_lifetest_offtime_store);

/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
/**
  * Function
  * qpnp_vib_lifetest_ontime_show
  *
  * Description
  * Show vib lifetest ontime.
  *
  * Dependencies
  * None
  *
  * Arguments
  * device: Device is sys/class/timed_output/vibrator.
  * device_attribute: Device attribute.
  * buffer: Echo buffer.
  *
  * Return value
  * None
  *
  * Side effects
  * None
  */
/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
static ssize_t qpnp_vib_lifetest_ontime_show(struct device* device, struct device_attribute* device_attribute, char* buffer)
{
	/**
	 * Init varaible
	 */
	ssize_t result = 0;

	/**
	 * Check parameter
	 */
	if(buffer == NULL)
	{
		QPNP_VIB_MSG_ERROR("Failed for null-point buffer!");
		result = -1;
		return result;
	}

	/**
	 * Echo ontime
	 */
	result = sprintf(buffer, "%d\n", qpnp_vib_lifetest.ontime);

	return result;
}

/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
/**
  * Function
  * qpnp_vib_lifetest_ontime_store
  *
  * Description
  * Store vib lifetest ontime.
  *
  * Dependencies
  * None
  *
  * Arguments
  * device: Device is sys/class/timed_output/vibrator.
  * device_attribute: Device attribute.
  * buffer: Input buffer.
  * count: Correct return.
  *
  * Return value
  * None
  *
  * Side effects
  * None
  */
/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
static ssize_t qpnp_vib_lifetest_ontime_store(struct device* device, struct device_attribute* device_attribute, const char* buffer, size_t count)
{
	/**
	 * Init varaibles
	 */
	int ontime = 0;
	ssize_t result = 0;

	/**
	 * Check parameter
	 */
	if(buffer == NULL)
	{
		QPNP_VIB_MSG_ERROR("Failed for null-point buffer!");
		result = -EINVAL;
		return result;
	}

	/**
	 * Sscanf ontime
	 */
	result = sscanf(buffer, "%d", &ontime);
	if(result != 1 || ontime < 1)
	{
		QPNP_VIB_MSG_ERROR("Failed for invalid ontime!");
		result = -EINVAL;
		return result;
	}

	/**
	 * Save ontime
	 */
	qpnp_vib_lifetest.ontime = ontime;

	result = count;
	return result;
}

static DEVICE_ATTR(lifetest_ontime, S_IRUGO|S_IWUSR, qpnp_vib_lifetest_ontime_show, qpnp_vib_lifetest_ontime_store);
/* QCI End: ALT Vibrator Daniel Wang 20140709 */

#ifdef CONFIG_PM
static int qpnp_vibrator_suspend(struct device *dev)
{
	struct qpnp_vib *vib = dev_get_drvdata(dev);

	hrtimer_cancel(&vib->vib_timer);
	cancel_work_sync(&vib->work);
	/* turn-off vibrator */
	qpnp_vib_set(vib, 0);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(qpnp_vibrator_pm_ops, qpnp_vibrator_suspend, NULL);

static int qpnp_vib_parse_dt(struct qpnp_vib *vib)
{
	struct spmi_device *spmi = vib->spmi;
	int rc;
	const char *mode;
	u32 temp_val;

	vib->timeout = QPNP_VIB_DEFAULT_TIMEOUT;
	rc = of_property_read_u32(spmi->dev.of_node,
			"qcom,vib-timeout-ms", &temp_val);
	if (!rc) {
		vib->timeout = temp_val;
	} else if (rc != -EINVAL) {
		dev_err(&spmi->dev, "Unable to read vib timeout\n");
		return rc;
	}

	vib->vtg_level = QPNP_VIB_DEFAULT_VTG_LVL;
	rc = of_property_read_u32(spmi->dev.of_node,
			"qcom,vib-vtg-level-mV", &temp_val);
	if (!rc) {
		vib->vtg_level = temp_val;
	} else if (rc != -EINVAL) {
		dev_err(&spmi->dev, "Unable to read vtg level\n");
		return rc;
	}

	vib->vtg_level /= 100;
	if (vib->vtg_level < QPNP_VIB_MIN_LEVEL)
		vib->vtg_level = QPNP_VIB_MIN_LEVEL;
	else if (vib->vtg_level > QPNP_VIB_MAX_LEVEL)
		vib->vtg_level = QPNP_VIB_MAX_LEVEL;

	vib->mode = QPNP_VIB_MANUAL;
	rc = of_property_read_string(spmi->dev.of_node, "qcom,mode", &mode);
	if (!rc) {
		if (strcmp(mode, "manual") == 0)
			vib->mode = QPNP_VIB_MANUAL;
		else if (strcmp(mode, "dtest1") == 0)
			vib->mode = QPNP_VIB_DTEST1;
		else if (strcmp(mode, "dtest2") == 0)
			vib->mode = QPNP_VIB_DTEST2;
		else if (strcmp(mode, "dtest3") == 0)
			vib->mode = QPNP_VIB_DTEST3;
		else {
			dev_err(&spmi->dev, "Invalid mode\n");
			return -EINVAL;
		}
	} else if (rc != -EINVAL) {
		dev_err(&spmi->dev, "Unable to read mode\n");
		return rc;
	}

	if (vib->mode != QPNP_VIB_MANUAL) {
		rc = of_property_read_u32(spmi->dev.of_node,
				"qcom,pwm-channel", &temp_val);
		if (!rc)
			vib->pwm_info.pwm_channel = temp_val;
		else
			return rc;

		rc = of_property_read_u32(spmi->dev.of_node,
				"qcom,period-us", &temp_val);
		if (!rc)
			vib->pwm_info.period_us = temp_val;
		else
			return rc;

		rc = of_property_read_u32(spmi->dev.of_node,
				"qcom,duty-us", &temp_val);
		if (!rc)
			vib->pwm_info.duty_us = temp_val;
		else
			return rc;
	}

	vib->active_low = of_property_read_bool(spmi->dev.of_node,
				"qcom,active-low");

	return 0;
}

/* QCI Begin: ALT Vibrator Daniel Wang 20140709 */
/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
/**
  * Function
  * qpnp_vib_work_func
  *
  * Description
  * This is the lifetest work function.
  *
  * Dependencies
  * None
  *
  * Arguments
  * work: Lifetest work.
  *
  * Return value
  * None
  *
  * Side effects
  * None
  */
/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
static void qpnp_vib_work_func(struct work_struct *work)
{
	/**
	 * Init variable
	 */
	int i = 0;

	/**
	 * Check variable
	 */
	if(qpnp_vib_lifetest.vib == NULL)
	{
		QPNP_VIB_MSG_ERROR("Failed for null-point vib!");
		return;
	}

	/**
	 * Star vib lifetest
	 */
	for(i = qpnp_vib_lifetest.count + 1; i <= qpnp_vib_lifetest.cycle; i++)
	{
		QPNP_VIB_MSG_ACTION("Do %6d lifetest.", i);

		/** Enable vib */
		qpnp_vib_set(qpnp_vib_lifetest.vib, 1);
		msleep(qpnp_vib_lifetest.ontime);

		/** Disable vib */
		qpnp_vib_set(qpnp_vib_lifetest.vib, 0);
		msleep(qpnp_vib_lifetest.offtime);

		/** Save count */
		qpnp_vib_lifetest.count = i;

		/** Check vib lifetest enable */
		if(qpnp_vib_lifetest.enable == false)
			break;
	}

	qpnp_vib_lifetest.enable = false;
}
/* QCI End: ALT Vibrator Daniel Wang 20140709 */

static int qpnp_vibrator_probe(struct spmi_device *spmi)
{
	struct qpnp_vib *vib;
	struct resource *vib_resource;
	int rc;

	vib = devm_kzalloc(&spmi->dev, sizeof(*vib), GFP_KERNEL);
	if (!vib)
		return -ENOMEM;

	vib->spmi = spmi;

	vib_resource = spmi_get_resource(spmi, 0, IORESOURCE_MEM, 0);
	if (!vib_resource) {
		dev_err(&spmi->dev, "Unable to get vibrator base address\n");
		return -EINVAL;
	}
	vib->base = vib_resource->start;

	rc = qpnp_vib_parse_dt(vib);
	if (rc) {
		dev_err(&spmi->dev, "DT parsing failed\n");
		return rc;
	}

	rc = qpnp_vibrator_config(vib);
	if (rc) {
		dev_err(&spmi->dev, "vib config failed\n");
		return rc;
	}

	mutex_init(&vib->lock);
	INIT_WORK(&vib->work, qpnp_vib_update);

	hrtimer_init(&vib->vib_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vib->vib_timer.function = qpnp_vib_timer_func;

	vib->timed_dev.name = "vibrator";
	vib->timed_dev.get_time = qpnp_vib_get_time;
	vib->timed_dev.enable = qpnp_vib_enable;

	dev_set_drvdata(&spmi->dev, vib);

	rc = timed_output_dev_register(&vib->timed_dev);
	if (rc < 0)
		return rc;

	/* QCI Begin: ALT Vibrator Daniel Wang 20140709 */
	/**
	 * Save vib for lifetest
	 */
	qpnp_vib_lifetest.vib = vib;
	
	/**
	 * Create attributes into sys/class/timed_output/vibrator
	 */
	rc = device_create_file(vib->timed_dev.dev, &dev_attr_lifetest_count);
	if(rc)
	{
		QPNP_VIB_MSG_ERROR("Failed to create count attribute!");
	}

	rc = device_create_file(vib->timed_dev.dev, &dev_attr_lifetest_cycle);
	if(rc)
	{
		QPNP_VIB_MSG_ERROR("Failed to create cycle attribute!");
	}

	rc = device_create_file(vib->timed_dev.dev, &dev_attr_lifetest_enable);
	if(rc)
	{
		QPNP_VIB_MSG_ERROR("Failed to create start attribute!");
	}

	rc = device_create_file(vib->timed_dev.dev, &dev_attr_lifetest_offtime);
	if(rc)
	{
		QPNP_VIB_MSG_ERROR("Failed to create offtime attribute!");
	}

	rc = device_create_file(vib->timed_dev.dev, &dev_attr_lifetest_ontime);
	if(rc)
	{
		QPNP_VIB_MSG_ERROR("Failed to create ontime attribute!");
	}

	/**
	 * Init vib lifetest work
	 */
	INIT_WORK(&qpnp_vib_lifetest.work, qpnp_vib_work_func);
	/* QCI End: ALT Vibrator Daniel Wang 20140709 */

	return rc;
}

static int qpnp_vibrator_remove(struct spmi_device *spmi)
{
	struct qpnp_vib *vib = dev_get_drvdata(&spmi->dev);

	cancel_work_sync(&vib->work);
	hrtimer_cancel(&vib->vib_timer);
	timed_output_dev_unregister(&vib->timed_dev);
	mutex_destroy(&vib->lock);

	return 0;
}

static struct of_device_id spmi_match_table[] = {
	{	.compatible = "qcom,qpnp-vibrator",
	},
	{}
};

static struct spmi_driver qpnp_vibrator_driver = {
	.driver		= {
		.name	= "qcom,qpnp-vibrator",
		.of_match_table = spmi_match_table,
		.pm	= &qpnp_vibrator_pm_ops,
	},
	.probe		= qpnp_vibrator_probe,
	.remove		= qpnp_vibrator_remove,
};

static int __init qpnp_vibrator_init(void)
{
	return spmi_driver_register(&qpnp_vibrator_driver);
}
module_init(qpnp_vibrator_init);

static void __exit qpnp_vibrator_exit(void)
{
	return spmi_driver_unregister(&qpnp_vibrator_driver);
}
module_exit(qpnp_vibrator_exit);

MODULE_DESCRIPTION("qpnp vibrator driver");
MODULE_LICENSE("GPL v2");
