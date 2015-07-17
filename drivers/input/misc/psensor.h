#ifndef _PSENSOR_H_
#define _PSENSOR_H_


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/input/mt.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/switch.h>

struct psensor_data {
	bool power_on;
	int irq;
	int enable_gpio;
	int p_sensor_value;

	struct regulator *vdd;
	struct switch_dev *psensor_dev;
	struct work_struct psensor_work;
	struct workqueue_struct *psensor_wq;
};

enum PSENSOR_STATUS{
	PSENSOR_STATUS_NEAR=0,
	PSENSOR_STATUS_FAR=1,
};

struct psensor_driver_t{
	char *name;
	int (*init)(void);
};


void send_psensor_uevent(enum PSENSOR_STATUS val);

#endif
