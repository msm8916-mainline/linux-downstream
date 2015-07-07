/* 
 * Copyright (C) 2015 ASUSTek Inc.
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

 /**************************/
/* Debug and Log System */
/************************/
#define MODULE_NAME	"ASH_Interface"
#undef dbg
#ifdef ASH_INTERFACE_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s] "fmt,MODULE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif
#define log(fmt, args...) printk(KERN_INFO "[%s] "fmt,MODULE_NAME,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s] "fmt,MODULE_NAME,##args)

 /************************/
/* IR Sensor Interface */
/**********************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include "IRsensor_Interface.h"
#include "ASH_Interface.h"


/*******************/
/*Sensor Property */
/******************/
static struct device_attribute proximity_property_attrs[] = {
		/*read only*/
	__ATTR(vendor, 0444, NULL, NULL),
	
};

static struct device_attribute light_property_attrs[] = {
	/*read only*/
	__ATTR(vendor, 0444, NULL, NULL),
	
};

int IRsensor_Interface_register(IRsensor_Interface *mInterface)
{
	int ret = 0;
	int interface_index;
	struct device *psensor_dev;
	struct device *lsensor_dev;
	
	/* psensor device */
	psensor_dev = ASH_Interface_device_create(psensor);
	if (IS_ERR(psensor_dev) || psensor_dev == NULL) {
		ret = PTR_ERR(psensor_dev);
		err("IRsensor_Interface_register : psensor create ERROR.\n");
		return ret;
	}	
	for (interface_index=0; interface_index < ARRAY_SIZE(proximity_property_attrs); interface_index++) {
		ret = device_create_file(psensor_dev, &proximity_property_attrs[interface_index]);
		if (ret)
			return ret;
	}

	/*lsensor device*/
	lsensor_dev = ASH_Interface_device_create(lsensor);
	if (IS_ERR(lsensor_dev) || lsensor_dev == NULL) {
		err("IRsensor_Interface_register : lsensor create ERROR.\n");
		ret = PTR_ERR(lsensor_dev);
		return ret;
	}
	for (interface_index=0; interface_index < ARRAY_SIZE(light_property_attrs); interface_index++) {
		ret = device_create_file(lsensor_dev, &light_property_attrs[interface_index]);
		if (ret)
			return ret;
	}
	
	return 0;
}

int IRsensor_Interface_unregister(void)
{
	ASH_Interface_device_remove(psensor);
	ASH_Interface_device_remove(lsensor);
	return 0;
}

