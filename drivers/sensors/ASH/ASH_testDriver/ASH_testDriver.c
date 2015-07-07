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
#define MODULE_NAME	"ASH_testDriver"
#undef dbg
#ifdef ASH_INTERFACE_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s] "fmt,MODULE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif
#define log(fmt, args...) printk(KERN_INFO "[%s] "fmt,MODULE_NAME,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s] "fmt,MODULE_NAME,##args)

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include "ASH_testDriver.h"
#include "../ASH_Interface/ASH_Interface.h"

//static struct device_attribute testDriver_attrs[] = {
		/*read only*/
//	__ATTR(srTest, 0444, NULL, NULL),
//};

static int __init testDriver_init(void)
{
	log("Driver INIT +++\n");

	ASH_Interface_device_create(psensor);
	ASH_Interface_device_create(lsensor);
	ASH_Interface_device_create(hallsensor);

	ASH_Interface_device_remove(lsensor);
		
	log("Driver INIT ---\n");
	return 0;
}

static void __exit testDriver_exit(void)
{
	log("Driver EXIT +++\n");

	
	log("Driver EXIT ---\n");
}

module_init(testDriver_init);
module_exit(testDriver_exit);

MODULE_AUTHOR("sr_Huang <sr_Huang@asus.com>");
MODULE_DESCRIPTION("Asus Sensor Hub Test Driver");
MODULE_LICENSE("GPL");