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

/********************************/
/* Asus Sensor Hub Interfacee */
/******************************/
#ifndef __LINUX_ASH_INTERFACE_H
#define __LINUX_ASH_INTERFACE_H

typedef enum{
	psensor = 0,
	lsensor,
	hallsensor,
}ASH_Interface_type;

#include <linux/device.h>
#include <linux/fs.h>
extern struct device *ASH_Interface_device_create(ASH_Interface_type type);
extern void ASH_Interface_device_remove(ASH_Interface_type type);

#endif
