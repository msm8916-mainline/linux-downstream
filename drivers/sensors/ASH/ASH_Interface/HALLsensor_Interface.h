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

/***************************/
/* Hall Sensor Interfacee */
/*************************/
#ifndef __LINUX_HALLSENSOR_INTERFACE_H
#define __LINUX_HALLSENSOR_INTERFACE_H

typedef struct{
	
}HALLsensor_Interface;

extern int HALLsensor_Interface_register();
extern int HALLsensor_Interface_unregister();

#endif