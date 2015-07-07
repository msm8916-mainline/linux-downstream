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

/*************************/
/* IR Sensor Interfacee */
/************************/
#ifndef __LINUX_IRSENSOR_INTERFACE_H
#define __LINUX_IRSENSOR_INTERFACE_H

typedef struct{
	
}IRsensor_Interface;

extern int IRsensor_Interface_register(IRsensor_Interface *mInterface);
extern int IRsensor_Interface_unregister(void);

#endif

