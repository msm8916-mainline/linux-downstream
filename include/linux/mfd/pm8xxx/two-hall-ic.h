/* include/linux/mfd/pm8xxx/cradle.h
 *
 * Copyright (c) 2011-2012, LG Electronics Inc, All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __PM8XXX_CRADLE_H__
#define __PM8XXX_CRADLE_H__

#define HALL_IC_DEV_NAME "bu52061nvx"

/* SMART COVER Support */
#define SMARTCOVER_BOTTOM_CLOSED	4
#define SMARTCOVER_TOP_CLOSED		3
#define SMARTCOVER_OPENED			0

/* SMART COVER FACTORY Support */
#define SMARTCOVER_FATORY_CLOSED	1
#define SMARTCOVER_FATORY_RIGHT	2
#define SMARTCOVER_FATORY_LEFT		3
#define SMARTCOVER_FATORY_OPENED	0

struct pm8xxx_cradle_platform_data {
	int hallic_left_detect_pin;
	int hallic_right_detect_pin;
	unsigned int hallic_left_irq;
	unsigned int hallic_right_irq;
	unsigned long irq_flags;
};

void cradle_set_deskdock(int state);
int cradle_get_deskdock(void);


#endif /* __PM8XXX_CRADLE_H__ */
