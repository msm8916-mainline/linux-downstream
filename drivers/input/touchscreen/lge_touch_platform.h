/* Lge_touch_platform.h
 *
 * Copyright (C) 2011 LGE.
 *
 * Author: yehan.ahn@lge.com, hyesung.shin@lge.com
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
#ifndef LGE_TOUCH_PLAFORM_H
#define LGE_TOUCH_PLAFORM_H

#include <linux/input/lge_touch_core.h>

int ts_dma_allocation(void);
int ts_i2c_msg_transfer(struct i2c_client *client, struct i2c_msg *msgs, int count);

int ts_gpio_init(unsigned gpio, const char *label,
				enum gpio_type type);
int ts_gpio_direction_output(unsigned gpio, int value);
int ts_gpio_direction_input(unsigned gpio);
void ts_gpio_set_value(unsigned gpio, int value);
int ts_gpio_get_value(unsigned gpio);

void ts_enable_irq(unsigned int irq_num);
void ts_disable_irq(unsigned int irq_num);
int ts_register_irq(struct lge_touch_data *ts, void* handler, void* threaded_handler);

#endif //LGE_TOUCH_PLAFORM_H
