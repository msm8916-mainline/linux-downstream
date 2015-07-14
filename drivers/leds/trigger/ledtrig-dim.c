/*
 * LED Dim Trigger
 *
 * Copyright (C) 2008 Bill Gatliff <bgat@xxxxxxxxxxxxxxx>
 *
 * "Dims" an LED based on system load. Derived from Atsushi Nemoto's
 * ledtrig-heartbeat.c.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include "../leds.h"

#define MPP_LED_FULL 40
#define LED_DIM_DELTA (MPP_LED_FULL / 8)

struct dim_trig_data {
	struct timer_list timer;
	unsigned int rebound;
	unsigned int brightness;
};

static void led_dim_function(unsigned long data)
{
	struct led_classdev *led_cdev = (struct led_classdev *)data;
	struct dim_trig_data *dim_data = led_cdev->trigger_data;

	if (!dim_data->rebound) {
		dim_data->brightness -= LED_DIM_DELTA;
		if (dim_data->brightness <= 0) {
			dim_data->rebound = 1;
			dim_data->brightness = 0;
		}
	} else {
		dim_data->brightness += LED_DIM_DELTA;
		if (dim_data->brightness >= MPP_LED_FULL) {
			dim_data->rebound = 0;
			dim_data->brightness = MPP_LED_FULL;
		}
	}

	led_set_brightness(led_cdev, dim_data->brightness);
	mod_timer(&dim_data->timer, jiffies + msecs_to_jiffies(100));
}


static void dim_trig_activate(struct led_classdev *led_cdev)
{
	struct dim_trig_data *dim_data;

	dim_data = kzalloc(sizeof(*dim_data), GFP_KERNEL);

	if (!dim_data)
		return;

	dim_data->rebound = 0;
	dim_data->brightness = MPP_LED_FULL;
	led_cdev->trigger_data = dim_data;
	setup_timer(&dim_data->timer, led_dim_function, (unsigned long)led_cdev);
	led_dim_function(dim_data->timer.data);
}

static void dim_trig_deactivate(struct led_classdev *led_cdev)
{
	struct dim_trig_data *dim_data = led_cdev->trigger_data;

	if (dim_data) {
		del_timer_sync(&dim_data->timer);
		kfree(dim_data);
	}
}

static struct led_trigger dim_led_trigger = {
	.name = "dim",
	.activate = dim_trig_activate,
	.deactivate = dim_trig_deactivate,
};


static int __init dim_trig_init(void)
{
	return led_trigger_register(&dim_led_trigger);
}


static void __exit dim_trig_exit(void)
{
	led_trigger_unregister(&dim_led_trigger);
}

module_init(dim_trig_init);
module_exit(dim_trig_exit);

MODULE_AUTHOR("Bill Gatliff <bgat@xxxxxxxxxxxxxxx>");
MODULE_DESCRIPTION("Dim LED trigger");
MODULE_LICENSE("GPL");
