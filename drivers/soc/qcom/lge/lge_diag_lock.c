/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/platform_device.h>

static struct platform_device lg_diag_cmd_device = {
	.name = "lg_diag_cmd",
	.id = -1,
	.dev    = {
		.platform_data = 0, /* &lg_diag_cmd_pdata */
	},
};

static int __init lge_diag_devices_init(void)
{
	return platform_device_register(&lg_diag_cmd_device);
}
arch_initcall(lge_diag_devices_init);
