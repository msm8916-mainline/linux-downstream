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
#include <linux/string.h>

static unsigned int on_hidden_reset;

unsigned int lge_get_hidden_reset_mode(void)
{
	return on_hidden_reset;
}
EXPORT_SYMBOL(lge_get_hidden_reset_mode);

static int __init lge_check_hidden_reset(char *reset_mode)
{
	if (!strncmp(reset_mode, "on", 2))
		on_hidden_reset = 1;

	return 1;
}
__setup("lge.hreset=", lge_check_hidden_reset);