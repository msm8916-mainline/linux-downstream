/* Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef OEM_MDSS_PANEL_INFO_H
#define OEM_MDSS_PANEL_INFO_H

#include <linux/list.h>
#include <linux/mdss_io_util.h>
#include <linux/irqreturn.h>
#include <linux/pinctrl/consumer.h>
#include <linux/gpio.h>

enum lcd_panel_type {
	JDI_INCELL_CMD_PANEL,
	JDI_INCELL_VIDEO_PANEL,
	LGD_INCELL_CMD_PANEL
};

struct lge_pan_info {
/*to do */
};

#endif /* OEM_MDSS_PANEL_INFO_H */
