/* arch/arm/mach-msm/lge/lge_kcal_ctrl.c
 *
 * Interface to calibrate display color temperature.
 *
 * Copyright (C) 2012 LGE
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

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <mach/board_lge.h>

MODULE_DESCRIPTION("LGE MISC driver");
MODULE_LICENSE("GPL v2");

