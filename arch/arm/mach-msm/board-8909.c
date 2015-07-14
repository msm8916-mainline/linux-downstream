/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
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
#if defined(CONFIG_KEYBOARD_PP2106)
#include <linux/input/pp2106-keypad.h>
#include <linux/input.h>
#include <soc/qcom/lge/lge_board_revision.h>
#endif
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <asm/mach/arch.h>
#include <soc/qcom/socinfo.h>
#include <mach/board.h>
#include <mach/msm_memtypes.h>
#include <soc/qcom/rpm-smd.h>
#include <soc/qcom/smd.h>
#include <soc/qcom/smem.h>
#include <soc/qcom/spm.h>
#include <soc/qcom/pm.h>
#include "board-dt.h"
#include "platsmp.h"


#if defined(CONFIG_KEYBOARD_PP2106)
static unsigned short pp2106_keycode[PP2106_KEYPAD_ROW][PP2106_KEYPAD_COL] = {
	{ KEY_LEFT, KEY_UP, KEY_F3, KEY_DOWN, KEY_F1},
	{ KEY_ENTER, KEY_RIGHT, KEY_F4, KEY_EXIT, KEY_SEND},
	{ KEY_F2, KEY_MENU, KEY_3, KEY_2, KEY_1},
	{ KEY_BACK, KEY_HOME, KEY_6, KEY_5, KEY_4},
	{ 0, 0, KEY_9, KEY_8, KEY_7},
	{ 0, 0, KEY_F6, KEY_0, KEY_F5},
};
static struct pp2106_keypad_platform_data pp2106_pdata = {
	.keypad_row = PP2106_KEYPAD_ROW,
	.keypad_col = PP2106_KEYPAD_COL,
	.keycode = (unsigned char *)pp2106_keycode,
	.reset_pin = 967,/*56*/
	.irq_pin = 969,/*58*/
	.sda_pin = 925,/*14*/
	.scl_pin = 926,/*15*/
};

static struct pp2106_keypad_platform_data pp2106_pdata_evb = {
	.keypad_row = PP2106_KEYPAD_ROW,
	.keypad_col = PP2106_KEYPAD_COL,
	.keycode = (unsigned char *)pp2106_keycode,
	.reset_pin = 988,/*77*/
	.irq_pin = 987,/*76*/
	.sda_pin = 917,/*6*/
	.scl_pin = 918,/*7*/
};

static struct platform_device pp2106_keypad_device = {
	.name = "pp2106-keypad",
	.id = 0,
	.dev = {
		.platform_data = &pp2106_pdata,
	},
};

static struct platform_device pp2106_keypad_device_evb = {
	.name = "pp2106-keypad",
	.id = 0,
	.dev = {
		.platform_data = &pp2106_pdata_evb,
	},
};
#endif

static void __init msm8909_dt_reserve(void)
{
	of_scan_flat_dt(dt_scan_for_memory_reserve, NULL);
}

static void __init msm8909_map_io(void)
{
	msm_map_msm8909_io();
}

static struct of_dev_auxdata msm8909_auxdata_lookup[] __initdata = {
	{}
};

/*
 * Used to satisfy dependencies for devices that need to be
 * run early or in a particular order. Most likely your device doesn't fall
 * into this category, and thus the driver should not be added here. The
 * EPROBE_DEFER can satisfy most dependency problems.
 */
void __init msm8909_add_drivers(void)
{
	msm_smd_init();
	msm_rpm_driver_init();
	msm_spm_device_init();
	msm_pm_sleep_status_init();
}

static void __init msm8909_init(void)
{
	struct of_dev_auxdata *adata = msm8909_auxdata_lookup;

	/*
	 * populate devices from DT first so smem probe will get called as part
	 * of msm_smem_init.  socinfo_init needs smem support so call
	 * msm_smem_init before it.
	 */
	of_platform_populate(NULL, of_default_bus_match_table, adata, NULL);
	msm_smem_init();

	if (socinfo_init() < 0)
		pr_err("%s: socinfo_init() failed\n", __func__);

	msm8909_add_drivers();
#if defined(CONFIG_KEYBOARD_PP2106)
	if (lge_get_board_revno() == HW_REV_D)
		platform_device_register(&pp2106_keypad_device_evb);
	else
		platform_device_register(&pp2106_keypad_device);
#endif
}

static const char *msm8909_dt_match[] __initconst = {
	"qcom,msm8909",
	NULL
};

DT_MACHINE_START(MSM8909_DT,
	"Qualcomm Technologies, Inc. MSM 8909 (Flattened Device Tree)")
	.map_io = msm8909_map_io,
	.init_machine = msm8909_init,
	.dt_compat = msm8909_dt_match,
	.reserve = msm8909_dt_reserve,
	.smp = &msm8916_smp_ops,
MACHINE_END
