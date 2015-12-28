/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
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
#include "../board-dt.h"
#include "../platsmp.h"

#include <mach/board_lge.h>

#if defined(CONFIG_LCD_KCAL)
#include <linux/module.h>
#include "../../../../drivers/video/msm/mdss/mdss_fb.h"
extern int update_preset_lcdc_lut(void);
#endif

#ifdef CONFIG_LGE_LCD_TUNING
#include "../../../../drivers/video/msm/mdss/mdss_dsi.h"
int tun_lcd[128];

int lcd_set_values(int *tun_lcd_t)
{
	memset(tun_lcd,0,128*sizeof(int));
	memcpy(tun_lcd,tun_lcd_t,128*sizeof(int));
	printk("lcd_set_values ::: tun_lcd[0]=[%x], tun_lcd[1]=[%x], tun_lcd[2]=[%x] ......\n"
			,tun_lcd[0],tun_lcd[1],tun_lcd[2]);
	return 0;
}
static int lcd_get_values(int *tun_lcd_t)
{
	memset(tun_lcd_t,0,128*sizeof(int));
	memcpy(tun_lcd_t,tun_lcd,128*sizeof(int));
	printk("lcd_get_values\n");
	return 0;
}

static struct lcd_platform_data lcd_pdata ={
	.set_values = lcd_set_values,
	.get_values = lcd_get_values,
};
static struct platform_device lcd_ctrl_device = {
	.name = "lcd_ctrl",
	.dev = {
	.platform_data = &lcd_pdata,
	}
};

void __init lge_add_lcd_ctrl_devices(void)
{
	platform_device_register(&lcd_ctrl_device);
}
#endif

static void __init msm8916_dt_reserve(void)
{
	of_scan_flat_dt(dt_scan_for_memory_reserve, NULL);
#ifdef CONFIG_MACH_LGE
	of_scan_flat_dt(lge_init_dt_scan_chosen, NULL);
#endif
}

static void __init msm8916_map_io(void)
{
	msm_map_msm8916_io();
}

static struct of_dev_auxdata msm8916_auxdata_lookup[] __initdata = {
	{}
};

#if defined(CONFIG_LCD_KCAL)
extern int g_kcal_r;
extern int g_kcal_g;
extern int g_kcal_b;

int kcal_set_values(int kcal_r, int kcal_g, int kcal_b)
{
	g_kcal_r = kcal_r;
	g_kcal_g = kcal_g;
	g_kcal_b = kcal_b;
	printk("kcal_set_values ::: red=[%d], green=[%d], blue=[%d]\n", g_kcal_r, g_kcal_g, g_kcal_b);
	return 0;
}

static int kcal_get_values(int *kcal_r, int *kcal_g, int *kcal_b)
{
	*kcal_r = g_kcal_r;
	*kcal_g = g_kcal_g;
	*kcal_b = g_kcal_b;
	printk("kcal_get_values\n");
	return 0;
}

static int kcal_refresh_values(void)
{
	printk("kcal_refresh_values\n");
	return update_preset_lcdc_lut();
}

static struct kcal_platform_data kcal_pdata = {
	.set_values = kcal_set_values,
	.get_values = kcal_get_values,
	.refresh_display = kcal_refresh_values
};

static struct platform_device kcal_platrom_device = {
	.name   = "kcal_ctrl",
	.dev = {
		.platform_data = &kcal_pdata,
	}
};

void __init lge_add_lcd_kcal_devices(void)
{
	pr_info (" KCAL_DEBUG : %s \n", __func__);
	platform_device_register(&kcal_platrom_device);
}
#endif

#if defined(CONFIG_PRE_SELF_DIAGNOSIS)
int pre_selfd_set_values(int kcal_r, int kcal_g, int kcal_b)
{
	return 0;
}

static int pre_selfd_get_values(int *kcal_r, int *kcal_g, int *kcal_b)
{
	return 0;
}

static struct pre_selfd_platform_data pre_selfd_pdata = {
	.set_values = pre_selfd_set_values,
	.get_values = pre_selfd_get_values,
};


static struct platform_device pre_selfd_platrom_device = {
	.name   = "pre_selfd_ctrl",
	.dev = {
		.platform_data = &pre_selfd_pdata,
	}
};

void __init lge_add_pre_selfd_devices(void)
{
	pr_info(" PRE_SELFD_DEBUG : %s\n", __func__);
	platform_device_register(&pre_selfd_platrom_device);
}
#endif /* CONFIG_PRE_SELF_DIAGNOSIS */

/*
 * Used to satisfy dependencies for devices that need to be
 * run early or in a particular order. Most likely your device doesn't fall
 * into this category, and thus the driver should not be added here. The
 * EPROBE_DEFER can satisfy most dependency problems.
 */
void __init msm8916_add_drivers(void)
{
	msm_smd_init();
	msm_rpm_driver_init();
	msm_spm_device_init();
	msm_pm_sleep_status_init();
#ifdef CONFIG_LGE_LCD_TUNING
	 lge_add_lcd_ctrl_devices();
#endif
#ifdef CONFIG_USB_G_LGE_ANDROID
	 lge_add_android_usb_devices();
#endif
#if defined(CONFIG_LCD_KCAL)
	 lge_add_lcd_kcal_devices();
#endif
#ifdef CONFIG_LGE_QFPROM_INTERFACE
	lge_add_qfprom_devices();
#endif
#if defined(CONFIG_PRE_SELF_DIAGNOSIS)
	lge_add_pre_selfd_devices();
#endif
}

static void __init msm8916_init(void)
{
	struct of_dev_auxdata *adata = msm8916_auxdata_lookup;

	/*
	 * populate devices from DT first so smem probe will get called as part
	 * of msm_smem_init.  socinfo_init needs smem support so call
	 * msm_smem_init before it.
	 */
	of_platform_populate(NULL, of_default_bus_match_table, adata, NULL);
	msm_smem_init();

	if (socinfo_init() < 0)
		pr_err("%s: socinfo_init() failed\n", __func__);

	msm8916_add_drivers();
}

static const char *msm8916_dt_match[] __initconst = {
	"qcom,msm8916",
	NULL
};

DT_MACHINE_START(MSM8916_DT, "Qualcomm MSM 8916 (Flattened Device Tree)")
	.map_io = msm8916_map_io,
	.init_machine = msm8916_init,
	.dt_compat = msm8916_dt_match,
	.reserve = msm8916_dt_reserve,
	.smp = &msm8916_smp_ops,
MACHINE_END
