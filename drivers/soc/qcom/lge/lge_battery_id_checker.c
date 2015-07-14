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

#include <soc/qcom/lge/lge_battery_id_checker.h>
#ifdef CONFIG_LGE_PM_CABLE_DETECTION
#include <soc/qcom/lge/lge_cable_detection.h>
#endif

int lge_battery_info = BATT_ID_UNKNOWN;
bool is_lge_battery_valid(void)
{
#ifdef CONFIG_LGE_PM_CABLE_DETECTION
	if (lge_pm_get_cable_type() == CABLE_56K ||
			lge_pm_get_cable_type() == CABLE_130K ||
			lge_pm_get_cable_type() == CABLE_910K)
		return true;
#endif

	if (lge_battery_info == BATT_ID_DS2704_N ||
			lge_battery_info == BATT_ID_DS2704_L ||
			lge_battery_info == BATT_ID_DS2704_C ||
			lge_battery_info == BATT_ID_ISL6296_N ||
			lge_battery_info == BATT_ID_ISL6296_L ||
			lge_battery_info == BATT_ID_ISL6296_C ||
			lge_battery_info == BATT_ID_RA4301_VC0 ||
			lge_battery_info == BATT_ID_RA4301_VC1 ||
			lge_battery_info == BATT_ID_RA4301_VC2 ||
			lge_battery_info == BATT_ID_SW3800_VC0 ||
			lge_battery_info == BATT_ID_SW3800_VC1 ||
			lge_battery_info == BATT_ID_SW3800_VC2)
		return true;

	return false;
}

int read_lge_battery_id(void)
{
	return lge_battery_info;
}

static int __init battery_information_setup(char *batt_info)
{
	if (!strcmp(batt_info, "DS2704_N"))
		lge_battery_info = BATT_ID_DS2704_N;
	else if (!strcmp(batt_info, "DS2704_L"))
		lge_battery_info = BATT_ID_DS2704_L;
	else if (!strcmp(batt_info, "DS2704_C"))
		lge_battery_info = BATT_ID_DS2704_C;
	else if (!strcmp(batt_info, "ISL6296_N"))
		lge_battery_info = BATT_ID_ISL6296_N;
	else if (!strcmp(batt_info, "ISL6296_L"))
		lge_battery_info = BATT_ID_ISL6296_L;
	else if (!strcmp(batt_info, "ISL6296_C"))
		lge_battery_info = BATT_ID_ISL6296_C;
	else if (!strcmp(batt_info, "RA4301_VC0"))
		lge_battery_info = BATT_ID_RA4301_VC0;
	else if (!strcmp(batt_info, "RA4301_VC1"))
		lge_battery_info = BATT_ID_RA4301_VC1;
	else if (!strcmp(batt_info, "RA4301_VC2"))
		lge_battery_info = BATT_ID_RA4301_VC2;
	else if (!strcmp(batt_info, "SW3800_VC0"))
		lge_battery_info = BATT_ID_SW3800_VC0;
	else if (!strcmp(batt_info, "SW3800_VC1"))
		lge_battery_info = BATT_ID_SW3800_VC1;
	else if (!strcmp(batt_info, "SW3800_VC2"))
		lge_battery_info = BATT_ID_SW3800_VC2;
	else if (!strcmp(batt_info, "NOT_PRESENT"))
		lge_battery_info = BATT_NOT_PRESENT;
	else
		lge_battery_info = BATT_ID_UNKNOWN;

	pr_info("Battery : %s %d\n", batt_info, lge_battery_info);

	return 1;
}

__setup("lge.battid=", battery_information_setup);
