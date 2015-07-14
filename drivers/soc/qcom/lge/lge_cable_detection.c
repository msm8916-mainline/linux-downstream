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

#include <soc/qcom/lge/lge_cable_detection.h>
#include <soc/qcom/lge/lge_board_revision.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/err.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/power_supply.h>

struct chg_cable_info_table {
	int threshhold;
	enum acc_cable_type type;
	unsigned ta_ma;
	unsigned usb_ma;
};

#define ADC_NO_INIT_CABLE   0
#define C_NO_INIT_TA_MA     0
#define C_NO_INIT_USB_MA    0
#define ADC_CABLE_NONE      1900000
#define C_NONE_TA_MA        700
#define C_NONE_USB_MA       500

#define MAX_CABLE_NUM		15
static bool cable_type_defined;
static struct chg_cable_info_table lge_acc_cable_type_data[MAX_CABLE_NUM];

void get_cable_data_from_dt(void *of_node)
{
	int i;
	u32 cable_value[3];
	struct device_node *node_temp = (struct device_node *)of_node;
	const char *propname[MAX_CABLE_NUM] = {
		"lge,no-init-cable",
		"lge,cable-mhl-1k",
		"lge,cable-u-28p7k",
		"lge,cable-28p7k",
		"lge,cable-56k",
		"lge,cable-100k",
		"lge,cable-130k",
		"lge,cable-180k",
		"lge,cable-200k",
		"lge,cable-220k",
		"lge,cable-270k",
		"lge,cable-330k",
		"lge,cable-620k",
		"lge,cable-910k",
		"lge,cable-none"
	};
	if (cable_type_defined) {
		pr_info("Cable type is already defined\n");
		return;
	}

	for (i = 0; i < MAX_CABLE_NUM; i++) {
		of_property_read_u32_array(node_temp, propname[i],
				cable_value, 3);
		lge_acc_cable_type_data[i].threshhold = cable_value[0];
		lge_acc_cable_type_data[i].type = i;
		lge_acc_cable_type_data[i].ta_ma = cable_value[1];
		lge_acc_cable_type_data[i].usb_ma = cable_value[2];
	}
	cable_type_defined = 1;
}

int lge_pm_get_cable_info(struct qpnp_vadc_chip *vadc,
		struct chg_cable_info *cable_info)
{
	char *type_str[] = {
		"NOT INIT", "MHL 1K", "U_28P7K", "28P7K", "56K",
		"100K", "130K", "180K", "200K", "220K",
		"270K", "330K", "620K", "910K", "OPEN"
	};

	struct qpnp_vadc_result result;
	struct chg_cable_info *info = cable_info;
	struct chg_cable_info_table *table;
	int table_size = ARRAY_SIZE(lge_acc_cable_type_data);
	int acc_read_value = 0;
	int i, rc;
	int count = 1;

	if (!info) {
		pr_err("%s : invalid info parameters\n", __func__);
		return -EINVAL;
	}

	if (!vadc) {
		pr_err("%s : invalid vadc parameters\n", __func__);
		return -EINVAL;
	}

	if (!cable_type_defined) {
		pr_err("%s : cable type is not defined yet.\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < count; i++) {
#ifndef CONFIG_MACH_MSM8909_LGPS22
		/* CF USB_ID is connected to MPP2 from rev.A */
		rc = qpnp_vadc_read(vadc, P_MUX2_1_1, &result);
#else
		if (lge_get_board_revno() > HW_REV_C)
			rc = qpnp_vadc_read(vadc, P_MUX3_1_1, &result);
		else
			rc = qpnp_vadc_read(vadc, P_MUX1_1_1, &result);
#endif
		if (rc < 0) {
			if (rc == -ETIMEDOUT) {
				/* reason: adc read timeout,
				 * assume it is open cable
				 */
				info->cable_type = CABLE_NONE;
				info->ta_ma = C_NONE_TA_MA;
				info->usb_ma = C_NONE_USB_MA;
			}
			pr_err("%s : adc read error - %d\n", __func__, rc);
			return rc;
		}

		acc_read_value = (int)result.physical;
		pr_info("%s : adc_read-%d\n", __func__, (int)result.physical);
		/* mdelay(10); */
	}

	info->cable_type = NO_INIT_CABLE;
	info->ta_ma = C_NO_INIT_TA_MA;
	info->usb_ma = C_NO_INIT_USB_MA;

	/* assume: adc value must be existed in ascending order */
	for (i = 0; i < table_size; i++) {
		table = &lge_acc_cable_type_data[i];

		if (acc_read_value <= table->threshhold) {
			info->cable_type = table->type;
			info->ta_ma = table->ta_ma;
			info->usb_ma = table->usb_ma;
			break;
		}
	}

	pr_err("\n\n[PM]Cable detected: %d(%s)(%d, %d)\n\n",
			acc_read_value, type_str[info->cable_type],
			info->ta_ma, info->usb_ma);

	return 0;
}

/* Belows are for using in interrupt context */
static struct chg_cable_info lge_cable_info;

enum acc_cable_type lge_pm_get_cable_type(void)
{
	return lge_cable_info.cable_type;
}

unsigned lge_pm_get_ta_current(void)
{
	return lge_cable_info.ta_ma;
}

unsigned lge_pm_get_usb_current(void)
{
	return lge_cable_info.usb_ma;
}

/* This must be invoked in process context */
void lge_pm_read_cable_info(struct qpnp_vadc_chip *vadc)
{
	lge_cable_info.cable_type = NO_INIT_CABLE;
	lge_cable_info.ta_ma = C_NO_INIT_TA_MA;
	lge_cable_info.usb_ma = C_NO_INIT_USB_MA;

	lge_pm_get_cable_info(vadc, &lge_cable_info);
}
