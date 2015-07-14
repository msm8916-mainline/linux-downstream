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

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>

#define XO_CLK_RATE	19200000
#define INIT_FUNC(x, y) ((x->y) = y)

#include "oem_mdss_dsi_common.h"
#include "oem_mdss_dsi.h"

static struct panel_list supp_panels[] = {
	{"TIANMA 320p cmd mode dsi panel", TIANMA_ILI9488_HVGA_CMD_PANEL},
	{"TIANMA 320p video mode dsi panel", TIANMA_ILI9488_HVGA_VIDEO_PANEL},
};

static int panel_id;

int panel_name_to_id(struct panel_list supp_panels[],
			  uint32_t supp_panels_size,
			  const char *panel_name)
{
	int i;
	int panel_id = UNKNOWN_PANEL;

	if (!panel_name) {
		pr_err("Invalid panel name\n");
		return panel_id;
	}

	/* Remove any leading whitespaces */
	panel_name += strspn(panel_name, " ");
	for (i = 0; i < supp_panels_size; i++) {
		if (!strncmp(panel_name, supp_panels[i].name,
			MAX_PANEL_ID_LEN)) {
			panel_id = supp_panels[i].id;
			break;
		}
	}

	return panel_id;
}

int pre_mdss_dsi_panel_power_ctrl(struct mdss_panel_data *pdata, int enable)
{
	int rc = 0;

	switch (panel_id) {
	case TIANMA_ILI9488_HVGA_CMD_PANEL:
		rc = tianma_hvga_cmd_pre_mdss_dsi_panel_power_ctrl(pdata, enable);
		break;

	default:
		break;
	}

	return rc;

}

int post_mdss_dsi_panel_power_ctrl(struct mdss_panel_data *pdata, int enable)
{
	int rc = 0;

	switch (panel_id) {
	case TIANMA_ILI9488_HVGA_CMD_PANEL:
		rc = tianma_hvga_cmd_post_mdss_dsi_panel_power_ctrl(pdata, enable);
		break;

	default:
		break;
	}

	return rc;

}

int pre_mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable)
{
	int rc = 0;

	switch (panel_id) {
	case TIANMA_ILI9488_HVGA_CMD_PANEL:
		break;

	default:
		break;
	}

	return rc;
}

int lge_msm_dss_enable_vreg(struct mdss_dsi_ctrl_pdata *ctrl_pdata, int enable)
{
	int rc = 0;

	switch (panel_id) {
	case TIANMA_ILI9488_HVGA_CMD_PANEL:
		break;

	default:
		break;
	}

	return rc;

}

int lge_mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable)
{
	int rc = 0;

	switch (panel_id) {
	case TIANMA_ILI9488_HVGA_CMD_PANEL:
		rc = tianma_hvga_cmd_mdss_dsi_panel_reset(pdata, enable);
		break;

	default:
		break;
	}

	return rc;
}

int lge_mdss_dsi_request_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int rc = 0;

	switch (panel_id) {
	case TIANMA_ILI9488_HVGA_CMD_PANEL:
		rc = tianma_hvga_cmd_mdss_dsi_request_gpios(ctrl_pdata);
		break;

	default:
		break;
	}

	return rc;
}

int lge_mdss_panel_parse_dt(struct device_node *np,
			struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int rc = 0;

	switch (panel_id) {
	case TIANMA_ILI9488_HVGA_CMD_PANEL:
		rc = tianma_hvga_cmd_mdss_panel_parse_dts(np, ctrl_pdata);
		break;

	default:
		break;
	}

	return rc;

}

int lge_panel_device_create(struct device_node *node,
	struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int rc = 0;

	switch (panel_id) {
	case TIANMA_ILI9488_HVGA_CMD_PANEL:
		rc = tianma_hvga_cmd_panel_device_create(node, ctrl_pdata);
		break;

	default:
		break;
	}

	return rc;

}

int lge_dsi_panel_device_register(struct device_node *pan_node,
				struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int rc = 0;

	switch (panel_id) {
	case TIANMA_ILI9488_HVGA_CMD_PANEL:
		rc = tianma_hvga_cmd_dsi_panel_device_register(pan_node, ctrl_pdata);
		break;

	default:
		break;
	}

	return rc;

}

int get_lge_panel_id(void)
{
	return panel_id;
}
void lge_mdss_dsi_seperate_panel_api_init(struct lge_mdss_dsi_interface *pdata, struct device_node *dsi_pan_node)
{
	static const char *panel_name;

	panel_name = of_get_property(dsi_pan_node, "qcom,mdss-dsi-panel-name", NULL);

	panel_id = panel_name_to_id(supp_panels,
			ARRAY_SIZE(supp_panels), panel_name);

	switch (panel_id) {
	case TIANMA_ILI9488_HVGA_CMD_PANEL:
		INIT_FUNC(pdata, lge_mdss_dsi_panel_reset);
		INIT_FUNC(pdata, lge_mdss_dsi_request_gpios);
		INIT_FUNC(pdata, lge_mdss_panel_parse_dt);
		INIT_FUNC(pdata, lge_panel_device_create);
		INIT_FUNC(pdata, lge_dsi_panel_device_register);
		break;

	default:
		break;
	}
}
