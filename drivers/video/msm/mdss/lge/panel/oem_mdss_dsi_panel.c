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
#include <linux/leds.h>

#include "oem_mdss_dsi_common.h"
#include "oem_mdss_dsi.h"

#define PIN_DDVDH "DDVDH"
#define PIN_RESET "RESET"
#define PANEL_SEQUENCE(name, state) pr_info("[PanelSequence][%s] %d\n", name, state);

int tianma_hvga_cmd_mdss_dsi_request_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int rc = 0;
	struct mdss_panel_info *pinfo = NULL;
	pinfo = &(ctrl_pdata->panel_data.panel_info);

	if (gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
		rc = gpio_request(ctrl_pdata->disp_en_gpio,
						"disp_enable");
		if (rc) {
			pr_err("request disp_en gpio failed, rc=%d\n",
				       rc);
			goto disp_en_gpio_err;
		}
	}

	rc = gpio_request(ctrl_pdata->rst_gpio, "disp_rst_n");
	if (rc) {
		pr_err("request reset gpio failed, rc=%d\n",
			rc);
		goto rst_gpio_err;
	}	

	if (gpio_is_valid(ctrl_pdata->bklt_en_gpio)) {
		rc = gpio_request(ctrl_pdata->bklt_en_gpio,
						"bklt_enable");
		if (rc) {
			pr_err("request bklt gpio failed, rc=%d\n",
				       rc);
			goto bklt_en_gpio_err;
		}
	}
	if (gpio_is_valid(ctrl_pdata->mode_gpio)) {
		rc = gpio_request(ctrl_pdata->mode_gpio, "panel_mode");
		if (rc) {
			pr_err("request panel mode gpio failed,rc=%d\n",
								rc);
			goto mode_gpio_err;
		}
	}
	return rc;

mode_gpio_err:
	if (gpio_is_valid(ctrl_pdata->bklt_en_gpio))
		gpio_free(ctrl_pdata->bklt_en_gpio);
bklt_en_gpio_err:
	gpio_free(ctrl_pdata->rst_gpio);
rst_gpio_err:
	if (gpio_is_valid(ctrl_pdata->disp_en_gpio))
		gpio_free(ctrl_pdata->disp_en_gpio);
disp_en_gpio_err:
	return rc;

}

int tianma_hvga_cmd_mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo = NULL;
	int i, rc = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (!gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
	}

	if (!gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
		return rc;
	}

	pr_info("%s: enable = %d\n", __func__, enable);
	pinfo = &(ctrl_pdata->panel_data.panel_info);

	if (enable) {
		if (lge_mdss_dsi.lge_mdss_dsi_request_gpios)
			lge_mdss_dsi.lge_mdss_dsi_request_gpios(ctrl_pdata);
		if (rc) {
			pr_err("gpio request failed\n");
			return rc;
		}

		if (!pinfo->cont_splash_enabled) {
			if (gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
				gpio_set_value((ctrl_pdata->disp_en_gpio), 1);
				PANEL_SEQUENCE(PIN_DDVDH, 1);
			}

		gpio_set_value((ctrl_pdata->rst_gpio), 0);
		PANEL_SEQUENCE(PIN_RESET, 0);
		mdelay(5);

		for (i = 0; i < pdata->panel_info.rst_seq_len; ++i) {
			gpio_set_value((ctrl_pdata->rst_gpio),
				pdata->panel_info.rst_seq[i]);
				PANEL_SEQUENCE(PIN_RESET, pdata->panel_info.rst_seq[i]);
			if (pdata->panel_info.rst_seq[++i])
				usleep(pinfo->rst_seq[i] * 1000);
		}

		if (gpio_is_valid(ctrl_pdata->bklt_en_gpio))
			gpio_set_value((ctrl_pdata->bklt_en_gpio), 1);
		}

		if (gpio_is_valid(ctrl_pdata->mode_gpio)) {
			if (pinfo->mode_gpio_state == MODE_GPIO_HIGH)
				gpio_set_value((ctrl_pdata->mode_gpio), 1);
			else if (pinfo->mode_gpio_state == MODE_GPIO_LOW)
				gpio_set_value((ctrl_pdata->mode_gpio), 0);
		}
		if (ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT) {
			pr_debug("%s: Panel Not properly turned OFF\n",
						__func__);
			ctrl_pdata->ctrl_state &= ~CTRL_STATE_PANEL_INIT;
			pr_debug("%s: Reset panel done\n", __func__);
		}
	} else {
		if (gpio_is_valid(ctrl_pdata->bklt_en_gpio)) {
			gpio_set_value((ctrl_pdata->bklt_en_gpio), 0);
			gpio_free(ctrl_pdata->bklt_en_gpio);
		}

		if (gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
			gpio_set_value((ctrl_pdata->disp_en_gpio), 0);
			PANEL_SEQUENCE(PIN_DDVDH, 0);
			gpio_free(ctrl_pdata->disp_en_gpio);
		}

		gpio_set_value((ctrl_pdata->rst_gpio), 0);
		PANEL_SEQUENCE(PIN_RESET, 0);
		gpio_free(ctrl_pdata->rst_gpio);

		if (gpio_is_valid(ctrl_pdata->mode_gpio))
			gpio_free(ctrl_pdata->mode_gpio);
	}
	return rc;

}

int jdi_qhd_dual_cmd_mdss_dsi_request_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int rc = 0;
	return rc;
}

int lgd_qhd_dual_cmd_mdss_dsi_request_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int rc = 0;
	return rc;
}

int tianma_hvga_cmd_mdss_panel_parse_dts(struct device_node *np,
			struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	return 0;
}

int tianma_hvga_cmd_panel_device_create(struct device_node *node,
	struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{

	return 0;
}

int lgd_jdi_qhd_cmd_panel_device_create(struct device_node *node,
	struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{

	return 0;
}
