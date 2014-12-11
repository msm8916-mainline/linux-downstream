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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fb.h>
#include <linux/notifier.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/iopoll.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>

#include "mdss_fb.h"
#include "mdss_dsi.h"
#include "mdss_panel.h"
#include "mdss_mdp.h"

#define STATUS_CHECK_INTERVAL_MS 5000
#define STATUS_CHECK_INTERVAL_MIN_MS 200
#define DSI_STATUS_CHECK_DISABLE 0
#if defined(CONFIG_FB_MSM_MDSS_SAMSUNG)
#define STATUS_CHECK_INTERVAL_MS_FOR_IRQ 500
#endif

static uint32_t interval = STATUS_CHECK_INTERVAL_MS;
static uint32_t dsi_status_disable = DSI_STATUS_CHECK_DISABLE;
struct dsi_status_data *pstatus_data;

#if defined(CONFIG_FB_MSM_MDSS_SAMSUNG)
/*
 * esd_irq_enable() - Enable or disable esd irq.
 *
 * @enable	: flag for enable or disabled
 * @nosync	: flag for disable irq with nosync
 * @data	: point ot struct mdss_panel_info
 */
static void esd_irq_enable(bool enable, bool nosync, void *data)
{
	/* The irq will enabled when do the request_threaded_irq() */
	static bool is_enabled = true;
	int gpio;
	unsigned long flags;
	struct mdss_panel_info *pinfo = (struct mdss_panel_info*)data;

	if (!pinfo) {
		pr_err("%s: pinfo is null\n", __func__);
		return;
	}

	spin_lock_irqsave(&pinfo->esd_recovery.irq_lock, flags);
	gpio = pinfo->esd_recovery.esd_gpio;

	if (enable == is_enabled) {
		pr_info("%s: ESD irq already %s\n",
				__func__, enable ? "enabled" : "disabled");
		goto error;
	}

	if (enable) {
		is_enabled = true;
		enable_irq(gpio_to_irq(gpio));
	} else {
		if (nosync)
			disable_irq_nosync(gpio_to_irq(gpio));
		else
			disable_irq(gpio_to_irq(gpio));
		is_enabled = false;
	}

	/* TODO: Disable log if the esd function stable */
	pr_info("%s: ESD irq %s with %s\n",
				__func__,
				enable ? "enabled" : "disabled",
				nosync ? "nosync" : "sync");
error:
	spin_unlock_irqrestore(&pinfo->esd_recovery.irq_lock, flags);

}

static irqreturn_t esd_irq_handler(int irq, void *handle)
{
	struct mdss_panel_info *pinfo;

	if (!handle) {
		pr_info("handle is null\n");
		return IRQ_HANDLED;
	}

	pinfo = (struct mdss_panel_info *)handle;

	pr_debug("%s++\n", __func__);
	if (!pinfo->esd_recovery.is_enabled_esd_recovery) {
		pr_info("%s: esd recovery is not enabled yet", __func__);
		return IRQ_HANDLED;
	}

	esd_irq_enable(false, true, (void *)pinfo);

	schedule_work(&pstatus_data->check_status.work);
	pr_debug("%s--\n", __func__);

	return IRQ_HANDLED;
}
#endif

/*
 * check_dsi_ctrl_status() - Reads MFD structure and
 * calls platform specific DSI ctrl Status function.
 * @work  : dsi controller status data
 */
static void check_dsi_ctrl_status(struct work_struct *work)
{
	struct dsi_status_data *pdsi_status = NULL;

	pdsi_status = container_of(to_delayed_work(work),
		struct dsi_status_data, check_status);

	if (!pdsi_status) {
		pr_err("%s: DSI status data not available\n", __func__);
		return;
	}

	if (!pdsi_status->mfd) {
		pr_err("%s: FB data not available\n", __func__);
		return;
	}

	if (pdsi_status->mfd->shutdown_pending ||
		!pdsi_status->mfd->panel_power_on) {
		pr_err("%s: panel off\n", __func__);
		return;
	}

	pdsi_status->mfd->mdp.check_dsi_status(work, interval);
}

/*
 * fb_event_callback() - Call back function for the fb_register_client()
 *			 notifying events
 * @self  : notifier block
 * @event : The event that was triggered
 * @data  : Of type struct fb_event
 *
 * This function listens for FB_BLANK_UNBLANK and FB_BLANK_POWERDOWN events
 * from frame buffer. DSI status check work is either scheduled again after
 * PANEL_STATUS_CHECK_INTERVAL or cancelled based on the event.
 */
static int fb_event_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	struct dsi_status_data *pdata = container_of(self,
				struct dsi_status_data, fb_notifier);
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo;
	struct msm_fb_data_type *mfd;
#if defined(CONFIG_FB_MSM_MDSS_SAMSUNG)
	int ret;
#endif

	mfd = evdata->info->par;
	ctrl_pdata = container_of(dev_get_platdata(&mfd->pdev->dev),
				struct mdss_dsi_ctrl_pdata, panel_data);
	if (!ctrl_pdata) {
		pr_err("%s: DSI ctrl not available\n", __func__);
		return NOTIFY_BAD;
	}

	pinfo = &ctrl_pdata->panel_data.panel_info;

	if (!(pinfo->esd_check_enabled)) {
		pr_debug("ESD check is not enaled in panel dtsi\n");
		return NOTIFY_DONE;
	}

	if (dsi_status_disable) {
		pr_debug("%s: DSI status disabled\n", __func__);
		return NOTIFY_DONE;
	}

#if defined(CONFIG_FB_MSM_MDSS_SAMSUNG)
	if (unlikely(!pinfo->esd_recovery.esd_recovery_init)) {
		pinfo->esd_recovery.esd_recovery_init = true;
		pinfo->esd_recovery.esd_irq_enable = esd_irq_enable;
		if (ctrl_pdata->status_mode == ESD_REG_IRQ) {
			if (gpio_is_valid(pinfo->esd_recovery.esd_gpio)) {
				gpio_request(pinfo->esd_recovery.esd_gpio, "esd_recovery");
				ret = request_threaded_irq(
						gpio_to_irq(pinfo->esd_recovery.esd_gpio),
						NULL,
						esd_irq_handler,
						pinfo->esd_recovery.irqflags,
						"esd_recovery",
						(void *)pinfo);
				if (ret)
					pr_err("%s : Failed to request_irq, ret=%d\n",
							__func__, ret);
				else
					esd_irq_enable(false, false, (void *)pinfo);
				interval = STATUS_CHECK_INTERVAL_MS_FOR_IRQ;
			}
		}
	}
#endif

	pdata->mfd = evdata->info->par;

	if (event == FB_EVENT_BLANK && evdata) {
		int *blank = evdata->data;
		struct dsi_status_data *pdata = container_of(self,
				struct dsi_status_data, fb_notifier);
		pdata->mfd = evdata->info->par;

		switch (*blank) {
		case FB_BLANK_UNBLANK:
#if defined(CONFIG_FB_MSM_MDSS_SAMSUNG)
			if (ctrl_pdata->status_mode == ESD_REG_IRQ)
				esd_irq_enable(true, false, (void *)pinfo);
			else
#endif
				schedule_delayed_work(&pdata->check_status,
					msecs_to_jiffies(interval));
			break;
		case FB_BLANK_POWERDOWN:
		case FB_BLANK_HSYNC_SUSPEND:
		case FB_BLANK_VSYNC_SUSPEND:
		case FB_BLANK_NORMAL:
#if defined(CONFIG_FB_MSM_MDSS_SAMSUNG)
			if (ctrl_pdata->status_mode == ESD_REG_IRQ) {
				esd_irq_enable(false, false, (void *)pinfo);
				cancel_work_sync(&pdata->check_status.work);
			} else
#endif
				cancel_delayed_work(&pdata->check_status);
			break;
		default:
			pr_err("Unknown case in FB_EVENT_BLANK event\n");
			break;
		}
	}
	return 0;
}

static int param_dsi_status_disable(const char *val, struct kernel_param *kp)
{
	int ret = 0;
	int int_val;

	ret = kstrtos32(val, 0, &int_val);
	if (ret)
		return ret;

	pr_info("%s: Set DSI status disable to %d\n",
			__func__, int_val);
	*((int *)kp->arg) = int_val;
	return ret;
}

static int param_set_interval(const char *val, struct kernel_param *kp)
{
	int ret = 0;
	int int_val;

	ret = kstrtos32(val, 0, &int_val);
	if (ret)
		return ret;
	if (int_val < STATUS_CHECK_INTERVAL_MIN_MS) {
		pr_err("%s: Invalid value %d used, ignoring\n",
						__func__, int_val);
		ret = -EINVAL;
	} else {
		pr_info("%s: Set check interval to %d msecs\n",
						__func__, int_val);
		*((int *)kp->arg) = int_val;
	}
	return ret;
}

int __init mdss_dsi_status_init(void)
{
	int rc = 0;

	pstatus_data = kzalloc(sizeof(struct dsi_status_data), GFP_KERNEL);
	if (!pstatus_data) {
		pr_err("%s: can't allocate memory\n", __func__);
		return -ENOMEM;
	}

	pstatus_data->fb_notifier.notifier_call = fb_event_callback;

	rc = fb_register_client(&pstatus_data->fb_notifier);
	if (rc < 0) {
		pr_err("%s: fb_register_client failed, returned with rc=%d\n",
								__func__, rc);
		kfree(pstatus_data);
		return -EPERM;
	}

	pr_info("%s: DSI status check interval:%d\n", __func__,	interval);

	INIT_DELAYED_WORK(&pstatus_data->check_status, check_dsi_ctrl_status);

	pr_debug("%s: DSI ctrl status work queue initialized\n", __func__);

	return rc;
}

void __exit mdss_dsi_status_exit(void)
{
	fb_unregister_client(&pstatus_data->fb_notifier);
	cancel_delayed_work_sync(&pstatus_data->check_status);
	kfree(pstatus_data);
	pr_debug("%s: DSI ctrl status work queue removed\n", __func__);
}

module_param_call(interval, param_set_interval, param_get_uint,
						&interval, 0644);
MODULE_PARM_DESC(interval,
		"Duration in milliseconds to send BTA command for checking"
		"DSI status periodically");

module_param_call(dsi_status_disable, param_dsi_status_disable, param_get_uint,
						&dsi_status_disable, 0644);
MODULE_PARM_DESC(dsi_status_disable,
		"Disable DSI status check");

module_init(mdss_dsi_status_init);
module_exit(mdss_dsi_status_exit);

MODULE_LICENSE("GPL v2");
