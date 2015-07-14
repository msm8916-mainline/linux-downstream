/* arch/arm/mach-msm/lge/lge_dock.c
 *
 * LGE Dock Driver.
 *
 * Copyright (C) 2013 LGE
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

#define pr_fmt(fmt) "%s %s: " fmt, "lge_dock", __func__

#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/switch.h>
#include <linux/power_supply.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <soc/qcom/lge/lge_cable_detection.h>

enum {
	EXTRA_DOCK_STATE_UNDOCKED = 0,
	EXTRA_DOCK_STATE_DESK = 1,
	EXTRA_DOCK_STATE_CAR = 2,
	EXTRA_DOCK_STATE_LE_DESK = 3,
	EXTRA_DOCK_STATE_HE_DESK = 4
};

static DEFINE_MUTEX(dock_lock);
static bool dock_state;

static struct switch_dev dockdev = {
	.name = "dock",
};

static int check_dock_cable_type(void)
{
	pr_debug("entered check_dock_cable_type");
#ifdef CONFIG_LGE_PM_CABLE_DETECTION
	if (lge_pm_get_cable_type() == CABLE_330K) {
		pr_debug("dock_state true");
		dock_state = true;
	} else
#endif
	{
		pr_debug("dock_state false");
		dock_state = false;
	}
	return 0;
}

void check_dock_connected(enum power_supply_type type)
{
	if (check_dock_cable_type() < 0)
		pr_err("can't read adc!\n");

	if ((dock_state) && type) {
		switch_set_state(&dockdev, EXTRA_DOCK_STATE_DESK);
		pr_info("desk dock\n");
	} else {
		switch_set_state(&dockdev, EXTRA_DOCK_STATE_UNDOCKED);
		pr_debug("undocked\n");
	}
	mutex_unlock(&dock_lock);
}

static int lge_dock_probe(struct platform_device *pdev)
{
	dock_state = false;
	return 0;
}

static int lge_dock_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver dock_driver = {
	.probe  = lge_dock_probe,
	.remove = lge_dock_remove,
	.driver = {
		.name = "lge_dock",
	},
};

static int __init lge_dock_init(void)
{
	int rc;
	rc = platform_driver_register(&dock_driver);
	if (switch_dev_register(&dockdev) < 0) {
		pr_err("failed to register dock driver.\n");
		rc = -ENODEV;
	}
	return rc;
}

static void __exit lge_dock_exit(void)
{
	switch_dev_unregister(&dockdev);
	platform_driver_unregister(&dock_driver);
}

module_init(lge_dock_init);
module_exit(lge_dock_exit);

MODULE_DESCRIPTION("LGE dock driver");
MODULE_LICENSE("GPL");
