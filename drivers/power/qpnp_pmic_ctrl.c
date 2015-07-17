/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#define pr_fmt(fmt)	"PMIC: %s: " fmt, __func__

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/spmi.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/alarmtimer.h>
#include <linux/bitops.h>
#if (defined(CONFIG_TCT_8X16_POP10) && defined(CONFIG_FT5X06_TOUCHPANEL_DRIVER_POP10))
extern void (*g_tpd_usb_plugin)(int);
#endif

#define CREATE_MASK(NUM_BITS, POS) \
	((unsigned char) (((1 << (NUM_BITS)) - 1) << (POS)))
#define LBC_MASK(MSB_BIT, LSB_BIT) \
	CREATE_MASK(MSB_BIT - LSB_BIT + 1, LSB_BIT)

/* Interrupt offsets */
#define INT_RT_STS_REG					0x10
#define BATT_PRES_IRQ                   BIT(0)

/* USB CHARGER PATH peripheral register offsets */
#define USB_PTH_STS_REG					0x09
#define USB_IN_VALID_MASK				LBC_MASK(7, 6)

/* CHARGER peripheral register offset */
#define CHG_USB_ENUM_T_STOP_REG			0x4E

/* BATTIF peripheral register offset */
#define BAT_IF_PRES_STATUS_REG			0x08
#define BATT_PRES_MASK					BIT(7)
#define BAT_IF_VREF_BAT_THM_CTRL_REG	0x4A
#define VREF_BATT_THERM_FORCE_ON		LBC_MASK(7, 6)
#define BAT_IF_BPD_CTRL_REG				0x48
#define BATT_BPD_CTRL_SEL_MASK			LBC_MASK(1, 0)
#define BATT_THM_EN						BIT(1)
#define BATT_ID_EN						BIT(0)

#define PERP_SUBTYPE_REG				0x05

/* Linear peripheral subtype values */
#define PMIC_BAT_IF_SUBTYPE				0x16
#define PMIC_USB_PTH_SUBTYPE			0x17

/* usb_interrupts */
struct pmic_irq {
	int		irq;
	unsigned long	disabled;
	bool            is_wake;
};

enum {
	USBIN_VALID = 0,
	MAX_IRQS,
};

extern int power_supply_set_charging_enabled(struct power_supply *psy, bool enable);

struct pmic_base_ctrl {
	struct device			*dev;
	struct spmi_device		*spmi;
	struct power_supply		*batt_psy;
	struct power_supply		*usb_psy;
	struct pmic_irq			irqs[MAX_IRQS];

	u16						bat_if_base;
	u16						usb_chg_base;
	bool					usb_present;
	spinlock_t				access_lock;

	struct qpnp_vadc_chip		*vadc_dev;
};


static int __qpnp_pmic_read(struct spmi_device *spmi, u16 base,
			u8 *val, int count)
{
	int rc = 0;

	rc = spmi_ext_register_readl(spmi->ctrl, spmi->sid, base, val, count);
	if (rc)
		pr_err("SPMI read failed base=0x%02x sid=0x%02x rc=%d\n",
				base, spmi->sid, rc);

	return rc;
}

static int __qpnp_pmic_write(struct spmi_device *spmi, u16 base,
			u8 *val, int count)
{
	int rc;

	rc = spmi_ext_register_writel(spmi->ctrl, spmi->sid, base, val,
					count);
	if (rc)
		pr_err("SPMI write failed base=0x%02x sid=0x%02x rc=%d\n",
				base, spmi->sid, rc);

	return rc;
}

static int qpnp_pmic_read(struct pmic_base_ctrl *ctrl, u16 base,
			u8 *val, int count)
{
	int rc = 0;
	struct spmi_device *spmi = ctrl->spmi;
	unsigned long flags;

	if (base == 0) {
		pr_err("base cannot be zero base=0x%02x sid=0x%02x rc=%d\n",
				base, spmi->sid, rc);
		return -EINVAL;
	}

	spin_lock_irqsave(&ctrl->access_lock, flags);
	rc = __qpnp_pmic_read(spmi, base, val, count);
	spin_unlock_irqrestore(&ctrl->access_lock, flags);

	return rc;
}

static int qpnp_pmic_write(struct pmic_base_ctrl *ctrl, u16 base,
			u8 *val, int count)
{
	int rc = 0;
	struct spmi_device *spmi = ctrl->spmi;
	unsigned long flags;

	if (base == 0) {
		pr_err("base cannot be zero base=0x%02x sid=0x%02x rc=%d\n",
				base, spmi->sid, rc);
		return -EINVAL;
	}

	spin_lock_irqsave(&ctrl->access_lock, flags);
	rc = __qpnp_pmic_write(spmi, base, val, count);
	spin_unlock_irqrestore(&ctrl->access_lock, flags);

	return rc;
}

static int qpnp_pmic_masked_write(struct pmic_base_ctrl *ctrl, u16 base,
				u8 mask, u8 val)
{
	int rc;
	u8 reg_val;
	struct spmi_device *spmi = ctrl->spmi;
	unsigned long flags;

	spin_lock_irqsave(&ctrl->access_lock, flags);
	rc = __qpnp_pmic_read(spmi, base, &reg_val, 1);
	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n", base, rc);
		goto out;
	}
	pr_debug("addr = 0x%x read 0x%x\n", base, reg_val);

	reg_val &= ~mask;
	reg_val |= val & mask;

	pr_debug("writing to base=%x val=%x\n", base, reg_val);

	rc = __qpnp_pmic_write(spmi, base, &reg_val, 1);
	if (rc)
		pr_err("spmi write failed: addr=%03X, rc=%d\n", base, rc);

out:
	spin_unlock_irqrestore(&ctrl->access_lock, flags);
	return rc;
}

static int qpnp_pmic_is_usb_chg_plugged_in(struct pmic_base_ctrl *ctrl)
{
	u8 usbin_valid_rt_sts;
	int rc;

	rc = qpnp_pmic_read(ctrl, ctrl->usb_chg_base + USB_PTH_STS_REG,
				&usbin_valid_rt_sts, 1);
	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n",
				ctrl->usb_chg_base + USB_PTH_STS_REG, rc);
		return rc;
	}

	pr_alert("usb_path_sts 0x%x\n", usbin_valid_rt_sts);

	return (usbin_valid_rt_sts & USB_IN_VALID_MASK) ? 1 : 0;
}


static int qpnp_pmic_is_batt_present(struct pmic_base_ctrl *ctrl)
{
	u8 batt_pres_rt_sts;
	int rc;

	rc = qpnp_pmic_read(ctrl, ctrl->bat_if_base + INT_RT_STS_REG,
				&batt_pres_rt_sts, 1);
	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n",
				ctrl->bat_if_base + INT_RT_STS_REG, rc);
		return rc;
	}

	return (batt_pres_rt_sts & BATT_PRES_IRQ) ? 1 : 0;
}


static int qpnp_pmic_get_batt_present(struct pmic_base_ctrl *ctrl)
{
	u8 reg_val;
	int rc;

	rc = qpnp_pmic_read(ctrl, ctrl->bat_if_base + BAT_IF_PRES_STATUS_REG,
				&reg_val, 1);
	if (rc) {
		pr_err("Failed to read battery status read failed rc=%d\n",
				rc);
		return 0;
	}

	return (reg_val & BATT_PRES_MASK) ? 1 : 0;
}

static int qpnp_pmic_bat_if_init(struct pmic_base_ctrl *ctrl)
{
	u8 reg_val;
	int rc;

	rc = qpnp_pmic_masked_write(ctrl,
			ctrl->bat_if_base + BAT_IF_BPD_CTRL_REG,
			BATT_BPD_CTRL_SEL_MASK, BATT_THM_EN);
	if (rc) {
		pr_err("Failed to choose BPD rc=%d\n", rc);
		return rc;
	}

	/* Force on VREF_BAT_THM */
	reg_val = VREF_BATT_THERM_FORCE_ON;
	rc = qpnp_pmic_write(ctrl,
			ctrl->bat_if_base + BAT_IF_VREF_BAT_THM_CTRL_REG,
			&reg_val, 1);
	if (rc) {
		pr_err("Failed to force on VREF_BAT_THM rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int qpnp_pmic_usb_path_init(struct pmic_base_ctrl *ctrl)
{
	int rc;
	u8 reg_val;

	if (qpnp_pmic_is_usb_chg_plugged_in(ctrl)) {
		reg_val = 0;
		rc = qpnp_pmic_write(ctrl,
			ctrl->usb_chg_base + CHG_USB_ENUM_T_STOP_REG,
			&reg_val, 1);
		if (rc) {
			pr_err("Failed to write enum stop rc=%d\n", rc);
			return -ENXIO;
		}
	}

	return rc;
}

#define OF_PROP_READ(ctrl, prop, qpnp_dt_property, retval, optional)	\
do {									\
	if (retval)							\
		break;							\
									\
	retval = of_property_read_u32(ctrl->spmi->dev.of_node,		\
					"qcom," qpnp_dt_property,	\
					&ctrl->prop);			\
									\
	if ((retval == -EINVAL) && optional)				\
		retval = 0;						\
	else if (retval)						\
		pr_err("Error reading " #qpnp_dt_property		\
				" property rc = %d\n", rc);		\
} while (0)

static irqreturn_t qpnp_pmic_usbin_valid_irq_handler(int irq, void *_ctrl)
{
	struct pmic_base_ctrl *ctrl = _ctrl;
	int usb_present = 0;

	usb_present = qpnp_pmic_is_usb_chg_plugged_in(ctrl);

	if (ctrl->usb_present ^ usb_present) {
		ctrl->usb_present = usb_present;
		power_supply_set_present(ctrl->usb_psy, ctrl->usb_present);
		power_supply_set_online(ctrl->usb_psy, ctrl->usb_present);
       //[BUG-FIX] Add -Begin by TCT-SZ weihong.chen 09/28/2014 PR.799807 change working frequency when usb pluged in .
       #if (defined(CONFIG_TCT_8X16_POP10) && defined(CONFIG_FT5X06_TOUCHPANEL_DRIVER_POP10))
       if(g_tpd_usb_plugin !=NULL)
        {
             if(ctrl->usb_present)
                 {
                 g_tpd_usb_plugin(1);
                }
                else
                 {
                g_tpd_usb_plugin(0);
                 }
          }
     #endif
     //[BUG-FIX] Add -End by TCT-SZ weihong.chen 09/28/2014 PR.799807 change working frequency when usb pluged in .
	}

	power_supply_set_charging_enabled(ctrl->batt_psy, ctrl->usb_present);

	return IRQ_HANDLED;
}

#define SPMI_REQUEST_IRQ(ctrl, idx, rc, irq_name, threaded, flags, wake)\
do {									\
	if (rc)								\
		break;							\
	if (ctrl->irqs[idx].irq) {					\
		if (threaded)						\
			rc = devm_request_threaded_irq(ctrl->dev,	\
				ctrl->irqs[idx].irq, NULL,		\
				qpnp_pmic_##irq_name##_irq_handler,	\
				flags, #irq_name, ctrl);		\
		else							\
			rc = devm_request_irq(ctrl->dev,		\
				ctrl->irqs[idx].irq,			\
				qpnp_pmic_##irq_name##_irq_handler,	\
				flags, #irq_name, ctrl);		\
		if (rc < 0) {						\
			pr_err("Unable to request " #irq_name " %d\n",	\
								rc);	\
		} else {						\
			rc = 0;						\
			if (wake) {					\
				enable_irq_wake(ctrl->irqs[idx].irq);	\
				ctrl->irqs[idx].is_wake = true;		\
			}						\
		}							\
	}								\
} while (0)

#define SPMI_GET_IRQ_RESOURCE(ctrl, rc, resource, idx, name)		\
do {									\
	if (rc)								\
		break;							\
									\
	rc = spmi_get_irq_byname(ctrl->spmi, resource, #name);		\
	if (rc < 0) {							\
		pr_err("Unable to get irq resource " #name "%d\n", rc);	\
	} else {							\
		ctrl->irqs[idx].irq = rc;				\
		rc = 0;							\
	}								\
} while (0)

static int qpnp_pmic_request_irqs(struct pmic_base_ctrl *ctrl)
{
	int rc = 0;
	SPMI_REQUEST_IRQ(ctrl, USBIN_VALID, rc, usbin_valid, 0,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
			| IRQF_ONESHOT, 1);
	return 0;
}

static int qpnp_pmic_get_irqs(struct pmic_base_ctrl *ctrl, u8 subtype,
					struct spmi_resource *spmi_resource)
{
	int rc = 0;

	switch (subtype) {
	case PMIC_BAT_IF_SUBTYPE:
		break;
	case PMIC_USB_PTH_SUBTYPE:
		pr_alert("get irqs.\n");
		SPMI_GET_IRQ_RESOURCE(ctrl, rc, spmi_resource,
						USBIN_VALID, usbin-valid);
		break;
	};

	return 0;
}

/* Get/Set initial state of charger */
static void determine_initial_status(struct pmic_base_ctrl *ctrl)
{
	ctrl->usb_present = qpnp_pmic_is_usb_chg_plugged_in(ctrl);
	power_supply_set_present(ctrl->usb_psy, ctrl->usb_present);

	/*
	 * Set USB psy online to avoid userspace from shutting down if battery
	 * capacity is at zero and no chargers online.
	 */
	if (ctrl->usb_present){
		power_supply_changed(ctrl->batt_psy);
		power_supply_set_online(ctrl->usb_psy, 1);
	}
	if(qpnp_pmic_is_batt_present(ctrl)){
		pr_alert("Battery is present\n");
	}
	power_supply_set_charging_enabled(ctrl->batt_psy, ctrl->usb_present);
}

static int qpnp_pmic_ctrl_probe(struct spmi_device *spmi)
{
	u8 subtype;
	struct pmic_base_ctrl *ctrl;
	struct resource *resource;
	struct spmi_resource *spmi_resource;
	struct power_supply *batt_psy, *usb_psy;
	int rc = 0;

	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy) {
		pr_err("battery supply not found deferring probe\n");
		return -EPROBE_DEFER;
	}

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		pr_err("usb supply not found deferring probe\n");
		return -EPROBE_DEFER;
	}

	ctrl = devm_kzalloc(&spmi->dev, sizeof(struct pmic_base_ctrl),
				GFP_KERNEL);
	if (!ctrl) {
		pr_err("memory allocation failed.\n");
		return -ENOMEM;
	}

	ctrl->batt_psy = batt_psy;
	ctrl->usb_psy = usb_psy;
	ctrl->dev = &spmi->dev;
	ctrl->spmi = spmi;

	dev_set_drvdata(&spmi->dev, ctrl);
	device_init_wakeup(&spmi->dev, 1);
	spin_lock_init(&ctrl->access_lock);

	spmi_for_each_container_dev(spmi_resource, spmi) {
		if (!spmi_resource) {
			pr_err("spmi resource absent\n");
			rc = -ENXIO;
			goto fail_chg_enable;
		}

		resource = spmi_get_resource(spmi, spmi_resource,
							IORESOURCE_MEM, 0);
		if (!(resource && resource->start)) {
			pr_err("node %s IO resource absent!\n",
						spmi->dev.of_node->full_name);
			rc = -ENXIO;
			goto fail_chg_enable;
		}

		rc = qpnp_pmic_read(ctrl, resource->start + PERP_SUBTYPE_REG,
					&subtype, 1);
		if (rc) {
			pr_err("Peripheral subtype read failed rc=%d\n", rc);
			goto fail_chg_enable;
		}

		switch (subtype) {
			case PMIC_USB_PTH_SUBTYPE:
				ctrl->usb_chg_base = resource->start;
				rc = qpnp_pmic_get_irqs(ctrl, subtype, spmi_resource);
				if (rc) {
					pr_err("Failed to get USB_PTH irqs rc=%d\n",
							rc);
					goto fail_chg_enable;
				}
				break;
			case PMIC_BAT_IF_SUBTYPE:
				ctrl->bat_if_base = resource->start;
				ctrl->vadc_dev = qpnp_get_vadc(ctrl->dev, "chg");
				if (IS_ERR(ctrl->vadc_dev)) {
					rc = PTR_ERR(ctrl->vadc_dev);
					if (rc != -EPROBE_DEFER)
						pr_err("vadc prop missing rc=%d\n",
								rc);
					goto fail_chg_enable;
				}
				break;
			default:
				pr_err("Invalid peripheral subtype=0x%x\n", subtype);
				rc = -EINVAL;
		}
	}

	rc = qpnp_pmic_bat_if_init(ctrl);
	if (rc) {
		pr_err("unable to initialize LBC BAT_IF rc=%d\n", rc);
		return rc;
	}
	rc = qpnp_pmic_usb_path_init(ctrl);
	if (rc) {
		pr_err("unable to initialize LBC USB path rc=%d\n", rc);
		return rc;
	}

	/* Get/Set charger's initial status */
	determine_initial_status(ctrl);

	rc = qpnp_pmic_request_irqs(ctrl);
	if (rc) {
		pr_err("unable to initialize LBC MISC rc=%d\n", rc);
	}

	if (!qpnp_pmic_get_batt_present(ctrl))
		pr_info("Battery absent and charging disabled !!!\n");

	return 0;
fail_chg_enable:
	dev_set_drvdata(&spmi->dev, NULL);
	return rc;
}

static int qpnp_pmic_ctrl_remove(struct spmi_device *spmi)
{
	dev_set_drvdata(&spmi->dev, NULL);
	return 0;
}

static struct of_device_id qpnp_pmic_match_table[] = {
	{ .compatible = "qcom,qpnp-pmic-ctrl", },
	{}
};

static struct spmi_driver qpnp_pmic_ctrl_driver = {
	.probe		= qpnp_pmic_ctrl_probe,
	.remove		= qpnp_pmic_ctrl_remove,
	.driver		= {
		.name		= "qcom,qpnp-pmic-ctrl",
		.owner		= THIS_MODULE,
		.of_match_table	= qpnp_pmic_match_table,
	},
};

static int __init qpnp_pmic_ctrl_init(void)
{
	return spmi_driver_register(&qpnp_pmic_ctrl_driver);
}
module_init(qpnp_pmic_ctrl_init);

static void __exit qpnp_pmic_ctrl_exit(void)
{
	spmi_driver_unregister(&qpnp_pmic_ctrl_driver);
}
module_exit(qpnp_pmic_ctrl_exit);

MODULE_DESCRIPTION("QPNP Linear charger driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" "qcom,qpnp-pmic-ctrl");
