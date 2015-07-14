/* Copyright (c) 2013-2014, LG Eletronics. All rights reserved.
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

#define pr_fmt(fmt) "[LGCC] %s : " fmt, __func__

#define CONFIG_LGE_PM_USE_BMS

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/power_supply.h>

#include <soc/qcom/lge/lge_charging_scenario.h>

#define MODULE_NAME "lge_charging_controller"
#define MONITOR_BATTEMP_POLLING_PERIOD  (60 * HZ)

extern int lgcc_is_charger_present(void);
extern int lgcc_set_ibat_current(int chg_current);
extern int lgcc_set_charging_enable(int enable);
extern void lgcc_charger_reginfo(void);

int lgcc_is_probed = 0;

struct lge_charging_controller{

	struct power_supply		*batt_psy;
	struct power_supply		*usb_psy;

#ifdef CONFIG_LGE_PM_USE_BMS
	struct power_supply		*bms_psy;
#endif

	struct delayed_work battemp_work;
	struct wake_lock lcs_wake_lock;

	enum lge_charging_states battemp_chg_state;

	int chg_current_te;
	int chg_current_max;
	int otp_ibat_current;
	int pseudo_chg_ui;
	int before_battemp;
	int batt_temp;
	int btm_state;
};

static struct lge_charging_controller *the_controller;

static int lgcc_thermal_mitigation;
ssize_t lgcc_set_thermal_chg_current(const char *val,
		struct kernel_param *kp){

	int ret;

	ret = param_set_int(val, kp);

	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}

	if (!the_controller) {
		pr_err("lgcc is not ready\n");
		return 0;
	}

	if (lgcc_thermal_mitigation <= 0)
		the_controller->chg_current_te = the_controller->chg_current_max;
	else
		the_controller->chg_current_te = lgcc_thermal_mitigation;

	pr_info("lgcc_thermal_mitigation = %d, chg_current_te_te = %d\n",
			lgcc_thermal_mitigation, the_controller->chg_current_te);

	cancel_delayed_work_sync(&the_controller->battemp_work);
	schedule_delayed_work(&the_controller->battemp_work, HZ*1);

	return 0;
}
module_param_call(lgcc_thermal_mitigation,
		lgcc_set_thermal_chg_current, NULL, &lgcc_thermal_mitigation, 0644);


static void lge_monitor_batt_temp_work(struct work_struct *work){

	struct charging_info req;
	struct charging_rsp res;
	bool is_changed = false;
	union power_supply_propval ret = {0,};

	the_controller->batt_psy->get_property(the_controller->batt_psy,
			POWER_SUPPLY_PROP_TEMP, &ret);
	req.batt_temp = ret.intval / 10;
	the_controller->batt_temp = req.batt_temp;

	the_controller->batt_psy->get_property(the_controller->batt_psy,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &ret);
	req.batt_volt = ret.intval;

	the_controller->batt_psy->get_property(the_controller->batt_psy,
			POWER_SUPPLY_PROP_CURRENT_NOW, &ret);
	req.current_now = ret.intval / 1000;

	the_controller->usb_psy->get_property(
			the_controller->usb_psy, POWER_SUPPLY_PROP_CURRENT_MAX, &ret);
	the_controller->chg_current_max = ret.intval / 1000;

	req.chg_current_ma = the_controller->chg_current_max;

	if (the_controller->chg_current_te != -EINVAL)
		req.chg_current_te = the_controller->chg_current_te;
	else
		req.chg_current_te = the_controller->chg_current_max;

	pr_debug("chg_current_max = %d\n", the_controller->chg_current_max);
	pr_debug("chg_curren_te = %d\n", the_controller->chg_current_te);

	req.is_charger = lgcc_is_charger_present();

	lge_monitor_batt_temp(req, &res);

	if (((res.change_lvl != STS_CHE_NONE) && req.is_charger) ||
			(res.force_update == true)) {
		if (res.change_lvl == STS_CHE_NORMAL_TO_DECCUR ||
				(res.state == CHG_BATT_DECCUR_STATE &&
				 res.dc_current != DC_CURRENT_DEF &&
				 res.change_lvl != STS_CHE_STPCHG_TO_DECCUR)) {
			pr_info("ibatmax_set STS_CHE_NORMAL_TO_DECCUR\n");
			the_controller->otp_ibat_current = res.dc_current;
			lgcc_set_ibat_current(the_controller->otp_ibat_current);
			lgcc_set_charging_enable(1);

		} else if (res.change_lvl == STS_CHE_NORMAL_TO_STPCHG ||
				(res.state == CHG_BATT_STPCHG_STATE)) {
			pr_info("ibatmax_set STS_CHE_NORMAL_TO_STPCHG\n");
			wake_lock(&the_controller->lcs_wake_lock);
			the_controller->otp_ibat_current = 0;
			lgcc_set_ibat_current(the_controller->otp_ibat_current);
			lgcc_set_charging_enable(0);

		} else if (res.change_lvl == STS_CHE_DECCUR_TO_NORAML) {
			pr_info("ibatmax_set STS_CHE_DECCUR_TO_NORAML\n");
			the_controller->otp_ibat_current = res.dc_current;
			lgcc_set_ibat_current(the_controller->otp_ibat_current);
			lgcc_set_charging_enable(1);

		} else if (res.change_lvl == STS_CHE_DECCUR_TO_STPCHG) {
			pr_info("ibatmax_set STS_CHE_DECCUR_TO_STPCHG\n");
			wake_lock(&the_controller->lcs_wake_lock);
			the_controller->otp_ibat_current = 0;
			lgcc_set_ibat_current(the_controller->otp_ibat_current);
			lgcc_set_charging_enable(0);

		} else if (res.change_lvl == STS_CHE_STPCHG_TO_NORMAL) {
			pr_info("ibatmax_set STS_CHE_STPCHG_TO_NORMAL\n");
			wake_unlock(&the_controller->lcs_wake_lock);
			the_controller->otp_ibat_current = res.dc_current;
			lgcc_set_ibat_current(the_controller->otp_ibat_current);
			lgcc_set_charging_enable(1);

		} else if (res.change_lvl == STS_CHE_STPCHG_TO_DECCUR) {
			pr_info("ibatmax_set STS_CHE_STPCHG_TO_DECCUR\n");
			the_controller->otp_ibat_current = res.dc_current;
			lgcc_set_ibat_current(the_controller->otp_ibat_current);
			lgcc_set_charging_enable(1);
			wake_unlock(&the_controller->lcs_wake_lock);

		} else if (res.force_update == true &&
				res.state == CHG_BATT_NORMAL_STATE &&
				res.dc_current != DC_CURRENT_DEF) {
			pr_info("ibatmax_set CHG_BATT_NORMAL_STATE\n");
			the_controller->otp_ibat_current = res.dc_current;
			lgcc_set_ibat_current(the_controller->otp_ibat_current);
			lgcc_set_charging_enable(1);
		}
	}

	pr_err("otp_ibat_current=%d\n", the_controller->otp_ibat_current);

	pr_debug("the_controller->pseudo_chg_ui = %d, res.pseudo_chg_ui = %d\n",
			the_controller->pseudo_chg_ui, res.pseudo_chg_ui);

	if (the_controller->pseudo_chg_ui ^ res.pseudo_chg_ui) {
		is_changed = true;
		the_controller->pseudo_chg_ui = res.pseudo_chg_ui;
	}

	pr_debug("the_controller->btm_state = %d, res.btm_state = %d\n",
			the_controller->btm_state, res.btm_state);
	if (the_controller->btm_state ^ res.btm_state) {
		is_changed = true;
		the_controller->btm_state = res.btm_state;
	}

	if (the_controller->before_battemp != req.batt_temp) {
		is_changed = true;
		the_controller->before_battemp = req.batt_temp;
	}

	if (is_changed == true)
		power_supply_changed(the_controller->batt_psy);

#ifdef CONFIG_LGE_PM_USE_BMS
	the_controller->bms_psy->get_property(the_controller->bms_psy,
			POWER_SUPPLY_PROP_CAPACITY, &ret);

	pr_info("Reported Capacity : %d / voltage : %d\n",
			ret.intval, req.batt_volt/1000);
#endif

	lgcc_charger_reginfo();

	schedule_delayed_work(&the_controller->battemp_work,
			MONITOR_BATTEMP_POLLING_PERIOD);
}

int get_pseudo_ui(void){

	if( !(the_controller == NULL) ){
		return the_controller->pseudo_chg_ui;
	}
	return 0;
}

int get_btm_state(void){

	if( !(the_controller == NULL) ){
		return the_controller->btm_state;
	}
	return 0;
}

void start_battemp_work(int delay){
	pr_debug("start_battemp_work~!!\n");
	schedule_delayed_work(&the_controller->battemp_work, (delay * HZ));
}
EXPORT_SYMBOL(start_battemp_work);

void stop_battemp_work(void){
	pr_debug("stop_battemp_work~!!\n");
	cancel_delayed_work(&the_controller->battemp_work);
}
EXPORT_SYMBOL(stop_battemp_work);

static int lge_charging_controller_probe(struct platform_device *pdev)
{
	struct lge_charging_controller *controller;

	controller = kzalloc(sizeof(struct lge_charging_controller), GFP_KERNEL);

	if(!controller){
		pr_err("lge_charging_controller memory allocation failed.\n");
		return -ENOMEM;
	}

	the_controller = controller;

	controller->usb_psy = power_supply_get_by_name("usb");

	if(!controller->usb_psy){
		pr_err("usb power_supply not found deferring probe\n");
		return -EPROBE_DEFER;
	}

	controller->batt_psy = power_supply_get_by_name("battery");

	if(!controller->batt_psy){
		pr_err("battery power_supply not found deferring probe\n");
		return -EPROBE_DEFER;
	}

#ifdef CONFIG_LGE_PM_USE_BMS
	controller->bms_psy = power_supply_get_by_name("bms");

	if(!controller->bms_psy){
		pr_err("bms power_supply not found deferring probe\n");
		return -EPROBE_DEFER;
	}
#endif

	wake_lock_init(&controller->lcs_wake_lock,
			WAKE_LOCK_SUSPEND, "lge_charging_scenario");

	INIT_DELAYED_WORK(&controller->battemp_work,
			lge_monitor_batt_temp_work);

	controller->chg_current_max = -EINVAL;
	controller->chg_current_te = controller->chg_current_max;

	controller->otp_ibat_current = 0;

	start_battemp_work(5);

	lgcc_is_probed = 1;

	pr_info("LG Charging controller probe done~!!\n");

	return 0;
}

static int lge_charging_controller_remove(struct platform_device *pdev)
{
	kfree(the_controller);
	return 0;
}

static struct platform_driver lge_charging_controller_driver = {
	.probe = lge_charging_controller_probe,
	.remove = lge_charging_controller_remove,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init lge_charging_controller_init(void)
{
	return platform_driver_register(&lge_charging_controller_driver);
}

static void lge_charging_controller_exit(void)
{
	platform_driver_unregister(&lge_charging_controller_driver);
}

static struct platform_device lge_charging_controller_platform_device = {
	.name   = "lge_charging_controller",
	.id = 0,
};

static int __init lge_charging_controller_device_init(void)
{
	pr_info("%s st\n", __func__);
	return platform_device_register(&lge_charging_controller_platform_device);
}

static void lge_charging_controller_device_exit(void)
{
	platform_device_unregister(&lge_charging_controller_platform_device);
}

late_initcall(lge_charging_controller_init);
module_exit(lge_charging_controller_exit);
late_initcall(lge_charging_controller_device_init);
module_exit(lge_charging_controller_device_exit);
MODULE_DESCRIPTION("LGE Charging Controller driver");
MODULE_LICENSE("GPL v2");
