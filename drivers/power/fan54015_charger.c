/*
 * fan54015.c  --  FAN54015 Switching Charger driver
 *
 * Copyright (C) 2014 Fairchild semiconductor Co.Ltd
 * Author: Bright Yang <bright.yang@fairchildsemi.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#define pr_fmt(fmt)	"FAN54015: %s: " fmt, __func__

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/timer.h>
#include <linux/param.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/spmi.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/uaccess.h>

#if (defined CONFIG_QPNP_BATT_STATUS_DEBUG)
#include <linux/time.h>
#include <linux/rtc.h>
#define INFO(fmt, args...) printk(KERN_WARNING fmt "\n" , ## args)
#else
#define INFO(fmt, args...) do { } while (0)
#endif

//#define QPNP_NTC_DEBUG_SIMULATE

/******************************************************************************
* Register addresses
******************************************************************************/
/********** FAN54015_REG_CONTROL0 (0x00) **********/
#define FAN54015_REG_CONTROL0           0x00
#define FAN54015_FAULT                  0x07
#define FAN54015_FAULT_SHIFT            0x00
#define FAN54015_BOOST              	0x01 << 3
#define FAN54015_BOOST_SHIFT            0x03
#define FAN54015_STAT               	0x03 << 4
#define FAN54015_STAT_SHIFT             0x04
#define FAN54015_EN_STAT            	0x01 << 6
#define FAN54015_EN_STAT_SHIFT          0x06
#define FAN54015_TMR_RST_OTG        	0x01 << 7  // writing a 1 resets the t32s timer, writing a 0 has no effect
#define FAN54015_TMR_RST_OTG_SHIFT      0x07
/******************************************************************************
* bit definitions
******************************************************************************/
// STAT [5:4]
enum {
	FAN54015_CHARGING_READY = 0x00,
	FAN54015_CHARGING_PROGRESS,
	FAN54015_CHARGING_DONE,
	FAN54015_CHARGING_FAULT,
};
// EN_STAT [6]
#define ENABLE_STAT 					0x01
#define DISABLE_STAT 					0x00
// TMR_RST [7]
#define RESET_32S_TIMER					0x01

/********** FAN54015_REG_CONTROL1 (0x01) **********/
#define FAN54015_REG_CONTROL1           0x01
#define FAN54015_OPA_MODE               0x01
#define FAN54015_OPA_MODE_SHIFT         0x00
#define FAN54015_HZ_MODE            	0x01 << 1
#define FAN54015_HZ_MODE_SHIFT          0x01
#define FAN54015_CE_N               	0x01 << 2
#define FAN54015_CE_N_SHIFT             0x02
#define FAN54015_TE                 	0x01 << 3
#define FAN54015_TE_SHIFT               0x03
#define FAN54015_VLOWV              	0x03 << 4
#define FAN54015_VLOWV_SHIFT            0x04
#define FAN54015_IINLIM             	0x03 << 6
#define FAN54015_IINLIM_SHIFT           0x06
/******************************************************************************
* bit definitions
******************************************************************************/
// OPA_MODE [0]
#define FAN54015_CHARGE_MODE			0x00
#define FAN54015_BOOST_MODE 			0x01
// HZ_MODE [1]
#define HIGH_IMPEDANCE_MODE				0x01
// CE/ [2]
#define ENABLE_CHARGER 					0x00
#define DISABLE_CHARGER 				0x01
// TE [3]
#define DISABLE_TERMINATION				0x00
#define ENABLE_TERMINATION 				0x01

/********** FAN54015_REG_OREG (0x02) **********/
#define FAN54015_REG_OREG               0x02
#define FAN54015_OTG_EN                 0x01
#define FAN54015_OTG_EN_SHIFT           0x00
#define FAN54015_OTG_PL             	0x01 << 1
#define FAN54015_OTG_PL_SHIFT           0x01
#define FAN54015_OREG               	0x3f << 2
#define FAN54015_OREG_SHIFT             0x02
/******************************************************************************
* bit definitions
******************************************************************************/
// OTG_EN [0]
#define DISABLE_OTG_MODE 				0x00
#define ENABLE_OTG_MODE 				0x01

/********** FAN54015_REG_IC_INFO (0x03) **********/
#define FAN54015_REG_IC_INFO            0x03
#define FAN54015_REV                    0x03
#define FAN54015_REV_SHIFT              0x00
#define FAN54015_PN                 	0x07 << 2
#define FAN54015_PN_SHIFT               0x02
#define FAN54015_VENDOR_CODE        	0x07 << 5
#define FAN54015_VENDOR_CODE_SHIFT      0x05

/********** FAN54015_REG_IBAT (0x04) **********/
#define FAN54015_REG_IBAT               0x04
#define FAN54015_ITERM                  0x07
#define FAN54015_ITERM_SHIFT            0x00
#define FAN54015_IOCHARGE           	0x07 << 4
#define FAN54015_IOCHARGE_SHIFT         0x04
#define FAN54015_RESET              	0x01 << 7
#define FAN54015_RESET_SHIFT            0x07

/********** FAN54015_REG_SP_CHARGER (0x05) **********/
#define FAN54015_REG_SP_CHARGER         0x05
#define FAN54015_VSP                    0x07
#define FAN54015_VSP_SHIFT              0x00
#define FAN54015_EN_LEVEL           	0x01 << 3
#define FAN54015_EN_LEVEL_SHIFT         0x03
#define FAN54015_SP                 	0x01 << 4
#define FAN54015_SP_SHIFT               0x04
#define FAN54015_IO_LEVEL           	0x01 << 5
#define FAN54015_IO_LEVEL_SHIFT         0x05
#define FAN54015_DIS_VREG           	0x01 << 6
#define FAN54015_DIS_VREG_SHIFT         0x06
/******************************************************************************
* bit definitions
******************************************************************************/
// IO_LEVEL [5]
#define ENABLE_IO_LEVEL 				0x00
#define DISABLE_IO_LEVEL 				0x01

/********** FAN54015_REG_SAFETY (0x06) **********/
#define FAN54015_REG_SAFETY             0x06
#define FAN54015_VSAFE                  0x0f
#define FAN54015_VSAFE_SHIFT            0x00
#define FAN54015_ISAFE              	0x07 << 4
#define FAN54015_ISAFE_SHIFT            0x04
/******************************************************************************
* bit definitions
******************************************************************************/
// VSAFE [3:0] 68mOhm
enum {
	FAN54015_VSAFE_4P20 = 0x00,
	FAN54015_VSAFE_4P22,
	FAN54015_VSAFE_4P24,
	FAN54015_VSAFE_4P26,
	FAN54015_VSAFE_4P28,
	FAN54015_VSAFE_4P30,
	FAN54015_VSAFE_4P32,
	FAN54015_VSAFE_4P34,
	FAN54015_VSAFE_4P36,
	FAN54015_VSAFE_4P38,
	FAN54015_VSAFE_4P40,
	FAN54015_VSAFE_4P42,
	FAN54015_VSAFE_4P44,
};
enum {
	FAN54015_ISAFE_550 = 0x00,
	FAN54015_ISAFE_650,
	FAN54015_ISAFE_750,
	FAN54015_ISAFE_850,
	FAN54015_ISAFE_1050,
	FAN54015_ISAFE_1350,
	FAN54015_ISAFE_1450,
};

/********** FAN54015_REG_MONITOR (0x10) **********/
#define FAN54015_REG_MONITOR            0x10
#define FAN54015_CV                     0x01
#define FAN54015_CV_SHIFT               0x00
#define FAN54015_VBUS_VALID         	0x01 << 1
#define FAN54015_VBUS_VALID_SHIFT       0x01
#define FAN54015_IBUS               	0x01 << 2
#define FAN54015_IBUS_SHIFT             0x02
#define FAN54015_ICHG               	0x01 << 3
#define FAN54015_ICHG_SHIFT             0x03
#define FAN54015_T_120              	0x01 << 4
#define FAN54015_T_120_SHIFT            0x04
#define FAN54015_LINCHG             	0x01 << 5
#define FAN54015_LINCHG_SHIFT           0x05
#define FAN54015_VBAT_CMP           	0x01 << 6
#define FAN54015_VBAT_CMP_SHIFT         0x06
#define FAN54015_ITERM_CMP          	0x01 << 7
#define FAN54015_ITERM_CMP_SHIFT        0x07

#define FAN54015_REG_MAX				0x11

/* reset the T32s timer every 10 seconds   */
#define T32S_RESET_INTERVAL 		10*HZ
#define T1S_SET_CHARGRE_INTERVAL	1*HZ

enum {
	USER	= BIT(0),
	THERMAL = BIT(1),
	CURRENT = BIT(2),
	TERMINATION = BIT(3),
};

static const u8 fan54015_def_reg[17] = {
    0x40,    // #0x00(CONTROL0)
    0x30,    // #0x01(CONTROL1)
    0x0a,    // #0x02(OREG)
    0x84,    // #0x03(IC_INFO)
    0x09,    // #0x04(IBAT)
    0x24,    // #0x05(SP_CHARGER)
    0x40,    // #0x06(SAFETY)
    0x00,    // #0x07 - unused
    0x00,    // #0x08 - unused
    0x00,    // #0x09 - unused
    0x00,    // #0x0a - unused
    0x00,    // #0x0b - unused
    0x00,    // #0x0c - unused
    0x00,    // #0x0d - unused
    0x00,    // #0x0e - unused
    0x00,    // #0x0f - unused
    0x00,    // #0x10(MONITOR)
};

static enum power_supply_property fan54015_charger_props[] = {
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
};

enum battery_health_type {
	BATTERY_HEALTH_GOOD,
	BATTERY_HEALTH_OVERHEAT,
	BATTERY_HEALTH_WARM,
	BATTERY_HEALTH_COOL,//it is not used now
	BATTERY_HEALTH_COLD,
};

enum thermal_mitigation_current{
	THERMAL_MITIGATION_1450,
	THERMAL_MITIGATION_700,
	THERMAL_MITIGATION_600,
	THERMAL_MITIGATION_0,
};

struct fan54015_otg_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

struct fan54015 {
	struct device *dev;
	struct i2c_client *client;
	int irq;
	struct delayed_work	charger_workqueue;
	struct delayed_work monitor_workqueue;
	struct power_supply *usb;
	struct power_supply	*bms_psy;
	struct power_supply battery;
	struct fan54015_otg_regulator otg_vreg;

	/* Charging parameters */
	u8 				regs[FAN54015_REG_MAX];
	int				iterm_ma;
	int				vfloat_mv;
	bool 			reset;
	bool 			charging_disabled;
	int				charging_disabled_status;
//[PLATFORM]-Modify-BEGIN by TCTSZ.Leo.guo, 2014/09/02, added for dock charger support
#if defined CONFIG_QPNP_DOCK_POWER_SUPPLY_SUPPORT
	int				dock_irq;
	int 			chg_dock_dec;
	struct delayed_work		dock_set_work;
#endif
//[PLATFORM]-Modify-END by TCTSZ.Leo.guo, 2014/09/02
#if defined CONFIG_QPNP_POWER_SWITCH_SUPPORT
	struct regulator 		*vbus_dec;
#endif

#if (defined CONFIG_QPNP_BATT_STATUS_DEBUG)
	int index;
#endif

	int				current_limit;
	struct dentry			*debug_root;
	struct qpnp_vadc_chip	*vadc_dev;

	unsigned int			thermal_levels;
	unsigned int			therm_lvl_sel;
	unsigned int			*thermal_mitigation;
	int				cfg_warm_bat_decidegc;
	int				cfg_cool_bat_decidegc;
	struct qpnp_adc_tm_chip	*adc_tm_dev;
	struct qpnp_adc_tm_btm_param	adc_param;
	enum battery_health_type bat_health;

	struct mutex		read_write_lock;
	struct mutex		charging_disable_lock;
	struct mutex		current_change_lock;

	struct wake_lock	monitor_wake_lock;
};

static int fan54015_charger_get_voltage_now(struct fan54015 *chip);
static int fan54015_charger_get_capacity(struct fan54015 *chip);
static int fan54015_charger_get_batt_temp(struct fan54015 *chip);

static int __fan54015_read(struct fan54015 *chip, int reg, u8 *val)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(chip->client, reg);
	if (ret < 0) {
		dev_err(chip->dev,
			"i2c read fail: can't read from %02x: %d\n", reg, ret);
		return ret;
	} else {
		*val = ret;
	}

	return 0;
}

static int __fan54015_write(struct fan54015 *chip, int reg, u8 val)
{
	s32 ret;
	ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	if (ret < 0) {
		dev_err(chip->dev,
			"i2c write fail: can't write %02x to %02x: %d\n",
			val, reg, ret);
		return ret;
	}
	if(((reg == FAN54015_REG_IBAT) && (val & FAN54015_RESET)) ||
		chip->reset){
		memcpy(chip->regs, fan54015_def_reg, 6); // resets charge paremeters, except the safety register(#6)
		chip->reset = false;
	}
	else{
		chip->regs[reg] = val;
	}

	return 0;
}

static int fan54015_set_value(struct fan54015 *chip, u8 reg, u8 reg_bit, u8 reg_shift, u8 val)
{
	int rc;
	u8 tmp;
	mutex_lock(&chip->read_write_lock);
	tmp = chip->regs[reg] & (~reg_bit);
	tmp |= (val << reg_shift);
	if(reg_bit == FAN54015_RESET && reg == FAN54015_REG_IBAT){
		chip->reset = true;
	}
	rc = __fan54015_write(chip, reg, tmp);
	mutex_unlock(&chip->read_write_lock);
	return rc;
}

static u8 fan54015_get_value(struct fan54015 *chip, u8 reg, u8 reg_bit, u8 reg_shift, u8* val)
{
	int rc = 0;
	mutex_lock(&chip->read_write_lock);
	rc = __fan54015_read(chip, reg, val);
	*val = ((*val) & reg_bit) >> reg_shift;
	mutex_unlock(&chip->read_write_lock);
	return rc;
}

#define MIN_FLOAT_MV			3500
#define MAX_FLOAT_MV			4400
#define VFLOAT_STEP_MV			20
#define DEFAULT_FLOAT_VOLTAGE	4360
static int fan54015_charger_set_float_voltage(struct fan54015 *chip, int vfloat_mv)
{
	u8 val;

	if ((vfloat_mv < MIN_FLOAT_MV) || (vfloat_mv > MAX_FLOAT_MV)) {
		dev_err(chip->dev, "bad float voltage mv =%d asked to set\n",
					vfloat_mv);
		return -EINVAL;
	}

	val = (vfloat_mv - MIN_FLOAT_MV) / VFLOAT_STEP_MV;

	return fan54015_set_value(chip, FAN54015_REG_OREG, FAN54015_OREG, FAN54015_OREG_SHIFT, val);  //OREG = 4.36V
}

#define MIN_FASTCHG_CURRENT			550
#define MAX_FASTCHG_CURRENT			1450
static int fastchg_current[] = {
	550, 650, 750, 850, 1050, 1150, 1350, 1450,
};
static int fan54015_charger_set_fastchg_current(struct fan54015 *chip, int ifastchg)
{
	u8 idx = 0;

	for (idx = ARRAY_SIZE(fastchg_current) - 1; idx >= 0; idx--) {
		if (fastchg_current[idx] <= ifastchg)
			break;
	}
	if (idx < 0) {
		pr_debug("Couldn't find fastchg %d mA.\n", ifastchg);
		idx = 0;
	}
	INFO("Fast charging current set to = %d\n", fastchg_current[idx]);

	return fan54015_set_value(chip, FAN54015_REG_IBAT, FAN54015_IOCHARGE,FAN54015_IOCHARGE_SHIFT, idx);
}

#define DEFAULT_TERM_CURRENT		194
static int termination_current[] = {
	49, 97, 146, 194, 243, 291, 340, 388,
};
static int fan54015_charger_set_termination_current(struct fan54015 *chip, int iterm)
{
	u8 idx;

	for (idx = ARRAY_SIZE(termination_current) - 1; idx >= 0; idx--) {
		if (termination_current[idx] <= iterm)
			break;
	}
	if (idx < 0) {
		pr_debug("Couldn't find termination %d mA.\n", iterm);
		idx = 0;
	}

	INFO("Set termination current: %d\n", termination_current[idx]);

	/* set termination current limit */
	fan54015_set_value(chip, FAN54015_REG_CONTROL1, FAN54015_TE, FAN54015_TE_SHIFT, ENABLE_TERMINATION);
	return fan54015_set_value(chip, FAN54015_REG_IBAT, FAN54015_ITERM, FAN54015_ITERM_SHIFT, idx);
}

/******************************************************************************
* Function: fan54015_charger_disable_charging
* Parameters: None
* Return: None
*
* Description:
*
******************************************************************************/
static int __fan54015_charger_disable_charging(struct fan54015 *chip, bool disable)
{
    return fan54015_set_value(chip, FAN54015_REG_CONTROL1, FAN54015_CE_N,FAN54015_CE_N_SHIFT, disable ? DISABLE_CHARGER : ENABLE_CHARGER);
}

static int fan54015_charger_disable_charging(struct fan54015 *chip, int reason,
								int disable)
{
	int rc = 0;
	int disabled;

	mutex_lock(&chip->charging_disable_lock);

	disabled = chip->charging_disabled_status;

	pr_debug("reason=%d requested_disable=%d disabled_status=%d\n",
					reason, disable, disabled);

	if (disable == true)
		disabled |= reason;
	else
		disabled &= ~reason;

	if (disabled)
		rc = __fan54015_charger_disable_charging(chip, true);
	else
		rc = __fan54015_charger_disable_charging(chip, false);

	if (rc)
		pr_err("Couldn't disable charging for reason=%d rc=%d\n",
							rc, reason);
	else
		chip->charging_disabled_status = disabled;

	mutex_unlock(&chip->charging_disable_lock);

	return rc;
}

/******************************************************************************
* Function:
* Parameters: None
* Return: None
*
* Description:  if VBUS present(charging & boost),write 1 every 10 seconds
*
******************************************************************************/
static void fan54015_monitor_work_callback(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct fan54015 *chip = container_of(dwork,
			struct fan54015, monitor_workqueue);
#if (defined CONFIG_QPNP_BATT_STATUS_DEBUG)
	u8 reg;
	struct timeval now;
	struct rtc_time tm;
	int voltage = fan54015_charger_get_voltage_now(chip);
	int capacity = fan54015_charger_get_capacity(chip);
	int temperature = fan54015_charger_get_batt_temp(chip);
	__fan54015_read(chip, FAN54015_REG_CONTROL0, &reg);

	chip->index++;
	do_gettimeofday(&now);
	rtc_time_to_tm(now.tv_sec, &tm);
	INFO("%-8d    %02d:%02d:%02d    %8d    %3d%%    %3dC    0x%2x",chip->index, tm.tm_hour, tm.tm_min, tm.tm_sec, voltage, capacity, temperature, reg);
#endif
	fan54015_set_value(chip, FAN54015_REG_CONTROL0, FAN54015_TMR_RST_OTG, FAN54015_TMR_RST_OTG_SHIFT, RESET_32S_TIMER);
	schedule_delayed_work(&chip->monitor_workqueue, T32S_RESET_INTERVAL);
}

static void fan54015_charger_work_callback(struct work_struct *work)
{
	char *str;
	struct delayed_work *dwork = to_delayed_work(work);
	struct fan54015 *chip = container_of(dwork, struct fan54015, charger_workqueue);
	int rc, therm_ma, current_ma;
	union power_supply_propval prop = {0,};

	if (!chip->charging_disabled){
		if(!wake_lock_active(&chip->monitor_wake_lock)){
			wake_lock(&chip->monitor_wake_lock);
		}
		if (chip->iterm_ma != -EINVAL)
			fan54015_charger_set_termination_current(chip, chip->iterm_ma);
		else
			fan54015_charger_set_termination_current(chip, DEFAULT_TERM_CURRENT);

		if (chip->vfloat_mv)
			fan54015_charger_set_float_voltage(chip, chip->vfloat_mv);
		else
			fan54015_charger_set_float_voltage(chip, DEFAULT_FLOAT_VOLTAGE);

		rc = chip->usb->get_property(chip->usb,
						POWER_SUPPLY_PROP_TYPE, &prop);
		if (rc < 0) pr_err("could not read USB TYPE property, rc=%d\n", rc);

		switch(prop.intval){
			case POWER_SUPPLY_TYPE_UNKNOWN:
				INFO("Current charger type is unknow charger.\n");
				break;
			case POWER_SUPPLY_TYPE_USB:
				INFO("Current charger type is standard usb charger.\n");
				/** Set input limit current to 500mA **/
				chip->current_limit = MIN_FASTCHG_CURRENT;
				fan54015_set_value(chip, FAN54015_REG_CONTROL1, FAN54015_IINLIM, FAN54015_IINLIM_SHIFT, 0x01);
				fan54015_charger_set_fastchg_current(chip, MIN_FASTCHG_CURRENT);
				break;
			case POWER_SUPPLY_TYPE_USB_ACA:
				str = "accessory";
			case POWER_SUPPLY_TYPE_USB_DCP:
				str = "dedicated";
				INFO("Current charger type is %s charger.\n", str);
				/** Set input limit current to no limit **/
				chip->current_limit = MAX_FASTCHG_CURRENT;
				fan54015_set_value(chip, FAN54015_REG_CONTROL1, FAN54015_IINLIM, FAN54015_IINLIM_SHIFT, 0x03);
				therm_ma = chip->thermal_mitigation[chip->therm_lvl_sel];
				current_ma = min(therm_ma, chip->current_limit);
				fan54015_charger_set_fastchg_current(chip, current_ma);
				break;
			default:
				break;
		}

		/** Set weak battery voltage to 3.40V **/
		fan54015_set_value(chip, FAN54015_REG_CONTROL1, FAN54015_VLOWV, FAN54015_VLOWV_SHIFT, 0x00);
		/** Set IO_LEVEL is 0. Output current is controlled by IOCHARGE bits.**/
		fan54015_set_value(chip, FAN54015_REG_SP_CHARGER, FAN54015_IO_LEVEL,FAN54015_IO_LEVEL_SHIFT, 0x00);

		fan54015_charger_disable_charging(chip, CURRENT, false);
	}
	else{
		fan54015_charger_disable_charging(chip, CURRENT, true);
		if(wake_lock_active(&chip->monitor_wake_lock)){
			wake_unlock(&chip->monitor_wake_lock);
		}
	}
	power_supply_changed(&chip->battery);
}

//[PLATFORM]-Modify-BEGIN by TCTSZ.Leo.guo, 2014/09/02, added for dock charger support
#if defined CONFIG_QPNP_DOCK_POWER_SUPPLY_SUPPORT
#define DOCK_POWER_SUPPLY_MAX_CURRENT		1500000
static void
fan54015_charger_dock_update_work(struct work_struct *work)
{
	int rc;
	union power_supply_propval prop = {0,};
	struct delayed_work *dwork = to_delayed_work(work);
	struct fan54015 *chip = container_of(dwork,
									struct fan54015, dock_set_work);
	prop.intval = DOCK_POWER_SUPPLY_MAX_CURRENT;
	rc = chip->usb->set_property(chip->usb,
					POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
	if (rc < 0)
		dev_err(chip->dev,
			"could not read USB current_max property, rc=%d\n", rc);
	prop.intval = POWER_SUPPLY_TYPE_USB_DCP;
	rc = chip->usb->set_property(chip->usb,
					POWER_SUPPLY_PROP_TYPE, &prop);
	if (rc < 0)
		dev_err(chip->dev,
			"could not read USB current_max property, rc=%d\n", rc);
}

#define MSM_SET_DOCK_STATUS_DELAY	5 /* 5msec */
static irqreturn_t fan54015_charger_dock_irq_callback(int irq, void *data)
{
	struct fan54015 *chip = data;

	/*schedule delayed work for 5msec for ID line state to settle*/
	schedule_delayed_work(&chip->dock_set_work,
		msecs_to_jiffies(MSM_SET_DOCK_STATUS_DELAY));

	return IRQ_HANDLED;
}
#endif
//[PLATFORM]-Modify-END by TCTSZ.Leo.guo

/******************************************************************************
* Function: FAN54015_Initialization
* Parameters: None
* Return: None
*
* Description:
*
******************************************************************************/
static int fan54015_charger_init_device(struct fan54015 *chip)
{
	INIT_DELAYED_WORK(&chip->charger_workqueue, fan54015_charger_work_callback);
	INIT_DELAYED_WORK(&chip->monitor_workqueue, fan54015_monitor_work_callback);
	memcpy(chip->regs, fan54015_def_reg, sizeof(fan54015_def_reg));
	/** Set safety settings for voltage and current **/
	fan54015_set_value(chip, FAN54015_REG_SAFETY, FAN54015_VSAFE, FAN54015_VSAFE_SHIFT, FAN54015_VSAFE_4P36);
	fan54015_set_value(chip, FAN54015_REG_SAFETY, FAN54015_ISAFE, FAN54015_ISAFE_SHIFT, FAN54015_ISAFE_1450);

	if (chip->iterm_ma != -EINVAL)
		fan54015_charger_set_termination_current(chip, chip->iterm_ma);
	else
		fan54015_charger_set_termination_current(chip, DEFAULT_TERM_CURRENT);

	if (chip->vfloat_mv != -EINVAL)
		fan54015_charger_set_float_voltage(chip, chip->vfloat_mv);
	else
		fan54015_charger_set_float_voltage(chip, DEFAULT_FLOAT_VOLTAGE);

	fan54015_charger_set_fastchg_current(chip, MAX_FASTCHG_CURRENT);

	/** Set weak battery voltage to 3.40V **/
	fan54015_set_value(chip, FAN54015_REG_CONTROL1, FAN54015_VLOWV, FAN54015_VLOWV_SHIFT, 0x00);
	/** Set input limit current to 500mA **/
	fan54015_set_value(chip, FAN54015_REG_CONTROL1, FAN54015_IINLIM, FAN54015_IINLIM_SHIFT, 0x01);
	/** Set IO_LEVEL is 0. Output current is controlled by IOCHARGE bits.**/
	fan54015_set_value(chip, FAN54015_REG_SP_CHARGER, FAN54015_IO_LEVEL,FAN54015_IO_LEVEL_SHIFT, 0x00);

	fan54015_charger_disable_charging(chip, CURRENT, false);

	schedule_delayed_work(&chip->monitor_workqueue, T32S_RESET_INTERVAL);
	return 0;
}

static int fan54015_charger_get_charging_status(struct fan54015 *chip)
{
	return !chip->charging_disabled;
}

static int fan54015_charger_get_batt_status(struct fan54015 *chip)
{
	int rc;
	int cnt = 1;
	u8 status = 0;
	do {
		cnt++;
		fan54015_get_value(chip, FAN54015_REG_CONTROL0, FAN54015_STAT, FAN54015_STAT_SHIFT, &status);
		msleep(10);
	} while (cnt <= 5);

	switch(status){
		case FAN54015_CHARGING_PROGRESS:
			rc = POWER_SUPPLY_STATUS_CHARGING;
			break;
		case FAN54015_CHARGING_DONE:
			rc = POWER_SUPPLY_STATUS_FULL;
			break;
		case FAN54015_CHARGING_FAULT:
			rc = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		default:
			rc = POWER_SUPPLY_STATUS_DISCHARGING;
	}

	return rc;
}

#define DEFAULT_TEMP		250
static int fan54015_charger_get_batt_temp(struct fan54015 *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

#ifdef FEATURE_TCTNB_MMITEST
	return DEFAULT_TEMP;
#endif

	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX1_BATT_THERM, &results);
	if (rc) {
		pr_debug("Unable to read batt temperature rc=%d\n", rc);
		return DEFAULT_TEMP;
	}
	pr_debug("get_bat_temp %d, %lld\n", results.adc_code,
							results.physical);

	return (int)results.physical;
}


static int fan54015_charger_get_batt_health(struct fan54015 *chip)
{
	union power_supply_propval ret = {0, };
	if(chip->bat_health == BATTERY_HEALTH_COLD)
		ret.intval = POWER_SUPPLY_HEALTH_COLD;
	else if(chip->bat_health == BATTERY_HEALTH_WARM)
		ret.intval = POWER_SUPPLY_HEALTH_WARM;
	else if(chip->bat_health == BATTERY_HEALTH_OVERHEAT)
		ret.intval = POWER_SUPPLY_HEALTH_OVERHEAT;
	else
		ret.intval = POWER_SUPPLY_HEALTH_GOOD;

	return ret.intval;
}

static int fan54015_charger_temp_level_set(struct fan54015 *chip,
							int lvl_sel)
{
	int rc = 0, therm_ma = MAX_FASTCHG_CURRENT;
	int current_ma;

	if (!chip->thermal_mitigation) {
		pr_err("Thermal mitigation not supported\n");
		return -EINVAL;
	}

	if (lvl_sel < 0) {
		pr_err("Unsupported level selected %d\n", lvl_sel);
		return -EINVAL;
	}

	if (lvl_sel >= chip->thermal_levels) {
		pr_err("Unsupported level selected %d forcing %d\n", lvl_sel,
				chip->thermal_levels - 1);
		lvl_sel = chip->thermal_levels - 1;
	}

	if (lvl_sel == chip->therm_lvl_sel)
		return 0;

	mutex_lock(&chip->current_change_lock);
	chip->therm_lvl_sel = lvl_sel;

	if (chip->therm_lvl_sel == (chip->thermal_levels - 1)) {
		/* Disable charging if highest value selected */
		INFO("Current battery status: %d, disenable charging. \n", chip->bat_health);
		rc = fan54015_charger_disable_charging(chip, THERMAL, true);
		if (rc < 0) {
			pr_err("Couldn't disable charging rc %d\n", rc);
			goto out;
		}
		goto out;
	}

	if (chip->therm_lvl_sel > 0
		&& chip->therm_lvl_sel < (chip->thermal_levels - 1)){
		/*
		 * consider thermal limit only when it is active and not at
		 * the highest level
		 */
		therm_ma = chip->thermal_mitigation[chip->therm_lvl_sel];
	}

	current_ma = min(therm_ma, chip->current_limit);
	fan54015_charger_set_fastchg_current(chip, current_ma);

	if(chip->charging_disabled_status & THERMAL){
		INFO("Current battery status: %d, re-enable charging. \n", chip->bat_health);
		rc = fan54015_charger_disable_charging(chip, THERMAL, false);
		if (rc < 0) {
			pr_err("Couldn't enable charging rc %d\n", rc);
			goto out;
		}
	}
out:
	mutex_unlock(&chip->current_change_lock);
	return rc;
}

static void
fan54015_charger_adc_notification(enum qpnp_tm_state state, void *ctx)
{
	struct fan54015 *chip = ctx;
    int temp;
	enum battery_health_type bh;

	if (state >= ADC_TM_STATE_NUM) {
		pr_err("invalid notification %d\n", state);
        return;
	}

	temp = fan54015_charger_get_batt_temp(chip);

	pr_debug("Current Temperature: %d State = %s\n", temp,
		state == ADC_TM_LOW_STATE ? "over_highthr" : "over_lowthr");

	bh = chip->bat_health;

	pr_debug("OLD STATE: highthr=%d, lowthr=%d, health=%d, notify=%d\n",
        chip->adc_param.high_temp, chip->adc_param.low_temp,chip->bat_health, chip->adc_param.state_request);

	if (state == ADC_TM_LOW_STATE)//higher then adc_param.high_temp
	{
		if(chip->bat_health == BATTERY_HEALTH_GOOD)
		{
			chip->bat_health = BATTERY_HEALTH_WARM;
			chip->adc_param.high_temp = 550;
			chip->adc_param.low_temp = 430;
			chip->adc_param.state_request = ADC_TM_HIGH_LOW_THR_ENABLE;
		}
		else if(chip->bat_health == BATTERY_HEALTH_OVERHEAT)
		{
			INFO("Error! batteryhealth: Overheat !\n");
		}
		else if(chip->bat_health == BATTERY_HEALTH_COLD)
		{
			chip->bat_health = BATTERY_HEALTH_GOOD;
			chip->adc_param.high_temp = 450;
			chip->adc_param.low_temp = 0;
			chip->adc_param.state_request = ADC_TM_HIGH_LOW_THR_ENABLE;
		}
		else if(chip->bat_health == BATTERY_HEALTH_WARM)
		{
			chip->bat_health = BATTERY_HEALTH_OVERHEAT;
			chip->adc_param.low_temp = 500;
			chip->adc_param.state_request = ADC_TM_HIGH_THR_ENABLE;
			INFO("Warnning! batteryhealth: Overheat !\n");
		}
	}
	else if (state == ADC_TM_HIGH_STATE)//lower then adc_param.low_temp
	{
		if(chip->bat_health == BATTERY_HEALTH_GOOD)
		{
			chip->bat_health = BATTERY_HEALTH_COLD;
			chip->adc_param.high_temp = 20;
			chip->adc_param.state_request = ADC_TM_LOW_THR_ENABLE;
			INFO("Warnning! batteryhealth: Cold !\n");
		}
		else if(chip->bat_health == BATTERY_HEALTH_OVERHEAT)
		{
			chip->bat_health = BATTERY_HEALTH_WARM;
			chip->adc_param.high_temp = 550;
			chip->adc_param.low_temp = 430;
			chip->adc_param.state_request = ADC_TM_HIGH_LOW_THR_ENABLE;
			INFO("Warnning! batteryhealth: Warm !\n");
		}
		else if(chip->bat_health == BATTERY_HEALTH_COLD)
		{
			INFO("Error! batteryhealth: Cold !\n");
		}
		else if(chip->bat_health == BATTERY_HEALTH_WARM)
		{
			chip->bat_health = BATTERY_HEALTH_GOOD;
			chip->adc_param.high_temp = 450;
			chip->adc_param.low_temp = 0;
			chip->adc_param.state_request = ADC_TM_HIGH_LOW_THR_ENABLE;
		}
	}

	pr_debug("NEW STATE: highthr=%d, lowthr=%d, health=%d, notify=%d\n",
                       chip->adc_param.high_temp, chip->adc_param.low_temp,chip->bat_health, chip->adc_param.state_request);
#if (!defined QPNP_NTC_DEBUG_SIMULATE)
	if (chip->bat_health != bh)
#endif
	{
		if(chip->bat_health == BATTERY_HEALTH_GOOD)
		{
			INFO("battery headth good, set charging current to:%d \n",
                THERMAL_MITIGATION_1450);
			/* set the limit current */
			fan54015_charger_temp_level_set(chip, THERMAL_MITIGATION_1450);
			power_supply_set_present(chip->usb, true);
		}
		else if(chip->bat_health == BATTERY_HEALTH_WARM)
		{
			INFO("battery headth warm, set charging current to:%d \n",
                THERMAL_MITIGATION_600);

            /* set the limit current */
            fan54015_charger_temp_level_set(chip, THERMAL_MITIGATION_600);
            power_supply_set_present(chip->usb, true);
        }
		else if((chip->bat_health == BATTERY_HEALTH_OVERHEAT) || (chip->bat_health == BATTERY_HEALTH_COLD))
		{
			INFO("battery headth overheat or cold, disable charging!\n");
			fan54015_charger_temp_level_set(chip, THERMAL_MITIGATION_0);
			power_supply_set_present(chip->usb, false);
		}
	}
#if (!defined QPNP_NTC_DEBUG_SIMULATE)
	if (qpnp_adc_tm_channel_measure(chip->adc_tm_dev, &chip->adc_param))
		pr_err("request ADC error\n");
#endif
}

static int fan54015_charger_get_current_now(struct fan54015 *chip)
{
	union power_supply_propval ret = {0,};

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
			  POWER_SUPPLY_PROP_CURRENT_NOW, &ret);

		return ret.intval;
	} else {
		pr_debug("No BMS supply registered return 0\n");
	}

	return 0;
}

static int fan54015_charger_get_voltage_now(struct fan54015 *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	rc = qpnp_vadc_read(chip->vadc_dev, VBAT_SNS, &results);
	if (rc) {
		pr_err("Unable to read vbat rc=%d\n", rc);
		return 0;
	}
	return results.physical;
}

#define DEFAULT_CAPACITY	50
static int fan54015_charger_get_capacity(struct fan54015 *chip)
{
	union power_supply_propval ret = {0,};
	int soc;

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_CAPACITY, &ret);
		soc = ret.intval;

		return soc;
	} else {
		pr_debug("No BMS supply registered return %d\n",
							DEFAULT_CAPACITY);
	}

	/*
	 * Return default capacity to avoid userspace
	 * from shutting down unecessarily
	 */
	return DEFAULT_CAPACITY;
}

static void fan54015_battery_set_charging_enable(struct fan54015 *chip, int enable)
{
	chip->charging_disabled = !enable;
	schedule_delayed_work(&chip->charger_workqueue ,T1S_SET_CHARGRE_INTERVAL);
}

static int fan54015_battery_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct fan54015 *chip = container_of(psy, struct fan54015, battery);
	switch (psp) {
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = fan54015_charger_get_batt_health(chip);
/* [PLATFORM]-Mod-BEGIN by TCTNB.FLF, FR-644906, 2014/05/04, disable tem check for mini */
#ifdef FEATURE_TCTNB_MMITEST
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
#endif
/* [PLATFORM]-Mod-END by TCTNB.FLF */
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = fan54015_charger_get_charging_status(chip);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = fan54015_charger_get_batt_status(chip);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = fan54015_charger_get_capacity(chip);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = fan54015_charger_get_current_now(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = fan54015_charger_get_voltage_now(chip);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = fan54015_charger_get_batt_temp(chip);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
/* [PLATFORM]-Mod-BEGIN by TCTSZ.Leo.guo, FR-801049, 2014/10/09, disable charge interface for mini */
static int fan54015_battery_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}
	return rc;
}
/* [PLATFORM]-Mod-END by TCTSZ.Leo.guo, FR-801049, 2014/10/09*/
static int fan54015_battery_set_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       const union power_supply_propval *val)
{
	struct fan54015 *chip = container_of(psy, struct fan54015, battery);
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		fan54015_battery_set_charging_enable(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		break;
	case POWER_SUPPLY_PROP_STATUS:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int fan54015_battery_register(struct fan54015 *chip)
{
	int ret;
	chip->battery.name = "battery";
	chip->battery.type = POWER_SUPPLY_TYPE_BATTERY;
	chip->battery.properties = fan54015_charger_props;
	chip->battery.num_properties = ARRAY_SIZE(fan54015_charger_props);
	chip->battery.get_property = fan54015_battery_get_property;
	chip->battery.set_property = fan54015_battery_set_property;
	chip->battery.property_is_writeable = fan54015_battery_is_writeable;

	ret = power_supply_register(chip->dev, &chip->battery);
	if (ret) {
		pr_err("failed to register battery: %d\n", ret);
	}

	return 0;
}

static int fan54015_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	return 0;
}

static int fan54015_otg_regulator_enable(struct regulator_dev *rdev)
{
	int rc;
	struct fan54015 *chip = rdev_get_drvdata(rdev);
#if defined CONFIG_QPNP_POWER_SWITCH_SUPPORT
	if(chip->vbus_dec)
		rc = regulator_enable(chip->vbus_dec);
	if(rc)
		pr_err("Couldn't enable vbus_dec\n");
#endif
	fan54015_set_value(chip, FAN54015_REG_CONTROL1, FAN54015_HZ_MODE, FAN54015_HZ_MODE_SHIFT, !HIGH_IMPEDANCE_MODE);
	fan54015_set_value(chip, FAN54015_REG_CONTROL1, FAN54015_OPA_MODE, FAN54015_OPA_MODE_SHIFT, FAN54015_BOOST_MODE);
	return 0;
}

static int fan54015_otg_regulator_disable(struct regulator_dev *rdev)
{
	int rc;
	struct fan54015 *chip = rdev_get_drvdata(rdev);
#if defined CONFIG_QPNP_POWER_SWITCH_SUPPORT
	if(chip->vbus_dec)
		rc = regulator_disable(chip->vbus_dec);
	if(rc)
		pr_err("Couldn't disable vbus_dec\n");
#endif
	fan54015_set_value(chip, FAN54015_REG_CONTROL1, FAN54015_OPA_MODE, FAN54015_OPA_MODE_SHIFT, FAN54015_CHARGE_MODE);
	return 0;
}

struct regulator_ops fan54015_otg_reg_ops = {
	.enable		= fan54015_otg_regulator_enable,
	.disable	= fan54015_otg_regulator_disable,
	.is_enabled = fan54015_otg_regulator_is_enable,
};

static int fan54015_regulator_init(struct fan54015 *chip)
{
	int rc = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};

	init_data = of_get_regulator_init_data(chip->dev, chip->dev->of_node);
	if (!init_data) {
		dev_err(chip->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	if (init_data->constraints.name) {
		chip->otg_vreg.rdesc.owner = THIS_MODULE;
		chip->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		chip->otg_vreg.rdesc.ops = &fan54015_otg_reg_ops;
		chip->otg_vreg.rdesc.name = init_data->constraints.name;

		cfg.dev = chip->dev;
		cfg.init_data = init_data;
		cfg.driver_data = chip;
		cfg.of_node = chip->dev->of_node;

		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_STATUS;

		chip->otg_vreg.rdev = regulator_register(
					&chip->otg_vreg.rdesc, &cfg);
		if (IS_ERR(chip->otg_vreg.rdev)) {
			rc = PTR_ERR(chip->otg_vreg.rdev);
			chip->otg_vreg.rdev = NULL;
			if (rc != -EPROBE_DEFER)
				dev_err(chip->dev,
					"OTG reg failed, rc=%d\n", rc);
		}
	}

	return rc;
}

static int fan54015_show_regsister(struct seq_file *m, void *data)
{
	struct fan54015 *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = 0x00; addr <= 0x06; addr++) {
		rc = __fan54015_read(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int fan54015_debugfs_open(struct inode *inode, struct file *file)
{
	struct fan54015 *chip = inode->i_private;

	return single_open(file, fan54015_show_regsister, chip);
}

static const struct file_operations fan54015_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= fan54015_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#if (defined QPNP_NTC_DEBUG_SIMULATE)
static int ntc_debugfs_show(struct seq_file *m, void *data)
{
	struct fan54015 *chip = m->private;
	seq_printf(m, "Battery Health = 0x%02x\n", chip->bat_health);
	return 0;
}

static ssize_t ntc_debugfs_simulate(struct file *file, const char __user *ubuf,
				size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct fan54015 *chip = s->private;

	char buf[64];

	memset(buf, 0x00, sizeof(buf));

	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
		return -EFAULT;

	if (!strncmp(buf, "up", 2)) {
		fan54015_charger_adc_notification(ADC_TM_LOW_STATE, chip);
	}
	else if (!strncmp(buf, "down", 4)) {
		fan54015_charger_adc_notification(ADC_TM_HIGH_STATE, chip);
	}
	else{
		printk("Usage: echo [up/down] > /sys/kernel/debug/simulate\n");
	}

	return count;
}

static int ntc_debugfs_open(struct inode *inode, struct file *file)
{
	struct fan54015 *chip = inode->i_private;

	return single_open(file, ntc_debugfs_show, chip);
}

static const struct file_operations ntc_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= ntc_debugfs_open,
	.read		= seq_read,
	.write		= ntc_debugfs_simulate,
	.llseek		= seq_lseek,
	.release	= single_release,
};
#endif

static int fan54015_charger_parse_parameters(struct fan54015 *chip)
{
	int rc;
	struct device_node *node = chip->dev->of_node;

	if (!node) {
		dev_err(chip->dev, "device tree info. missing\n");
		return -EINVAL;
	}

#if defined CONFIG_QPNP_DOCK_POWER_SUPPLY_SUPPORT
	chip->chg_dock_dec = of_get_named_gpio(node, "qcom,chg-dock-dec", 0);
#endif

	rc = of_property_read_u32(node, "qcom,float-voltage-mv",
						&chip->vfloat_mv);
	if (rc < 0)
		chip->vfloat_mv = -EINVAL;

	rc = of_property_read_u32(node, "qcom,iterm-ma", &chip->iterm_ma);
	if (rc < 0)
		chip->iterm_ma = -EINVAL;

#if defined CONFIG_QPNP_POWER_SWITCH_SUPPORT
	chip->vbus_dec = devm_regulator_get(chip->dev, "vbus_dec");
	if (IS_ERR(chip->vbus_dec)) {
		dev_err(chip->dev, "Couldn't vbus detect regulator.");
		return -EPROBE_DEFER;
	}
#endif

	chip->vadc_dev = qpnp_get_vadc(chip->dev, "fan54015");
	if (IS_ERR(chip->vadc_dev)) {
		rc = PTR_ERR(chip->vadc_dev);
		chip->vadc_dev = NULL;
			if (rc != -EPROBE_DEFER)
				dev_err(chip->dev, "Couldn't get vadc device, rc=%d\n", rc);
		return rc;
	}

	if (of_find_property(node, "qcom,thermal-mitigation",
					&chip->thermal_levels)) {
		chip->thermal_mitigation = devm_kzalloc(chip->dev,
					chip->thermal_levels,
						GFP_KERNEL);

		if (chip->thermal_mitigation == NULL) {
			pr_err("thermal mitigation kzalloc() failed.\n");
			return -ENOMEM;
		}

		chip->thermal_levels /= sizeof(int);
		rc = of_property_read_u32_array(node,
				"qcom,thermal-mitigation",
				chip->thermal_mitigation, chip->thermal_levels);
		if (rc) {
			pr_err("Couldn't read threm limits rc = %d\n", rc);
			return rc;
		}
	}

	rc = of_property_read_u32(node, "qcom,warm-bat-decidegc", &chip->cfg_warm_bat_decidegc);
	if (rc < 0)
		chip->cfg_warm_bat_decidegc = -EINVAL;
	rc = of_property_read_u32(node, "qcom,cool-bat-decidegc", &chip->cfg_cool_bat_decidegc);
	if (rc < 0)
		chip->cfg_cool_bat_decidegc = -EINVAL;

	if (rc) {
		pr_err("Error reading battery temp prop rc=%d\n", rc);
		return rc;
	}

	if (chip->cfg_cool_bat_decidegc || chip->cfg_warm_bat_decidegc) {
		chip->adc_tm_dev = qpnp_get_adc_tm(chip->dev, "chg");
		if (IS_ERR(chip->adc_tm_dev)) {
			rc = PTR_ERR(chip->adc_tm_dev);
			if (rc != -EPROBE_DEFER)
				pr_err("Failed to get adc-tm rc=%d\n", rc);
			return rc;
		}
	}

	return 0;
}

static int fan54015_charger_probe(
    struct i2c_client *client, const struct i2c_device_id *id)
{
    int rc = 0;
	struct fan54015 *chip;
	struct power_supply *usb, *bms;

	chip = kzalloc(sizeof(struct fan54015), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	usb = power_supply_get_by_name("usb");
	if (!usb) {
		dev_dbg(chip->dev, "USB supply not found; defer probe\n");
		return -EPROBE_DEFER;
	}
	chip->usb = usb;

	bms = power_supply_get_by_name("bms");
	if (!bms){
		dev_dbg(chip->dev, "bms supply not found; defer probe\n");
		return -EPROBE_DEFER;
	}
	chip->bms_psy = bms;

	chip->dev = &client->dev;
	chip->client = client;
	chip->irq = client->irq;

	mutex_init(&chip->read_write_lock);
	mutex_init(&chip->charging_disable_lock);
	mutex_init(&chip->current_change_lock);

	i2c_set_clientdata(client, chip);

	rc = fan54015_regulator_init(chip);
	if  (rc) {
		dev_err(&client->dev,
			"Couldn't initialize smb349 ragulator rc=%d\n", rc);
		return rc;
	}

	rc = fan54015_charger_parse_parameters(chip);
	if (rc < 0) {
		dev_err(chip->dev, "Unable to parse DT nodes\n");
		return rc;
	}

	rc = fan54015_charger_init_device(chip);
	if (rc) {
		dev_err(chip->dev, "device init failure: %d\n", rc);
		goto fail_init;
	}
	wake_lock_init(&chip->monitor_wake_lock, WAKE_LOCK_SUSPEND,
			"fan54015-monitor");

//[PLATFORM]-Modify-BEGIN by TCTSZ.Leo.guo, 2014/09/02, added for dock charger support
#if defined CONFIG_QPNP_DOCK_POWER_SUPPLY_SUPPORT
	INIT_DELAYED_WORK(&chip->dock_set_work, fan54015_charger_dock_update_work);
	if (gpio_is_valid(chip->chg_dock_dec)) {
		/** gpio request for dock charger detect **/
		rc = gpio_request(chip->chg_dock_dec, "CHG_DOCK_DECTECT");
		if (rc < 0) {
			dev_err(chip->dev, "gpio req failed for charger detect.\n");
			chip->chg_dock_dec = 0;
			return rc;
		}
		chip->dock_irq = gpio_to_irq(chip->chg_dock_dec);
		if (chip->dock_irq) {
			rc = request_irq(chip->dock_irq,
					  fan54015_charger_dock_irq_callback,
					  IRQF_TRIGGER_RISING,
					  "charger_dock_irq", chip);
			if (rc) {
				dev_err(chip->dev, "request irq failed for dock detect.\n");
				return rc;
			}
		}
	}
#endif
//[PLATFORM]-Modify-BEGIN by TCTSZ.Leo.guo, 2014/09/02

	if (chip->cfg_cool_bat_decidegc || chip->cfg_warm_bat_decidegc){
		chip->adc_param.low_temp = chip->cfg_cool_bat_decidegc;
		chip->adc_param.high_temp = chip->cfg_warm_bat_decidegc;
		chip->adc_param.timer_interval = ADC_MEAS1_INTERVAL_1S;
		chip->adc_param.state_request = ADC_TM_HIGH_LOW_THR_ENABLE;
		chip->adc_param.btm_ctx = chip;
		chip->adc_param.threshold_notification =
			fan54015_charger_adc_notification;
		chip->adc_param.channel = LR_MUX1_BATT_THERM;

		rc = qpnp_adc_tm_channel_measure(chip->adc_tm_dev,
					&chip->adc_param);
		if (rc) {
			pr_err("request ADC error rc=%d\n", rc);
		}
	}

	rc = fan54015_battery_register(chip);
	if (rc < 0) {
		dev_err(&client->dev,
			"Unable to register batt_psy rc = %d\n", rc);
		goto fail_init;
	}

	chip->debug_root = debugfs_create_dir("fan54015", NULL);
	if (!chip->debug_root)
		dev_err(chip->dev, "Couldn't create debug dir\n");

	if (chip->debug_root) {
		struct dentry *ent;
		ent = debugfs_create_file("registers", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &fan54015_debugfs_ops);
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create status debug file rc = %d\n",
				rc);
#if (defined QPNP_NTC_DEBUG_SIMULATE)
		ent = debugfs_create_file("simulate", S_IFREG | S_IRUGO | S_IWUSR,
					  chip->debug_root, chip,
					  &ntc_debugfs_ops);
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create count debug file rc = %d\n",
				rc);
#endif
	}
	return 0;
fail_init:
	regulator_unregister(chip->otg_vreg.rdev);
	kfree(chip);
	return rc;
}

static void fan54015_charger_shutdown(struct i2c_client *client)
{
	struct fan54015 *chip = i2c_get_clientdata(client);
	fan54015_set_value(chip, FAN54015_REG_IBAT, FAN54015_RESET, FAN54015_RESET_SHIFT, 0x01);
}

static int fan54015_charger_remove(struct i2c_client *client)
{
    return 0;
}

static int  fan54015_charger_suspend(struct i2c_client *client, pm_message_t message)
{
    return 0;
}

static int  fan54015_charger_resume(struct i2c_client *client)
{
    return 0;
}

static struct of_device_id fan54015_match_table[] = {
	{ .compatible = "qcom,fan54015_charger",},
	{ },
};

static const struct i2c_device_id fan54015_i2c_id[] = {
	{ "fan54015", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, fan54015_i2c_id);

static struct i2c_driver fan54015_charger_driver = {
    .driver = {
		.name		= "fan54015_charger",
		.owner		= THIS_MODULE,
		.of_match_table	= fan54015_match_table,
    },
    .probe    = fan54015_charger_probe,
    .remove   = fan54015_charger_remove,
    .suspend  = fan54015_charger_suspend,
    .resume   = fan54015_charger_resume,
    .shutdown = fan54015_charger_shutdown,
    .id_table = fan54015_i2c_id,
};
module_i2c_driver(fan54015_charger_driver);

MODULE_AUTHOR("Bright Yang<bright.yang@fairchildsemi.com>");
MODULE_DESCRIPTION("I2C bus driver for FAN54015 Switching Charger");
MODULE_LICENSE("GPL v2");
