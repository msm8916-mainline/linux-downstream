/* Copyright (c) 2013 LGE Inc. All rights reserved.
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

#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/debugfs.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/bitops.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/reboot.h>
#include <linux/wakelock.h>
#include <soc/qcom/smem.h>
#include <linux/init.h>

#ifdef CONFIG_LGE_PM_CHARGING_SAFETY_TIMER
#include <linux/time.h>
#include <linux/timer.h>
#endif

#ifdef CONFIG_SENSORS_QPNP_ADC_VOLTAGE
#include <linux/qpnp/qpnp-adc.h>
#endif

#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
#include <mach/lge_charging_scenario.h>
#endif

#include <linux/usb/otg.h>
#include <linux/power/bq24262_charger.h>

#ifdef CONFIG_LGE_PM_PSEUDO_BATTERY
#include <mach/board_lge.h>
#endif

#include <mach/rpm-regulator.h>

#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
#define MONITOR_BATTEMP_POLLING_PERIOD          (600*100)
#endif
#define IRQ_CHECK_PERIOD 	(5000)
#ifdef CONFIG_LGE_PM_DEBUG_CHECK_LOG
#define DEBUG_POLLING_PERIOD 	(300*100)
#endif

#ifndef BIT
#define BIT(x)	(1 << (x))
#endif

/* Register definitions */
#define R00_STATUS_CONTROL_REG	          	0x00
#define R01_CONTROL_REG			0x01
#define R02_CONTROL_BAT_VOL_REG		0x02
#define R03_VENDER_PART_REV_REG		0x03
#define R04_BAT_TERM_FAST_CHARGE_CUR_REG	0x04
#define R05_VINDPM_VOL_DPPM_STAT_REG		0x05
#define R06_SAFETY_TMR_NTC_MON_REG		0x06

/* REG00 Status Control Register */
#define TMR_RST                 BIT(7)
#define BOOST_MODE_MASK         BIT(6)
#define CHRG_STAT_MASK          (BIT(5)|BIT(4))
#define EN_SHIPMODE             BIT(3)
#define CHG_FAULT_MASK          (BIT(2)|BIT(1)|BIT(0))

/* REG01 Control Register */
#define IINLIM_MASK             (BIT(7)|BIT(6)|BIT(5)|BIT(4))
#define EN_STAT                 (BIT(7)|BIT(3))
#define EN_CHG_TERM_MASK        (BIT(7)|BIT(2))
#define CHG_CONFIG_MASK         (BIT(7)|BIT(1))
#define HZ_MODE                 (BIT(7)|BIT(0))
//#define ALL_REG_CLEAR      BIT(7)

/* REG02 Control Battery Voltage Register */
#define VBREG_MASK              (BIT(7)|BIT(6)|BIT(5)|BIT(4)|BIT(3)|BIT(2))
#define MOD_FREQ                (BIT(1)|BIT(0))


/* REG03 Control INFO Register */
#define VENDER                  (BIT(7)|BIT(6)|BIT(5))
#define PART_NUMBER             (BIT(4)|BIT(3))
#define REVISION                (BIT(2)|BIT(1)|BIT(0))


/* REG04 Battery Termination Fast Charge Current Register */
#define ICHG_MASK               (BIT(7)|BIT(6)|BIT(5)|BIT(4)|BIT(3))
#define ITERM_MASK              (BIT(2)|BIT(1)|BIT(0))


/* REG05 VINDPM Voltage DPPM Status Register */
#define MINSYS_STATUS           BIT(7)
#define DPM_STATUS              BIT(6)
#define LOW_CHG                 BIT(5)
#define DPDM_EN                 BIT(4)
#define CD_STATUS               BIT(3)
#define VINDPM_MASK             (BIT(2)|BIT(1)|BIT(0))


/* REG06 Safety Timer/ NTC Monitor Register */
#define XTMR_EN                 BIT(7)
#define TMR_BIT                 (BIT(6)|BIT(5))
#define BOOST_ILIM              BIT(4)
#define TS_EN                   BIT(3)
#define TS_FAULT                (BIT(2)|BIT(1))
#define BINDPM_OFF              BIT(0)

#ifndef CONFIG_LGE_PM_CHARGING_BQ24262_CHARGER
/* BQ05 Charge Termination, Timer-Control Register MASK */
#define I2C_TIMER_MASK          (BIT(5)|BIT(4))
#define EN_CHG_TIMER_MASK	BIT(3)
#define CHG_TIMER_MASK 		(BIT(2)|BIT(1))

/* BQ06 IR Compensation, Thermal Regulation Control Register MASK */
#define IR_COMP_R_MASK		(BIT(7)|BIT(6)|BIT(5))
#define IR_COMP_VCLAMP_MASK 	(BIT(4)|BIT(3)|BIT(2))

/* BQ07 Misc-Operation Control Register MASK */
#define BATFET_DISABLE_MASK 	BIT(5)

/* BQ08 SYSTEM_STATUS_REG Mask */
#define VBUS_STAT_MASK 		(BIT(7)|BIT(6))
#define PRE_CHARGE_MASK 	BIT(4)
#define FAST_CHARGE_MASK 	BIT(5)
#define DPM_STAT_MASK		BIT(3)
#define PG_STAT_MASK		BIT(2)
#define THERM_STAT_MASK 	BIT(1)
#define VSYS_STAT_MASK 		BIT(0)

/* BQ09 FAULT_REG Mask */
#define CHRG_FAULT_MASK 	(BIT(5)|BIT(4))
#endif

#define LEVEL_CH_UV_TO_MV 1000

#define LT_CABLE_56K		6
#define LT_CABLE_130K		7
#define LT_CABLE_910K		11


#define INPUT_CURRENT_LIMIT_100mA	100
#define INPUT_CURRENT_LIMIT_150mA	150
#define INPUT_CURRENT_LIMIT_500mA	500
#define INPUT_CURRENT_LIMIT_900mA	900
#define INPUT_CURRENT_LIMIT_1500mA	1500
#define INPUT_CURRENT_LIMIT_2500mA	2500

#ifdef CONFIG_LGE_PM_USB_CURRENT_MAX
#define USB_CURRENT_MAX 900
#endif

enum bq24262_chg_status {
	BQ_CHG_STATUS_NONE 		= 0,
	BQ_CHG_STATUS_FAST_CHARGE 	= 1,
	BQ_CHG_STATUS_FULL          = 2,
	BQ_CHG_STATUS_EXCEPTION		= 3,
};

static const char * const bq24262_chg_status[] = {
	"none",
	"fast-charge",
	"full",
	"exception"
};

static const char * const chg_state_str[] = {
	"Ready",
	"Charge in progress",
	"Charge done",
	"Fault"
};
#ifdef CONFIG_CHECK_NULL_SCRIPT
#define NULL_CHECK_VOID(p)	\
	if (!(p)) { 	\
		pr_err("FATAL (%s)\n", __func__);	\
		return ;	\
	}
#define NULL_CHECK(p, err)	\
	if (!(p)) {	\
		pr_err("FATAL (%s)\n", __func__);	\
		return err;	\
	}
#endif
enum bq24262_chg_fault_state {
	BQ_FAULT_NORMAL,
	BQ_FAULT_VIN_OVP,
	BQ_FAULT_LOW_SUPPLY,
	BQ_FAULT_THERMAL_SHUTDOWN,
	BQ_FAULT_BATT_TEMP,
	BQ_FAULT_TIMER,
	BQ_FAULT_BATT_OVP,
	BQ_FAULT_NO_BATT,
};

static const char * const fault_str[] = {
	"Normal",
	"Boost Mode OVP",
	"Low Supply or Boost Mode Over",
	"Thermal Shutdown"
	"Battery Temp Fault"
	"Timer Fault Watchdoc"
	"Battery OVP"
	"No Battery"
};

struct bq24262_chip {
	struct i2c_client  *client;
	struct dentry  *dent;

	int  chg_current_ma;
	/* init current ma for thermal engine  */
	int  chg_current_init_ma;
	int  term_current_ma;
	int  int_gpio;
	int  irq;

	int host_mode;
	int usb_online;
	int ac_present;
	int ac_online;
	int chg_type;

	int ext_chg_disen;
	int system_vol_status;
	int stat_gpio;
	bool suspend;
	int vin_limit_mv;
	int cur_limit_ma;
	bool batt_present;
#ifdef CONFIG_LGE_PM_DEBUG_CHECK_LOG
	struct delayed_work 	charging_inform_work;
#endif
	enum bq24262_chg_status	chg_status;
	enum bq24262_chg_status	chg_cable_status;
	enum bq24262_chg_fault_state fault_state;

	struct delayed_work  irq_work;
	struct delayed_work  update_heartbeat_work;
	struct delayed_work  dpm_detect_work;
#ifdef CONFIG_LGE_PM_BQ2426X_USING_WATCHDOG
	struct delayed_work  check_watchdog_work;
	struct mutex setting_env;
#endif
	struct wake_lock        uevent_wake_lock;
#ifdef CONFIG_SENSORS_QPNP_ADC_VOLTAGE
	struct qpnp_vadc_chip *vadc_dev;
#endif

#ifdef CONFIG_LGE_THERMALE_CHG_CONTROL
	int chg_current_te;
#endif
#ifdef I2C_SUSPEND_WORKAROUND
	struct delayed_work 	check_suspended_work;
#endif
	bool usb_present;
	bool invalid_temp;
	struct wake_lock  chg_wake_lock;
	struct power_supply  *usb_psy;
	struct power_supply  ac_psy;
	struct power_supply  batt_psy;
	bool check_eoc_complete;
#ifdef CONFIG_LGE_PM_CHARGING_SAFETY_TIMER
	int safety_timeout;
	bool safety_time_enable;
	bool safety_chg_done;
	int safety_counter;
#endif

#ifndef CONFIG_THERMAL_QPNP_ADC_TM
	struct qpnp_adc_tm_btm_param  adc_param;
#endif

#ifdef CONFIG_LGE_PM_USB_CURRENT_MAX
	bool usb_current_max_enabled;
#endif

#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
	enum lge_btm_states 	btm_state;
	int pseudo_ui_chg;
	struct wake_lock		lcs_wake_lock;
	struct delayed_work 	battemp_work;

	int otp_ibat_current;
	bool reached_temp_level;
	bool thermal_engine_control;
	bool probe_success;
#endif
#ifdef CONFIG_LGE_PM_BATTERY_EXTERNAL_FUELGAUGE
	struct power_supply  *fuelgauge;
#endif
//	int set_chg_current_ma;
	int regulation_mV;
};

#ifdef CONFIG_LGE_PM_CHARGING_SAFETY_TIMER
struct timer_list safety_timer;
#endif

#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
bool	bq24262_stpchg_factory_testmode;
bool	bq24262_start_chg_factory_testmode;
#endif
#ifdef CONFIG_LGE_PM_CHARGING_TEST_FOR_ART
bool 	setting_chg_current_testmode;
#endif

static struct bq24262_chip *the_chip;

#ifdef CONFIG_LGE_PM_PSEUDO_BATTERY
extern struct pseudo_batt_info_type pseudo_batt_info;
#endif

struct debug_reg {
	char  *name;
	u8  reg;
};

#define FACTORY_CABLE_CAPACITY 75
#define FACTORY_CABLE_VOLTAGE 3850
#define FACTORY_CABLE_TEMP        400

#define BQ24262_DEBUG_REG(x) {#x, x##_REG}

static struct debug_reg bq24262_debug_regs[] = {
	BQ24262_DEBUG_REG(R00_STATUS_CONTROL),
	BQ24262_DEBUG_REG(R01_CONTROL),
	BQ24262_DEBUG_REG(R02_CONTROL_BAT_VOL),
	BQ24262_DEBUG_REG(R03_VENDER_PART_REV),
	BQ24262_DEBUG_REG(R04_BAT_TERM_FAST_CHARGE_CUR),
	BQ24262_DEBUG_REG(R05_VINDPM_VOL_DPPM_STAT),
	BQ24262_DEBUG_REG(R06_SAFETY_TMR_NTC_MON),
};

static unsigned int last_stop_charging;
static int previous_batt_temp;
static int charger_type_check;
static int bq24262_charging_state = 1; /* 1 : charing, 0 : bq24262 charging */
int lge_power_test_flag_charger = 0;

static int bq24262_enable_charging(struct bq24262_chip *chip, bool enable);
static int bq24262_set_hz_mode(struct bq24262_chip *chip, bool enable);
static int bq24262_get_hz_mode(struct bq24262_chip *chip);
static void bq24262_charging_setting(struct bq24262_chip *chip);
void bq24262_charging_set(int val);
static int bq24262_get_prop_batt_present(struct bq24262_chip *chip);
static int bq24262_get_prop_charge_type(struct bq24262_chip *chip);
static int bq24262_set_input_i_limit(struct bq24262_chip *chip, int ma);
static int bq24262_get_input_i_limit(struct bq24262_chip *chip);
static bool bq24262_is_charger_present(struct bq24262_chip *chip);
static bool bq24262_is_otg_mode(struct bq24262_chip *chip);
static int bq24262_set_ibat_max(struct bq24262_chip *chip, int ma);
static int bq24262_enable_otg(struct bq24262_chip *chip, bool enable);
static int bq24262_set_vbat_max(struct bq24262_chip *chip, int mv);
static int bq24262_get_dpm_state(struct bq24262_chip * chip);
static void bq24262_set_clear_reg(struct bq24262_chip *chip);
static unsigned int cable_type;
static int bq24262_get_vbat_max(struct bq24262_chip *chip);

static bool is_factory_cable(void)
{
	unsigned int cable_info;
	cable_info = lge_pm_get_cable_type();

	if ((cable_info == CABLE_56K ||
		cable_info == CABLE_130K ||
		cable_info == CABLE_910K) ||
		(cable_type == LT_CABLE_56K ||
		cable_type == LT_CABLE_130K ||
		cable_type == LT_CABLE_910K))
		return true;
	else
		return false;
}

static bool is_factory_cable_56k(void)
{
	unsigned int cable_info;
	cable_info = lge_pm_get_cable_type();

	if (cable_info == CABLE_56K ||
			cable_type == LT_CABLE_56K)
		return true;
	else
		return false;
}

static bool is_factory_cable_130k(void)
{
	unsigned int cable_info;
	cable_info = lge_pm_get_cable_type();

	if (cable_info == CABLE_130K ||
			cable_type == LT_CABLE_130K)
		return true;
	else
		return false;
}

static bool is_factory_cable_910k(void)
{
	unsigned int cable_info;
	cable_info = lge_pm_get_cable_type();

	if (cable_info == CABLE_910K ||
			cable_type == LT_CABLE_910K)
		return true;
	else
		return false;
}

#ifdef CONFIG_LGE_PM_CHARGING_SAFETY_TIMER
static DEFINE_SPINLOCK(timer_lock);

static void bq24262_timer_func(unsigned long timer);

static void bq24262_safety_time_start(void) {
//	init_timer(&safety_timer);					/* init timer */
	safety_timer.expires = jiffies + (60 * HZ);	/* 60 sec unit */
	safety_timer.function = bq24262_timer_func;
	safety_timer.data = the_chip->safety_timeout;
	add_timer(&safety_timer);
	pr_debug("safety timer init\n");
}

static void bq24262_safety_time_end(void) {
	del_timer(&safety_timer);
	pr_debug("safety time end\n");
}

static void bq24262_start_set_safety_timer(void) {
	unsigned long flags;

	if (!the_chip->safety_time_enable)
		return;

	spin_lock_irqsave(&timer_lock, flags);
	bq24262_safety_time_start();
	spin_unlock_irqrestore(&timer_lock, flags);
}

static void bq24262_timer_func(unsigned long timer) {

	if(the_chip->ac_present)
		the_chip->safety_counter++;

	bq24262_safety_time_end();
	pr_debug("timer : %d, safety : %d\n", the_chip->safety_counter, the_chip->safety_timeout);
	if ((the_chip->safety_counter == the_chip->safety_timeout)
			&& !(the_chip->check_eoc_complete)) {
		the_chip->safety_chg_done = true;
		the_chip->safety_counter = the_chip->safety_timeout;
		pr_err("Time up stop charging\n");
	} else if (the_chip->check_eoc_complete) {
		the_chip->safety_counter = 0;
		pr_err("Cancel safety time by chg_done : %d\n",
				the_chip->check_eoc_complete);
		if (wake_lock_active(&the_chip->chg_wake_lock))
			wake_unlock(&the_chip->chg_wake_lock);
	} else if (!the_chip->safety_time_enable) {
		the_chip->safety_counter = 0;
		pr_err("Cancel safety time by disable\n");
	} else {
		the_chip->safety_chg_done = false;
		pr_info("Not yet timeout\n");
		pr_debug("ac_present : %d, chg_status : %d\n",
				the_chip->ac_present, the_chip->chg_cable_status);
		if(the_chip->ac_present &&
				the_chip->chg_cable_status == BQ_CHG_STATUS_FAST_CHARGE) {
			if (the_chip->safety_counter == 0)
				pr_info("Start to safety timer\n");

			bq24262_start_set_safety_timer();
		} else {
			pr_err("Not setting safety timer by stop charging\n");
			the_chip->safety_counter = 0;
		}
	}

}
#endif

static void (*bq24262_notify_vbus_state_func_ptr)(int);
int bq24262_charger_register_vbus_sn(void (*callback)(int))
{
	pr_debug("%p\n", callback);
	bq24262_notify_vbus_state_func_ptr = callback;
	return 0;
}
EXPORT_SYMBOL_GPL(bq24262_charger_register_vbus_sn);

/* this is passed to the hsusb via platform_data msm_otg_pdata */
void bq24262_charger_unregister_vbus_sn(void (*callback)(int))
{
	pr_debug("%p\n", callback);
	bq24262_notify_vbus_state_func_ptr = NULL;
}
EXPORT_SYMBOL_GPL(bq24262_charger_unregister_vbus_sn);

void bq24262_charger_force_update_batt_psy(void)
{
	struct bq24262_chip *chip = the_chip;

	if (!chip) {
		pr_err("called before init\n");
		return;
	}

	chip = the_chip;
	pr_debug("%s\n",__func__);
	power_supply_changed(&chip->batt_psy);
}
EXPORT_SYMBOL_GPL(bq24262_charger_force_update_batt_psy);

int32_t bq24262_is_ready(void)
{
	struct bq24262_chip *chip = the_chip;

	if (!chip)
		return -EPROBE_DEFER;
	return 0;
}
EXPORT_SYMBOL(bq24262_is_ready);


static void bq24262_notify_usb_of_the_plugin_event(int plugin)
{
	plugin = !!plugin;
	if (bq24262_notify_vbus_state_func_ptr) {
		pr_debug("notifying plugin\n");
		(*bq24262_notify_vbus_state_func_ptr) (plugin);
	} else {
		pr_debug("unable to notify plugin\n");
	}
}

static void bq24262_get_state(struct bq24262_chip *chip, u8 reg_r00)
{
	u8 reg_chgstate_mask = 0;

	reg_chgstate_mask = reg_r00 & CHRG_STAT_MASK;
	chip->chg_cable_status = reg_chgstate_mask >> 4;
	chip->fault_state = reg_r00 & CHG_FAULT_MASK;

	pr_debug("chg_state [%s] fault state [%s]\n",
			bq24262_chg_status[chip->chg_status],fault_str[chip->fault_state]);

	return;
}

static unsigned int cable_smem_size;
static int bq24262_read_reg(struct i2c_client *client, int reg, u8 *val)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "i2c read fail: can't read from %02x: %d\n",
				reg, ret);
		return ret;
	} else {
		*val = ret;
	}

	return 0;
}

static int bq24262_write_reg(struct i2c_client *client, int reg, u8 val)
{
	s32 ret;
	pr_debug("%s reg = %x val = %x\n",__func__, reg,val);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		dev_err(&client->dev, "i2c write fail: can't write %02x to %02x: %d\n",
				val, reg, ret);
		return ret;
	}
	return 0;
}

static int bq24262_masked_write(struct i2c_client *client, int reg,
		u8 mask, u8 val)
{
	s32 rc;
	u8 temp;

	rc = bq24262_read_reg(client, reg, &temp);
	if (rc) {
		pr_err("bq24262_read_reg failed: reg=%03X, rc=%d\n",
				reg, rc);
		return rc;
	}

	temp &= ~mask;
	temp |= val & mask;

	pr_debug("%s : reg = %x mask %x val = %x temp = %x\n",
			__func__,reg,mask,val,temp);

	rc = bq24262_write_reg(client, reg, temp);
	if (rc) {
		pr_err("bq24262_write_reg failed: reg=%03X, rc=%d\n",
				reg, rc);
		return rc;
	}

	return 0;
}

static int set_reg(void *data, u64 val)
{
	u32 addr = (u32) data;
	int ret;
	struct i2c_client *client = the_chip->client;

	ret = bq24262_write_reg(client, addr, (u8) val);

	return ret;
}

static int get_reg(void *data, u64 *val)
{
	u32 addr = (u32) data;
	u8 temp;
	int ret;
	struct i2c_client *client = the_chip->client;

	ret = bq24262_read_reg(client, addr, &temp);
	if (ret < 0)
		return ret;

	*val = temp;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(reg_fops, get_reg, set_reg, "0x%02llx\n");


static char *bq24262_power_supplied_to[] = {
	"battery",
};

static enum power_supply_property bq24262_power_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED,
	POWER_SUPPLY_PROP_CHARGING_COMPLETE,
#ifdef CONFIG_LGE_PM_CHARGING_SAFETY_TIMER
	POWER_SUPPLY_PROP_SAFETY_CHARGER_TIMER,
#endif
};

static enum power_supply_property bq24262_batt_power_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_INPUT_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_EXT_PWR_CHECK,
#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
	POWER_SUPPLY_PROP_BATTERY_ID_CHECKER,
	POWER_SUPPLY_PROP_VALID_BATT,
#endif

#ifdef CONFIG_LGE_PM_PSEUDO_BATTERY
	POWER_SUPPLY_PROP_PSEUDO_BATT,
#endif

#ifdef CONFIG_LGE_PM_CHARGING_SAFETY_TIMER
	POWER_SUPPLY_PROP_SAFETY_TIMER,
#endif

#ifdef CONFIG_LGE_PM_USB_CURRENT_MAX
	POWER_SUPPLY_PROP_USB_CURRENT_MAX,
#endif

#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
	POWER_SUPPLY_PROP_HW_REV,
#endif
};

struct input_ma_limit_entry {
	int  icl_ma;
	u8  value;
};

static struct input_ma_limit_entry icl_ma_table[] = {
	{100, 0x00},
	{150, 0x01},
	{500, 0x02},
	{900, 0x03},
	{1500, 0x04},
	{1950, 0x05},
	{2500, 0x06},
	{2000, 0x07},
};

#ifndef CONFIG_LGE_PM_IRQ_NOT_WORKGROUND
static int get_prop_hw_rev(void)
{
	return lge_get_board_revno();
}
#endif
#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
static char *get_prop_hw_rev_name(void)
{
	char *name;
	name = lge_get_board_rev();
	return name;
}
#endif
#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
static int get_prop_batt_id_valid(void)
{
	return (int)is_lge_battery_valid();
}
#endif

#define CHARGER_TYPE_USB 1
#define CHARGER_TYPE_AC  1
int bq24262_set_usb_power_supply_type(enum power_supply_type type)
{
	struct bq24262_chip *chip = the_chip;
	int ret_chg = 0;
	int ret_limit = 0;

	pr_info("%s : %d\n", __func__,type);
	if(type == POWER_SUPPLY_TYPE_USB_CDP || type == POWER_SUPPLY_TYPE_USB
			|| type == POWER_SUPPLY_TYPE_USB_ACA) {
		power_supply_set_online(chip->usb_psy, CHARGER_TYPE_USB);
	} else if(type == POWER_SUPPLY_TYPE_USB_DCP) {
		power_supply_set_online(&(chip->ac_psy), CHARGER_TYPE_AC);
	}
	charger_type_check = type;
	pr_err("charger_type_check : %d\n", charger_type_check);

	bq24262_charging_setting(chip);

	ret_limit = bq24262_set_input_i_limit(chip, chip->cur_limit_ma);
	ret_chg = bq24262_set_ibat_max(chip, chip->chg_current_ma);
	if (ret_limit || ret_chg) {
		pr_err("Failed to set chg & limit current ret_chg : %d, ret_limit : %d\n",
				ret_limit, ret_chg);
		return ret_limit | ret_chg;
	}

#ifdef CONFIG_LGE_PM
	if (type < POWER_SUPPLY_TYPE_USB)
#else
	if (type < POWER_SUPPLY_TYPE_USB && type > POWER_SUPPLY_TYPE_BATTERY)
#endif
		return -EINVAL;

	chip->usb_psy->type= type;
	power_supply_changed(chip->usb_psy);
	power_supply_changed(&chip->ac_psy);
	power_supply_changed(&chip->batt_psy);

	return 0;
}

static int bq24262_get_register(struct bq24262_chip *chip)
{
	int ret;
	u8 sys_status = 0;

//	pr_info("=========================================================================\n");
	ret = bq24262_read_reg(chip->client, 0x00, &sys_status);
	if (ret) {
		pr_err("0x00 fail to read R00_STATUS_CONTROL_REG. ret=%d\n", ret);
	}
	pr_info(" 0x00 %x\n",sys_status);

	ret = bq24262_read_reg(chip->client, 0x01, &sys_status);
	if (ret) {
		pr_err("0x01 fail to read R01_STATUS_CONTROL_REG. ret=%d\n", ret);
	}
	pr_info(" 0x01 %x\n",sys_status);

	ret = bq24262_read_reg(chip->client, 0x02, &sys_status);
	if (ret) {
		pr_err("0x02 fail to read R02_STATUS_CONTROL_REG. ret=%d\n", ret);
	}
	pr_info(" 0x02 %x\n",sys_status);


	ret = bq24262_read_reg(chip->client, 0x03, &sys_status);
	if (ret) {
		pr_err("0x03 fail to read R03_STATUS_CONTROL_REG. ret=%d\n", ret);
	}
	pr_info(" 0x03 %x\n",sys_status);


	ret = bq24262_read_reg(chip->client, 0x04, &sys_status);
	if (ret) {
		pr_err("0x04 fail to read R04_STATUS_CONTROL_REG. ret=%d\n", ret);
	}
	pr_info(" 0x04 %x\n",sys_status);


	ret = bq24262_read_reg(chip->client, 0x05, &sys_status);
	if (ret) {
		pr_err("0x05 fail to read R05_STATUS_CONTROL_REG. ret=%d\n", ret);
	}
	pr_info(" 0x05 %x\n",sys_status);


	ret = bq24262_read_reg(chip->client, 0x06, &sys_status);
	if (ret) {
		pr_err("0x06 fail to read R06_STATUS_CONTROL_REG. ret=%d\n", ret);
	}
	pr_info(" 0x06 %x\n",sys_status);
//	pr_info("=========================================================================\n");


	return ret;
}

#define DEFAULT_TEMP		250
#ifdef CONFIG_SENSORS_QPNP_ADC_VOLTAGE
int bq24262_get_prop_batt_temp_raw(void)
{
	int rc = 0;
	static int last_temp;
	struct qpnp_vadc_result results;

	rc = qpnp_vadc_read(the_chip->vadc_dev,	LR_MUX1_BATT_THERM, &results);
	if (rc) {
		pr_err("Failed to read batt temperature. rc = %d\n", rc);
		return last_temp;
	}

	last_temp = (int) results.physical;
	return (int)results.physical;
}
#endif

static int bq24262_get_prop_batt_temp(struct bq24262_chip *chip)
{
#ifdef CONFIG_SENSORS_QPNP_ADC_VOLTAGE
	int rc = 0;
	struct qpnp_vadc_result results;

#ifdef CONFIG_LGE_PM_PSEUDO_BATTERY
	if (pseudo_batt_info.mode) {
		pr_debug("battery fake mode : %d \n", pseudo_batt_info.mode);
		return pseudo_batt_info.temp * 10;
	} else if (is_factory_cable() && chip->invalid_temp) {
		pr_err("factory cable : %d \n", DEFAULT_TEMP / 10);
		return DEFAULT_TEMP;
	}
#endif
	rc = qpnp_vadc_read(chip->vadc_dev,	LR_MUX1_BATT_THERM, &results);
	if (rc) {
		pr_err("Failed to read batt temperature. rc = %d\n", rc);

		if (previous_batt_temp != 600 && previous_batt_temp != -300) {
			pr_debug("Reported last_temp : %d \n", previous_batt_temp);
			return previous_batt_temp;
		} else {
			pr_err("Not exsist previous_batt_temp \n");
			return DEFAULT_TEMP;
		}
	} else {
		pr_debug("Get batt_temp_adc = %d, batt_temp : %lld\n",
				results.adc_code, results.physical);
		previous_batt_temp = (int)results.physical;

		return (int)results.physical;
	}
#else
	return DEFAULT_TEMP;
#endif
}

static int bq24262_get_prop_batt_health(struct bq24262_chip *chip)
{
#ifndef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
	int batt_temp = 0;
#endif
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
	if (chip->btm_state == BTM_HEALTH_OVERHEAT)
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	if (chip->btm_state == BTM_HEALTH_COLD)
		return POWER_SUPPLY_HEALTH_COLD;
	else
		return POWER_SUPPLY_HEALTH_GOOD;
#else
	batt_temp = bq24262_get_prop_batt_temp(chip);

	if (batt_temp >= 550)
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	if (batt_temp <= -100)
		return POWER_SUPPLY_HEALTH_COLD;
	else
		return POWER_SUPPLY_HEALTH_GOOD;
#endif
}

#define DEFAULT_VOLTAGE		4000000
static int bq24262_get_prop_batt_voltage_now(struct bq24262_chip *chip)
{
#ifdef CONFIG_LGE_PM_BATTERY_EXTERNAL_FUELGAUGE
	int voltage_now = 0;
	union power_supply_propval ret = {0,};
#elif defined (CONFIG_SENSORS_QPNP_ADC_VOLTAGE)
	int rc = 0;
	struct qpnp_vadc_result results;
#endif

#ifdef CONFIG_LGE_PM_PSEUDO_BATTERY
	if (pseudo_batt_info.mode) {
		pr_debug("Set pseudo battery mode\n");
		return DEFAULT_VOLTAGE;
	}
#endif
	if(is_factory_cable() && chip->invalid_temp) {
		pr_err("set factory cable volatge\n");
		return FACTORY_CABLE_VOLTAGE;
	}
#ifdef CONFIG_LGE_PM_BATTERY_EXTERNAL_FUELGAUGE
	if (chip->fuelgauge == NULL)
		chip->fuelgauge = power_supply_get_by_name("fuelgauge");

	if (chip->fuelgauge != NULL) {
		chip->fuelgauge->get_property(chip->fuelgauge, POWER_SUPPLY_PROP_VOLTAGE_NOW, &ret);
		voltage_now = ret.intval * 1000;
	}

	if (voltage_now == 0)
		return DEFAULT_VOLTAGE;
	else
		return voltage_now;
#elif defined (CONFIG_SENSORS_QPNP_ADC_VOLTAGE)
	rc = qpnp_vadc_read(chip->vadc_dev, VBAT_SNS, &results);
	if (rc) {
		pr_err("Failed to read batt temperature. rc = %d\n", rc);

		return DEFAULT_VOLTAGE;
	} else {
		pr_debug("Get batt_voltage = %lld\n",
				results.physical);

		return (int)results.physical;
	}
#else
	printk("No external chargin, failed to read qpnp sensor \n");
	return DEFAULT_VOLTAGE;
#endif
}

#define DEFAULT_CAPACITY	50
#define COMPLETE_CAPACITY 	100
static int bq24262_get_prop_batt_capacity(struct bq24262_chip *chip) {
#ifdef CONFIG_LGE_PM_BATTERY_EXTERNAL_FUELGAUGE
	int capacity = 0;
	union power_supply_propval ret = {0,};
#endif

	if (is_factory_cable() && chip->invalid_temp) {
		pr_err("Set factory cable soc\n");
		return FACTORY_CABLE_CAPACITY;
	}

#ifdef CONFIG_LGE_PM_BATTERY_EXTERNAL_FUELGAUGE
	if (chip->fuelgauge == NULL)
		chip->fuelgauge = power_supply_get_by_name("fuelgauge");

	if (chip->fuelgauge != NULL) {
		chip->fuelgauge->get_property(chip->fuelgauge, POWER_SUPPLY_PROP_CAPACITY, &ret);

#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
		if(bq24262_start_chg_factory_testmode == true)
		{
			pr_err("factory_testmode charging enable\n");
			ret.intval = COMPLETE_CAPACITY;
		}
#endif
		capacity = ret.intval;
	} else {
		printk("%s : Failed to read externalfuel gauge\n", __func__);
		return DEFAULT_CAPACITY;
	}

	return capacity;
#else
	pr_err("CONFIG_BATTERY_EXTERNAL_FUELGAUGE is not defined.\n");
	return DEFAULT_CAPACITY;
#endif
}

static int bq24262_get_prop_batt_status(struct bq24262_chip *chip)
{
	int chg_type = bq24262_get_prop_charge_type(chip);
	int batt_present = chip->batt_present;
	int capacity = bq24262_get_prop_batt_capacity(chip);
	int batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
	static int previous_batt_status;
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
	struct charging_rsp res;
#endif

	if (chip->ac_present) {
		if ((capacity >= 100 && batt_present)) {
			batt_status = POWER_SUPPLY_STATUS_FULL;
		} else if ((chg_type == POWER_SUPPLY_CHARGE_TYPE_TRICKLE ||
					chg_type == POWER_SUPPLY_CHARGE_TYPE_FAST)
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
				&& (res.state != CHG_BATT_STPCHG_STATE)
#endif
				&& chip->chg_cable_status == BQ_CHG_STATUS_FAST_CHARGE
				&& capacity < 100) {
			batt_status = POWER_SUPPLY_STATUS_CHARGING;

			return batt_status;
		} else if((chg_type != POWER_SUPPLY_CHARGE_TYPE_TRICKLE ||
					chg_type != POWER_SUPPLY_CHARGE_TYPE_FAST)
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
				|| (res.state == CHG_BATT_STPCHG_STATE)
#endif
				) {
			batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;

			return batt_status;
		} else if ((chip->chg_cable_status == BQ_CHG_STATUS_EXCEPTION)
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
				&& (res.state != CHG_BATT_STPCHG_STATE)
#endif
				) {
			if (previous_batt_status == POWER_SUPPLY_STATUS_CHARGING)
				batt_status = POWER_SUPPLY_STATUS_CHARGING;
			else
				batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
	} else {
		batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
	}
	previous_batt_status = batt_status;
	pr_info("type = %d status = %d", chg_type, batt_status);

	return batt_status;
}

static int bq24262_batt_power_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
	char *buf;
#endif
	struct bq24262_chip *chip = container_of(psy,
			struct bq24262_chip, batt_psy);

	switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:
			val->intval = bq24262_get_prop_batt_status(chip);
			break;
		case POWER_SUPPLY_PROP_CHARGE_TYPE:
			val->intval = bq24262_get_prop_charge_type(chip);
			break;
		case POWER_SUPPLY_PROP_HEALTH:
			val->intval = bq24262_get_prop_batt_health(chip);
			break;
		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = chip->batt_present;
			break;
		case POWER_SUPPLY_PROP_TECHNOLOGY:
			val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
			val->intval = bq24262_get_vbat_max(chip);
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
			val->intval = 3400 * 1000;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
				val->intval = bq24262_get_prop_batt_voltage_now(chip);
			break;
		case POWER_SUPPLY_PROP_TEMP:
			val->intval = bq24262_get_prop_batt_temp(chip);
			break;
		case POWER_SUPPLY_PROP_CAPACITY:
			if (pseudo_batt_info.mode) {
				val->intval = pseudo_batt_info.capacity;
				break;
			}
			val->intval = bq24262_get_prop_batt_capacity(chip);
			break;
		case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
			val->intval = chip->cur_limit_ma;
			break;

		case POWER_SUPPLY_PROP_CURRENT_NOW:
			val->intval = 0;//bq24262_get_prop_batt_current_now(chip);
			break;

		case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
			val->intval = 2100;//bq24262_get_prop_batt_full_design(chip);
			break;

		case POWER_SUPPLY_PROP_CURRENT_MAX:
			val->intval = chip->chg_current_ma;
			break;

		case POWER_SUPPLY_PROP_CHARGING_ENABLED:
			val->intval = (bq24262_get_prop_charge_type(chip)
					!= POWER_SUPPLY_CHARGE_TYPE_NONE);
			break;

		case POWER_SUPPLY_PROP_EXT_PWR_CHECK:
			val->intval = lge_pm_get_cable_type();
			break;

#ifdef CONFIG_LGE_PM_PSEUDO_BATTERY
		case POWER_SUPPLY_PROP_PSEUDO_BATT:
			val->intval = pseudo_batt_info.mode;
			break;
#endif

#ifdef CONFIG_LGE_PM_USB_CURRENT_MAX
		case POWER_SUPPLY_PROP_USB_CURRENT_MAX:
			if(chip->usb_current_max_enabled)
				val->intval = 1;
			else
				val->intval = 0;
			break;
#endif

#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
		case POWER_SUPPLY_PROP_BATTERY_ID_CHECKER:
			val->intval = read_lge_battery_id();
			break;
		case POWER_SUPPLY_PROP_VALID_BATT:
			val->intval = get_prop_batt_id_valid();
			break;
#endif

#ifdef CONFIG_LGE_PM_CHARGING_SAFETY_TIMER
		case POWER_SUPPLY_PROP_SAFETY_TIMER:
			val->intval = chip->safety_time_enable;
			break;
#endif

#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
		case POWER_SUPPLY_PROP_HW_REV:
			buf = get_prop_hw_rev_name();
			val->strval = buf;
		break;
#endif

		default:
			return -EINVAL;
	}

	return 0;
}


static int bq24262_batt_power_set_property(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct bq24262_chip *chip = container_of(psy,
			struct bq24262_chip, batt_psy);

	switch (psp) {
		case POWER_SUPPLY_PROP_CHARGING_ENABLED:
			pr_debug("bq24262 set_propert change_enable = %d\n",val->intval);
			bq24262_enable_charging(chip, val->intval);
			break;
		case POWER_SUPPLY_PROP_PRESENT:
			chip->batt_present = (val->intval);
			pr_debug("BATTERY %s! \n", val->intval ? "Inserted" : "Removed");
			break;

#ifdef CONFIG_LGE_PM_CHARGING_SAFETY_TIMER
		case POWER_SUPPLY_PROP_SAFETY_TIMER:
			chip->safety_time_enable = val->intval;
			if (chip->safety_time_enable) {
				if (chip->chg_cable_status == BQ_CHG_STATUS_FAST_CHARGE
						&& !timer_pending(&safety_timer)
						&& !(chip->safety_chg_done)
						&& !(chip->check_eoc_complete)
						&& !(is_factory_cable())) {
					pr_err("Starting safety timer\n");
					bq24262_start_set_safety_timer();
				}
			} else {
				pr_err("Stop safety timer and after previous set tiemr\n");
			}
			break;
#endif

#ifdef CONFIG_LGE_PM_USB_CURRENT_MAX
		case POWER_SUPPLY_PROP_USB_CURRENT_MAX:
			if(val->intval){
				pr_info("usb_current_max on\n");
				chip->usb_current_max_enabled = true;
			}else{
				pr_info("usb_current_max off\n");
				chip->usb_current_max_enabled = false;
			}

			bq24262_set_ibat_max(chip, chip->chg_current_ma);
			bq24262_set_input_i_limit(chip, chip->cur_limit_ma);
			break;
#endif

		default:
			return -EINVAL;
	}
	pr_debug("%s\n",__func__);
	power_supply_changed(&chip->batt_psy);
	return 0;
}


static int bq24262_batt_power_property_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	switch (psp) {
		case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		case POWER_SUPPLY_PROP_PRESENT:
#ifdef CONFIG_LGE_PM_CHARGING_SAFETY_TIMER
		case POWER_SUPPLY_PROP_SAFETY_TIMER:
#endif
#ifdef CONFIG_LGE_PM_USB_CURRENT_MAX
		case POWER_SUPPLY_PROP_USB_CURRENT_MAX:
#endif
			return 1;
		default:
			break;
	}

	return 0;
}

#define PSEUDO_BATTERY_CHG_CURRENT 700
static void bq24262_charging_setting(struct bq24262_chip *chip)
{
	int set_counter = 0;
	int i = 0;
	int usb_in = chip->ac_present;
	pr_debug("%s \n",__func__);

	if(charger_type_check == POWER_SUPPLY_TYPE_USB_DCP) {
		chip->usb_online = 0;
	} else if (charger_type_check == POWER_SUPPLY_TYPE_USB_CDP
			|| charger_type_check == POWER_SUPPLY_TYPE_USB
			|| charger_type_check == POWER_SUPPLY_TYPE_USB_ACA) {
		chip->usb_online = 1;
	} else {
		chip->usb_online = 0;
	}

#ifdef CONFIG_LGE_PM_PSEUDO_BATTERY
	if (pseudo_batt_info.mode) {
		if(chip->usb_online && usb_in) {
			chip->cur_limit_ma = INPUT_CURRENT_LIMIT_900mA;
			chip->chg_current_ma = INPUT_CURRENT_LIMIT_500mA;
			pr_info("Pseudo battery in USB i_lim= %d chg_curr= %d \n",
					chip->cur_limit_ma, chip->chg_current_ma);
		}
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
		else if (chip->ac_online && usb_in && chip->thermal_engine_control) {
			chip->cur_limit_ma = INPUT_CURRENT_LIMIT_900mA;
			chip->chg_current_ma = PSEUDO_BATTERY_CHG_CURRENT;
			pr_info("Pseudo battery on thermal in TA i_lim= %d chg_curr= %d \n",
					chip->cur_limit_ma, chip->chg_current_ma);
		}
#endif
		return;
	}
#endif

	if (is_factory_cable()) {
		chip->cur_limit_ma = INPUT_CURRENT_LIMIT_2500mA;
		chip->chg_current_ma = INPUT_CURRENT_LIMIT_500mA;
		pr_info("Factory cable wtlim i_lim= %d chg_curr= %d \n",
				chip->cur_limit_ma, chip->chg_current_ma);
	} else if (chip->usb_online && usb_in) {
		chip->cur_limit_ma = INPUT_CURRENT_LIMIT_500mA;
		chip->chg_current_ma = lge_pm_get_usb_current();
		pr_info("USB_charging wtlim i_lim= %d chg_curr= %d \n",
				chip->cur_limit_ma, chip->chg_current_ma);
	} else if (chip->ac_online && usb_in) {
		chip->chg_current_ma = lge_pm_get_ta_current();
		chip->chg_current_init_ma = lge_pm_get_ta_current();
		pr_info("DC_charging chg_current_init_ma = %d \n",
				chip->chg_current_init_ma);
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
		if (chip->thermal_engine_control)
			pr_info("thermal_engine_control : true\n");
		else
			pr_info("thermal_engine_control : false\n");

		if (chip->reached_temp_level) {
			pr_err("reached_temp : %d\n", chip->reached_temp_level);
			chip->chg_current_ma = chip->otp_ibat_current;
			chip->cur_limit_ma = INPUT_CURRENT_LIMIT_500mA;
			pr_info("reached_temp charging : %d\n", chip->chg_current_ma);
			return;
		}

		/* 2nd plug-in TA on thermal control */
		if ((chip->thermal_engine_control == true) && (chip->reached_temp_level == false)) {
			chip->chg_current_ma = chip->chg_current_te;
			pr_info("thermal-engine control chg_current_ma=%d\n",
				chip->chg_current_ma);

			if (chip->chg_current_te == chip->chg_current_init_ma)
				chip->thermal_engine_control = false;
		}
#endif
		for (i = ARRAY_SIZE(icl_ma_table) - 1; i >= 0; i--) {
			if (chip->chg_current_ma >= icl_ma_table[i].icl_ma)
				break;

			set_counter = i;
			pr_debug("charging_setting #1-2 set_count : %d\n", set_counter);
		}
		chip->cur_limit_ma = icl_ma_table[set_counter].icl_ma;
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
		pr_info("DC_charging i_lim= %d chg_curr_ma= %d chg_curr_te= %d \n",
				chip->cur_limit_ma, chip->chg_current_ma, chip->chg_current_te);
#endif
	} else { /* default */
		chip->cur_limit_ma = INPUT_CURRENT_LIMIT_500mA;
		chip->chg_current_ma = INPUT_CURRENT_LIMIT_500mA;
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
		pr_info("Defualt i_lim= %d chg_curr_ma= %d chg_curr_te= %d \n",
				chip->cur_limit_ma, chip->chg_current_ma, chip->chg_current_te);
#endif
	}
}

static void bq24262_batt_external_power_changed(struct power_supply *psy)
{
	struct bq24262_chip *chip = container_of(psy,
			struct bq24262_chip, batt_psy);

	pr_debug("%s \n",__func__);
	power_supply_changed(&chip->batt_psy);
}

static int bq24262_power_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct bq24262_chip *chip = container_of(psy, struct bq24262_chip,
			ac_psy);
	switch (psp) {
		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = chip->ac_present;
			break;
		case POWER_SUPPLY_PROP_ONLINE:
//			val->intval = (bq24262_get_prop_charge_type(chip)
//					!= POWER_SUPPLY_CHARGE_TYPE_NONE);
			val->intval = chip->ac_online;
			break;
		case POWER_SUPPLY_PROP_CHARGING_ENABLED:
			val->intval = (bq24262_get_prop_charge_type(chip)
					!= POWER_SUPPLY_CHARGE_TYPE_NONE);
			break;
		case POWER_SUPPLY_PROP_CURRENT_MAX:
			val->intval = chip->chg_current_ma;
			break;
		case POWER_SUPPLY_PROP_CHARGE_TYPE:
			val->intval = bq24262_get_prop_charge_type(chip);
			break;
		case POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED:
			val->intval = bq24262_get_input_i_limit(chip);
			break;
		case POWER_SUPPLY_PROP_CHARGING_COMPLETE:
			if(bq24262_get_prop_batt_capacity(chip) == 100)
				val->intval = 1;
			else
				val->intval = 0;
			break;
#ifdef CONFIG_LGE_PM_CHARGING_SAFETY_TIMER
		case POWER_SUPPLY_PROP_SAFETY_CHARGER_TIMER:
			val->intval = chip->safety_time_enable;
			break;
#endif
		default:
			return -EINVAL;
	}
	return 0;
}

static int bq24262_power_set_property(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct bq24262_chip *chip = container_of(psy, struct bq24262_chip,
			ac_psy);
	switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
			chip->ac_online = val->intval;
			pr_debug("%s ac_online : %d, usb_online : %d\n",
					__func__, chip->ac_online, chip->usb_online);
			break;
		case POWER_SUPPLY_PROP_CHARGING_ENABLED:
			bq24262_enable_charging(chip, val->intval);
			power_supply_changed(&chip->batt_psy);
			break;
		case POWER_SUPPLY_PROP_CURRENT_MAX:
			if (chip->chg_current_ma != val->intval) {
#ifdef CONFIG_LGE_PM_CHARGING_TEST_FOR_ART
				setting_chg_current_testmode = true;
#endif
				bq24262_set_ibat_max(chip, val->intval);
			} else {
				bq24262_set_ibat_max(chip, chip->chg_current_ma);
#ifdef CONFIG_LGE_PM_CHARGING_TEST_FOR_ART
				setting_chg_current_testmode = false;
#endif
			}
			break;
		case POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED:
			if (chip->cur_limit_ma != val->intval) {
				bq24262_set_input_i_limit(chip, val->intval);
#ifdef CONFIG_LGE_PM_CHARGING_TEST_FOR_ART
				setting_chg_current_testmode = true;
#endif
			} else {
				bq24262_set_input_i_limit(chip, chip->cur_limit_ma);
#ifdef CONFIG_LGE_PM_CHARGING_TEST_FOR_ART
				setting_chg_current_testmode = false;
#endif
			}
			break;

#ifdef CONFIG_LGE_PM_CHARGING_SAFETY_TIMER
		case POWER_SUPPLY_PROP_SAFETY_CHARGER_TIMER:
			chip->safety_time_enable = val->intval;
			if (chip->safety_time_enable) {
				if (chip->chg_cable_status == BQ_CHG_STATUS_FAST_CHARGE
						&& !timer_pending(&safety_timer)
						&& !(chip->safety_chg_done)
						&& !(chip->check_eoc_complete)
						&& !(is_factory_cable())) {
					pr_err("Starting safety timer\n");
					bq24262_start_set_safety_timer();
				}
			} else {
				pr_err("Stop safety timer and after previous set tiemr\n");
			}
			break;
#endif

		default:
			return -EINVAL;
	}
	pr_debug("%s\n",__func__);
	power_supply_changed(&chip->ac_psy);
	return 0;
}

static int bq24262_power_property_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	switch (psp) {
		case POWER_SUPPLY_PROP_PRESENT:
		case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		case POWER_SUPPLY_PROP_CURRENT_MAX:
		case POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED:
#ifdef CONFIG_LGE_PM_CHARGING_SAFETY_TIMER
		case POWER_SUPPLY_PROP_SAFETY_CHARGER_TIMER:
#endif
		case POWER_SUPPLY_PROP_CHARGING_COMPLETE:
			return 1;
		default:
			break;
	}

	return 0;
}

#define LOW_SOC_HEARTBEAT_MS	 20000
#define MIN_LOW_SOC_HEARTBEAT_MS 3000
#define UPDATE_TIME_MS 60000
static void update_heartbeat(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct bq24262_chip *chip = container_of(dwork,
			struct bq24262_chip, update_heartbeat_work);

	if (bq24262_get_prop_batt_capacity(chip) <= 20){
		if (bq24262_get_prop_batt_capacity(chip) <= 1) {
			printk("%s : SOC update per 1sec\n",__func__);
			schedule_delayed_work(&chip->update_heartbeat_work,
					round_jiffies_relative(msecs_to_jiffies
						(MIN_LOW_SOC_HEARTBEAT_MS)));
		} else {
			printk("%s : SOC update per 20sec\n",__func__);
			schedule_delayed_work(&chip->update_heartbeat_work,
					round_jiffies_relative(msecs_to_jiffies
						(LOW_SOC_HEARTBEAT_MS)));
		}
	} else {
		printk("%s : SOC update per 60sec\n",__func__);
		schedule_delayed_work(&chip->update_heartbeat_work,
				round_jiffies_relative(msecs_to_jiffies
					(UPDATE_TIME_MS)));
	}
}


static ssize_t at_chg_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int r;
	bool b_chg_ok = false;
	int chg_type;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	chg_type = bq24262_get_prop_charge_type(the_chip);
	if (chg_type != POWER_SUPPLY_CHARGE_TYPE_NONE) {
		b_chg_ok = true;
		r = snprintf(buf, 3, "%d\n", b_chg_ok);
		pr_debug("[Diag] true ! buf = %s, charging=1\n", buf);
	} else {
		b_chg_ok = false;
		r = snprintf(buf, 3, "%d\n", b_chg_ok);
		pr_debug("[Diag] false ! buf = %s, charging=0\n", buf);
	}

	return r;
}

static ssize_t at_chg_status_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int ret = 0;

	if (!count) {
		pr_err("[Diag] count 0 error\n");
		return -EINVAL;
	}

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (strncmp(buf, "0", 1) == 0) {
		/* stop charging */
#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
		lge_power_test_flag_charger = 1;
		bq24262_stpchg_factory_testmode = true;
		bq24262_start_chg_factory_testmode = false;
#endif
		pr_debug("[Diag] stop charging start\n");
		ret = bq24262_enable_charging(the_chip, false);

	} else if (strncmp(buf, "1", 1) == 0) {
		/* start charging */
#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
		bq24262_stpchg_factory_testmode = false;
		bq24262_start_chg_factory_testmode = true;
#endif
		pr_debug("[Diag] start charging start\n");
		ret = bq24262_enable_charging(the_chip, true);
	}

	if (ret)
		return -EINVAL;

	return 1;
}

static ssize_t at_chg_complete_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int guage_level = 0;
	int r = 0;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	guage_level = bq24262_get_prop_batt_capacity(the_chip);

	if (guage_level == 100) {
		r = snprintf(buf, 3, "%d\n", 0);
		pr_debug("[Diag] buf = %s, gauge==100\n", buf);
	} else {
		r = snprintf(buf, 3, "%d\n", 1);
		pr_debug("[Diag] buf = %s, gauge<=100\n", buf);
	}

	return r;
}

static ssize_t at_chg_complete_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int ret = 0;

	if (!count) {
		pr_err("[Diag] count 0 error\n");
		return -EINVAL;
	}

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (strncmp(buf, "0", 1) == 0) {
		/* charging not complete */
		pr_debug("[Diag] charging not complete start\n");
#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
		bq24262_stpchg_factory_testmode = false;
		bq24262_start_chg_factory_testmode = true;
#endif
		ret = bq24262_enable_charging(the_chip, true);
	} else if (strncmp(buf, "1", 1) == 0) {
		/* charging complete */
		pr_debug("[Diag] charging complete start\n");
#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
		bq24262_stpchg_factory_testmode = true;
		bq24262_start_chg_factory_testmode = false;
#endif
		ret = bq24262_enable_charging(the_chip, false);
	}

	if (ret)
		return -EINVAL;

	return 1;
}

static ssize_t at_pmic_reset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int r = 0;
	bool pm_reset = true;

	msleep(3000); /* for waiting return values of testmode */

	machine_restart(NULL);

	r = snprintf(buf, 3, "%d\n", pm_reset);

	return r;
}
static ssize_t at_otg_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int otg_mode;
	int r = 0;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	otg_mode = bq24262_is_otg_mode(the_chip);
	if(otg_mode) {
		otg_mode = 1;
		r = snprintf(buf, 3, "%d\n", otg_mode);
		pr_debug("[Diag] true ! buf = %s, OTG Enabled\n", buf);
	} else {
		otg_mode = 0;
		r = snprintf(buf, 3, "%d\n", otg_mode);
		pr_debug("[Diag] false ! buf = %s, OTG Disabled\n", buf);
	}
	return r;
}

static ssize_t at_otg_status_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int ret = 0;

	if (!count) {
		pr_err("[Diag] count 0 error\n");
		return -EINVAL;
	}

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (strncmp(buf, "0", 1) == 0) {
		pr_debug("[Diag] OTG Disable start\n");
		if(bq24262_is_otg_mode(the_chip))
			ret = bq24262_enable_otg(the_chip, false);

	} else if (strncmp(buf, "1", 1) == 0) {
		pr_debug("[Diag] OTG Enable start\n");
		if(!bq24262_is_otg_mode(the_chip))
			ret = bq24262_enable_otg(the_chip, true);
	}

	if(ret)
		return -EINVAL;
	return 1;
}

static ssize_t at_current_limit_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	int r = 0;
	bool current_limit = true;

	r = sprintf(buf, "%d\n", current_limit);

	return r;
}

static DEVICE_ATTR(at_charge, 0644, at_chg_status_show, at_chg_status_store);
static DEVICE_ATTR(at_current1, 0444, at_current_limit_show, NULL);
static DEVICE_ATTR(at_chcomp1, 0644, at_chg_complete_show, at_chg_complete_store);
static DEVICE_ATTR(at_pmrst1, 0440, at_pmic_reset_show, NULL);
static DEVICE_ATTR(at_otg, 0644, at_otg_status_show, at_otg_status_store);

#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
static int bq24262_thermal_mitigation;
static int bq24262_set_thermal_chg_current_set(const char *val, struct kernel_param *kp)
{
	int ret;

	if (!the_chip->probe_success) {
		pr_info("probe is not success : thermal chg_current not set\n");
		return -EINVAL;
	}

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}

	if (is_factory_cable()) {
		pr_info("plugged factory cable\n");
		return 0;
	}
#ifdef CONFIG_LGE_THERMALE_CHG_CONTROL
	pr_info("thermal-engine set chg current to %d\n",
			bq24262_thermal_mitigation);

	if (bq24262_thermal_mitigation <= 0) {
		the_chip->chg_current_te = the_chip->chg_current_init_ma;
		pr_info("bq24262_thermal_mitigation=0 : current to %d(mA)\n",
			the_chip->chg_current_te);
	} else {
		the_chip->chg_current_te = bq24262_thermal_mitigation;
		pr_info("bq24262_thermal_mitigation set current to %d(mA)\n",
			the_chip->chg_current_te);
	}

#ifdef CONFIG_LGE_PM_PSEUDO_BATTERY
	if ((pseudo_batt_info.mode == 1)&&(!is_factory_cable())) {
		the_chip->chg_current_te = PSEUDO_BATTERY_CHG_CURRENT;
		pr_info("Battery Fake Mode on thermal_mitigation set current %d(mA)\n",
			the_chip->chg_current_te);
	}
#endif

	the_chip->thermal_engine_control = true;

	if (the_chip->thermal_engine_control)
		pr_info("thermal_engine_control : true\n");

	cancel_delayed_work_sync(&the_chip->battemp_work);
	schedule_delayed_work(&the_chip->battemp_work, HZ*1);
#else
	pr_err("thermal-engine chg current control not enabled\n");
#endif
	return 0;
}
module_param_call(bq24262_thermal_mitigation, bq24262_set_thermal_chg_current_set,
		param_get_uint, &bq24262_thermal_mitigation, 0644);
#endif
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
static int temp_before;
static void bq24262_monitor_batt_temp(struct work_struct *work)
{
	struct bq24262_chip *chip =
		container_of(work, struct bq24262_chip, battemp_work.work);
	struct charging_info req;
	struct charging_rsp res;
	bool is_changed = false;
	union power_supply_propval ret = {0,};

#ifdef CONFIG_LGE_PM_CHARGING_SAFETY_TIMER
	if (chip->safety_time_enable) {
		if(chip->safety_chg_done) {
			bq24262_enable_charging(chip, 0);
			pr_err("Enable safety timer charging disable\n");
		}
	}
#endif
	chip->batt_psy.get_property(&(chip->batt_psy),
			POWER_SUPPLY_PROP_TEMP, &ret);
	req.batt_temp = ret.intval / 10;

	/* caution!! Scale from mV to uV for lge_monitor_batt_temp() */
	chip->batt_psy.get_property(&(chip->batt_psy),
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &ret);
	req.batt_volt = ret.intval;

	chip->batt_psy.get_property(&(chip->batt_psy),
			POWER_SUPPLY_PROP_CURRENT_NOW, &ret);
	req.current_now = ret.intval;

#ifdef CONFIG_LGE_THERMALE_CHG_CONTROL
	bq24262_thermal_mitigation = chip->chg_current_ma;
	req.chg_current_ma = chip->chg_current_ma;
	req.chg_current_te = chip->chg_current_te;
	pr_info("thermal-engine set req.chg_current_ma = %d, req.chg_current_te = %d\n",
			req.chg_current_ma, req.chg_current_te);
	if (chip->chg_current_te == chip->chg_current_init_ma) {
		chip->thermal_engine_control = false;
		pr_info("thermal_engine_control : false\n");
	}
#endif
	if (chip->ac_online || chip->usb_online)
		req.is_charger_changed = true;
	else
		req.is_charger_changed = false;

	req.is_charger = chip->ac_present;

	lge_monitor_batt_temp(req, &res);

	if (((res.change_lvl != STS_CHE_NONE) && req.is_charger) ||
			(res.force_update == true)) {
		if (res.change_lvl == STS_CHE_NORMAL_TO_DECCUR &&
			(res.state == CHG_BATT_DECCUR_STATE &&
				res.dc_current != DC_CURRENT_DEF)) {
			chip->reached_temp_level = true;
			chip->otp_ibat_current = res.dc_current;
			pr_info("N->D charging current : %d\n", chip->otp_ibat_current);
		} else if (res.change_lvl == STS_CHE_NORMAL_TO_STPCHG ||
			res.state == CHG_BATT_STPCHG_STATE) {
			chip->reached_temp_level = true;
			wake_lock(&chip->lcs_wake_lock);
			bq24262_enable_charging(chip, !res.disable_chg);
			pr_info("N->S charge stop : %d\n", !res.disable_chg);
		} else if (res.change_lvl == STS_CHE_DECCUR_TO_NORMAL) {
			chip->reached_temp_level = false;
			chip->otp_ibat_current = res.dc_current;
			pr_info("D->N charging current : %d\n",  chip->otp_ibat_current);
		} else if (res.change_lvl == STS_CHE_DECCUR_TO_STPCHG) {
			chip->reached_temp_level = true;
			wake_lock(&chip->lcs_wake_lock);
			bq24262_enable_charging(chip, !res.disable_chg);
			pr_info("D->S charge stop : %d\n", !res.disable_chg);
		} else if (res.change_lvl == STS_CHE_STPCHG_TO_NORMAL) {
			chip->reached_temp_level = false;
			chip->otp_ibat_current = res.dc_current;
			bq24262_enable_charging(chip, !res.disable_chg);
			pr_info("S->N charge start : %d\n", !res.disable_chg);
			wake_unlock(&chip->lcs_wake_lock);
		} else if (res.change_lvl == STS_CHE_STPCHG_TO_DECCUR) {
			chip->reached_temp_level = true;
			bq24262_enable_charging(chip, !res.disable_chg);
			chip->otp_ibat_current = res.dc_current;
			pr_info("S->D charging current : %d\n",  chip->otp_ibat_current);
			wake_unlock(&chip->lcs_wake_lock);
		} else if (res.force_update == true && res.state == CHG_BATT_NORMAL_STATE &&
				res.dc_current != DC_CURRENT_DEF) {
			chip->reached_temp_level = false;
			chip->otp_ibat_current = res.dc_current;
		}
	}

	if (chip->reached_temp_level)
		pr_info("otp_ibat_current=%d\n", chip->otp_ibat_current);

#ifdef CONFIG_LGE_PM_CHARGING_TEST_FOR_ART
	if ((setting_chg_current_testmode) && (chip->usb_online))
		pr_err("Not chg_current by testmode : %d & usb_online : %d\n",
				setting_chg_current_testmode, chip->usb_online);
	else
#endif
	bq24262_charging_setting(chip);

	/* 1st plug-in TA on thermal control */
	if ((chip->thermal_engine_control == true) && (chip->reached_temp_level == false)) {
		pr_info("thermal_engine_control : true\n");
		chip->chg_current_ma = chip->chg_current_te;
		pr_info("thermal-engine control chg_current_ma=%d\n", chip->chg_current_ma);
	} else {
		pr_info("thermal_engine_control after charging_setting : false\n");
	}
#ifdef CONFIG_LGE_PM_CHARGING_TEST_FOR_ART
	if ((!setting_chg_current_testmode) && (!chip->usb_online)) {
		bq24262_set_ibat_max(chip, chip->chg_current_ma);
		bq24262_set_input_i_limit(chip, chip->cur_limit_ma);
	}
#else
	bq24262_set_ibat_max(chip, chip->chg_current_ma);
	bq24262_set_input_i_limit(chip, chip->cur_limit_ma);
#endif
	if (chip->pseudo_ui_chg ^ res.pseudo_chg_ui) {
		is_changed = true;
		chip->pseudo_ui_chg = res.pseudo_chg_ui;
	}

	if (chip->btm_state ^ res.btm_state) {
		is_changed = true;
		chip->btm_state = res.btm_state;
	}

	if (temp_before != req.batt_temp) {
		is_changed = true;
		temp_before = req.batt_temp;
	}

	if (is_changed == true)
		power_supply_changed(&chip->batt_psy);

	bq24262_get_register(chip);

	if(req.is_charger) {
		if ((res.state == CHG_BATT_NORMAL_STATE) && (req.batt_temp > 0))
			schedule_delayed_work(&chip->battemp_work,
					msecs_to_jiffies(MONITOR_BATTEMP_POLLING_PERIOD));
		else if (res.state == CHG_BATT_DECCUR_STATE || res.state == CHG_BATT_WARNIG_STATE)
			schedule_delayed_work(&chip->battemp_work,
					msecs_to_jiffies(MONITOR_BATTEMP_POLLING_PERIOD / 3));
		else if (res.state == CHG_BATT_STPCHG_STATE || req.batt_temp <= 0)
			schedule_delayed_work(&chip->battemp_work,
					msecs_to_jiffies(MONITOR_BATTEMP_POLLING_PERIOD / 6));
	} else {
		pr_info("Stop workground monitor_temp\n");
		if (wake_lock_active(&chip->lcs_wake_lock))
			wake_unlock(&chip->lcs_wake_lock);
	}
}
#endif

static int bq24262_create_debugfs_entries(struct bq24262_chip *chip)
{
	int i;

	chip->dent = debugfs_create_dir(BQ24262_NAME, NULL);
	if (IS_ERR(chip->dent)) {
		pr_err("bq24262 driver couldn't create debugfs dir\n");
		return -EFAULT;
	}

	for (i = 0 ; i < ARRAY_SIZE(bq24262_debug_regs) ; i++) {
		char *name = bq24262_debug_regs[i].name;
		u32 reg = bq24262_debug_regs[i].reg;
		struct dentry *file;

		file = debugfs_create_file(name, 0644, chip->dent,
				(void *) reg, &reg_fops);
		if (IS_ERR(file)) {
			pr_err("debugfs_create_file %s failed.\n", name);
			return -EFAULT;
		}
	}

	return 0;
}

#ifdef CONFIG_USED_CHARGER_RESET
#define CHARGER_RESET_REG 7
#define CHARGER_RESET BIT(7)
static void external_charger_reset(struct bq24262_chip *chip) {
	bool enable = true;
	int ret = 0;
	u8 reset_en = ((u8) enable << CHARGER_RESET_REG);
	u8 reset_dis = ((u8) !enable << CHARGER_RESET_REG);

	ret = bq24262_masked_write(chip->client, R01_CONTROL_REG,
			CHARGER_RESET, reset_en);
	if (ret) {
		pr_err("Failed to control defualts value %d \n", ret);
		return ;
	}

	ret = bq24262_masked_write(chip->client, R01_CONTROL_REG,
			CHARGER_RESET, reset_dis);
	if (ret) {
		pr_err("Failed to control defualts value %d \n", ret);
		return ;
	}
}
#endif

#ifdef CONFIG_LGE_PM_BQ2426X_USING_WATCHDOG
#define WATCHDOG_NOT_RESET 1
#define WATCHDOG_RESET_REG 7
static int bq24262_set_watchdog(struct bq24262_chip *chip) {
	int ret = 0;
	u8 watchdog_disable = ((u8) WATCHDOG_NOT_RESET << WATCHDOG_RESET_REG);

	pr_err("watchdog reg : %02x\n", watchdog_disable);
	ret = bq24262_masked_write(chip->client, R00_STATUS_CONTROL_REG,
			TMR_RST, watchdog_disable);
	if (ret) {
		pr_err("Failed to control defualts value\n");
		return ret;
	}

	return 0;
}

static int bq24262_read_watchdog(struct bq24262_chip *chip) {
	int ret = 0;
	u8 reg_val = 0;
	bool watchdog_reset_off;

	ret = bq24262_read_reg(chip->client, R00_STATUS_CONTROL_REG, &reg_val);
	if (ret) {
		pr_err("Failed to read watchdog reg\n");
		return ret;
	}
	pr_debug("watchdog reg : %02x\n", reg_val);
	reg_val &= TMR_RST;
	reg_val = reg_val >> WATCHDOG_RESET_REG;

	if (reg_val == 0x00)
		watchdog_reset_off = true;
	else
		watchdog_reset_off = false;

	return watchdog_reset_off;
}

static void bq24262_check_watchdog_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct bq24262_chip *chip =
		container_of(dwork, struct bq24262_chip, check_watchdog_work);
	int ret = 0;
	int delay = (100 * 100);  /* 10 sec */
	bool watchdog_status  = bq24262_read_watchdog(chip);

	if (!watchdog_status) {
		pr_err("Failed to read watchdog reset\n");
	}

	mutex_lock(&chip->setting_env);
	ret = bq24262_set_watchdog(chip);
	if (ret) {
		pr_err("Failed to set watchdog reset\n");
		delay = delay / 100;
	}
	bq24262_get_register(chip);
	mutex_unlock(&chip->setting_env);
	if (chip->ac_present)
		delay = delay * 3;   /* setting 30 sec with TA/USB */
	else
		delay = delay / 2;

		schedule_delayed_work(&chip->check_watchdog_work, round_jiffies_relative
				(msecs_to_jiffies(delay)));

}
#endif

#define OTG_ENABLE_SHIFT  6
static int bq24262_enable_otg(struct bq24262_chip *chip, bool enable)
{
	int ret;
	u8 val = ((u8)enable << OTG_ENABLE_SHIFT);

	pr_debug("otg enable = %d\n", enable);

	ret = bq24262_masked_write(chip->client, R00_STATUS_CONTROL_REG,
			BOOST_MODE_MASK, val);
	if (ret) {
		pr_err("failed to set CHG_CONFIG rc=%d\n", ret);
		return ret;
	}

	return 0;
}

static void bq24262_set_clear_reg(struct bq24262_chip *chip)
{
	bq24262_enable_charging(chip, true);
	bq24262_set_vbat_max(chip, chip->regulation_mV);
	wake_lock_timeout(&chip->uevent_wake_lock, HZ*1);
	pr_debug("wtlim i_lim= %d chg_curr= %d \n",
			chip->cur_limit_ma, chip->chg_current_ma);
	bq24262_charging_setting(chip);
	bq24262_set_input_i_limit(chip, chip->cur_limit_ma);
	bq24262_set_ibat_max(chip, chip->chg_current_ma);

	last_stop_charging = 0;
	power_supply_changed(&chip->batt_psy);
	pr_info("%s : reg clear !! all registers are changed\n",__func__);

	return;
}

static void bq24262_remove_set_reg(struct bq24262_chip *chip)
{
	bq24262_set_vbat_max(chip, chip->regulation_mV);
	wake_lock_timeout(&chip->uevent_wake_lock, HZ*1);
//	bq24262_charging_setting(chip);
	if (chip->batt_present)
		chip->cur_limit_ma = INPUT_CURRENT_LIMIT_500mA;

	chip->chg_current_ma = INPUT_CURRENT_LIMIT_500mA;
	pr_err("Remove set reg i_lim= %d chg_curr= %d \n",
			chip->cur_limit_ma, chip->chg_current_ma);
	last_stop_charging = 0;

	bq24262_set_input_i_limit(chip, chip->cur_limit_ma);
	bq24262_set_ibat_max(chip, chip->chg_current_ma);
	bq24262_enable_charging(chip, true);
	chip->ac_online = 0;
	chip->usb_online = 0;
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
	chip->reached_temp_level = false;
	cancel_delayed_work(&chip->battemp_work);
#endif
#ifdef CONFIG_LGE_PM_CHARGING_SAFETY_TIMER
	chip->safety_chg_done = false;
	chip->safety_counter = 0;
#endif
#ifdef CONFIG_LGE_PM_CHARGING_TEST_FOR_ART
	setting_chg_current_testmode = false;
#endif
	chip->invalid_temp = false;
	chip->check_eoc_complete = false;
	charger_type_check = POWER_SUPPLY_TYPE_UNKNOWN;
	cancel_delayed_work(&chip->dpm_detect_work);
	pr_info("%s : Cable removed!! all registers are changed\n",__func__);
	if(bq24262_get_hz_mode(chip))
		bq24262_set_hz_mode(chip, false);
	return;
}

#define SET_ILIMIT_SHIFT 4
static int bq24262_set_input_i_limit(struct bq24262_chip *chip, int ma)
{
	int i;
	u8 temp = 0;
	int ret = 0;

#ifdef CONFIG_LGE_PM_USB_CURRENT_MAX
	if(chip->usb_current_max_enabled){
		pr_info("USB Current Max: set input limit to %d(mA)\n", USB_CURRENT_MAX);
		ma = USB_CURRENT_MAX;
	}
#endif

	if (ma < INPUT_CURRENT_LIMIT_100mA)
		ma = INPUT_CURRENT_LIMIT_100mA;
	if (ma > INPUT_CURRENT_LIMIT_2500mA)
		ma = INPUT_CURRENT_LIMIT_2500mA;

	for (i = ARRAY_SIZE(icl_ma_table) - 1; i >= 0; i--) {
		if (i < 0) {
			pr_err("can't find %d in icl_ma_table. Use min.\n", ma);
			i = 0;
		}

		if (icl_ma_table[i].icl_ma == ma) {
			temp = icl_ma_table[i].value;
			break;
		} else if (icl_ma_table[i].icl_ma < ma) {
			temp = icl_ma_table[i+1].value;
			break;
		}
	}
	pr_debug("mA : %d, icl_ma_table : %d\n", ma, temp);
	temp = temp << SET_ILIMIT_SHIFT;
	ret = bq24262_masked_write(chip->client, R01_CONTROL_REG, IINLIM_MASK, temp);
	if (ret) {
		pr_err("Failed to set i_limit ret = %d\n", ret);
		return ret;
	}

	return 0;
}

static int bq24262_get_input_i_limit(struct bq24262_chip *chip)
{
	int i,ret;
	u8 sys_control;

	ret = bq24262_read_reg(chip->client, R01_CONTROL_REG, &sys_control);
	if (ret) {
		pr_err("failed to read R01_CONTROL_REG rc=%d\n", ret);
		return false;
	}

	if (!bq24262_is_charger_plugin())
		return 0;


	sys_control &= IINLIM_MASK;
	sys_control = sys_control >> SET_ILIMIT_SHIFT;
	sys_control &=~EN_STAT;

	for (i = ARRAY_SIZE(icl_ma_table) - 1; i >= 0; i--) {
		if (icl_ma_table[i].value == sys_control)
			break;
	}

	return icl_ma_table[i].icl_ma;
}

#define EN_CHG_TERM_BIT 2
static int bq24262_enable_charger_current_termination(struct bq24262_chip *chip, u8 bit_set)
{
	bit_set = bit_set << EN_CHG_TERM_BIT;
	return bq24262_masked_write(chip->client, R01_CONTROL_REG,
			EN_CHG_TERM_MASK, bit_set);
}

#define EN_CHG_INT_BIT 3
static int bq24262_enable_interrupt(struct bq24262_chip *chip, u8 int_set)
{
	int_set = int_set << EN_CHG_INT_BIT;
	return bq24262_masked_write(chip->client, R01_CONTROL_REG,
			EN_STAT, int_set);
}

#define CHG_ENABLE_SHIFT  1
static int bq24262_enable_charging(struct bq24262_chip *chip, bool enable)
{
	int ret;
	u8 val = (u8)(!enable << CHG_ENABLE_SHIFT);

	pr_debug(" wtlim charging_enable=%d\n", enable);

	ret = bq24262_masked_write(chip->client, R01_CONTROL_REG,
			CHG_CONFIG_MASK, val);
	if (ret){
		pr_err("failed to set CHG_CONFIG ret=%d\n", ret);
		return ret;
	}

	return 0;
}

static int bq24262_set_hz_mode(struct bq24262_chip *chip, bool enable)
{
	int ret;
	u8 val = (u8)(enable);

	pr_debug("enable=%d\n", enable);

	ret = bq24262_masked_write(chip->client, R01_CONTROL_REG,
			HZ_MODE, val);
	if (ret) {
		pr_err("failed to set CHG_CONFIG ret=%d\n", ret);
		return ret;
	}

	return 0;
}

#define HZ_MOCE_GET 1
static int bq24262_get_hz_mode(struct bq24262_chip *chip)
{
	int ret,value;
	u8 val;

	ret = bq24262_read_reg(chip->client, R01_CONTROL_REG, &val);
	if (ret) {
		pr_err("failed to set CHG_CONFIG ret=%d\n", ret);
		return ret;
	}
	value = (int)(val & HZ_MOCE_GET);
	pr_debug("hz mode = %d\n",value);
	return value;
}

#define VBAT_SET_SHIFT 2
#define VBAT_MAX_MV  4440
#define VBAT_MIN_MV  3500
#define VBAT_STEP_MV  20
static int bq24262_set_vbat_max(struct bq24262_chip *chip, int mv)
{
	u8 reg_val = 0;
	int set_vbat = 0;

	if (mv < VBAT_MIN_MV)
		mv = VBAT_MIN_MV;
	if (mv > VBAT_MAX_MV)
		mv = VBAT_MAX_MV;

	reg_val = (mv - VBAT_MIN_MV)/VBAT_STEP_MV;
	set_vbat = reg_val * VBAT_STEP_MV + VBAT_MIN_MV;
	reg_val = reg_val << VBAT_SET_SHIFT;

	pr_debug("req_vbat = %d set_vbat = %d reg_val = 0x%02x\n",
			mv, set_vbat, reg_val);

	return bq24262_masked_write(chip->client, R02_CONTROL_BAT_VOL_REG,
			VBREG_MASK, reg_val);
}

static int bq24262_get_vbat_max(struct bq24262_chip *chip) {
	u8 sys_status = 0;
	int ret = 0;
	int get_vbat = 0;

	ret = bq24262_read_reg(chip->client, R02_CONTROL_BAT_VOL_REG, &sys_status);
	if (ret) {
		pr_err("Failed to read vbat_max ret : %d\n", ret);
		return 0;
	}

	sys_status &= VBREG_MASK;
	sys_status = sys_status >> VBAT_SET_SHIFT;
	get_vbat = sys_status * VBAT_STEP_MV + VBAT_MIN_MV;
	pr_debug("Get vbat_mmax %d\n", get_vbat);

	return get_vbat * 1000;
}

#define IBAT_MAX_MA  3000
#define IBAT_MIN_MA  500
#define IBAT_STEP_MA  100
#define SET_IBAT_SHIFT 3
static int bq24262_set_ibat_max(struct bq24262_chip *chip, int ma)
{
	u8 reg_val = 0;
	int set_ibat = 0;
	int ret = 0;
	u8 check_ibat = 0;
	int read_reg_ibat = 0;
	int set_reg_ibat = 0;

#ifdef CONFIG_LGE_PM_USB_CURRENT_MAX
	if(chip->usb_current_max_enabled){
		pr_info("USB Current Max: set ibat to %d(mA)\n", USB_CURRENT_MAX);
		ma = USB_CURRENT_MAX;
	}
#endif

	if (ma < IBAT_MIN_MA)
		ma = IBAT_MIN_MA;
	if (ma > IBAT_MAX_MA)
		ma = IBAT_MAX_MA;

	ret = bq24262_read_reg(chip->client, R04_BAT_TERM_FAST_CHARGE_CUR_REG, &check_ibat);
	if (ret) {
		pr_err("Failed to read ibat_reg ret = %d\n", ret);
		return ret;
	}
	check_ibat &= ICHG_MASK;
	check_ibat = check_ibat >> SET_IBAT_SHIFT;
	read_reg_ibat = check_ibat * IBAT_STEP_MA + IBAT_MIN_MA;
	pr_debug("setting ibat(%d) and set ibat(%d)\n", ma, read_reg_ibat);

	if (read_reg_ibat != ma) {
		reg_val = (ma-IBAT_MIN_MA)/IBAT_STEP_MA;
		set_ibat = reg_val * IBAT_STEP_MA + IBAT_MIN_MA;
		reg_val = reg_val << SET_IBAT_SHIFT;
//		chip->set_chg_current_ma = set_ibat;
		pr_debug("Setting chg_mA = %d, set_ibat = %d, reg_val = 0x%02x\n",
				ma, set_ibat, reg_val);
		ret = bq24262_masked_write(chip->client, R04_BAT_TERM_FAST_CHARGE_CUR_REG,
				ICHG_MASK, reg_val);
		if (ret) {
			pr_err("Failed to set ibat ret = %d\n", ret);
			return ret;
		}
	}

	ret = bq24262_read_reg(chip->client, R04_BAT_TERM_FAST_CHARGE_CUR_REG, &check_ibat);
	if (ret) {
		pr_err("Failed to read ibat_reg ret = %d\n", ret);
		return ret;
	}
	check_ibat &= ICHG_MASK;
	check_ibat = check_ibat >> SET_IBAT_SHIFT;
	set_reg_ibat = check_ibat * IBAT_STEP_MA + IBAT_MIN_MA;

	pr_debug("Finally set ibat(%d) and set ibat(%d) are same\n", ma, set_reg_ibat);

	return 0;
}

#define ITERM_MIN_MA  50
#define ITERM_MAX_MA  400
#define ITERM_STEP_MA  50
static int bq24262_set_term_current(struct bq24262_chip *chip, int ma)
{
	u8 reg_val = 0;
	int set_ma = 0;

	if (ma < ITERM_MIN_MA)
		ma = ITERM_MIN_MA;
	if (ma > ITERM_MAX_MA)
		ma = ITERM_MAX_MA;

	reg_val = (ma-ITERM_MIN_MA)/ITERM_STEP_MA;
	set_ma = reg_val * ITERM_STEP_MA + ITERM_MIN_MA;
	pr_debug("req_i = %d set_i = %d reg_val = 0x%02x\n",
			ma, set_ma, reg_val);

	return bq24262_masked_write(chip->client, R04_BAT_TERM_FAST_CHARGE_CUR_REG,
			ITERM_MASK, reg_val);
}

#define VIN_LIMIT_MIN_MV	4200
#define VIN_LIMIT_MAX_MV	4788
#define VIN_LIMIT_STEP_MV	84
static int bq24262_set_input_vin_limit(struct bq24262_chip *chip, int mv)
{
	u8 reg_val = 0;
	int set_vin = 0;

	if (mv < VIN_LIMIT_MIN_MV)
		mv = VIN_LIMIT_MIN_MV;
	if (mv > VIN_LIMIT_MAX_MV)
		mv = VIN_LIMIT_MAX_MV;

	reg_val = (mv - VIN_LIMIT_MIN_MV)/VIN_LIMIT_STEP_MV;
	set_vin = reg_val * VIN_LIMIT_STEP_MV + VIN_LIMIT_MIN_MV;

	pr_debug("%s, req_vin = %d set_vin = %d reg_val = 0x%02x\n", __func__,
			mv, set_vin, reg_val);

	return bq24262_masked_write(chip->client, R05_VINDPM_VOL_DPPM_STAT_REG,
			VINDPM_MASK, reg_val);
}

#define SYSTEM_VOLTAGE_SHIFT 7
static int bq24262_check_system_voltage(struct bq24262_chip *chip) {
	int ret = 0;
	u8 sys_status = 0;

		ret = bq24262_read_reg(chip->client, R05_VINDPM_VOL_DPPM_STAT_REG,
				&sys_status);
		if (ret) {
			pr_err("Failed system voltage enable setting ret = %d\n", ret);
			return ret;
		}
	sys_status &= MINSYS_STATUS;
	sys_status = sys_status >> SYSTEM_VOLTAGE_SHIFT;

	return (int)sys_status;
}

static bool bq24262_is_otg_mode(struct bq24262_chip *chip)
{
	u8 temp = 0;
	int ret;

	ret = bq24262_read_reg(chip->client, R00_STATUS_CONTROL_REG, &temp);
	if (ret) {
		pr_err("failed to read R00_STATUS_CONTROL_REG rc=%d\n", ret);
		return false;
	}
	pr_debug("%s = %x\n",__func__,temp);
	return ((temp & BOOST_MODE_MASK) == BOOST_MODE_MASK) ? true : false;
}

static bool bq24262_is_charger_present(struct bq24262_chip *chip)
{
	int ret = 0;
	u8 sys_status, fault_status;
	bool power_ok;

	if (is_factory_cable() && chip->invalid_temp) {
		pr_info("DC is present(PIF);\n");
		return 1;
	}

	ret = bq24262_read_reg(chip->client, R00_STATUS_CONTROL_REG, &sys_status);
	if (ret) {
		pr_err("failed to read R00_STATUS_CONTROL_REG ret=%d\n", ret);
		return false;
	}

	fault_status = (sys_status & (CHRG_STAT_MASK | CHG_FAULT_MASK));

	if(fault_status == BQ_FAULT_LOW_SUPPLY) {
		power_ok = false;
		pr_err("DC is disconnect - Low power supply\n");
	} else if (fault_status == 0x32) {
		power_ok = false;
		pr_err("DC is disconnect - Exception\n");
	} else {
		power_ok = true;
		pr_err("DC is present.\n");
	}

	return power_ok;
}


static int bq24262_get_prop_charge_type(struct bq24262_chip *chip)
{
	int ret = 0;
	u8 sys_status = 0;
	enum bq24262_chg_status status;
	int chg_type = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	u8 after_sys_status = 0;
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
	struct charging_rsp res;
#endif

	if(is_factory_cable() && !chip->batt_present)
		return 1;

	ret = bq24262_read_reg(chip->client, R00_STATUS_CONTROL_REG, &sys_status);
	if (ret) {
		pr_err("fail to read R00_STATUS_CONTROL_REG. ret=%d\n", ret);
		chg_type = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
		goto exception_handling;
	}

	sys_status = (sys_status & CHRG_STAT_MASK) >> 4;
	pr_debug("charge type %d\n", sys_status);
	if(sys_status == 0x01) {
		chg_type = POWER_SUPPLY_CHARGE_TYPE_FAST;
		status = BQ_CHG_STATUS_FAST_CHARGE;
	} else if (sys_status == 0x02) {
		chg_type = POWER_SUPPLY_CHARGE_TYPE_FAST;
		status = BQ_CHG_STATUS_FULL;
	} else { /* Fault */
		if (after_sys_status == 0x01) {
			chg_type = POWER_SUPPLY_CHARGE_TYPE_FAST;
			status = BQ_CHG_STATUS_FAST_CHARGE;
		} else if (after_sys_status == 0x02) {
			chg_type = POWER_SUPPLY_CHARGE_TYPE_FAST;
			status = BQ_CHG_STATUS_FULL;
		} else {
			chg_type = POWER_SUPPLY_CHARGE_TYPE_NONE;
			status = BQ_CHG_STATUS_NONE;
		}
	}
	pr_debug("bq-chg-status (%d=%s).\n", status, bq24262_chg_status[status]);

	if (chip->chg_status != status) {
		if((status == BQ_CHG_STATUS_NONE)
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
			|| res.state == CHG_BATT_STPCHG_STATE
#endif
			|| (status == BQ_CHG_STATUS_FULL)) {
			wake_unlock(&chip->chg_wake_lock);
			pr_info("Charging stopped chg_wakelock : %d\n",
					wake_lock_active(&chip->chg_wake_lock));
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
			cancel_delayed_work_sync(&chip->battemp_work);
#endif
		} else {
			pr_debug("Charging started.\n");
			wake_lock(&chip->chg_wake_lock);
		}
		chip->chg_status = status;
	}
	after_sys_status = sys_status;

	return chg_type;

exception_handling:
//	chip->chg_status = BQ_CHG_STATUS_EXCEPTION;
	if (wake_lock_active(&chip->chg_wake_lock)) {
		pr_err("exception_handling : unlock chg_wake_lock.\n");
		wake_unlock(&chip->chg_wake_lock);
	}
	return chg_type;

}

#define TA_USB_INSERT_TOLERANCE   70
#define EOC_THRESHOLD 4280
#define CAPACITY_FULL	100
static void bq24262_eoc_soc_check(struct bq24262_chip *chip)
{
	int capacity = bq24262_get_prop_batt_capacity(chip);
	int voltage = bq24262_get_prop_batt_voltage_now(chip);

	if (chip->check_eoc_complete && chip->batt_present)
		voltage = voltage - TA_USB_INSERT_TOLERANCE;

	pr_debug("voltage : %d, soc : %d\n", voltage, capacity);
	if (capacity >= CAPACITY_FULL && voltage < EOC_THRESHOLD) {
		pr_info("Re-charging, start charging again\n");
		chip->check_eoc_complete = false;
		bq24262_enable_charging(chip,true);
	} else if (capacity >= CAPACITY_FULL && voltage >= EOC_THRESHOLD) {
		pr_info("Charging Complete\n");
		chip->check_eoc_complete = true;
	} else {
		if (chip->chg_cable_status == BQ_CHG_STATUS_FULL) {
			chip->check_eoc_complete = true;
		} else {
			pr_info("Not charging complte and continuously to do\n");
			bq24262_enable_charging(chip, true);
			chip->check_eoc_complete = false;
		}
	}

	return;
}

#ifdef CONFIG_LGE_PM_BQ2426X_USING_WATCHDOG
static int bq24262_setting_environment(struct bq24262_chip *chip) {
	int ret = 0;
	u8 reg02_val = 0;
	u8 reg04_val = 0;
	u8 reg05_val = 0;

	ret = bq24262_read_reg(chip->client, 0x02, &reg02_val);
	if (ret) {
		pr_err("0x02 fail to read REG. ret=%d\n", ret);
	}
	reg02_val &= VBREG_MASK;
	reg02_val = reg02_val >> 2;
	pr_err("reg02_val = 0x%02x\n", reg02_val);

	if (reg02_val != 0x2d) {
		pr_err("Happened watchdog reset and reg02 cleared\n");
		ret = bq24262_set_vbat_max(chip, chip->regulation_mV);
		if (ret) {
			pr_err("failed to set vbat max\n");
			return ret;
		}
	}

	ret = bq24262_read_reg(chip->client, 0x04, &reg04_val);
	if (ret) {
		pr_err("0x04 fail to read REG. ret=%d\n", ret);
	}

	reg04_val &= ITERM_MASK;
	pr_err("reg04_val = 0x%02x\n", reg04_val);

	if (reg04_val != 0x02) {
		pr_err("Happened watchdog reset and reg04 cleared\n");
		ret = bq24262_set_term_current(chip, chip->term_current_ma);
		if (ret) {
			pr_err("failed to set charge termination current\n");
			return ret;
		}
	}

	ret = bq24262_read_reg(chip->client, 0x05, &reg05_val);
	if (ret) {
		pr_err("0x05 fail to read REG. ret=%d\n", ret);
	}

	reg05_val &= VINDPM_MASK;
	pr_err("reg05_val = 0x%02x\n", reg05_val);

	if (reg05_val != 0x04) {
		pr_err("Happened watchdog reset and reg05 cleared\n");
		ret = bq24262_set_input_vin_limit(chip, chip->vin_limit_mv);
		if (ret) {
			pr_err("failed to set input voltage limit\n");
			return ret;
		}
	}

	return 0;
}
#endif

#ifndef CONFIG_LGE_PM_IRQ_NOT_WORKGROUND
static void bq24262_irq_worker_rev_a(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct bq24262_chip *chip =
				container_of(dwork, struct bq24262_chip, irq_work);

	u8 reg_val = 0;
	int ret = 0;
	int cable_present = bq24262_is_charger_present(chip);
	int ret_chg = 0;
	int ret_limit = 0;
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
	struct charging_rsp res;
#endif
	static int check_time;

	if (cable_present) {
		bq24262_enable_charging(chip, true);
	}
	check_time++;
#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
	if(!cable_present)
		lge_power_test_flag_charger = 0;
#endif

	ret = bq24262_read_reg(chip->client, R00_STATUS_CONTROL_REG, &reg_val);
	if (ret) {
		pr_err("failed to read R00_STATUS_CONTROL_REG. val=%d\n", ret);
		return;
	}
	pr_debug("%s R00_STATUS_CONTROL_REG: 0x%2x\n", __func__,reg_val);

	bq24262_get_state(chip, reg_val);
	pr_err("%s BQcharger State [%s]\n",__func__,chg_state_str[chip->chg_cable_status]);
	if (check_time < 2)
		bq24262_get_register(chip);

	/* If register 00 was 0x20, The charging state done. It is mean EOC */
	if(chip->chg_cable_status == BQ_CHG_STATUS_FAST_CHARGE
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
			&& res.state != CHG_BATT_STPCHG_STATE
#endif
			&& chip->fault_state == BQ_FAULT_NORMAL) {
		pr_err("Charging in progress\n");
		bq24262_enable_charging(chip, true);
	} else if (chip->chg_cable_status == BQ_CHG_STATUS_FULL
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
			&& res.state != CHG_BATT_STPCHG_STATE
#endif
			&& chip->fault_state == BQ_FAULT_NORMAL){
		pr_info("Not charging - charging complete \n");
		bq24262_eoc_soc_check(chip);
		msleep(200);
	} else if ((chip->chg_cable_status == BQ_CHG_STATUS_EXCEPTION
				&& cable_present)
			&& chip->fault_state == BQ_FAULT_NORMAL) {
		pr_err("Exception but usb connected \n");
		bq24262_set_clear_reg(chip);
		msleep(200);
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
	} else if (res.state == CHG_BATT_STPCHG_STATE) {
			bq24262_enable_charging(chip, !res.disable_chg);
			pr_err("charge set : %d\n", !res.disable_chg);
#endif
	} else {
#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
		if(cable_present)
			goto IRQ_PASS;
#endif
		bq24262_remove_set_reg(chip);
	}
	pr_err("ac_present = %d cable_present = %d\n",
			chip->ac_present, cable_present);

	if (chip->ac_present ^ cable_present) {
		bq24262_notify_usb_of_the_plugin_event(cable_present);
		chip->ac_present = cable_present;
		power_supply_set_present(chip->usb_psy, chip->ac_present);

		wake_lock_timeout(&chip->uevent_wake_lock, HZ*2);
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
		if (!cable_present &&
				wake_lock_active(&chip->lcs_wake_lock))
			wake_unlock(&chip->lcs_wake_lock);
#endif
		pr_err("check_irq set_power_supply\n");
	}

	bq24262_charging_setting(chip);
	ret_limit = bq24262_set_input_i_limit(chip, chip->cur_limit_ma);
	ret_chg = bq24262_set_ibat_max(chip, chip->chg_current_ma);
	if (ret_limit || ret_chg) {
		pr_err("Failed to set chg & limit current ret_chg : %d, ret_limit : %d\n",
				ret_limit, ret_chg);
		return;
	}
	power_supply_changed(&chip->batt_psy);
	schedule_delayed_work(&chip->irq_work,
			round_jiffies_relative(msecs_to_jiffies(IRQ_CHECK_PERIOD)));
	return;

IRQ_PASS:
	pr_err("%s : IRQ_PASS \n",__func__);

	if(!cable_present){
		power_supply_set_online(chip->usb_psy, cable_present);
		power_supply_set_online(&(chip->ac_psy), cable_present);
		charger_type_check = POWER_SUPPLY_TYPE_UNKNOWN;
		bq24262_remove_set_reg(chip);
		cancel_delayed_work_sync(&chip->dpm_detect_work);
		power_supply_set_present(chip->usb_psy, chip->ac_present);
		pr_err("%s : cable_preset %d \n", __func__, cable_present);
	}

	chip->ac_present = cable_present;

	schedule_delayed_work(&chip->irq_work,
			round_jiffies_relative(msecs_to_jiffies(IRQ_CHECK_PERIOD)));
	return;
}
#endif

static void bq24262_irq_worker(struct work_struct *work)
{
	struct bq24262_chip *chip =
		container_of(work, struct bq24262_chip, irq_work.work);

	u8 reg_val = 0;
	int ret = 0;
	int cable_present = bq24262_is_charger_present(chip);
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
	struct charging_rsp res;
#endif

#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
	if(!cable_present)
		lge_power_test_flag_charger = 0;
#endif

#ifdef CONFIG_LGE_PM_BQ2426X_USING_WATCHDOG
	if (cable_present) {
		mutex_lock(&chip->setting_env);
		ret = bq24262_set_watchdog(chip);
		if (ret) {
			pr_err("Failed setting watchdog reset\n");
		}

		ret = bq24262_setting_environment(chip);
		if (ret) {
			pr_err("Failed setting environment\n");
		}
		mutex_unlock(&chip->setting_env);
	}
	pr_err("Set watchdog and enviroment value\n");
#endif

	ret = bq24262_read_reg(chip->client, R00_STATUS_CONTROL_REG, &reg_val);
	if (ret) {
		pr_err("failed to read R00_STATUS_CONTROL_REG. val=%d\n", ret);
		return;
	}
	pr_debug("%s R00_STATUS_CONTROL_REG: 0x%2x\n", __func__,reg_val);

	bq24262_get_state(chip, reg_val);
//	chip->chg_status = chip->chg_cable_status;
	pr_info("%s BQcharger State [%s]\n",__func__,chg_state_str[chip->chg_cable_status]);
//	bq24262_get_register(chip);

 	chip->batt_present = bq24262_get_prop_batt_present(chip);
	/* If register 00 was 0x20, The charging state done. It is mean EOC */
	if(chip->chg_cable_status == BQ_CHG_STATUS_FAST_CHARGE
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
			&& res.state != CHG_BATT_STPCHG_STATE
#endif
			&& chip->fault_state == BQ_FAULT_NORMAL) {
		chip->check_eoc_complete = false;
		pr_info("Charging in progress\n");
	} else if (chip->chg_cable_status == BQ_CHG_STATUS_FULL
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
			&& res.state != CHG_BATT_STPCHG_STATE
#endif
			&& chip->fault_state == BQ_FAULT_NORMAL){
		bq24262_eoc_soc_check(chip);
	} else if ((chip->chg_cable_status == BQ_CHG_STATUS_EXCEPTION
				&& cable_present)
			&& chip->fault_state == BQ_FAULT_NORMAL) {
		bq24262_set_clear_reg(chip);
	} else {
		msleep(200);
#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
	if((cable_present) && (chip->batt_present))
		goto IRQ_PASS;
#endif
		bq24262_remove_set_reg(chip);
	}
	pr_info("ac_present = %d cable_present = %d, ac xor cable = %d\n",
			chip->ac_present, cable_present, chip->ac_present^cable_present);
	if ((chip->ac_present ^ cable_present) || !chip->batt_present) {
		wake_lock_timeout(&chip->uevent_wake_lock, HZ*2);

#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
		if (!cable_present &&
				wake_lock_active(&chip->lcs_wake_lock))
			wake_unlock(&chip->lcs_wake_lock);
#endif
		bq24262_notify_usb_of_the_plugin_event(cable_present);
		chip->ac_present = cable_present;
		power_supply_set_present(chip->usb_psy, cable_present);
		power_supply_changed(&chip->batt_psy);
	}

IRQ_PASS:
	pr_info("%s : IRQ_PASS \n",__func__);
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
	if(chip->batt_present)
		schedule_delayed_work(&chip->battemp_work,
				round_jiffies_relative(msecs_to_jiffies(100)));
#endif

	if(!cable_present) {
		power_supply_set_online(chip->usb_psy, cable_present);
		power_supply_set_online(&(chip->ac_psy), cable_present);
	} else {
		schedule_delayed_work(&chip->dpm_detect_work, round_jiffies_relative
				(msecs_to_jiffies(100)));
#ifdef CONFIG_LGE_PM_CHARGING_SAFETY_TIMER
		pr_info("enable : %d, chg_done : %d, eoc : %d\n", chip->safety_time_enable,
				chip->safety_chg_done, chip->check_eoc_complete);
		pr_info("timer_pending : %d, PIF : %d", timer_pending(&safety_timer),
				is_factory_cable());
		if (chip->safety_time_enable) {
			if (chip->chg_cable_status == BQ_CHG_STATUS_FAST_CHARGE
				&& !timer_pending(&safety_timer) && !(chip->safety_chg_done)
					&& !(chip->check_eoc_complete) && !(is_factory_cable())) {
				bq24262_start_set_safety_timer();
				pr_info("Starting safety timer\n");
			}
		}
#endif
	}
	return;
}

static int bq24262_get_dpm_state(struct bq24262_chip * chip)
{
	int ret;
	u8 status;

	ret = bq24262_read_reg(chip->client, R05_VINDPM_VOL_DPPM_STAT_REG, &status);
	if (ret) {
		pr_err("failed to read R05_VINDPM_VOL_DPPM_STAT_REG. val=%d\n", ret);
		return ret;
	}
	status &= DPM_STATUS;
	status=status >>4;

	pr_debug("Reg05 0x%x val = 0x%x\n", R05_VINDPM_VOL_DPPM_STAT_REG, status);

	if(status == 0x04){
		return true;
	}
	else
		return false;
}

static void bq24262_dpm_detect_work(struct work_struct *work)
{
	struct bq24262_chip *chip =
		container_of(work, struct bq24262_chip, dpm_detect_work.work);

	int ret = 0;
	int dpm_val = bq24262_get_input_i_limit(chip);
	u8 status = 0;
	int i;
	int step_counter = 0;
	int new_set_chg_limit_current_ma = 0;
	bool dpm_enable = bq24262_get_dpm_state(chip);
	static int delay = 100;
	int get_chg_current = 0;
	static int before_get_current;
	static bool check_dpm_flag;

	if (!(chip->ac_present || is_factory_cable()) && chip->usb_online )
		return;

	ret = bq24262_read_reg(chip->client,  R04_BAT_TERM_FAST_CHARGE_CUR_REG, &status);
	if (ret) {
		pr_err("Failed to read ibat_reg ret = %d\n", ret);
		return;
	}
	status &= ICHG_MASK;
	status = status >> SET_IBAT_SHIFT;
	get_chg_current = status * IBAT_STEP_MA + IBAT_MIN_MA;

	if (dpm_enable) {
		pr_err("[DPM] Occured and decreasing chg_current : %d\n",
					get_chg_current);
		check_dpm_flag = true;
	} else {
		delay = 10 * 1000;
		if (check_dpm_flag) {
			pr_info("[DPM] Not detected and increasing chg_current : %d\n",
					get_chg_current);
			check_dpm_flag = false;
		}
	}
	if ((get_chg_current != before_get_current) && !chip->usb_online
			&& !chip->reached_temp_level) {
		for (i = ARRAY_SIZE(icl_ma_table); i >= 0; i--) {
			if (get_chg_current >= icl_ma_table[i].icl_ma)
				break;
			step_counter = i;
		}

		new_set_chg_limit_current_ma = icl_ma_table[step_counter].icl_ma;
		ret = bq24262_set_input_i_limit(chip, new_set_chg_limit_current_ma);
		if (ret < 0) {
			pr_err("Fail to set i_limit at dpm. ret = %d\n", ret);
			goto DPM_PASS;
		}
		pr_info("[DPM] %s, and limit current %d -> %d\n",
				dpm_enable ? "Happened" : "Not happened",
				dpm_val, new_set_chg_limit_current_ma);
	}
	before_get_current = get_chg_current;

DPM_PASS:
	if ((chip->ac_present) && (!chip->usb_online))
		schedule_delayed_work(&chip->dpm_detect_work, round_jiffies_relative
				(msecs_to_jiffies(delay)));
	else
		pr_info("Not set dpm work\n");
}

#ifdef CONFIG_LGE_PM_DEBUG_CHECK_LOG
#define CHG_CONFIG_FLAG 	BIT(1)
static void charging_infor_log_worker(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct bq24262_chip *chip = container_of(dwork, struct bq24262_chip,
			charging_inform_work);

	int ret = 0;	int bat_volt = 0; 	int bat_soc = 0; 	int charging_status = 0;
	u8 sys_status_1 = 0; 	u8 sys_status_2 = 0; 	int batt_temp = 0;
	int ibat_limit = 0; 	int ibat_chg = 0; 	int vbat_limit = 0; 	bool dpm_state = 0;
	int fault_status = 0; 	int chg_state = 0; 	int iterm = 0; 	int low_chg = 0;
	int batt_present = 0; 	int charger_present = 0;	int chg_done = 0;
	int charging_enable = 0;int vbat_min = 3400;	int delay = 0;
	bool chg_wakelock = false; bool lcs_wakelock = false; bool uevent_wakelock = false;

	ret = bq24262_read_reg(chip->client, 0x00, &sys_status_1);
	if (ret) {
		pr_err("0x00 fail to read R00_STATUS_CONTROL_REG. ret=%d\n", ret);
	}

	sys_status_2 = sys_status_1;
	sys_status_1 &= CHRG_STAT_MASK;
	charging_status = sys_status_1 >> 4;

	sys_status_2 = sys_status_2 & CHG_FAULT_MASK;
	fault_status = sys_status_2;

	if (charging_status != 0x03 && fault_status != 0x02)
		charger_present = 1;
	else
		charger_present = 0;

	if (charging_status == 0x02)
		chg_done = 1;
	else
		chg_done = 0;

	if (wake_lock_active(&chip->lcs_wake_lock))
		lcs_wakelock = true;
	else
		lcs_wakelock = false;

	if (wake_lock_active(&chip->chg_wake_lock))
		chg_wakelock = true;
	else
		chg_wakelock = false;

	if (wake_lock_active(&chip->uevent_wake_lock))
		uevent_wakelock = true;
	else
		uevent_wakelock = false;

	batt_present = chip->batt_present;
	bat_soc = bq24262_get_prop_batt_capacity(chip);
	bat_volt = bq24262_get_prop_batt_voltage_now(chip);

	if ((pseudo_batt_info.mode) ||
			(is_factory_cable() && !chip->invalid_temp)) {
		batt_temp = bq24262_get_prop_batt_temp_raw();
	} else {
		batt_temp = bq24262_get_prop_batt_temp(chip);
	}

	ibat_limit = bq24262_get_input_i_limit(chip);
	vbat_limit = bq24262_get_vbat_max(chip);
	dpm_state = bq24262_get_dpm_state(chip);

	ret = bq24262_read_reg(chip->client, 0x01, &sys_status_1);
	if (ret) {
		pr_err("0x01 fail to read R01_STATUS_CONTROL_REG. ret=%d\n", ret);
	}

	sys_status_1 &= CHG_CONFIG_FLAG;
	chg_state = sys_status_1 >> 1;

	if (charging_status == 0x01 && (!chg_state))
		charging_enable = 1;
	else
		charging_enable = 0;

	ret = bq24262_read_reg(chip->client, 0x04, &sys_status_1);
	if (ret) {
		pr_err("0x04 fail to read R04_STATUS_CONTROL_REG. ret=%d\n", ret);
	}

	sys_status_2 = sys_status_1;
	sys_status_1 &= ICHG_MASK;
	sys_status_1 = sys_status_1 >> SET_IBAT_SHIFT;
	ibat_chg = sys_status_1 * IBAT_STEP_MA + IBAT_MIN_MA;

	sys_status_2 &= ITERM_MASK;
	iterm = sys_status_2 * ITERM_STEP_MA + ITERM_MIN_MA;

	ret = bq24262_read_reg(chip->client, 0x05, &sys_status_1);
	if (ret) {
		pr_err("0x05 fail to read R05_STATUS_CONTROL_REG. ret=%d\n", ret);
	}

	sys_status_1 &= LOW_CHG;
	low_chg = sys_status_1 >> 5;

	pr_info("[DEBUG] charger_present : %s, usb_type : %s, dpm_state : %d\n",
			charger_present ? "Insert" : "Remove",
			chip->ac_online ? "AC" : chip->usb_online ? "USB" : "NONE", dpm_state);
	pr_info("[DEBUS] pseudo_batt %s, vbat_max(min) : %d mV(%d mV)\n",
			pseudo_batt_info.mode ? "Enable" : "Disable", vbat_limit/1000, vbat_min);
	pr_info("[DEBUG] chg_state : %s, ibat_limit : %d mA, ibat_chg : %d mA, iterm : %d mA\n",
			charging_enable ? "Enable" : "Disable", ibat_limit, ibat_chg, iterm);
	pr_info("[DEBUG] chg_done : %s, low_chg : %s\n", chg_done ? "Done" : "Not_yet",
			low_chg ? "Enable" : "Disable");
	pr_info("[DEBUG] batt_present : %s, batt_volt : %d mV, batt_soc : %d %%, batt_temp : %d\n",
			batt_present ? "Insert" : "Remove", bat_volt/1000, bat_soc, batt_temp);
	pr_info("[DEBUG] chg_wakelock : %s, lcs_wakelock : %s, uevent_wakelock : %s\n",
			chg_wakelock ? "True" : "False", lcs_wakelock ? "True" : "False",
			uevent_wakelock ? "True" : "False");

	if (charger_present) {
		if (chg_done)
			delay = DEBUG_POLLING_PERIOD;
		else
			delay = DEBUG_POLLING_PERIOD * 8;
	} else {
		delay = DEBUG_POLLING_PERIOD;
	}

	schedule_delayed_work(&chip->charging_inform_work,
					round_jiffies_relative(msecs_to_jiffies(delay)));
}
#endif

int bq24262_is_charger_plugin(void)
{
	int cable_present = 0;
	pr_debug("%s \n",__func__);

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	cable_present = the_chip->ac_present;
	pr_debug("%s %d \n",__func__,cable_present);

	return cable_present;
}

static int bq24262_get_prop_batt_present(struct bq24262_chip *chip)
{
	int temp = 0;
	bool batt_present;
	int ret = 0;
	u8 sys_status;

#ifdef CONFIG_LGE_PM_PSEUDO_BATTERY
	if (pseudo_batt_info.mode)
		return 1;
#endif

	ret = bq24262_read_reg(chip->client, R00_STATUS_CONTROL_REG, &sys_status);
	if (ret) {
		pr_err("failed to read R00_STATUS_CONTROL_REG ret=%d\n", ret);
		return false;
	}
	sys_status &= CHG_FAULT_MASK;

	if (is_factory_cable())
		temp = bq24262_get_prop_batt_temp_raw();
	else
		temp = bq24262_get_prop_batt_temp(chip);

	if ((temp <= -300 || temp >= 790) || (sys_status == 0x07)) {
		pr_err("\n\n  Battery missing(over temp : %d and charger : %02x)\n\n",
				temp, sys_status);
		batt_present = false;
		chip->invalid_temp = true;
	} else {
		batt_present = true;
		chip->invalid_temp = false;
	}

	if (is_factory_cable() && chip->invalid_temp) {
		pr_info("Connected PIF, and force setting\n");
		batt_present = 1;
	}

	pr_debug("present=%d, chip->batt_present = %d\n",
			batt_present ? 1 : 0, chip->batt_present);

	return batt_present;
}

static void bq24262_remove_debugfs_entries(struct bq24262_chip *chip)
{
	if (chip->dent)
		debugfs_remove_recursive(chip->dent);
}

static int __devinit bq24262_init_batt_psy(struct bq24262_chip *chip)
{
	int ret;

	chip->batt_psy.name = "battery";
	chip->batt_psy.type = POWER_SUPPLY_TYPE_BATTERY;
	chip->batt_psy.properties = bq24262_batt_power_props;
	chip->batt_psy.num_properties =
					ARRAY_SIZE(bq24262_batt_power_props);
	chip->batt_psy.get_property = bq24262_batt_power_get_property;
	chip->batt_psy.set_property = bq24262_batt_power_set_property;
	chip->batt_psy.property_is_writeable =
					bq24262_batt_power_property_is_writeable;
	chip->batt_psy.external_power_changed =
					bq24262_batt_external_power_changed;

	ret = power_supply_register(&chip->client->dev,
			&chip->batt_psy);

	if (ret) {
		pr_err("failed to register power_supply. ret=%d.\n", ret);
		return ret;
	}

	return 0;
}

static int __devinit bq24262_init_ac_psy(struct bq24262_chip *chip)
{
	int ret = 0;

	chip->ac_psy.name = "ac";
	chip->ac_psy.type = POWER_SUPPLY_TYPE_MAINS;
	chip->ac_psy.supplied_to = bq24262_power_supplied_to;
	chip->ac_psy.num_supplicants = ARRAY_SIZE(bq24262_power_supplied_to);
	chip->ac_psy.properties = bq24262_power_props;
	chip->ac_psy.num_properties = ARRAY_SIZE(bq24262_power_props);
	chip->ac_psy.get_property = bq24262_power_get_property;
	chip->ac_psy.set_property = bq24262_power_set_property;
	chip->ac_psy.property_is_writeable =
		bq24262_power_property_is_writeable;

	ret = power_supply_register(&chip->client->dev,
			&chip->ac_psy);
	if (ret) {
		pr_err("failed to register power_supply. ret=%d.\n", ret);
		return ret;
	}

	return 0;
}

#define EN_CHG_TERM_ENABLE	0x1
static int bq24262_hw_init(struct bq24262_chip *chip)
{
	int ret = 0;

	ret = bq24262_enable_otg(chip, false);
	if (ret) {
		pr_err("failed to set otg mode\n");
		return ret;
	}
	ret = bq24262_set_input_vin_limit(chip, chip->vin_limit_mv);
	if (ret) {
		pr_err("failed to set input voltage limit\n");
		return ret;
	}

	if(is_factory_cable_56k() || is_factory_cable_130k() || is_factory_cable_910k()) {
		ret = bq24262_set_input_i_limit(chip, INPUT_CURRENT_LIMIT_1500mA);
	} else {
		ret = bq24262_set_input_i_limit(chip, chip->cur_limit_ma);
	}
	pr_debug("chg_current : %d, limit_current : %d\n", chip->chg_current_ma, chip->cur_limit_ma);
	if (ret) {
		pr_err("Failed to set input current limit\n");
		return ret;
	}

#ifdef CONFIG_LGE_THERMALE_CHG_CONTROL
	bq24262_thermal_mitigation = chip->chg_current_ma;
	chip->chg_current_te = chip->chg_current_ma;
#endif

	chip->system_vol_status = bq24262_check_system_voltage(chip);
	if (ret) {
		pr_err("failed to set system_voltage\n");
		return ret;
	}

	/* regulation voltage : 4.4V */
	ret = bq24262_set_vbat_max(chip, chip->regulation_mV);
	if (ret) {
		pr_err("failed to set vbat max\n");
		return ret;
	}

	ret = bq24262_set_ibat_max(chip, chip->chg_current_ma);
	if (ret) {
		pr_err("failed to set charging current\n");
		return ret;
	}

	/* Enable Charge Current Termination*/
	ret = bq24262_enable_charger_current_termination(chip, EN_CHG_TERM_ENABLE);
	if (ret) {
		pr_err("failed to enable chg termination\n");
		return ret;
	}
	ret = bq24262_set_term_current(chip, chip->term_current_ma);
	if (ret) {
		pr_err("failed to set charge termination current\n");
		return ret;
	}

	/* Enable Interrupt*/
	ret = bq24262_enable_interrupt(chip, true);
	if (ret) {
		pr_err("failed to enable interrupt\n");
		return ret;
	}

	return 0;
}

static int bq24262_parse_dt(struct device_node *dev_node,
		struct bq24262_chip *chip) {
	int ret;

	chip->int_gpio = of_get_named_gpio(dev_node, "ti,int-gpio", 0);
	pr_info("int-gpio = %d\n", chip->int_gpio);

	if (chip->int_gpio < 0) {
		pr_err("Fail to get int-gpio. \n");
		ret = chip->int_gpio;
		goto out;
	}

	chip->stat_gpio = of_get_named_gpio(dev_node, "ti,stat-gpio", 0);
	pr_info("stat_gpio = %d\n", chip->stat_gpio);
	if (chip->stat_gpio < 0) {
		pr_err("failed to get stat_gpio\n");
		ret = chip->stat_gpio;
		goto out;
	}

	chip->ext_chg_disen = of_get_named_gpio(dev_node, "ti,ext-chg-disen-gpio", 0);
	pr_info("ext_chg_disen = %d\n", chip->ext_chg_disen);
	if (chip->ext_chg_disen < 0) {
		pr_err("failed to get ext_chg_disen\n");
		ret = chip->ext_chg_disen;
		goto out;
	}

	ret = of_property_read_u32(dev_node, "ti,term-current-ma",
			&(chip->term_current_ma));
	pr_info("bq24262 term_current_ma = %d.\n",
			chip->term_current_ma);
	if (ret) {
		pr_err("Unable to read term-current-ma.\n");
	}

	ret = of_property_read_u32(dev_node, "ti,vbat-max-mv",
			&chip->regulation_mV);
	pr_info("bq24262 vbat_max_mv = %d.\n",
			chip->regulation_mV);
	if (ret) {
		pr_err("Unable to read vbat-max-mv.\n");
	}

	ret = of_property_read_u32(dev_node, "ti,vin-limit-mv",
			&chip->vin_limit_mv);
	pr_info("bq24262 vin_limit_mv = %d.\n",
			chip->vin_limit_mv);
	if (ret) {
		pr_err("Unable to read vin-limit-mv.\n");
	}

#ifdef SET_CHG_LIMIT_CURRENT_USER
	ret = of_property_read_u32(dev_node, "ti,cur-limit-ma",
			&chip->cur_limit_ma);
	pr_info("bq24262 cur_limit_ma = %d.\n",
			chip->cur_limit_ma);
	if (ret) {
		pr_err("Unable to read cur-limit-ma.\n");
		goto out;
	}
#endif

#ifdef CONFIG_LGE_PM_CHARGING_SAFETY_TIMER
	ret = of_property_read_u32(dev_node, "ti,safety-time",
			&chip->safety_timeout);
	pr_info("bq24262 safety timeout = %d.\n",
			chip->safety_timeout);
	if (ret) {
		pr_err("Unable to read safety timeout\n");
	}
	if (chip->safety_timeout < 240) {
		chip->safety_time_enable = false;
		pr_err("Not supported safety timer(%s) under 4 hours\n",
				chip->safety_time_enable ? "enable" : "disable");
	} else {
		chip->safety_time_enable = true;
		pr_info("Setting safety timer(%s) : %d hours\n",
				chip->safety_time_enable ? "enable" : "disable",
				chip->safety_timeout/60);
	}
#endif

	pr_info("Read complete from dtsi file\n");
	return 0;
out:
	return ret;
}

static void cable_init_termination(struct bq24262_chip *chip)
{
//	chip->ac_present = bq24262_is_charger_present(chip);
	pr_debug("%s cable present = %d\n",__func__,chip->ac_present);
	bq24262_notify_usb_of_the_plugin_event(chip->ac_present);
	pr_debug("cable_init_termination = %d \n", true);
	bq24262_enable_charging(chip, true);
}


static irqreturn_t bq24262_irq(int irq, void *dev_id)
{
	struct bq24262_chip *chip = dev_id;
	pr_debug("%s\n",__func__);
#ifdef I2C_SUSPEND_WORKAROUND
	schedule_delayed_work(&chip->check_suspended_work,
		msecs_to_jiffies(100));
#else
#ifdef CONFIG_LGE_PM_IRQ_NOT_WORKGROUND
	schedule_delayed_work(&chip->irq_work, msecs_to_jiffies(0));
#else
	if (get_prop_hw_rev() > HW_REV_A)
		schedule_delayed_work(&chip->irq_work, msecs_to_jiffies(0));
#endif
#endif

	return IRQ_HANDLED;
}

#ifdef I2C_SUSPEND_WORKAROUND
static void bq24262_check_suspended_worker(struct work_struct *work)
{
	struct bq24262_chip *chip =
		container_of(work, struct bq24262_chip, check_suspended_work.work);

	if (chip->suspend) {
		pr_debug("bq24296 suspended. try i2c operation after 100ms.\n");
		schedule_delayed_work(&chip->check_suspended_work, msecs_to_jiffies(100));
    } else {
		pr_debug("bq24296 resumed. do bq24296_irq.\n");
#ifdef CONFIG_LGE_PM_IRQ_NOT_WORKGROUND
		schedule_delayed_work(&chip->irq_work, 0);
#else
		if (get_prop_hw_rev() > HW_REV_A)
			schedule_delayed_work(&chip->irq_work, 0);
#endif
	}
}
#endif //I2C_SUSPEND_WORKAROUND

static int bq24262_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct device_node *dev_node = client->dev.of_node;
	struct bq24262_chip *chip;
	int ret = 0;

	unsigned int *p_cable_type = (unsigned int *)
		(smem_get_entry(SMEM_ID_VENDOR1, &cable_smem_size, 0, 0));

	if (p_cable_type)
		cable_type = *p_cable_type;
	else
		cable_type = 0;

	pr_debug("cable_type is = %d\n", cable_type);

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_BYTE_DATA)) {
		pr_err("i2c func fail.\n");
		return -EIO;
	}

	chip = kzalloc(sizeof(struct bq24262_chip), GFP_KERNEL);
	if (!chip) {
		pr_err("failed to alloc memory\n");
		return -ENOMEM;
	}

	chip->probe_success = false;

	chip->client = client;
	chip->usb_psy = power_supply_get_by_name("usb");
	if (!chip->usb_psy) {
		pr_err("usb supply not found deferring probe\n");
		ret = -EPROBE_DEFER;
		goto error;
	}

	if(dev_node) {
		get_cable_data_from_dt(dev_node);

		ret = bq24262_parse_dt(dev_node, chip);
		if (ret) {
			pr_err("Failed to parse dt\n");
			goto error;
		}
		msleep(200);
		chip->vadc_dev = qpnp_get_vadc(&client->dev, "bq24262");
		if (IS_ERR(chip->vadc_dev)) {
			ret = PTR_ERR(chip->vadc_dev);
			if (ret != -EPROBE_DEFER)
				pr_err("vadc property missing\n");
			else
				pr_err("probe defer due to not initializing vadc\n");

			goto error;
		}
		lge_pm_read_cable_info(chip->vadc_dev);
	}

	the_chip = chip;

	if (bq24262_get_prop_batt_present(chip))
		chip->batt_present = true;
	else
		chip->batt_present = false;

	last_stop_charging = 0;
	previous_batt_temp = 250;
	charger_type_check = POWER_SUPPLY_TYPE_UNKNOWN;
	bq24262_charging_state = 0;
	lge_power_test_flag_charger = 0;

#ifdef CONFIG_LGE_PM_CHARGING_SAFETY_TIMER
	init_timer(&safety_timer);
#endif

	ret = gpio_request_one(chip->int_gpio, GPIOF_DIR_IN,
			"bq24262_int");
	if (ret) {
		pr_err("failed to request int_gpio ret = %d\n", ret);
		goto error;
	}
#ifdef CONFIG_LGE_PM_IRQ_NOT_WORKGROUND
	chip->irq = gpio_to_irq(chip->int_gpio);
	if (chip->irq < 0) {
		pr_err("irq value is not valid\n");
		gpio_free(chip->int_gpio);
		goto err_req_irq;
	}
	pr_debug("int_gpio irq#=%d.\n", chip->irq);
	ret = request_irq(chip->irq, bq24262_irq,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			"bq24262_irq", chip);
	if (ret) {
		pr_err("request_irq %d failed\n", chip->irq);
		goto err_req_irq;
	}

	ret = enable_irq_wake(chip->irq);
	if (ret < 0) {
		pr_err("Failed to set irq\n");
		goto err_req_irq;
	}
#else
	if (get_prop_hw_rev() > HW_REV_A) {
		chip->irq = gpio_to_irq(chip->int_gpio);
		if (chip->irq < 0) {
			pr_err("irq value is not valid\n");
			gpio_free(chip->int_gpio);
			goto err_req_irq;
		}
		pr_debug("int_gpio irq#=%d.\n", chip->irq);

		ret = request_irq(chip->irq, bq24262_irq,
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				"bq24262_irq", chip);
		if (ret) {
			pr_err("request_irq %d failed\n", chip->irq);
			goto err_req_irq;
		}

		ret = enable_irq_wake(chip->irq);
		if (ret < 0) {
			pr_err("Failed to set irq\n");
			goto err_req_irq;
		}
	}
#endif
	ret = gpio_request_one(chip->ext_chg_disen, GPIOF_DIR_OUT | GPIOF_INIT_LOW,
			"bq24262_ext_chg_disen");
	if (ret) {
		pr_err("failed to request ext_chg_disen ret = %d\n", ret);
		goto error;
	}

	ret = gpio_request_one(chip->stat_gpio, GPIOF_DIR_IN,
			"bq24262_ext_stat");
	if (ret) {
		pr_err("failed to request ext_stat ret = %d\n", ret);
		goto error;
	}

	i2c_set_clientdata(client, chip);

#ifdef CONFIG_LGE_PM_BQ2426X_USING_WATCHDOG
	ret = bq24262_set_watchdog(chip);
	if (ret) {
		pr_err("Failed to set watchdog reset\n");
		goto err_hw_init;
	}
#endif

	bq24262_charging_setting(chip);

#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
	chip->otp_ibat_current = chip->chg_current_ma;
	pr_info("jjb bq24262_probe : otp_ibat_current : %d, chg_current_ma : %d\n", chip->otp_ibat_current, chip->chg_current_ma);
	chip->reached_temp_level = false;
	chip->thermal_engine_control = false;
#endif

	ret = bq24262_hw_init(chip);
	if (ret) {
		pr_err("bq24262_hwinit failed.ret=%d\n",ret);
		goto err_hw_init;
	}

	wake_lock_init(&chip->chg_wake_lock,
			WAKE_LOCK_SUSPEND, BQ24262_NAME);
	wake_lock_init(&chip->uevent_wake_lock,
			WAKE_LOCK_SUSPEND, "bq24262_chg_uevent");
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
	wake_lock_init(&chip->lcs_wake_lock,
			WAKE_LOCK_SUSPEND, "LGE charging scenario");
#endif

	ret = bq24262_init_batt_psy(chip);
	if (ret) {
		pr_err("bq24262_init_batt_psy failed ret=%d\n", ret);
		goto err_init_batt_psy;
	}

	ret = bq24262_init_ac_psy(chip);
	if (ret) {
		pr_err("bq24262_init_ac_psy failed ret=%d\n", ret);
		goto err_init_ac_psy;
	}

	INIT_DELAYED_WORK(&chip->update_heartbeat_work, update_heartbeat);
	schedule_delayed_work(&chip->update_heartbeat_work,
			round_jiffies_relative(msecs_to_jiffies
				(UPDATE_TIME_MS)));

//	cable_init_termination(chip);

	ret = bq24262_create_debugfs_entries(chip);
	if (ret) {
		pr_err("bq24262_create_debugfs_entries failed ret=%d\n", ret);
		goto err_debugfs;
	}

#ifdef CONFIG_LGE_PM_IRQ_NOT_WORKGROUND
	INIT_DELAYED_WORK(&chip->irq_work, bq24262_irq_worker);
#else
	if(get_prop_hw_rev() <= HW_REV_A) {
		INIT_DELAYED_WORK(&chip->irq_work, bq24262_irq_worker_rev_a);
		schedule_delayed_work(&chip->irq_work,
				round_jiffies_relative(msecs_to_jiffies(IRQ_CHECK_PERIOD)));
	} else {
		INIT_DELAYED_WORK(&chip->irq_work, bq24262_irq_worker);
	}
#endif
	INIT_DELAYED_WORK(&chip->dpm_detect_work, bq24262_dpm_detect_work);


#ifdef CONFIG_LGE_PM_BQ2426X_USING_WATCHDOG
	INIT_DELAYED_WORK(&chip->check_watchdog_work, bq24262_check_watchdog_work);
	schedule_delayed_work(&chip->check_watchdog_work,
			round_jiffies_relative(msecs_to_jiffies(100)));
	mutex_init(&chip->setting_env);
#endif

#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
	INIT_DELAYED_WORK(&chip->battemp_work, bq24262_monitor_batt_temp);
#endif
#ifdef I2C_SUSPEND_WORKAROUND
	INIT_DELAYED_WORK(&chip->check_suspended_work,
			bq24262_check_suspended_worker);
#endif
#ifdef CONFIG_LGE_PM_DEBUG_CHECK_LOG
	INIT_DELAYED_WORK(&chip->charging_inform_work, charging_infor_log_worker);
	schedule_delayed_work(&chip->charging_inform_work,
				round_jiffies_relative(msecs_to_jiffies(DEBUG_POLLING_PERIOD)));
#endif

	chip->usb_present = bq24262_is_charger_present(chip);
	bq24262_enable_charging(chip, true);
	if(chip->usb_present) {
		power_supply_set_present(chip->usb_psy, chip->usb_present);
		chip->ac_present = chip->usb_present;
		schedule_delayed_work(&chip->dpm_detect_work, msecs_to_jiffies(100));
#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
		if (read_lge_battery_id()) {
			bq24262_enable_charging(chip, true);
			pr_debug("Inserted vaild battery\n");
		} else {
			bq24262_enable_charging(chip, false);
			pr_err("Inserted invaild battery!!! As soon as power-off\n");
		}
#endif
	}

	cable_init_termination(chip);

	bq24262_get_register(chip);
	ret = device_create_file(&client->dev, &dev_attr_at_charge);
	if (ret < 0) {
		pr_err("%s:File dev_attr_at_charge creation failed: %d\n",
				__func__, ret);
		ret = -ENODEV;
		goto err_at_charge;
	}

	ret = device_create_file(&client->dev, &dev_attr_at_current1);
	if (ret < 0) {
		pr_err("%s:File dev_attr_at_current creation failed: %d\n",
				__func__, ret);
		ret = -ENODEV;
		goto err_at_current;
	}


	ret = device_create_file(&client->dev, &dev_attr_at_chcomp1);
	if (ret < 0) {
		pr_err("%s:File dev_attr_at_chcomp creation failed: %d\n",
				__func__, ret);
		ret = -ENODEV;
		goto err_at_chcomp;
	}

	ret = device_create_file(&client->dev, &dev_attr_at_pmrst1);
	if (ret < 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, ret);
		ret = -ENODEV;
		goto err_at_pmrst;
	}

	ret = device_create_file(&client->dev, &dev_attr_at_otg);
	if (ret < 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, ret);
		ret = -ENODEV;
		goto err_at_otg;
	}


#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
	schedule_delayed_work(&chip->battemp_work, 5*HZ);
#endif
#ifdef CONFIG_LGE_PM_IRQ_NOT_WORKGROUND
	schedule_delayed_work(&chip->irq_work, msecs_to_jiffies(2000));
#else
	if (get_prop_hw_rev() > HW_REV_A) {
		schedule_delayed_work(&chip->irq_work, msecs_to_jiffies(2000));
	}
#endif
	if (ret)
		goto probe_fail;

	chip->probe_success = true;
	pr_info("bq24262 external charger probe : success\n");
	return 0;

probe_fail:
err_at_otg:
	device_remove_file(&client->dev, &dev_attr_at_pmrst1);
err_at_pmrst:
	device_remove_file(&client->dev, &dev_attr_at_chcomp1);
err_at_chcomp:
	device_remove_file(&client->dev, &dev_attr_at_charge);
err_at_charge:
err_at_current:
	device_remove_file(&client->dev, &dev_attr_at_current1);
err_req_irq:
	bq24262_remove_debugfs_entries(chip);
err_debugfs:
	power_supply_unregister(&chip->ac_psy);
err_init_ac_psy:
	power_supply_unregister(&chip->batt_psy);
err_init_batt_psy:
	wake_lock_destroy(&chip->chg_wake_lock);
	wake_lock_destroy(&chip->uevent_wake_lock);
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
	wake_lock_destroy(&chip->lcs_wake_lock);
#endif
err_hw_init:
	if (chip->int_gpio)
		gpio_free(chip->int_gpio);
	if (chip->ext_chg_disen)
		gpio_free(chip->ext_chg_disen);
	if (chip->stat_gpio)
		gpio_free(chip->stat_gpio);
error:
	kfree(chip);
	pr_err("fail to probe\n");
	return ret;

}

static int bq24262_remove(struct i2c_client *client)
{
	struct bq24262_chip *chip = i2c_get_clientdata(client);

	bq24262_remove_debugfs_entries(chip);

	device_remove_file(&client->dev, &dev_attr_at_charge);
	device_remove_file(&client->dev, &dev_attr_at_current1);
	device_remove_file(&client->dev, &dev_attr_at_chcomp1);
	device_remove_file(&client->dev, &dev_attr_at_pmrst1);
	device_remove_file(&client->dev, &dev_attr_at_otg);

	wake_lock_destroy(&chip->chg_wake_lock);
	wake_lock_destroy(&chip->uevent_wake_lock);
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
	wake_lock_destroy(&chip->lcs_wake_lock);
#endif
	cancel_delayed_work_sync(&chip->dpm_detect_work);
#ifdef CONFIG_LGE_PM_BQ2426X_USING_WATCHDOG
	cancel_delayed_work_sync(&chip->check_watchdog_work);
	mutex_destroy(&chip->setting_env);
#endif
#ifdef CONFIG_LGE_PM_DEBUG_CHECK_LOG
	cancel_delayed_work_sync(&chip->charging_inform_work);
#endif
	power_supply_unregister(&chip->ac_psy);
	power_supply_unregister(&chip->batt_psy);

	if (chip->irq)
		free_irq(chip->irq, chip);
	if (chip->int_gpio)
		gpio_free(chip->int_gpio);
	if (chip->ext_chg_disen)
		gpio_free(chip->ext_chg_disen);
	if (chip->stat_gpio)
		gpio_free(chip->stat_gpio);

	kfree(chip);
	chip = NULL;
	the_chip = NULL;
	return 0;
}

static const struct i2c_device_id bq24262_id[] = {
	{BQ24262_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, bq24262_id);

static const struct of_device_id bq24262_match[] = {
	{ .compatible = "ti,bq24262-charger", },
	{ },
};

static int bq24262_resume(struct i2c_client *client)
{
	struct bq24262_chip *chip = i2c_get_clientdata(client);

	chip->suspend = false;

	if ( chip && device_may_wakeup(&client->dev)) {
		disable_irq_wake(chip->irq);
	}
	pr_info("%s : resume sucess\n",__func__);

//	schedule_delayed_work(&chip->irq_work, msecs_to_jiffies(150));
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
	if(delayed_work_pending(&chip->battemp_work))
		cancel_delayed_work(&chip->battemp_work);
	if (the_chip->ac_present)
		schedule_delayed_work(&chip->battemp_work, msecs_to_jiffies(400));
#endif
	schedule_delayed_work(&chip->update_heartbeat_work, 0);
	if (the_chip->ac_present)
		schedule_delayed_work(&chip->dpm_detect_work, msecs_to_jiffies(500));
#ifdef CONFIG_LGE_PM_BQ2426X_USING_WATCHDOG
	schedule_delayed_work(&chip->check_watchdog_work, msecs_to_jiffies(0));
#endif
#ifdef CONFIG_LGE_PM_DEBUG_CHECK_LOG
	schedule_delayed_work(&chip->charging_inform_work, msecs_to_jiffies(500));
#endif
	return 0;
}

static int bq24262_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct bq24262_chip *chip = i2c_get_clientdata(client);

	chip->suspend = true;

//	mb();
//	flush_work(&chip->irq_work.work);

	if (chip && device_may_wakeup(&client->dev)) {
		enable_irq_wake(chip->irq);
	}
	pr_info("%s : suspend sucess\n", __func__);

	cancel_delayed_work_sync(&chip->update_heartbeat_work);
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
	cancel_delayed_work_sync(&chip->battemp_work);
#endif
	cancel_delayed_work_sync(&chip->dpm_detect_work);
#ifdef CONFIG_LGE_PM_BQ2426X_USING_WATCHDOG
	cancel_delayed_work_sync(&chip->check_watchdog_work);
#endif
#ifdef CONFIG_LGE_PM_DEBUG_CHECK_LOG
	cancel_delayed_work_sync(&chip->charging_inform_work);
#endif
	return 0;
}

static struct i2c_driver bq24262_driver = {
	.driver	= {
		.name	= BQ24262_NAME,
		.owner	= THIS_MODULE,
		.of_match_table	= of_match_ptr(bq24262_match),
	},
	.probe		= bq24262_probe,
	.remove		= bq24262_remove,
	.id_table	= bq24262_id,
	.resume     = bq24262_resume,
	.suspend    = bq24262_suspend,

};

static int __init bq24262_init(void)
{
	int result;
	result =i2c_add_driver(&bq24262_driver);
	pr_debug("bq24262_init result %d\n", result);

	return result;
}

module_init(bq24262_init);

static void __exit bq24262_exit(void)
{
	pr_debug("bq24262_exit\n");
	return i2c_del_driver(&bq24262_driver);
}
module_exit(bq24262_exit);

MODULE_DESCRIPTION("Driver for BQ24262 external charger chip");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:" BQ24262_NAME);
