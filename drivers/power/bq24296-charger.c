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

#define pr_fmt(fmt)	"bq24296:%s: " fmt, __func__

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/bitops.h>
#include <linux/ktime.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/wakelock.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/consumer.h>
#include <linux/spmi.h>
/*
#ifdef pr_debug
#undef pr_debug
#define pr_debug(fmt, ...) \
	printk(KERN_WARNING pr_fmt(fmt), ##__VA_ARGS__)
#endif
*/
//#define USB3_USED
#define BQ24296_NAME  "bq24296-charger"
/* Register definitions */
#define INPUT_SRC_CONT_REG              0X00
#define PWR_ON_CONF_REG                 0X01
#define CHARGE_CUR_CONT_REG             0X02
#define PRE_CHARGE_TERM_CUR_REG         0X03
#define CHARGE_VOLT_CONT_REG            0X04
#define CHARGE_TERM_TIMER_CONT_REG      0X05
#define IR_COMP_THERM_CONT_REG          0X06
#define MISC_OPERATION_CONT_REG         0X07
#define SYSTEM_STATUS_REG               0X08
#define FAULT_REG                       0X09
#define VENDOR_PART_REV_STATUS_REG      0X0A

#define EN_HIZ_MASK                0x80
#define RESET_REGISTER_MASK        0x80
#define CHG_CONFIG_MASK            0x30
#define EN_CHG_MASK                0x10
#define PG_STAT_MASK               0x04
#define OTG_EN_MASK                0x20
#define VBUS_STAT_MASK             0xC0
#define PRE_CHARGE_MASK            0x10
#define FAST_CHARGE_MASK           0x20
#define CHARGING_MASK (PRE_CHARGE_MASK | FAST_CHARGE_MASK)
#define CHG_DONE_MASK              0x30
#define INPUT_CURRENT_LIMIT_MASK   0x07
#define INPUT_VOLTAGE_LIMIT_MASK   0x78
#define SYSTEM_MIN_VOLTAGE_MASK    0x0E
#define PRECHG_CURRENT_MASK        0xF0
#define TERM_CURRENT_MASK          0x0F
#define CHG_VOLTAGE_LIMIT_MASK     0xFC
#define CHG_CURRENT_LIMIT_MASK     0xFC
#define EN_CHG_TERM_MASK           0xA0
#define EN_CHG_TIMER_MASK          0x08
#define I2C_TIMER_MASK             0x30
#define CHG_TIMER_LIMIT_MASK       0x06
#define IR_COMP_R_MASK             0xE0
#define IR_COMP_VCLAMP_MASK        0x1C
#define THERM_REG_MASK             0x03
#define BOOST_LIM_MASK             0x01
#define CHRG_FAULT_STATUS_MASK 	0x3
#define bq24296_FAST_CHG_MAX_MA		2000
#define BQ24296_CHRG_INPUT_DEFAULT_MAX_CURRENT_MA	2000
#define BQ24296_CHRG_IBAT_DEFAULT_MAX_CURRENT_MA	1600
#define BATTERY_DEFAULT_FCC_1C_MA		2300
struct bq24296_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

struct bq24296_chip {
	int  chg_current_ma;
	int  term_current_ma;
	int  vbat_max_mv;
	int  pre_chg_current_ma;
	int  sys_vmin_mv;
	int  vin_limit_mv;
	int  wlc_vin_limit_mv;
	int  int_gpio;
	int  otg_en_gpio;
	int  chg_en_gpio;
	int  irq;
	struct i2c_client  *client;
	struct work_struct  irq_work;
	int  irq_scheduled_time_status;
	spinlock_t irq_work_lock;
	struct delayed_work  vbat_work;
	struct delayed_work  input_limit_work;
	struct delayed_work  therm_work;
	struct delayed_work recharge_work;
	struct dentry  *dent;
	struct wake_lock  chg_wake_lock;
	struct wake_lock  icl_wake_lock;
	struct wake_lock  irq_wake_lock;
	struct power_supply  *usb_psy;
	struct power_supply  ac_psy;
	struct power_supply  *wlc_psy;
	struct power_supply  batt_psy;
	struct power_supply	 *bms_psy;
	struct qpnp_adc_tm_btm_param  adc_param;
	struct mutex		set_ibat_mutex;
	struct delayed_work timer_work;
	bool charger_enable_soft_term;
	bool charger_enable_recharge_fourty_min;
	int soft_term_capacity;
	int soft_term_timer;
	int the_timer;
	int  usb_online;
	int  ac_online;
	int	 usb_psy_type;
	int  ext_pwr;
	int  wlc_pwr;
	int  wlc_support;
	int  ext_ovp_otg_ctrl;
	int  set_chg_current_ma;
	int  vbat_noti_stat;
	int  step_dwn_thr_mv;
	int  step_dwn_currnet_ma;
	int  icl_idx;
	bool icl_first;
	int  icl_fail_cnt;
	int  icl_vbus_mv;
	int  dwn_chg_i_ma;
	int  up_chg_i_ma;
	int  dwn_input_i_ma;
	int  up_input_i_ma;
	int  wlc_dwn_i_ma;
	int  wlc_dwn_input_i_ma;
	int  batt_health;
	int  saved_ibat_ma;
	int  saved_input_i_ma;
	bool  therm_mitigation;
	int  max_input_i_ma;
	struct device 	*dev;
	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pin_default;
	const char		*bms_psy_name;
	int	fake_battery_soc;
	struct qpnp_vadc_chip   *vadc_dev;
	struct qpnp_adc_tm_chip	*adc_tm_dev;
	unsigned long bat_vol;
	int cuttent_input_limit;
	int cms_set_current_input_limit;
      int soc;
      int ui_soc;
      int cal_soc;
	int last_soc;
	bool  full_flag;
	bool		batt_full;
	bool		batt_full_flag;
	int		chg_present;
       long		psy_status;
	struct delayed_work hw_init_delayed_work;
	unsigned long    charge_begin;
	 int			chg_tmout_mins;
	 int 	chg_phase;
	 int empty_enabled;
	struct bq24296_regulator	otg_vreg;
	bool is_support_otg;
	bool powerpath_switch;
	int		vbus_vol;
	bool charger_enable_gpio;
	int  input_max_current_ma;
	int  real_input_max_current_ma;
	int  ibat_max_current_ma;
	int  batt_1C_current_ma;
	int adjust_dpm_flag;
	bool adjust_dpm_enable;
	struct mutex	hiz_mutex;
	int temp_count;
	int temp_ma;
	long cpu_temp;
	bool cpu_flag;
	bool charger_enable_temp_current;
	int *adjust_chg_temp;
	int *adjust_chg_current;
	struct delayed_work charger_ovp_monitor_work;
	bool vcdt_present;
};

static struct bq24296_chip *the_chip;
static int input_limit_idx = 0;
static bool bbk_cmcc_attribute = false;

struct debug_reg {
	char  *name;
	u8  reg;
};

#define bq24296_DEBUG_REG(x) {#x, x##_REG}

static struct debug_reg bq24296_debug_regs[] = {
	bq24296_DEBUG_REG(INPUT_SRC_CONT),
	bq24296_DEBUG_REG(PWR_ON_CONF),
	bq24296_DEBUG_REG(CHARGE_CUR_CONT),
	bq24296_DEBUG_REG(PRE_CHARGE_TERM_CUR),
	bq24296_DEBUG_REG(CHARGE_VOLT_CONT),
	bq24296_DEBUG_REG(CHARGE_TERM_TIMER_CONT),
	bq24296_DEBUG_REG(IR_COMP_THERM_CONT),
	bq24296_DEBUG_REG(MISC_OPERATION_CONT),
	bq24296_DEBUG_REG(SYSTEM_STATUS),
	bq24296_DEBUG_REG(FAULT),
	bq24296_DEBUG_REG(VENDOR_PART_REV_STATUS),
};

struct current_limit_entry {
	int input_limit;
	int chg_limit;
};

static struct current_limit_entry adap_tbl[] = {
	{500, 300},
	{700, 500},
	{900, 700},
	{1200, 1024},
	{2000, 1536},
};
enum {
	bq24296_CHG_OV_STATUS = 0,
	bq24296_CHG_TIMEOUT_STATUS = 4,
};
extern int sensor_get_temp(uint32_t sensor_id, long *temp);

static int bq24296_step_down_detect_disable(struct bq24296_chip *chip);
//static int bq24296_get_soc_from_batt_psy(struct bq24296_chip *chip);
static void bq24296_trigger_recharge(struct bq24296_chip *chip);
static int bq24296_get_vbus_voltage_now(struct bq24296_chip *chip);
static int bq24296_hw_init(struct bq24296_chip *chip);
static int bq24296_read_reg(struct i2c_client *client, int reg, u8 *val)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev,
			"i2c read fail: can't read from %02x: %d\n",
			reg, ret);
		return ret;
	} else {
		*val = ret;
	}

	return 0;
}

static int bq24296_write_reg(struct i2c_client *client, int reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		dev_err(&client->dev,
			"i2c write fail: can't write %02x to %02x: %d\n",
			val, reg, ret);
		return ret;
	}
	return 0;
}

static int bq24296_masked_write(struct i2c_client *client, int reg,
			       u8 mask, u8 val)
{
	s32 rc;
	u8 temp;

	rc = bq24296_read_reg(client, reg, &temp);
	if (rc) {
		pr_err("bq24296_read_reg failed: reg=%03X, rc=%d\n",
				reg, rc);
		return rc;
	}

	temp &= ~mask;
	temp |= val & mask;

	rc = bq24296_write_reg(client, reg, temp);
	if (rc) {
		pr_err("bq24296_write failed: reg=%03X, rc=%d\n",
				reg, rc);
		return rc;
	}

	return 0;
}

static bool bq24296_is_charger_present(struct bq24296_chip *chip)
{
	u8 temp;
	bool chg_online;
	int ret;

	ret = bq24296_read_reg(chip->client, SYSTEM_STATUS_REG, &temp);
	if (ret) {
		pr_err("failed to read SYSTEM_STATUS_REG rc=%d\n", ret);
		return false;
	}

	chg_online = temp & PG_STAT_MASK;
	pr_debug("charger present = %d\n", chg_online);

	return !!chg_online;
}

#define EN_HIZ_SHIFT 7
static int bq24296_enable_hiz(struct bq24296_chip *chip, bool enable)
{
	int ret;
	u8 val = (u8)(!!enable << EN_HIZ_SHIFT);
	mutex_lock(&chip->hiz_mutex);
	ret = bq24296_masked_write(chip->client, INPUT_SRC_CONT_REG,
				EN_HIZ_MASK, val);
	if (ret) {
		pr_err("failed to set HIZ rc=%d\n", ret);
		mutex_unlock(&chip->hiz_mutex);
		return ret;
	}
	chip->empty_enabled = enable;
	mutex_unlock(&chip->hiz_mutex);
	return 0;
}
static int bq24296_is_chg_enabled(struct bq24296_chip *chip)
{
	int ret;
	u8 temp;

	ret = bq24296_read_reg(chip->client, PWR_ON_CONF_REG, &temp);
	if (ret) {
		pr_err("failed to read PWR_ON_CONF_REG rc=%d\n", ret);
		return ret;
	}

	return (temp & CHG_CONFIG_MASK) == 0x10;
}
static int bq24296_get_prop_current_input_max(struct bq24296_chip *chip)
{
	return chip->real_input_max_current_ma * 1000;
}

#define CHG_ENABLE_SHIFT  4
static int bq24296_enable_charging(struct bq24296_chip *chip, bool enable)
{
	int ret;
	int chg_en_gpio;
	bool enable_check;
	u8 val;
	if(chip->charger_enable_gpio){
	enable_check = !enable;


	if (chip->chg_en_gpio)
		gpio_direction_output(chip->chg_en_gpio, enable_check);

	chg_en_gpio = gpio_get_value(chip->chg_en_gpio);

		pr_info("chg_en_gpio=%d,chip->chg_en_gpio = %d\n", chg_en_gpio, chip->chg_en_gpio);
	}else{
	    val = (u8)(!!enable << CHG_ENABLE_SHIFT);
		ret = bq24296_masked_write(chip->client, PWR_ON_CONF_REG, EN_CHG_MASK, val);	
		if(ret)
			pr_err("failed to set EN_CHG rc=%d\n", ret);
		else
			pr_info("charger enable=%d\n", enable);
	}

	return 0;
}

struct input_ma_limit_entry {
	int  icl_ma;
	u8  value;
};

static struct input_ma_limit_entry icl_ma_table[] = {
	{100, 0x00},
	{150, 0x01},
	{500, 0x02},
	{900, 0x03},
	{1000, 0x04},
	{1500, 0x05},
	{2000, 0x06},
	{3000, 0x07},
};

#define INPUT_CURRENT_LIMIT_MIN_MA  100
#define INPUT_CURRENT_LIMIT_MAX_MA  3000
static int bq24296_set_input_i_limit(struct bq24296_chip *chip, int ma)
{
	int i;
	u8 temp;
	
	if (ma < INPUT_CURRENT_LIMIT_MIN_MA
			|| ma > INPUT_CURRENT_LIMIT_MAX_MA) {
		pr_err("bad mA=%d asked to set\n", ma);
		return -EINVAL;
	}

	for (i = ARRAY_SIZE(icl_ma_table) - 1; i >= 0; i--) {
		if (icl_ma_table[i].icl_ma <= ma)
			break;
	}

	if (i < 0) {
		pr_err("can't find %d in icl_ma_table. Use min.\n", ma);
		i = 0;
	}

	temp = icl_ma_table[i].value;

	if (ma > chip->max_input_i_ma) {
		chip->saved_input_i_ma = ma;
		pr_info("reject %d mA due to therm mitigation\n", ma);
		return 0;
	}

	if (!chip->therm_mitigation)
		chip->saved_input_i_ma = ma;

	chip->therm_mitigation = false;
	pr_err("input current limit = %d setting 0x%02x\n", ma, temp);
	return bq24296_masked_write(chip->client, INPUT_SRC_CONT_REG,
			INPUT_CURRENT_LIMIT_MASK, temp);
}
static void bq24296_show_register(struct bq24296_chip *chip);
static int mitigate_tbl[] = {3000, 900, 500, 100};
static void bq24296_therm_mitigation_work(struct work_struct *work)
{
	struct bq24296_chip *chip = container_of(work,
				struct bq24296_chip, therm_work.work);
	int ret;
	int input_limit_ma;

	return;
	chip->max_input_i_ma = mitigate_tbl[input_limit_idx];
	if (chip->max_input_i_ma < chip->saved_input_i_ma) {
		input_limit_ma = chip->max_input_i_ma;
		chip->therm_mitigation = true;
	} else {
		input_limit_ma = chip->saved_input_i_ma;
		chip->therm_mitigation = false;
	}

	ret = bq24296_set_input_i_limit(chip, input_limit_ma);
	if (ret)
		pr_err("failed to set input current limit as %d\n",
					input_limit_ma);
}

static int bq24296_therm_set_input_i_limit(const char *val,
					const struct kernel_param *kp)
{
	int ret;

	if (!the_chip)
		return -ENODEV;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("failed to set input_limit_idx\n");
		return ret;
	}

	if (input_limit_idx >= ARRAY_SIZE(mitigate_tbl))
		input_limit_idx = ARRAY_SIZE(mitigate_tbl) - 1;

	if (!power_supply_is_system_supplied())
		return 0;

	schedule_delayed_work(&the_chip->therm_work,
			msecs_to_jiffies(2000));

	return 0;
}

static struct kernel_param_ops therm_input_limit_ops = {
	.set = bq24296_therm_set_input_i_limit,
	.get = param_get_int,
};
module_param_cb(input_current_idx, &therm_input_limit_ops,
				&input_limit_idx, 0644);

#define IBAT_MAX_MA  4532
#define IBAT_MIN_MA  512
#define IBAT_STEP_MA  64
static int bq24296_set_ibat_max(struct bq24296_chip *chip, int ma)
{
	u8 reg_val = 0;
	int set_ibat = 0;
	int ret;

	mutex_lock(&chip->set_ibat_mutex);

	if (ma > IBAT_MAX_MA) {
		pr_err("bad mA=%d asked to set\n", ma);
		mutex_unlock(&chip->set_ibat_mutex);
		return -EINVAL;
	}

	if(ma < IBAT_MIN_MA){
		if ((chip->batt_health == POWER_SUPPLY_HEALTH_OVERHEAT)
				&& (ma > chip->set_chg_current_ma)) {
			chip->saved_ibat_ma = ma;
			pr_info("reject %d mA setting due to overheat\n", ma);
			mutex_unlock(&chip->set_ibat_mutex);
			return 0;
		}

		reg_val = (512 - IBAT_MIN_MA)/IBAT_STEP_MA;
		set_ibat = reg_val * IBAT_STEP_MA + IBAT_MIN_MA;
		reg_val = reg_val << 2;
		pr_err("small current req_ibat = %d set_ibat = %d reg_val = 0x%02x\n",
				ma, set_ibat, reg_val);

		ret = bq24296_masked_write(chip->client, CHARGE_CUR_CONT_REG,
				CHG_CURRENT_LIMIT_MASK, reg_val);
		if(!ret)
			chip->set_chg_current_ma = ma;

		mutex_unlock(&chip->set_ibat_mutex);

		return ret;
	}

	if ((chip->batt_health == POWER_SUPPLY_HEALTH_OVERHEAT)
			&& (ma > chip->set_chg_current_ma)) {
		chip->saved_ibat_ma = ma;
		pr_info("reject %d mA setting due to overheat\n", ma);
		mutex_unlock(&chip->set_ibat_mutex);
		return 0;
	}

	reg_val = (ma - IBAT_MIN_MA)/IBAT_STEP_MA;
	set_ibat = reg_val * IBAT_STEP_MA + IBAT_MIN_MA;
	reg_val = reg_val << 2;
	pr_err("req_ibat = %d set_ibat = %d reg_val = 0x%02x\n",
				ma, set_ibat, reg_val);

	ret = bq24296_masked_write(chip->client, CHARGE_CUR_CONT_REG,
			CHG_CURRENT_LIMIT_MASK, reg_val);
	if(!ret)
		chip->set_chg_current_ma = ma;

	mutex_unlock(&chip->set_ibat_mutex);
	return ret;
}

static int bq24296_check_restore_ibatt(struct bq24296_chip *chip,
				int old_health, int new_health)
{
	if (old_health == new_health)
		return 0;

	switch (new_health) {
	case POWER_SUPPLY_HEALTH_OVERHEAT:
		chip->saved_ibat_ma = chip->set_chg_current_ma;
		break;
	case POWER_SUPPLY_HEALTH_GOOD:
		chip->batt_health = new_health;
		if (chip->saved_ibat_ma) {
			bq24296_set_ibat_max(chip,
					chip->saved_ibat_ma);
			pr_info("restore ibat max = %d by decreasing temp",
						chip->saved_ibat_ma);
		}
		chip->saved_ibat_ma = 0;
		break;
	case POWER_SUPPLY_HEALTH_COLD:
	case POWER_SUPPLY_HEALTH_UNKNOWN:
	default:
		break;
	}

	return 0;
}

#define VIN_LIMIT_MIN_MV  3880
#define VIN_LIMIT_MAX_MV  5080
#define VIN_LIMIT_STEP_MV  80
static int bq24296_set_input_vin_limit(struct bq24296_chip *chip, int mv)
{
	u8 reg_val = 0;
	int set_vin = 0;

	union power_supply_propval volt_now = {0,};

	int ret;
	u8 reg_old_val;

	int vbat_uv,i;
	int vbat_uv_sum = 0;

	//adjust dpm ,according to battery voltage
	if(chip->adjust_dpm_enable){
		for(i=0; i<3; i++){
			chip->batt_psy.get_property(&(chip->batt_psy),
					POWER_SUPPLY_PROP_VOLTAGE_NOW, &volt_now);
			vbat_uv_sum += volt_now.intval;
		}
		vbat_uv = vbat_uv_sum/3;

		pr_err("### volt_now = %d ###\n",vbat_uv);

		if(vbat_uv < 4100000){
			if(!chip->adjust_dpm_flag)
				mv = 4440;
			else{
				if(vbat_uv < 3900000){
					mv = 4440;
					chip->adjust_dpm_flag = 0;
				}
				else
					mv = 4600;
			}
		}else{
			mv = 4600;
			chip->adjust_dpm_flag = 1;
		}
	}else{
		mv = 4600;
	}

	if (mv < VIN_LIMIT_MIN_MV || mv > VIN_LIMIT_MAX_MV) {
		pr_err("bad mV=%d asked to set\n", mv);
		return -EINVAL;
	}
	pr_err("### adjust_dpm_flag %d, set dpm volt = %d ###\n",chip->adjust_dpm_flag,mv);

	reg_val = (mv - VIN_LIMIT_MIN_MV)/VIN_LIMIT_STEP_MV;
	set_vin = reg_val * VIN_LIMIT_STEP_MV + VIN_LIMIT_MIN_MV;
	reg_val = reg_val << 3;

	ret = bq24296_read_reg(chip->client, INPUT_SRC_CONT_REG, &reg_old_val);
	if (ret) {
		pr_err("i2c read fail\n");
		return false;
	}

	reg_old_val &= INPUT_VOLTAGE_LIMIT_MASK;

	pr_err("### req_vin = %d set_vin = %d reg_val = 0x%02x reg_old_val = 0x%02x###\n",
				mv, set_vin, reg_val,reg_old_val);

	if(reg_old_val == reg_val){
		pr_err("### no set ###\n");
		return 0;
	}

	return bq24296_masked_write(chip->client, INPUT_SRC_CONT_REG,
			INPUT_VOLTAGE_LIMIT_MASK, reg_val);
}

#define VBAT_MAX_MV  4400
#define VBAT_MIN_MV  3504
#define VBAT_STEP_MV  16
static int bq24296_set_vbat_max(struct bq24296_chip *chip, int mv)
{
	u8 reg_val = 0;
	int set_vbat = 0;

	if (mv < VBAT_MIN_MV || mv > VBAT_MAX_MV) {
		pr_err("bad mv=%d asked to set\n", mv);
		return -EINVAL;
	}

	reg_val = (mv - VBAT_MIN_MV)/VBAT_STEP_MV;
	set_vbat = reg_val * VBAT_STEP_MV + VBAT_MIN_MV;
	reg_val = reg_val << 2;

	pr_debug("req_vbat = %d set_vbat = %d reg_val = 0x%02x\n",
				mv, set_vbat, reg_val);

	return bq24296_masked_write(chip->client, CHARGE_VOLT_CONT_REG,
			CHG_VOLTAGE_LIMIT_MASK, reg_val);
}

#define SYSTEM_VMIN_LOW_MV  3000
#define SYSTEM_VMIN_HIGH_MV  3700
#define SYSTEM_VMIN_STEP_MV  100
static int bq24296_set_system_vmin(struct bq24296_chip *chip, int mv)
{
	u8 reg_val = 0;
	int set_vmin = 0;

	if (mv < SYSTEM_VMIN_LOW_MV || mv > SYSTEM_VMIN_HIGH_MV) {
		pr_err("bad mv=%d asked to set\n", mv);
		return -EINVAL;
	}

	reg_val = (mv - SYSTEM_VMIN_LOW_MV)/SYSTEM_VMIN_STEP_MV;
	set_vmin = reg_val * SYSTEM_VMIN_STEP_MV + SYSTEM_VMIN_LOW_MV;
	reg_val = reg_val << 1;

	pr_debug("req_vmin = %d set_vmin = %d reg_val = 0x%02x\n",
				mv, set_vmin, reg_val);

	return bq24296_masked_write(chip->client, PWR_ON_CONF_REG,
			SYSTEM_MIN_VOLTAGE_MASK, reg_val);
}

#define IPRECHG_MIN_MA  128
#define IPRECHG_MAX_MA  2048
#define IPRECHG_STEP_MA  128
static int bq24296_set_prechg_i_limit(struct bq24296_chip *chip, int ma)
{
	u8 reg_val = 0;
	int set_ma = 0;

	if (ma < IPRECHG_MIN_MA || ma > IPRECHG_MAX_MA) {
		pr_err("bad ma=%d asked to set\n", ma);
		return -EINVAL;
	}

	reg_val = (ma - IPRECHG_MIN_MA)/IPRECHG_STEP_MA;
	set_ma = reg_val * IPRECHG_STEP_MA + IPRECHG_MIN_MA;
	reg_val = reg_val << 4;

	pr_debug("req_i = %d set_i = %d reg_val = 0x%02x\n",
				ma, set_ma, reg_val);

	return bq24296_masked_write(chip->client, PRE_CHARGE_TERM_CUR_REG,
			PRECHG_CURRENT_MASK, reg_val);
}

#define ITERM_MIN_MA  128
#define ITERM_MAX_MA  2048
#define ITERM_STEP_MA  128
static int bq24296_set_term_current(struct bq24296_chip *chip, int ma)
{
	u8 reg_val = 0;
	int set_ma = 0;

	if (ma < ITERM_MIN_MA || ma > ITERM_MAX_MA) {
		pr_err("bad mv=%d asked to set\n", ma);
		return -EINVAL;
	}

	reg_val = (ma - ITERM_MIN_MA)/ITERM_STEP_MA;
	set_ma = reg_val * ITERM_STEP_MA + ITERM_MIN_MA;

	pr_debug("req_i = %d set_i = %d reg_val = 0x%02x\n",
				ma, set_ma, reg_val);

	return bq24296_masked_write(chip->client, PRE_CHARGE_TERM_CUR_REG,
			TERM_CURRENT_MASK, reg_val);
}

#define IRCOMP_R_MIN_MOHM  0
#define IRCOMP_R_MAX_MOHM  70
#define IRCOMP_R_STEP_MOHM  10
static int bq24296_set_ir_comp_resister(struct bq24296_chip *chip, int mohm)
{
	u8 reg_val = 0;
	int set_ma = 0;
return 0;
	if (mohm < IRCOMP_R_MIN_MOHM
			|| mohm > IRCOMP_R_MAX_MOHM) {
		pr_err("bad r=%d asked to set\n", mohm);
		return -EINVAL;
	}

	reg_val = (mohm - IRCOMP_R_MIN_MOHM)/IRCOMP_R_STEP_MOHM;
	set_ma = reg_val * IRCOMP_R_STEP_MOHM + IRCOMP_R_MIN_MOHM;
	reg_val = reg_val << 5;

	pr_debug("req_r = %d set_r = %d reg_val = 0x%02x\n",
				mohm, set_ma, reg_val);

	return bq24296_masked_write(chip->client, IR_COMP_THERM_CONT_REG,
			IR_COMP_R_MASK, reg_val);
}

#define IRCOMP_VCLAMP_MIN_MV  0
#define IRCOMP_VCLAMP_MAX_MV  112
#define IRCOMP_VCLAMP_STEP_MV  16
static int bq24296_set_vclamp_mv(struct bq24296_chip *chip, int mv)
{
	u8 reg_val = 0;
	int set_ma = 0;
return 0;
	if (mv < IRCOMP_VCLAMP_MIN_MV
			|| mv > IRCOMP_VCLAMP_MAX_MV) {
		pr_err("bad mv=%d asked to set\n", mv);
		return -EINVAL;
	}

	reg_val = (mv - IRCOMP_VCLAMP_MIN_MV)/IRCOMP_VCLAMP_STEP_MV;
	set_ma = reg_val * IRCOMP_VCLAMP_STEP_MV + IRCOMP_VCLAMP_MIN_MV;
	reg_val = reg_val << 2;

	pr_debug("req_mv = %d set_mv = %d reg_val = 0x%02x\n",
				mv, set_ma, reg_val);

	return bq24296_masked_write(chip->client, IR_COMP_THERM_CONT_REG,
			IR_COMP_VCLAMP_MASK, reg_val);
}

static void bq24296_show_register(struct bq24296_chip *chip)
{
	int rc;
	u8 temp = 0,reg;
	for(reg = 0; reg <= 0xa; reg++)
	{
	  	rc = bq24296_read_reg(chip->client, reg, &temp);
		pr_err("reg=0x%x,value=0x%x\n",reg,temp);
	}
}

static void charger_ovp_monitor_work(struct work_struct *work)
{
	int work_period_ms = msecs_to_jiffies(5000);

	struct bq24296_chip *chip = container_of(work,
					struct bq24296_chip, charger_ovp_monitor_work.work);

	bool ext_pwr,chg_ovp;
	u8 system_status, fault_status;
	int rc;

	rc = bq24296_read_reg(chip->client, SYSTEM_STATUS_REG, &system_status);
	rc = bq24296_read_reg(chip->client, FAULT_REG, &fault_status);
	rc = bq24296_read_reg(chip->client, FAULT_REG, &fault_status);


	ext_pwr = !!(system_status & PG_STAT_MASK);
	chg_ovp = ((fault_status >> 4 & CHRG_FAULT_STATUS_MASK) == 1);

	pr_err("power_good=%d,chg_ovp=%d\n",ext_pwr,chg_ovp);

	if(!ext_pwr && !chg_ovp){
		pr_err("external ovp,shutdown\n");
		power_supply_set_present(chip->usb_psy, 0);
		power_supply_changed(chip->usb_psy);
	}else if(ext_pwr && !chg_ovp){
		pr_err("power good,but not ovp\n");
	}else{
		schedule_delayed_work(&chip->charger_ovp_monitor_work,
			round_jiffies_relative(work_period_ms));
	}
}

extern void charger_connect_judge(char on_or_off);
static void bq24296_irq_worker(struct work_struct *work)
{
	struct bq24296_chip *chip =
		container_of(work, struct bq24296_chip, irq_work);
	union power_supply_propval ret = {0,};
	bool ext_pwr,chg_ovp;
	bool wlc_pwr = 0;
	bool chg_done = false;
	u8 temp, fault_status;
	int rc;
	unsigned long flags;
	static int recharge = 0;

	if (!wake_lock_active(&chip->irq_wake_lock))
		wake_lock(&chip->irq_wake_lock);

	msleep(100 * chip->irq_scheduled_time_status);

	rc = bq24296_read_reg(chip->client, SYSTEM_STATUS_REG, &temp);
	rc = bq24296_read_reg(chip->client, FAULT_REG, &fault_status);
	rc = bq24296_read_reg(chip->client, FAULT_REG, &fault_status);
	/* Open up for next possible interrupt handler beyond read reg
	 * asap, lest we miss an interrupt
	 */
	spin_lock_irqsave(&chip->irq_work_lock, flags);
	chip->irq_scheduled_time_status = 0;
	spin_unlock_irqrestore(&chip->irq_work_lock, flags);

	if (rc) {
		pr_err("failed to read SYSTEM_STATUS_REG rc=%d\n", rc);
		goto irq_worker_exit;
	}

	ext_pwr = !!(temp & PG_STAT_MASK);
	chg_ovp = ((fault_status >> 4 & CHRG_FAULT_STATUS_MASK) == 1);
	if(chg_ovp){
		rc = bq24296_read_reg(chip->client, FAULT_REG, &fault_status);
		chg_ovp = ((fault_status >> 4 & CHRG_FAULT_STATUS_MASK) == 1);
		if(!chip->vcdt_present){
			if(chg_ovp){
				set_bit(bq24296_CHG_OV_STATUS, &chip->psy_status);
				cancel_delayed_work_sync(&chip->charger_ovp_monitor_work);
				schedule_delayed_work(&chip->charger_ovp_monitor_work,0);
				pr_err("charger ovp\n");
			}else{
				pr_err("charger un-ovp\n");
			}
		}else{
			bq24296_get_vbus_voltage_now(chip);
			if(chip->vbus_vol > 6300000 && chg_ovp){
				set_bit(bq24296_CHG_OV_STATUS, &chip->psy_status);
				pr_err("charger ovp\n");
			}else if(chip->vbus_vol < 3600000 && chg_ovp){
				chg_ovp = 0;
				pr_err("usb unplug\n");
			}else{
				pr_err("bad power source\n");
			}
		}
	}
	pr_err("chg_ovp=%d,fault_status=%d,xxx=%d\n",chg_ovp,fault_status,fault_status & CHRG_FAULT_STATUS_MASK);
	ext_pwr = ext_pwr || chg_ovp;
	if(ext_pwr){
			chip->chg_present = true;
			charger_connect_judge(1);
			chip->cms_set_current_input_limit = chip->input_max_current_ma;
	        if (!wake_lock_active(&chip->chg_wake_lock)){
			pr_info("enable charger get chg_wake_lock\n");
        		wake_lock(&chip->chg_wake_lock);
		}
	}else{
			chip->chg_present = false;
			chip->batt_full = false;
			chip->batt_full_flag = false;
			chip->adjust_dpm_flag = 0;
			charger_connect_judge(0);
			chip->set_chg_current_ma = 0;
			chip->usb_psy_type = 0;

			chip->temp_count = 0;
			chip->temp_ma = 0;
			chip->cpu_temp = 0;
			chip->cpu_flag = false;

			clear_bit(bq24296_CHG_TIMEOUT_STATUS, &chip->psy_status);
			clear_bit(bq24296_CHG_OV_STATUS, &chip->psy_status);
			if (wake_lock_active(&chip->chg_wake_lock)){
			pr_info("disable charger release chg_wake_lock\n");
			wake_unlock(&chip->chg_wake_lock);
			}
			//fake bat_full clear the_time
			if(chip->charger_enable_soft_term){
				if(chip->soc >= chip->soft_term_capacity && chip->the_timer > 0)
					schedule_delayed_work(&chip->timer_work, 0);
				else
					chip->the_timer = 0;
			}
			recharge = 0;
			cancel_delayed_work_sync(&chip->recharge_work);
			cancel_delayed_work_sync(&chip->charger_ovp_monitor_work);
	}
	chg_done = (temp & CHARGING_MASK) == 0x30 ? true : false;
	pr_err("chg_done=%d,temp=%d,fault_staus=%d\n",chg_done, temp, fault_status);
	if (chg_done) {
		cancel_delayed_work_sync(&chip->timer_work);
		chip->the_timer = 0;
		chip->batt_full = true;
		//add for PA exception
		 if(chip->powerpath_switch){
			bq24296_enable_hiz(chip, true);
		 }
		//end
		power_supply_changed(&chip->batt_psy);
		pr_err("charge done!!\n");

		if(chip->charger_enable_recharge_fourty_min && !recharge){
			recharge = 1;
			bq24296_enable_charging(chip, false);
			msleep(2000);
			rc = bq24296_masked_write(chip->client, CHARGE_TERM_TIMER_CONT_REG,
					0x80, 0);
			if (rc) {
				pr_err("failed to disable termination\n");
			}
			pr_err("### 4 recharge now  ###\n");
			schedule_delayed_work(&chip->recharge_work,
				round_jiffies_relative(msecs_to_jiffies(40*60*1000)));

			bq24296_enable_charging(chip, true);
		}
	}
	if (chip->wlc_psy) {
		chip->wlc_psy->get_property(chip->wlc_psy,
				POWER_SUPPLY_PROP_PRESENT, &ret);
		wlc_pwr = ret.intval;
	}

	if ((chip->ext_pwr ^ ext_pwr) || (chip->wlc_pwr ^ wlc_pwr)) {
		if(ext_pwr){
			bq24296_hw_init(chip);
			bq24296_enable_charging(chip, true);
		}else{
			if(chip->empty_enabled)
				bq24296_enable_hiz(chip, false);
		}
		pr_err("power source changed! ext_pwr = %d wlc_pwr = %d\n",
				ext_pwr, wlc_pwr);
		if (wake_lock_active(&chip->icl_wake_lock))
			wake_unlock(&chip->icl_wake_lock);

		cancel_delayed_work_sync(&chip->input_limit_work);
		cancel_delayed_work_sync(&chip->therm_work);
		bq24296_step_down_detect_disable(chip);
		chip->saved_ibat_ma = 0;
		chip->max_input_i_ma = INPUT_CURRENT_LIMIT_MAX_MA;

		if (chip->wlc_psy) {
			if (wlc_pwr && ext_pwr) {
				chip->wlc_pwr = true;
				power_supply_set_online(chip->wlc_psy, true);
			} else if (chip->wlc_pwr && !(ext_pwr && wlc_pwr)) {
				chip->wlc_pwr = false;
				power_supply_set_online(chip->wlc_psy, false);
			}
		}
		chip->ext_pwr = ext_pwr;
		if (!wlc_pwr) {
			pr_err("notify vbus to usb otg ext_pwr = %d\n", ext_pwr);
			power_supply_set_present(chip->usb_psy, ext_pwr);
		}

	}

irq_worker_exit:
	wake_lock_timeout(&chip->irq_wake_lock, 2*HZ);
}

#ifdef CONFIG_THERMAL_QPNP_ADC_TM
#undef CONFIG_THERMAL_QPNP_ADC_TM
#endif
#ifdef CONFIG_THERMAL_QPNP_ADC_TM
#define DISABLE_HIGH_THR 6000000
#define DISABLE_LOW_THR 0
static void bq24296_vbat_work(struct work_struct *work)
{
	struct bq24296_chip *chip =
		container_of(work, struct bq24296_chip, vbat_work.work);
	int step_current_ma;
	int step_input_i_ma;

	if (chip->vbat_noti_stat == ADC_TM_HIGH_STATE) {
		step_current_ma = chip->dwn_chg_i_ma;
		step_input_i_ma = chip->dwn_input_i_ma;
		chip->adc_param.state_request = ADC_TM_LOW_THR_ENABLE;
		chip->adc_param.high_thr = DISABLE_HIGH_THR;
		chip->adc_param.low_thr = (chip->step_dwn_thr_mv - 100) * 1000;
	} else {
		step_current_ma = chip->up_chg_i_ma;
		step_input_i_ma = chip->up_input_i_ma;
		chip->adc_param.state_request = ADC_TM_HIGH_THR_ENABLE;
		chip->adc_param.high_thr = chip->step_dwn_thr_mv * 1000;
		chip->adc_param.low_thr = DISABLE_LOW_THR;
	}

	if (bq24296_is_charger_present(chip)) {
		pr_info("change to chg current = %d, input_limit = %d\n",
				step_current_ma, step_input_i_ma);
		bq24296_set_input_i_limit(chip, step_input_i_ma);
		bq24296_set_ibat_max(chip, step_current_ma);
		qpnp_adc_tm_channel_measure(&chip->adc_param);
	}
	wake_unlock(&chip->chg_wake_lock);
}

static void bq24296_vbat_notification(enum qpnp_tm_state state, void *ctx)
{
	struct bq24296_chip *chip = ctx;

	wake_lock(&chip->chg_wake_lock);
	chip->vbat_noti_stat = state;
	schedule_delayed_work(&chip->vbat_work, msecs_to_jiffies(100));
}

static int bq24296_step_down_detect_disable(struct bq24296_chip *chip)
{
	int ret;

	chip->adc_param.state_request = ADC_TM_HIGH_LOW_THR_DISABLE;
	chip->adc_param.threshold_notification = bq24296_vbat_notification;
	chip->adc_param.channel = VBAT_SNS;

	ret = qpnp_adc_tm_channel_measure(&chip->adc_param);
	if (ret)
		pr_err("request ADC error %d\n", ret);

	cancel_delayed_work_sync(&chip->vbat_work);
	if (wake_lock_active(&chip->chg_wake_lock)) {
		pr_debug("releasing wakelock\n");
		wake_unlock(&chip->chg_wake_lock);
	}

	return ret;
}

static int bq24296_step_down_detect_init(struct bq24296_chip *chip)
{
	int ret;

	ret = qpnp_adc_tm_is_ready();
	if (ret) {
		pr_err("qpnp_adc is not ready");
		return ret;
	}

	chip->adc_param.high_thr = chip->step_dwn_thr_mv * 1000;
	chip->adc_param.low_thr = DISABLE_LOW_THR;
	chip->adc_param.timer_interval = ADC_MEAS1_INTERVAL_2S;
	chip->adc_param.state_request = ADC_TM_HIGH_LOW_THR_ENABLE;
	chip->adc_param.btm_ctx = chip;
	chip->adc_param.threshold_notification = bq24296_vbat_notification;
	chip->adc_param.channel = VBAT_SNS;

	ret = qpnp_adc_tm_channel_measure(&chip->adc_param);
	if (ret)
		pr_err("request ADC error %d\n", ret);

	return ret;
}
#else
static void bq24296_vbat_work(struct work_struct *work)
{
	pr_warn("vbat notification is not supported!\n");
}
static int bq24296_step_down_detect_disable(struct bq24296_chip *chip)
{
	pr_warn("vbat notification is not supported!\n");
	return 0;
}

static int bq24296_step_down_detect_init(struct bq24296_chip *chip)
{
	pr_warn("vbat notification is not supported!\n");
	return 0;
}
#endif

static irqreturn_t bq24296_irq(int irq, void *dev_id)
{
	struct bq24296_chip *chip = dev_id;
	unsigned long flags;

	spin_lock_irqsave(&chip->irq_work_lock, flags);
	if (chip->irq_scheduled_time_status == 0) {
		schedule_work(&chip->irq_work);
		chip->irq_scheduled_time_status = 1;
	}
	spin_unlock_irqrestore(&chip->irq_work_lock, flags);

	return IRQ_HANDLED;
}

static int set_reg(void *data, u64 val)
{
	u32 addr = *((u32*) data);
	int ret;
	struct i2c_client *client = the_chip->client;

	ret = bq24296_write_reg(client, addr, (u8) val);

	return ret;
}

static int get_reg(void *data, u64 *val)
{
	u32 addr = *((u32*) data);
	u8 temp;
	int ret;
	struct i2c_client *client = the_chip->client;

	ret = bq24296_read_reg(client, addr, &temp);
	if (ret < 0)
		return ret;

	*val = temp;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(reg_fops, get_reg, set_reg, "0x%02llx\n");

#define OTG_ENABLE_SHIFT  5
static int bq24296_enable_otg(struct bq24296_chip *chip, bool enable)
{
	int ret;
	u8 val = (u8)(!!enable << OTG_ENABLE_SHIFT);

	pr_info("otg enable = %d\n", enable);
	if(enable){
		ret = bq24296_masked_write(chip->client, PWR_ON_CONF_REG,
			BOOST_LIM_MASK, 0);
		ret = bq24296_masked_write(chip->client, CHARGE_TERM_TIMER_CONT_REG,
				0x30, 0x00);
		if (ret) {
			pr_err("failed to disable watchdog\n");
		}
	}else{
		ret = bq24296_masked_write(chip->client, CHARGE_TERM_TIMER_CONT_REG,
				0x30, 0x20);
		if (ret) {
			pr_err("failed to enable default watchdog\n");
		}
	}

	ret = bq24296_masked_write(chip->client, PWR_ON_CONF_REG,
					OTG_EN_MASK, val);
	if (ret) {
		pr_err("failed to set OTG_EN rc=%d\n", ret);
		return ret;
	}

	if (chip->otg_en_gpio)
		gpio_set_value(chip->otg_en_gpio, enable);
	return 0;
}

static bool bq24296_is_otg_mode(struct bq24296_chip *chip)
{
	u8 temp;
	int ret;

	ret = bq24296_read_reg(chip->client, PWR_ON_CONF_REG, &temp);
	if (ret) {
		pr_err("failed to read OTG enable bits =%d\n", ret);
		return false;
	}

	return !!(temp & OTG_EN_MASK);
}
static bool bq24296_is_chg_done(struct bq24296_chip *chip)
{
	int ret;
	u8 temp;

	ret = bq24296_read_reg(chip->client, SYSTEM_STATUS_REG, &temp);
	if (ret) {
		pr_err("i2c read fail\n");
		return false;
	}

	return (temp & CHG_DONE_MASK) == CHG_DONE_MASK;
}

static void bq24296_trigger_recharge(struct bq24296_chip *chip)
{

	if (chip->batt_health != POWER_SUPPLY_HEALTH_GOOD)
		return;

	if (!bq24296_is_chg_done(chip))
		return;

	bq24296_enable_hiz(chip, true);
	bq24296_enable_hiz(chip, false);
}

#define WLC_BOUNCE_INTERVAL_MS 15000
#define WLC_BOUNCE_COUNT 3
static bool bq24296_is_wlc_bounced(struct bq24296_chip *chip)
{
	ktime_t now_time;
	uint32_t interval_ms;
	static ktime_t prev_time;
	static int bounced_cnt = 0;

	now_time = ktime_get();

	interval_ms = (uint32_t)ktime_to_ms(ktime_sub(now_time, prev_time));
	if (interval_ms < WLC_BOUNCE_INTERVAL_MS)
		bounced_cnt ++;
	else
		bounced_cnt = 0;

	prev_time = now_time;
	if (bounced_cnt >= WLC_BOUNCE_COUNT) {
		pr_info("detect wlc bouncing!\n");
		bounced_cnt = 0;
		return true;
	}

	return false;
}

static int bq24296_select_current_by_cc(struct bq24296_chip *chip, int type, int limit_current_max)
{
		union power_supply_propval ret = {0,};
		int mA,tmp_mA;

		if(!limit_current_max){
			bq24296_enable_charging(chip, false);
			chip->set_chg_current_ma  = 0;
			return 0;
		}
		if (type == POWER_SUPPLY_TYPE_USB || POWER_SUPPLY_TYPE_USB_FLOATED == type) {
			#if defined(USB3_USED)
			if(type == POWER_SUPPLY_TYPE_USB){
				chip->usb_psy->get_property(chip->usb_psy,
					  POWER_SUPPLY_PROP_CURRENT_MAX, &ret);
			}else{
				ret.intval = 500 * 1000;
			}
			tmp_mA = 512;
			#else
			if(type == POWER_SUPPLY_TYPE_USB){
				ret.intval = 500 * 1000;
				tmp_mA = 512;
			}else{
				ret.intval = 900 * 1000;
				tmp_mA = 900;
			}
			#endif
			mA = min(limit_current_max, tmp_mA);
			//bq24296_set_input_vin_limit(chip, chip->vin_limit_mv);
			/*if ibat set current is small than 512, we use input current as limit current*/
			if(mA < 512){
				if(mA < 150)
					ret.intval = 100 * 1000;
				else
					ret.intval = 150 * 1000;
			}
			chip->cuttent_input_limit = ret.intval /1000;
			bq24296_set_input_i_limit(chip, ret.intval / 1000);
			bq24296_set_ibat_max(chip, mA);
			pr_err("usb is online! i_limit = %d v_limit = %d\n",	ret.intval / 1000, chip->vin_limit_mv);

		} else if(type == POWER_SUPPLY_TYPE_USB_DCP){
			chip->icl_first = true;
			//bq24296_set_input_vin_limit(chip, chip->icl_vbus_mv - 2 * VIN_LIMIT_STEP_MV);
			mA = min(limit_current_max, chip->ibat_max_current_ma);
			/*if ibat set current is small than 400, we use input current as limit current*/
			if(mA < 400){
				if(mA < 150)
					chip->cuttent_input_limit = 100;
				else
					chip->cuttent_input_limit = 150;
				chip->real_input_max_current_ma = chip->cuttent_input_limit;
			}else{
				chip->real_input_max_current_ma = chip->input_max_current_ma;
				if(chip->cms_set_current_input_limit >= chip->input_max_current_ma){
					chip->cms_set_current_input_limit = chip->input_max_current_ma;
					chip->cuttent_input_limit = chip->cms_set_current_input_limit;
				}else{
					chip->cuttent_input_limit = chip->cms_set_current_input_limit;
				}
			}
			bq24296_set_input_i_limit(chip, chip->cuttent_input_limit);
			bq24296_set_ibat_max(chip, mA);
			pr_err("dc set current, input:%d ma, ibat:%d ma\n", chip->cuttent_input_limit, mA);
		}else{
			chip->set_chg_current_ma = 0;
			pr_info("chg type detect fail\n");
			return -1;
		}
		return 0;
}

static void bq24296_recharge_work(struct work_struct *work)
{
	int rc;
	struct bq24296_chip *chip = container_of(work,
					struct bq24296_chip,
					recharge_work.work);

	bq24296_enable_charging(chip, false);
	msleep(2000);
	rc = bq24296_masked_write(chip->client, CHARGE_TERM_TIMER_CONT_REG,
						0x80, 0x80);
	if (rc) {
		pr_err("####### failed to enable termination ####\n");
	}
	pr_err("##### 3 enable termination  ##### \n");

}


static void bq24296_timer_work(struct work_struct *work)
{
	int work_period_ms = msecs_to_jiffies(10000);

	struct bq24296_chip *chip = container_of(work,
					struct bq24296_chip,
					timer_work.work);

	if(chip->the_timer > 0)
		chip->the_timer--;
	pr_err("### discharging ### %d \n",chip->the_timer);

	if(chip->soc >= chip->soft_term_capacity && chip->the_timer > 0){
		schedule_delayed_work(&chip->timer_work,
			round_jiffies_relative(work_period_ms));
	}else if(chip->soc < chip->soft_term_capacity){
		chip->the_timer = 0;
	}

}
static int bq24296_is_termination_of_soft(struct bq24296_chip *chip)
{
	union power_supply_propval ret = {0, };
	int soc = 0;

	if(bq24296_is_chg_enabled(chip)){
		if (chip->bms_psy) {
			chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_CAPACITY, &ret);
			soc = ret.intval;
			if(soc >= chip->soft_term_capacity){
				cancel_delayed_work_sync(&chip->timer_work);
				pr_err("### charging ### %d \n",chip->the_timer);
				if(chip->the_timer++ >= chip->soft_term_timer){
					chip->the_timer--;
					pr_err("### charging terminate(not bq24296) ###\n");
					msleep(5000);
					chip->batt_full = true;
					power_supply_changed(&chip->batt_psy);
				}
			}
		}
	}else{
	}

	return 0;
}
static int bq24296_check_vbat_max(struct bq24296_chip *chip)
{
	union power_supply_propval val = {0,};
	static int mv = 0;

	if(mv != chip->vbat_max_mv){
		if(!chip->bms_psy){
			pr_err("### bms not found! ###\n");
			return -1;
		}else{
			chip->bms_psy->get_property(chip->bms_psy, POWER_SUPPLY_PROP_VOLTAGE_MAX, &val);
			pr_err("#### get battery max_voltage_uv : %d ####\n", val.intval);
			if(val.intval == 4350)
				mv = 4352;
			else if(val.intval == 4380 || val.intval == 4400)
				mv = 4384;
			else
				mv = val.intval;

			if(mv != 0){
				pr_err("### final set vbat = %d ###\n",mv);
				bq24296_set_vbat_max(chip, mv);
				chip->vbat_max_mv = mv;
				return 1;
			}
		}
	}
	return 0;
}
//add start : according to cpu temp,adjust charge current
static int bq24296_adjust_chg_cur_by_cpu_temp(struct bq24296_chip *chip,int current_ma)
{
	long curr_cpu_temp = 0;

	pr_err("### new init current_ma=%d ###\n", current_ma);
	if(chip->temp_count >=15)
	{
		sensor_get_temp(5, &curr_cpu_temp);
		chip->cpu_temp += curr_cpu_temp;
		chip->cpu_temp = chip->cpu_temp / 3;

		if(chip->cpu_temp > chip->adjust_chg_temp[2])
			chip->temp_ma = chip->adjust_chg_current[3];
		else if(chip->cpu_temp > chip->adjust_chg_temp[1] && chip->cpu_temp <= chip->adjust_chg_temp[2])
			chip->temp_ma = chip->adjust_chg_current[2];
		else if(chip->cpu_temp > chip->adjust_chg_temp[0] && chip->cpu_temp <= chip->adjust_chg_temp[1])
			chip->temp_ma = chip->adjust_chg_current[1];
		else
			chip->temp_ma = chip->adjust_chg_current[0];

		pr_err("### temp_count=%d,cpu_temp = %ld,temp_ma=%d ###\n",chip->temp_count, chip->cpu_temp,chip->temp_ma);
		chip->temp_count = 0;
		chip->cpu_temp = 0;
		chip->cpu_flag = true;
	}else{
		chip->temp_count++;
		switch(chip->temp_count)
		{
			case 13:
					sensor_get_temp(5, &curr_cpu_temp);
					break;
			case 14:
					sensor_get_temp(5, &curr_cpu_temp);
					break;
			default:
					curr_cpu_temp = 0;
		}
		chip->cpu_temp += curr_cpu_temp;
		pr_err("### temp_count=%d,cpu_temp = %ld,temp_ma=%d ###\n",chip->temp_count, chip->cpu_temp,chip->temp_ma);
      }

	if(chip->cpu_flag){
		pr_err("### current_ma=%d, temp_ma=%d,dev->cpu_flag=%d ###\n",current_ma, chip->temp_ma, chip->cpu_flag);
		current_ma = min(current_ma, chip->temp_ma);
	}
	pr_err("### final current_ma=%d ###\n", current_ma);
	return current_ma;
}
//add end
#define IBAT_CURRENT_LIMIT_MAX_MA	3008
static int bq24296_set_chg_current_now(struct bq24296_chip *chip, int current_ma)
{
	int ret = 0;
	
    if(current_ma > IBAT_CURRENT_LIMIT_MAX_MA || current_ma < 0){
		pr_err("invalid current %d\n", current_ma);
		return 0;
	}
	  
	/*limit the current for 700mA adapter*/
	if(current_ma > chip->ibat_max_current_ma)
		current_ma = chip->ibat_max_current_ma;

	//according to cpu temp,adjust charge current
	if(chip->charger_enable_temp_current){
		current_ma = bq24296_adjust_chg_cur_by_cpu_temp(chip,current_ma);
	}

	pr_debug("chip->set_chg_current_ma=%d,current_ma=%d,usb_type=%d\n",chip->set_chg_current_ma, current_ma, chip->usb_psy_type);
	if(chip->set_chg_current_ma != current_ma || chip->cms_set_current_input_limit != chip->cuttent_input_limit){
		//chip->set_chg_current_ma = current_ma;
	    	ret = bq24296_select_current_by_cc(chip, chip->usb_psy_type, current_ma);
		if(ret < 0)
			clear_bit(bq24296_CHG_TIMEOUT_STATUS, &chip->psy_status);
		
	}
	ret = bq24296_masked_write(chip->client, PWR_ON_CONF_REG,
			0x40, 0x40);
	if (ret) {
		pr_err("failed to kick dog\n");
	}

	if(chip->charger_enable_soft_term){
		if (!chip->batt_full){
			bq24296_is_termination_of_soft(chip);
			pr_err("### %d %d %d ###\n",chip->charger_enable_soft_term,
					chip->soft_term_capacity,chip->soft_term_timer);
		}
	}
	bq24296_set_input_vin_limit(chip, chip->vin_limit_mv);

	bq24296_check_vbat_max(chip);

	bq24296_show_register(chip);
    return 0;
}

static void bq24296_battery_external_power_changed(struct power_supply *psy)
{
	pr_debug("\n");
}

#define WLC_INPUT_I_LIMIT_MA 900
#define USB_MAX_IBAT_MA 1500
static void bq24296_external_power_changed(struct power_supply *psy)
{
	struct bq24296_chip *chip = container_of(psy,
					struct bq24296_chip, ac_psy);
	union power_supply_propval ret = {0,};
	union power_supply_propval usb_type = {0,};
	int wlc_online = 0;
	int wlc_chg_current_ma = 0;
	int rc = 0;

	pr_debug("\n");
	if (!chip->bms_psy && chip->bms_psy_name){
		chip->bms_psy = power_supply_get_by_name((char *)chip->bms_psy_name);
	}

	if (chip->bms_psy) {
		chip->bms_psy->set_property(chip->bms_psy,
				POWER_SUPPLY_PROP_MPP2, &ret);
	}
	chip->usb_psy->get_property(chip->usb_psy,
			  POWER_SUPPLY_PROP_ONLINE, &ret);
	chip->usb_online = ret.intval;

	chip->usb_psy->get_property(chip->usb_psy,
			POWER_SUPPLY_PROP_CHG_TYPE, &usb_type);

	if (chip->wlc_support) {
		chip->wlc_psy->get_property(chip->wlc_psy,
				  POWER_SUPPLY_PROP_ONLINE, &ret);
		wlc_online = ret.intval;

		chip->wlc_psy->get_property(chip->wlc_psy,
				  POWER_SUPPLY_PROP_CURRENT_MAX, &ret);
		wlc_chg_current_ma = ret.intval / 1000;
	}
	pr_warn("usb_online=%d,last_type=%d,type=%d\n",
			chip->usb_online,chip->usb_psy_type,usb_type.intval);
	if (chip->usb_online && bq24296_is_charger_present(chip)) {

			if(chip->usb_psy_type != usb_type.intval){
				rc = bq24296_select_current_by_cc(chip, usb_type.intval,  chip->ibat_max_current_ma);
				if(rc < 0)
					clear_bit(bq24296_CHG_TIMEOUT_STATUS, &chip->psy_status);
				schedule_delayed_work(&chip->input_limit_work,
								msecs_to_jiffies(200));
				pr_warn("type changed start or stop charge\n");
				chip->usb_psy_type = usb_type.intval;
		  }

	} else if (wlc_online) {
		chip->dwn_chg_i_ma = chip->wlc_dwn_i_ma;
		chip->up_chg_i_ma = wlc_chg_current_ma;
		chip->dwn_input_i_ma = chip->wlc_dwn_input_i_ma;
		if (bq24296_is_wlc_bounced(chip))
			chip->up_input_i_ma = chip->wlc_dwn_input_i_ma;
		else
			chip->up_input_i_ma = WLC_INPUT_I_LIMIT_MA;
		bq24296_set_input_vin_limit(chip, chip->wlc_vin_limit_mv);
		bq24296_set_input_i_limit(chip, chip->up_input_i_ma);
		bq24296_set_ibat_max(chip, wlc_chg_current_ma);
		bq24296_step_down_detect_init(chip);
		pr_info("wlc is online! i_limit = %d v_limit = %d\n",
				wlc_chg_current_ma, chip->wlc_vin_limit_mv);
	}

	power_supply_changed(&chip->batt_psy);
}

#define FAIL_DEFAULT_SOC 50

static int bq24296_get_prop_charge_type(struct bq24296_chip *chip)
{
	int chg_type = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	int ret;
	u8 temp;

	ret = bq24296_read_reg(chip->client, SYSTEM_STATUS_REG, &temp);
	if (ret) {
		pr_err("i2c read fail\n");
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}

	temp = temp & CHARGING_MASK;

	if (temp == FAST_CHARGE_MASK)
		chg_type = POWER_SUPPLY_CHARGE_TYPE_FAST;
	else if (temp == PRE_CHARGE_MASK)
		chg_type = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	else
		chg_type = POWER_SUPPLY_CHARGE_TYPE_NONE;

     if (chip->chg_phase != chg_type) 
     {
    	if (chg_type == POWER_SUPPLY_CHARGE_TYPE_NONE) {
    		pr_info("Charging stopped.\n");
			
    	} else{
		pr_info("Charging started.\n");
            chip->charge_begin = jiffies;
    	}
     }
     
    chip->chg_phase = chg_type;
    return chg_type;
}

static bool bq24296_is_recharging(struct bq24296_chip *chip)
{
	bool rechg = false;
	int chg_type = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	if(!chip->powerpath_switch){
		chg_type = bq24296_get_prop_charge_type(chip);

		if (chip->ui_soc == 100 && chip->batt_full_flag && (chg_type == POWER_SUPPLY_CHARGE_TYPE_TRICKLE || chg_type == POWER_SUPPLY_CHARGE_TYPE_FAST))
			rechg = true;
		else
			rechg = false;
	}else{
	//add for PA exception
	      chg_type = bq24296_get_prop_charge_type(chip);
		pr_err("ui_soc=%d,bat_full=%d,cal_soc=%d,last_soc=%d,chg_type=%d\n",chip->ui_soc,chip->batt_full,chip->cal_soc,chip->last_soc,chg_type);
		 if(chip->ui_soc == 100 && chip->batt_full_flag && chip->cal_soc != chip->last_soc && chip->cal_soc < chip->last_soc)
			rechg = true;
		else if(chip->ui_soc == 100 && chip->batt_full_flag && (chg_type == POWER_SUPPLY_CHARGE_TYPE_TRICKLE || chg_type == POWER_SUPPLY_CHARGE_TYPE_FAST))
			rechg = true;
		else
			rechg = false;
	//end
	}
	pr_info("rechargering:%d\n", rechg);

	return rechg;
}

static int bq24296_get_prop_chg_status(struct bq24296_chip *chip)
{
	int chg_type = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	int chg_status = POWER_SUPPLY_STATUS_UNKNOWN;
	//int soc = 0;
	bool recharging = bq24296_is_recharging(chip);

	if(chip->batt_full_flag && chip->cal_soc >= 98){
		chg_status = POWER_SUPPLY_STATUS_FULL;
		return chg_status;
	}

	if(recharging && chip->cal_soc < 98){
		chip->batt_full = false;
		chip->batt_full_flag = false;
	}

	//soc = bq24296_get_soc_from_batt_psy(chip);
	chg_type = bq24296_get_prop_charge_type(chip);
	switch (chg_type) {
	case POWER_SUPPLY_CHARGE_TYPE_NONE:
		if (bq24296_is_charger_present(chip)) {
			if(chip->batt_full && chip->ui_soc >= 100){
				chg_status = POWER_SUPPLY_STATUS_FULL;
				chip->batt_full_flag = true;
			}
			else if(chip->batt_full && chip->ui_soc < 100)
				chg_status = POWER_SUPPLY_STATUS_CHARGING;
			else
				chg_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		} else {
			chg_status = POWER_SUPPLY_STATUS_DISCHARGING;
		}
		break;
	case POWER_SUPPLY_CHARGE_TYPE_TRICKLE:
	case POWER_SUPPLY_CHARGE_TYPE_FAST:
		chg_status = POWER_SUPPLY_STATUS_CHARGING;
		break;
	default:
		break;
	}

	pr_err("chg status = %d\n", chg_status);
	return chg_status;
}

static ssize_t Show_CMCC_Attribute(struct device *dev,struct device_attribute *attr, char *buf)
{

	pr_err("[CMCC_Attribute] Show_CMCC_Attribute,bbk_cmcc_attribute=%d\n", bbk_cmcc_attribute);
	return sprintf(buf, "%d\n", bbk_cmcc_attribute);
}

static ssize_t Store_CMCC_Attribute(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{

	if(buf!=NULL && size!=0 && strstr(buf, "LG4")!= NULL)
		bbk_cmcc_attribute = true;
	else
		bbk_cmcc_attribute = false;

	pr_err("[CMCC_Attribute] Store_CMCC_Attribute ,bbk_cmcc_attribute =%d\n",bbk_cmcc_attribute);

	return size;
}

static DEVICE_ATTR(CMCC_Attribute,0664,Show_CMCC_Attribute,Store_CMCC_Attribute);
static struct attribute *cmcc_attrs[] = {
	&dev_attr_CMCC_Attribute.attr,
	NULL
};
static const struct attribute_group cmcc_attr_group = {
	.attrs = cmcc_attrs,
};

#define UNINIT_VBUS_UV 5000000
static int bq24296_get_prop_input_voltage(struct bq24296_chip *chip)
{
	int volt;
	if(!chip->vcdt_present){
		volt = UNINIT_VBUS_UV;
	}else{
		volt = bq24296_get_vbus_voltage_now(chip);
	}
	return volt;
}
static int bq24296_get_prop_battery_health_status(struct bq24296_chip *chip)
{

	unsigned long timeout; 
	if(chip->charge_begin > 0)
	{
		timeout= chip->charge_begin + chip->chg_tmout_mins * 60 * HZ;
		if(chip->chg_phase > POWER_SUPPLY_CHARGE_TYPE_NONE &&
			time_after(jiffies, timeout)){
			if(bbk_cmcc_attribute)
			;//ignore for ccmc
			else
			set_bit(bq24296_CHG_TIMEOUT_STATUS, &chip->psy_status);
		}
		if(chip->chg_phase > POWER_SUPPLY_CHARGE_TYPE_NONE){
		pr_err("charging runs %d seconds,status=%ld,chg_tmout_mins=%d\n", jiffies_to_msecs(jiffies-chip->charge_begin)/ 1000,chip->psy_status,chip->chg_tmout_mins);
		}
	}

	pr_err("the psy_status=%ld\n", chip->psy_status);
	return chip->psy_status;
}
static int bq24296_wait_chg_status(struct bq24296_chip *chip, int chg)
{
	int retry = 0;
	for(retry = 0; retry < 15; retry++){
		bq24296_get_prop_charge_type(chip);
		pr_err("chg=%d,chg_phase=%d,retry=%d\n",chg,chip->chg_phase,retry);
		if(chg &&  chip->chg_phase > POWER_SUPPLY_CHARGE_TYPE_NONE)
			return 0;
		if(!chg && chip->chg_phase == POWER_SUPPLY_CHARGE_TYPE_NONE)
			return 0;
		msleep(200);
	}
	pr_err("wait %s status timeout!\n", chg ? "chg" : "none-chg");
	return -ETIME;
}
static void bq24296_input_limit_worker(struct work_struct *work)
{
	struct bq24296_chip *chip = container_of(work, struct bq24296_chip,
						input_limit_work.work);
	int vbus_mv = 0;
	
	//pr_err("chip->cuttent_input_limit=%d\n",chip->cuttent_input_limit);
	//bq24296_set_input_i_limit(chip, chip->cuttent_input_limit);
	bq24296_wait_chg_status(chip, chip->ext_pwr);
	
	return;
	vbus_mv = bq24296_get_prop_input_voltage(chip);
	vbus_mv = vbus_mv/1000;

	pr_info("vbus_mv = %d\n", vbus_mv);

	if (chip->icl_first && chip->icl_idx > 0) {
		chip->icl_fail_cnt++;
		if (chip->icl_fail_cnt > 1)
			vbus_mv = chip->icl_vbus_mv;
		else
			chip->icl_idx = 0;
	}
	chip->icl_first = false;

	if (vbus_mv > chip->icl_vbus_mv
			&& chip->icl_idx < (ARRAY_SIZE(adap_tbl) - 1)) {
		chip->icl_idx++;
		bq24296_set_input_i_limit(chip,
				adap_tbl[chip->icl_idx].input_limit);
		bq24296_set_ibat_max(chip,
				adap_tbl[chip->icl_idx].chg_limit);
		schedule_delayed_work(&chip->input_limit_work,
					msecs_to_jiffies(500));
	} else {
		if (chip->icl_idx > 0 && vbus_mv <= chip->icl_vbus_mv)
			chip->icl_idx--;

		bq24296_set_input_vin_limit(chip, chip->vin_limit_mv);
		bq24296_set_input_i_limit(chip,
				adap_tbl[chip->icl_idx].input_limit);
		bq24296_set_ibat_max(chip,
				adap_tbl[chip->icl_idx].chg_limit);

		if (adap_tbl[chip->icl_idx].chg_limit
				> chip->step_dwn_currnet_ma) {
			chip->dwn_chg_i_ma = chip->step_dwn_currnet_ma;
			chip->up_chg_i_ma = adap_tbl[chip->icl_idx].chg_limit;
			chip->dwn_input_i_ma = adap_tbl[chip->icl_idx].input_limit;
			chip->up_input_i_ma = adap_tbl[chip->icl_idx].input_limit;
			bq24296_step_down_detect_init(chip);
		}

		pr_info("optimal input i limit = %d chg limit = %d\n",
					adap_tbl[chip->icl_idx].input_limit,
					adap_tbl[chip->icl_idx].chg_limit);
		chip->icl_idx = 0;
		chip->icl_fail_cnt = 0;
		wake_unlock(&chip->icl_wake_lock);
	}
}

static char *bq24296_power_supplied_to[] = {
	"cms",
};

static enum power_supply_property bq24296_power_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_INPUT_CURRENT_MAX,
	POWER_SUPPLY_PROP_LIMIT_INPUT,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
};

static int bq24296_power_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct bq24296_chip *chip = container_of(psy,
					struct bq24296_chip,
					ac_psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bq24296_get_prop_battery_health_status(chip);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = chip->batt_health;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval =  chip->batt_1C_current_ma;//chip->set_chg_current_ma * 1000;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chip->ac_online;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = bq24296_get_prop_charge_type(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = bq24296_get_prop_input_voltage(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = chip->vbat_max_mv * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = bq24296_is_chg_enabled(chip);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
        val->intval = bq24296_get_prop_current_input_max(chip);
        break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		pr_debug("get bbk_cmcc_attribute=%d\n",bbk_cmcc_attribute);
		val->intval = bbk_cmcc_attribute;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int bq24296_power_set_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val)
{
	struct bq24296_chip *chip = container_of(psy,
						struct bq24296_chip,
						ac_psy);
	unsigned long flags;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		spin_lock_irqsave(&chip->irq_work_lock, flags);
		if (chip->irq_scheduled_time_status == 0) {
			schedule_work(&chip->irq_work);
			/* Set by wireless needs 2 units of 100 ms delay */
			chip->irq_scheduled_time_status = 2;
		}
		spin_unlock_irqrestore(&chip->irq_work_lock, flags);
		return 0;
	case POWER_SUPPLY_PROP_HEALTH:
		bq24296_check_restore_ibatt(chip,
				chip->batt_health, val->intval);
		chip->batt_health = val->intval;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		chip->ac_online = val->intval;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		bq24296_set_ibat_max(chip, val->intval / 1000);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		bq24296_enable_charging(chip, val->intval);
		if (val->intval)
			bq24296_trigger_recharge(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		bq24296_set_vbat_max(chip, val->intval / 1000);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		bq24296_set_chg_current_now(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_LIMIT_INPUT:
		chip->cms_set_current_input_limit = val->intval;
		if(chip->cms_set_current_input_limit > chip->input_max_current_ma)
			chip->cms_set_current_input_limit = chip->input_max_current_ma;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		pr_debug("set bbk_cmcc_attribute=%d\n",val->intval);
		bbk_cmcc_attribute = !!val->intval;
		break;
	default:
		return -EINVAL;
	}
	power_supply_changed(&chip->ac_psy);
	return 0;
}
static char *bq24296_battery_supplied_to[] = {
	"no",//"bms",
};

static enum power_supply_property bq24296_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
};
static int bq24296_get_prop_batt_present(struct bq24296_chip *chip)
{

	return 1;

}

#define DEFAULT_BATT_CAPACITY	50
static int bq24296_get_prop_batt_capacity(struct bq24296_chip *chip)
{
	union power_supply_propval ret = {0, };
	static bool the_flag = true;
	if (chip->fake_battery_soc >= 0)
		return chip->fake_battery_soc;
	if (!chip->bms_psy && chip->bms_psy_name){
		chip->bms_psy = power_supply_get_by_name((char *)chip->bms_psy_name);
	}

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_CAPACITY, &ret);
		chip->soc= ret.intval;
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_VOLTAGE_AVG, &ret);
		chip->cal_soc = ret.intval;
		
		 pr_err("soc=%d,ui_soc=%d,bat_vol=%ld,chg_present=%d,cal_soc=%d\n",chip->soc,chip->ui_soc,chip->bat_vol,chip->chg_present,chip->cal_soc);
		 pr_err("powerpath_switch=%d,empty_enabled=%d\n",chip->powerpath_switch,chip->empty_enabled);
			if(chip->batt_full == true)
			{
				the_flag = false;
				chip->ui_soc = chip->soc;
				chip->full_flag = true;
				 //add for PA exception
				 if(chip->powerpath_switch){
					 if(bq24296_is_recharging(chip)){
	 					if(chip->empty_enabled == true)
							bq24296_enable_hiz(chip, false);
					 }
					 chip->last_soc = chip->cal_soc;
				 }
				 //end
			      return chip->ui_soc;
			}else{
				//add for PA exception
				if(chip->powerpath_switch){
					if(chip->empty_enabled == true)
						bq24296_enable_hiz(chip, false);
				}
				//end
			}
			chip->ui_soc = chip->soc;
			if(chip->chg_present){
				if(the_flag == true)
					if(chip->ui_soc == 100 && chip->full_flag == false)
						chip->full_flag = true;
				if(chip->ui_soc < 100){
					the_flag = false;
					chip->full_flag = false;
				}
			    if(chip->ui_soc >=100 && !chip->full_flag)
			        chip->ui_soc = 99;
			}else{
				if(the_flag == true)
					if(chip->ui_soc == 100 && chip->full_flag == false)
						chip->full_flag = true;
			    if(chip->ui_soc<100){
					the_flag = false;
			        if(chip->full_flag==true)
			            chip->full_flag=false;
			    }else if(chip->ui_soc>=100 && chip->full_flag==false){
			        chip->ui_soc = 99;
			    }
			}
			//add for factory test
			if(!chip->chg_present){
				if(chip->bat_vol > 3600000 && !chip->ui_soc)
					return 1;
			}
			//end
			return chip->ui_soc;
	}
	

	return DEFAULT_BATT_CAPACITY;
}
#define DEFAULT_TEMP 250
static int bq24296_get_prop_batt_temp(struct bq24296_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX1_BATT_THERM, &results);
	if (rc) {
		pr_debug("Unable to read batt temperature rc=%d\n", rc);
		return DEFAULT_TEMP;
	}
	pr_debug("get_bat_temp %d, %lld\n",
		results.adc_code, results.physical);
	return (int)results.physical;
}

static int bq24296_get_prop_battery_voltage_now(struct bq24296_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;
	int vbat_uv,i;
	int vbat_uv_sum = 0;
	for(i=0; i<3; i++){
		rc = qpnp_vadc_read(chip->vadc_dev, VBAT_SNS, &results);
		if (rc) {
			chip->bat_vol = 0;
			pr_err("Unable to read vbat rc=%d\n", rc);
			return 0;
		}
		vbat_uv_sum += results.physical;
	}
	vbat_uv = vbat_uv_sum/3;
	chip->bat_vol = vbat_uv;
	return vbat_uv;
}
#define ADC_TO_VBUS_SCALE	6
static int bq24296_get_vbus_voltage_now(struct bq24296_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;
	rc = qpnp_vadc_read(chip->vadc_dev, P_MUX2_1_1, &results);
	if (rc) {
		chip->vbus_vol = 0;
		pr_err("Unable to read vbus rc=%d\n", rc);
		return 0;
	}
	pr_debug("vbus adc value:%lld, vbus voltage:%lld uv\n", results.physical, results.physical * ADC_TO_VBUS_SCALE);
	chip->vbus_vol = results.physical * ADC_TO_VBUS_SCALE;
	return chip->vbus_vol;
}
static void set_prop_charging_empty(struct bq24296_chip *chip, 
	int enabled)
{
	int empty_enabled = !!enabled;
	pr_info("set charge empty:%d\n", empty_enabled);
	if(empty_enabled){
		power_supply_set_supply_type(chip->usb_psy, 0);
		power_supply_changed(chip->usb_psy);
		bq24296_enable_hiz(chip, true);
	} else {
		bq24296_enable_hiz(chip, false);
		power_supply_set_supply_type(chip->usb_psy, POWER_SUPPLY_TYPE_USB);
		power_supply_changed(chip->usb_psy);
	}
}
static int bq24296_battery_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct bq24296_chip *chip = container_of(psy,
				struct bq24296_chip, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bq24296_get_prop_chg_status(chip);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = bq24296_get_prop_batt_present(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = bq24296_is_chg_enabled(chip);//chip->chg_enabled;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = bq24296_get_prop_charge_type(chip);//get_prop_charge_type(chip);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = bq24296_get_prop_batt_capacity(chip);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = chip->batt_health;//get_prop_batt_health(chip);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_FLASH_CURRENT_MAX:
		val->intval = 2000;
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		val->intval = 1500;//chip->therm_lvl_sel;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = bq24296_get_prop_batt_temp(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = bq24296_get_prop_battery_voltage_now(chip);
	break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
	    val->intval = chip->vbat_max_mv * 1000;
	    break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = 500 * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = chip->batt_full;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int bq24296_battery_set_property(struct power_supply *psy,
				  enum power_supply_property prop,
				  const union power_supply_propval *val)
{
	struct bq24296_chip *chip = container_of(psy,
						struct bq24296_chip,
						batt_psy);
	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		//chip->chg_enabled = val->intval;
		bq24296_enable_charging(chip,val->intval);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		chip->fake_battery_soc = val->intval;
		power_supply_changed(&chip->batt_psy);
		break;
	case POWER_SUPPLY_PROP_DC_CUTOFF:
		set_prop_charging_empty(chip, val->intval);
		break;
	default:
		return -EINVAL;
	}
	return 0;

}
static int bq24296_battery_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CAPACITY:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}
	return rc;
}
static int bq24296_create_debugfs_entries(struct bq24296_chip *chip)
{
	int i;

	chip->dent = debugfs_create_dir(BQ24296_NAME, NULL);
	if (IS_ERR(chip->dent)) {
		pr_err("bq24296 driver couldn't create debugfs dir\n");
		return -EFAULT;
	}

	for (i = 0 ; i < ARRAY_SIZE(bq24296_debug_regs) ; i++) {
		char *name = bq24296_debug_regs[i].name;
		u32 reg = bq24296_debug_regs[i].reg;
		struct dentry *file;

		file = debugfs_create_file(name, 0644, chip->dent,
					(void *) &reg, &reg_fops);
		if (IS_ERR(file)) {
			pr_err("debugfs_create_file %s failed.\n", name);
			return -EFAULT;
		}
	}

	return 0;
}

static int qb24296_chg_otg_regulator_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct bq24296_chip *chip = rdev_get_drvdata(rdev);
	rc = bq24296_enable_otg(chip, true);
	if (rc)
		dev_err(chip->dev, "Couldn't enable  OTG mode rc=%d\n", rc);
	return rc;
}
static int qb24296_chg_otg_regulator_disable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct bq24296_chip *chip = rdev_get_drvdata(rdev);
	rc = bq24296_enable_otg(chip, false);
	if (rc)
		dev_err(chip->dev, "Couldn't disable  OTG mode rc=%d\n", rc);
	return rc;
}
static int qb24296_chg_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	struct bq24296_chip *chip = rdev_get_drvdata(rdev);
	return bq24296_is_otg_mode(chip);
}
struct regulator_ops qb24296_chg_otg_reg_ops = {
	.enable		= qb24296_chg_otg_regulator_enable,
	.disable	= qb24296_chg_otg_regulator_disable,
	.is_enabled	= qb24296_chg_otg_regulator_is_enable,
};
static int bq24296_regulator_init(struct bq24296_chip *chip)
{
	int rc = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};
	init_data = of_get_regulator_init_data(chip->dev, chip->dev->of_node);
	if (!init_data) {
		pr_err("Unable to allocate memory\n");
		return -ENOMEM;
	}
	if (init_data->constraints.name) {
		chip->otg_vreg.rdesc.owner = THIS_MODULE;
		chip->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		chip->otg_vreg.rdesc.ops = &qb24296_chg_otg_reg_ops;
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
				pr_err("OTG register failed, rc=%d\n", rc);
		}
	}
	return rc;
}
static int bq24296_init_ac_psy(struct bq24296_chip *chip)
{
	int ret = 0;

	chip->ac_psy.name = "smb-dc";
	chip->ac_psy.type = POWER_SUPPLY_TYPE_MAINS;
	chip->ac_psy.supplied_to = bq24296_power_supplied_to;
	chip->ac_psy.num_supplicants = ARRAY_SIZE(bq24296_power_supplied_to);
	chip->ac_psy.properties = bq24296_power_props;
	chip->ac_psy.num_properties = ARRAY_SIZE(bq24296_power_props);
	chip->ac_psy.get_property = bq24296_power_get_property;
	chip->ac_psy.set_property = bq24296_power_set_property;
	chip->ac_psy.external_power_changed = bq24296_external_power_changed;

	ret = power_supply_register(&chip->client->dev,
				&chip->ac_psy);
	if (ret) {
		pr_err("failed to register power_supply. ret=%d.\n", ret);
		return ret;
	}

	return 0;
}

static int bq24296_init_bat_psy(struct bq24296_chip *chip)
{
		int ret = 0;

		chip->batt_psy.name		= "battery";
		chip->batt_psy.type		= POWER_SUPPLY_TYPE_BATTERY;
		chip->batt_psy.supplied_to = bq24296_battery_supplied_to;
		chip->batt_psy.num_supplicants = ARRAY_SIZE(bq24296_battery_supplied_to);
		chip->batt_psy.get_property	= bq24296_battery_get_property;
		chip->batt_psy.set_property	= bq24296_battery_set_property;
		chip->batt_psy.properties	= bq24296_battery_properties;
		chip->batt_psy.num_properties	= ARRAY_SIZE(bq24296_battery_properties);
		chip->batt_psy.external_power_changed = bq24296_battery_external_power_changed;
		chip->batt_psy.property_is_writeable = bq24296_battery_is_writeable;
		ret = power_supply_register(&chip->client->dev, &chip->batt_psy);
		if (ret) {
			pr_err("failed to register power_supply. ret=%d.\n", ret);
			return ret;
		}

		return 0;
}
static int bq24296_hw_init(struct bq24296_chip *chip)
{
	int ret = 0;
	//ret = bq24296_write_reg(chip->client, PWR_ON_CONF_REG,
	//		RESET_REGISTER_MASK);
	//if (ret) {
	//	pr_err("failed to reset register\n");
	//	return ret;
	//}
	/*ret = bq24296_set_input_vin_limit(chip, chip->vin_limit_mv);
	if (ret) {
		pr_err("failed to set input voltage limit\n");
		return ret;
	}*/

	ret = bq24296_masked_write(chip->client, INPUT_SRC_CONT_REG,
			INPUT_VOLTAGE_LIMIT_MASK, 0x48);//dpm 4600v
	if (ret) {
		pr_err("failed to set dpm 4600\n");
		return ret;
	}

	ret = bq24296_set_system_vmin(chip, chip->sys_vmin_mv);
	if (ret) {
		pr_err("failed to set system min voltage\n");
		return ret;
	}

	ret = bq24296_set_prechg_i_limit(chip, chip->pre_chg_current_ma);
	if (ret) {
		pr_err("failed to set pre-charge current\n");
		return ret;
	}

	ret = bq24296_set_term_current(chip, chip->term_current_ma);
	if (ret) {
		pr_err("failed to set charge termination current\n");
		return ret;
	}

	ret = bq24296_set_vbat_max(chip, chip->vbat_max_mv);
	if (ret) {
		pr_err("failed to set vbat max\n");
		return ret;
	}

	ret = bq24296_write_reg(chip->client, CHARGE_TERM_TIMER_CONT_REG,
			EN_CHG_TERM_MASK);
	if (ret) {
		pr_err("failed to enable chg termination\n");
		return ret;
	}

	ret = bq24296_set_ir_comp_resister(chip, IRCOMP_R_MAX_MOHM);
	if (ret) {
		pr_err("failed to set ir compensation resister\n");
		return ret;
	}

	ret = bq24296_set_vclamp_mv(chip, IRCOMP_VCLAMP_MAX_MV);
	if (ret) {
		pr_err("failed to set ir vclamp voltage\n");
		return ret;
	}


	ret = bq24296_write_reg(chip->client, IR_COMP_THERM_CONT_REG,  0x73);
	if (ret) {
		pr_err("failed to set 0x06\n");
		return ret;
	}
	ret = bq24296_masked_write(chip->client, PWR_ON_CONF_REG,
			BOOST_LIM_MASK, 0);
	if (ret) {
		pr_err("failed to set boost current limit\n");
		return ret;
	}
	ret = bq24296_enable_hiz(chip, false);
	if (ret) {
		pr_err("failed to set HIZ rc=%d\n", ret);
		return ret;
	}
	ret = bq24296_masked_write(chip->client, PWR_ON_CONF_REG, 0x40, 0x40);
	if (ret) {
		pr_err("failed to kick dog\n");
		return ret;
	}
	return 0;
}

static int bq24296_parse_dt(struct device_node *dev_node,
			   struct bq24296_chip *chip)
{
	int ret = 0;
	int count = 0;

	chip->int_gpio =
		of_get_named_gpio(dev_node, "ti,int-gpio", 0);
	if (chip->int_gpio < 0) {
		pr_err("failed to get int-gpio.\n");
		ret = chip->int_gpio;
		goto out;
	}

	ret = of_property_read_u32(dev_node, "ti,term-current-ma",
				   &(chip->term_current_ma));
	if (ret) {
		pr_err("Unable to read term_current_ma.\n");
		return ret;
	}

	ret = of_property_read_u32(dev_node, "ti,vbat-max-mv",
				   &chip->vbat_max_mv);
	if (ret) {
		pr_err("Unable to read vbat-max-mv.\n");
		return ret;
	}

	ret = of_property_read_u32(dev_node, "ti,pre-chg-current-ma",
				   &chip->pre_chg_current_ma);
	if (ret) {
		pr_err("Unable to read pre-chg-current-ma.\n");
		return ret;
	}

	ret = of_property_read_u32(dev_node, "ti,sys-vimin-mv",
				   &chip->sys_vmin_mv);
	if (ret) {
		pr_err("Unable to read sys-vimin-mv.\n");
		return ret;
	}

	ret = of_property_read_u32(dev_node, "ti,vin-limit-mv",
				   &chip->vin_limit_mv);
	if (ret) {
		pr_err("Unable to read vin-limit-mv.\n");
		return ret;
	}

	ret = of_property_read_u32(dev_node, "ti,wlc-vin-limit-mv",
				   &chip->wlc_vin_limit_mv);
	if (ret) {
		pr_err("Unable to read wlc-vin-limit-mv.\n");
		return ret;
	}

	ret = of_property_read_u32(dev_node, "ti,step-dwn-current-ma",
				   &chip->step_dwn_currnet_ma);
	if (ret) {
		pr_err("Unable to read step-dwn-current-ma.\n");
		return ret;
	}

	ret = of_property_read_u32(dev_node, "ti,step-dwn-thr-mv",
				   &chip->step_dwn_thr_mv);
	if (ret) {
		pr_err("Unable to read step down threshod voltage.\n");
		return ret;
	}

	ret = of_property_read_u32(dev_node, "ti,icl-vbus-mv",
				   &chip->icl_vbus_mv);
	if (ret) {
		pr_err("Unable to read icl threshod voltage.\n");
		return ret;
	}

	ret = of_property_read_u32(dev_node, "ti,wlc-step-dwn-i-ma",
				   &chip->wlc_dwn_i_ma);
	if (ret) {
		pr_err("Unable to read step down current for wlc.\n");
		return ret;
	}

	ret = of_property_read_u32(dev_node, "ti,wlc-dwn-input-i-ma",
				   &chip->wlc_dwn_input_i_ma);
	if (ret) {
		pr_err("Unable to read step down input i limit for wlc.\n");
		return ret;
	}
	chip->powerpath_switch = of_property_read_bool(dev_node,
					"vivo,powerpath-switch");
	chip->wlc_support =
			of_property_read_bool(dev_node, "ti,wlc-support");

	chip->ext_ovp_otg_ctrl =
			of_property_read_bool(dev_node,
					"ti,ext-ovp-otg-ctrl");
	chip->is_support_otg = 	of_property_read_bool(dev_node,
					"vivo,support-otg");
	if (chip->ext_ovp_otg_ctrl) {
		chip->otg_en_gpio =
			of_get_named_gpio(dev_node, "ti,otg-en-gpio", 0);
		if (chip->otg_en_gpio < 0) {
			pr_err("Unable to get named gpio for otg_en_gpio.\n");
			return chip->otg_en_gpio;
		}
	}
	
	chip->charger_enable_gpio = of_property_read_bool(dev_node,
					"vivo,charger-enable-gpio");
	if(chip->charger_enable_gpio){
		chip->chg_en_gpio =
			of_get_named_gpio(dev_node, "ti,chg-en-gpio", 0);
		if (chip->chg_en_gpio < 0) {
			pr_err("Unable to get named gpio for chg_en_gpio.\n");
			return chip->chg_en_gpio;
		}
	}
	ret = of_property_read_u32(dev_node, "vivo,input-max-current-ma", &chip->input_max_current_ma);
	if (ret < 0)
		chip->input_max_current_ma = BQ24296_CHRG_INPUT_DEFAULT_MAX_CURRENT_MA;

	chip->cms_set_current_input_limit = chip->input_max_current_ma;

	ret = of_property_read_u32(dev_node, "vivo,ibat-max-current-ma", &chip->ibat_max_current_ma);
	if (ret < 0)
		chip->ibat_max_current_ma = BQ24296_CHRG_IBAT_DEFAULT_MAX_CURRENT_MA;
	ret = of_property_read_u32(dev_node, "vivo,batt-1C-current-ma", &chip->batt_1C_current_ma);
	if (ret < 0)
		chip->batt_1C_current_ma = BATTERY_DEFAULT_FCC_1C_MA;

	//soft termination
	chip->charger_enable_soft_term = of_property_read_bool(dev_node,
					"vivo,charger-enable-soft-term");
	ret = of_property_read_u32(dev_node, "vivo,soft-term-capacity", &chip->soft_term_capacity);
	if (ret < 0)
		chip->soft_term_capacity = 99;
	ret = of_property_read_u32(dev_node, "vivo,soft-term-timer", &chip->soft_term_timer);
	if (ret < 0)
		chip->soft_term_timer = 120;

	//adjust charge current according to temp
	chip->charger_enable_temp_current = of_property_read_bool(dev_node,
					"vivo,charger-enable-adjust-current-temp");
	if (chip->charger_enable_temp_current){
		of_get_property(dev_node, "vivo,adjust-chg-temp-scale-level", &count);
		count /= sizeof(int);
		chip->adjust_chg_temp= kzalloc(sizeof(int) * count, GFP_KERNEL);
		if (!chip->adjust_chg_temp) {
			pr_err("%s kzalloc failed %d\n", __func__, __LINE__);
			chip->charger_enable_temp_current = false;
			//goto SCALE_ERR;
		}
		ret = of_property_read_u32_array(dev_node, "vivo,adjust-chg-temp-scale-level",
					chip->adjust_chg_temp, count);
		if (ret < 0) {
			pr_err("%s get temp scale levels failed %d\n", __func__, __LINE__);
			kfree(chip->adjust_chg_temp);
			chip->charger_enable_temp_current = false;
			//goto SCALE_ERR;
		}

		of_get_property(dev_node, "vivo,adjust-chg-current-scale-level", &count);
		count /= sizeof(int);
		chip->adjust_chg_current = kzalloc(sizeof(int) * count, GFP_KERNEL);
		if (!chip->adjust_chg_current) {
			pr_err("%s kzalloc failed %d\n", __func__, __LINE__);
			chip->charger_enable_temp_current = false;
			//goto SCALE_ERR;
		}
		ret = of_property_read_u32_array(dev_node, "vivo,adjust-chg-current-scale-level",
					chip->adjust_chg_current, count);
		if (ret < 0) {
			pr_err("%s get current scale levels failed %d\n", __func__, __LINE__);
			kfree(chip->adjust_chg_current);
			chip->charger_enable_temp_current = false;
			//goto SCALE_ERR;
		}
	}

	//recharge fourty min
	chip->charger_enable_recharge_fourty_min = of_property_read_bool(dev_node,
					"vivo,charger-enable-recharge-fourty-min");

	//adjust dpm according to battery voltage
	chip->adjust_dpm_enable = of_property_read_bool(dev_node,
					"vivo,adjust-dpm-enable");

	//use vcdt ,get vbus voltage
	chip->vcdt_present = of_property_read_bool(dev_node,
						"vivo,vcdt-present");

	/* read the bms power supply name */
	ret = of_property_read_string(dev_node, "bbk,bms-psy-name",
						&chip->bms_psy_name);
	if (ret)
		chip->bms_psy_name = NULL;
	ret = of_property_read_u32(dev_node, "ti,chg-tmout-mins",
				   &(chip->chg_tmout_mins));
	if (ret) {
		pr_err("Unable to read chg-tmout-mins.\n");
	}
	pr_info("first check=%d,%d,%d,%d,%d\n",chip->int_gpio, chip->chg_current_ma, chip->term_current_ma,chip->vbat_max_mv,chip->chg_tmout_mins);
	pr_info("second check=%d,%d,%d,%d\n",chip->pre_chg_current_ma, chip->sys_vmin_mv, chip->vin_limit_mv, chip->wlc_vin_limit_mv);
	pr_info("third check=%d,%d,%d,%d\n",chip->step_dwn_currnet_ma, chip->step_dwn_thr_mv, chip->icl_vbus_mv,chip->wlc_dwn_i_ma);
	pr_info("fourth check=%d,%d,%d,%d,%s\n",chip->wlc_dwn_input_i_ma, chip->wlc_support, chip->ext_ovp_otg_ctrl, chip->chg_en_gpio,chip->bms_psy_name);
	pr_info("fifth check=%d,%d,%d\n",chip->input_max_current_ma, chip->ibat_max_current_ma, chip->batt_1C_current_ma);
	pr_info("support otg:%d, charger enable use gpio:%d\n", chip->is_support_otg, chip->charger_enable_gpio);
out:
	return ret;
}
/*
static void bq24296_chgin_check(struct bq24296_chip *chip)
{
	u8 reg,rc;
	bool ext_pwr;
	rc = bq24296_read_reg(chip->client, SYSTEM_STATUS_REG, &reg);
	if (rc) {
		pr_err( "Couldn't read SYSTEM_STATUS_REG rc = %d\n", rc);
	}
	ext_pwr = !!(reg & PG_STAT_MASK);
	pr_debug("ext_pwr=%d\n",ext_pwr);
	if (ext_pwr)
		power_supply_set_present(chip->usb_psy, ext_pwr);
	else
		power_supply_set_present(chip->usb_psy, ext_pwr);

	return;
}
*/
static void hw_init_work(struct work_struct *work)
{
	struct bq24296_chip *chip = container_of(work,
				struct bq24296_chip,
				hw_init_delayed_work.work);
	pr_debug("\n");
	bq24296_hw_init(chip);
	
}
static int bq24296_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct device_node *dev_node = client->dev.of_node;
	struct bq24296_chip *chip;
	int ret = 0;
	unsigned long flags;

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_BYTE_DATA)) {
		pr_err("i2c func fail.\n");
		return -EIO;
	}
	chip = kzalloc(sizeof(struct bq24296_chip), GFP_KERNEL);
	if (!chip) {
		pr_err("failed to alloc memory\n");
		return -ENOMEM;
	}
	chip->client = client;
	chip->dev = &client->dev;
	chip->fake_battery_soc = -EINVAL;
	chip->usb_psy = power_supply_get_by_name("usb");
	if (!chip->usb_psy) {
		pr_err("usb supply not found deferring probe\n");
		ret = -EPROBE_DEFER;
		goto error;
	}

	chip->vadc_dev = qpnp_get_vadc(chip->dev, "chg");
	if (IS_ERR(chip->vadc_dev)) {
		ret = PTR_ERR(chip->vadc_dev);
		if (ret != -EPROBE_DEFER)
			pr_err("vadc property missing\n");
		goto error;
	}
	chip->adc_tm_dev = qpnp_get_adc_tm(chip->dev, "chg");
	if (IS_ERR(chip->adc_tm_dev)) {
		ret = PTR_ERR(chip->adc_tm_dev);
		if (ret != -EPROBE_DEFER)
			pr_err("adc_tm property missing\n");
		goto error;
	}
	if (dev_node) {
		ret = bq24296_parse_dt(dev_node, chip);
		if (ret) {
			pr_err("failed to parse dt\n");
			goto error;
		}
	} else {
	}
	chip->set_chg_current_ma = 0;//chip->ibat_max_current_ma;
	chip->batt_health = POWER_SUPPLY_HEALTH_GOOD;
	chip->max_input_i_ma = INPUT_CURRENT_LIMIT_MAX_MA;
	mutex_init(&chip->set_ibat_mutex);
	mutex_init(&chip->hiz_mutex);
	

	if (chip->wlc_support) {
		chip->wlc_psy = power_supply_get_by_name("wireless");
		if (!chip->wlc_psy) {
			pr_err("wireless supply not found deferring probe\n");
			ret = -EPROBE_DEFER;
			goto error;
		}
	}
	if(chip->charger_enable_gpio){
	ret =  gpio_request_one(chip->chg_en_gpio, GPIOF_DIR_OUT,  "chg_en");
	if (ret) {
		pr_err("failed to request chg_en_gpio\n");
		goto error;
	}
	}
	ret =  gpio_request_one(chip->int_gpio, GPIOF_DIR_IN,
			"bq24296_int");
	if (ret) {
		pr_err("failed to request int_gpio\n");
		goto error;
	}
	chip->irq = gpio_to_irq(chip->int_gpio);

	if (chip->otg_en_gpio) {
		ret = gpio_request_one(chip->otg_en_gpio,
				GPIOF_OUT_INIT_LOW, "otg_en");
		if (ret) {
			pr_err("otg_en_gpio request failed for %d ret=%d\n",
					chip->otg_en_gpio, ret);
			goto err_otg_en_gpio;
		}
	}

	i2c_set_clientdata(client, chip);
	ret = bq24296_hw_init(chip);
	if (ret) {
		pr_err("failed to init hw\n");
		goto err_hw_init;
	}
	the_chip = chip;

	ret = bq24296_init_bat_psy(chip);
	if (ret) {
		pr_err("bq24296_init_battery_psy failed\n");
		goto err_hw_init;
	}
	ret = bq24296_init_ac_psy(chip);
	if (ret) {
		pr_err("bq24296_init_ac_psy failed\n");
		goto err_hw_init;
	}
	if(chip->is_support_otg){
		ret = bq24296_regulator_init(chip);
		if  (ret) {
			pr_err("Couldn't initialize bq24296 ragulator rc=%d\n", ret);
			goto err_regulator_init;
		}
	}
	ret = bq24296_create_debugfs_entries(chip);
	if (ret) {
		pr_err("bq24296_create_debugfs_entries failed\n");
		goto err_debugfs;
	}

	spin_lock_init(&chip->irq_work_lock);
	chip->irq_scheduled_time_status = 0;

	wake_lock_init(&chip->chg_wake_lock,
		       WAKE_LOCK_SUSPEND, BQ24296_NAME);
	wake_lock_init(&chip->icl_wake_lock,
		       WAKE_LOCK_SUSPEND, "icl_wake_lock");
	wake_lock_init(&chip->irq_wake_lock,
			WAKE_LOCK_SUSPEND, BQ24296_NAME "irq");

	INIT_DELAYED_WORK(&chip->recharge_work, bq24296_recharge_work);
	INIT_DELAYED_WORK(&chip->timer_work, bq24296_timer_work);
	INIT_DELAYED_WORK(&chip->vbat_work, bq24296_vbat_work);
	INIT_DELAYED_WORK(&chip->input_limit_work, bq24296_input_limit_worker);
	INIT_DELAYED_WORK(&chip->therm_work, bq24296_therm_mitigation_work);
	INIT_WORK(&chip->irq_work, bq24296_irq_worker);
	INIT_DELAYED_WORK(&chip->hw_init_delayed_work, hw_init_work);
	INIT_DELAYED_WORK(&chip->charger_ovp_monitor_work, charger_ovp_monitor_work);

	if (chip->irq) {
		ret = request_irq(chip->irq, bq24296_irq,
				IRQF_TRIGGER_FALLING,
				"bq24296_irq", chip);
		if (ret) {
			pr_err("request_irq %d failed\n", chip->irq);
			goto err_req_irq;
		}
		enable_irq_wake(chip->irq);
	}
	bq24296_enable_charging(chip, true);
	//bq24296_chgin_check(chip);
	spin_lock_irqsave(&chip->irq_work_lock, flags);
	if (chip->irq_scheduled_time_status == 0) {
		schedule_work(&chip->irq_work);
		chip->irq_scheduled_time_status = 20;
	}
	spin_unlock_irqrestore(&chip->irq_work_lock, flags);

	ret = sysfs_create_group(&chip->dev->kobj,
					&cmcc_attr_group);
	if (ret)
		pr_err("cmcc attribute register fail\n");

	pr_info("probe success\n");

	return 0;

err_req_irq:
	wake_lock_destroy(&chip->chg_wake_lock);
	wake_lock_destroy(&chip->icl_wake_lock);
	wake_lock_destroy(&chip->irq_wake_lock);
	if (chip->dent)
		debugfs_remove_recursive(chip->dent);
err_debugfs:
	power_supply_unregister(&chip->ac_psy);
err_regulator_init:
	if(chip->is_support_otg)
		regulator_unregister(chip->otg_vreg.rdev);
err_hw_init:
	if (chip->otg_en_gpio)
		   gpio_free(chip->otg_en_gpio);
err_otg_en_gpio:
	if (chip->int_gpio)
		gpio_free(chip->int_gpio);
error:
	kfree(chip);
	pr_info("fail to probe\n");
	return ret;

}

static int bq24296_remove(struct i2c_client *client)
{
	struct bq24296_chip *chip = i2c_get_clientdata(client);

	if (chip->irq)
		free_irq(chip->irq, chip);
	wake_lock_destroy(&chip->chg_wake_lock);
	wake_lock_destroy(&chip->icl_wake_lock);
	wake_lock_destroy(&chip->irq_wake_lock);

	if (chip->dent)
		debugfs_remove_recursive(chip->dent);
	power_supply_unregister(&chip->ac_psy);
	if (chip->otg_en_gpio)
		   gpio_free(chip->otg_en_gpio);
	if (chip->int_gpio)
		gpio_free(chip->int_gpio);
	kfree(chip);

	return 0;
}

static const struct i2c_device_id bq24296_id[] = {
	{BQ24296_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, bq24296_id);

static const struct of_device_id bq24296_match[] = {
	{ .compatible = "ti,bq24296-charger", },
	{ },
};

static struct i2c_driver bq24296_driver = {
	.driver	= {
			.name	= BQ24296_NAME,
			.owner	= THIS_MODULE,
			.of_match_table = of_match_ptr(bq24296_match),
	},
	.probe		= bq24296_probe,
	.remove		= bq24296_remove,
	.id_table	= bq24296_id,
};

static int __init bq24296_init(void)
{
	return i2c_add_driver(&bq24296_driver);
}
module_init(bq24296_init);

static void __exit bq24296_exit(void)
{
	return i2c_del_driver(&bq24296_driver);
}
module_exit(bq24296_exit);
MODULE_AUTHOR("ChoongRyeol Lee <choongryeol.lee@lge.com>");
MODULE_DESCRIPTION("BQ24296 Charger Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:" BQ24296_NAME);
